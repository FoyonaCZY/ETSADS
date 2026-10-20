//! GPS-constrained, successor-only route. Unsupported manoeuvres terminate the
//! usable prefix; a nearest-road guess never replaces a missing GPS route.
use super::core::{LeadIn, Road, RoadState, Roads};
use anyhow::{Context, ensure};
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    corridor::{
        ChannelKind, CorridorError, CorridorPolicy, GlobalNativeRouteIndex, NativeRouteIndex,
    },
    project_polyline,
    sector::{CacheLimits, SectorStore},
    traffic::{
        ControlKind, RightOfWay, SignalDecision, SignalHoldReason, SignalObservation,
        TrafficContext,
    },
};
use etsads_msgs::{Agent, OrientedBox, Point3, Pose2, frames::wrap_angle};
use serde::Serialize;
use std::{
    collections::{HashMap, HashSet},
    path::{Path, PathBuf},
    sync::mpsc::{Receiver, SyncSender, sync_channel},
    time::{Duration, Instant},
};

pub struct Navigator {
    store: SectorStore,
    annotations: AnnotationStore,
    global: GlobalNativeRouteIndex,
}
impl Navigator {
    pub fn inspect(&self, pose: Pose2, z: f64) -> anyhow::Result<serde_json::Value> {
        let center = Point3 {
            x: pose.x,
            y: pose.y,
            z,
        };
        let region = self.store.load_2km(center)?;
        let traffic = self
            .annotations
            .load_for_region(center, 2000.0, &region)?
            .traffic;
        let mut nearby = Vec::new();
        for lane in &region.map.lanes {
            if let Ok((p, point)) = project_polyline(0, &lane.center, center)
                && p.distance < 35.0
                && (p.z - z).abs() < 3.0
            {
                nearby.push((p.distance,serde_json::json!({"id":lane.id,"projection":p,"point":point,"node_start":lane.node_start,"node_end":lane.node_end,"successors":lane.successors,"simple":simple_lane(&traffic,&lane.id),"validated":lane.centerline_validated,"truck":lane.truck_access})));
            }
        }
        nearby.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(
            serde_json::json!({"source":region.source_id,"nearby":nearby.into_iter().take(12).map(|p|p.1).collect::<Vec<_>>()}),
        )
    }
    /// Read-only: resolve growing prefixes of the GPS node list to find the
    /// first checkpoint that cannot be joined, with the lanes touching it.
    pub fn route_probe(
        &self,
        pose: Pose2,
        z: f64,
        speed: f64,
        gps: &[u64],
        position: usize,
    ) -> anyhow::Result<serde_json::Value> {
        let center = Point3 {
            x: pose.x,
            y: pose.y,
            z,
        };
        let region = self.store.load_2km(center)?;
        let native = NativeRouteIndex::from_region(&region)?;
        let lanes_at = |node: u64| -> Vec<serde_json::Value> {
            region
                .map
                .lanes
                .iter()
                .filter(|l| l.node_start == node || l.node_end == node)
                .map(|l| serde_json::json!({"id":l.id,"start":l.node_start,"end":l.node_end,"truck":l.truck_access,"validated":l.centerline_validated,"successors":l.successors,"first":l.center.first().map(|p| [p.x, p.y, p.z]),"last":l.center.last().map(|p| [p.x, p.y, p.z])}))
                .collect()
        };
        let roads = Roads::from_export(region.map.clone())?;
        let current = match roads.observe(pose, z, speed, None) {
            Ok(current) => current,
            Err(error) => {
                let leads: Vec<_> = roads
                    .lead_ins(pose, z)
                    .iter()
                    .take(12)
                    .map(|l| {
                        let nodes = region
                            .map
                            .lanes
                            .iter()
                            .find(|x| x.id == l.state.lane)
                            .map(|x| [x.node_start, x.node_end]);
                        let tries: Vec<String> = (3..gps.len())
                            .find_map(|k| {
                                let from = 2;
                                native
                                    .resolve_local_prefix(
                                        &self.global,
                                        &gps[from..k],
                                        &l.state.lane,
                                        l.state.station_m,
                                        &route_policy(),
                                    )
                                    .err()
                                    .map(|e| format!("2..{k} fails at node {}: {e}", gps[k - 1]))
                            })
                            .into_iter()
                            .collect();
                        serde_json::json!({"lane":l.state.lane,"station":l.state.station_m,"curve_m":l.length_m,"nodes":nodes,"resolve":tries})
                    })
                    .collect();
                return Ok(
                    serde_json::json!({"off_lane":format!("{error:#}"),"lead_ins":leads,"gps":gps.iter().take(8).collect::<Vec<_>>()}),
                );
            }
        };
        let mut gps = gps.to_vec();
        gps.dedup();
        // Start where `prepare` would: our lane's own checkpoint if listed.
        let lane_nodes = region
            .map
            .lanes
            .iter()
            .find(|l| l.id == current.lane)
            .map(|l| [l.node_start, l.node_end]);
        let from = lane_nodes
            .and_then(|nodes| {
                nodes
                    .iter()
                    .filter(|n| **n != 0)
                    .find_map(|n| gps.iter().position(|g| g == n))
            })
            .unwrap_or(position.min(gps.len().saturating_sub(1)));
        let mut steps = Vec::new();
        for k in from + 1..=gps.len().min(from + 40) {
            match native.resolve_local_prefix(
                &self.global,
                &gps[from..k],
                &current.lane,
                current.station_m,
                &route_policy(),
            ) {
                Ok(c) => steps.push(serde_json::json!({"k":k,"node":gps[k-1],"ok":true,"lanes":c.corridor.steps.len(),"local_terminal":c.must_stop_at_local_terminal})),
                Err(e) => {
                    steps.push(serde_json::json!({"k":k,"node":gps[k-1],"ok":false,"error":e.to_string(),"lanes_at_node":lanes_at(gps[k-1]),"lanes_at_previous":if k > 1 { lanes_at(gps[k-2]) } else { vec![] }}));
                    break;
                }
            }
        }
        // From every lane leaving the checkpoint node: how far the route
        // resolves, and the first failing checkpoint.
        let node = gps.get(from).copied().unwrap_or(0);
        let alternatives: Vec<_> = region
            .map
            .lanes
            .iter()
            .filter(|l| l.node_start == node || l.node_end == node)
            .flat_map(|l| {
                let mut ids = vec![l.id.clone()];
                ids.extend(l.successors.iter().cloned());
                ids
            })
            .map(|lane| {
                let mut reached = None;
                let mut error = None;
                for k in from + 1..=gps.len().min(from + 60) {
                    match native.resolve_local_prefix(
                        &self.global,
                        &gps[from..k],
                        &lane,
                        0.0,
                        &route_policy(),
                    ) {
                        Ok(_) => reached = Some(k),
                        Err(e) => {
                            error = Some(serde_json::json!({"k":k,"node":gps[k - 1],"previous":gps.get(k.wrapping_sub(2)),"error":e.to_string(),"lanes_at_node":lanes_at(gps[k - 1]),"lanes_at_previous":k.checked_sub(2).map(|i| lanes_at(gps[i]))}));
                            break;
                        }
                    }
                }
                serde_json::json!({"lane":lane,"reached_k":reached,"error":error})
            })
            .collect();
        Ok(
            serde_json::json!({"current_lane":current.lane,"lane_nodes":lane_nodes,"from":from,"station":current.station_m,"gps":gps,"steps":steps,"alternatives":alternatives}),
        )
    }
    pub fn open(path: &Path) -> anyhow::Result<Self> {
        Ok(Self {
            store: SectorStore::open(path, CacheLimits::default())?,
            annotations: AnnotationStore::open(path, AnnotationLimits::default())?,
            global: GlobalNativeRouteIndex::load_source(path)?,
        })
    }
    /// Prepare from GPS checkpoint `from` (of the deduplicated list): the
    /// game keeps passed checkpoints in its list, and the corridor resolver
    /// needs the route to begin at the current one (r10: every rolling
    /// extension failed mid-route). `lane` is the lane the truck is locked
    /// to, which free matching cannot always tell apart inside a junction.
    pub fn prepare_from(
        &self,
        pose: Pose2,
        z: f64,
        speed: f64,
        gps: &[u64],
        from: usize,
        lane: Option<&str>,
    ) -> anyhow::Result<Guidance> {
        let center = Point3 {
            x: pose.x,
            y: pose.y,
            z,
        };
        let region = self.store.load_2km(center)?;
        let native = NativeRouteIndex::from_region(&region)?;
        let traffic = self
            .annotations
            .load_for_region(center, 2000.0, &region)?
            .traffic;
        // Every lane centre, including ones a truck may not use: conflicting
        // traffic is projected onto them at junctions.
        let centers: HashMap<String, LaneCenter> = region
            .map
            .lanes
            .iter()
            .map(|lane| {
                (
                    lane.id.clone(),
                    LaneCenter {
                        points: lane.center.clone(),
                        predecessors: lane.predecessors.clone(),
                    },
                )
            })
            .collect();
        let traffic_lanes = TrafficLanes::new(&region.map.lanes);
        let region_nodes: HashMap<String, (u64, u64)> = region
            .map
            .lanes
            .iter()
            .map(|lane| (lane.id.clone(), (lane.node_start, lane.node_end)))
            .collect();
        let roads = Roads::from_export(region.map)?;
        let matched = match lane {
            Some(lane) => roads
                .observe(pose, z, speed, Some(lane))
                .or_else(|_| roads.observe(pose, z, speed, None)),
            None => roads.observe(pose, z, speed, None),
        };
        // Off the lane network (a yard or a fuel station): join a lane ahead
        // by a gentle curve, driven slowly. Only from a standstill or crawl.
        // Every reachable lane is a candidate; the route decides which one.
        let starts: Vec<(RoadState, Option<LeadIn>)> = match matched {
            Ok(current) => vec![(current, None)],
            Err(error) => {
                let leads = if speed.abs() < 3.0 {
                    roads.lead_ins(pose, z)
                } else {
                    Vec::new()
                };
                if leads.is_empty() {
                    return Err(error);
                }
                leads
                    .into_iter()
                    .take(12)
                    .map(|lead| (lead.state.clone(), Some(lead)))
                    .collect()
            }
        };
        let mut gps = gps.to_vec();
        gps.dedup();
        // Lane changes are allowed when resolving the corridor so that a route
        // needing one still yields the drivable prefix up to that point; the
        // step loop below ends the prefix at the first non-successor channel.
        let policy = route_policy();
        let first_from = from.min(gps.len().saturating_sub(1));
        let try_start = |current: &RoadState, lead: bool| {
            let resolve = |from: usize| {
                native.resolve_local_prefix(
                    &self.global,
                    &gps[from..],
                    &current.lane,
                    current.station_m,
                    &policy,
                )
            };
            let mut from = first_from;
            let mut attempt = resolve(from);
            // ETS2LA's list still starts where the route was planned, even
            // after a reload: start from our lane's own checkpoint when that
            // resolves. Inside a junction our lane has no checkpoint of its
            // own; the route then has to begin at the next one.
            let mut candidates = vec![from];
            if matches!(attempt, Err(CorridorError::NoCorridor)) {
                let (start, end) = region_nodes.get(&current.lane).copied().unwrap_or_default();
                candidates.extend(
                    [start, end]
                        .iter()
                        .filter(|n| **n != 0)
                        .filter_map(|n| gps[from..].iter().position(|g| g == n))
                        .filter(|i| *i > 0)
                        .map(|i| from + i),
                );
                candidates.push(from + 1);
                // Leaving a yard, the route still lists the yard's own nodes
                // before the road the lead-in joins (L12: two checkpoints).
                if lead {
                    candidates.extend([from + 2, from + 3]);
                }
                candidates.retain(|i| *i < gps.len());
                for &index in &candidates[1..] {
                    if let Ok(found) = resolve(index) {
                        from = index;
                        attempt = Ok(found);
                        break;
                    }
                }
            }
            // A break further along the 2 km window (missing map link) must
            // not discard the part that does connect: drive the longest
            // prefix and stop before the break, from whichever start has one.
            let mut truncated = false;
            if matches!(attempt, Err(CorridorError::NoCorridor)) {
                for &start in &candidates {
                    let prefix = |k: usize| {
                        native.resolve_local_prefix(
                            &self.global,
                            &gps[start..k],
                            &current.lane,
                            current.station_m,
                            &policy,
                        )
                    };
                    let mut best = None;
                    for k in start + 2..gps.len() {
                        match prefix(k) {
                            Ok(found) => best = Some(found),
                            Err(_) => break,
                        }
                    }
                    if let Some(found) = best {
                        from = start;
                        attempt = Ok(found);
                        truncated = true;
                        break;
                    }
                }
            }
            (from, attempt, truncated)
        };
        let mut chosen = None;
        for (index, (state, lead)) in starts.iter().enumerate() {
            let (from, attempt, truncated) = try_start(state, lead.is_some());
            let ok = attempt.is_ok();
            if ok || chosen.is_none() {
                chosen = Some((index, from, attempt, truncated));
            }
            if ok {
                break;
            }
        }
        let (index, from, attempt, truncated) = chosen.expect("at least one start");
        let (current, lead_in) = starts.into_iter().nth(index).expect("chosen start");
        let pending = &gps[from..];
        let resolved = attempt.map_err(|error| {
            // The game routes through a U-turn when it heads for our lane's
            // end node and then straight back to its start node.
            let lane = region_nodes.get(&current.lane).copied();
            let u_turn = lane.is_some_and(|(start, end)| {
                pending
                    .windows(2)
                    .take(3)
                    .any(|w| w[0] == end && w[1] == start)
            });
            match error {
                CorridorError::NoCorridor if u_turn => {
                    anyhow::anyhow!("导航路线需要掉头，请手动掉头并驶入路线后再启用")
                }
                CorridorError::NoCorridor => anyhow::anyhow!(
                    "当前车道接不上导航路线（可能需要换道或掉头），请手动驶入路线后再启用"
                ),
                other => anyhow::Error::new(other),
            }
        })?;
        let reached_prefix = from
            + resolved
                .corridor
                .steps
                .first()
                .map_or(0, |s| s.checkpoint_index);
        let unique: std::collections::HashSet<_> = resolved
            .corridor
            .steps
            .iter()
            .map(|s| s.source_lane_id.as_str())
            .collect();
        ensure!(
            unique.len() == resolved.corridor.steps.len(),
            "导航重复经过同一车道，首版需要人工处理该回环"
        );
        // The route's lanes and their approaches: traffic there is ahead of
        // or behind the truck, never crossing it at a junction.
        let own: HashSet<&str> = unique
            .iter()
            .flat_map(|id| {
                std::iter::once(*id).chain(
                    centers
                        .get(*id)
                        .into_iter()
                        .flat_map(|c| c.predecessors.iter().map(String::as_str)),
                )
            })
            .collect();
        let mut points = Vec::<Point3>::new();
        let mut lane_ends: Vec<(String, f64, u64)> = Vec::new();
        let lead_in_m = lead_in.as_ref().map_or(0.0, |l| l.length_m);
        if let Some(lead) = &lead_in {
            // The lane's own point at the joining station follows below.
            points.extend(&lead.points[..lead.points.len() - 1]);
        }
        let mut terminal = if truncated {
            "前方地图路段无法接续，请准备接管"
        } else if resolved.must_stop_at_local_terminal {
            "本地导航范围结束，请手动接管"
        } else {
            "即将到达目的地，请准备接管"
        }
        .to_owned();
        let mut controls = Vec::<RouteControl>::new();
        let mut lane_changes = Vec::<LaneChange>::new();
        // Arc length on the previous lane where a lane change starts, set
        // when the next step is reached through a lateral channel.
        let mut change_start = None::<(f64, f64)>;
        for (i, step) in resolved.corridor.steps.iter().enumerate() {
            let channel = i
                .checked_sub(1)
                .and_then(|p| resolved.corridor.allowed_channels.get(p));
            let changing = channel.is_some_and(|c| c.kind != ChannelKind::Successor);
            if i > 0 && channel.is_none() {
                terminal = "导航数据不完整，请手动接管".into();
                break;
            }
            let junction = match lane_support(&traffic, &step.source_lane_id) {
                Ok(junction) => junction,
                Err(reason) => {
                    terminal = reason.into();
                    break;
                }
            };
            let Some(road) = roads.roads.get(&step.source_lane_id) else {
                terminal = "导航几何不完整".into();
                break;
            };
            // Where this lane's own geometry starts: after a lead-in curve, or
            // after the S-blend of a lane change into it.
            let join = if changing {
                let (start, end) = change_start.take().expect("set by the previous step");
                let from = &roads.roads[&resolved.corridor.steps[i - 1].source_lane_id];
                let ratio = road.length() / from.length();
                let begin_m = points
                    .windows(2)
                    .map(|p| p[0].distance_xy(p[1]))
                    .sum::<f64>();
                let samples = ((end - start) / 2.0).ceil().max(8.0) as usize;
                for k in 0..=samples {
                    let t = k as f64 / samples as f64;
                    let w = t * t * (3.0 - 2.0 * t);
                    let a = start + t * (end - start);
                    let (Some((pa, _)), Some((pb, _))) =
                        (from.sample(a), road.sample((a * ratio).min(road.length())))
                    else {
                        continue;
                    };
                    let p = Point3 {
                        x: pa.x + w * (pb.x - pa.x),
                        y: pa.y + w * (pb.y - pa.y),
                        z: pa.z + w * (pb.z - pa.z),
                    };
                    if points
                        .last()
                        .is_none_or(|last| last.distance_xy(p) > MIN_PATH_STEP_M)
                    {
                        points.push(p);
                    }
                }
                let end_m = points
                    .windows(2)
                    .map(|p| p[0].distance_xy(p[1]))
                    .sum::<f64>();
                lane_changes.push(LaneChange {
                    start_m: begin_m,
                    end_m,
                    left: channel.is_some_and(|c| c.kind == ChannelKind::ChangeLeft),
                    target: road.points.clone(),
                });
                (end * ratio).min(road.length())
            } else if i == 0 {
                lead_in.as_ref().map_or(0.0, |l| l.state.station_m)
            } else {
                0.0
            };
            // A lane change out of this lane: where it starts and how long it
            // takes. About 4.5 s at 80 km/h when there is room, never under
            // 30 m; the speed planner slows for the resulting curvature.
            let outgoing = resolved
                .corridor
                .allowed_channels
                .get(i)
                .filter(|c| c.kind != ChannelKind::Successor);
            if let Some(constraint) = outgoing.and_then(|c| c.lane_change.as_ref()) {
                let start = (constraint.earliest_start_s_m.max(join)
                    + if i == 0 { 15.0 } else { 0.0 })
                .min(constraint.latest_start_s_m);
                let length = (constraint.latest_start_s_m + constraint.minimum_distance_m - start)
                    .min(LANE_CHANGE_LENGTH_M)
                    .min(road.length() - 1.0 - start);
                if length < 30.0 || start < join {
                    terminal = "导航需要换道但距离不足，请手动接管".into();
                    // Keep this lane up to its end; the route stops there.
                } else {
                    change_start = Some((start, start + length));
                }
            } else if outgoing.is_some() {
                terminal = "导航需要换道，请手动接管".into();
            }
            if let Some(last) = points.last().copied().filter(|_| join == 0.0) {
                let first = road.points[0];
                if last.distance_xy(first) > MAX_JOINT_GAP_M || (last.z - first.z).abs() > 0.5 {
                    terminal = format!(
                        "导航连接几何不连续（{} 间距 {:.2} m，高差 {:.2} m）",
                        step.source_lane_id,
                        last.distance_xy(first),
                        last.z - first.z
                    );
                    break;
                }
                // The exporter links lanes up to 1 m apart (L5 stopped at a
                // 1.00 m joint). Blend a visible offset instead of stepping
                // sideways at the joint, long enough that the S-curve stays
                // gentler than JOINT_MAX_CURVATURE: r13 blended a 1.5 m joint
                // where a road widens over 10 m (0.09 /m) and braked from
                // 80 to 25 km/h for it.
                if last.distance_xy(first) > 0.2 {
                    blend_joint(&mut points, first);
                }
                let last = *points.last().expect("path points");
                // Headings over about 2 m on each side of the joint: native
                // lanes can end in a sub-millimetre segment whose direction
                // is meaningless (r11: a straight junction read as 40°).
                let back = points
                    .iter()
                    .rev()
                    .find(|p| p.distance_xy(last) >= 2.0)
                    .unwrap_or(&points[points.len() - 2]);
                let previous_heading = (last.y - back.y).atan2(last.x - back.x);
                let ahead = road
                    .sample(road.length().min(2.0))
                    .map_or(first, |(p, _)| p);
                let next_heading = (ahead.y - first.y).atan2(ahead.x - first.x);
                if wrap_angle(next_heading - previous_heading).abs() > 0.35 {
                    terminal = "前方转弯需要手动接管".into();
                    break;
                }
            }
            let base = points
                .windows(2)
                .map(|p| p[0].distance_xy(p[1]))
                .sum::<f64>()
                - join;
            if junction {
                controls.extend(route_controls(
                    &traffic,
                    &step.source_lane_id,
                    road,
                    base,
                    &centers,
                    &own,
                )?);
            }
            let end = match change_start {
                Some((start, _)) => start,
                None => step.exit_s_m.min(road.length()),
            };
            let mut station = 0.0;
            let mut clipped = Vec::new();
            if join > 0.0 {
                clipped.push(road.sample(join).context("导航起点无法采样")?.0);
            }
            for (n, p) in road.points.iter().enumerate() {
                if n > 0 {
                    station += p.distance_xy(road.points[n - 1]);
                }
                if station >= end {
                    break;
                }
                if join == 0.0 || station > join {
                    clipped.push(*p);
                }
            }
            clipped.push(road.sample(end).context("导航终点无法采样")?.0);
            for p in &clipped {
                if points
                    .last()
                    .is_none_or(|last| last.distance_xy(*p) > MIN_PATH_STEP_M)
                {
                    points.push(*p);
                }
            }
            let length = points
                .windows(2)
                .map(|p| p[0].distance_xy(p[1]))
                .sum::<f64>();
            lane_ends.push((step.source_lane_id.clone(), length, step.node_end));
            if outgoing.is_some() && change_start.is_none() {
                break;
            }
        }
        ensure!(points.len() >= 2, "当前导航路段需要手动驾驶");
        let path = Road::path("gps".into(), points)?;
        let roads = Roads {
            roads: HashMap::from([("gps".into(), path)]),
        };
        let observed = roads.observe(pose, z, speed, Some("gps"))?;
        ensure!(
            observed.remaining_m > 35.0 + speed * 3.0 + speed * speed / 3.0,
            "可用导航路段太短（{terminal}），请先手动驶过"
        );
        let groups: std::collections::HashSet<_> =
            controls.iter().filter_map(|c| c.group.clone()).collect();
        let signal_heads = TrafficContext {
            signals: traffic
                .signals
                .iter()
                .filter(|s| groups.contains(&s.id))
                .cloned()
                .collect(),
            ..Default::default()
        };
        Ok(Guidance {
            roads,
            lane_ends,
            reached_prefix,
            gps,
            terminal,
            progress_s: observed.station_m,
            tracks: HashMap::new(),
            approach_cap: None,
            controls,
            signal_heads,
            signals_seen: Vec::new(),
            stop_dwell: HashMap::new(),
            traffic_lanes,
            lead_in_m,
            lane_changes,
        })
    }
}
/// Prepares the next route on its own thread while driving, so map I/O never
/// stalls the control loop. At most one request is in flight.
pub struct PrepareRequest {
    pub pose: Pose2,
    pub z: f64,
    pub speed: f64,
    pub gps: Vec<u64>,
    /// Checkpoint index (deduplicated list) the truck has most recently passed.
    pub from: usize,
    pub lane: Option<String>,
}
pub struct Preparer {
    requests: SyncSender<PrepareRequest>,
    results: Receiver<anyhow::Result<Guidance>>,
    pending: bool,
    last: Option<Instant>,
}
impl Preparer {
    pub fn spawn(map: PathBuf) -> Self {
        let (requests, request_rx) = sync_channel::<PrepareRequest>(1);
        let (result_tx, results) = sync_channel(1);
        let _ = std::thread::Builder::new()
            .name("etsads-route".into())
            .spawn(move || {
                let navigator = Navigator::open(&map);
                for r in request_rx {
                    let result = match &navigator {
                        Ok(n) => {
                            n.prepare_from(r.pose, r.z, r.speed, &r.gps, r.from, r.lane.as_deref())
                        }
                        Err(e) => Err(anyhow::anyhow!("{e:#}")),
                    };
                    if result_tx.send(result).is_err() {
                        break;
                    }
                }
            });
        Self {
            requests,
            results,
            pending: false,
            last: None,
        }
    }
    /// Ask for a route from this pose unless one is pending or recent.
    pub fn request(&mut self, request: PrepareRequest) {
        if self.pending
            || self
                .last
                .is_some_and(|t| t.elapsed() < Duration::from_secs(3))
        {
            return;
        }
        if self.requests.try_send(request).is_ok() {
            self.pending = true;
            self.last = Some(Instant::now());
        }
    }
    pub fn poll(&mut self) -> Option<anyhow::Result<Guidance>> {
        let result = self.results.try_recv().ok()?;
        self.pending = false;
        Some(result)
    }
}

/// A lane change on the route, as path stations of the rear axle.
#[derive(Clone, Debug)]
struct LaneChange {
    start_m: f64,
    end_m: f64,
    left: bool,
    /// Target lane centreline, to find traffic in it.
    target: Vec<Point3>,
}
/// Path points closer than this are merged: a native lane may end in a
/// sub-millimetre segment whose direction is noise (r11).
const MIN_PATH_STEP_M: f64 = 0.05;
/// Largest lane joint gap the path bridges (m, the exporter links up to 2 m),
/// the longest blend of a visible offset, and the peak curvature (1/m) a
/// smoothstep blend may add: 6 · gap / length².
const MAX_JOINT_GAP_M: f64 = 2.0;
const JOINT_BLEND_M: f64 = 60.0;
const JOINT_MAX_CURVATURE: f64 = 0.002;
/// Shift the end of the path toward the next lane's start `first` with a
/// smooth weight, so the curve keeps its shape. Cutting the end off and
/// joining with a chord turned a tight junction curve into a corner the
/// locked matcher rejected (L18).
fn blend_joint(points: &mut [Point3], first: Point3) {
    let Some(&last) = points.last() else {
        return;
    };
    let gap = (first.x - last.x, first.y - last.y, first.z - last.z);
    let blend = (6.0 * last.distance_xy(first) / JOINT_MAX_CURVATURE)
        .sqrt()
        .clamp(4.0, JOINT_BLEND_M);
    let mut back = 0.0;
    let mut index = points.len() - 1;
    while index > 0 && back < blend {
        let along = points[index].distance_xy(points[index - 1]);
        let t = 1.0 - back / blend;
        let w = t * t * (3.0 - 2.0 * t);
        points[index].x += w * gap.0;
        points[index].y += w * gap.1;
        points[index].z += w * gap.2;
        back += along;
        index -= 1;
    }
}
/// Lane-change blend length when the road allows it (about 4.5 s at 80 km/h).
const LANE_CHANGE_LENGTH_M: f64 = 100.0;
/// A junction rule on the route, projected onto the GPS path.
#[derive(Clone, Debug)]
struct RouteControl {
    /// Path station where the truck's front must stop.
    station_m: f64,
    kind: ControlKind,
    priority: RightOfWay,
    group: Option<String>,
    conservative: bool,
    /// Every conflicting lane was found; its traffic is fully visible.
    complete: bool,
    conflicts: Vec<ConflictLane>,
    /// The route lane turns left by more than 0.35 rad (longer gap needed).
    left_turn: bool,
    key: String,
}
#[derive(Clone, Debug)]
struct ConflictLane {
    points: Vec<Point3>,
    /// Station of the conflict point on that lane, beyond its end for an
    /// approach lane.
    conflict_s: f64,
}
/// A lane centre and the lanes leading into it.
struct LaneCenter {
    points: Vec<Point3>,
    predecessors: Vec<String>,
}
/// Cross traffic is watched this far up its approach. r12 saw it only on
/// the junction's own lane, 20–40 m long, so every junction was entered at
/// 15 km/h.
const CONFLICT_APPROACH_M: f64 = 200.0;
/// ETS2LA reports at most this many vehicles; a full list may omit some.
const TRAFFIC_SLOTS: usize = 40;
/// Stop-line speed of a give-way junction whose cross traffic is all seen.
const CLEAR_LINE_SPEED: f64 = 30.0 / 3.6;
/// Stop-line speed where traffic or the rule cannot be fully known.
const CAUTION_LINE_SPEED: f64 = 4.2;
/// Time gap behind a vehicle that reaches a zip merge first.
const ZIP_FOLLOW_S: f64 = 1.5;
/// A conflicting lane and its approaches up to `CONFLICT_APPROACH_M`; lanes
/// in `own` (the route and its approaches) are left out. None when the lane
/// is missing or too short to project the conflict point onto.
fn conflict_lanes(
    id: &str,
    position: Point3,
    centers: &HashMap<String, LaneCenter>,
    own: &HashSet<&str>,
) -> Option<Vec<ConflictLane>> {
    let lane = centers.get(id).filter(|c| c.points.len() >= 2)?;
    let (q, _) = project_polyline(0, &lane.points, position).ok()?;
    let mut out = vec![ConflictLane {
        points: lane.points.clone(),
        conflict_s: q.s,
    }];
    let mut seen = HashSet::from([id]);
    let mut open = vec![(id, q.s)];
    while let Some((id, ahead)) = open.pop() {
        for before in &centers[id].predecessors {
            if own.contains(before.as_str()) || !seen.insert(before.as_str()) {
                continue;
            }
            let Some(lane) = centers.get(before).filter(|c| c.points.len() >= 2) else {
                continue;
            };
            let length = lane
                .points
                .windows(2)
                .map(|p| p[0].distance_xy(p[1]))
                .sum::<f64>();
            out.push(ConflictLane {
                points: lane.points.clone(),
                conflict_s: length + ahead,
            });
            if length + ahead < CONFLICT_APPROACH_M {
                open.push((before, length + ahead));
            }
        }
    }
    Some(out)
}
/// Ok(false): plain road. Ok(true): junction lane whose rules are known and
/// handled by `RouteControl`. Err: the route must end before this lane.
/// Corridor policy for following the game's GPS: lanes closed only to AI
/// trucks (service-area roads, L14 stopped 1 km short of a fuel station)
/// are used where the route needs them, at a cost that keeps truck lanes
/// preferred.
fn route_policy() -> CorridorPolicy {
    CorridorPolicy {
        ai_restricted_penalty_m: Some(200.0),
        unmarked_change_penalty_m: Some(100.0),
        ..CorridorPolicy::default()
    }
}
fn lane_support(traffic: &TrafficContext, id: &str) -> Result<bool, &'static str> {
    if !traffic.has_annotations_for_route(&[id.to_owned()]) {
        return Err("前方路段缺少路口标注，请手动处理");
    }
    if simple_lane(traffic, id) {
        return Ok(false);
    }
    let controls = traffic.upcoming_stop_controls(&[id.to_owned()]);
    if controls
        .iter()
        .any(|c| c.control_kind == ControlKind::Signal && c.priority == RightOfWay::Unknown)
    {
        return Err("前方信号灯规则未知，请手动处理");
    }
    // A signal without group data is read from the observed lamp nearest the
    // stop line (`Guidance::nearest_signal`), or given way to when dark; an
    // unrecorded rule is crossed as give-way (`unknown_rule`).
    Ok(true)
}
fn unknown_rule(kind: ControlKind, priority: RightOfWay) -> bool {
    kind == ControlKind::Unknown && priority == RightOfWay::Unknown
}
fn route_controls(
    traffic: &TrafficContext,
    id: &str,
    road: &Road,
    base: f64,
    centers: &HashMap<String, LaneCenter>,
    own: &HashSet<&str>,
) -> anyhow::Result<Vec<RouteControl>> {
    let turn = wrap_angle(
        road.sample(road.length()).context("路口车道无法采样")?.1
            - road.sample(0.0).context("路口车道无法采样")?.1,
    );
    let mut out = Vec::new();
    for control in traffic.upcoming_stop_controls(&[id.to_owned()]) {
        let (p, _) = project_polyline(0, &road.points, control.position)?;
        let mut conflicts = Vec::new();
        let mut complete = true;
        for other in &control.conflict_lane_ids {
            // A conflict lane missing or too short to project is left
            // unchecked; the control then counts as conservative.
            match conflict_lanes(other, control.position, centers, own) {
                Some(lanes) => conflicts.extend(lanes),
                None => complete = false,
            }
        }
        out.push(RouteControl {
            station_m: base + p.s - control.stop_before_m,
            kind: control.control_kind,
            priority: control.priority,
            group: control.signal_group_id.clone(),
            // A conflict whose other lane is missing cannot be checked, and a
            // junction whose rule the map does not record is crossed the way
            // a driver would: as give-way, slowly, yielding to cross traffic.
            conservative: control.requires_conservative_wait
                || !complete
                || unknown_rule(control.control_kind, control.priority),
            complete,
            conflicts,
            left_turn: turn > 0.35,
            key: format!("{id}:{:.2}:{:.2}", control.position.x, control.position.y),
        });
    }
    Ok(out)
}

/// Front-bumper gap to the first path slice whose ego-width rectangle the
/// body overlaps, from just behind the rear axle up to `SWEEP_RANGE_M`. Near
/// the truck the corridor also covers its current lateral offset.
fn sweep_gap(
    path: &Road,
    road: &RoadState,
    body: OrientedBox,
    reach: f64,
    range: f64,
) -> Option<f64> {
    let rear = road.station_m;
    let mut s = (rear - 1.0).max(0.0);
    let end = (rear + range).min(path.length());
    let target = [
        body.pose.x,
        body.pose.y,
        body.pose.yaw,
        body.length / 2.0,
        body.width / 2.0,
    ];
    while s <= end {
        let (p, heading) = path.sample(s)?;
        let half_width = corridor_half_width(road, reach, s);
        if obb_overlap([p.x, p.y, heading, 0.6, half_width], target) {
            // Refine the first contact to 0.1 m inside the last 1 m step,
            // measured at the slice front edge.
            let mut contact = s;
            let mut probe = s - 0.9;
            while probe < s {
                if probe >= 0.0
                    && let Some((q, h)) = path.sample(probe)
                    && obb_overlap([q.x, q.y, h, 0.6, half_width], target)
                {
                    contact = probe;
                    break;
                }
                probe += 0.1;
            }
            return Some((contact + 0.6 - rear - REAR_AXLE_TO_FRONT_M).max(0.0));
        }
        s += 1.0;
    }
    None
}
/// Near the truck the corridor also covers its current lateral offset.
fn corridor_half_width(road: &RoadState, reach: f64, s: f64) -> f64 {
    reach + road.lateral_m.abs() * (1.0 - (s - road.station_m) / 20.0).clamp(0.0, 1.0)
}
/// Separating-axis test of two oriented rectangles
/// `[centre x, centre y, yaw, half length, half width]`.
fn obb_overlap(a: [f64; 5], b: [f64; 5]) -> bool {
    let extent = |o: [f64; 5], axis: f64| {
        o[3] * (o[2] - axis).cos().abs() + o[4] * (o[2] - axis).sin().abs()
    };
    [
        a[2],
        a[2] + std::f64::consts::FRAC_PI_2,
        b[2],
        b[2] + std::f64::consts::FRAC_PI_2,
    ]
    .into_iter()
    .all(|axis| {
        let distance = ((b[0] - a[0]) * axis.cos() + (b[1] - a[1]) * axis.sin()).abs();
        distance <= extent(a, axis) + extent(b, axis)
    })
}

fn simple_lane(traffic: &TrafficContext, id: &str) -> bool {
    if !traffic.has_annotations_for_route(&[id.to_owned()]) {
        return false;
    }
    if !traffic.stops_for_lane(id).is_empty()
        || !traffic.signals_for_lane(id).is_empty()
        || !traffic.conflicts_for_lane(id).is_empty()
    {
        return false;
    }
    id.contains(":road:")
        || traffic
            .control_for_lane(id)
            .is_some_and(|c| c.control_kind == ControlKind::PriorityRoad)
}

/// Measured tractor width is about 2.5 m. The clearance covers tracking error;
/// together they are the swept corridor, not a forged lane width.
const EGO_HALF_WIDTH_M: f64 = 1.3;
const LATERAL_CLEARANCE_M: f64 = 0.45;
/// Rear-axle reference to the front bumper of the measured four-wheel tractor.
const REAR_AXLE_TO_FRONT_M: f64 = 5.0;
/// Observed lateral drift below this is lane-keeping noise, not a cut-in.
const LATERAL_RATE_DEADBAND_MPS: f64 = 0.25;
const MAX_LATERAL_RATE_MPS: f64 = 1.5;
const PREDICTION_TIMES_S: [f64; 7] = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
/// Crossing bodies do not follow this path; straight extrapolation is only
/// trusted for a short horizon.
const CROSSING_HORIZON_S: f64 = 2.0;
/// Planned deceleration toward the end of the usable route prefix.
const STOP_DECELERATION: f64 = 1.2;
/// Within this distance a mapped light that ETS2LA does not report is dark.
const DARK_SIGNAL_DISTANCE_M: f64 = 60.0;
/// Bodies slower than this are tested as stationary footprints.
const STATIONARY_SPEED: f64 = 0.5;
/// Speed on the lead-in curve from a yard onto the first lane (m/s).
const LEAD_IN_SPEED: f64 = 8.0 / 3.6;
/// Bend speed: lateral acceleration 2.0 m/s² up to 40 km/h, easing to
/// 1.8 m/s² at 90 km/h. r11 used 1.2 m/s² everywhere and crept through
/// town bends at 20–30 km/h with 0.2 m p95 tracking error to spare. r12
/// held 1.5 m/s² and more above 50 km/h with no offset beyond 0.5 m.
const CURVE_LATERAL_LOW: f64 = 2.0;
const CURVE_LATERAL_HIGH: f64 = 1.8;
const CURVE_LATERAL_TIGHT: f64 = 1.5;
const TIGHT_RADIUS_M: f64 = 30.0;
/// Below this radius the lateral limit falls to `CURVE_LATERAL_LOCK` at 6 m.
const VERY_TIGHT_RADIUS_M: f64 = 12.0;
const CURVE_LATERAL_LOCK: f64 = 0.6;
const OPEN_RADIUS_M: f64 = 80.0;
const CURVE_LATERAL_FADE_FROM: f64 = 40.0 / 3.6;
const CURVE_LATERAL_FADE_TO: f64 = 90.0 / 3.6;
/// Half chord of the three-point curvature estimate (m).
const CURVE_CHORD_HALF_M: f64 = 5.0;
/// Spacing of the curvature samples ahead (m).
const CURVE_STEP_M: f64 = 3.0;
/// Wheel rate a curvature change may demand (rad/s) from 10 m/s, below the
/// measured 0.18 rad/s so tracking corrections keep some authority. The
/// steering slew doubles towards 5 m/s and so does this rate: r12 turned the
/// wheel at up to 0.8 rad/s below 30 km/h but crept through junction turns
/// at the 10 m/s rate.
const WHEEL_RATE_RAD_S: f64 = 0.15;
/// Fastest speed at which a curvature change of `rate` (1/m per m) can be
/// steered: wheelbase · rate · v must not exceed the wheel rate at v.
fn transition_speed(rate: f64) -> f64 {
    let demand = WHEELBASE_M * rate.max(1e-9);
    let slow = 2.0 * WHEEL_RATE_RAD_S / demand;
    if slow <= 5.0 {
        return slow;
    }
    // Between 5 and 10 m/s the wheel rate is 1.5 / v.
    let middle = (10.0 * WHEEL_RATE_RAD_S / demand).sqrt();
    if middle <= 10.0 {
        middle
    } else {
        WHEEL_RATE_RAD_S / demand
    }
}
/// Planning wheelbase of a two-axle tractor (m).
const WHEELBASE_M: f64 = 3.8;
/// Planned deceleration into a bend (m/s²).
const CURVE_DECELERATION: f64 = 1.5;
/// Following time gap behind a leader (s).
const FOLLOW_HEADWAY_S: f64 = 2.0;
/// Tight turns (roundabouts, junctions) at 1.5 m/s²: L1 circled R 24
/// roundabouts at 2.0 m/s² and drifted 0.8–1.1 m at every entry and exit.
fn tight_lateral(curvature: f64) -> f64 {
    let radius = 1.0 / curvature.max(1e-6);
    // Turns near the steering lock (yard exits): r13 took a 5 m radius at
    // 9 km/h and needed more wheel than the calibrated range.
    if radius < VERY_TIGHT_RADIUS_M {
        let t = ((radius - 6.0) / (VERY_TIGHT_RADIUS_M - 6.0)).clamp(0.0, 1.0);
        return CURVE_LATERAL_LOCK + (CURVE_LATERAL_TIGHT - CURVE_LATERAL_LOCK) * t;
    }
    let t = ((radius - TIGHT_RADIUS_M) / (OPEN_RADIUS_M - TIGHT_RADIUS_M)).clamp(0.0, 1.0);
    CURVE_LATERAL_TIGHT + (CURVE_LATERAL_LOW - CURVE_LATERAL_TIGHT) * t
}
fn curve_lateral(speed: f64) -> f64 {
    let t = ((speed - CURVE_LATERAL_FADE_FROM) / (CURVE_LATERAL_FADE_TO - CURVE_LATERAL_FADE_FROM))
        .clamp(0.0, 1.0);
    CURVE_LATERAL_LOW + (CURVE_LATERAL_HIGH - CURVE_LATERAL_LOW) * t
}
/// The constraint that set the speed target, logged to find what holds the
/// truck below the limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Binding {
    Limit,
    Ceiling,
    LeadIn,
    Curve,
    /// The wheel cannot turn fast enough for a change of curvature.
    CurveRate,
    RouteEnd,
    JunctionCaution,
    JunctionStop,
    Traffic,
}

/// Lateral position of another body relative to the fixed GPS path. Cut-ins
/// use this observed drift; single-frame heading extrapolation turns every
/// vehicle that follows a bend into a phantom intruder.
#[derive(Clone, Copy)]
struct LateralTrack {
    time_s: f64,
    d: f64,
    rate: f64,
    last_seen_s: f64,
}
type TrackKey = (u64, usize, u64);

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub enum Motion {
    SameDirection,
    Opposing,
    Crossing,
}
/// The constraint that produced the lowest gap, logged so a braking event can
/// be attributed to one object and one prediction step.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct LeadEvidence {
    pub agent_id: u64,
    /// 0 is the vehicle body, 1.. are its trailers.
    pub body: usize,
    pub motion: Motion,
    pub prediction_s: f64,
    /// Body centre along the path, relative to the ego rear axle.
    pub station_m: f64,
    pub lateral_m: f64,
    pub half_lateral_m: f64,
    pub lateral_rate_mps: f64,
    pub relative_heading_rad: f64,
    pub speed_mps: f64,
    pub gap_m: f64,
}

pub struct Guidance {
    roads: Roads,
    lane_ends: Vec<(String, f64, u64)>,
    gps: Vec<u64>,
    reached_prefix: usize,
    pub terminal: String,
    progress_s: f64,
    tracks: HashMap<TrackKey, LateralTrack>,
    approach_cap: Option<f64>,
    controls: Vec<RouteControl>,
    signal_heads: TrafficContext,
    signals_seen: Vec<SignalObservation>,
    /// Stop-sign dwell start time; `INFINITY` once the dwell is complete.
    stop_dwell: HashMap<String, f64>,
    traffic_lanes: TrafficLanes,
    /// Length of the slow curve from a yard onto the first lane (0 if none).
    lead_in_m: f64,
    lane_changes: Vec<LaneChange>,
}
/// Every lane centre of the prepared region, cars' lanes included, to predict
/// moving traffic along its lane instead of straight ahead: on a bend, a
/// straight 2 s extrapolation of an oncoming car crosses our lane (r10 braked
/// and stopped for cars 20 m beside the route).
#[derive(Default)]
struct TrafficLanes {
    lanes: Vec<Road>,
}
/// A body matched to a lane: station and offset along it.
struct LaneFollow {
    lane: usize,
    s: f64,
    d: f64,
}
impl TrafficLanes {
    fn new(lanes: &[etsads_map::importer::ExportLane]) -> Self {
        Self {
            lanes: lanes
                .iter()
                .filter_map(|l| Road::path(l.id.clone(), l.center.clone()).ok())
                .collect(),
        }
    }
    /// The lane this body is driving along, if one is close and aligned.
    fn matched(&self, pose: Pose2, z: f64) -> Option<LaneFollow> {
        let mut best = None::<(f64, LaneFollow)>;
        for (index, lane) in self.lanes.iter().enumerate() {
            if !lane.near(pose.x, pose.y, 2.0) {
                continue;
            }
            let Ok((p, _)) = project_polyline(
                0,
                &lane.points,
                Point3 {
                    x: pose.x,
                    y: pose.y,
                    z,
                },
            ) else {
                continue;
            };
            if p.distance > 1.8
                || (p.z - z).abs() > 2.0
                || wrap_angle(p.heading - pose.yaw).abs() > 0.5
                || p.distance > p.d.abs() + 0.5
            {
                continue;
            }
            if best.as_ref().is_none_or(|(d, _)| p.distance < *d) {
                best = Some((
                    p.distance,
                    LaneFollow {
                        lane: index,
                        s: p.s,
                        d: p.d,
                    },
                ));
            }
        }
        best.map(|b| b.1)
    }
    /// Where a lane-following body is after `travel` metres; past the lane's
    /// end (a junction) it carries on straight.
    fn advance(&self, follow: &LaneFollow, yaw: f64, travel: f64) -> Pose2 {
        let lane = &self.lanes[follow.lane];
        let s = (follow.s + travel).clamp(0.0, lane.length());
        let beyond = follow.s + travel - s;
        let (p, heading) = lane.sample(s).unwrap_or((lane.points[0], yaw));
        Pose2 {
            x: p.x - follow.d * heading.sin() + beyond * heading.cos(),
            y: p.y + follow.d * heading.cos() + beyond * heading.sin(),
            yaw: heading,
        }
    }
}
impl Guidance {
    /// Read-only diagnostic of the junction rules on this route.
    pub fn describe_controls(&self) -> serde_json::Value {
        serde_json::json!({
            "controls": self.controls.iter().map(|c| serde_json::json!({
                "station_m": c.station_m, "kind": c.kind, "priority": c.priority,
                "group": c.group, "conservative": c.conservative,
                "conflict_lanes": c.conflicts.len(), "left_turn": c.left_turn, "key": c.key,
            })).collect::<Vec<_>>(),
            "signal_heads": self.signal_heads.signals.iter().map(|s| serde_json::json!({
                "id": s.id, "kind": s.kind, "x": s.x, "y": s.y, "z": s.z,
            })).collect::<Vec<_>>(),
        })
    }
    /// Fresh ETS2LA traffic-light states for this frame.
    pub fn observe_signals(&mut self, signals: Vec<SignalObservation>) {
        self.signals_seen = signals;
    }
    /// The nearest junction stop ahead (front-bumper gap and reason), a speed
    /// cap for controls that only require a cautious approach, and a notice
    /// for the driver (e.g. a dark traffic light handled as give-way).
    /// A signal without group data in the map: use the observed lamp head
    /// nearest the stop line (within 15 m, normally our own). None observed
    /// is handled like a dark signal (give way).
    fn nearest_signal(&self, station_m: f64) -> SignalDecision {
        let Some((stop, _)) = self.roads.roads["gps"].sample(station_m) else {
            return SignalDecision::Hold(SignalHoldReason::MissingGroup);
        };
        self.signals_seen
            .iter()
            .filter(|s| s.age_us <= 200_000 && (s.position.z - stop.z).abs() < 8.0)
            .map(|s| (s.position.distance_xy(stop), s.raw_state))
            .filter(|(d, _)| *d <= 15.0)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map_or(
                SignalDecision::Hold(SignalHoldReason::MissingGroup),
                |(_, state)| {
                    if state == 8 {
                        SignalDecision::Proceed
                    } else {
                        SignalDecision::Hold(SignalHoldReason::NotGreen(state))
                    }
                },
            )
    }
    fn junction_rules(
        &mut self,
        road: &RoadState,
        speed: f64,
        agents: &[Agent],
        time_s: f64,
    ) -> (Option<(f64, String)>, f64, Option<String>) {
        let mut stop: Option<(f64, String)> = None;
        let mut cap = f64::INFINITY;
        let mut notice = None::<String>;
        let controls = std::mem::take(&mut self.controls);
        for control in &controls {
            let distance = control.station_m - road.station_m - REAR_AXLE_TO_FRONT_M - 1.0;
            if !(-2.0..=250.0).contains(&distance) {
                continue;
            }
            let mut hold = None::<&str>;
            let mut cautious = false;
            let mut green = false;
            let mut note = None::<&str>;
            if control.kind == ControlKind::Signal || control.priority == RightOfWay::SignalRequired
            {
                let decision = match control.group.as_deref() {
                    Some(group) => {
                        self.signal_heads
                            .signal_decision(group, &self.signals_seen, 1.5, 200_000)
                    }
                    None => self.nearest_signal(control.station_m),
                };
                // Amber: go on only when a comfortable stop is impossible.
                let late_amber = matches!(
                    decision,
                    SignalDecision::Hold(SignalHoldReason::NotGreen(1))
                ) && speed * 0.3 + speed * speed / (2.0 * 2.0) > distance.max(0.0);
                // A light that is not working (no lamp heads in the map, none
                // observed although ETS2LA's 40-slot list has room, or OFF /
                // SLEEP) makes this an unsignalised junction: give way. Far
                // away a missing observation is only "not known yet".
                let unobserved = matches!(
                    decision,
                    SignalDecision::Hold(SignalHoldReason::MissingObservation)
                ) && self.signals_seen.len() < 40;
                let dark = matches!(
                    decision,
                    SignalDecision::Hold(
                        SignalHoldReason::MissingGroup | SignalHoldReason::NotGreen(0 | 32)
                    )
                ) || (unobserved && distance <= DARK_SIGNAL_DISTANCE_M);
                if dark || (unobserved && distance > DARK_SIGNAL_DISTANCE_M) {
                    cautious = true;
                    if dark {
                        note = Some("信号灯未工作，按让行规则通过");
                    }
                } else if !decision.can_proceed() && !late_amber {
                    hold = Some(match decision {
                        SignalDecision::Hold(SignalHoldReason::NotGreen(_)) => "红灯，停车等待",
                        _ => "看不到信号灯状态，停车等待",
                    });
                } else {
                    green = decision.can_proceed();
                }
            }
            if control.kind == ControlKind::Stop {
                let done = self
                    .stop_dwell
                    .get(&control.key)
                    .is_some_and(|t| t.is_infinite());
                if !done {
                    if distance <= 1.0 && speed.abs() < 0.1 {
                        let since = *self.stop_dwell.entry(control.key.clone()).or_insert(time_s);
                        if time_s - since >= 1.0 {
                            self.stop_dwell.insert(control.key.clone(), f64::INFINITY);
                        } else {
                            hold = Some("停车让行：先停稳");
                        }
                    } else {
                        hold = Some("停车让行：在停止线前停车");
                    }
                }
            }
            // Conflicting traffic: arrival time at the conflict point.
            let mut arrivals = Vec::new();
            for lane in &control.conflicts {
                for agent in agents {
                    let Ok((p, _)) = project_polyline(
                        0,
                        &lane.points,
                        Point3 {
                            x: agent.pose.x,
                            y: agent.pose.y,
                            z: agent.z,
                        },
                    ) else {
                        continue;
                    };
                    if p.distance > 2.5
                        || (p.z - agent.z).abs() > 3.0
                        || wrap_angle(agent.pose.yaw - p.heading).abs() > 0.6
                    {
                        continue;
                    }
                    let gap = lane.conflict_s - p.s - agent.length / 2.0;
                    if gap < -agent.length {
                        continue;
                    }
                    if gap <= 0.0 {
                        arrivals.push(0.0);
                    } else if agent.speed > 0.3 {
                        let v = agent.speed;
                        arrivals.push((v * v + 2.0 * gap).sqrt() - v);
                    }
                }
            }
            let ego_arrival = distance.max(0.0) / speed.max(3.0);
            // Traffic is fully known when every conflicting lane was found
            // and ETS2LA's 40 vehicle slots are not all taken.
            let visible = control.complete && agents.len() < TRAFFIC_SLOTS - 2;
            let zip = control.priority == RightOfWay::ZipMerge && visible && !cautious;
            if zip {
                // A zip merge is not a stop: fall in 1.5 s behind traffic
                // that reaches the merge first; traffic behind falls in
                // behind the truck.
                for t in arrivals.iter().filter(|t| **t <= ego_arrival + 0.5) {
                    cap = cap.min(distance.max(0.0) / (t + ZIP_FOLLOW_S));
                }
            } else if !arrivals.is_empty()
                && etsads_planning::behavior::must_yield(ego_arrival, &arrivals, control.left_turn)
            {
                hold = Some("让行：等待冲突车道来车");
            }
            if let Some(reason) = hold {
                if stop.as_ref().is_none_or(|(d, _)| distance < *d) {
                    stop = Some((distance.max(0.0), reason.to_owned()));
                }
            } else if (control.conservative || cautious) && distance > 0.0 && !zip {
                // A green light with every conflict visible needs no extra
                // care. A give-way whose approaches are clear is crossed at
                // 30 km/h; an unknown rule, a dark light or unseen traffic at
                // 15 km/h.
                let line = if green && visible {
                    continue;
                } else if visible && !cautious && !unknown_rule(control.kind, control.priority) {
                    CLEAR_LINE_SPEED
                } else {
                    CAUTION_LINE_SPEED
                };
                if let Some(note) = note.or_else(|| {
                    unknown_rule(control.kind, control.priority)
                        .then_some("路口规则未知，按让行规则通过")
                }) {
                    notice.get_or_insert_with(|| note.to_owned());
                }
                cap = cap.min((line * line + 2.0 * STOP_DECELERATION * distance).sqrt());
            }
        }
        self.controls = controls;
        (stop, cap, notice)
    }
    /// Replace the route with a freshly prepared one while driving, if it
    /// coincides with the current path at the truck and reaches further.
    /// Returns the extension in metres, or `None` when the old route stays.
    pub fn adopt(
        &mut self,
        mut next: Guidance,
        pose: Pose2,
        z: f64,
        speed: f64,
    ) -> anyhow::Result<Option<f64>> {
        let current =
            self.roads
                .observe_near(pose, z, speed, Some("gps"), Some(self.progress_s))?;
        let candidate = next.roads.observe(pose, z, speed, Some("gps"))?;
        ensure!(
            (candidate.lateral_m - current.lateral_m).abs() < 0.3
                && wrap_angle(candidate.heading_error_rad - current.heading_error_rad).abs() < 0.05,
            "新导航路段与当前路径不重合"
        );
        let extension = candidate.remaining_m - current.remaining_m;
        if extension < 30.0 {
            return Ok(None);
        }
        next.progress_s = candidate.station_m;
        // Offsets are measured from coinciding centrelines, so lateral
        // tracks stay valid across the swap.
        next.tracks = std::mem::take(&mut self.tracks);
        next.approach_cap = None;
        next.stop_dwell = std::mem::take(&mut self.stop_dwell);
        *self = next;
        Ok(Some(extension))
    }
    pub fn observe(
        &mut self,
        pose: Pose2,
        z: f64,
        speed: f64,
        gps: &[u64],
    ) -> anyhow::Result<RoadState> {
        let mut gps = gps.to_vec();
        gps.dedup();
        ensure!(!gps.is_empty(), "GPS路线消失，请接管");
        let road = self
            .roads
            .observe_near(pose, z, speed, Some("gps"), Some(self.progress_s))?;
        ensure!(
            road.station_m + 3.0 >= self.progress_s && road.station_m - self.progress_s < 15.0,
            "导航定位跳变，请接管"
        );
        if gps != self.gps {
            let consumed = (1..self.gps.len())
                .find(|&i| self.gps[i..] == gps)
                .context("GPS改道或目的地变化，请重新准备辅助")?;
            for (index, node) in self.gps[..consumed].iter().enumerate() {
                let reached = self
                    .lane_ends
                    .iter()
                    .any(|(_, s, end)| end == node && *s <= road.station_m + 10.0)
                    || index < self.reached_prefix;
                ensure!(reached, "GPS跳过尚未到达的路段，请接管");
            }
            self.gps = gps;
            self.reached_prefix = self.reached_prefix.saturating_sub(consumed);
        }
        self.progress_s = road.station_m;
        Ok(road)
    }
    /// The steered path from 5 m behind to 90 m ahead of station `s`, in at
    /// most `PLANNED_PATH_MAX` points, for the independent supervisor.
    pub fn path_ahead(&self, s: f64) -> Vec<[f64; 2]> {
        use etsads_msgs::protocol::PLANNED_PATH_MAX;
        let path = &self.roads.roads["gps"];
        let start = (s - 5.0).clamp(0.0, path.length());
        let end = (s + 90.0).min(path.length());
        let step = ((end - start) / (PLANNED_PATH_MAX - 1) as f64).max(1.0);
        (0..PLANNED_PATH_MAX)
            .map(|i| start + i as f64 * step)
            .take_while(|t| *t <= end + 1e-6)
            .filter_map(|t| path.sample(t.min(end)))
            .map(|(p, _)| [p.x, p.y])
            .collect()
    }
    /// Index of the GPS checkpoint this route was resolved from.
    pub fn checkpoint(&self) -> usize {
        self.reached_prefix
    }
    pub fn lane(&self, s: f64) -> &str {
        self.lane_ends
            .iter()
            .find(|(_, end, _)| *end >= s)
            .map_or("gps", |(id, _, _)| id)
    }
    /// Index of the last GPS checkpoint passed by station `s`; a route
    /// prepared from there begins at the truck's current checkpoint.
    pub fn passed_checkpoint(&self, s: f64) -> usize {
        let mut index = self.reached_prefix.min(self.gps.len().saturating_sub(1));
        for (_, end_s, node) in &self.lane_ends {
            if *end_s > s {
                break;
            }
            if let Some(i) = self.gps[index..].iter().position(|n| n == node) {
                index += i;
            }
        }
        index
    }
    #[allow(clippy::too_many_arguments)]
    pub fn speed(
        &mut self,
        road: &RoadState,
        pose: Pose2,
        speed: f64,
        limit: f64,
        ceiling: f64,
        agents: &[Agent],
        time_s: f64,
    ) -> anyhow::Result<SpeedPlan> {
        ensure!(
            limit.is_finite()
                && (0.0..=45.0).contains(&limit)
                && speed.is_finite()
                && speed >= -0.2
                && ceiling.is_finite()
                && (0.0..=super::core::MAX_SPEED).contains(&ceiling)
                && time_s.is_finite(),
            "当前道路限速数据无效"
        );
        let (mut junction_stop, junction_cap, mut junction_note) =
            self.junction_rules(road, speed, agents, time_s);
        // A lane change waits at its start point until the target lane has
        // a gap; once started it is completed.
        if let Some(hold) = self.lane_change_hold(road, speed, agents)
            && junction_stop.as_ref().is_none_or(|(g, _)| hold.0 < *g)
        {
            junction_stop = Some(hold);
        }
        if let Some(change) = self.active_lane_change(road.station_m, speed) {
            junction_note.get_or_insert_with(|| {
                format!(
                    "{}向{}换道",
                    if road.station_m >= change.start_m {
                        "正在"
                    } else {
                        "准备"
                    },
                    if change.left { "左" } else { "右" }
                )
            });
        }
        let path = &self.roads.roads["gps"];
        // Anticipate curvature over the braking horizon; do not wait until the
        // truck enters the bend. 1.2 m/s² lateral and 1.5 m/s² longitudinal.
        // A fresh zero is the user's requested "no displayed restriction"
        // case. It uses the explicit cruise ceiling, never an infinite speed.
        let (mut target, mut binding) = if limit == 0.0 || ceiling < limit - 0.3 {
            (ceiling, Binding::Ceiling)
        } else {
            ((limit - 0.3).max(0.0), Binding::Limit)
        };
        let mut bind = |target: &mut f64, value: f64, why: Binding| {
            if value < *target {
                *target = value;
                binding = why;
            }
        };
        // Leaving a yard on the lead-in curve: walking pace until on the lane.
        if road.station_m < self.lead_in_m {
            bind(&mut target, LEAD_IN_SPEED, Binding::LeadIn);
        }
        let horizon = (speed * speed / 3.0 + speed * 3.0 + 30.0).max(60.0);
        let mut d = 0.0;
        let mut previous = [None::<f64>; 2];
        while d <= horizon && road.station_m + d < path.length() {
            let signed = path.chord_curvature(road.station_m + d, CURVE_CHORD_HALF_M);
            let curvature = signed.abs();
            // Where curvature changes (a roundabout exit, an S-bend) the
            // wheel must turn at wheelbase · dκ/ds · v. The 2.0 m/s² L1
            // roundabout exit at 27 km/h reversed the wheel too late and
            // drifted 0.9 m.
            // A reversal keeps the 10 m/s wheel rate at every speed.
            if let Some(last) = previous[1] {
                let rate = (signed - last).abs() / (2.0 * CURVE_STEP_M);
                let transition = if signed * last < 0.0 {
                    WHEEL_RATE_RAD_S / (WHEELBASE_M * rate.max(1e-6))
                } else {
                    transition_speed(rate)
                };
                bind(
                    &mut target,
                    (transition * transition
                        + 2.0 * CURVE_DECELERATION * (d - 2.0 * CURVE_STEP_M - 8.0).max(0.0))
                    .sqrt(),
                    Binding::CurveRate,
                );
            }
            previous = [Some(signed), previous[0]];
            // The corner speed is solved with the lateral limit at that speed.
            let mut corner = (tight_lateral(curvature) / curvature.max(1e-6)).sqrt();
            if corner > CURVE_LATERAL_FADE_FROM {
                corner = corner.min((curve_lateral(corner) / curvature.max(1e-6)).sqrt());
            }
            bind(
                &mut target,
                (corner * corner + 2.0 * CURVE_DECELERATION * (d - 8.0).max(0.0)).sqrt(),
                Binding::Curve,
            );
            d += CURVE_STEP_M;
        }
        let stop_gap = (road.remaining_m - 18.0).max(0.0);
        // Comfortable stop at the end of the usable prefix. Inside the final
        // approach the target may only fall: r9 braked hard below the 3 m/s²
        // profile, fell under it and re-accelerated before stopping.
        let profile = (2.0 * STOP_DECELERATION * stop_gap).sqrt();
        bind(&mut target, profile, Binding::RouteEnd);
        let approach = stop_gap < speed * speed / (2.0 * STOP_DECELERATION) + 25.0;
        if stop_gap > speed * speed / (2.0 * STOP_DECELERATION) + 40.0 {
            self.approach_cap = None;
        }
        // Junction rules: a stop target (red light, stop sign, yield) and a
        // cautious-approach cap. Unlike the route end these may clear again.
        bind(&mut target, junction_cap, Binding::JunctionCaution);
        if let Some((gap, _)) = &junction_stop {
            bind(
                &mut target,
                (2.0 * STOP_DECELERATION * gap).sqrt(),
                Binding::JunctionStop,
            );
        }
        let idm = etsads_planning::speed::IdmConfig {
            desired_speed: target.max(0.1),
            time_headway: FOLLOW_HEADWAY_S,
            minimum_gap: 10.0,
            max_acceleration: 1.2,
            ..Default::default()
        };
        let free = idm.acceleration(speed.max(0.0), None)?;
        let mut acceleration = free;
        let (traffic_path, station_offset) = path.window(
            (road.station_m - 20.0).max(0.0),
            road.station_m + horizon + 60.0,
        );
        // Swept corridor of the rear axle and the front bumper, both relative
        // to the path centre; tracking error widens it on its own side only.
        let front_lateral = road.lateral_m - REAR_AXLE_TO_FRONT_M * road.heading_error_rad.sin();
        let reach = EGO_HALF_WIDTH_M + LATERAL_CLEARANCE_M;
        let corridor = (
            road.lateral_m.min(front_lateral).min(0.0) - reach,
            road.lateral_m.max(front_lateral).max(0.0) + reach,
        );
        let mut lead = None::<LeadEvidence>;
        for agent in agents {
            ensure!(agent.valid(), "交通对象数据无效");
            if (agent.pose.x - pose.x).hypot(agent.pose.y - pose.y) > horizon + 120.0 {
                continue;
            }
            let bodies = std::iter::once(OrientedBox {
                pose: agent.pose,
                length: agent.length,
                width: agent.width,
            })
            .chain(agent.trailers.iter().copied());
            for (index, body) in bodies.enumerate() {
                // Height must be compared at the other body's road station.
                // Comparing with ego altitude would discard leaders on hills.
                // A degenerate window (a very short path end) has no tangent;
                // such a body cannot be judged here and is left to the AEB.
                let Ok((current, _)) = project_polyline(
                    0,
                    traffic_path,
                    Point3 {
                        x: body.pose.x,
                        y: body.pose.y,
                        z: agent.z,
                    },
                ) else {
                    continue;
                };
                // A clamped projection beyond either window end is not beside
                // this path section; its distance exceeds the normal offset.
                if (agent.z - current.z).abs() > 2.5 || current.distance > current.d.abs() + 0.5 {
                    continue;
                }
                let angle = wrap_angle(body.pose.yaw - current.heading);
                let half_station =
                    body.length * angle.cos().abs() / 2.0 + body.width * angle.sin().abs() / 2.0;
                let half_lateral =
                    body.length * angle.sin().abs() / 2.0 + body.width * angle.cos().abs() / 2.0;
                let station = current.s + station_offset - road.station_m;
                let key = (agent.id, index, body.length.to_bits());
                let lateral_rate = Self::track(&mut self.tracks, key, time_s, current.d);
                let motion = if angle.abs() < 0.35 {
                    Motion::SameDirection
                } else if angle.abs() > std::f64::consts::PI - 0.35 {
                    Motion::Opposing
                } else {
                    Motion::Crossing
                };
                let along = agent.speed * angle.cos();
                // Stationary and crossing bodies: test the actual footprint
                // against the swept corridor along the path. Projecting a
                // crosswise body near a sharp turn misjudged its offset and
                // deadlocked against a car waiting to give way to us (r9).
                if motion == Motion::Crossing || agent.speed.abs() < STATIONARY_SPEED {
                    let moving = agent.speed.abs() >= STATIONARY_SPEED;
                    let follow = if moving {
                        self.traffic_lanes.matched(body.pose, agent.z)
                    } else {
                        None
                    };
                    let mut best = None::<(f64, f64)>;
                    for t in [0.0, 0.5, 1.0, 1.5, 2.0] {
                        if t > 0.0 && !moving {
                            break;
                        }
                        let moved = OrientedBox {
                            pose: match &follow {
                                Some(f) => {
                                    self.traffic_lanes
                                        .advance(f, body.pose.yaw, agent.speed * t)
                                }
                                None => Pose2 {
                                    x: body.pose.x + agent.speed * t * body.pose.yaw.cos(),
                                    y: body.pose.y + agent.speed * t * body.pose.yaw.sin(),
                                    yaw: body.pose.yaw,
                                },
                            },
                            ..body
                        };
                        // A body that cannot reach the corridor needs no sweep.
                        let travel = agent.speed.abs() * t;
                        if current.distance
                            > reach
                                + road.lateral_m.abs()
                                + body.length.hypot(body.width) / 2.0
                                + travel
                                + 1.0
                        {
                            continue;
                        }
                        if let Some(gap) = sweep_gap(path, road, moved, reach, horizon + 60.0)
                            && best.is_none_or(|(g, _)| gap < g)
                        {
                            best = Some((gap, t));
                        }
                    }
                    if let Some((gap, t)) = best {
                        acceleration =
                            acceleration.min(idm.acceleration(speed.max(0.0), Some((gap, 0.0)))?);
                        if lead.is_none_or(|l| gap < l.gap_m) {
                            lead = Some(LeadEvidence {
                                agent_id: agent.id,
                                body: index,
                                motion,
                                prediction_s: t,
                                station_m: station,
                                lateral_m: current.d,
                                half_lateral_m: half_lateral,
                                lateral_rate_mps: lateral_rate,
                                relative_heading_rad: angle,
                                speed_mps: 0.0,
                                gap_m: gap,
                            });
                        }
                    }
                    continue;
                }
                for t in PREDICTION_TIMES_S {
                    // Path-following bodies keep their observed offset and only
                    // drift by the measured lateral rate. Crossing bodies move
                    // straight and are stationary obstructions where they land.
                    let (s, d) = match motion {
                        Motion::SameDirection => (station, current.d + lateral_rate * t),
                        Motion::Opposing => {
                            (station + along.min(0.0) * t, current.d + lateral_rate * t)
                        }
                        Motion::Crossing => {
                            if t > CROSSING_HORIZON_S {
                                break;
                            }
                            let point = Point3 {
                                x: body.pose.x + agent.speed * t * body.pose.yaw.cos(),
                                y: body.pose.y + agent.speed * t * body.pose.yaw.sin(),
                                z: agent.z,
                            };
                            let Ok((p, _)) = project_polyline(0, traffic_path, point) else {
                                continue;
                            };
                            if p.distance > p.d.abs() + 0.5 {
                                continue;
                            }
                            (p.s + station_offset - road.station_m, p.d)
                        }
                    };
                    if d + half_lateral <= corridor.0
                        || d - half_lateral >= corridor.1
                        || s + half_station < -2.0
                    {
                        continue;
                    }
                    let lead_speed = if motion == Motion::SameDirection {
                        along.max(0.0)
                    } else {
                        0.0
                    };
                    let gap = (s - half_station - REAR_AXLE_TO_FRONT_M).max(0.0);
                    acceleration = acceleration
                        .min(idm.acceleration(speed.max(0.0), Some((gap, lead_speed)))?);
                    if lead.is_none_or(|l| gap < l.gap_m) {
                        lead = Some(LeadEvidence {
                            agent_id: agent.id,
                            body: index,
                            motion,
                            prediction_s: t,
                            station_m: s,
                            lateral_m: d,
                            half_lateral_m: half_lateral,
                            lateral_rate_mps: lateral_rate,
                            relative_heading_rad: angle,
                            speed_mps: lead_speed,
                            gap_m: gap,
                        });
                    }
                    // The earliest overlapping step binds a path-following
                    // body; its later steps have the same or a larger gap.
                    if motion != Motion::Crossing {
                        break;
                    }
                }
            }
        }
        self.tracks.retain(|_, t| time_s - t.last_seen_s < 2.0);
        let traffic_bound = acceleration < free - 0.05;
        if stop_gap < speed * speed / (2.0 * STOP_DECELERATION) + 3.0 {
            acceleration = acceleration.min(-STOP_DECELERATION);
        }
        if junction_stop
            .as_ref()
            .is_some_and(|(gap, _)| *gap < speed * speed / (2.0 * STOP_DECELERATION) + 0.5)
        {
            acceleration = acceleration.min(-STOP_DECELERATION);
        }
        // On a free road the pedal controller drives toward the target
        // itself; capping it at one second of IDM acceleration held the
        // throttle near 0.3 and every restart crawled (r11 town driving).
        // Only a constraint from traffic or a hard stop shapes it here.
        if acceleration < free - 0.05 || acceleration < 0.0 {
            // Accelerating behind traffic follows three seconds of the IDM
            // rate, so a leader pulling away is followed briskly.
            let horizon_s = if acceleration > 0.0 { 3.0 } else { 1.0 };
            let shaped = (speed + acceleration * horizon_s).max(0.0);
            if shaped < target {
                target = shaped;
                if traffic_bound {
                    binding = Binding::Traffic;
                }
            }
        }
        if approach {
            // Monotone within the approach; a truck that stopped short may
            // still creep (at most 1.5 m/s) up to the stop point.
            let floor = profile.min(1.5).min(target);
            target = target
                .min(self.approach_cap.unwrap_or(f64::INFINITY))
                .max(floor);
            self.approach_cap = Some(target);
        }
        let junction_approach = junction_stop
            .as_ref()
            .is_some_and(|(gap, _)| *gap < speed * speed / (2.0 * STOP_DECELERATION) + 25.0);
        let reason = if let Some((_, reason)) = &junction_stop {
            reason.clone()
        } else if stop_gap < speed * speed / 3.0 + speed * 3.0 + 30.0 {
            self.terminal.clone()
        } else if let Some(note) = junction_note {
            note
        } else if lead.is_some_and(|l| l.gap_m < speed * 4.0 + 25.0) {
            "前车 / 切入交通，保持安全距离".into()
        } else {
            "按 GPS 行驶 · 随路段限速调整".into()
        };
        Ok(SpeedPlan {
            target,
            acceleration,
            binding,
            approach: approach || junction_approach,
            lead_gap: lead.map(|l| l.gap_m),
            lead,
            reason,
        })
    }
    /// The lane change being approached (within about 6 s) or performed.
    fn active_lane_change(&self, station: f64, speed: f64) -> Option<&LaneChange> {
        self.lane_changes
            .iter()
            .find(|c| station < c.end_m && c.start_m - station < (speed * 6.0).max(50.0))
    }
    /// Turn signal for the route: `Some(true)` left, `Some(false)` right.
    pub fn blinker(&self, station: f64, speed: f64) -> Option<bool> {
        self.active_lane_change(station, speed).map(|c| c.left)
    }
    /// Stop short of a lane change whose target lane has no gap: a vehicle
    /// that would be beside us when the blend starts, assuming both keep
    /// their speeds until then.
    fn lane_change_hold(
        &self,
        road: &RoadState,
        speed: f64,
        agents: &[Agent],
    ) -> Option<(f64, String)> {
        let change = self
            .lane_changes
            .iter()
            .find(|c| c.start_m >= road.station_m)?;
        let distance = change.start_m - road.station_m;
        if distance > speed * speed / (2.0 * STOP_DECELERATION) + 40.0 {
            return None;
        }
        let path = &self.roads.roads["gps"];
        let (here, _) = path.sample(road.station_m)?;
        let (ego, _) = project_polyline(0, &change.target, here).ok()?;
        let wait_s = distance / speed.max(1.0);
        let (t0, t1) = (change.target[0], change.target[1]);
        let start_heading = (t1.y - t0.y).atan2(t1.x - t0.x);
        let blocked = agents.iter().any(|agent| {
            let Ok((p, _)) = project_polyline(
                0,
                &change.target,
                Point3 {
                    x: agent.pose.x,
                    y: agent.pose.y,
                    z: agent.z,
                },
            ) else {
                return false;
            };
            // Before the target lane's first point (a vehicle coming up from
            // behind on the previous road section): extend it straight back.
            let (s, offset, heading) = if p.s < 1e-6 {
                let (dx, dy) = (agent.pose.x - t0.x, agent.pose.y - t0.y);
                let (c, n) = (start_heading.cos(), start_heading.sin());
                (dx * c + dy * n, -dx * n + dy * c, start_heading)
            } else if p.distance > p.d.abs() + 0.5 {
                return false;
            } else {
                (p.s, p.d, p.heading)
            };
            let angle = wrap_angle(agent.pose.yaw - heading);
            if offset.abs() > 2.5 || (agent.z - p.z).abs() > 2.5 || angle.abs() > 0.6 {
                return false;
            }
            let along = agent.speed * angle.cos();
            // Centre-to-rear-axle gap when the blend would start.
            let gap = s - ego.s + (along - speed) * wait_s;
            let half = agent.length / 2.0;
            let ahead_clear =
                gap - half - REAR_AXLE_TO_FRONT_M >= 5.0 + 1.0 * (speed - along).max(0.0);
            let behind_clear = -gap - half - 1.0 >= 8.0 + 2.5 * (along - speed).max(0.0)
                // A vehicle behind that is not closing in cannot block the
                // change. L3: cars queued behind the waiting truck were
                // projected onto the target lane's backward extension and
                // the two waited for each other.
                || (-gap - half - 1.0 >= 0.0 && along <= speed + 0.5);
            !(ahead_clear || behind_clear)
        });
        blocked.then(|| (distance.max(0.0), "等待换道间隙".to_owned()))
    }
    /// Observed lateral rate relative to the fixed GPS path. The first sighting
    /// and implausible jumps (identifier reuse, re-projection to another part
    /// of the window) report zero drift instead of inventing a cut-in.
    fn track(
        tracks: &mut HashMap<TrackKey, LateralTrack>,
        key: TrackKey,
        time_s: f64,
        d: f64,
    ) -> f64 {
        let fresh = LateralTrack {
            time_s,
            d,
            rate: 0.0,
            last_seen_s: time_s,
        };
        let entry = tracks.entry(key).or_insert(fresh);
        entry.last_seen_s = time_s;
        let dt = time_s - entry.time_s;
        if !(0.0..=1.0).contains(&dt) {
            *entry = fresh;
        } else if dt >= 0.1 {
            let observed = (d - entry.d) / dt;
            if observed.abs() > 5.0 {
                entry.rate = 0.0;
            } else {
                // First-order filter with a 0.4 s time constant.
                entry.rate += (observed - entry.rate) * (dt / 0.4).min(1.0);
            }
            entry.time_s = time_s;
            entry.d = d;
        }
        if entry.rate.abs() < LATERAL_RATE_DEADBAND_MPS {
            0.0
        } else {
            entry
                .rate
                .clamp(-MAX_LATERAL_RATE_MPS, MAX_LATERAL_RATE_MPS)
        }
    }
}
pub struct SpeedPlan {
    pub target: f64,
    pub binding: Binding,
    pub acceleration: f64,
    /// Inside the final approach to the end of the usable route.
    pub approach: bool,
    pub lead_gap: Option<f64>,
    pub lead: Option<LeadEvidence>,
    pub reason: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    fn guidance() -> Guidance {
        Guidance {
            roads: Roads {
                roads: HashMap::from([(
                    "gps".into(),
                    Road::path(
                        "gps".into(),
                        vec![
                            Point3 {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                            },
                            Point3 {
                                x: 1000.0,
                                y: 0.0,
                                z: 0.0,
                            },
                        ],
                    )
                    .unwrap(),
                )]),
            },
            lane_ends: vec![("lane".into(), 1000.0, 2)],
            gps: vec![1, 2],
            reached_prefix: 1,
            terminal: "end".into(),
            progress_s: 20.0,
            tracks: HashMap::new(),
            approach_cap: None,
            controls: Vec::new(),
            signal_heads: TrafficContext::default(),
            signals_seen: Vec::new(),
            stop_dwell: HashMap::new(),
            traffic_lanes: TrafficLanes::default(),
            lead_in_m: 0.0,
            lane_changes: Vec::new(),
        }
    }
    fn agent(x: f64, y: f64, speed: f64) -> Agent {
        Agent {
            id: 1,
            pose: Pose2 { x, y, yaw: 0.0 },
            z: 0.0,
            speed,
            acceleration: 0.0,
            length: 5.0,
            width: 2.0,
            lane_id: None,
            trailers: vec![],
        }
    }
    #[test]
    fn clear_road_approaches_limit_but_stopped_lead_and_cutin_brake() {
        let mut g = guidance();
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 20.0, &[1, 2]).unwrap();
        let clear = g.speed(&road, pose, 20.0, 25.0, 25.0, &[], 0.0).unwrap();
        assert!(clear.target > 20.0);
        let blocked = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[agent(50.0, 0.0, 0.0)], 0.0)
            .unwrap();
        assert_eq!(blocked.acceleration, -4.0);
        let adjacent = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[agent(50.0, 4.0, 0.0)], 0.0)
            .unwrap();
        assert!(adjacent.target > 20.0);
        // A single frame with a converging heading is not yet a cut-in; the
        // observed lateral drift over the following frames is.
        let mut cutin = agent(55.0, 4.0, 10.0);
        cutin.id = 7;
        cutin.pose.yaw = -0.1;
        let first = g
            .speed(
                &road,
                pose,
                20.0,
                25.0,
                25.0,
                std::slice::from_ref(&cutin),
                1.0,
            )
            .unwrap();
        assert!(first.lead.is_none());
        let mut braked = false;
        for i in 1..=6 {
            cutin.pose.y = 4.0 - 0.1 * i as f64;
            let plan = g
                .speed(
                    &road,
                    pose,
                    20.0,
                    25.0,
                    25.0,
                    std::slice::from_ref(&cutin),
                    1.0 + 0.1 * i as f64,
                )
                .unwrap();
            braked |= plan.acceleration < 0.0 && plan.lead.is_some_and(|l| l.agent_id == 7);
        }
        assert!(
            braked,
            "observed 1 m/s drift toward the corridor must brake"
        );
        assert!(
            g.speed(&road, pose, 20.0, 0.0, 25.0, &[], 0.0)
                .unwrap()
                .target
                > 20.0
        );
        assert!(
            g.speed(&road, pose, 20.0, f64::NAN, 25.0, &[], 0.0)
                .is_err()
        );
    }
    #[test]
    fn reroute_never_silently_chooses_nearest_road() {
        let mut g = guidance();
        assert!(
            g.observe(
                Pose2 {
                    x: 20.0,
                    y: 0.0,
                    yaw: 0.0
                },
                0.0,
                5.0,
                &[1, 3]
            )
            .is_err()
        );
        assert!(
            g.observe(
                Pose2 {
                    x: 20.0,
                    y: 0.0,
                    yaw: 0.0
                },
                0.0,
                5.0,
                &[]
            )
            .is_err()
        );
    }
    #[test]
    fn limit_drop_and_terminal_boundary_brake_and_unknown_controls_never_pass() {
        let mut g = guidance();
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 20.0, &[1, 2]).unwrap();
        let reduced = g.speed(&road, pose, 20.0, 13.9, 25.0, &[], 0.0).unwrap();
        assert!(reduced.target <= 13.6 && reduced.acceleration < -0.2);
        let mut end = road.clone();
        end.station_m = 970.0;
        end.remaining_m = 30.0;
        let stop = g
            .speed(&end, Pose2 { x: 970.0, ..pose }, 15.0, 25.0, 25.0, &[], 0.0)
            .unwrap();
        assert!(stop.target < 6.1 && stop.acceleration <= -STOP_DECELERATION);
        assert!(!simple_lane(&TrafficContext::default(), "1:road:right:0"));
    }
    #[test]
    fn gps_prefix_can_only_consume_reached_checkpoints() {
        let mut g = guidance();
        g.gps = vec![1, 2, 3];
        g.lane_ends.push(("next".into(), 1200.0, 3));
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        assert!(g.observe(pose, 0.0, 5.0, &[2, 3]).is_ok());
        assert!(g.observe(pose, 0.0, 5.0, &[3]).is_err());
    }
    fn straight(length: f64, offset: f64) -> Guidance {
        let mut g = guidance();
        g.roads.roads.insert(
            "gps".into(),
            Road::path(
                "gps".into(),
                vec![
                    Point3 {
                        x: 0.0,
                        y: offset,
                        z: 0.0,
                    },
                    Point3 {
                        x: length,
                        y: offset,
                        z: 0.0,
                    },
                ],
            )
            .unwrap(),
        );
        g
    }
    fn control(kind: ControlKind, priority: RightOfWay, station_m: f64) -> RouteControl {
        RouteControl {
            station_m,
            kind,
            priority,
            group: None,
            conservative: false,
            complete: true,
            conflicts: vec![],
            left_turn: false,
            key: format!("test:{station_m}"),
        }
    }
    /// Drive the planner closed-loop with an ideal plant; returns (x, speed).
    fn drive(g: &mut Guidance, x0: f64, v0: f64, seconds: f64, agents: &[Agent]) -> (f64, f64) {
        let dt = 1.0 / 30.0;
        let (mut x, mut v) = (x0, v0);
        g.progress_s = x0;
        // One monotone clock across calls, so stop-sign dwells accumulate.
        let start = CLOCK.with(std::cell::Cell::get);
        for i in 0..(seconds / dt) as usize {
            let pose = Pose2 {
                x,
                y: 0.0,
                yaw: 0.0,
            };
            let road = g.observe(pose, 0.0, v, &[1, 2]).unwrap();
            let plan = g
                .speed(&road, pose, v, 25.0, 15.0, agents, start + i as f64 * dt)
                .unwrap();
            let a = if plan.acceleration < 0.0 {
                plan.acceleration
            } else {
                ((plan.target - v) * 1.5).clamp(-1.0, 1.0)
            };
            v = (v + a * dt).max(0.0);
            x += v * dt;
            CLOCK.with(|c| c.set(c.get() + dt));
        }
        (x, v)
    }
    thread_local! {
        static CLOCK: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) };
    }
    #[test]
    fn red_light_stops_before_the_line_and_green_releases() {
        let mut g = guidance();
        g.signal_heads = TrafficContext {
            signals: vec![
                serde_json::from_value(serde_json::json!({
                    "id": "g", "owner_uid": 1, "semaphore_id": 1, "type": "TrafficLight",
                    "x": 300.0, "y": 4.0, "z": 0.0, "yaw": 0.0, "profile": "p",
                    "cycle_delay_s": 0.0, "interval_0": 10.0, "interval_1": 3.0,
                    "interval_2": 10.0, "interval_3": 3.0
                }))
                .unwrap(),
            ],
            ..Default::default()
        };
        let mut light = control(ControlKind::Signal, RightOfWay::SignalRequired, 300.0);
        light.group = Some("g".into());
        g.controls = vec![light];
        let head = |state| SignalObservation {
            position: Point3 {
                x: 300.0,
                y: 4.0,
                z: 0.0,
            },
            raw_state: state,
            age_us: 0,
        };
        g.observe_signals(vec![head(2)]);
        let (x, v) = drive(&mut g, 100.0, 12.0, 40.0, &[]);
        let line = 300.0 - REAR_AXLE_TO_FRONT_M - 1.0;
        assert!(v < 0.05 && x <= line + 0.3 && x > line - 8.0, "x={x} v={v}");
        g.observe_signals(vec![head(8)]);
        let (x, _) = drive(&mut g, x, 0.0, 30.0, &[]);
        assert!(x > 320.0, "green must release the stop, x={x}");
        // ETS2LA reports the 40 nearest lamps. A full list without this light
        // proves nothing: stop. Room in the list means the light is dark.
        let far = |i: usize| SignalObservation {
            position: Point3 {
                x: 5000.0 + i as f64 * 10.0,
                y: 0.0,
                z: 0.0,
            },
            raw_state: 8,
            age_us: 0,
        };
        let mut g2 = guidance();
        g2.signal_heads = g.signal_heads.clone();
        g2.controls = g.controls.clone();
        g2.observe_signals((0..40).map(far).collect());
        let (x, v) = drive(&mut g2, 100.0, 12.0, 40.0, &[]);
        assert!(v < 0.05 && x <= line + 0.3, "x={x}");
        let mut g3 = guidance();
        g3.signal_heads = g.signal_heads.clone();
        g3.controls = g.controls.clone();
        g3.observe_signals((0..4).map(far).collect());
        let (x, _) = drive(&mut g3, 100.0, 12.0, 40.0, &[]);
        assert!(x > 320.0, "a dark light is crossed as give-way, x={x}");
    }
    #[test]
    fn ungrouped_signal_follows_the_lamp_at_its_stop_line() {
        let mut g = guidance();
        g.controls = vec![control(
            ControlKind::Signal,
            RightOfWay::SignalRequired,
            300.0,
        )];
        let head = |x: f64, state| SignalObservation {
            position: Point3 { x, y: 4.0, z: 0.0 },
            raw_state: state,
            age_us: 0,
        };
        // Our lamp at the line is red; a far head (other approach) is green.
        g.observe_signals(vec![head(300.0, 2), head(330.0, 8)]);
        let (x, v) = drive(&mut g, 100.0, 12.0, 40.0, &[]);
        let line = 300.0 - REAR_AXLE_TO_FRONT_M - 1.0;
        assert!(
            v < 0.05 && x <= line + 0.3,
            "red at the line holds, x={x} v={v}"
        );
        g.observe_signals(vec![head(300.0, 8), head(330.0, 2)]);
        let (x, _) = drive(&mut g, x, 0.0, 30.0, &[]);
        assert!(x > 320.0, "green at the line releases, x={x}");
        // No lamp observed near the line: crossed as give-way.
        let mut dark = guidance();
        dark.controls = g.controls.clone();
        let (x, _) = drive(&mut dark, 100.0, 12.0, 40.0, &[]);
        assert!(x > 320.0, "dark signal is given way to, x={x}");
    }
    #[test]
    fn yield_waits_for_crossing_traffic_and_crosses_slowly_when_clear() {
        let mut g = guidance();
        let mut give_way = control(ControlKind::Yield, RightOfWay::Yield, 300.0);
        give_way.conservative = true;
        // A crossing lane running north through x=300.
        give_way.conflicts = vec![ConflictLane {
            points: vec![
                Point3 {
                    x: 300.0,
                    y: -200.0,
                    z: 0.0,
                },
                Point3 {
                    x: 300.0,
                    y: 200.0,
                    z: 0.0,
                },
            ],
            conflict_s: 200.0,
        }];
        g.controls = vec![give_way];
        let mut crossing = agent(300.0, -60.0, 12.0);
        crossing.pose.yaw = std::f64::consts::FRAC_PI_2;
        let pose = Pose2 {
            x: 270.0,
            y: 0.0,
            yaw: 0.0,
        };
        g.progress_s = 270.0;
        let road = g.observe(pose, 0.0, 6.0, &[1, 2]).unwrap();
        let blocked = g
            .speed(
                &road,
                pose,
                6.0,
                25.0,
                15.0,
                std::slice::from_ref(&crossing),
                0.0,
            )
            .unwrap();
        assert!(blocked.reason.starts_with("让行"));
        assert!(blocked.target <= (2.0 * STOP_DECELERATION * 24.0).sqrt() + 1e-6);
        // Clear crossing: approach cap only, no stop; 30 km/h at the line
        // when all cross traffic is visible, 15 km/h when it may not be.
        let clear = g.speed(&road, pose, 6.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(!clear.reason.starts_with("让行"));
        let cap = |line: f64| (line * line + 2.0 * STOP_DECELERATION * 24.0).sqrt();
        assert!((clear.target - cap(CLEAR_LINE_SPEED)).abs() < 1e-6);
        g.controls[0].complete = false;
        let unseen = g.speed(&road, pose, 6.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!((unseen.target - cap(CAUTION_LINE_SPEED)).abs() < 1e-6);
        g.controls[0].complete = true;
        let full: Vec<Agent> = (0..TRAFFIC_SLOTS)
            .map(|i| agent(5000.0 + i as f64 * 20.0, 0.0, 0.0))
            .collect();
        let crowded = g.speed(&road, pose, 6.0, 25.0, 15.0, &full, 0.0).unwrap();
        assert!((crowded.target - cap(CAUTION_LINE_SPEED)).abs() < 1e-6);
        let (x, _) = drive(&mut g, 200.0, 12.0, 30.0, &[]);
        assert!(x > 320.0, "clear yield must be crossed, x={x}");
    }
    #[test]
    fn green_light_is_crossed_at_road_speed() {
        let mut g = guidance();
        let mut light = control(ControlKind::Signal, RightOfWay::SignalRequired, 300.0);
        light.conservative = true;
        g.controls = vec![light];
        let lamp = |state| {
            vec![SignalObservation {
                position: Point3 {
                    x: 300.0,
                    y: 4.0,
                    z: 0.0,
                },
                raw_state: state,
                age_us: 0,
            }]
        };
        g.observe_signals(lamp(8));
        let pose = Pose2 {
            x: 270.0,
            y: 0.0,
            yaw: 0.0,
        };
        g.progress_s = 270.0;
        let road = g.observe(pose, 0.0, 12.0, &[1, 2]).unwrap();
        let plan = g.speed(&road, pose, 12.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(plan.target > 14.0, "{} {}", plan.target, plan.reason);
        // r12 crossed every green light at 15 km/h. A dark one still is.
        g.observe_signals(lamp(0));
        let dark = g.speed(&road, pose, 12.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(dark.target < 10.0, "{}", dark.target);
    }
    #[test]
    fn zip_merge_falls_in_behind_without_stopping() {
        let mut g = guidance();
        let mut merge = control(ControlKind::Yield, RightOfWay::ZipMerge, 300.0);
        merge.conservative = true;
        // The merging lane joins from the left, parallel at y = 4.
        merge.conflicts = vec![ConflictLane {
            points: vec![
                Point3 {
                    x: 0.0,
                    y: 4.0,
                    z: 0.0,
                },
                Point3 {
                    x: 300.0,
                    y: 4.0,
                    z: 0.0,
                },
            ],
            conflict_s: 300.0,
        }];
        g.controls = vec![merge];
        let pose = Pose2 {
            x: 200.0,
            y: 0.0,
            yaw: 0.0,
        };
        g.progress_s = 200.0;
        let road = g.observe(pose, 0.0, 14.0, &[1, 2]).unwrap();
        // Nobody on the merging lane: no slowdown at all.
        let free = g.speed(&road, pose, 14.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(free.target > 14.5, "{}", free.target);
        // A car alongside, reaching the merge first: slow to fall in behind, never
        // a stop at the merge point.
        let beside = agent(200.0, 4.0, 14.0);
        let plan = g
            .speed(&road, pose, 14.0, 25.0, 15.0, &[beside], 0.0)
            .unwrap();
        assert!(
            !plan.reason.starts_with("让行"),
            "{} {}",
            plan.target,
            plan.reason
        );
        assert!(
            plan.target < 13.5 && plan.target > 8.0,
            "{} {}",
            plan.target,
            plan.reason
        );
        // A car well behind does not slow the truck.
        let behind = agent(120.0, 4.0, 14.0);
        let plan = g
            .speed(&road, pose, 14.0, 25.0, 15.0, &[behind], 0.0)
            .unwrap();
        assert!(plan.target > 14.5, "{} {}", plan.target, plan.reason);
    }
    #[test]
    fn conflict_lanes_include_approaches_but_not_the_route() {
        let line = |x0: f64, x1: f64| LaneCenter {
            points: vec![
                Point3 {
                    x: x0,
                    y: 0.0,
                    z: 0.0,
                },
                Point3 {
                    x: x1,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            predecessors: vec![],
        };
        let mut centers = HashMap::new();
        centers.insert("junction".to_owned(), line(0.0, 30.0));
        centers.insert("a".to_owned(), line(-120.0, 0.0));
        centers.insert("b".to_owned(), line(-240.0, -120.0));
        centers.insert("c".to_owned(), line(-360.0, -240.0));
        centers.insert("ours".to_owned(), line(-50.0, 0.0));
        centers.get_mut("junction").unwrap().predecessors = vec!["a".into(), "ours".into()];
        centers.get_mut("a").unwrap().predecessors = vec!["b".into()];
        centers.get_mut("b").unwrap().predecessors = vec!["c".into()];
        let position = Point3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        };
        let own = HashSet::from(["ours"]);
        let lanes = conflict_lanes("junction", position, &centers, &own).unwrap();
        // The junction lane, then a (10 + 120 m) and b (250 m); b reaches
        // past 200 m so c is not walked.
        let stations: Vec<f64> = lanes.iter().map(|l| l.conflict_s).collect();
        assert_eq!(stations.len(), 3, "{stations:?}");
        assert!((stations[0] - 10.0).abs() < 1e-6);
        assert!((stations[1] - 130.0).abs() < 1e-6);
        assert!((stations[2] - 250.0).abs() < 1e-6);
        assert!(conflict_lanes("missing", position, &centers, &own).is_none());
    }
    #[test]
    fn stop_sign_requires_a_full_stop_before_proceeding() {
        let mut g = guidance();
        g.controls = vec![control(ControlKind::Stop, RightOfWay::Yield, 300.0)];
        let line = 300.0 - REAR_AXLE_TO_FRONT_M - 1.0;
        let (mut x, mut v) = (150.0, 12.0);
        let mut stopped_before_line = false;
        for step in 0..60 {
            (x, v) = drive(&mut g, x, v, 0.5, &[]);
            assert!(
                stopped_before_line || x <= line + 0.3,
                "crossed the line without a full stop at step {step}: x={x}"
            );
            stopped_before_line |= v < 0.05 && x > line - 1.5;
        }
        assert!(stopped_before_line && x > 320.0, "x={x}");
    }
    #[test]
    fn rolling_route_is_adopted_only_where_it_coincides_and_reaches_further() {
        let pose = Pose2 {
            x: 300.0,
            y: 0.0,
            yaw: 0.0,
        };
        let mut g = straight(500.0, 0.0);
        g.tracks.insert(
            (1, 0, 0),
            LateralTrack {
                time_s: 0.0,
                d: 4.0,
                rate: 0.0,
                last_seen_s: 0.0,
            },
        );
        assert!(
            g.adopt(straight(510.0, 0.0), pose, 0.0, 10.0)
                .unwrap()
                .is_none()
        );
        assert!(g.adopt(straight(1500.0, 2.0), pose, 0.0, 10.0).is_err());
        let extension = g.adopt(straight(1500.0, 0.0), pose, 0.0, 10.0).unwrap();
        assert!((extension.unwrap() - 1000.0).abs() < 1e-6);
        assert_eq!(g.roads.roads["gps"].length(), 1500.0);
        assert_eq!(g.tracks.len(), 1, "lateral tracks survive the swap");
        assert!((g.progress_s - 300.0).abs() < 1e-6);
    }
    #[test]
    fn terminal_approach_never_reaccelerates_and_stops_before_the_end() {
        let mut g = guidance();
        g.progress_s = 850.0;
        let mut pose = Pose2 {
            x: 850.0,
            y: 0.0,
            yaw: 0.0,
        };
        // Start at the cruise speed with a lag-free but over-braking plant:
        // the truck decelerates 30 % harder than planned, as in r9.
        let mut speed = 30.0 / 3.6;
        let mut previous_target = f64::INFINITY;
        let dt = 1.0 / 60.0;
        for step in 0..60 * 60 {
            let road = g.observe(pose, 0.0, speed, &[1, 2]).unwrap();
            let plan = g
                .speed(&road, pose, speed, 25.0, 30.0 / 3.6, &[], step as f64 * dt)
                .unwrap();
            if g.approach_cap.is_some() {
                // Only the creep floor (≤ 1.5 m/s) may lift a target.
                assert!(
                    plan.target <= previous_target.max(1.5) + 1e-6,
                    "target rose from {previous_target} to {} at x={}",
                    plan.target,
                    pose.x
                );
                previous_target = plan.target;
            }
            let accel = if plan.acceleration < 0.0 {
                plan.acceleration * 1.3
            } else {
                ((plan.target - speed) * 0.8).min(plan.acceleration)
            };
            speed = (speed + accel * dt).max(0.0);
            pose.x += speed * dt;
            if speed == 0.0 {
                break;
            }
        }
        assert!(speed < 0.05, "never stopped, speed={speed}");
        assert!(
            pose.x <= 1000.0 - 18.0 + 0.5,
            "overran the stop point: x={}",
            pose.x
        );
    }
    #[test]
    fn stopped_leader_closed_loop_stops_with_clearance() {
        let mut g = guidance();
        let lead = agent(160.0, 0.0, 0.0);
        let mut pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let mut speed = 25.0;
        for _ in 0..1500 {
            let road = g.observe(pose, 0.0, speed, &[1, 2]).unwrap();
            let plan = g
                .speed(
                    &road,
                    pose,
                    speed,
                    25.0,
                    25.0,
                    std::slice::from_ref(&lead),
                    0.0,
                )
                .unwrap();
            speed = (speed + plan.acceleration * 0.02).max(0.0);
            pose.x += speed * 0.02;
            assert!(pose.x + 5.0 < 157.5, "synthetic collision at x={}", pose.x);
        }
        assert!(speed < 0.15 && pose.x < 148.0, "x={} speed={speed}", pose.x);
    }
    #[test]
    fn hill_leader_and_body_overlap_remain_collision_constraints() {
        let mut g = guidance();
        g.roads.roads.insert(
            "gps".into(),
            Road::path(
                "gps".into(),
                vec![
                    Point3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    Point3 {
                        x: 1000.0,
                        y: 0.0,
                        z: 100.0,
                    },
                ],
            )
            .unwrap(),
        );
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 2.0, 20.0, &[1, 2]).unwrap();
        let mut hill = agent(70.0, 0.0, 0.0);
        hill.z = 7.0;
        assert!(
            g.speed(&road, pose, 20.0, 25.0, 25.0, &[hill], 0.0)
                .unwrap()
                .acceleration
                < -1.0
        );
        let mut overlap = agent(20.0, 0.0, 0.0);
        overlap.z = 2.0;
        assert_eq!(
            g.speed(&road, pose, 5.0, 25.0, 25.0, &[overlap], 0.0)
                .unwrap()
                .acceleration,
            -4.0
        );
        let mut bridge = agent(70.0, 0.0, 0.0);
        bridge.z = 17.0;
        assert!(
            g.speed(&road, pose, 20.0, 25.0, 25.0, &[bridge], 0.0)
                .unwrap()
                .acceleration
                > 0.0
        );
    }
    /// Left-hand bend of radius 120 m. Straight-line extrapolation of a body
    /// that follows the inner lane leaves the bend tangentially and crosses
    /// the ego lane within three seconds; that was a phantom-leader source.
    fn bend() -> (Guidance, f64) {
        const RADIUS: f64 = 120.0;
        let points = (0..=120)
            .map(|i| {
                let a = i as f64 * 0.01;
                Point3 {
                    x: RADIUS * a.sin(),
                    y: RADIUS * (1.0 - a.cos()),
                    z: 0.0,
                }
            })
            .collect();
        let mut g = guidance();
        g.roads
            .roads
            .insert("gps".into(), Road::path("gps".into(), points).unwrap());
        g.lane_ends = vec![("lane".into(), RADIUS * 1.2, 2)];
        g.progress_s = 0.0;
        (g, RADIUS)
    }
    #[test]
    fn a_widening_road_joint_is_blended_gently() {
        // r13: the next lane starts 1.5 m aside where a road widens; the
        // 10 m blend made a 0.09 /m S-bend and the truck braked to 25 km/h.
        let mut points: Vec<Point3> = (0..=200)
            .map(|i| Point3 {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            })
            .collect();
        blend_joint(
            &mut points,
            Point3 {
                x: 200.5,
                y: 1.5,
                z: 0.0,
            },
        );
        points.extend((1..100).map(|i| Point3 {
            x: 200.5 + i as f64,
            y: 1.5,
            z: 0.0,
        }));
        let road = Road::path("gps".into(), points).unwrap();
        let peak = (0..280)
            .map(|s| road.chord_curvature(s as f64, CURVE_CHORD_HALF_M).abs())
            .fold(0.0, f64::max);
        assert!(peak < 0.004, "peak curvature {peak}");
    }
    #[test]
    fn a_turn_near_steering_lock_is_taken_at_walking_pace() {
        // r13: a 5 m radius yard exit at 9 km/h ran out of calibrated wheel.
        let corner = |r: f64| (tight_lateral(1.0 / r) * r).sqrt() * 3.6;
        assert!(corner(5.0) < 7.0, "{}", corner(5.0));
        assert!((tight_lateral(1.0 / 12.0) - CURVE_LATERAL_TIGHT).abs() < 1e-9);
        assert!(corner(20.0) > 19.0);
    }
    #[test]
    fn tight_bend_uses_1_5_m_s2_and_a_curvature_reversal_is_slower() {
        // 60 m straight, 40 m left at R 24 (a roundabout), 40 m right at
        // R 30 (its exit), 100 m straight: the L1 drift geometry.
        let mut points = Vec::new();
        let (mut x, mut y, mut heading) = (0.0_f64, 0.0_f64, 0.0_f64);
        for i in 0..240 {
            points.push(Point3 { x, y, z: 0.0 });
            let s = i as f64;
            let curvature = if (60.0..100.0).contains(&s) {
                1.0 / 24.0
            } else if (100.0..140.0).contains(&s) {
                -1.0 / 30.0
            } else {
                0.0
            };
            heading += curvature;
            x += heading.cos();
            y += heading.sin();
        }
        let mut g = guidance();
        g.roads.roads.insert(
            "gps".into(),
            Road::path("gps".into(), points.clone()).unwrap(),
        );
        g.lane_ends = vec![("lane".into(), 239.0, 2)];
        let target_at = |g: &mut Guidance, s: usize| {
            let a = points[s];
            let b = points[s + 1];
            let pose = Pose2 {
                x: a.x,
                y: a.y,
                yaw: (b.y - a.y).atan2(b.x - a.x),
            };
            g.progress_s = s as f64 - 1.0;
            let road = g.observe(pose, 0.0, 3.0, &[1, 2]).unwrap();
            g.speed(&road, pose, 3.0, 25.0, 25.0, &[], 0.0).unwrap()
        };
        let steady = target_at(&mut g, 70);
        let corner = (1.5_f64 * 24.0).sqrt();
        assert_eq!(steady.binding, Binding::Curve);
        assert!(
            steady.target > corner - 0.4,
            "steady bend {:.1} km/h",
            steady.target * 3.6
        );
        let reversal = target_at(&mut g, 96);
        assert!(
            reversal.target < steady.target - 0.5,
            "reversal {:.1} vs steady {:.1} km/h",
            reversal.target * 3.6,
            steady.target * 3.6
        );
    }
    #[test]
    fn a_route_crossing_itself_keeps_the_pass_the_truck_is_on() {
        // East along y = 0, around a loop, then south across x = 50 at right
        // angles: the L22 service-area route.
        let corners: [(f64, f64); 5] = [
            (0.0, 0.0),
            (100.0, 0.0),
            (100.0, 20.0),
            (50.0, 20.0),
            (50.0, -40.0),
        ];
        let mut points = Vec::new();
        for pair in corners.windows(2) {
            let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
            let n = ((x1 - x0).hypot(y1 - y0) as usize).max(1);
            for i in 0..n {
                let t = i as f64 / n as f64;
                points.push(Point3 {
                    x: x0 + t * (x1 - x0),
                    y: y0 + t * (y1 - y0),
                    z: 0.0,
                });
            }
        }
        points.push(Point3 {
            x: 50.0,
            y: -40.0,
            z: 0.0,
        });
        let mut g = guidance();
        g.roads
            .roads
            .insert("gps".into(), Road::path("gps".into(), points).unwrap());
        g.lane_ends = vec![("lane".into(), 250.0, 2)];
        g.progress_s = 49.0;
        let pose = Pose2 {
            x: 50.3,
            y: 0.1,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 8.0, &[1, 2]).unwrap();
        assert!((road.station_m - 50.3).abs() < 0.5, "{}", road.station_m);
    }
    fn on_bend(radius: f64, angle: f64, offset_left: f64, yaw: f64, speed: f64) -> Agent {
        let r = radius - offset_left;
        let mut a = agent(r * angle.sin(), radius - r * angle.cos(), speed);
        a.pose.yaw = yaw;
        a
    }
    #[test]
    fn moving_traffic_is_predicted_along_its_lane_not_straight_ahead() {
        // A side road 20 m north of us bends from heading −120° onto a road
        // parallel to ours: the r10 phantom crossings.
        let mut lane = Vec::new();
        let (mut x, mut y, mut heading) = (60.0_f64, 20.0_f64, -2.094_f64);
        for i in 0..120 {
            lane.push(Point3 { x, y, z: 0.0 });
            if i < 13 {
                heading -= 1.0 / 12.0;
            }
            x += heading.cos();
            y += heading.sin();
        }
        let mut g = guidance();
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 15.0, &[1, 2]).unwrap();
        let mut car = agent(60.0, 20.0, 12.0);
        car.pose.yaw = -2.094;
        let straight = g
            .speed(
                &road,
                pose,
                15.0,
                25.0,
                25.0,
                std::slice::from_ref(&car),
                0.0,
            )
            .unwrap();
        assert!(
            straight.lead.is_some(),
            "without a lane it is extrapolated across us"
        );
        g.traffic_lanes = TrafficLanes {
            lanes: vec![Road::path("side".into(), lane).unwrap()],
        };
        let following = g
            .speed(
                &road,
                pose,
                15.0,
                25.0,
                25.0,
                std::slice::from_ref(&car),
                0.0,
            )
            .unwrap();
        assert!(following.lead.is_none(), "{:?}", following.lead);
    }
    #[test]
    fn lane_change_waits_for_a_gap_and_signals_its_direction() {
        let mut g = guidance();
        g.lane_changes = vec![LaneChange {
            start_m: 100.0,
            end_m: 200.0,
            left: true,
            target: vec![
                Point3 {
                    x: 0.0,
                    y: 3.5,
                    z: 0.0,
                },
                Point3 {
                    x: 1000.0,
                    y: 3.5,
                    z: 0.0,
                },
            ],
        }];
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 20.0, &[1, 2]).unwrap();
        assert_eq!(g.blinker(road.station_m, 20.0), Some(true));
        // Far before the change: no signal yet.
        assert_eq!(g.blinker(road.station_m - 0.0, 2.0), None);
        // Alongside in the target lane at the same speed: wait.
        let beside = agent(25.0, 3.5, 20.0);
        let plan = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[beside], 0.0)
            .unwrap();
        assert_eq!(plan.reason, "等待换道间隙");
        // Far ahead at the same speed: go.
        let ahead = agent(220.0, 3.5, 20.0);
        let plan = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[ahead], 0.0)
            .unwrap();
        assert_ne!(plan.reason, "等待换道间隙");
        // Behind and closing at 5 m/s: wait for it to pass.
        let closing = agent(-20.0, 3.5, 25.0);
        let plan = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[closing], 0.0)
            .unwrap();
        assert_eq!(plan.reason, "等待换道间隙");
        // In our own lane, not the target: no effect on the change.
        let own = agent(400.0, 0.0, 20.0);
        let plan = g.speed(&road, pose, 20.0, 25.0, 25.0, &[own], 0.0).unwrap();
        assert_ne!(plan.reason, "等待换道间隙");
        // Waiting at the start point, a queue that formed behind (L3) does
        // not close in and must not hold the change forever.
        let queued = agent(5.0, 3.5, 0.0);
        let plan = g
            .speed(&road, pose, 0.0, 25.0, 25.0, &[queued], 0.0)
            .unwrap();
        assert_ne!(plan.reason, "等待换道间隙");
    }
    #[test]
    fn crosswise_car_beside_a_sharp_turn_is_not_an_obstacle_but_one_on_it_is() {
        // Straight to x=50, a 15 m-radius left turn, then north along x=65.
        let mut points = vec![Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }];
        for i in 0..=18 {
            let a = i as f64 * std::f64::consts::FRAC_PI_2 / 18.0;
            points.push(Point3 {
                x: 50.0 + 15.0 * a.sin(),
                y: 15.0 - 15.0 * a.cos(),
                z: 0.0,
            });
        }
        points.push(Point3 {
            x: 65.0,
            y: 200.0,
            z: 0.0,
        });
        let mut g = guidance();
        g.roads
            .roads
            .insert("gps".into(), Road::path("gps".into(), points).unwrap());
        g.lane_ends = vec![("lane".into(), 300.0, 2)];
        g.progress_s = 40.0;
        let pose = Pose2 {
            x: 40.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 3.0, &[1, 2]).unwrap();
        // Waiting on the crossing road, 6 m short of our turn: the r9 deadlock.
        let mut waiting = agent(62.0, -6.0, 0.0);
        waiting.pose.yaw = std::f64::consts::FRAC_PI_2;
        let plan = g
            .speed(
                &road,
                pose,
                3.0,
                25.0,
                15.0,
                std::slice::from_ref(&waiting),
                0.0,
            )
            .unwrap();
        assert!(plan.lead.is_none(), "{:?}", plan.lead);
        // The same car standing across our exit lane blocks it.
        let mut blocking = agent(65.0, 25.0, 0.0);
        blocking.pose.yaw = 0.0;
        let plan = g
            .speed(
                &road,
                pose,
                3.0,
                25.0,
                15.0,
                std::slice::from_ref(&blocking),
                0.0,
            )
            .unwrap();
        let lead = plan.lead.expect("crosswise car on the exit lane");
        // Path from x=40: 10 m straight, 23.6 m of arc, then 10 m north to the
        // car's near side (y=24), less the 5 m rear-axle-to-bumper offset.
        let expected = 10.0 + 15.0 * std::f64::consts::FRAC_PI_2 + 9.0 - REAR_AXLE_TO_FRONT_M;
        assert!(
            (lead.gap_m - expected).abs() < 0.8,
            "gap {} vs {expected}",
            lead.gap_m
        );
    }
    #[test]
    fn bend_followers_opposing_and_kerb_parked_vehicles_are_not_leaders() {
        let (mut g, radius) = bend();
        let pose = Pose2 {
            x: radius * 0.1_f64.sin(),
            y: radius * (1.0 - 0.1_f64.cos()),
            yaw: 0.1,
        };
        let road = g.observe(pose, 0.0, 10.0, &[1, 2]).unwrap();
        let mut follower = on_bend(radius, 0.35, 4.5, 0.35, 12.0);
        follower.id = 1;
        let mut opposing = on_bend(radius, 0.45, 4.5, 0.45 + std::f64::consts::PI, 12.0);
        opposing.id = 2;
        let mut parked = on_bend(radius, 0.4, -3.0, 0.4, 0.0);
        parked.id = 3;
        let mut parked_bus = on_bend(radius, 0.6, -3.4, 0.6, 0.0);
        (parked_bus.id, parked_bus.length, parked_bus.width) = (4, 12.1, 2.75);
        for (i, t) in [0.0, 0.2, 0.4, 0.6].into_iter().enumerate() {
            let plan = g
                .speed(
                    &road,
                    pose,
                    10.0,
                    25.0,
                    25.0,
                    &[
                        follower.clone(),
                        opposing.clone(),
                        parked.clone(),
                        parked_bus.clone(),
                    ],
                    t,
                )
                .unwrap();
            assert!(plan.lead.is_none(), "step {i}: phantom {:?}", plan.lead);
        }
        let mut stopped = on_bend(radius, 0.5, 0.3, 0.5, 0.0);
        stopped.id = 5;
        let plan = g
            .speed(
                &road,
                pose,
                10.0,
                25.0,
                25.0,
                std::slice::from_ref(&stopped),
                0.8,
            )
            .unwrap();
        let lead = plan.lead.expect("in-lane stopped vehicle on the bend");
        assert_eq!((lead.agent_id, lead.motion), (5, Motion::SameDirection));
        assert!((lead.gap_m - (radius * 0.4 - 2.5 - REAR_AXLE_TO_FRONT_M)).abs() < 0.5);
        assert!(plan.acceleration < -1.0);
    }
}
