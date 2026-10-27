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
    /// Simulation: every lane within 2 km of `center` and its traffic
    /// controls (for AI traffic).
    pub fn region_lanes(
        &self,
        center: Point3,
    ) -> anyhow::Result<(Vec<etsads_map::importer::ExportLane>, TrafficContext)> {
        let region = self.store.load_2km(center)?;
        let traffic = self
            .annotations
            .load_for_region(center, 2000.0, &region)?
            .traffic;
        Ok((region.map.lanes, traffic))
    }
    /// Simulation: the nearest lane centre heading the truck's way, as a
    /// rear-axle pose and height (a driver putting the truck back on its lane).
    pub fn lane_centre(&self, pose: Pose2, z: f64) -> anyhow::Result<(Pose2, f64)> {
        let center = Point3 {
            x: pose.x,
            y: pose.y,
            z,
        };
        let region = self.store.load_2km(center)?;
        region
            .map
            .lanes
            .iter()
            .filter(|l| l.truck_access != Some(false))
            .filter_map(|l| {
                let (p, point) = project_polyline(0, &l.center, center).ok()?;
                (p.distance < 8.0
                    && (p.z - z).abs() < 3.0
                    && wrap_angle(p.heading - pose.yaw).abs() < 0.8)
                    .then_some((p, point))
            })
            .min_by(|a, b| a.0.distance.total_cmp(&b.0.distance))
            .map(|(p, point)| {
                (
                    Pose2 {
                        x: point.x,
                        y: point.y,
                        yaw: p.heading,
                    },
                    p.z,
                )
            })
            .context("附近没有同向车道")
    }
    /// Simulation: where a driver who took over rejoins the route. From the
    /// lane the truck is on, lanes are followed (changing to a neighbour where
    /// the map allows) to the end of the nearest one that reaches GPS
    /// checkpoint `from + 1` or later; the truck is placed a few metres into
    /// the lane continuing the route, centred and stopped. Returns the
    /// rear-axle pose, its height, the checkpoint reached and the distance
    /// driven.
    pub fn drive_through(
        &self,
        pose: Pose2,
        z: f64,
        gps: &[u64],
        from: usize,
    ) -> anyhow::Result<(Pose2, f64, usize, f64)> {
        use std::collections::{BinaryHeap, HashMap as Map};
        let center = Point3 {
            x: pose.x,
            y: pose.y,
            z,
        };
        let region = self.store.load_2km(center)?;
        let lanes: Map<&str, &etsads_map::importer::ExportLane> = region
            .map
            .lanes
            .iter()
            .filter(|l| l.truck_access != Some(false) && l.center.len() >= 2)
            .map(|l| (l.id.as_str(), l))
            .collect();
        let length = |l: &etsads_map::importer::ExportLane| {
            l.center
                .windows(2)
                .map(|p| p[0].distance_xy(p[1]))
                .sum::<f64>()
        };
        // The lane under the truck, heading the same way.
        let (start, start_s) = lanes
            .values()
            .filter_map(|l| {
                let (p, _) = project_polyline(0, &l.center, center).ok()?;
                (p.distance < 6.0
                    && (p.z - z).abs() < 3.0
                    && wrap_angle(p.heading - pose.yaw).abs() < 0.8)
                    .then_some((l.id.as_str(), p.s, p.distance))
            })
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .map(|(id, s, _)| (id, s))
            .context("附近没有可接续的车道")?;
        let ahead = |node: u64| gps[from + 1..].iter().take(12).position(|g| *g == node);
        // Dijkstra over successors (and permitted lane changes, 30 m extra).
        #[derive(PartialEq)]
        struct Open(f64, String);
        impl Eq for Open {}
        impl PartialOrd for Open {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }
        impl Ord for Open {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                other.0.total_cmp(&self.0)
            }
        }
        let mut cost: Map<String, f64> = Map::new();
        let mut open = BinaryHeap::new();
        cost.insert(start.to_owned(), -start_s);
        open.push(Open(-start_s, start.to_owned()));
        let mut found = None;
        while let Some(Open(c, id)) = open.pop() {
            if c > cost[&id] || c > 6000.0 {
                continue;
            }
            let lane = lanes[id.as_str()];
            let end = c + length(lane);
            if let Some(k) = ahead(lane.node_end) {
                found = Some((id.clone(), from + 1 + k, end));
                break;
            }
            let changes = [
                lane.left.as_ref().filter(|_| lane.can_change_left),
                lane.right.as_ref().filter(|_| lane.can_change_right),
            ];
            let next = lane
                .successors
                .iter()
                .map(|s| (s, end))
                .chain(changes.into_iter().flatten().map(|s| (s, c + 30.0)));
            for (next, at) in next {
                if lanes.contains_key(next.as_str()) && at < *cost.get(next).unwrap_or(&f64::MAX) {
                    cost.insert(next.clone(), at);
                    open.push(Open(at, next.clone()));
                }
            }
        }
        let (id, checkpoint, driven) = found.context("驾驶员也找不到通往下一个导航点的车道")?;
        let lane = lanes[id.as_str()];
        // Into the lane that continues the route, else just before the end.
        let follow = gps.get(checkpoint + 1).copied();
        let continuing = lane
            .successors
            .iter()
            .filter_map(|s| lanes.get(s.as_str()))
            .max_by_key(|l| (Some(l.node_end) == follow, length(l) > 8.0));
        let (points, at) = match continuing {
            Some(l) if length(l) > 8.0 => (&l.center, 5.0),
            _ => (&lane.center, (length(lane) - 5.0).max(0.0)),
        };
        let road = super::core::Road::path("rejoin".into(), points.clone())?;
        let (p, heading) = road.sample(at).context("接续点无法采样")?;
        Ok((
            Pose2 {
                x: p.x,
                y: p.y,
                yaw: heading,
            },
            p.z,
            checkpoint,
            driven.max(0.0),
        ))
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
        let traffic_lanes = TrafficLanes::new(&region.map.lanes, &traffic);
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
            // The GPS nodes either side of the first link the map lacks.
            let mut truncated = None::<[u64; 2]>;
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
                    let mut broken = None;
                    for k in start + 2..gps.len() {
                        match prefix(k) {
                            Ok(found) => best = Some(found),
                            Err(_) => {
                                broken = Some([gps[k - 2], gps[k - 1]]);
                                break;
                            }
                        }
                    }
                    if let Some(found) = best {
                        from = start;
                        attempt = Ok(found);
                        truncated = broken.or(Some([0, 0]));
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
        let mut terminal = if truncated.is_some() {
            "前方地图路段无法接续，请准备接管"
        } else if resolved.beyond_map_node.is_some() {
            "导航路线前方进入地图外路段（多为渡轮），到达后请手动接管"
        } else if resolved.must_stop_at_local_terminal {
            "本地导航范围结束，请手动接管"
        } else {
            "即将到达目的地，请准备接管"
        }
        .to_owned();
        let mut controls = Vec::<RouteControl>::new();
        let mut lane_changes = Vec::<LaneChange>::new();
        let mut corners = Vec::<(f64, f64)>::new();
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
            let mut join = if changing {
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
                // Road heights are exported at one nominal level per road: a
                // lane joined 2 cm from its junction stood 0.63 m above it and
                // ended the route there, repeatedly (live). Lanes the map links
                // end to end are on one level; a step is only judged where the
                // joint is a visible gap.
                let max_rise = if last.distance_xy(first) <= TIGHT_JOINT_M {
                    TIGHT_JOINT_STEP_M
                } else {
                    0.5
                };
                if last.distance_xy(first) > MAX_JOINT_GAP_M || (last.z - first.z).abs() > max_rise
                {
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
                let turn = wrap_angle(next_heading - previous_heading);
                if turn.abs() > 0.35 {
                    // A moderate corner between lanes is rounded off as a
                    // driver does, one right after another together with it
                    // (junctions often join a short connector lane at an
                    // angle at each end); a sharper one, or one where a lane
                    // change begins, is left to the driver.
                    let before: f64 = points.windows(2).map(|p| p[0].distance_xy(p[1])).sum();
                    let previous = corners
                        .last()
                        .copied()
                        .filter(|(_, end)| before - end <= FILLET_CUT_M + 2.0);
                    let rounded =
                        if change_start.is_none_or(|(start, _)| start > FILLET_CUT_M + 5.0) {
                            fillet_joint(
                                &mut points,
                                previous_heading,
                                turn,
                                road,
                                previous.map(|(start, _)| before - start),
                            )
                        } else {
                            Err("正在换道")
                        };
                    match rounded {
                        Ok((cut, back)) => {
                            join = cut;
                            let after: f64 =
                                points.windows(2).map(|p| p[0].distance_xy(p[1])).sum();
                            if previous.is_some() {
                                corners.pop();
                            }
                            corners.push((before - back, after));
                        }
                        Err(why) => {
                            terminal = format!(
                                "前方转弯需要手动接管（车道折角 {:.0}°，{why}）",
                                turn.to_degrees().abs()
                            );
                            break;
                        }
                    }
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
        let path = Road::path("gps".into(), ease_curve_ends(&points))?;
        let roads = Roads {
            feed_forward: super::core::ABSOLUTE_FEED_FORWARD,
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
            route_break: truncated,
            progress_s: observed.station_m,
            tracks: HashMap::new(),
            approach_cap: None,
            controls,
            signal_heads,
            signals_seen: Vec::new(),
            stop_dwell: HashMap::new(),
            yield_wait: HashMap::new(),
            traffic_lanes,
            lead_in_m,
            lane_changes,
            corners,
            wheel_rate_cap: None,
            combination: Default::default(),
            corner_cuts: Vec::new(),
            trailer_offsets: Vec::new(),
            trailer_corner_cut: None,
            nudge: 0.0,
            nudge_at: None,
            yielding: None,
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
/// Map curves often start and end abruptly. With relative steering the
/// wheel turns slowly, so the speed planner slowed for each change of
/// curvature to what the wheel could follow. Shifting the line by at most
/// this much within the lane spreads those changes out: the path relaxes
/// toward the curvature averaged over ±`EASE_WINDOW_M`, so a steady curve
/// keeps its line and only its ends move (simulation, 90 km/h cap: 34.6 →
/// 41.0 km/h on r17 and seven random routes, the same 0.58 m peak offset,
/// no drifts; moving whole curves inward as well was no faster and put a
/// trailer further inside).
const EASE_MAX_M: f64 = 0.25;
const EASE_WINDOW_M: usize = 12;
const EASE_ITERATIONS: usize = 200;
/// Turns tighter than the first radius (junctions, hairpins) are not eased
/// over the long window, fading in up to the second: their speed is the
/// corner's own, and a clamped shift there kinked the path (1.6 m drifts at
/// 7 km/h in junction turns). From 20 m it left every roundabout alone
/// (rings of 14.5-22 m, and the flares within 15 m of them), whose entry
/// and exit reversals were crept through at 5 km/h; from 12 m the 44
/// roundabout routes took 7% less time, free or with 15 cars, with no
/// drift on the general routes (simulation).
const EASE_TIGHT_RADIUS_M: (f64, f64) = (12.0, 30.0);
/// Tight turns are eased toward their curvature averaged over only
/// ±`EASE_TIGHT_WINDOW_M`, by up to `EASE_TIGHT_MAX_M`: the wheel starts
/// turning a few metres before the corner, as a driver's does, and a 10 m
/// junction turn moves about 0.3 m inside, within the bound, so the shift is
/// never clamped into a kink. Kept on the map's line, junction turns were
/// crept through at 5 km/h and the truck still ran 0.6 m wide (live). The
/// random routes took 2–4% less time with 28% less of it below 10 km/h,
/// and the roundabout routes 5% less, with no drift (simulation).
const EASE_TIGHT_WINDOW_M: usize = 5;
const EASE_TIGHT_MAX_M: f64 = 0.5;
/// The path resampled every metre and eased (see `EASE_MAX_M`); the first
/// and last 20 m keep their line.
fn ease_curve_ends(points: &[Point3]) -> Vec<Point3> {
    let Ok(road) = Road::path("ease".into(), points.to_vec()) else {
        return points.to_vec();
    };
    let length = road.length();
    let whole = length.floor() as usize;
    let samples: Vec<(f64, Point3, f64)> = (0..=whole)
        .map(|i| i as f64)
        .chain((length - whole as f64 > MIN_PATH_STEP_M).then_some(length))
        .filter_map(|s| road.sample(s).map(|(p, h)| (s, p, h)))
        .collect();
    if samples.len() < 3 {
        return points.to_vec();
    }
    let normal = |h: f64| (-h.sin(), h.cos());
    let curvature: Vec<f64> = samples
        .iter()
        .map(|&(s, _, _)| road.chord_curvature(s, 3.0))
        .collect();
    let window = |values: &[f64], i: usize, half: usize| -> Vec<f64> {
        values[i.saturating_sub(half)..(i + half + 1).min(values.len())].to_vec()
    };
    let averaged: Vec<f64> = (0..samples.len())
        .map(|i| {
            let w = window(&curvature, i, EASE_WINDOW_M);
            w.iter().sum::<f64>() / w.len() as f64
        })
        .collect();
    // Allowed shift: none in tight turns or near the ends, eroded over
    // ±15 m and averaged over ±10 m so it never steps.
    let (tight, wide) = EASE_TIGHT_RADIUS_M;
    let open: Vec<f64> = samples
        .iter()
        .map(|&(s, _, _)| {
            let k = road.chord_curvature(s, 5.0).abs();
            let ends = (s / 20.0).min((length - s) / 20.0).clamp(0.0, 1.0);
            ((1.0 / tight - k) / (1.0 / tight - 1.0 / wide)).clamp(0.0, 1.0) * ends
        })
        .collect();
    let eroded: Vec<f64> = (0..open.len())
        .map(|i| window(&open, i, 15).into_iter().fold(1.0, f64::min))
        .collect();
    // Smoothed openness: 1 on open roads, 0 in tight turns.
    let openness: Vec<f64> = (0..eroded.len())
        .map(|i| {
            let w = window(&eroded, i, 10);
            w.iter().sum::<f64>() / w.len() as f64
        })
        .collect();
    let ends: Vec<f64> = samples
        .iter()
        .map(|&(s, _, _)| (s / 20.0).min((length - s) / 20.0).clamp(0.0, 1.0))
        .collect();
    let limit: Vec<f64> = (0..eroded.len())
        .map(|i| {
            let w = window(&ends, i, 10);
            let ends = w.iter().sum::<f64>() / w.len() as f64;
            EASE_MAX_M * openness[i] + EASE_TIGHT_MAX_M * (1.0 - openness[i]) * ends
        })
        .collect();
    let averaged: Vec<f64> = (0..samples.len())
        .map(|i| {
            let w = window(&curvature, i, EASE_TIGHT_WINDOW_M);
            let tight = w.iter().sum::<f64>() / w.len() as f64;
            openness[i] * averaged[i] + (1.0 - openness[i]) * tight
        })
        .collect();
    // Jacobi relaxation of the sideways shift: each point moves halfway to
    // where the averaged curvature would put it between its neighbours.
    let mut shift = vec![0.0_f64; samples.len()];
    for _ in 0..EASE_ITERATIONS {
        let previous = shift.clone();
        for i in 1..samples.len() - 1 {
            let at = |j: usize| {
                let (_, p, h) = samples[j];
                let n = normal(h);
                (p.x + n.0 * previous[j], p.y + n.1 * previous[j])
            };
            let (a, b, c) = (at(i - 1), at(i), at(i + 1));
            let n = normal(samples[i].2);
            let toward = ((a.0 + c.0) / 2.0 - b.0) * n.0 + ((a.1 + c.1) / 2.0 - b.1) * n.1
                - averaged[i] / 2.0;
            shift[i] = (previous[i] + 0.5 * toward).clamp(-limit[i], limit[i]);
        }
    }
    let mut eased: Vec<Point3> = samples
        .iter()
        .zip(&shift)
        .map(|(&(_, p, h), d)| {
            let n = normal(h);
            Point3 {
                x: p.x + n.0 * d,
                y: p.y + n.1 * d,
                z: p.z,
            }
        })
        .collect();
    eased.dedup_by(|b, a| a.distance_xy(*b) <= MIN_PATH_STEP_M);
    eased
}
/// Largest lane joint gap the path bridges (m, the exporter links up to 2 m),
/// the longest blend of a visible offset, and the peak curvature (1/m) a
/// smoothstep blend may add: 6 · gap / length².
const MAX_JOINT_GAP_M: f64 = 2.0;
/// A joint this tight (m) may step by up to `TIGHT_JOINT_STEP_M` in height.
const TIGHT_JOINT_M: f64 = 0.2;
const TIGHT_JOINT_STEP_M: f64 = 1.5;
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
/// Corners between lanes up to this angle (rad) are rounded off with a
/// curve whose ends lie `FILLET_CUT_M` along the path from the corner, of
/// mean curvature up to `FILLET_MAX_CURVATURE` (1/m), and driven at
/// `CORNER_SPEED` (m/s), reached `CORNER_APPROACH_M` before the corner
/// slowing at `CORNER_DECELERATION` (m/s²).
const FILLET_MAX_TURN_RAD: f64 = 0.6;
/// Corners close together are rounded off as one turn up to this (rad).
const FILLET_MAX_MERGED_RAD: f64 = 1.8;
const FILLET_CUT_M: f64 = 4.0;
const FILLET_MAX_CURVATURE: f64 = 0.25;
/// Peak curvature (1/m) a rounded-off corner may reach, checked with
/// `FILLET_CHECK_M` of each lane either side over 3 m chords. Most corners
/// peak at 0.15–0.28; at 0.32 the wheel fell behind the reversal out of a
/// left-curving lane (simulation: a 0.64 m drift).
const FILLET_MAX_PEAK_CURVATURE: f64 = 0.3;
const FILLET_CHECK_M: f64 = 3.0;
const CORNER_SPEED: f64 = 8.0 / 3.6;
const CORNER_APPROACH_M: f64 = 12.0;
const CORNER_DECELERATION: f64 = 1.0;
/// Rounds off the corner at the end of `points` (arriving at `heading`,
/// turning by `turn` into `next`) with a curve matching both lanes'
/// positions and headings, and returns how far into `next` it ends and
/// how far back along `points` it starts: `FILLET_CUT_M`, or `merge` (m)
/// back to where a corner just before began, the two rounded off as one.
/// Junction lanes often begin at 20–34° to the road; handing each back to
/// the driver stopped the assist at most junction turns.
fn fillet_joint(
    points: &mut Vec<Point3>,
    heading: f64,
    turn: f64,
    next: &Road,
    merge: Option<f64>,
) -> Result<(f64, f64), &'static str> {
    if turn.abs() > FILLET_MAX_TURN_RAD {
        return Err("折角过大");
    }
    let cut = FILLET_CUT_M.min(next.length() / 2.0);
    let reach = merge.unwrap_or(cut);
    let back_length: f64 = points.windows(2).map(|p| p[0].distance_xy(p[1])).sum();
    if cut < 1.5 || back_length < reach + 2.0 {
        return Err("前后车道太短");
    }
    let (end, to) = next.sample(cut).ok_or("前后车道太短")?;
    // Back to where the curve starts, on the path as it is.
    let mut removed = Vec::new();
    let mut back = 0.0;
    while points.len() >= 3 && back < reach - 1e-6 {
        let n = points.len();
        back += points[n - 1].distance_xy(points[n - 2]);
        removed.extend(points.pop());
    }
    let n = points.len();
    let start = points[n - 1];
    let before = points
        .iter()
        .rev()
        .find(|p| p.distance_xy(start) >= 1.0)
        .copied()
        .unwrap_or(points[n - 2]);
    let from = if start.distance_xy(before) > 0.2 {
        (start.y - before.y).atan2(start.x - before.x)
    } else {
        heading
    };
    // The turn the curve really makes: the next lane may itself bend
    // sharply, and a corner that becomes too tight a turn stays with the
    // driver.
    let chord = start.distance_xy(end);
    let bend = wrap_angle(to - from).abs();
    let max_bend = if merge.is_some() {
        FILLET_MAX_MERGED_RAD
    } else {
        f64::INFINITY
    };
    if bend / chord.max(1e-6) > FILLET_MAX_CURVATURE || bend > max_bend {
        points.extend(removed.into_iter().rev());
        return Err("修圆后转弯过急");
    }
    // A cubic Hermite curve: position and heading match both lanes, both of
    // which may curve near a junction (a circular arc between their
    // tangents left a step where the next lane curved).
    let samples = (chord / 0.5).ceil().max(4.0) as usize;
    let (t0, t1) = (
        (chord * from.cos(), chord * from.sin()),
        (chord * to.cos(), chord * to.sin()),
    );
    let mut curve = Vec::with_capacity(samples);
    for k in 1..=samples {
        let t = k as f64 / samples as f64;
        let (t2, t3) = (t * t, t * t * t);
        let (h00, h10, h01, h11) = (
            2.0 * t3 - 3.0 * t2 + 1.0,
            t3 - 2.0 * t2 + t,
            -2.0 * t3 + 3.0 * t2,
            t3 - t2,
        );
        let p = Point3 {
            x: h00 * start.x + h10 * t0.0 + h01 * end.x + h11 * t1.0,
            y: h00 * start.y + h10 * t0.1 + h01 * end.y + h11 * t1.1,
            z: start.z + t * (end.z - start.z),
        };
        if curve
            .last()
            .or(points.last())
            .is_none_or(|last: &Point3| last.distance_xy(p) > MIN_PATH_STEP_M)
        {
            curve.push(p);
        }
    }
    // Sharper than the truck steers, with both lanes either side, it stays
    // with the driver.
    let lead: Vec<Point3> = points
        .iter()
        .rev()
        .take_while(|p| p.distance_xy(start) <= FILLET_CHECK_M)
        .copied()
        .collect();
    let trail = (1..=(FILLET_CHECK_M / 0.5) as usize)
        .filter_map(|i| next.sample(cut + i as f64 * 0.5).map(|(p, _)| p));
    let around: Vec<Point3> = lead
        .into_iter()
        .rev()
        .chain(curve.iter().copied())
        .chain(trail)
        .collect();
    if peak_curvature(&around) > FILLET_MAX_PEAK_CURVATURE {
        points.extend(removed.into_iter().rev());
        return Err("修圆后曲率过大");
    }
    points.extend(curve);
    Ok((cut, back))
}
/// Largest curvature (1/m) of a polyline, over chords of about 1.5 m either
/// side of each point.
fn peak_curvature(points: &[Point3]) -> f64 {
    let mut peak = 0.0_f64;
    for (i, b) in points.iter().enumerate() {
        let a = points[..i].iter().rev().find(|p| p.distance_xy(*b) >= 1.5);
        let c = points[i + 1..].iter().find(|p| p.distance_xy(*b) >= 1.5);
        if let (Some(a), Some(c)) = (a, c) {
            let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
            let sides = a.distance_xy(*b) * b.distance_xy(*c) * a.distance_xy(*c);
            peak = peak.max(2.0 * cross.abs() / sides.max(1e-9));
        }
    }
    peak
}
/// Through the corner from `start` to `end`, with the tractor's rear axle
/// on the path and the kingpin over it: the furthest (m) the first
/// trailer's axle group, `wheelbase` behind, runs off the path, and
/// whether the trailer's sides pass within `OPPOSING_CLEARANCE_M` of a lane
/// running the other way among `lanes`. The axle group does not slip
/// sideways, so it turns toward the kingpin as it is drawn along. Cutting
/// the inside of a turn across the oncoming lane, the trailer struck cars
/// waiting at the line of the road being turned into (simulation: five
/// times on seven routes).
fn trailer_cut(path: &Road, start: f64, end: f64, wheelbase: f64, lanes: &[Road]) -> (f64, bool) {
    const STEP_M: f64 = 0.25;
    if wheelbase <= 0.0 {
        return (0.0, false);
    }
    let from = (start - 2.0 * wheelbase).max(0.0);
    let to = (end + 2.0 * wheelbase).min(path.length());
    let Some((_, mut yaw)) = path.sample(from) else {
        return (f64::INFINITY, true);
    };
    // Lanes near this stretch of the path.
    let (window, _) = path.window(from, to);
    let near: Vec<&Road> = lanes
        .iter()
        .filter(|lane| window.iter().any(|p| lane.near(p.x, p.y, 20.0)))
        .collect();
    let mut peak = 0.0_f64;
    let mut opposing = false;
    let mut s = from;
    while s + STEP_M <= to {
        s += STEP_M;
        let Some((p, heading)) = path.sample(s) else {
            break;
        };
        yaw = wrap_angle(yaw + STEP_M * wrap_angle(heading - yaw).sin() / wheelbase);
        // Behind the start of the path the axle group has nothing to be off.
        if s < wheelbase {
            continue;
        }
        let (c, n) = ((yaw.cos(), yaw.sin()), (-yaw.sin(), yaw.cos()));
        let axle = (p.x - wheelbase * c.0, p.y - wheelbase * c.1);
        let off = (0..=(3.0 * wheelbase / STEP_M) as usize)
            .filter_map(|k| path.sample(s - k as f64 * STEP_M))
            .map(|(q, _)| (q.x - axle.0).hypot(q.y - axle.1))
            .fold(f64::INFINITY, f64::min);
        peak = peak.max(off);
        if opposing {
            continue;
        }
        // Both sides, from just behind the kingpin to the rear.
        for back in [2.0, 0.5 * wheelbase, wheelbase, wheelbase + 3.0] {
            for side in [-TRAILER_HALF_WIDTH_M, TRAILER_HALF_WIDTH_M] {
                let point = Point3 {
                    x: p.x - back * c.0 + side * n.0,
                    y: p.y - back * c.1 + side * n.1,
                    z: p.z,
                };
                opposing |= near.iter().any(|lane| {
                    lane.near(point.x, point.y, OPPOSING_CLEARANCE_M)
                        && project_polyline(0, &lane.points, point).is_ok_and(|(q, _)| {
                            q.distance < OPPOSING_CLEARANCE_M
                                && wrap_angle(q.heading - yaw).abs() > OPPOSING_HEADING_RAD
                        })
                });
            }
        }
    }
    (peak, opposing)
}
/// The offset (m, left positive) from the path of a trailer's axle group,
/// `wheelbase` behind a kingpin over the tractor's rear axle on the path,
/// as it passes each metre of the path; the largest where it passes one
/// metre more than once. Empty without a trailer.
fn trailer_offsets(path: &Road, wheelbase: f64) -> Vec<f64> {
    const STEP_M: f64 = 0.25;
    let Some((_, mut yaw)) = path.sample(0.0).filter(|_| wheelbase > 0.0) else {
        return Vec::new();
    };
    let mut out = vec![0.0_f64; path.length() as usize + 1];
    let mut s = 0.0;
    while s + STEP_M <= path.length() {
        s += STEP_M;
        let Some((p, heading)) = path.sample(s) else {
            break;
        };
        yaw = wrap_angle(yaw + STEP_M * wrap_angle(heading - yaw).sin() / wheelbase);
        if s < wheelbase {
            continue;
        }
        let axle = (p.x - wheelbase * yaw.cos(), p.y - wheelbase * yaw.sin());
        let nearest = (0..=(3.0 * wheelbase / STEP_M) as usize)
            .map(|k| s - k as f64 * STEP_M)
            .filter_map(|at| path.sample(at).map(|q| (at, q)))
            .min_by(|a, b| {
                let d = |(q, _): &(Point3, f64)| (q.x - axle.0).hypot(q.y - axle.1);
                d(&a.1).total_cmp(&d(&b.1))
            });
        if let Some((at, (q, h))) = nearest {
            let d = -(axle.0 - q.x) * h.sin() + (axle.1 - q.y) * h.cos();
            let slot = &mut out[(at as usize).min(path.length() as usize)];
            if d.abs() > slot.abs() {
                *slot = d;
            }
        }
    }
    out
}
/// A trailer side this close (m) to the centre of a lane running the other
/// way would touch a car standing on it; "the other way" is beyond 120°.
const OPPOSING_CLEARANCE_M: f64 = 1.6;
const OPPOSING_HEADING_RAD: f64 = 2.1;
const TRAILER_HALF_WIDTH_M: f64 = 1.3;
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
    /// The route joins the conflicting lane heading its way (a roundabout
    /// entry, a slip road) rather than crossing it.
    merge: bool,
    /// The conflict point lies this far beyond the stop line (m).
    beyond_m: f64,
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
/// Stop-line speed where traffic or the rule cannot be fully known.
const CAUTION_LINE_SPEED: f64 = 4.2;
/// Time gap behind a vehicle that reaches a zip merge first.
const ZIP_FOLLOW_S: f64 = 1.5;
/// A conflicting lane and its approaches up to `CONFLICT_APPROACH_M`; lanes
/// in `own` (the route and its approaches) are left out, and so are
/// approaches that must themselves give way before joining (`gives_way`):
/// at a roundabout the ring's predecessors include every other entry arm,
/// whose traffic yields to the ring (r14 waited 35 s for cars approaching
/// the opposite arm). Such traffic is watched once it is on the ring.
/// None when the lane is missing or too short to project the conflict
/// point onto.
fn conflict_lanes(
    id: &str,
    position: Point3,
    centers: &HashMap<String, LaneCenter>,
    own: &HashSet<&str>,
    gives_way: &dyn Fn(&str) -> bool,
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
            if own.contains(before.as_str()) || !seen.insert(before.as_str()) || gives_way(before) {
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
    // An approach with its own stop line, give-way or light yields before
    // joining the conflicting lane.
    let gives_way = |lane: &str| {
        traffic
            .upcoming_stop_controls(&[lane.to_owned()])
            .iter()
            .any(|c| {
                matches!(
                    c.control_kind,
                    ControlKind::Signal | ControlKind::Stop | ControlKind::Yield
                )
            })
    };
    for control in traffic.upcoming_stop_controls(&[id.to_owned()]) {
        let (p, _) = project_polyline(0, &road.points, control.position)?;
        let mut conflicts = Vec::new();
        let mut complete = true;
        let mut merge = !control.conflict_lane_ids.is_empty();
        for other in &control.conflict_lane_ids {
            // A conflict lane missing or too short to project is left
            // unchecked; the control then counts as conservative.
            match conflict_lanes(other, control.position, centers, own, &gives_way) {
                Some(lanes) => {
                    merge &= project_polyline(0, &lanes[0].points, control.position).is_ok_and(
                        |(q, _)| wrap_angle(q.heading - p.heading).abs() < MERGE_ANGLE_RAD,
                    );
                    conflicts.extend(lanes)
                }
                None => {
                    merge = false;
                    complete = false
                }
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
            merge,
            beyond_m: control.stop_before_m,
            key: format!("{id}:{:.2}:{:.2}", control.position.x, control.position.y),
        });
    }
    Ok(out)
}

/// Front-bumper gap to the first path slice whose ego-width rectangle the
/// body overlaps, from just behind the rear axle up to `SWEEP_RANGE_M`. Near
/// the truck the corridor also covers its current lateral offset, and a
/// trailer's cut inside a turn (`trailer`, see `trailer_offsets`).
fn sweep_gap(
    path: &Road,
    road: &RoadState,
    body: OrientedBox,
    reach: f64,
    range: f64,
    trailer: &[f64],
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
    // A slice is where the front reaches station `s`. In a tight turn the
    // cab is then off the path: it points along the path where the rear
    // axle is, 5 m back, and swings out of the turn by up to 3 m. The
    // slice spans the path and the cab, or a car waiting beside a junction
    // turn was not seen (taken at the slice's own curvature, the swing
    // was missed where the turn straightens and the cab is still out).
    let slice = |s: f64| -> Option<[f64; 5]> {
        let (p, heading) = path.sample(s)?;
        let half_width = corridor_half_width(road, reach, s);
        let swing = match path.sample(s - REAR_AXLE_TO_FRONT_M) {
            Some((r, h)) if s >= REAR_AXLE_TO_FRONT_M => {
                let front = (
                    r.x + REAR_AXLE_TO_FRONT_M * h.cos(),
                    r.y + REAR_AXLE_TO_FRONT_M * h.sin(),
                );
                // Left of the path at s positive.
                (-(front.0 - p.x) * heading.sin() + (front.1 - p.y) * heading.cos())
                    .clamp(-3.0, 3.0)
            }
            _ => 0.0,
        };
        // The trailer's axle group passes this station later, inside the
        // turn: a car waiting at the line of the road turned into was
        // struck by it (simulation).
        let cut = trailer.get(s.max(0.0) as usize).copied().unwrap_or(0.0);
        let (low, high) = (swing.min(cut).min(0.0), swing.max(cut).max(0.0));
        let side = (low + high) / 2.0;
        Some([
            p.x - side * heading.sin(),
            p.y + side * heading.cos(),
            heading,
            0.6,
            half_width + (high - low) / 2.0,
        ])
    };
    while s <= end {
        if obb_overlap(slice(s)?, target) {
            // Refine the first contact to 0.1 m inside the last 1 m step,
            // measured at the slice front edge.
            let mut contact = s;
            let mut probe = s - 0.9;
            while probe < s {
                if probe >= 0.0
                    && let Some(piece) = slice(probe)
                    && obb_overlap(piece, target)
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
/// Near the truck the corridor also covers its current lateral offset, at
/// the rear axle or the front, which a heading error puts further out: out
/// of an S-bend the cab pointed 0.2 rad off the path and struck a car
/// waiting 2 m beside it (simulation).
fn corridor_half_width(road: &RoadState, reach: f64, s: f64) -> f64 {
    let front = road.lateral_m - REAR_AXLE_TO_FRONT_M * road.heading_error_rad.sin();
    reach
        + road.lateral_m.abs().max(front.abs())
            * (1.0 - (s - road.station_m) / 20.0).clamp(0.0, 1.0)
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
/// Moving over within the lane for an oncoming car on the centre line: at
/// most `MAX_NUDGE_M` (a 3.5 m lane leaves 0.5 m beside a 2.5 m truck), at
/// `NUDGE_RATE_MPS`, for cars up to `NUDGE_RANGE_M` ahead, above
/// `NUDGE_MIN_SPEED` and below `NUDGE_MAX_CURVATURE` over 60 m; not toward a
/// car our way within `NUDGE_BESIDE_M` along and `NUDGE_BESIDE_LATERAL_M`
/// across.
const MAX_NUDGE_M: f64 = 0.3;
const NUDGE_RATE_MPS: f64 = 0.1;
const NUDGE_RANGE_M: f64 = 150.0;
const NUDGE_MIN_SPEED: f64 = 30.0 / 3.6;
const NUDGE_MAX_CURVATURE: f64 = 1.0 / 300.0;
const NUDGE_BESIDE_M: f64 = 30.0;
const NUDGE_BESIDE_LATERAL_M: f64 = 5.0;
/// Crossing bodies do not follow this path; straight extrapolation is only
/// trusted for a short horizon.
const CROSSING_HORIZON_S: f64 = 2.0;
/// Planned deceleration toward the end of the usable route prefix.
const STOP_DECELERATION: f64 = 1.2;
/// Within this distance a mapped light that ETS2LA does not report is dark.
const DARK_SIGNAL_DISTANCE_M: f64 = 60.0;
/// A light first seen at `DARK_SIGNAL_DISTANCE_M` is stopped for at this
/// deceleration (m/s²) at most.
const SIGNAL_SEEN_DECELERATION: f64 = 2.0;
/// Bodies slower than this are tested as stationary footprints.
const STATIONARY_SPEED: f64 = 0.5;
/// Oncoming bodies slower than this are tested as footprints too (m/s).
const CREEPING_SPEED: f64 = 3.0;
/// A moving body is swept along its lane over `CLEARING_HORIZON_S` (steps of
/// 0.5 s); one out of the corridor again by then, `CLEARING_MARGIN_S` before
/// the truck could reach it accelerating at `CLEARING_ACCELERATION`, passes
/// ahead and is not waited for.
const CLEARING_HORIZON_S: f64 = 4.0;
const CLEARING_MARGIN_S: f64 = 1.0;
const CLEARING_ACCELERATION: f64 = 1.5;
/// A body braking harder than this may stop in the corridor (m/s²).
const CLEARING_BRAKING: f64 = -1.0;
/// Only below this (m/s): a crossing car judged clear and then not, at
/// 50 km/h, braked the truck hard (simulation: 30 more hard brakes per
/// 100 km); roundabout entries and junctions are taken slower.
const CLEARING_MAX_SPEED: f64 = 7.0;
/// A moving car crossing the path is waited for this much closer than the
/// 10 m standstill gap to traffic ahead, fading out by
/// `CROSSING_CREDIT_SPEED` (m/s).
const CROSSING_GAP_CREDIT_M: f64 = 6.0;
const CROSSING_CREDIT_SPEED: f64 = 5.0;
/// Gap to conflicting traffic (s) needed to cross or join it, and to turn
/// across it; the latter shortens to `PATIENT_LEFT_TURN_GAP_S` between
/// `PATIENCE_AFTER_S` and that plus `PATIENCE_RAMP_S` of waiting.
const CROSSING_GAP_S: f64 = 4.0;
/// A route lane within this angle (rad) of the conflicting lane at the
/// conflict point joins it rather than crossing it.
const MERGE_ANGLE_RAD: f64 = 0.6;
/// Joining traffic: the truck's acceleration up to the merge (m/s²), and how
/// far ahead of (behind) the truck's front a car may reach the merge (s).
const MERGE_ACCELERATION: f64 = 1.0;
const MERGE_AHEAD_S: f64 = 1.0;
const MERGE_BEHIND_S: f64 = 3.0;
/// A conflicting car braking harder than this (m/s²) to stop at least
/// `STOPPING_MARGIN_M` short of the conflict point is not arriving.
const STOPPING_DECELERATION: f64 = 0.5;
const STOPPING_MARGIN_M: f64 = 2.0;
/// A standing car is waiting for the truck when its lane runs through the
/// truck within this distance (m) ahead of it.
const WAITING_LOOKAHEAD_M: f64 = 25.0;
/// Truck body behind the rear axle without a trailer (m).
const REAR_OVERHANG_M: f64 = 1.5;
const LEFT_TURN_GAP_S: f64 = 8.0;
const PATIENT_LEFT_TURN_GAP_S: f64 = 6.0;
const PATIENCE_AFTER_S: f64 = 15.0;
const PATIENCE_RAMP_S: f64 = 30.0;
/// Speed on the lead-in curve from a yard onto the first lane (m/s).
const LEAD_IN_SPEED: f64 = 8.0 / 3.6;
/// Bend speed: lateral acceleration 2.6 m/s² up to 40 km/h, easing to
/// 2.3 m/s² at 90 km/h. r11 used 1.2 m/s² everywhere and crept through
/// town bends at 20–30 km/h with 0.2 m p95 tracking error to spare. r12
/// held 1.5 m/s² and more above 50 km/h with no offset beyond 0.5 m.
///
/// Raised by 30% (and the trailer share from 0.8 to 0.9) on the driver's
/// word that bends were slowed for needlessly: live, a 220 m bend was taken
/// at 61 km/h and a 108 m one at 47 (1.3–1.6 m/s² with a trailer). Over r17
/// at a 90 km/h cap the higher limits added no drift and at most 0.07 m of
/// peak offset (simulation).
const CURVE_LATERAL_LOW: f64 = 2.6;
const CURVE_LATERAL_HIGH: f64 = 2.3;
const CURVE_LATERAL_TIGHT: f64 = 1.95;
const TIGHT_RADIUS_M: f64 = 30.0;
/// Below this radius the lateral limit falls to `CURVE_LATERAL_LOCK` at 6 m.
const VERY_TIGHT_RADIUS_M: f64 = 12.0;
const CURVE_LATERAL_LOCK: f64 = 0.6;
const OPEN_RADIUS_M: f64 = 80.0;
/// Share of the lateral acceleration limits used with a trailer attached.
const TRAILER_LATERAL_SHARE: f64 = 0.9;
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
/// Fastest speed at which relative steering with a wheel rate budget of
/// `cap` (rad/s at a crawl) steers a curvature change of `rate` (1/m per
/// m). The game turns the wheel more slowly at speed (r17), so the speed is
/// the one at which the scaled budget still suffices.
fn relative_transition_speed(cap: f64, rate: f64) -> f64 {
    let reach = cap / (WHEELBASE_M * rate.max(1e-6));
    let mut v = reach;
    for _ in 0..4 {
        v = reach * super::io::relative_speed_factor(v);
    }
    v
}
/// Fastest speed at which relatively steered wheels, whose turning rate
/// ramps at the servo's slew, follow a change of curvature change `bend`
/// (1/m³): a reversal in an S-bend needs the wheel stopped and turned back.
fn relative_ramp_speed(cap: f64, bend: f64) -> f64 {
    let accel = cap * RELATIVE_RAMP_SHARE;
    let reach = (accel / (WHEELBASE_M * bend.max(1e-9))).sqrt();
    let mut v = reach;
    for _ in 0..4 {
        v = reach * super::io::relative_speed_factor(v).sqrt();
    }
    v
}
/// The wheel rate budget's acceleration as a share of it (1/s): the servo
/// ramps the input at 0.76/s and plans with 90 % of that.
const RELATIVE_RAMP_SHARE: f64 = 0.9 * 0.76;
/// The same for the curvature the steering follows, averaged over a few
/// metres rather than read from the 10 m chords: r17 took a 12 m left kink
/// between a right bend and a straight at 47 km/h, the chords flattened it,
/// and the wheel reversed too late into the safety stop. Replayed, the kink
/// holds the lane at 40 km/h.
fn relative_steered_transition(path: &Road, s: f64, cap: f64) -> f64 {
    let change = path.mean_curvature(s, CURVE_STEP_M)
        - path.mean_curvature(s - 2.0 * CURVE_STEP_M, CURVE_STEP_M);
    relative_transition_speed(cap, change.abs() / (2.0 * CURVE_STEP_M))
}
/// Share of the relatively steered wheel's rate the planner spends on the
/// road's changes of curvature; the rest corrects tracking errors. At 0.85
/// the wheel had no room left after an S-bend: accelerating out of the
/// exit it overshot into 1.3–1.6 m drifts at a few junctions, and which
/// ones changed with every small perturbation (simulation). At 0.72 the
/// peak offset was 0.31 m on every run, for 1.5 % less average speed.
const RELATIVE_RATE_SHARE: f64 = 0.72;
/// A change of curvature centred this far behind the rear axle (m) still
/// limits the speed.
const CHANGE_BEHIND_M: f64 = 1.5;
/// Planning wheelbase of a two-axle tractor (m).
const WHEELBASE_M: f64 = 3.8;
/// A change of curvature is reached at its speed this long (s) at the
/// current speed before it, at most `CURVE_RATE_LEAD_M` ahead. A fixed 8 m
/// had the truck creep at 5 km/h for 6 s up to every roundabout entry and
/// exit; at 1 s the 44 roundabout routes took 6% less time, free or with
/// 15 cars, with fewer hard brakes and no drift (simulation).
const CURVE_RATE_LEAD_S: f64 = 1.0;
const CURVE_RATE_LEAD_M: f64 = 8.0;
/// Planned deceleration into a bend (m/s²).
const CURVE_DECELERATION: f64 = 1.5;
/// Free-road slowing above the target: the overspeed shed per second at
/// most (s).
const FREE_OVERSPEED_S: f64 = 1.0;
/// The truck stops this far (m) before the end of the usable route.
const ROUTE_END_STOP_M: f64 = 18.0;
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
/// Agent, body, body length bits, and the traffic lane the offset is taken
/// from (`None`: this path).
type TrackKey = (u64, usize, u64, Option<usize>);

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
    /// The GPS nodes either side of the first link the exported map lacks,
    /// when the route ends there (diagnostics: `probe-nodes` tells why).
    pub route_break: Option<[u64; 2]>,
    progress_s: f64,
    tracks: HashMap<TrackKey, LateralTrack>,
    approach_cap: Option<f64>,
    controls: Vec<RouteControl>,
    signal_heads: TrafficContext,
    signals_seen: Vec<SignalObservation>,
    /// Stop-sign dwell start time; `INFINITY` once the dwell is complete.
    stop_dwell: HashMap<String, f64>,
    /// When the truck stopped to give way at each control.
    yield_wait: HashMap<String, f64>,
    traffic_lanes: TrafficLanes,
    /// Length of the slow curve from a yard onto the first lane (0 if none).
    lead_in_m: f64,
    lane_changes: Vec<LaneChange>,
    /// Rounded-off corners between lanes: path stations where each starts
    /// and ends.
    corners: Vec<(f64, f64)>,
    /// Wheel rate budget (rad/s) when relative steering limits how fast the
    /// wheel can turn; `None` with absolute steering.
    wheel_rate_cap: Option<f64>,
    /// Attached trailers (none: a solo tractor).
    combination: super::trailer::Combination,
    /// How far (m) the first trailer's axle group is predicted to run off
    /// the path through each corner, for `combination`, and whether the
    /// trailer sweeps a lane running the other way (`trailer_cut`).
    corner_cuts: Vec<(f64, bool)>,
    /// With a trailer, its axle group's offset (m, left positive) from the
    /// path as it passes each metre of it (`trailer_offsets`).
    trailer_offsets: Vec<f64>,
    /// With a trailer, corners predicted to cut at most this far (m), and
    /// not across a lane running the other way, are driven; the route ends
    /// before any other. None: before every one.
    trailer_corner_cut: Option<f64>,
    /// Steered offset from the path (m, left positive) to give an oncoming
    /// car on the centre line room, and when it was last updated (s).
    nudge: f64,
    nudge_at: Option<f64>,
    /// Why the truck last gave way at a junction (diagnostics).
    yielding: Option<YieldEvidence>,
}
/// Every lane centre of the prepared region, cars' lanes included, to predict
/// moving traffic along its lane instead of straight ahead: on a bend, a
/// straight 2 s extrapolation of an oncoming car crosses our lane (r10 braked
/// and stopped for cars 20 m beside the route).
#[derive(Default)]
struct TrafficLanes {
    lanes: Vec<Road>,
    /// The lanes each lane leads on to (indices into `lanes`).
    successors: Vec<Vec<usize>>,
    /// Station of the first line on each lane where its traffic gives way
    /// (a give-way, a stop sign, a roundabout entry).
    gives_way: Vec<Option<f64>>,
}
/// A body matched to a lane: station and offset along it.
struct LaneFollow {
    lane: usize,
    s: f64,
    d: f64,
}
/// Ways on through the lane graph followed for one body at most; past
/// this, each way takes the straightest successor.
const MAX_LANE_WAYS: usize = 4;
/// A car that would brake harder than this (m/s²) to stop at its give-way
/// line is taken to drive on over it.
const YIELDING_DECELERATION: f64 = 2.5;
impl TrafficLanes {
    fn new(lanes: &[etsads_map::importer::ExportLane], traffic: &TrafficContext) -> Self {
        let kept: Vec<(&etsads_map::importer::ExportLane, Road)> = lanes
            .iter()
            .filter_map(|l| Some((l, Road::path(l.id.clone(), l.center.clone()).ok()?)))
            .collect();
        let index: HashMap<&str, usize> = kept
            .iter()
            .enumerate()
            .map(|(i, (l, _))| (l.id.as_str(), i))
            .collect();
        let successors = kept
            .iter()
            .map(|(l, _)| {
                l.successors
                    .iter()
                    .filter_map(|s| index.get(s.as_str()).copied())
                    .collect()
            })
            .collect();
        let mut gives_way: Vec<Option<f64>> = vec![None; kept.len()];
        let mut line = |lane: &str, at: Point3, before: f64| {
            if let Some(&i) = index.get(lane)
                && let Ok((p, _)) = project_polyline(0, &kept[i].1.points, at)
            {
                let s = (p.s - before).max(0.0);
                gives_way[i] = Some(gives_way[i].map_or(s, |g: f64| g.min(s)));
            }
        };
        for pair in &traffic.conflicts {
            for lane in [&pair.lane_a, &pair.lane_b] {
                if pair.priority_for(lane) == Some(RightOfWay::Yield) {
                    line(lane, pair.position(), pair.radius_m);
                }
            }
        }
        for stop in &traffic.stop_lines {
            if matches!(stop.control_kind, ControlKind::Stop | ControlKind::Yield) {
                line(&stop.lane_id, stop.position(), 0.0);
            }
        }
        Self {
            lanes: kept.into_iter().map(|(_, road)| road).collect(),
            successors,
            gives_way,
        }
    }
    #[cfg(test)]
    fn chain(lanes: Vec<Road>, successors: Vec<Vec<usize>>) -> Self {
        let gives_way = vec![None; lanes.len()];
        Self {
            lanes,
            successors,
            gives_way,
        }
    }
    /// How far a lane-following body can still go along `way` before it
    /// must give way, if a line comes and it can stop for it comfortably:
    /// cars entering a roundabout were predicted to drive on into the ring
    /// and the truck, with right of way there, stopped for each one in turn
    /// (simulation: 40 s in the ring).
    fn yields_within(
        &self,
        way: &[usize],
        follow: &LaneFollow,
        speed: f64,
        length: f64,
    ) -> Option<f64> {
        let mut offset = -follow.s;
        for (k, i) in way.iter().enumerate() {
            if let Some(line) = self.gives_way[*i]
                && (k > 0 || line >= follow.s)
            {
                let room = offset + line - length / 2.0;
                return (room >= 0.0 && speed * speed <= 2.0 * YIELDING_DECELERATION * room)
                    .then_some(room);
            }
            offset += self.lanes[*i].length();
        }
        None
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
    /// The ways a lane-following body may take over the next `travel`
    /// metres: its lane and the lanes after it, one list per branch (at most
    /// `MAX_LANE_WAYS`). A way ends early where the map links no lane on.
    /// Carried straight on past its lane's end, a car leaving a roundabout
    /// by the exit beside our entry was put across the truck's nose, and
    /// each one held the truck in turn (live and simulation).
    fn ways(&self, follow: &LaneFollow, travel: f64) -> Vec<Vec<usize>> {
        let mut done = Vec::new();
        let mut open = vec![(
            vec![follow.lane],
            self.lanes[follow.lane].length() - follow.s,
        )];
        while let Some((way, reach)) = open.pop() {
            let last = *way.last().expect("a way has a lane");
            let next = &self.successors[last];
            if reach >= travel || next.is_empty() || way.len() >= 32 {
                done.push(way);
                continue;
            }
            let branches = if done.len() + open.len() + next.len() <= MAX_LANE_WAYS {
                next.clone()
            } else {
                let heading = |i: usize, s: f64| self.lanes[i].sample(s).map_or(0.0, |p| p.1);
                let end = heading(last, self.lanes[last].length());
                next.iter()
                    .copied()
                    .min_by(|a, b| {
                        wrap_angle(heading(*a, 0.0) - end)
                            .abs()
                            .total_cmp(&wrap_angle(heading(*b, 0.0) - end).abs())
                    })
                    .into_iter()
                    .collect()
            };
            for lane in branches {
                let mut way = way.clone();
                way.push(lane);
                open.push((way, reach + self.lanes[lane].length()));
            }
        }
        done
    }
    /// Length of a way from the body on.
    fn reach(&self, way: &[usize], follow: &LaneFollow) -> f64 {
        way.iter().map(|i| self.lanes[*i].length()).sum::<f64>() - follow.s
    }
    /// Where a lane-following body is after `travel` metres along `way`;
    /// past its end (no lane linked on) it carries on straight.
    fn advance(&self, way: &[usize], follow: &LaneFollow, yaw: f64, travel: f64) -> Pose2 {
        let mut s = follow.s + travel;
        let mut lane = &self.lanes[follow.lane];
        for (k, i) in way.iter().enumerate() {
            lane = &self.lanes[*i];
            if s <= lane.length() || k + 1 == way.len() {
                break;
            }
            s -= lane.length();
        }
        let along = s.clamp(0.0, lane.length());
        let beyond = s - along;
        let (p, heading) = lane.sample(along).unwrap_or((lane.points[0], yaw));
        Pose2 {
            x: p.x - follow.d * heading.sin() + beyond * heading.cos(),
            y: p.y + follow.d * heading.cos() + beyond * heading.sin(),
            yaw: heading,
        }
    }
}
impl Guidance {
    /// With relative steering the wheel turns at most `rate_per_s` of its
    /// travel per second (about 0.29 rad/s at the default 0.4, against
    /// 0.72 rad/s with absolute steering at walking pace). Curvature changes
    /// are then budgeted with the same margin as the absolute 1.0/s slew.
    /// Trailers change gap acceptance, lane-change clearance and how fast
    /// a bend is taken (a loaded semitrailer rolls over well before a solo
    /// tractor slides).
    pub fn set_combination(&mut self, combination: super::trailer::Combination) {
        if self.corner_cuts.len() != self.corners.len()
            || combination.first_wheelbase_m != self.combination.first_wheelbase_m
        {
            let path = &self.roads.roads["gps"];
            self.trailer_offsets = if combination.trailers > 0 {
                trailer_offsets(path, combination.first_wheelbase_m)
            } else {
                Vec::new()
            };
            self.corner_cuts = self
                .corners
                .iter()
                .map(|&(start, end)| {
                    trailer_cut(
                        path,
                        start,
                        end,
                        combination.first_wheelbase_m,
                        &self.traffic_lanes.lanes,
                    )
                })
                .collect();
        }
        self.combination = combination;
    }
    /// See `trailer_corner_cut`.
    pub fn set_trailer_corner_cut(&mut self, cut: Option<f64>) {
        self.trailer_corner_cut = cut;
    }
    pub fn set_relative_steering(&mut self, rate_per_s: Option<f64>) {
        self.wheel_rate_cap = rate_per_s.map(|r| 2.0 * WHEEL_RATE_RAD_S * RELATIVE_RATE_SHARE * r);
        self.roads.feed_forward = rate_per_s.map_or(
            super::core::ABSOLUTE_FEED_FORWARD,
            super::core::FeedForward::relative,
        );
    }
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
        let mut yielding = None::<YieldEvidence>;
        let controls = std::mem::take(&mut self.controls);
        // Points along the truck from its tail to its front, and whether a
        // lane, ahead of station `from` on it, runs through the truck.
        let footprint: Vec<Point3> = {
            let path = &self.roads.roads["gps"];
            let tail = road.station_m - self.combination.behind_hitch_m.max(REAR_OVERHANG_M);
            (0..=((road.station_m + REAR_AXLE_TO_FRONT_M - tail).ceil() as usize))
                .filter_map(|i| path.sample(tail + i as f64).map(|(p, _)| p))
                .collect()
        };
        let blocked_by = |points: &[Point3], from: f64, width: f64| {
            let mut s = 0.0;
            points.windows(2).any(|w| {
                s += w[0].distance_xy(w[1]);
                (from..from + WAITING_LOOKAHEAD_M).contains(&s)
                    && footprint
                        .iter()
                        .any(|f| f.distance_xy(w[1]) < EGO_HALF_WIDTH_M + width / 2.0 + 0.3)
            })
        };
        for control in &controls {
            let distance = control.station_m - road.station_m - REAR_AXLE_TO_FRONT_M - 1.0;
            if !(-2.0..=250.0).contains(&distance) {
                continue;
            }
            let mut hold = None::<&str>;
            let mut cautious = false;
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
                    // Which check holds the truck: long waits at a light
                    // left no trace of why (live: 83-366 s, three times).
                    hold = Some(match decision {
                        SignalDecision::Hold(SignalHoldReason::NotGreen(_)) => "红灯，停车等待",
                        SignalDecision::Hold(SignalHoldReason::Stale) => {
                            "看不到信号灯状态（状态过期），停车等待"
                        }
                        SignalDecision::Hold(SignalHoldReason::AmbiguousGeometry) => {
                            "看不到信号灯状态（灯头归属不清），停车等待"
                        }
                        SignalDecision::Hold(SignalHoldReason::ConflictingStates) => {
                            "看不到信号灯状态（同组灯状态不一致），停车等待"
                        }
                        SignalDecision::Hold(SignalHoldReason::MissingObservation) => {
                            "看不到信号灯状态（交通插件未报告这组灯），停车等待"
                        }
                        SignalDecision::Hold(SignalHoldReason::UnresolvedProfile) => {
                            "看不到信号灯状态（灯型未解析），停车等待"
                        }
                        _ => "看不到信号灯状态，停车等待",
                    });
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
            // Conflicting traffic: arrival time at the conflict point. With
            // right of way (on a roundabout ring, a priority road) the other
            // side gives way; a car that does come out in front is met by the
            // traffic sweep. Waiting for them stopped the truck on the ring
            // for each car at the next entry (live).
            let mut arrivals = Vec::new();
            let mut arriving = Vec::new();
            let conflicts: &[ConflictLane] = if control.priority == RightOfWay::HasPriority {
                &[]
            } else {
                &control.conflicts
            };
            for lane in conflicts {
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
                    // A car standing in the conflict area whose way on runs
                    // through the truck is waiting for it, as ETS2 AI does
                    // for a player in its way: counting it as arriving left
                    // both waiting for each other (simulation: 90 s). One
                    // waiting for anything else may go at any moment.
                    if agent.speed <= 0.3 {
                        if gap <= 0.0 && !blocked_by(&lane.points, p.s, agent.width) {
                            {
                                arrivals.push(0.0);
                                arriving.push(agent.id);
                            }
                        }
                        continue;
                    }
                    if gap <= 0.0 {
                        {
                            arrivals.push(0.0);
                            arriving.push(agent.id);
                        }
                    } else {
                        // A car braking to a stop well short of the conflict
                        // (for its own line, a queue, or the truck) is not
                        // arriving: taken as speeding up, each car coming to
                        // a stop behind a queue held the truck in turn
                        // (simulation: 90 s at a left turn).
                        let v = agent.speed;
                        let stops_short = agent.acceleration < -STOPPING_DECELERATION
                            && v * v / (-2.0 * agent.acceleration) < gap - STOPPING_MARGIN_M;
                        if !stops_short {
                            {
                                arrivals.push((v * v + 2.0 * gap).sqrt() - v);
                                arriving.push(agent.id);
                            }
                        }
                    }
                }
            }
            let ego_arrival = distance.max(0.0) / speed.max(3.0);
            let evidence = |ego_s: f64| YieldEvidence {
                control: control.key.clone(),
                merge: control.merge,
                ego_s,
                arrivals: arriving
                    .iter()
                    .copied()
                    .zip(arrivals.iter().copied())
                    .collect(),
            };
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
            } else if !arrivals.is_empty() && control.merge {
                // Joining traffic heading our way (a roundabout ring): a car
                // through the merge a second before the truck's front is
                // followed, one three seconds behind it falls in behind the
                // combination. Counted to the stop line and with the trailer
                // as a crossing, a gap of 8.5 s was needed and a busy ring
                // never gave one (live and simulation: 30–50 s at an entry).
                let d = distance.max(0.0) + control.beyond_m;
                let v = speed.max(0.0);
                let at = ((v * v + 2.0 * MERGE_ACCELERATION * d).sqrt() - v) / MERGE_ACCELERATION;
                if arrivals
                    .iter()
                    .any(|t| *t > at - MERGE_AHEAD_S && *t < at + MERGE_BEHIND_S)
                {
                    hold = Some("让行：等待冲突车道来车");
                    yielding = Some(evidence(at));
                }
            } else if !arrivals.is_empty() && {
                // A trailer keeps the conflict point occupied after the
                // tractor has crossed: traffic arriving before the whole
                // combination is through counts as arriving with it.
                let occupied = self.combination.behind_hitch_m / speed.max(3.0);
                let arrivals: Vec<f64> = arrivals
                    .iter()
                    .map(|t| {
                        if *t > ego_arrival {
                            (t - occupied).max(ego_arrival)
                        } else {
                            *t
                        }
                    })
                    .collect();
                // Turning across oncoming traffic needs a long gap; after
                // a long wait at the line it is shortened as a driver's is
                // (simulation: 90 s waiting in steady town traffic).
                let waited = if speed.abs() < 0.5 && distance < 5.0 {
                    time_s - *self.yield_wait.entry(control.key.clone()).or_insert(time_s)
                } else {
                    self.yield_wait.remove(&control.key);
                    0.0
                };
                let gap = if control.left_turn {
                    let patience = ((waited - PATIENCE_AFTER_S) / PATIENCE_RAMP_S).clamp(0.0, 1.0);
                    LEFT_TURN_GAP_S - (LEFT_TURN_GAP_S - PATIENT_LEFT_TURN_GAP_S) * patience
                } else {
                    CROSSING_GAP_S
                };
                arrivals.iter().any(|t| (ego_arrival - t).abs() < gap)
            } {
                hold = Some("让行：等待冲突车道来车");
                yielding = Some(evidence(ego_arrival));
            }
            if let Some(reason) = hold {
                if stop.as_ref().is_none_or(|(d, _)| distance < *d) {
                    stop = Some((distance.max(0.0), reason.to_owned()));
                }
            } else if (control.conservative || cautious) && distance > 0.0 && !zip {
                // With every conflicting approach seen and no car due within
                // the gap of the truck there is nothing to give way to: on
                // empty Norwegian roads each unlit mapped light and give-way
                // was taken at 15–30 km/h (live, 12% of the time). Unseen
                // traffic is approached at 15 km/h at the line. A light not
                // yet seen may still be working: approached so that a
                // comfortable stop is possible once it is.
                if let Some(note) = note.or_else(|| {
                    unknown_rule(control.kind, control.priority)
                        .then_some("路口规则未知，按让行规则通过")
                }) {
                    notice.get_or_insert_with(|| note.to_owned());
                }
                let unseen_light = cautious && note.is_none();
                if !visible {
                    cap = cap.min(
                        (CAUTION_LINE_SPEED * CAUTION_LINE_SPEED
                            + 2.0 * STOP_DECELERATION * distance)
                            .sqrt(),
                    );
                } else if unseen_light {
                    let known = (2.0 * SIGNAL_SEEN_DECELERATION * DARK_SIGNAL_DISTANCE_M).sqrt();
                    cap = cap.min(
                        (known * known
                            + 2.0 * STOP_DECELERATION * (distance - DARK_SIGNAL_DISTANCE_M))
                            .max(0.0)
                            .sqrt(),
                    );
                }
            }
        }
        self.controls = controls;
        self.yielding = yielding;
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
        let mut candidate = next.roads.observe(pose, z, speed, Some("gps"))?;
        // With a trailer both routes end before their first rounded corner.
        next.set_combination(self.combination);
        next.trailer_corner_cut = self.trailer_corner_cut;
        let mut current = current;
        current.remaining_m = current
            .remaining_m
            .min(self.usable_end() - current.station_m);
        candidate.remaining_m = candidate
            .remaining_m
            .min(next.usable_end() - candidate.station_m);
        ensure!(
            (candidate.lateral_m - current.lateral_m).abs() < ADOPT_LATERAL_M
                && wrap_angle(candidate.heading_error_rad - current.heading_error_rad).abs()
                    < ADOPT_HEADING_RAD,
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
        next.nudge = self.nudge;
        next.nudge_at = self.nudge_at;
        next.approach_cap = None;
        next.stop_dwell = std::mem::take(&mut self.stop_dwell);
        next.yield_wait = std::mem::take(&mut self.yield_wait);
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
        let mut road = road;
        let end = self.usable_end();
        if end - road.station_m < road.remaining_m {
            road.remaining_m = end - road.station_m;
            self.terminal = "前方路口转弯带挂车，请手动接管".into();
        }
        // Steered beside the path by the nudge: the offset and the pursuit
        // target are taken from the shifted line, so the speed holds and the
        // drift watch judge the truck against where it is meant to be.
        if self.nudge != 0.0
            && let Some((origin, _)) = self.roads.roads["gps"].sample(road.station_m)
        {
            let heading = (road.preview.y - origin.y).atan2(road.preview.x - origin.x);
            road.preview.x -= self.nudge * heading.sin();
            road.preview.y += self.nudge * heading.cos();
            road.lateral_m -= self.nudge;
        }
        Ok(road)
    }
    /// Where the usable route ends: with a trailer, before the first
    /// rounded-off corner ahead, with the front stopped short of it. Those
    /// are junction turns at about a 5 m radius, where a semi-trailer cuts
    /// inside by several metres; neither the corridor nor the simulation
    /// models that yet, so the driver takes them.
    fn usable_end(&self) -> f64 {
        let length = self.roads.roads["gps"].length();
        if self.combination.trailers == 0 {
            return length;
        }
        let allowed = |i: usize| {
            self.trailer_corner_cut.is_some_and(|limit| {
                self.corner_cuts
                    .get(i)
                    .is_some_and(|(cut, opposing)| !opposing && *cut <= limit)
            })
        };
        self.corners
            .iter()
            .enumerate()
            .find(|(i, (start, _))| *start > self.progress_s - 1.0 && !allowed(*i))
            .map_or(length, |(_, (start, _))| {
                (start + ROUTE_END_STOP_M - REAR_AXLE_TO_FRONT_M).min(length)
            })
    }
    /// Length of the route path (m).
    pub fn length(&self) -> f64 {
        self.roads.roads["gps"].length()
    }
    /// The route path point and heading at station `s` (simulation: road
    /// height and grade).
    pub fn sample(&self, s: f64) -> Option<(Point3, f64)> {
        let path = &self.roads.roads["gps"];
        path.sample(s.clamp(0.0, path.length()))
    }
    /// Signed curvature (1/m) of the route path at station `s`, over the
    /// planner's 3 m step (simulation diagnostics).
    pub fn curvature(&self, s: f64) -> f64 {
        self.roads.roads["gps"].mean_curvature(s, CURVE_STEP_M)
    }
    /// The route path from station `from` to `to` every `step` metres:
    /// station, position and curvature (simulation diagnostics).
    pub fn path_window(&self, from: f64, to: f64, step: f64) -> Vec<(f64, Point3, f64)> {
        let path = &self.roads.roads["gps"];
        let mut out = Vec::new();
        let mut s = from.max(0.0);
        while s <= to.min(path.length()) {
            if let Some((p, _)) = path.sample(s) {
                out.push((s, p, path.mean_curvature(s, 1.5)));
            }
            s += step;
        }
        out
    }
    /// Traffic-light heads of the route's signal groups: id, position and
    /// the profile's phase intervals (simulation).
    pub fn signal_heads(&self) -> Vec<(String, Point3, [f64; 4], f64)> {
        self.signal_heads
            .signals
            .iter()
            .map(|s| {
                (
                    s.id.clone(),
                    s.position(),
                    [s.interval_0, s.interval_1, s.interval_2, s.interval_3],
                    s.cycle_delay_s,
                )
            })
            .collect()
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
        // A rounded-off corner between lanes is a junction turn, taken at
        // walking pace and slowed for in good time: relatively steered, the
        // wheel starts turning for it well before.
        for (start, end) in &self.corners {
            if road.station_m <= *end + REAR_AXLE_TO_FRONT_M {
                let d = (*start - road.station_m - CORNER_APPROACH_M).max(0.0);
                bind(
                    &mut target,
                    (CORNER_SPEED * CORNER_SPEED + 2.0 * CORNER_DECELERATION * d).sqrt(),
                    Binding::Curve,
                );
            }
        }
        let horizon = (speed * speed / 3.0 + speed * 3.0 + 30.0).max(60.0);
        let wheel_rate_cap = self.wheel_rate_cap;
        // Samples on a fixed grid of the path: sampled from the truck, a
        // short kink fell between samples or onto one as the truck moved, and
        // the target jumped between them every frame (simulation: 20 pedal
        // reversals per km). The grid starts 12 m behind: a change of
        // curvature is judged over the samples around its middle, and one
        // centred at the truck is still being steered. Judged only from
        // samples ahead, a reversal under way at the truck no longer bound
        // and the target leapt up halfway through S-bends (simulation:
        // 1.3 m drifts and safety stops).
        let mut d = (CURVE_STEP_M - road.station_m.rem_euclid(CURVE_STEP_M)) % CURVE_STEP_M
            - 4.0 * CURVE_STEP_M;
        let mut previous = [None::<f64>; 4];
        while d <= horizon && road.station_m + d < path.length() {
            if road.station_m + d < 0.0 {
                d += CURVE_STEP_M;
                continue;
            }
            let signed = path.chord_curvature(road.station_m + d, CURVE_CHORD_HALF_M);
            let curvature = signed.abs();
            // Where curvature changes (a roundabout exit, an S-bend) the
            // wheel must turn at wheelbase · dκ/ds · v. The 2.0 m/s² L1
            // roundabout exit at 27 km/h reversed the wheel too late and
            // drifted 0.9 m.
            // A reversal keeps the 10 m/s wheel rate at every speed.
            if let Some(last) = previous[1].filter(|_| d - CURVE_STEP_M >= -CHANGE_BEHIND_M) {
                let rate = (signed - last).abs() / (2.0 * CURVE_STEP_M);
                let mut transition = if signed * last < 0.0 {
                    WHEEL_RATE_RAD_S / (WHEELBASE_M * rate.max(1e-6))
                } else {
                    transition_speed(rate)
                };
                if let Some(cap) = wheel_rate_cap {
                    transition = transition
                        .min(relative_transition_speed(cap, rate))
                        .min(relative_steered_transition(path, road.station_m + d, cap));
                    // The wheel also needs time to start and stop turning.
                    if let Some(older) =
                        previous[3].filter(|_| d - 2.0 * CURVE_STEP_M >= -CHANGE_BEHIND_M)
                    {
                        let bend =
                            (signed - 2.0 * last + older).abs() / (2.0 * CURVE_STEP_M).powi(2);
                        transition = transition.min(relative_ramp_speed(cap, bend));
                    }
                }
                let lead = (CURVE_RATE_LEAD_S * speed).min(CURVE_RATE_LEAD_M);
                bind(
                    &mut target,
                    (transition * transition
                        + 2.0 * CURVE_DECELERATION * (d - 2.0 * CURVE_STEP_M - lead).max(0.0))
                    .sqrt(),
                    Binding::CurveRate,
                );
            }
            previous = [Some(signed), previous[0], previous[1], previous[2]];
            if d < 0.0 {
                d += CURVE_STEP_M;
                continue;
            }
            // The corner speed is solved with the lateral limit at that speed.
            let lateral_share = if self.combination.trailers > 0 {
                TRAILER_LATERAL_SHARE
            } else {
                1.0
            };
            let mut corner =
                (lateral_share * tight_lateral(curvature) / curvature.max(1e-6)).sqrt();
            if corner > CURVE_LATERAL_FADE_FROM {
                corner = corner
                    .min((lateral_share * curve_lateral(corner) / curvature.max(1e-6)).sqrt());
            }
            bind(
                &mut target,
                (corner * corner + 2.0 * CURVE_DECELERATION * (d - 8.0).max(0.0)).sqrt(),
                Binding::Curve,
            );
            d += CURVE_STEP_M;
        }
        let stop_gap = (road.remaining_m - ROUTE_END_STOP_M).max(0.0);
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
        // Above the target the free road slows the truck by the overspeed
        // per second at most: the IDM's (v/v₀)⁴ read 0.3 m/s over a 6 km/h
        // bend as −0.9 m/s² and braked, the throttle came back below it and
        // the pedals swapped every half second (simulation).
        let free = idm
            .acceleration(speed.max(0.0), None)?
            .max(-(speed - target).max(0.0) / FREE_OVERSPEED_S);
        let mut acceleration = free;
        let (traffic_path, station_offset) = path.window(
            (road.station_m - 20.0).max(0.0),
            road.station_m + horizon + 60.0,
        );
        // Swept corridor of the rear axle and the front bumper, both relative
        // to the path centre; tracking error widens it on its own side only.
        // `road` is measured from the nudged line; the corridor is from the
        // path's centre, around the line the truck keeps to.
        let actual = road.lateral_m + self.nudge;
        let front_lateral = actual - REAR_AXLE_TO_FRONT_M * road.heading_error_rad.sin();
        let reach = EGO_HALF_WIDTH_M + LATERAL_CLEARANCE_M;
        let corridor = (
            actual.min(front_lateral).min(self.nudge) - reach,
            actual.max(front_lateral).max(self.nudge) + reach,
        );
        // Room an oncoming car on its lane needs on either side (m, left
        // and right of the path), and whether a car our way is beside us.
        let (mut give_left, mut give_right) = (0.0_f64, 0.0_f64);
        let (mut beside_left, mut beside_right) = (false, false);
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
                let key = (agent.id, index, body.length.to_bits(), None);
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
                // So are oncoming cars creeping up to their line: in a tight
                // junction turn the cab swings into the opposite lane, and a
                // car tested as following its lane was only seen once it
                // stopped, too late (simulation: a collision at 3 km/h).
                if motion == Motion::Crossing
                    || agent.speed.abs() < STATIONARY_SPEED
                    || (motion == Motion::Opposing && agent.speed.abs() < CREEPING_SPEED)
                {
                    let moving = agent.speed.abs() >= STATIONARY_SPEED;
                    let follow = if moving {
                        self.traffic_lanes.matched(body.pose, agent.z)
                    } else {
                        None
                    };
                    // Each way the body may take on is swept on its own; a
                    // body without a lane moves straight (one way).
                    let ways = follow.as_ref().map_or_else(
                        || vec![Vec::new()],
                        |f| {
                            self.traffic_lanes
                                .ways(f, agent.speed.abs() * CLEARING_HORIZON_S)
                        },
                    );
                    // The closest binding way: gap, time and merging speed.
                    let mut binds = None::<(f64, f64, f64)>;
                    for way in &ways {
                        let stops_in = follow.as_ref().and_then(|f| {
                            self.traffic_lanes
                                .yields_within(way, f, agent.speed, body.length)
                        });
                        let mut best = None::<(f64, f64)>;
                        let mut merging = 0.0_f64;
                        // Nearest gap and last time in the corridor over the
                        // longer sweep of a lane-following body.
                        let mut nearest = f64::INFINITY;
                        let mut last_inside = None::<f64>;
                        let steps = (CLEARING_HORIZON_S / 0.5).round() as usize;
                        for t in (0..=steps).map(|i| i as f64 * 0.5) {
                            if t > 0.0 && !moving || t > 2.0 && follow.is_none() {
                                break;
                            }
                            let moved = OrientedBox {
                                pose: match &follow {
                                    Some(f) => self.traffic_lanes.advance(
                                        way,
                                        f,
                                        body.pose.yaw,
                                        (agent.speed * t).min(stops_in.unwrap_or(f64::INFINITY)),
                                    ),
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
                            // A lane-following body that, within the sweep,
                            // ends up on the path heading along it merges
                            // ahead of the truck (a car on a roundabout ring, a
                            // slip road): it is followed at its speed, not
                            // waited for as a standing obstacle (r14 braked
                            // hard behind ring cars driving away).
                            let aligned = follow
                                .as_ref()
                                .and(
                                    project_polyline(
                                        0,
                                        traffic_path,
                                        Point3 {
                                            x: moved.pose.x,
                                            y: moved.pose.y,
                                            z: agent.z,
                                        },
                                    )
                                    .ok(),
                                )
                                .filter(|(p, _)| p.d.abs() < reach)
                                .map(|(p, _)| wrap_angle(moved.pose.yaw - p.heading))
                                .filter(|a| a.abs() < 0.35);
                            if let Some(a) = aligned
                                && t <= 2.0
                            {
                                merging = merging.max((agent.speed * a.cos()).max(0.0));
                            }
                            let Some(gap) = sweep_gap(
                                path,
                                road,
                                moved,
                                reach,
                                horizon + 60.0,
                                &self.trailer_offsets,
                            ) else {
                                continue;
                            };
                            nearest = nearest.min(gap);
                            last_inside = Some(t);
                            if t <= 2.0 && best.is_none_or(|(g, _)| gap < g) {
                                best = Some((gap, t));
                            }
                        }
                        // A car crossing ahead, out of the corridor again well
                        // before the truck could get there, is not waited for:
                        // a ring car passing a roundabout entry 20 m ahead
                        // stopped the truck for every car in turn (live), and
                        // so did those that left the ring before the entry.
                        let clears = follow.is_some()
                            && speed < CLEARING_MAX_SPEED
                            && agent.acceleration > CLEARING_BRAKING
                            && last_inside.is_some_and(|t| {
                                let v = speed.max(0.0);
                                let arrival =
                                    ((v * v + 2.0 * CLEARING_ACCELERATION * nearest).sqrt() - v)
                                        / CLEARING_ACCELERATION;
                                t < CLEARING_HORIZON_S && t + CLEARING_MARGIN_S < arrival
                            });
                        if let Some((gap, t)) = best.filter(|_| !clears) {
                            // A car crossing through is gone in moments: at
                            // walking pace the truck waits for it closer than
                            // the following distance. Held 10 m back, it
                            // stopped well short of a roundabout's line, too
                            // far back to find a gap in the ring (simulation).
                            let credit = if motion == Motion::Crossing && moving {
                                CROSSING_GAP_CREDIT_M
                                    * (1.0 - speed / CROSSING_CREDIT_SPEED).clamp(0.0, 1.0)
                            } else {
                                0.0
                            };
                            acceleration = acceleration.min(
                                idm.acceleration(speed.max(0.0), Some((gap + credit, merging)))?,
                            );
                            if binds.is_none_or(|(g, _, _)| gap < g) {
                                binds = Some((gap, t, merging));
                            }
                        }
                    }
                    if let Some((gap, t, lead_speed)) = binds
                        && lead.is_none_or(|l| gap < l.gap_m)
                    {
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
                            speed_mps: lead_speed,
                            gap_m: gap,
                        });
                    }
                    continue;
                }
                // A body on a lane of its own keeps to it: its drift is taken
                // against that lane and only carried past the lane's centre
                // while it moves away from it (a lane change). Cars turning
                // in from a side road converge on the opposite lane; their
                // drift against this path, extrapolated, put them 3 m into
                // ours (live: stopped for oncoming cars at a tunnel junction).
                let own_lane = (motion != Motion::Crossing)
                    .then(|| self.traffic_lanes.matched(body.pose, agent.z))
                    .flatten()
                    .map(|follow| {
                        let key = (agent.id, index, body.length.to_bits(), Some(follow.lane));
                        let rate = Self::track(&mut self.tracks, key, time_s, follow.d);
                        (follow, rate)
                    });
                // The ways on along its lane, for an oncoming body; one going
                // our way only drifts where it is.
                // An oncoming car on its own lane whose near side is inside
                // the corridor: the truck moves over within its lane, as a
                // driver does, rather than braking (live: braked from 65 to
                // 19 km/h for one 1.0-1.7 m off its lane centre).
                if motion == Motion::Opposing
                    && own_lane.is_some()
                    && (0.0..NUDGE_RANGE_M).contains(&station)
                {
                    let need = reach - (current.d.abs() - half_lateral);
                    if need > 0.0 && current.d > 0.0 {
                        give_right = give_right.max(need);
                    } else if need > 0.0 {
                        give_left = give_left.max(need);
                    }
                }
                if motion == Motion::SameDirection
                    && station.abs() < NUDGE_BESIDE_M
                    && (1.0..NUDGE_BESIDE_LATERAL_M).contains(&current.d.abs())
                {
                    if current.d > 0.0 {
                        beside_left = true;
                    } else {
                        beside_right = true;
                    }
                }
                let last_t = PREDICTION_TIMES_S[PREDICTION_TIMES_S.len() - 1];
                let ways = own_lane
                    .as_ref()
                    .map(|(follow, _)| self.traffic_lanes.ways(follow, agent.speed.abs() * last_t));
                for t in PREDICTION_TIMES_S {
                    // Path-following bodies keep their observed offset and only
                    // drift by the measured lateral rate, along each way their
                    // lane leads on. Crossing bodies move straight and are
                    // stationary obstructions where they land. Past the end of
                    // the lanes the map links (a dead end) a body's way is
                    // unknown: it is then taken to follow this path as
                    // before, not to carry straight on.
                    let travel = if motion == Motion::Opposing {
                        agent.speed * t
                    } else {
                        0.0
                    };
                    let mut at = Vec::new();
                    let mut on_lane = false;
                    if let (Some((follow, rate)), Some(ways)) = (&own_lane, &ways)
                        && motion != Motion::Crossing
                    {
                        let drift = if rate * follow.d < 0.0 {
                            (rate * t).clamp(-follow.d.abs(), follow.d.abs())
                        } else {
                            rate * t
                        };
                        let drifted = LaneFollow {
                            d: follow.d + drift,
                            ..*follow
                        };
                        for way in ways {
                            if self.traffic_lanes.reach(way, follow) < travel {
                                continue;
                            }
                            on_lane = true;
                            let travel = self
                                .traffic_lanes
                                .yields_within(way, follow, agent.speed.abs(), body.length)
                                .map_or(travel, |room| travel.min(room));
                            let moved =
                                self.traffic_lanes
                                    .advance(way, &drifted, body.pose.yaw, travel);
                            let Ok((p, _)) = project_polyline(
                                0,
                                traffic_path,
                                Point3 {
                                    x: moved.x,
                                    y: moved.y,
                                    z: agent.z,
                                },
                            ) else {
                                continue;
                            };
                            if p.distance <= p.d.abs() + 0.5 {
                                at.push((p.s + station_offset - road.station_m, p.d));
                            }
                        }
                    }
                    if !on_lane {
                        at.push(match motion {
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
                        });
                    }
                    let lead_speed = if motion == Motion::SameDirection {
                        along.max(0.0)
                    } else {
                        0.0
                    };
                    let Some((gap, s, d)) = at
                        .into_iter()
                        .filter(|(s, d)| {
                            d + half_lateral > corridor.0
                                && d - half_lateral < corridor.1
                                && s + half_station >= -2.0
                        })
                        .map(|(s, d)| ((s - half_station - REAR_AXLE_TO_FRONT_M).max(0.0), s, d))
                        .min_by(|a, b| a.0.total_cmp(&b.0))
                    else {
                        continue;
                    };
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
        // The nudge: only on a straight, quick, plain stretch, toward a side
        // with no car beside the truck, never both ways at once.
        let calm = speed > NUDGE_MIN_SPEED
            && self.active_lane_change(road.station_m, speed).is_none()
            && (0..=6).all(|k| {
                self.curvature(road.station_m + 10.0 * k as f64).abs() < NUDGE_MAX_CURVATURE
            })
            && self.corners.iter().all(|(start, end)| {
                road.station_m > end + REAR_AXLE_TO_FRONT_M || road.station_m + 60.0 < *start
            });
        let nudge_target = match (calm, give_left > 0.0, give_right > 0.0) {
            (true, false, true) if !beside_right => -give_right.min(MAX_NUDGE_M),
            (true, true, false) if !beside_left => give_left.min(MAX_NUDGE_M),
            _ => 0.0,
        };
        let dt = self
            .nudge_at
            .map_or(0.0, |at| (time_s - at).clamp(0.0, 0.2));
        self.nudge_at = Some(time_s);
        self.nudge = nudge_target.clamp(
            self.nudge - NUDGE_RATE_MPS * dt,
            self.nudge + NUDGE_RATE_MPS * dt,
        );
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
        // Only a constraint from traffic or a hard stop shapes it here. The
        // free road's own deceleration above the target is not: at walking
        // pace a 0.3 m/s overshoot made the target 3 km/h lower than the
        // bend's, the brake undershot it and the throttle came back
        // (simulation: a throttle–brake cycle every 0.6 s in junctions).
        if acceleration < free - 0.05 {
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
            yielding: self.yielding.clone(),
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
            // The trailer (if any) extends the truck this far behind the axle.
            let behind_clear = -gap - half - 1.0 - self.combination.behind_hitch_m
                >= 8.0 + 2.5 * (along - speed).max(0.0)
                // A vehicle behind that is not closing in cannot block the
                // change. L3: cars queued behind the waiting truck were
                // projected onto the target lane's backward extension and
                // the two waited for each other.
                || (-gap - half - 1.0 - self.combination.behind_hitch_m >= 0.0
                    && along <= speed + 0.5);
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
/// A new route is swapped in only where its path coincides with the current
/// one at the truck. Eased curve ends differ near a route's start (where
/// the new one keeps its line), so the swap waits for where they agree
/// rather than jump the reference by up to `EASE_MAX_M`.
const ADOPT_LATERAL_M: f64 = 0.08;
const ADOPT_HEADING_RAD: f64 = 0.02;
/// The cars a junction hold waits for: id and arrival time at the
/// conflict point (s), against the truck's own (`ego_s`).
#[derive(Clone, Debug, Serialize)]
pub struct YieldEvidence {
    pub control: String,
    pub merge: bool,
    pub ego_s: f64,
    pub arrivals: Vec<(u64, f64)>,
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
    pub yielding: Option<YieldEvidence>,
}

#[cfg(test)]
mod tests {
    use super::*;
    /// r17: the 12 m left kink of the lane split, between a right bend and
    /// the straight, is planned below the 40 km/h it held in replay; the
    /// straight beyond stays fast.
    #[test]
    fn relative_steering_slows_for_the_r17_lane_split() {
        let points: Vec<Point3> = include_str!("../../testdata/r17-split-path.txt")
            .lines()
            .filter(|l| !l.starts_with('#'))
            .map(|l| {
                let v: Vec<f64> = l.split_whitespace().map(|v| v.parse().unwrap()).collect();
                Point3 {
                    x: v[0],
                    y: v[1],
                    z: 0.0,
                }
            })
            .collect();
        let path = Road::path("gps".into(), points).unwrap();
        let cap = 2.0 * WHEEL_RATE_RAD_S * 0.85 * 0.4;
        let speed = |from: f64, to: f64| {
            (from as usize..to as usize)
                .map(|s| relative_steered_transition(&path, s as f64, cap))
                .fold(f64::INFINITY, f64::min)
                * 3.6
        };
        let (kink, straight) = (speed(90.0, 115.0), speed(118.0, 130.0));
        eprintln!("r17 split: kink {kink:.0} km/h, straight beyond {straight:.0} km/h");
        assert!(kink < 40.0, "kink planned at {kink:.0} km/h");
        assert!(straight > 60.0, "straight planned at {straight:.0} km/h");
    }
    fn guidance() -> Guidance {
        Guidance {
            roads: Roads {
                feed_forward: super::super::core::ABSOLUTE_FEED_FORWARD,
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
            route_break: None,
            progress_s: 20.0,
            tracks: HashMap::new(),
            approach_cap: None,
            controls: Vec::new(),
            signal_heads: TrafficContext::default(),
            signals_seen: Vec::new(),
            stop_dwell: HashMap::new(),
            yield_wait: HashMap::new(),
            traffic_lanes: TrafficLanes::default(),
            lead_in_m: 0.0,
            corners: Vec::new(),
            lane_changes: Vec::new(),
            wheel_rate_cap: None,
            combination: Default::default(),
            corner_cuts: Vec::new(),
            trailer_offsets: Vec::new(),
            trailer_corner_cut: None,
            nudge: 0.0,
            nudge_at: None,
            yielding: None,
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
    fn a_car_merging_ahead_along_its_lane_is_followed_not_waited_for() {
        // A slip road (or a roundabout ring) joins the route 25 m ahead at
        // 35°; a car on it 8 m short of the join drives on along the route.
        let mut g = guidance();
        let join = 60.0;
        let mut points = Vec::new();
        for i in 0..=30 {
            let back = 30.0 - i as f64;
            let a = 35.0_f64.to_radians();
            points.push(Point3 {
                x: join - back * a.cos(),
                y: -back * a.sin(),
                z: 0.0,
            });
        }
        for i in 1..=100 {
            points.push(Point3 {
                x: join + i as f64,
                y: 0.0,
                z: 0.0,
            });
        }
        g.traffic_lanes = TrafficLanes::chain(
            vec![Road::path("slip".into(), points).unwrap()],
            vec![Vec::new()],
        );
        let pose = Pose2 {
            x: 35.0,
            y: 0.0,
            yaw: 0.0,
        };
        g.progress_s = 34.0;
        let road = g.observe(pose, 0.0, 8.0, &[1, 2]).unwrap();
        let a = 35.0_f64.to_radians();
        let merging = Agent {
            pose: Pose2 {
                x: join - 8.0 * a.cos(),
                y: -8.0 * a.sin(),
                yaw: a,
            },
            ..agent(0.0, 0.0, 10.0)
        };
        let plan = g
            .speed(&road, pose, 8.0, 25.0, 25.0, &[merging], 0.0)
            .unwrap();
        let lead = plan.lead.expect("the merging car is the lead");
        assert!(lead.speed_mps > 8.0, "followed at its speed: {lead:?}");
        assert!(
            plan.acceleration > -1.5,
            "no hard stop for a car driving away: {}",
            plan.acceleration
        );
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
    #[test]
    fn a_trailer_is_predicted_to_cut_inside_a_turn() {
        // 60 m straight, a 90° left turn of radius 15 m, 60 m straight.
        let mut points: Vec<Point3> = (0..=60)
            .map(|i| Point3 {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            })
            .collect();
        for k in 1..=60 {
            let a = k as f64 / 60.0 * std::f64::consts::FRAC_PI_2;
            points.push(Point3 {
                x: 60.0 + 15.0 * a.sin(),
                y: 15.0 - 15.0 * a.cos(),
                z: 0.0,
            });
        }
        for i in 1..=60 {
            points.push(Point3 {
                x: 75.0,
                y: 15.0 + i as f64,
                z: 0.0,
            });
        }
        let path = Road::path("gps".into(), points).unwrap();
        let arc = 15.0 * std::f64::consts::FRAC_PI_2;
        let (cut, opposing) = trailer_cut(&path, 60.0, 60.0 + arc, 8.0, &[]);
        assert!(!opposing);
        let steady = 15.0 - (15.0_f64.powi(2) - 64.0).sqrt();
        assert!(cut > 1.8 && cut < steady + 0.05, "{cut} vs {steady}");
        assert!(trailer_cut(&path, 5.0, 20.0, 8.0, &[]).0 < 0.05);
        assert_eq!(trailer_cut(&path, 60.0, 80.0, 0.0, &[]), (0.0, false));
        // The oncoming lane of the road turned into, 3.5 m inside the exit,
        // is swept as the trailer cuts in; a lane 6 m inside is not.
        let lane = |x: f64| {
            Road::path(
                "other".into(),
                (0..=30)
                    .map(|i| Point3 {
                        x,
                        y: 45.0 - i as f64,
                        z: 0.0,
                    })
                    .collect(),
            )
            .unwrap()
        };
        assert!(trailer_cut(&path, 60.0, 60.0 + arc, 8.0, &[lane(71.5)]).1);
        assert!(!trailer_cut(&path, 60.0, 60.0 + arc, 8.0, &[lane(69.0)]).1);
        // Metre by metre: inside the turn (left, positive) by about as much,
        // none on the straight before it.
        let offsets = trailer_offsets(&path, 8.0);
        let peak = offsets.iter().copied().fold(0.0_f64, f64::max);
        assert!((peak - cut).abs() < 0.1, "{peak} vs {cut}");
        assert!(offsets.iter().all(|d| *d > -0.05));
        assert!(offsets[..55].iter().all(|d| d.abs() < 0.05));
        assert!(trailer_offsets(&path, 0.0).is_empty());
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
            merge: false,
            beyond_m: 0.0,
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
    fn yield_waits_for_crossing_traffic_and_keeps_its_speed_when_clear() {
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
        // Clear crossing: no slowing when all cross traffic is visible,
        // 15 km/h at the line when it may not be.
        let clear = g.speed(&road, pose, 6.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(!clear.reason.starts_with("让行"));
        assert!(
            !matches!(
                clear.binding,
                Binding::JunctionCaution | Binding::JunctionStop
            ),
            "{:?}",
            clear.binding
        );
        let cap = |line: f64| (line * line + 2.0 * STOP_DECELERATION * 24.0).sqrt();
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
    /// Live, on a roundabout ring: the truck stopped for each car coming up
    /// to the next entry, which gives way to the ring.
    #[test]
    fn with_right_of_way_the_truck_does_not_wait_for_the_side_road() {
        let plan = |priority: RightOfWay| {
            let mut g = guidance();
            let kind = if priority == RightOfWay::HasPriority {
                ControlKind::PriorityRoad
            } else {
                ControlKind::Yield
            };
            let mut junction = control(kind, priority, 300.0);
            junction.conflicts = vec![ConflictLane {
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
            g.controls = vec![junction];
            let pose = Pose2 {
                x: 270.0,
                y: 0.0,
                yaw: 0.0,
            };
            g.progress_s = 270.0;
            let road = g.observe(pose, 0.0, 6.0, &[1, 2]).unwrap();
            let mut side = agent(300.0, -40.0, 10.0);
            side.pose.yaw = std::f64::consts::FRAC_PI_2;
            g.speed(&road, pose, 6.0, 25.0, 15.0, &[side], 0.0).unwrap()
        };
        let priority = plan(RightOfWay::HasPriority);
        assert!(!priority.reason.starts_with("让行"), "{}", priority.reason);
        assert!(priority.yielding.is_none());
        let giving_way = plan(RightOfWay::Yield);
        assert!(
            giving_way.reason.starts_with("让行"),
            "{}",
            giving_way.reason
        );
        let evidence = giving_way
            .yielding
            .expect("the hold says what it waits for");
        assert_eq!(evidence.arrivals.len(), 1);
    }
    #[test]
    fn a_car_stopping_short_does_not_hold_and_waiting_shortens_the_turn_gap() {
        let mut g = guidance();
        let mut turn = control(ControlKind::Yield, RightOfWay::Yield, 300.0);
        turn.left_turn = true;
        // Oncoming traffic crosses our path at x=300, running north.
        turn.conflicts = vec![ConflictLane {
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
        g.controls = vec![turn];
        // Standing at the line.
        let pose = Pose2 {
            x: 293.5,
            y: 0.0,
            yaw: 0.0,
        };
        g.progress_s = 293.5;
        let road = g.observe(pose, 0.0, 0.0, &[1, 2]).unwrap();
        let north = |y: f64, speed: f64, acceleration: f64| Agent {
            pose: Pose2 {
                x: 300.0,
                y,
                yaw: std::f64::consts::FRAC_PI_2,
            },
            acceleration,
            ..agent(300.0, y, speed)
        };
        let reason = |g: &mut Guidance, car: Agent, time_s: f64| {
            g.speed(&road, pose, 0.0, 25.0, 15.0, &[car], time_s)
                .unwrap()
                .reason
        };
        // Arriving in under 4 s it holds the truck; braking to a stop 10 m
        // short of the crossing it does not.
        assert!(reason(&mut g, north(-40.0, 10.0, 0.0), 0.0).starts_with("让行"));
        assert!(!reason(&mut g, north(-40.0, 10.0, -2.0), 0.0).starts_with("让行"));
        // Arriving in 7 s: too short a gap to turn across at first, taken
        // after a long wait.
        let later = north(-83.0, 8.0, 0.0);
        assert!(reason(&mut g, later.clone(), 0.0).starts_with("让行"));
        assert!(reason(&mut g, later.clone(), 10.0).starts_with("让行"));
        assert!(!reason(&mut g, later, 50.0).starts_with("让行"));
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
        // r12 crossed every green light at 15 km/h. A dark one is given way
        // at: at road speed with no one coming, slowly when traffic on its
        // approaches cannot all be seen.
        g.observe_signals(lamp(0));
        let dark = g.speed(&road, pose, 12.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(dark.target > 14.0, "{}", dark.target);
        g.controls[0].complete = false;
        let unseen = g.speed(&road, pose, 12.0, 25.0, 15.0, &[], 0.0).unwrap();
        assert!(unseen.target < 10.0, "{}", unseen.target);
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
        let open = |_: &str| false;
        let lanes = conflict_lanes("junction", position, &centers, &own, &open).unwrap();
        // The junction lane, then a (10 + 120 m) and b (250 m); b reaches
        // past 200 m so c is not walked.
        let stations: Vec<f64> = lanes.iter().map(|l| l.conflict_s).collect();
        assert_eq!(stations.len(), 3, "{stations:?}");
        assert!((stations[0] - 10.0).abs() < 1e-6);
        assert!((stations[1] - 130.0).abs() < 1e-6);
        assert!((stations[2] - 250.0).abs() < 1e-6);
        assert!(conflict_lanes("missing", position, &centers, &own, &open).is_none());
        // A roundabout ring's approach that gives way itself (another entry
        // arm) is not walked, nor anything behind it.
        let arm_yields = |lane: &str| lane == "a";
        let lanes = conflict_lanes("junction", position, &centers, &own, &arm_yields).unwrap();
        assert_eq!(lanes.len(), 1);
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
            (1, 0, 0, None),
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
        // Relative steering turns the wheel more slowly: the reversal and
        // the turn-in are taken slower, the steady bend is not.
        g.set_relative_steering(Some(0.4));
        let relative = target_at(&mut g, 96);
        assert_eq!(relative.binding, Binding::CurveRate);
        assert!(
            relative.target < reversal.target - 0.3,
            "relative {:.1} vs absolute {:.1} km/h",
            relative.target * 3.6,
            reversal.target * 3.6
        );
        assert!((target_at(&mut g, 70).target - steady.target).abs() < 1e-9);
    }
    #[test]
    fn a_moderate_corner_between_lanes_is_rounded_off() {
        let straight = |x0: f64, heading: f64| -> Vec<Point3> {
            (0..=20)
                .map(|i| Point3 {
                    x: x0 + i as f64 * heading.cos(),
                    y: i as f64 * heading.sin(),
                    z: 0.0,
                })
                .collect()
        };
        let turn = 0.5_f64;
        let next = Road::path("next".into(), straight(20.0, turn)).unwrap();
        let mut points = straight(0.0, 0.0);
        let (cut, back) = fillet_joint(&mut points, 0.0, turn, &next, None).unwrap();
        assert!((FILLET_CUT_M..FILLET_CUT_M + 1.0).contains(&back));
        assert!((cut - FILLET_CUT_M).abs() < 1e-9);
        // It ends where the next lane continues, heading its way.
        let (end, _) = next.sample(cut).unwrap();
        let n = points.len();
        assert!(points[n - 1].distance_xy(end) < 0.05);
        let last = (points[n - 1].y - points[n - 2].y).atan2(points[n - 1].x - points[n - 2].x);
        assert!((last - turn).abs() < 0.05, "{last}");
        // Smooth: no step turns sharply, and the path passes inside the
        // corner by about a circular arc's offset.
        for w in points.windows(3) {
            let a = (w[1].y - w[0].y).atan2(w[1].x - w[0].x);
            let b = (w[2].y - w[1].y).atan2(w[2].x - w[1].x);
            assert!(wrap_angle(b - a).abs() < 0.06);
        }
        let radius = cut / (turn / 2.0).tan();
        let inside = radius * (1.0 / (turn / 2.0).cos() - 1.0);
        let corner = Point3 {
            x: 20.0,
            y: 0.0,
            z: 0.0,
        };
        let closest = points
            .iter()
            .map(|p| p.distance_xy(corner))
            .fold(f64::INFINITY, f64::min);
        assert!((closest - inside).abs() < 0.3, "{closest} vs {inside}");
        // Sharper corners stay with the driver, and the path is left as it was.
        let mut points = straight(0.0, 0.0);
        let sharp = Road::path("sharp".into(), straight(20.0, 0.8)).unwrap();
        assert_eq!(
            fillet_joint(&mut points, 0.0, 0.8, &sharp, None),
            Err("折角过大")
        );
        assert_eq!(points.len(), 21);
        // So do corners that turn into a tight curve.
        let tight: Vec<Point3> = (0..=20)
            .map(|i| {
                let a = 0.5 + i as f64 * 0.3;
                Point3 {
                    x: 20.0 + 3.0 * a.sin() - 3.0 * 0.5_f64.sin(),
                    y: 3.0 * 0.5_f64.cos() - 3.0 * a.cos(),
                    z: 0.0,
                }
            })
            .collect();
        let tight = Road::path("tight".into(), tight).unwrap();
        let mut points = straight(0.0, 0.0);
        assert!(fillet_joint(&mut points, 0.0, 0.5, &tight, None).is_err());
        assert_eq!(points.len(), 21);
        // A junction connector 6 m long joined at 0.4 rad at each end: the
        // second corner comes right after the first and both are rounded
        // off as one turn from where the first began.
        let segment = |from: Point3, heading: f64, length: f64| -> Vec<Point3> {
            (0..=(length as usize))
                .map(|i| Point3 {
                    x: from.x + i as f64 * heading.cos(),
                    y: from.y + i as f64 * heading.sin(),
                    z: 0.0,
                })
                .collect()
        };
        let corner = Point3 {
            x: 20.0,
            y: 0.0,
            z: 0.0,
        };
        let connector = segment(corner, 0.4, 6.0);
        let joint = *connector.last().unwrap();
        let connector = Road::path("connector".into(), connector).unwrap();
        let onward = Road::path("onward".into(), segment(joint, 0.8, 20.0)).unwrap();
        let mut points = straight(0.0, 0.0);
        let (cut, back) = fillet_joint(&mut points, 0.0, 0.4, &connector, None).unwrap();
        let start = 20.0 - back;
        points.push(joint);
        let length = |p: &[Point3]| p.windows(2).map(|w| w[0].distance_xy(w[1])).sum::<f64>();
        let before = length(&points);
        assert!((cut - 3.0).abs() < 1e-9);
        let (cut, merged) =
            fillet_joint(&mut points, 0.4, 0.4, &onward, Some(before - start)).unwrap();
        assert!((before - merged - start).abs() < 1.0, "{merged}");
        assert!(
            points
                .iter()
                .filter(|p| p.x < start - 0.5)
                .all(|p| p.y == 0.0)
        );
        let n = points.len();
        assert!(points[n - 1].distance_xy(onward.sample(cut).unwrap().0) < 0.05);
        for w in points.windows(3) {
            let a = (w[1].y - w[0].y).atan2(w[1].x - w[0].x);
            let b = (w[2].y - w[1].y).atan2(w[2].x - w[1].x);
            assert!(wrap_angle(b - a).abs() < 0.08);
        }
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
        g.traffic_lanes = TrafficLanes::chain(
            vec![Road::path("side".into(), lane).unwrap()],
            vec![Vec::new()],
        );
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
    /// Straight, then an arc of `radius` turning `angle`, then straight.
    fn arc_path(radius: f64, angle: f64) -> Vec<Point3> {
        let mut points: Vec<Point3> = (0..100)
            .map(|i| Point3 {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            })
            .collect();
        let steps = (radius * angle).ceil() as usize;
        for k in 0..=steps {
            let a = angle * k as f64 / steps as f64;
            points.push(Point3 {
                x: 100.0 + radius * a.sin(),
                y: radius * (1.0 - a.cos()),
                z: 0.0,
            });
        }
        let end = *points.last().unwrap();
        for i in 1..=100 {
            points.push(Point3 {
                x: end.x + i as f64 * angle.cos(),
                y: end.y + i as f64 * angle.sin(),
                z: 0.0,
            });
        }
        points
    }
    fn steepest_change(road: &Road) -> f64 {
        (10..road.length() as usize - 10)
            .map(|s| {
                let s = s as f64;
                (road.chord_curvature(s + 3.0, 3.0) - road.chord_curvature(s - 3.0, 3.0)).abs()
                    / 6.0
            })
            .fold(0.0, f64::max)
    }
    fn largest_shift(from: &[Point3], to: &[Point3]) -> f64 {
        to.iter()
            .map(|p| project_polyline(0, from, *p).unwrap().0.distance)
            .fold(0.0, f64::max)
    }
    #[test]
    fn curve_ends_are_eased_within_the_lane_and_tight_turns_over_a_few_metres() {
        let original = arc_path(80.0, 1.2);
        let eased = ease_curve_ends(&original);
        assert!(largest_shift(&original, &eased) <= EASE_MAX_M + 0.01);
        let (before, after) = (
            Road::path("a".into(), original.clone()).unwrap(),
            Road::path("b".into(), eased.clone()).unwrap(),
        );
        let (steep, eased_steep) = (steepest_change(&before), steepest_change(&after));
        assert!(eased_steep < 0.6 * steep, "{steep:.5} → {eased_steep:.5}");
        // Mid-arc the line is kept.
        let middle = before.sample(100.0 + 48.0).unwrap().0;
        let (p, _) = project_polyline(0, &eased, middle).unwrap();
        assert!(p.distance < 0.03, "{}", p.distance);
        // A junction-like turn of 10 m radius is entered and left over a few
        // metres, without a clamped (kinked) shift.
        let tight = arc_path(10.0, 1.57);
        let eased_tight = ease_curve_ends(&tight);
        let moved = largest_shift(&tight, &eased_tight);
        assert!((0.1..EASE_TIGHT_MAX_M - 0.05).contains(&moved), "{moved}");
        let (before, after) = (
            Road::path("a".into(), tight.clone()).unwrap(),
            Road::path("b".into(), eased_tight).unwrap(),
        );
        let (steep, eased_steep) = (steepest_change(&before), steepest_change(&after));
        assert!(eased_steep < 0.6 * steep, "{steep:.5} → {eased_steep:.5}");
        // A roundabout ring of 15 m radius has its ends eased, a little.
        let ring = arc_path(15.0, 1.57);
        let moved = largest_shift(&ring, &ease_curve_ends(&ring));
        assert!((0.04..EASE_MAX_M).contains(&moved), "{moved}");
    }
    /// Live, a roundabout entry: each ring car crossing 20 m ahead stopped
    /// the truck, though it was gone seconds before the truck got there.
    #[test]
    fn a_car_crossing_well_ahead_is_not_waited_for_but_a_slow_one_is() {
        let plan = |truck: f64, car_y: f64, car_speed: f64, braking: f64| {
            let mut g = guidance();
            let lane = (0..=80)
                .map(|i| Point3 {
                    x: 45.0,
                    y: 40.0 - i as f64,
                    z: 0.0,
                })
                .collect();
            g.traffic_lanes = TrafficLanes::chain(
                vec![Road::path("ring".into(), lane).unwrap()],
                vec![Vec::new()],
            );
            let pose = Pose2 {
                x: 20.0,
                y: 0.0,
                yaw: 0.0,
            };
            let road = g.observe(pose, 0.0, truck, &[1, 2]).unwrap();
            let mut car = agent(45.0, car_y, car_speed);
            car.pose.yaw = -std::f64::consts::FRAC_PI_2;
            car.acceleration = braking;
            g.speed(&road, pose, truck, 25.0, 25.0, &[car], 0.0)
                .unwrap()
        };
        // 25 m ahead at 5 m/s; the car, 8 m to the side at 10 m/s, is
        // through the corridor within 1.5 s.
        let passing = plan(5.0, 8.0, 10.0, 0.0);
        assert!(passing.lead.is_none(), "{:?}", passing.lead);
        assert!(passing.acceleration > -0.3, "{}", passing.acceleration);
        // The same car braking may stop across the path.
        let braking = plan(5.0, 8.0, 10.0, -3.0);
        assert!(braking.lead.is_some());
        // A car creeping across at 2 m/s is still there when the truck is.
        let slow = plan(5.0, 3.0, 2.0, 0.0);
        assert!(slow.lead.is_some());
        assert!(slow.acceleration < -0.5, "{}", slow.acceleration);
    }
    /// Live, a roundabout entry: ring cars leaving by the exit beside it
    /// were carried straight on past the end of their ring lane, across the
    /// nose of the truck waiting at the line, and held it one after another.
    /// Followed onto the exit they pass beside it; a way on that does cross
    /// the path is still waited for.
    #[test]
    fn a_ring_car_taking_the_exit_beside_the_truck_is_not_waited_for() {
        let line = |a: (f64, f64), b: (f64, f64)| -> Vec<Point3> {
            let n = (b.0 - a.0).hypot(b.1 - a.1).ceil() as usize;
            (0..=n)
                .map(|i| {
                    let k = i as f64 / n as f64;
                    Point3 {
                        x: a.0 + (b.0 - a.0) * k,
                        y: a.1 + (b.1 - a.1) * k,
                        z: 0.0,
                    }
                })
                .collect()
        };
        let plan = |onward: Vec<Point3>| {
            let mut g = guidance();
            // The ring lane ends 7 m ahead of the nose and 6 m to the left;
            // straight on it would cross the path 2 m ahead of the nose.
            let ring = line((40.0, 16.0), (32.0, 6.0));
            let mut exit = vec![
                Point3 {
                    x: 32.0,
                    y: 6.0,
                    z: 0.0,
                },
                Point3 {
                    x: 30.0,
                    y: 4.8,
                    z: 0.0,
                },
            ];
            exit.extend(line((28.0, 4.5), (-50.0, 4.5)));
            g.traffic_lanes = TrafficLanes::chain(
                vec![
                    Road::path("ring".into(), ring).unwrap(),
                    Road::path("exit".into(), exit).unwrap(),
                    Road::path("onward".into(), onward).unwrap(),
                ],
                vec![vec![1, 2], Vec::new(), Vec::new()],
            );
            let pose = Pose2 {
                x: 20.0,
                y: 0.0,
                yaw: 0.0,
            };
            let road = g.observe(pose, 0.0, 0.0, &[1, 2]).unwrap();
            let mut car = agent(36.0, 11.0, 6.0);
            car.pose.yaw = (-10.0_f64).atan2(-8.0);
            g.speed(&road, pose, 0.0, 25.0, 25.0, &[car], 0.0).unwrap()
        };
        // Its other way on stays on the ring, away from the path.
        let leaving = plan(line((32.0, 6.0), (25.0, 20.0)));
        assert!(leaving.lead.is_none(), "{:?}", leaving.lead);
        assert!(leaving.acceleration > 0.0, "{}", leaving.acceleration);
        // A way on across the path in front of the truck.
        let crossing = plan(line((32.0, 6.0), (22.0, -6.0)));
        assert!(crossing.lead.is_some());
        assert!(crossing.acceleration <= 0.0, "{}", crossing.acceleration);
    }
    /// Simulation, inside a roundabout: cars coming up to the next entry
    /// were predicted on into the ring across the truck's path, and the
    /// truck, with right of way, stopped for each one. One that can still
    /// stop comfortably at its give-way line is taken to; one too fast to is
    /// not.
    #[test]
    fn a_car_that_can_stop_at_its_give_way_line_is_taken_to() {
        let plan = |car_speed: f64, line: Option<f64>| {
            let mut g = guidance();
            let lane = (0..=80)
                .map(|i| Point3 {
                    x: 45.0,
                    y: 40.0 - i as f64,
                    z: 0.0,
                })
                .collect();
            g.traffic_lanes = TrafficLanes::chain(
                vec![Road::path("entry".into(), lane).unwrap()],
                vec![Vec::new()],
            );
            // The line 3.5 m short of the path.
            g.traffic_lanes.gives_way[0] = line;
            let pose = Pose2 {
                x: 20.0,
                y: 0.0,
                yaw: 0.0,
            };
            let road = g.observe(pose, 0.0, 5.0, &[1, 2]).unwrap();
            let mut car = agent(45.0, 12.0, car_speed);
            car.pose.yaw = -std::f64::consts::FRAC_PI_2;
            g.speed(&road, pose, 5.0, 25.0, 25.0, &[car], 0.0).unwrap()
        };
        let unmarked = plan(5.0, None);
        assert!(unmarked.lead.is_some(), "without a line it may cross");
        let giving_way = plan(5.0, Some(36.5));
        assert!(giving_way.lead.is_none(), "{:?}", giving_way.lead);
        // 8 m/s with 6 m to its line would need 5.3 m/s².
        let too_fast = plan(8.0, Some(36.5));
        assert!(too_fast.lead.is_some());
    }
    /// Live, a tunnel junction: cars turning in from a side road came onto
    /// the opposite lane from beyond it, closing on this path at 1.5 m/s.
    /// Extrapolated for 2 s that put them in our lane and the truck stopped;
    /// they settled in their own lane 4.5 m away.
    #[test]
    fn the_truck_moves_over_for_an_oncoming_car_on_the_centre_line() {
        let mut g = guidance();
        let lane = (0..=400)
            .rev()
            .map(|i| Point3 {
                x: i as f64,
                y: 3.5,
                z: 0.0,
            })
            .collect();
        g.traffic_lanes = TrafficLanes::chain(
            vec![Road::path("opposite".into(), lane).unwrap()],
            vec![Vec::new()],
        );
        let pose = Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        };
        let road = g.observe(pose, 0.0, 20.0, &[1, 2]).unwrap();
        // 1.0 m off its lane centre toward ours, 140 m ahead, steady.
        let car = |t: f64| {
            let mut car = agent(160.0 - 15.0 * t, 2.5, 15.0);
            car.pose.yaw = std::f64::consts::PI;
            car
        };
        let first = g
            .speed(&road, pose, 20.0, 25.0, 25.0, &[car(0.0)], 0.0)
            .unwrap();
        assert!(first.lead.is_some(), "its near side is in the corridor");
        let mut last = first;
        for step in 1..=30 {
            let t = step as f64 * 0.1;
            last = g
                .speed(&road, pose, 20.0, 25.0, 25.0, &[car(t)], t)
                .unwrap();
        }
        assert!((-0.3..-0.1).contains(&g.nudge), "{}", g.nudge);
        assert!(last.lead.is_none(), "{:?}", last.lead);
        // The shifted line is what the truck is measured against.
        let shifted = g.observe(pose, 0.0, 20.0, &[1, 2]).unwrap();
        assert!((shifted.lateral_m - (road.lateral_m - g.nudge)).abs() < 1e-9);
        // Gone, the truck moves back.
        for step in 31..=70 {
            let t = step as f64 * 0.1;
            g.speed(&road, pose, 20.0, 25.0, 25.0, &[], t).unwrap();
        }
        assert_eq!(g.nudge, 0.0);
    }
    #[test]
    fn an_oncoming_car_settling_into_its_lane_is_not_a_cut_in() {
        let run = |with_lane: bool| {
            let mut g = guidance();
            if with_lane {
                let lane = (0..=200)
                    .rev()
                    .map(|i| Point3 {
                        x: i as f64,
                        y: 4.5,
                        z: 0.0,
                    })
                    .collect();
                g.traffic_lanes = TrafficLanes::chain(
                    vec![Road::path("opposite".into(), lane).unwrap()],
                    vec![Vec::new()],
                );
            }
            let pose = Pose2 {
                x: 20.0,
                y: 0.0,
                yaw: 0.0,
            };
            let road = g.observe(pose, 0.0, 6.0, &[1, 2]).unwrap();
            let mut plan = None;
            for step in 0..6 {
                let t = step as f64 * 0.1;
                let mut car = agent(60.0 - 8.0 * t, 5.9 - 1.5 * t, 8.0);
                car.pose.yaw = std::f64::consts::PI + 0.19;
                plan = Some(g.speed(&road, pose, 6.0, 25.0, 25.0, &[car], t).unwrap());
            }
            plan.unwrap()
        };
        let path_drift = run(false);
        assert!(
            path_drift.lead.is_some(),
            "its drift against this path alone is a cut-in"
        );
        let settling = run(true);
        assert!(settling.lead.is_none(), "{:?}", settling.lead);
        assert!(settling.acceleration > -0.5, "{}", settling.acceleration);
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
