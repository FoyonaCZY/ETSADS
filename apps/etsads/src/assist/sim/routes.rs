//! Routes for the simulation that nobody drove: a random walk over the
//! map's lanes from a start point, written in the captured route format
//! (`testdata/r17-route.txt`), so the assist meets junctions and bends the
//! recorded drives never passed.
use super::super::navigation::Navigator;
use super::traffic::Rng;
use anyhow::{Context, ensure};
use etsads_map::importer::ExportLane;
use etsads_msgs::{Point3, frames::wrap_angle};
use std::collections::{HashMap, HashSet};
use std::fmt::Write;

/// Lanes are reloaded once the walk is this far from where they were
/// loaded (they cover 2 km around it).
const RELOAD_M: f64 = 1200.0;
/// A successor turning back more than this (rad) is a U-turn.
const U_TURN_RAD: f64 = 2.3;
/// The SDK reference is this far ahead of the rear axle (m).
const SDK_AHEAD_OF_REAR_M: f64 = 2.02;

fn length(lane: &ExportLane) -> f64 {
    lane.center.windows(2).map(|p| p[0].distance_xy(p[1])).sum()
}
fn heading(a: Point3, b: Point3) -> f64 {
    (b.y - a.y).atan2(b.x - a.x)
}
/// A random route of about `km` world kilometres from a road lane near
/// `start`, as route-file text: a depth-first walk that picks successors
/// at random, never drives a lane twice or turns back, and backs out of
/// dead ends (roads into yards end without a lane on).
pub fn random_route(
    map: &std::path::Path,
    start: Point3,
    km: f64,
    seed: u64,
) -> anyhow::Result<String> {
    let navigator = Navigator::open(map)?;
    let mut rng = Rng::new(seed);
    // Lanes loaded so far, and where they were loaded around.
    let mut lanes: HashMap<String, ExportLane> = HashMap::new();
    let mut centres: Vec<Point3> = Vec::new();
    let mut load = |at: Point3, lanes: &mut HashMap<String, ExportLane>| -> anyhow::Result<()> {
        if centres.iter().any(|c| c.distance_xy(at) < RELOAD_M) {
            return Ok(());
        }
        let (region, _) = navigator.region_lanes(at)?;
        for lane in region {
            if lane.truck_access != Some(false) && lane.center.len() >= 2 && length(&lane) > 0.5 {
                lanes.entry(lane.id.clone()).or_insert(lane);
            }
        }
        centres.push(at);
        Ok(())
    };
    load(start, &mut lanes)?;
    // A road lane (not a junction's) near the start, long enough to begin on.
    let mut candidates: Vec<&ExportLane> = lanes
        .values()
        .filter(|l| !l.id.contains(":prefab:") && length(l) > 40.0)
        .filter(|l| l.center[0].distance_xy(start) < 400.0)
        .collect();
    candidates.sort_by(|a, b| a.id.cmp(&b.id));
    ensure!(!candidates.is_empty(), "起点附近没有可用车道");
    let first =
        candidates[(rng.next_f64() * candidates.len() as f64) as usize % candidates.len()].clone();
    let rear = {
        // 10 m into the lane.
        let road = super::super::core::Road::path("start".into(), first.center.clone())?;
        road.sample(10.0).context("起点无法采样")?
    };
    // The walk: each lane with the successors not yet tried.
    // Lanes and nodes driven: a route through the same node twice (a
    // loop) is one the assist hands back.
    let mut visited: HashSet<String> = HashSet::from([first.id.clone()]);
    let mut nodes_seen: HashSet<u64> = HashSet::from([first.node_start, first.node_end]);
    let mut path: Vec<(ExportLane, Vec<String>)> = Vec::new();
    let mut driven = 0.0;
    let mut backtracks = 0;
    let mut current = Some(first.clone());
    while let Some(lane) = current.take() {
        driven += length(&lane);
        let end = *lane.center.last().context("车道几何为空")?;
        load(end, &mut lanes)?;
        let n = lane.center.len();
        let out = heading(lane.center[n - 2], lane.center[n - 1]);
        let onward = |l: &&ExportLane| wrap_angle(heading(l.center[0], l.center[1]) - out).abs();
        // Lanes linked on, or starting at the node this one ends at: the
        // route resolver joins lanes through their nodes, and the export
        // misses many links into junctions.
        let at_node: Vec<&String> = if lane.node_end == 0 {
            Vec::new()
        } else {
            lanes
                .values()
                .filter(|l| l.node_start == lane.node_end && l.id != lane.id)
                .map(|l| &l.id)
                .collect()
        };
        let mut options: Vec<String> = lane
            .successors
            .iter()
            .chain(at_node)
            .filter_map(|id| lanes.get(id))
            .filter(|l| onward(l) < U_TURN_RAD)
            .map(|l| l.id.clone())
            .collect();
        options.sort();
        options.dedup();
        // Shuffled: tried from the back.
        for i in (1..options.len()).rev() {
            let j = (rng.next_f64() * (i + 1) as f64) as usize % (i + 1);
            options.swap(i, j);
        }
        path.push((lane, options));
        if driven >= km * 1000.0 {
            break;
        }
        // On to an untried lane, backing out of dead ends.
        while let Some((lane, options)) = path.last_mut() {
            if let Some(id) = options.pop() {
                let node = lanes.get(&id).map_or(0, |l| l.node_end);
                if (node == 0 || !nodes_seen.contains(&node)) && visited.insert(id.clone()) {
                    nodes_seen.insert(node);
                    current = lanes.get(&id).cloned();
                    break;
                }
                continue;
            }
            driven -= length(lane);
            path.pop();
            backtracks += 1;
            ensure!(backtracks < 20_000, "随机路线回溯过多");
        }
    }
    ensure!(
        path.len() >= 3 && driven > 500.0,
        "随机路线太短（{:.0} m）",
        driven
    );
    let mut nodes: Vec<(u64, f64)> = Vec::new();
    let mut at = 0.0;
    for (i, (lane, _)) in path.iter().enumerate() {
        if i == 0 {
            nodes.push((lane.node_start, 0.0));
        }
        at += length(lane);
        if lane.node_end != 0 && nodes.last().is_none_or(|n| n.0 != lane.node_end) {
            nodes.push((lane.node_end, at));
        }
    }
    nodes.retain(|n| n.0 != 0);
    let total = at;
    let sdk = (
        rear.0.x + SDK_AHEAD_OF_REAR_M * rear.1.cos(),
        rear.0.y + SDK_AHEAD_OF_REAR_M * rear.1.sin(),
    );
    let mut text = String::new();
    writeln!(
        text,
        "# Random route (sim-route seed {seed}, {:.1} km, {backtracks} dead ends backed out of) over the exported map: node_uid distance_to_end_m time_to_end_s",
        total / 1000.0
    )?;
    writeln!(
        text,
        "# start sdk_pose {} {} {} z {} navigation_distance_m {}",
        sdk.0,
        sdk.1,
        rear.1,
        rear.0.z,
        total - 10.0
    )?;
    for (node, at) in nodes {
        writeln!(text, "{node} {:.1} {:.1}", total - at, (total - at) / 15.0)?;
    }
    Ok(text)
}
