//! Generate an explicitly map-derived route fixture, never a claimed observed
//! ETS2LA navigation route. Useful when the real recording contains no route.
use etsads_map::{
    corridor::{CorridorPolicy, NativeRouteIndex},
    sector::{CacheLimits, SectorStore},
};
use etsads_msgs::Point3;
use std::{error::Error, fs};
fn main() -> Result<(), Box<dyn Error>> {
    let a: Vec<_> = std::env::args().collect();
    if a.len() != 5 {
        return Err("usage: native_corridor_probe MAP X Y REPORT.json".into());
    }
    let loaded = SectorStore::open(&a[1], CacheLimits::default())?.load_2km(Point3 {
        x: a[2].parse()?,
        y: a[3].parse()?,
        z: 0.0,
    })?;
    let index = NativeRouteIndex::from_exported(&loaded.map)?;
    let mut found = None;
    'starts: for first in index.lanes.values().filter(|l| {
        l.node_start != 0
            && l.node_end != 0
            && l.truck_access == Some(true)
            && l.centerline_validated
    }) {
        let mut stack = vec![(
            first.source_lane_id.clone(),
            vec![first.node_start],
            Vec::<String>::new(),
        )];
        while let Some((id, mut route, mut visited)) = stack.pop() {
            let lane = &index.lanes[&id];
            if visited.contains(&id) || visited.len() > 10 {
                continue;
            }
            visited.push(id.clone());
            for node in [lane.node_start, lane.node_end] {
                if node != 0 && route.last() != Some(&node) {
                    route.push(node);
                }
            }
            if route.len() >= 4
                && let Ok(corridor) = index.resolve(
                    &route,
                    &first.source_lane_id,
                    0.0,
                    &CorridorPolicy::default(),
                )
                && corridor.steps.len() >= 3
            {
                found = Some(corridor);
                break 'starts;
            }
            for successor in &lane.successors {
                let n = &index.lanes[successor];
                if n.truck_access == Some(true) && n.centerline_validated {
                    stack.push((successor.clone(), route.clone(), visited.clone()));
                }
            }
        }
    }
    let corridor = found.ok_or("no suitable real-map corridor fixture")?;
    let report = serde_json::json!({"fixture_source":"ordered native endpoints derived from existing real map; NOT an observed game navigation route",
        "source_id":loaded.source_id,"source_map":a[1],"game_navigation_acceptance":false,"driving_acceptance":false,
        "native_corridor":corridor});
    fs::write(&a[4], serde_json::to_vec_pretty(&report)?)?;
    println!(
        "saved real-map topology fixture: {} steps, {} ordered nodes",
        report["native_corridor"]["steps"].as_array().unwrap().len(),
        report["native_corridor"]["ordered_node_uids"]
            .as_array()
            .unwrap()
            .len()
    );
    Ok(())
}
