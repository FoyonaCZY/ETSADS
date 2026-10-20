//! Read-only candidate -> shared gate geometry preview on installed-game data.
//! Unknown lane width/speed remain unknown; no driving graph is manufactured.
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    sector::{CacheLimits, SectorStore},
};
use etsads_msgs::Point3;
use etsads_runtime::controlled_route::gates::preview_source_candidate;
use std::{error::Error, fs};
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 5 {
        return Err("usage: gate_source_audit BUNDLE X Y REPORT.json".into());
    }
    let center = Point3 {
        x: args[2].parse()?,
        y: args[3].parse()?,
        z: 0.0,
    };
    let region = SectorStore::open(&args[1], CacheLimits::default())?.load_2km(center)?;
    let annotations = AnnotationStore::open(&args[1], AnnotationLimits::default())?
        .load_for_region(center, 2000.0, &region)?;
    let mut previews = Vec::new();
    let mut unsupported = Vec::new();
    for candidate in &annotations.gate_candidates {
        let lane = region
            .map
            .lanes
            .iter()
            .find(|l| l.id == candidate.source_lane_id)
            .ok_or("gate references missing actual source lane")?;
        assert!(!candidate.activation_verified);
        match preview_source_candidate(candidate, &lane.center) {
            Ok(preview) => previews.push(preview),
            Err(reason) => {
                unsupported.push(serde_json::json!({"candidate":candidate,"reason":reason}))
            }
        }
    }
    assert!(!previews.is_empty());
    let audit = region.map.audit();
    assert!(!audit.driveable);
    let report = serde_json::json!({"source":"installed_game_map_static_candidates","map_source_id":region.source_id,"annotations_source_id":annotations.source_id,
        "source_audit":audit,"previews":previews,"unsupported":unsupported,"widths_or_permissions_invented":false,
        "source_geometry_shared_with_runtime_gate_builder":true,"runtime_input_published":false,"phase_acceptance":false});
    fs::write(&args[4], serde_json::to_vec_pretty(&report)?)?;
    println!(
        "{} supported static gate previews; {} explicitly unsupported; source driveable=false",
        report["previews"].as_array().unwrap().len(),
        report["unsupported"].as_array().unwrap().len()
    );
    Ok(())
}
