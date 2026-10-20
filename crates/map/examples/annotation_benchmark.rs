//! Compare actual normalized Parquet annotations with their portable shards.
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    corridor::{CorridorPolicy, GlobalNativeRouteIndex, NativeRouteIndex},
    sector::{CacheLimits, SectorStore},
};
use etsads_msgs::Point3;
use sha2::{Digest, Sha256};
use std::{error::Error, fs, time::Instant};
fn main() -> Result<(), Box<dyn Error>> {
    let a: Vec<_> = std::env::args().collect();
    if a.len() != 6 {
        return Err("usage: annotation_benchmark PARQUET SECTORS X Y REPORT.json".into());
    }
    let center = Point3 {
        x: a[3].parse()?,
        y: a[4].parse()?,
        z: 0.0,
    };
    let loaded = SectorStore::open(&a[2], CacheLimits::default())?.load_2km(center)?;
    let started = Instant::now();
    let global = GlobalNativeRouteIndex::load_source(&a[2])?;
    let global_ms = started.elapsed().as_secs_f64() * 1000.0;
    assert_eq!(
        global.source_manifest_sha256,
        format!(
            "{:x}",
            Sha256::digest(fs::read(std::path::Path::new(&a[1]).join("manifest.json"))?)
        )
    );
    let sector = AnnotationStore::open(&a[2], AnnotationLimits::default())?;
    let started = Instant::now();
    let cold = sector.load_for_region(center, 2000.0, &loaded)?;
    let cold_ms = started.elapsed().as_secs_f64() * 1000.0;
    let started = Instant::now();
    let warm = sector.load_for_region(center, 2000.0, &loaded)?;
    let warm_ms = started.elapsed().as_secs_f64() * 1000.0;
    assert_eq!(warm.stats.compressed_bytes_read, 0);
    assert_eq!(
        serde_json::to_value(&cold.traffic)?,
        serde_json::to_value(&warm.traffic)?
    );
    let legacy = AnnotationStore::open(&a[1], AnnotationLimits::default())?;
    let started = Instant::now();
    let old = legacy.load_region(center, 2000.0, &loaded.map)?;
    let old_ms = started.elapsed().as_secs_f64() * 1000.0;
    assert_eq!(
        serde_json::to_value(&cold.traffic)?,
        serde_json::to_value(&old.traffic)?
    );
    assert_eq!(
        serde_json::to_value(&cold.gate_candidates)?,
        serde_json::to_value(&old.gate_candidates)?
    );
    // A source-derived route fixture exercises the real topology without
    // claiming the previously empty game GPS route was populated.
    let index = NativeRouteIndex::from_region(&loaded)?;
    let fixture: serde_json::Value =
        serde_json::from_slice(&fs::read("artifacts/map-native-corridor-tromso.json")?)?;
    let route: Vec<u64> =
        serde_json::from_value(fixture["native_corridor"]["ordered_node_uids"].clone())?;
    let first = fixture["native_corridor"]["steps"][0]["source_lane_id"]
        .as_str()
        .unwrap();
    let local =
        index.resolve_local_prefix(&global, &route, first, 0.0, &CorridorPolicy::default())?;
    let report = serde_json::json!({"source":"existing_installed_game_export","metadata_exactly_equal":true,"gate_candidates_exactly_equal":true,
        "counts":{"conflicts":cold.traffic.conflicts.len(),"controls":cold.traffic.lane_controls.len(),"signals":cold.traffic.signals.len(),"links":cold.traffic.signal_lanes.len(),"stops":cold.traffic.stop_lines.len(),"gate_candidates":cold.gate_candidates.len(),"reference_only_lanes":cold.traffic.reference_only_lane_ids.len()},
        "cold":cold.stats,"cold_ms":cold_ms,"warm":warm.stats,"warm_ms":warm_ms,"legacy_ms":old_ms,
        "global_native_nodes":global.node_count(),"global_load_ms":global_ms,"source_manifest_sha256":global.source_manifest_sha256,
        "source_derived_local_corridor":local,"actual_game_gps_observed":false,"driving_acceptance":false});
    fs::write(&a[5], serde_json::to_vec_pretty(&report)?)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
