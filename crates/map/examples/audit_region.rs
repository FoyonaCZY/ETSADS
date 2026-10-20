use etsads_map::importer::{ExportedMap, RegionPolicy};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: audit_region RUNTIME_MAP_JSON")?;
    let mut map = ExportedMap::from_path(&path)?;
    let audit = map.audit();
    map.lanes.retain(|l| {
        l.centerline_validated && l.navigation_width.is_some() && l.truck_access == Some(true)
    });
    let ids = map.lanes.iter().map(|l| l.id.clone()).collect();
    let region = map.into_commissioning_region(
        RegionPolicy {
            region_id: "OFFLINE_ADAPTER_TEST_ONLY".into(),
            evidence_path: path,
            verified_lane_ids: ids,
            allow_navigation_width: true,
            require_fresh_telemetry_speed_limit: true,
            maximum_speed_mps: 6.0,
            maximum_lateral_error_m: 0.6,
        },
        20.0,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"driving_authorized":false,"original_audit":audit,"offline_adapter_lanes":region.imported.graph.lanes.len(),"physical_boundaries_verified":region.physical_boundaries_verified,"maximum_speed_mps":region.policy.maximum_speed_mps})
        )?
    );
    Ok(())
}
