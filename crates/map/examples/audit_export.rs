use etsads_map::importer::{ExportedMap, load_parquet_region};
use etsads_msgs::Point3;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = std::env::args()
        .nth(1)
        .ok_or("usage: audit_export EXPORT_DIRECTORY")?;
    let json = ExportedMap::from_path(std::path::Path::new(&directory).join("runtime-map.json"))?;
    let first = json
        .lanes
        .first()
        .and_then(|l| l.center.first())
        .copied()
        .ok_or("empty map")?;
    let parquet = load_parquet_region(
        &directory,
        Point3 {
            x: first.x,
            y: first.y,
            z: first.z,
        },
        5000.0,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"runtime_json":json.audit(),"parquet_region":parquet.audit()})
        )?
    );
    Ok(())
}
