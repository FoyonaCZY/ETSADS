//! Real-data equality and logical I/O evidence. No game or IPC access.
use etsads_map::sector::{CacheLimits, SectorStore};
use etsads_msgs::Point3;
use std::{error::Error, fs, time::Duration};
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 7 {
        return Err("usage: sector_benchmark LEGACY PARTITION X Y RADIUS REPORT.json".into());
    }
    let center = Point3 {
        x: args[3].parse()?,
        y: args[4].parse()?,
        z: 0.0,
    };
    let radius = args[5].parse()?;
    let limits = CacheLimits::default();
    let store = SectorStore::open(&args[2], limits.clone())?;
    let cold = store.load_region(center, radius)?;
    eprintln!(
        "partition cold: {} lanes, {} points, {:.1} ms",
        cold.stats.returned_lanes, cold.stats.returned_points, cold.stats.elapsed_ms
    );
    let warm = store.load_region(center, radius)?;
    assert_eq!(cold.map.lanes, warm.map.lanes);
    assert_eq!(warm.stats.compressed_bytes_read, 0);
    let legacy = SectorStore::open(&args[1], limits.clone())?.load_region(center, radius)?;
    assert_eq!(cold.map.lanes.len(), legacy.map.lanes.len());
    for (a, b) in cold.map.lanes.iter().zip(&legacy.map.lanes) {
        assert_eq!(a, b, "lane {} differs", a.id);
    }
    assert_eq!(cold.map.audit(), legacy.map.audit());
    let fresh = SectorStore::open(&args[2], limits)?;
    let ticket = fresh.prefetch_2km(center)?;
    let prefetch = loop {
        if let Some(stats) = fresh.wait_prefetch(ticket, Duration::from_secs(30))? {
            break stats;
        }
    };
    let prefetched = fresh.load_2km(center)?;
    assert_eq!(prefetched.stats.compressed_bytes_read, 0);
    let report = serde_json::json!({
        "source":"existing_installed_game_export", "center":center, "radius_m":radius,
        "lane_fields_and_points_exactly_equal":true,"audit_exactly_equal":true,
        "audit":cold.map.audit(), "legacy":legacy.stats,"cold":cold.stats,"warm":warm.stats,
        "prefetch_2km":prefetch,"after_prefetch_2km":prefetched.stats,
        "byte_measurement":"logical compressed bytes delivered to readers; excludes index/manifest and OS disk cache effects",
        "cache_budget_excludes":"caller-owned result maps and decoder bounded scratch space",
        "game_acceptance":false
    });
    let text = serde_json::to_string_pretty(&report)?;
    fs::write(&args[6], &text)?;
    println!("{text}");
    Ok(())
}
