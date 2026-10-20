//! Tiny synthetic storage fixture. Its manifest explicitly denies game use.
use flate2::{Compression, write::GzEncoder};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, io::Write, path::Path};
fn gzip(root: &Path, path: &str, bytes: &[u8]) -> Result<Value, Box<dyn Error>> {
    let mut enc = GzEncoder::new(Vec::new(), Compression::fast());
    enc.write_all(bytes)?;
    let data = enc.finish()?;
    fs::write(root.join(path), &data)?;
    Ok(
        json!({"path":path,"bytes":data.len(),"uncompressed_bytes":bytes.len(),"sha256":format!("{:x}",Sha256::digest(&data))}),
    )
}
fn main() -> Result<(), Box<dyn Error>> {
    let arg = std::env::args().nth(1).ok_or("output directory required")?;
    let root = Path::new(&arg);
    fs::create_dir(root)?;
    fs::create_dir(root.join("sectors"))?;
    fs::create_dir(root.join("traffic"))?;
    let manifest = json!({"schema_version":1,"coordinate_system":"ENU","units":"metres","source":"synthetic_test_only",
        "game_version":"SYNTHETIC_NOT_A_GAME_VERSION","cache_key":"synthetic-navigation","accepted_for_autonomous_driving":false});
    let manifest = serde_json::to_vec_pretty(&manifest)?;
    fs::write(root.join("manifest.json"), &manifest)?;
    let manifest_hash = format!("{:x}", Sha256::digest(&manifest));
    let mut lanes = Vec::new();
    let mut traffic = Vec::new();
    for (i, (id, x, end, next, prev)) in [
        ("a", 0.0, 1800.0, Some("m"), None),
        ("m", 1800.0, 4200.0, Some("z"), Some("a")),
        ("z", 4200.0, 6000.0, None, Some("m")),
    ]
    .into_iter()
    .enumerate()
    {
        let lane = json!({"id":id,"owner_uid":i+1,"node_start":i+1,"node_end":i+2,
            "center":[{"x":x,"y":0.0,"z":0.0},{"x":end,"y":0.0,"z":0.0}],"width":4.5,"speed_limit":5.0,"navigation_width":4.5,
            "geometry_status":"synthetic_test_only","centerline_validated":true,"physical_boundaries_verified":true,"truck_access":true,
            "successors":next.into_iter().collect::<Vec<_>>(),"predecessors":prev.into_iter().collect::<Vec<_>>()});
        let bounds = json!([x, 0.0, end, 0.0]);
        let mut line = serde_json::to_vec(&json!({"bounds":bounds,"lane":lane}))?;
        line.push(b'\n');
        let mut descriptor = gzip(root, &format!("sectors/{i}.jsonl.gz"), &line)?;
        descriptor["sector_x"] = i.into();
        descriptor["sector_z"] = 0.into();
        descriptor["bounds"] = bounds.clone();
        descriptor["lane_count"] = 1.into();
        descriptor["point_count"] = 2.into();
        lanes.push(descriptor);
        let context = json!({"schema_version":1,"source_lane_ids":[id],"reference_only_lane_ids":[],"annotations_present":true,
            "conflicts":[],"lane_controls":[],"signals":[],"signal_lanes":[],"stop_lines":[]});
        let mut descriptor = gzip(
            root,
            &format!("traffic/{i}.json.gz"),
            &serde_json::to_vec(&context)?,
        )?;
        descriptor["bounds"] = bounds;
        descriptor["sector_x"] = i.into();
        descriptor["sector_z"] = 0.into();
        traffic.push(descriptor);
    }
    let total: u64 = lanes.iter().map(|s| s["bytes"].as_u64().unwrap()).sum();
    let lane_index = serde_json::to_vec_pretty(
        &json!({"schema_version":1,"format":"etsads_sector_lanes_jsonl_gzip_v1","coordinate_system":"ENU","units":"metres",
        "source":"synthetic_test_only","source_manifest_sha256":manifest_hash,"source_cache_key":"synthetic-navigation","accepted_for_autonomous_driving":false,
        "lane_count":3,"point_count":6,"compressed_bytes":total,"shards":lanes}),
    )?;
    let lane_hash = format!("{:x}", Sha256::digest(&lane_index));
    fs::write(root.join("sector_index.json"), lane_index)?;
    let native = json!({"schema_version":1,"source_manifest_sha256":manifest_hash,"source_cache_key":"synthetic-navigation","topology_only":true,
        "nodes":[{"uid":1,"next":[2]},{"uid":2,"next":[3]},{"uid":3,"next":[4]},{"uid":4,"next":[]}]});
    let native_descriptor = gzip(
        root,
        "native_route_index.json.gz",
        &serde_json::to_vec(&native)?,
    )?;
    fs::write(
        root.join("traffic_index.json"),
        serde_json::to_vec_pretty(
            &json!({"schema_version":1,"format":"etsads_sector_traffic_json_gzip_v1","coordinate_system":"ENU","units":"metres",
        "source_manifest_sha256":manifest_hash,"lane_index_sha256":lane_hash,"source_cache_key":"synthetic-navigation","present_tables":["conflict_pairs","lane_controls","signals","signal_lanes","stop_lines"],
        "missing_tables":[],"shards":traffic,"native_route_index":native_descriptor,"accepted_for_autonomous_driving":false}),
        )?,
    )?;
    fs::write(
        root.join("README.txt"),
        "SYNTHETIC TEST ONLY. Not measured/authorized for game driving. Query (0,0), radius 2000 gives [a,m]; query (6000,0) gives [m,z]. Runtime m changes 2 -> 1. Native route 1,2,3,4 spans both windows. No game acceptance or evidence token is supplied.\n",
    )?;
    println!("{}", root.display());
    Ok(())
}
