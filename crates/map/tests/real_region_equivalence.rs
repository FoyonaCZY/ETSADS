//! An explicit installed-game regression, kept separate from portable unit
//! fixtures. Never treats map parity as autonomous-driving acceptance.
use etsads_map::importer::{ExportedMap, load_parquet_region};
use etsads_msgs::Point3;
use std::{collections::BTreeMap, path::PathBuf};

#[test]
#[ignore = "set ETSADS_PARQUET_EXPORT and ETSADS_REGION_SLICE to a real source export and its freshly generated JSON slice"]
fn real_json_slice_and_parquet_region_have_identical_driving_graphs() {
    let source =
        PathBuf::from(std::env::var("ETSADS_PARQUET_EXPORT").expect("source export required"));
    let slice =
        PathBuf::from(std::env::var("ETSADS_REGION_SLICE").expect("JSON slice directory required"));
    let manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(slice.join("manifest.json")).unwrap()).unwrap();
    let center = Point3 {
        x: manifest["center"]["x"].as_f64().unwrap(),
        y: manifest["center"]["y"].as_f64().unwrap(),
        z: manifest["center"]["z"].as_f64().unwrap_or(0.0),
    };
    let radius = manifest["radius_m"].as_f64().unwrap();
    let json_path = slice.join("runtime-map.json");
    let original: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&json_path).unwrap()).unwrap();
    // An interior-only fixture cannot exercise the reported clipping defect.
    let boundary_links = original["boundary_links"]
        .as_array()
        .expect("new slice boundary provenance missing")
        .len();
    assert!(
        boundary_links > 0,
        "fixture must include links leaving the loaded region"
    );
    let boundary_neighbors = original["boundary_neighbors"]
        .as_array()
        .expect("new slice boundary neighbor provenance missing")
        .len();
    let json = ExportedMap::from_path(&json_path).unwrap();
    let parquet = load_parquet_region(&source, center, radius).unwrap();
    let json_audit = json.audit();
    let parquet_audit = parquet.audit();
    assert_eq!(
        json_audit, parquet_audit,
        "loading paths disagree about map qualification"
    );
    assert_eq!(json_audit.unknown_links, 0);
    assert_eq!(json_audit.invalid_neighbors, 0);
    assert!(
        !json_audit.driveable,
        "real unaccepted map must remain gated"
    );
    assert!(!json.accepted_for_autonomous_driving);
    assert!(!parquet.accepted_for_autonomous_driving);
    let normalize = |map: ExportedMap| -> BTreeMap<_, _> {
        map.lanes
            .into_iter()
            .map(|mut lane| {
                lane.successors.sort();
                lane.predecessors.sort();
                (lane.id.clone(), lane)
            })
            .collect()
    };
    let json_lanes = normalize(json);
    let parquet_lanes = normalize(parquet);
    assert_eq!(
        json_lanes.keys().collect::<Vec<_>>(),
        parquet_lanes.keys().collect::<Vec<_>>()
    );
    let mut neighbors = 0;
    let mut permitted = 0;
    for (id, lane) in &json_lanes {
        let mut json_metadata = lane.clone();
        let mut parquet_metadata = parquet_lanes[id].clone();
        assert_eq!(
            json_metadata.center.len(),
            parquet_metadata.center.len(),
            "lane {id} point count"
        );
        for (index, (left, right)) in json_metadata
            .center
            .iter()
            .zip(&parquet_metadata.center)
            .enumerate()
        {
            assert_eq!(left, right, "lane {id} point {index} differs");
        }
        json_metadata.center.clear();
        parquet_metadata.center.clear();
        assert_eq!(
            json_metadata, parquet_metadata,
            "source lane {id} metadata differs"
        );
        neighbors += usize::from(lane.left.is_some()) + usize::from(lane.right.is_some());
        permitted += usize::from(lane.can_change_left) + usize::from(lane.can_change_right);
        assert!(
            !lane.physical_boundaries_verified,
            "slice must not invent physical boundary evidence"
        );
    }
    assert!(
        neighbors > 0 || boundary_neighbors > 0,
        "fixture must exercise actual neighboring lanes"
    );
    eprintln!(
        "installed-game parity: lanes={} points={} internal_neighbors={} permitted_native_changes={} clipped_boundary_links={} boundary_neighbors={} audit_driveable={}",
        json_lanes.len(),
        json_audit.point_count,
        neighbors,
        permitted,
        boundary_links,
        boundary_neighbors,
        json_audit.driveable
    );
}
