//! All data in navigation-sectors is explicitly synthetic, not game evidence.
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    corridor::{CorridorError, CorridorPolicy, GlobalNativeRouteIndex, NativeRouteIndex},
    importer::RegionPolicy,
    sector::{CacheLimits, SectorStore},
};
use etsads_msgs::Point3;
fn fixture() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("testdata/navigation-sectors")
}
fn policy() -> RegionPolicy {
    RegionPolicy {
        region_id: "synthetic_test_only".into(),
        evidence_path: "synthetic_fixture_not_game_evidence".into(),
        verified_lane_ids: vec!["a".into(), "m".into(), "z".into()],
        allow_navigation_width: false,
        require_fresh_telemetry_speed_limit: true,
        maximum_speed_mps: 5.0,
        maximum_lateral_error_m: 0.5,
    }
}
#[test]
fn moving_windows_keep_source_identity_and_rebind_runtime_ids() {
    let source = fixture();
    let store = SectorStore::open(&source, CacheLimits::default()).unwrap();
    let metadata = AnnotationStore::open(&source, AnnotationLimits::default()).unwrap();
    let global = GlobalNativeRouteIndex::load_source(&source).unwrap();
    assert_eq!(global.node_count(), 4);
    let first = store.load_2km(Point3::default()).unwrap();
    let second = store
        .load_2km(Point3 {
            x: 6000.0,
            y: 0.0,
            z: 0.0,
        })
        .unwrap();
    assert_eq!(first.source_id, second.source_id);
    let p = policy();
    let first_policy = p.for_loaded_map(&first.map).unwrap();
    assert_eq!(first_policy.verified_lane_ids, vec!["a", "m"]);
    assert_eq!(first_policy.evidence_path, p.evidence_path);
    assert_eq!(first_policy.maximum_speed_mps, p.maximum_speed_mps);
    let first_graph = first
        .map
        .clone()
        .into_commissioning_region(first_policy, 5.0)
        .unwrap();
    let second_graph = second
        .map
        .clone()
        .into_commissioning_region(p.for_loaded_map(&second.map).unwrap(), 5.0)
        .unwrap();
    assert_eq!(first_graph.imported.runtime_to_source[&2], "m");
    assert_eq!(second_graph.imported.runtime_to_source[&1], "m");
    let native = NativeRouteIndex::from_region(&first)
        .unwrap()
        .for_region_graph(&first_graph)
        .unwrap();
    let local = native
        .resolve_local_prefix(&global, &[1, 2, 3, 4], "a", 0.0, &CorridorPolicy::default())
        .unwrap();
    assert_eq!(local.next_boundary_checkpoint, Some(4));
    assert_eq!(local.remaining_node_uids, vec![4]);
    assert!(local.must_stop_at_local_terminal);
    assert_eq!(
        local
            .corridor
            .bind(&native, &first_graph.imported)
            .unwrap()
            .runtime_lane_ids,
        vec![1, 2]
    );
    let native = NativeRouteIndex::from_region(&second)
        .unwrap()
        .for_region_graph(&second_graph)
        .unwrap();
    let local = native
        .resolve_local_prefix(&global, &[2, 3, 4], "m", 1000.0, &CorridorPolicy::default())
        .unwrap();
    assert!(!local.must_stop_at_local_terminal);
    assert_eq!(local.next_boundary_checkpoint, None);
    assert_eq!(
        local
            .corridor
            .bind(&native, &second_graph.imported)
            .unwrap()
            .runtime_lane_ids,
        vec![1, 2]
    );
    let a = metadata
        .load_for_region(Point3::default(), 2000.0, &first)
        .unwrap();
    assert!(
        a.traffic
            .has_annotations_for_route(&["a".into(), "m".into()])
    );
    let a2 = metadata
        .load_for_region(Point3::default(), 2000.0, &first)
        .unwrap();
    assert_eq!(a2.stats.compressed_bytes_read, 0);
    assert_eq!(a.source_id, a2.source_id);
}
#[test]
fn unknown_remote_disconnected_and_unapproved_are_distinct() {
    let source = fixture();
    let store = SectorStore::open(&source, CacheLimits::default()).unwrap();
    let global = GlobalNativeRouteIndex::load_source(&source).unwrap();
    let loaded = store.load_2km(Point3::default()).unwrap();
    let native = NativeRouteIndex::from_region(&loaded).unwrap();
    // An unknown node ends the route there: the known prefix is driven and
    // the truck must stop before it. One unknown from the start is refused.
    let cut = native
        .resolve_local_prefix(&global, &[1, 2, 99], "a", 0.0, &CorridorPolicy::default())
        .unwrap();
    assert_eq!(cut.next_boundary_checkpoint, Some(99));
    assert!(cut.must_stop_at_local_terminal);
    assert_eq!(cut.remaining_node_uids.last(), Some(&99));
    assert_eq!(
        native
            .resolve_local_prefix(&global, &[99, 1, 2], "a", 0.0, &CorridorPolicy::default())
            .unwrap_err(),
        CorridorError::UnknownNativeUid { node_uid: 99 }
    );
    assert_eq!(
        native
            .resolve_local_prefix(&global, &[1, 3], "a", 0.0, &CorridorPolicy::default())
            .unwrap_err(),
        CorridorError::DisconnectedNativeNodes { from: 1, to: 3 }
    );
    let mut p = policy();
    p.verified_lane_ids = vec!["a".into()];
    let region = loaded
        .map
        .clone()
        .into_commissioning_region(p, 5.0)
        .unwrap();
    let subset = native.for_region_graph(&region).unwrap();
    assert_eq!(subset.lanes.len(), 1);
    assert!(subset.lanes["a"].successors.is_empty());
    assert_eq!(
        subset
            .resolve_local_prefix(&global, &[1, 2, 3, 4], "a", 0.0, &CorridorPolicy::default())
            .unwrap_err(),
        CorridorError::UnauthorizedNativeCheckpoint { node_uid: 3 }
    );
    let mut mixed = native.clone();
    mixed.source_id = Some("incorrect_source".into());
    assert!(matches!(
        mixed.resolve_local_prefix(&global, &[1, 2], "a", 0.0, &CorridorPolicy::default()),
        Err(CorridorError::Invalid(_))
    ));
    let mut wrong = loaded.clone();
    wrong.source_id = "incorrect_source".into();
    assert!(
        AnnotationStore::open(&source, AnnotationLimits::default())
            .unwrap()
            .load_for_region(Point3::default(), 2000.0, &wrong)
            .is_err()
    );
    p = policy();
    p.verified_lane_ids = vec!["not_in_map".into()];
    assert!(p.for_loaded_map(&loaded.map).is_err());
}
