use super::*;
use etsads_control::identification::{LongitudinalModel, SteeringRow, VehicleCalibration};
use etsads_map::{
    Lane,
    importer::{ImportedGraph, RegionGraph, RegionPolicy},
};
use etsads_msgs::{protocol::*, *};
use etsads_recorder::{Record, Recorder, RunMetadata};
use etsads_runtime::{DrivingMode, ExecutionMode, RuntimeConfig};

pub(super) fn fixture() -> (RegionGraph, RuntimeConfig, VehicleCalibration) {
    let graph = LaneGraph::new(vec![
        Lane {
            id: 1,
            center: vec![
                Point3 {
                    x: -200.0,
                    ..Default::default()
                },
                Point3 {
                    x: 100.0,
                    ..Default::default()
                },
            ],
            width: 4.0,
            speed_limit: 8.0,
            successors: vec![2],
            predecessors: vec![],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        },
        Lane {
            id: 2,
            center: vec![
                Point3 {
                    x: 100.0,
                    ..Default::default()
                },
                Point3 {
                    x: 500.0,
                    ..Default::default()
                },
            ],
            width: 4.0,
            speed_limit: 8.0,
            successors: vec![],
            predecessors: vec![1],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        },
    ])
    .unwrap();
    let region = RegionGraph {
        imported: ImportedGraph {
            graph,
            runtime_to_source: [(1, "fixture:road:1".into()), (2, "fixture:road:2".into())].into(),
            native_owners: [(1, 1), (2, 2)].into(),
        },
        policy: RegionPolicy {
            region_id: "synthetic".into(),
            evidence_path: "synthetic fixture".into(),
            verified_lane_ids: vec!["fixture:road:1".into(), "fixture:road:2".into()],
            allow_navigation_width: false,
            require_fresh_telemetry_speed_limit: true,
            maximum_speed_mps: 8.0,
            maximum_lateral_error_m: 0.8,
        },
        physical_boundaries_verified: true,
    };
    let geometry = VehicleGeometry {
        has_trailer: false,
        trailer_wheelbase: 0.0,
        trailer_length: 0.0,
        ..Default::default()
    };
    let cal = VehicleCalibration {
        schema_version: 1,
        vehicle_id: "fixture".into(),
        source_run: "synthetic recording fixture".into(),
        evidence: "synthetic".into(),
        geometry,
        geometry_fingerprint: None,
        steering_axis_source: "synthetic".into(),
        reference: None,
        conditions: None,
        front_bumper: None,
        envelope_measurement: None,
        sample_count: 100,
        steering_rows: [0.0, 15.0]
            .into_iter()
            .map(|speed| SteeringRow {
                speed,
                samples: (-20..=20)
                    .map(|i| {
                        let axis = i as f64 / 20.0;
                        (axis, (axis * 0.6).tan() / 3.8)
                    })
                    .collect(),
                counts: vec![100; 41],
                rms_curvature: 0.0,
            })
            .collect(),
        delay_s: Some(0.1),
        steering_tau_s: Some(0.2),
        delay_correlation: Some(1.0),
        longitudinal: Some(LongitudinalModel {
            coefficients: [0.0, 1.2, -4.0, 0.0, 0.0],
            training_rmse: 0.0,
            validation_rmse: 0.0,
            sample_count: 100,
        }),
        dynamic_speed_range_mps: Some([0.0, 15.0]),
        accepted_for_control: true,
        issues: vec![],
    };
    let config = RuntimeConfig {
        execution: ExecutionMode::Replay,
        vehicle_id: "fixture".into(),
        vehicle_configuration_verified: true,
        mode: DrivingMode::LaneFollow {
            route: vec![1, 2],
            maximum_speed_mps: 6.0,
        },
        forward_lateral: Default::default(),
        route_source: Default::default(),
    };
    (region, config, cal)
}
fn recording(path: &Path) {
    let mut recorder = Recorder::create(path, RunMetadata::new("synthetic", "fixture")).unwrap();
    for i in 0..1001u64 {
        let x = i as f64 * 0.16;
        let raw = TelemetryFrame {
            header: WireHeader {
                frame_id: i + 1,
                seq: i + 1,
                sim_time_us: i * 20_000,
                qpc: 1 + i * 20_000,
                ..Default::default()
            },
            qpc_frequency: 1_000_000,
            paused_simulation_time_us: i * 20_000,
            position_scs: [x, 0.0, 0.0],
            orientation_turns: [0.75, 0.0, 0.0],
            speed_mps: 8.0,
            acceleration_local: [0.0, 0.0, if (550..560).contains(&i) { 4.0 } else { 0.0 }],
            navigation_speed_limit_mps: 8.0,
            gear: 1,
            ..Default::default()
        };
        recorder
            .record("/telemetry", &Record::telemetry(&raw))
            .unwrap();
        let mut ext = Box::<ExtensionFrame>::default();
        ext.header = raw.header;
        ext.available = 10;
        let mut actor = ets2la::Traffic {
            position: [(x - 35.0) as f32, 0.0, 0.0],
            quaternion_wxyz: [
                std::f32::consts::FRAC_1_SQRT_2,
                0.0,
                -std::f32::consts::FRAC_1_SQRT_2,
                0.0,
            ],
            size: [2.0, 2.0, 4.0],
            speed: 10.0,
            acceleration: 0.0,
            trailer_count: 0,
            id: 7,
            is_tmp: 0,
            is_trailer: 0,
            trailers: [bytemuck::Zeroable::zeroed(); 3],
        };
        ext.traffic[..174].copy_from_slice(bytemuck::bytes_of(&actor));
        actor.position[0] = (x + 70.0) as f32;
        actor.speed = 3.0;
        actor.id = 8;
        ext.traffic[174..348].copy_from_slice(bytemuck::bytes_of(&actor));
        let signal = ets2la::Signal {
            position: [90.0, 0.0, 0.0],
            sector: [0, 0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            kind: 1,
            time_remaining: 10.0,
            state: 2,
            id: 4,
        };
        ext.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
        recorder
            .record(
                "/extension",
                &Record::wire(&*ext, i + 1, i * 20_000, 1 + i * 20_000),
            )
            .unwrap();
        let stack = serde_json::json!({"request_control":i<600,"header":{"frame_id":i+1}});
        recorder
            .record(
                "/stack",
                &Record {
                    schema_version: 1,
                    frame_id: i + 1,
                    sim_time_us: i * 20_000,
                    qpc: 1 + i * 20_000,
                    raw_payload: serde_json::to_vec(&stack).unwrap(),
                    ..Default::default()
                },
            )
            .unwrap();
    }
    recorder.finish().unwrap();
}
fn temporary() -> std::path::PathBuf {
    std::env::temp_dir().join(format!(
        "etsads-scenarios-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ))
}
#[test]
fn metric_baseline_binds_source_window_map_and_model_and_rejects_failed_runs() {
    let (region, config, cal) = fixture();
    let metrics: BTreeMap<String, CaseMetrics> =
        serde_json::from_str(include_str!("../../testdata/recording-baseline.json")).unwrap();
    let (id, value) = metrics.first_key_value().unwrap();
    let case = RecordedCase {
        id: id.clone(),
        trigger: Trigger::HardBraking,
        trigger_frame: 501,
        trigger_sim_time_us: 10_000_000,
        start_frame: 1,
        start_sim_time_us: 0,
        end_frame: 751,
        end_sim_time_us: 15_000_000,
        complete_pre_window: true,
        complete_post_window: true,
    };
    let library = ScenarioLibrary {
        schema_version: 1,
        source: "synthetic".into(),
        source_hashes: [("meta.json".into(), "synthetic source fingerprint".into())].into(),
        map_sha256: "synthetic map fingerprint".into(),
        settings: Default::default(),
        cases: vec![case],
        phase_acceptance: false,
        notes: vec![],
    };
    let identity = BaselineIdentity::new(&library, &config, &cal, &region.policy).unwrap();
    let mut value = value.clone();
    value.simulated_seconds = 15.0;
    let document = BaselineDocument {
        schema_version: 1,
        identity: identity.clone(),
        completed: true,
        metrics: [(id.clone(), value)].into(),
    };
    document.validate_for(&identity).unwrap();
    for kind in 0..6 {
        let mut other = identity.clone();
        match kind {
            0 => {
                other
                    .source_hashes
                    .insert("meta.json".into(), "changed".into());
            }
            1 => other.map_sha256 = "changed".into(),
            2 => other.cases[0].start_frame += 1,
            3 => other.calibration_sha256 = "changed".into(),
            4 => other.runtime_configuration_sha256 = "changed".into(),
            _ => other.region_policy_sha256 = "changed".into(),
        }
        assert!(document.validate_for(&other).is_err());
    }
    let mut failed = document.clone();
    failed.metrics.get_mut(id).unwrap().fault = Some("fault".into());
    assert!(failed.validate_for(&identity).is_err());
    let mut truncated = document.clone();
    truncated.metrics.get_mut(id).unwrap().simulated_seconds = 2.0;
    assert!(truncated.validate_for(&identity).is_err());
    let mut incomplete = document;
    incomplete.completed = false;
    assert!(incomplete.validate_for(&identity).is_err());
}
#[test]
fn full_recorded_library_reacts_to_ego_and_preserves_provenance() {
    let retained = std::env::var_os("ETSADS_SCENARIO_FIXTURE_DIR");
    let root = retained
        .as_ref()
        .map(std::path::PathBuf::from)
        .unwrap_or_else(temporary);
    assert!(!root.exists(), "fixture output must be new");
    let run = root.join("run");
    recording(&run);
    let (region, config, cal) = fixture();
    let library = extract(
        &run,
        &region.imported.graph,
        ExtractionSettings {
            roundabout_lane_ids: vec![2],
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(library.source, "synthetic");
    assert!(!library.phase_acceptance);
    assert_eq!(
        library
            .cases
            .iter()
            .map(|c| c.trigger)
            .collect::<BTreeSet<_>>(),
        [
            Trigger::Takeover,
            Trigger::HardBraking,
            Trigger::RedLightApproach,
            Trigger::Roundabout
        ]
        .into()
    );
    assert!(
        library
            .cases
            .iter()
            .filter(|c| c.trigger != Trigger::RedLightApproach)
            .all(|c| c.complete_pre_window)
    );
    let mut reports = vec![];
    let baseline: BTreeMap<String, CaseMetrics> =
        serde_json::from_str(include_str!("../../testdata/recording-baseline.json")).unwrap();
    assert_eq!(baseline.len(), library.cases.len());
    for case in &library.cases {
        let mut seen = 0;
        let report = run_case(
            &run,
            &library,
            case,
            config.clone(),
            cal.clone(),
            region.clone(),
            |step| {
                seen += 1;
                assert!(step.agents.iter().all(Agent::valid));
                Ok(())
            },
        )
        .unwrap();
        assert!(report.completed, "{}: {:?}", case.id, report.metrics.fault);
        assert!(seen > 500);
        assert_eq!(report.metrics.collision_frames, 0);
        assert!(report.metrics.reactive_ego_braking_steps > 0);
        assert!(report.metrics.distance_m > 20.0);
        assert!(!report.phase_acceptance && !report.control_written);
        check_regression(&report.metrics, &report.metrics).unwrap();
        check_regression(&baseline[&case.id], &report.metrics).unwrap();
        let mut worse = report.metrics.clone();
        worse.collision_frames += 1;
        assert!(check_regression(&report.metrics, &worse).is_err());
        reports.push(report);
    }
    // Each revision exercises the complete generated library. Preserve an
    // explicit artifact when requested by CI/local validation.
    if let Ok(output) = std::env::var("ETSADS_SCENARIO_TEST_REPORT") {
        std::fs::write(output, serde_json::to_vec_pretty(&reports).unwrap()).unwrap();
    }
    if retained.is_some() {
        use etsads_map::importer::{ExportLane, ExportedMap};
        let exported = ExportedMap {
            schema_version: 1,
            accepted_for_autonomous_driving: false,
            source: "synthetic scenario regression fixture".into(),
            lanes: region
                .imported
                .graph
                .lanes
                .values()
                .map(|lane| ExportLane {
                    id: region.imported.runtime_to_source[&lane.id].clone(),
                    owner_uid: lane.id,
                    node_start: lane.id,
                    node_end: lane.id + 1,
                    center: lane.center.clone(),
                    width: Some(lane.width),
                    speed_limit: Some(lane.speed_limit),
                    geometry_status: "synthetic fixture".into(),
                    centerline_validated: true,
                    physical_boundaries_verified: true,
                    truck_access: Some(true),
                    successors: lane
                        .successors
                        .iter()
                        .map(|id| region.imported.runtime_to_source[id].clone())
                        .collect(),
                    predecessors: lane
                        .predecessors
                        .iter()
                        .map(|id| region.imported.runtime_to_source[id].clone())
                        .collect(),
                    ..Default::default()
                })
                .collect(),
        };
        for (name, value) in [
            ("map.json", serde_json::to_value(exported).unwrap()),
            ("policy.json", serde_json::to_value(&region.policy).unwrap()),
            ("config.json", serde_json::to_value(&config).unwrap()),
            ("calibration.json", serde_json::to_value(&cal).unwrap()),
            (
                "settings.json",
                serde_json::to_value(&library.settings).unwrap(),
            ),
            ("library.json", serde_json::to_value(&library).unwrap()),
        ] {
            std::fs::write(root.join(name), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
        }
    }
    let mut changed = library.clone();
    changed.source = "game".into();
    assert!(
        run_case(
            &run,
            &changed,
            &changed.cases[0],
            config.clone(),
            cal.clone(),
            region.clone(),
            |_| Ok(())
        )
        .is_err()
    );
    changed = library.clone();
    changed
        .source_hashes
        .insert("meta.json".into(), "0".repeat(64));
    assert!(
        run_case(
            &run,
            &changed,
            &changed.cases[0],
            config.clone(),
            cal.clone(),
            region.clone(),
            |_| Ok(())
        )
        .is_err()
    );
    let mut live = config;
    live.execution = ExecutionMode::Live;
    assert!(
        run_case(
            &run,
            &library,
            &library.cases[0],
            live,
            cal,
            region,
            |_| Ok(())
        )
        .is_err()
    );
    if retained.is_none() {
        std::fs::remove_dir_all(root).unwrap();
    }
}
