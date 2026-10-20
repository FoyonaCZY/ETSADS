use super::*;
use bytemuck::Zeroable;
use etsads_map::{
    annotations::GateCandidate,
    corridor::{CorridorPolicy, NativeRouteIndex},
    importer::{ExportLane, ExportedMap},
};
use etsads_msgs::{Pose2, VehicleGeometry};
use gates::{GateMode, GatePlane, derive_native_gates};
fn source_map() -> ExportedMap {
    ExportedMap {
        schema_version: 1,
        accepted_for_autonomous_driving: true,
        source: "synthetic gate test only".into(),
        lanes: vec![ExportLane {
            id: "a".into(),
            owner_uid: 7,
            node_start: 1,
            node_end: 2,
            center: vec![
                Point3::default(),
                Point3 {
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            width: Some(4.5),
            speed_limit: Some(8.0),
            centerline_validated: true,
            physical_boundaries_verified: true,
            truck_access: Some(true),
            geometry_status: "synthetic".into(),
            ..Default::default()
        }],
    }
}
fn candidate(kind: etsads_map::traffic::SemaphoreKind) -> GateCandidate {
    GateCandidate {
        owner_uid: 7,
        semaphore_id: 0,
        native_signal_group_id: "g".into(),
        source_lane_id: "a".into(),
        kind,
        position: Point3 {
            x: 40.0,
            y: 0.0,
            z: 0.0,
        },
        stop_target: Some(Point3 {
            x: 35.0,
            y: 0.0,
            z: 0.0,
        }),
        stop_geometry_status: Some("curve_start_proxy".into()),
        toll_classification_verified: false,
        activation_verified: false,
        semantics_resolved: true,
        profile: "arbitrary_name_not_used".into(),
        profile_resolution: Some("native_ppd".into()),
        profile_evidence_id: None,
    }
}
fn mission(
    mode: GateMode,
) -> (
    ControlledRouteMission,
    etsads_map::importer::ImportedGraph,
    VehicleGeometry,
) {
    let map = source_map();
    let native = NativeRouteIndex::from_exported(&map).unwrap();
    let imported = map.into_lane_graph().unwrap();
    let corridor = native
        .resolve(&[1, 2], "a", 0.0, &CorridorPolicy::default())
        .unwrap()
        .bind(&native, &imported)
        .unwrap();
    let kind = match mode {
        GateMode::Manual => etsads_map::traffic::SemaphoreKind::BarrierManualTimed,
        GateMode::Automatic => etsads_map::traffic::SemaphoreKind::BarrierAutomatic,
        GateMode::Distance => etsads_map::traffic::SemaphoreKind::BarrierDistanceActivated,
    };
    let tolls = derive_native_gates(
        &imported.graph,
        &imported.runtime_to_source,
        &corridor,
        &[candidate(kind)],
        &[],
    )
    .unwrap();
    let traffic=TrafficContext::from_json(&serde_json::json!({"schema_version":1,"source_lane_ids":["a"],"conflicts":[],"lane_controls":[],
        "signals":[{"id":"g","owner_uid":7,"semaphore_id":0,"type":kind,"x":40.0,"y":0.0,"z":0.0,"yaw":0.0,"profile":"synthetic","cycle_delay_s":0.0,"interval_0":999.0,"interval_1":0.0,"interval_2":0.0,"interval_3":0.0}],
        "signal_lanes":[{"signal_id":"g","lane_id":"a"}],"stop_lines":[{"id":"s","lane_id":"a","signal_id":"g","rule":"","x":35.0,"y":0.0,"z":0.0,"geometry_status":"curve_start_proxy","control_kind":"signal"}]}).to_string()).unwrap();
    let mission = ControlledRouteMission {
        route: vec![1],
        maximum_speed_mps: 5.0,
        boundary: VerifiedBoundary {
            polygon: vec![(-50.0, -20.0), (200.0, -20.0), (200.0, 20.0), (-50.0, 20.0)],
            physical_boundaries_verified: true,
            evidence_path: "synthetic".into(),
        },
        traffic,
        lane_changes: vec![],
        tolls,
        roundabout_exits: vec![],
        live_manoeuvres_verified: false,
        control_evidence_path: "synthetic only".into(),
    };
    let geometry = VehicleGeometry {
        has_trailer: false,
        trailer_wheelbase: 0.0,
        trailer_length: 0.0,
        ..Default::default()
    };
    (mission, imported, geometry)
}
fn update(
    engine: &mut RouteEngine,
    graph: &LaneGraph,
    g: VehicleGeometry,
    time: f64,
    x: Option<f64>,
    state: Option<i32>,
) -> RouteDirective {
    let x = x.unwrap_or(40.0 - 3.0 - g.resolved_envelope().tractor_front_from_rear_axle_m);
    let ego = EgoState {
        header: etsads_msgs::Header {
            frame_id: (time * 10.0) as u64 + 1,
            sim_time_us: (time * 1e6) as u64,
            seq: 0,
        },
        pose: Pose2 {
            x,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut ext = Box::<ExtensionFrame>::default();
    ext.header.frame_id = ego.header.frame_id;
    ext.available = 8;
    if let Some(state) = state {
        let mut signal = etsads_game::ets2la::Signal::zeroed();
        signal.position = [40.0, 0.0, 0.0];
        signal.kind = 2;
        signal.state = state;
        ext.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
    }
    engine
        .update(
            graph,
            ego,
            LaneState {
                lane_id: 1,
                s: x,
                d: 0.0,
                heading_err: 0.0,
                confidence: 1.0,
            },
            &[],
            Some(&ext),
            &[],
        )
        .unwrap()
}
#[test]
fn native_candidates_select_current_corridor_and_preserve_unknown_gates() {
    let map = source_map();
    let native = NativeRouteIndex::from_exported(&map).unwrap();
    let imported = map.into_lane_graph().unwrap();
    let mut corridor = native
        .resolve(&[1, 2], "a", 0.0, &CorridorPolicy::default())
        .unwrap()
        .bind(&native, &imported)
        .unwrap();
    let c = candidate(etsads_map::traffic::SemaphoreKind::BarrierAutomatic);
    let result = derive_native_gates(
        &imported.graph,
        &imported.runtime_to_source,
        &corridor,
        std::slice::from_ref(&c),
        &[],
    )
    .unwrap();
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].gate.as_ref().unwrap().mode, GateMode::Automatic);
    assert!(!result[0].gate.as_ref().unwrap().requires_payment_event);
    corridor.native.steps[0].exit_s_m = 20.0;
    assert!(
        derive_native_gates(
            &imported.graph,
            &imported.runtime_to_source,
            &corridor,
            std::slice::from_ref(&c),
            &[]
        )
        .unwrap()
        .is_empty()
    );
    corridor.native.steps[0].exit_s_m = 100.0;
    let mut unknown = c.clone();
    unknown.semantics_resolved = false;
    assert!(
        derive_native_gates(
            &imported.graph,
            &imported.runtime_to_source,
            &corridor,
            &[unknown],
            &[]
        )
        .is_err()
    );
    let mut conflicting = c.clone();
    conflicting.kind = etsads_map::traffic::SemaphoreKind::BarrierManualTimed;
    assert!(
        derive_native_gates(
            &imported.graph,
            &imported.runtime_to_source,
            &corridor,
            &[c, conflicting],
            &[]
        )
        .is_err()
    );
}
#[test]
fn automatic_and_distance_open_gate_advance_without_green_light_or_activation() {
    for mode in [GateMode::Automatic, GateMode::Distance] {
        let (mission, imported, g) = mission(mode);
        let mut engine = RouteEngine::new(
            mission,
            &imported.graph,
            imported.runtime_to_source,
            g,
            false,
        )
        .unwrap();
        for time in [0.0, 1.0, 2.0] {
            let output = update(&mut engine, &imported.graph, g, time, None, Some(1));
            assert!(!output.intent.activate);
            assert_eq!(output.speed_limit, 0.0);
        }
        let open = update(&mut engine, &imported.graph, g, 3.0, None, Some(3));
        assert!(open.speed_limit > 0.0);
        assert!(open.stop_distance.is_none());
        assert!(!open.intent.activate);
        let missing = update(&mut engine, &imported.graph, g, 4.0, None, None);
        assert_eq!(missing.speed_limit, 0.0);
        assert!(!missing.intent.activate);
        let passed = update(&mut engine, &imported.graph, g, 5.0, Some(60.0), Some(3));
        assert!(passed.intent.gate.unwrap().action.passage_observed);
    }
}
#[test]
fn manual_gate_requires_explicit_authority_and_fresh_state() {
    for authorized in [false, true] {
        let (mission, imported, g) = mission(GateMode::Manual);
        let mut engine = RouteEngine::new(
            mission,
            &imported.graph,
            imported.runtime_to_source,
            g,
            authorized,
        )
        .unwrap();
        update(&mut engine, &imported.graph, g, 0.0, None, Some(1));
        assert!(
            !update(&mut engine, &imported.graph, g, 1.0, None, None)
                .intent
                .activate
        );
        let ready = update(&mut engine, &imported.graph, g, 2.0, None, Some(1));
        assert_eq!(ready.intent.activate, authorized);
        assert!(ready.intent.hold_brake);
        assert!(
            !update(&mut engine, &imported.graph, g, 3.0, None, Some(1))
                .intent
                .activate
        );
    }
}
#[test]
fn trailer_rear_must_clear_even_when_tractor_has_passed() {
    let plan = GatePlan {
        mode: GateMode::Automatic,
        native_signal_group_id: None,
        source_lane_id: "synthetic".into(),
        observation_positions: vec![Point3::default()],
        clearance_planes: vec![GatePlane {
            position: Point3::default(),
            heading: 0.0,
        }],
        requires_payment_event: false,
        geometry_source: "synthetic".into(),
        profile_evidence_ids: vec![],
        source_stop_target: None,
    };
    let ego = EgoState {
        pose: Pose2 {
            x: 5.0,
            ..Default::default()
        },
        ..Default::default()
    };
    assert!(!plan.clearance(ego, VehicleGeometry::default()).1);
    assert!(
        plan.clearance(
            EgoState {
                pose: Pose2 {
                    x: 40.0,
                    ..Default::default()
                },
                ..ego
            },
            VehicleGeometry::default()
        )
        .1
    );
}
#[test]
fn gate_state_survives_numeric_id_rebinding_without_a_second_activation() {
    let (mission, imported, g) = mission(GateMode::Manual);
    let mut engine = RouteEngine::new(
        mission.clone(),
        &imported.graph,
        imported.runtime_to_source.clone(),
        g,
        true,
    )
    .unwrap();
    update(&mut engine, &imported.graph, g, 0.0, None, Some(1));
    assert!(
        update(&mut engine, &imported.graph, g, 1.0, None, Some(1))
            .intent
            .activate
    );
    let mut lane = imported.graph.lanes[&1].clone();
    lane.id = 17;
    let graph = LaneGraph::new(vec![lane]).unwrap();
    let mut replacement = mission;
    replacement.route = vec![17];
    replacement.tolls[0].lane_id = 17;
    engine
        .rebind(replacement, &graph, BTreeMap::from([(17, "a".into())]), g)
        .unwrap();
    assert_eq!(
        engine.gates.values().next().unwrap().machine.phase(),
        GatePhase::Waiting
    );
    let x = 40.0 - 3.0 - g.resolved_envelope().tractor_front_from_rear_axle_m;
    let ego = EgoState {
        header: etsads_msgs::Header {
            frame_id: 30,
            sim_time_us: 2_000_000,
            seq: 0,
        },
        pose: Pose2 {
            x,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut ext = Box::<ExtensionFrame>::default();
    ext.available = 8;
    ext.header.frame_id = 30;
    let mut signal = etsads_game::ets2la::Signal::zeroed();
    signal.kind = 2;
    signal.state = 1;
    signal.position = [40.0, 0.0, 0.0];
    ext.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
    let out = engine
        .update(
            &graph,
            ego,
            LaneState {
                lane_id: 17,
                s: x,
                d: 0.0,
                heading_err: 0.0,
                confidence: 1.0,
            },
            &[],
            Some(&ext),
            &[],
        )
        .unwrap();
    assert!(!out.intent.activate);
    assert_eq!(out.intent.gate.unwrap().action.attempts, 1);
}

#[test]
fn native_runtime_derives_manual_interaction_from_an_empty_mission_template() {
    use crate::{Runtime, RuntimeStatus, navigation::*};
    use etsads_map::{
        corridor::GlobalNativeRouteIndex,
        importer::RegionPolicy,
        sector::{ReadStats, RegionLoad},
    };
    let (mut template, _, _) = mission(GateMode::Manual);
    template.route.clear();
    template.tolls.clear();
    let source_id = "ab".repeat(32);
    let loaded = RegionLoad {
        map: source_map(),
        source_id: source_id.clone(),
        stats: ReadStats::default(),
    };
    let native = NativeRouteIndex::from_region(&loaded).unwrap();
    let authorization = RegionPolicy {
        region_id: "synthetic gate integration".into(),
        evidence_path: "synthetic; never live proof".into(),
        verified_lane_ids: vec!["a".into()],
        allow_navigation_width: false,
        require_fresh_telemetry_speed_limit: true,
        maximum_speed_mps: 5.0,
        maximum_lateral_error_m: 0.8,
    };
    let region = loaded
        .map
        .into_commissioning_region(authorization.clone(), 8.0)
        .unwrap();
    let global = GlobalNativeRouteIndex::from_json(
        &serde_json::json!({
            "schema_version":1,"source_manifest_sha256":"synthetic","source_cache_key":"synthetic",
            "topology_only":true,"nodes":[{"uid":1,"next":[2]},{"uid":2,"next":[]}]
        })
        .to_string(),
        "synthetic",
        "synthetic",
        &source_id,
    )
    .unwrap();
    let map = NavigationRegion {
        native,
        region: region.clone(),
        global: std::sync::Arc::new(global),
        traffic: template.traffic.clone(),
        gate_candidates: vec![candidate(
            etsads_map::traffic::SemaphoreKind::BarrierManualTimed,
        )],
        authorization,
        road_evidence: None,
        road_boundary: None,
        provenance: MapProvenance {
            source_id: source_id.clone(),
            annotations_source_id: "synthetic annotations".into(),
            center: Point3::default(),
            radius_m: 2000.0,
            generation: 1,
            reason: "synthetic gate integration".into(),
            reads: ReadStats::default(),
        },
    };
    let (mut config, _) = crate::replay::controlled_fixture(false, false, false);
    config.mode = crate::DrivingMode::ControlledRoute { mission: template };
    config.route_source = serde_json::from_value(
        serde_json::json!({"kind":"native_gps","expected_source_id":source_id}),
    )
    .unwrap();
    let calibration = crate::replay::low_speed_calibration();
    let state = etsads_control::TruckState {
        pose: Pose2 {
            x: 40.0
                - 3.0
                - calibration
                    .geometry
                    .resolved_envelope()
                    .tractor_front_from_rear_axle_m,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut runtime = Runtime::new(config, calibration, Some(region)).unwrap();
    runtime.install_native_region(map).unwrap();
    let mut activation_frames = 0;
    for frame in 0..120 {
        let mut snapshot = crate::replay::simulated_frame(state, frame, None, false);
        snapshot.scene.route = vec![1, 2];
        let extension = snapshot.extension.as_mut().unwrap();
        extension.available |= 8 | 16;
        let mut signal = etsads_game::ets2la::Signal::zeroed();
        signal.position = [40.0, 0.0, 0.0];
        signal.kind = 2;
        signal.state = 1;
        extension.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
        let output = runtime.step(&snapshot);
        assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
        if output.intent.activate && output.planned_this_frame {
            activation_frames += 1;
        }
        if output.status == RuntimeStatus::Running {
            let gate = output.intent.gate.as_ref().unwrap();
            assert_eq!(gate.native_signal_group_id.as_deref(), Some("g"));
            assert_eq!(gate.mode, GateMode::Manual);
            assert_eq!(gate.source_lane_id, "a");
            assert!(!gate.payment_required);
        }
    }
    assert_eq!(activation_frames, 1);
}
