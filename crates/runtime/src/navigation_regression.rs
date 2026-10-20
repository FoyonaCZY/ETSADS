//! Synthetic integration of native observations, map windows and the real
//! route/controller stack. No live permissions or IPC are created here.
use crate::{navigation::*, *};
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    corridor::{GlobalNativeRouteIndex, NativeRouteIndex},
    importer::{ExportLane, ExportedMap, RegionPolicy},
    sector::{CacheLimits, ReadStats, RegionLoad, SectorStore},
    traffic::TrafficContext,
};
use std::{path::PathBuf, sync::Arc};

fn source_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../map/testdata/navigation-sectors")
}
fn policy(ids: Vec<String>) -> RegionPolicy {
    RegionPolicy {
        region_id: "synthetic-native-navigation".into(),
        evidence_path: "synthetic geometry; never game evidence".into(),
        verified_lane_ids: ids,
        allow_navigation_width: false,
        require_fresh_telemetry_speed_limit: true,
        maximum_speed_mps: 8.0,
        maximum_lateral_error_m: 0.8,
    }
}
fn disk_region(center: f64, generation: u64) -> NavigationRegion {
    let path = source_fixture();
    let center = Point3 {
        x: center,
        y: 0.0,
        z: 0.0,
    };
    let loaded = SectorStore::open(&path, CacheLimits::default())
        .unwrap()
        .load_2km(center)
        .unwrap();
    let annotations = AnnotationStore::open(&path, AnnotationLimits::default())
        .unwrap()
        .load_for_region(center, 2000.0, &loaded)
        .unwrap();
    let native = NativeRouteIndex::from_region(&loaded).unwrap();
    let authorization = policy(vec!["a".into(), "m".into(), "z".into()]);
    let local = authorization.for_loaded_map(&loaded.map).unwrap();
    let region = loaded.map.into_commissioning_region(local, 8.0).unwrap();
    NavigationRegion {
        native: native.for_region_graph(&region).unwrap(),
        region,
        global: Arc::new(GlobalNativeRouteIndex::load_source(path).unwrap()),
        traffic: annotations.traffic,
        gate_candidates: annotations.gate_candidates,
        authorization,
        road_evidence: None,
        road_boundary: None,
        provenance: MapProvenance {
            source_id: loaded.source_id,
            annotations_source_id: annotations.source_id,
            center,
            radius_m: 2000.0,
            generation,
            reason: "synthetic disk window change".into(),
            reads: loaded.stats,
        },
    }
}
pub fn synthetic_disk_navigation_fixture(
    center: f64,
    generation: u64,
) -> (
    RuntimeConfig,
    etsads_control::identification::VehicleCalibration,
    NavigationRegion,
) {
    let map = disk_region(center, generation);
    let config = native_config(&map);
    let mut calibration = crate::replay::low_speed_calibration();
    calibration.evidence = "synthetic disk fixture; never live calibration".into();
    (config, calibration, map)
}
fn native_config(map: &NavigationRegion) -> RuntimeConfig {
    let (mut config, _) = crate::replay::controlled_fixture(false, false, false);
    if let DrivingMode::ControlledRoute { mission } = &mut config.mode {
        mission.route.clear();
        mission.boundary.polygon = vec![
            (-100.0, -20.0),
            (6500.0, -20.0),
            (6500.0, 20.0),
            (-100.0, 20.0),
        ];
    }
    config.route_source = RouteSource::NativeGps(Box::new(NativeGpsSettings {
        expected_source_id: map.provenance.source_id.clone(),
        corridor_policy: Default::default(),
        tolls: vec![],
        roundabout_exits: vec![],
        road_evidence: map.road_evidence.clone(),
        road_boundary_path: map
            .road_evidence
            .as_ref()
            .map(|_| "synthetic-road-boundary.json".into()),
    }));
    config
}
fn frame(state: TruckState, index: u64, gps: &[u64]) -> GameSnapshot {
    let mut snapshot = crate::replay::simulated_frame(state, index, None, false);
    snapshot.scene.route = gps.to_vec();
    snapshot.extension.as_mut().unwrap().available |= 16;
    snapshot
}
fn branch_region(length: f64) -> NavigationRegion {
    branch_region_mode(length, false)
}
fn branch_region_mode(length: f64, road: bool) -> NavigationRegion {
    let a = "100:road:0".to_string();
    let b = "100:road:1".to_string();
    let wrong = "200:road:0".to_string();
    let exit = "300:road:0".to_string();
    let lane = |id: String, owner, node_start, node_end, y, start, end, successors: Vec<String>| {
        ExportLane {
            id,
            owner_uid: owner,
            node_start,
            node_end,
            center: vec![
                Point3 {
                    x: start,
                    y,
                    z: 0.0,
                },
                Point3 { x: end, y, z: 0.0 },
            ],
            width: Some(4.5),
            speed_limit: Some(if road { 14.0 } else { 8.0 }),
            geometry_status: "synthetic verified fixture".into(),
            centerline_validated: true,
            physical_boundaries_verified: true,
            truck_access: Some(true),
            successors,
            ..Default::default()
        }
    };
    let mut lanes = vec![
        lane(a.clone(), 100, 1, 2, 0.0, 0.0, length, vec![wrong.clone()]),
        lane(b.clone(), 100, 1, 2, 4.5, 0.0, length, vec![exit.clone()]),
        lane(wrong, 200, 2, 3, 0.0, length, length + 200.0, vec![]),
        lane(exit, 300, 2, 4, 4.5, length, length + 200.0, vec![]),
    ];
    lanes[0].left = Some(b.clone());
    lanes[0].can_change_left = true;
    lanes[0].left_change_allowed_raw = Some(true);
    lanes[1].right = Some(a);
    lanes[1].can_change_right = true;
    lanes[1].right_change_allowed_raw = Some(true);
    let source = "ab".repeat(32);
    let loaded = RegionLoad {
        map: ExportedMap {
            schema_version: 1,
            source: "synthetic".into(),
            accepted_for_autonomous_driving: false,
            lanes,
        },
        source_id: source.clone(),
        stats: ReadStats::default(),
    };
    let native = NativeRouteIndex::from_region(&loaded).unwrap();
    let mut authorization = policy(loaded.map.lanes.iter().map(|l| l.id.clone()).collect());
    let road_evidence = road.then(|| etsads_map::importer::RoadEvidence {
        centerline_evidence_path: "synthetic centerline measurement".into(),
        physical_boundary_evidence_path: "synthetic physical boundary measurement".into(),
        vehicle_access_evidence_path: "synthetic vehicle access".into(),
    });
    let traffic=TrafficContext::from_json(&serde_json::json!({"schema_version":1,"lanes":loaded.map.lanes.iter().map(|l|serde_json::json!({"id":l.id})).collect::<Vec<_>>(),"conflicts":[],"lane_controls":[],"signals":[],"signal_lanes":[],"stop_lines":[]}).to_string()).unwrap();
    let region = if let Some(evidence) = &road_evidence {
        authorization.maximum_speed_mps = 14.0;
        loaded
            .map
            .into_road_region(
                etsads_map::importer::RoadRegionPolicy {
                    authorization: authorization.clone(),
                    evidence: evidence.clone(),
                },
                14.0,
            )
            .unwrap()
            .region
    } else {
        loaded
            .map
            .into_commissioning_region(authorization.clone(), 8.0)
            .unwrap()
    };
    let document = serde_json::json!({"schema_version":1,"source_manifest_sha256":"synthetic","source_cache_key":"synthetic","topology_only":true,"nodes":[{"uid":1,"next":[2]},{"uid":2,"next":[3,4]},{"uid":3,"next":[]},{"uid":4,"next":[]}]});
    let global =
        GlobalNativeRouteIndex::from_json(&document.to_string(), "synthetic", "synthetic", &source)
            .unwrap();
    let road_boundary = road_evidence.as_ref().map(|e| RoadBoundary {
        schema_version: 1,
        map_source_id: source.clone(),
        coordinate_system: RoadBoundaryCoordinates::EnuMetres,
        verified_source_lane_ids: authorization.verified_lane_ids.clone(),
        boundary: crate::parking_mission::VerifiedBoundary {
            polygon: vec![
                (-100.0, -20.0),
                (length + 300.0, -20.0),
                (length + 300.0, 20.0),
                (-100.0, 20.0),
            ],
            physical_boundaries_verified: true,
            evidence_path: e.physical_boundary_evidence_path.clone(),
        },
    });
    NavigationRegion {
        native,
        region,
        global: Arc::new(global),
        traffic,
        gate_candidates: vec![],
        authorization,
        road_evidence,
        road_boundary,
        provenance: MapProvenance {
            source_id: source,
            annotations_source_id: "synthetic-explicit-annotations".into(),
            center: Point3::default(),
            radius_m: 2000.0,
            generation: 1,
            reason: "synthetic native branch and mandatory lane change".into(),
            reads: ReadStats::default(),
        },
    }
}

#[derive(Serialize)]
pub struct NativeNavigationReport {
    pub evidence: &'static str,
    pub cross_region: Vec<RuntimeOutput>,
    pub lane_change: Vec<RuntimeOutput>,
    pub lane_change_final: TruckState,
    pub lane_change_constraint_violations: usize,
    pub unknown_uid_reason: String,
    pub reroute_reason: String,
    pub deadline_hold_final: TruckState,
    pub road_operation: Vec<RuntimeOutput>,
    pub road_boundary: Option<RoadBoundary>,
    pub road_geometry: etsads_msgs::VehicleGeometry,
}
fn road_calibration() -> etsads_control::identification::VehicleCalibration {
    let mut calibration = crate::replay::low_speed_calibration();
    let fingerprint =
        etsads_control::identification::geometry_fingerprint(&crate::replay::measured()).unwrap();
    let envelope = etsads_msgs::MeasuredEnvelope {
        tractor_front_from_rear_axle_m: 4.5,
        tractor_rear_overhang_m: 1.7,
        trailer: Some(etsads_msgs::TrailerEnvelope {
            front_from_hitch_m: 1.2,
            rear_from_axle_m: 4.4,
        }),
    };
    calibration.geometry.measured_envelope = Some(envelope);
    calibration.geometry_fingerprint = Some(fingerprint.clone());
    calibration.front_bumper = None;
    calibration.envelope_measurement = Some(etsads_control::identification::EnvelopeMeasurement {
        geometry_fingerprint: fingerprint,
        envelope,
        evidence_path: "explicit synthetic exterior model; not game evidence".into(),
    });
    calibration.dynamic_speed_range_mps = Some([0.0, 20.0]);
    calibration.evidence = "synthetic road dynamics; not live calibration".into();
    calibration
}
fn road_operation() -> Result<Vec<RuntimeOutput>, String> {
    let map = branch_region_mode(1000.0, true);
    let mut config = native_config(&map);
    config.mode = DrivingMode::LaneFollow {
        route: vec![],
        maximum_speed_mps: 12.0,
    };
    let calibration = road_calibration();
    let geometry = calibration.geometry;
    let mut runtime =
        Runtime::new(config, calibration, Some(map.region.clone())).map_err(|e| e.to_string())?;
    runtime
        .install_native_region(map)
        .map_err(|e| e.to_string())?;
    let mut state = TruckState {
        pose: Pose2 {
            x: 20.0,
            ..Default::default()
        },
        speed: 10.0,
        ..Default::default()
    };
    let mut outputs = vec![];
    for i in 0..400 {
        let output = runtime.step(&frame(state, i, &[1, 2, 3]));
        if output.status == RuntimeStatus::Fault {
            return Err(output.reason);
        }
        if output.status == RuntimeStatus::Running {
            outputs.push(output.clone());
        }
        let command = output.actuation;
        let a = command.throttle * 1.2
            - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
        state = state
            .step(geometry, command.steer * 0.6, a, 0.02)
            .map_err(|e| e.to_string())?;
    }
    Ok(outputs)
}
pub fn run_native_navigation_regression() -> Result<NativeNavigationReport, String> {
    let first = disk_region(1200.0, 1);
    let mut runtime = Runtime::new(
        native_config(&first),
        crate::replay::low_speed_calibration(),
        Some(first.region.clone()),
    )
    .map_err(|e| e.to_string())?;
    runtime
        .install_native_region(first)
        .map_err(|e| e.to_string())?;
    let mut state = TruckState {
        pose: Pose2 {
            x: 2500.0,
            y: 0.0,
            yaw: 0.0,
        },
        speed: 6.0,
        ..Default::default()
    };
    runtime.step(&frame(state, 0, &[2, 3, 4]));
    let mut cross_region = vec![runtime.step(&frame(state, 1, &[2, 3, 4]))];
    runtime
        .install_native_region(disk_region(4500.0, 2))
        .map_err(|e| e.to_string())?;
    state.pose.x += 0.12;
    cross_region.push(runtime.step(&frame(state, 2, &[2, 3, 4])));
    for output in &cross_region {
        if output.status != RuntimeStatus::Running {
            return Err(output.reason.clone());
        }
    }
    let reroute_reason = runtime.step(&frame(state, 3, &[2, 4])).reason;
    let map = branch_region(300.0);
    let mut runtime = Runtime::new(
        native_config(&map),
        crate::replay::low_speed_calibration(),
        Some(map.region.clone()),
    )
    .map_err(|e| e.to_string())?;
    runtime
        .install_native_region(map)
        .map_err(|e| e.to_string())?;
    state = TruckState {
        pose: Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        },
        speed: 6.0,
        ..Default::default()
    };
    let mut lane_change = Vec::new();
    let mut violations = 0;
    for i in 0..1500 {
        let output = runtime.step(&frame(state, i, &[1, 2, 4]));
        if output.status == RuntimeStatus::Fault {
            return Err(format!("native lane-change frame {i}: {}", output.reason));
        }
        if output.planned_this_frame || i == 0 {
            lane_change.push(output.clone());
        }
        if state.pose.x > 285.0 && (state.pose.y - 4.5).abs() > 0.3 {
            violations += 1;
        }
        let command = output.actuation;
        let a = command.throttle * 1.2
            - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
        state = state
            .step(
                etsads_msgs::VehicleGeometry::default(),
                command.steer * 0.6,
                a,
                0.02,
            )
            .map_err(|e| e.to_string())?;
        if state.pose.x > 100.0 && (state.pose.y - 4.5).abs() < 0.3 {
            break;
        }
    }
    let lane_change_final = state;
    let map = branch_region(100.0);
    let mut runtime = Runtime::new(
        native_config(&map),
        crate::replay::low_speed_calibration(),
        Some(map.region.clone()),
    )
    .map_err(|e| e.to_string())?;
    runtime
        .install_native_region(map)
        .map_err(|e| e.to_string())?;
    state = TruckState {
        pose: Pose2 {
            x: 20.0,
            y: 0.0,
            yaw: 0.0,
        },
        speed: 3.0,
        ..Default::default()
    };
    for i in 0..1000 {
        let mut observation = frame(state, i, &[1, 2, 4]);
        // A real target-lane side-by-side body prevents gap admission. It is
        // not an obstacle in the current lane; stopping is a deadline decision.
        observation.scene.agents.push(Agent {
            id: 44,
            pose: Pose2 {
                x: state.pose.x,
                y: 4.5,
                yaw: 0.0,
            },
            z: 0.0,
            speed: state.speed,
            acceleration: 0.0,
            length: 6.0,
            width: 2.5,
            lane_id: None,
            trailers: vec![],
        });
        let output = runtime.step(&observation);
        if output.status == RuntimeStatus::Fault {
            return Err(format!("native deadline frame {i}: {}", output.reason));
        }
        let command = output.actuation;
        let a = command.throttle * 1.2
            - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
        state = state
            .step(
                etsads_msgs::VehicleGeometry::default(),
                command.steer * 0.6,
                a,
                0.02,
            )
            .map_err(|e| e.to_string())?;
    }
    let map = branch_region(300.0);
    let mut unknown = Runtime::new(
        native_config(&map),
        crate::replay::low_speed_calibration(),
        Some(map.region.clone()),
    )
    .map_err(|e| e.to_string())?;
    unknown
        .install_native_region(map)
        .map_err(|e| e.to_string())?;
    unknown.step(&frame(state, 0, &[1, 2, 999]));
    let unknown_uid_reason = unknown.step(&frame(state, 1, &[1, 2, 999])).reason;
    Ok(NativeNavigationReport {
        evidence: "synthetic Runtime Replay only; no game actuation or phase acceptance",
        cross_region,
        lane_change,
        lane_change_final,
        lane_change_constraint_violations: violations,
        unknown_uid_reason,
        reroute_reason,
        deadline_hold_final: state,
        road_operation: road_operation()?,
        road_boundary: branch_region_mode(1000.0, true).road_boundary,
        road_geometry: road_calibration().geometry,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn live_road_worker_keeps_boundary_when_receiving_native_planning_context() {
        let mut map = branch_region_mode(1000.0, true);
        map.road_boundary.as_mut().unwrap().boundary.polygon =
            vec![(-100.0, -10.0), (45.0, -10.0), (45.0, 10.0), (-100.0, 10.0)];
        let mut config = native_config(&map);
        config.execution = ExecutionMode::Live;
        config.mode = DrivingMode::LaneFollow {
            route: vec![],
            maximum_speed_mps: 10.0,
        };
        let mut calibration = crate::replay::with_measured_envelope(crate::replay::calibration());
        calibration.dynamic_speed_range_mps = Some([5.0, 10.0]);
        let proof = VerifiedRoadOperation::from_host_verification(
            &config,
            &calibration,
            map.scope_digest().unwrap(),
            [9; 32],
        )
        .unwrap();
        let mut runtime =
            Runtime::new_live_road(config, calibration, Some(map.region.clone()), proof, None)
                .unwrap();
        runtime.install_native_region(map).unwrap();
        let state = TruckState {
            pose: Pose2 {
                x: 20.0,
                ..Default::default()
            },
            speed: 6.0,
            ..Default::default()
        };
        runtime.step(&frame(state, 0, &[1, 2, 3]));
        let mut failure = None;
        for n in 1..12 {
            let output = runtime.step(&frame(state, n, &[1, 2, 3]));
            assert!(
                !output.request_control,
                "unsafe worker candidate reached control"
            );
            if output.status == RuntimeStatus::Fault {
                failure = Some(output.reason);
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert!(failure.is_some_and(|reason| reason.contains("continuous sweep")));
    }
    fn road_runtime(map: NavigationRegion) -> Runtime {
        let mut config = native_config(&map);
        config.mode = DrivingMode::LaneFollow {
            route: vec![],
            maximum_speed_mps: 12.0,
        };
        let mut calibration = crate::replay::low_speed_calibration();
        calibration.dynamic_speed_range_mps = Some([0.0, 20.0]);
        let mut runtime = Runtime::new(config, calibration, Some(map.region.clone())).unwrap();
        // Shared helper allows the direct continuous-sweep counterexample to
        // test independently of planning a future straightening manoeuvre.
        runtime.road_boundary = map.road_boundary.clone();
        runtime.install_native_region(map.clone()).unwrap();
        runtime
    }
    #[test]
    fn road_requires_measured_enu_data_and_rejects_future_body_exit_before_command() {
        let mut map = branch_region_mode(1000.0, true);
        let mut config = native_config(&map);
        config.mode = DrivingMode::LaneFollow {
            route: vec![],
            maximum_speed_mps: 12.0,
        };
        let mut calibration = crate::replay::low_speed_calibration();
        calibration.dynamic_speed_range_mps = Some([0.0, 20.0]);
        let mut missing = map.clone();
        missing.road_boundary = None;
        let mut runtime =
            Runtime::new(config.clone(), calibration, Some(map.region.clone())).unwrap();
        assert!(
            runtime
                .install_native_region(missing)
                .unwrap_err()
                .to_string()
                .contains("boundary data")
        );
        let mut data = serde_json::to_value(map.road_boundary.as_ref().unwrap()).unwrap();
        data["coordinate_system"] = serde_json::json!("SCS");
        assert!(serde_json::from_value::<RoadBoundary>(data).is_err());
        let boundary = &mut map.road_boundary.as_mut().unwrap().boundary;
        boundary.polygon = vec![(-100.0, -10.0), (45.0, -10.0), (45.0, 10.0), (-100.0, 10.0)];
        let mut runtime = road_runtime(map);
        let state = TruckState {
            pose: Pose2 {
                x: 20.0,
                ..Default::default()
            },
            speed: 10.0,
            ..Default::default()
        };
        runtime.step(&frame(state, 0, &[1, 2, 3]));
        let output = runtime.step(&frame(state, 1, &[1, 2, 3]));
        assert_eq!(output.status, RuntimeStatus::Fault, "{}", output.reason);
        assert!(
            output
                .reason
                .contains("planned tractor/trailer continuous sweep")
        );
        assert!(!output.request_control && output.trajectory.is_none());
        assert!(
            output.control_ego.pose.x
                + runtime
                    .calibration
                    .geometry
                    .resolved_envelope()
                    .tractor_front_from_rear_axle_m
                < 45.0
        );
    }
    #[test]
    fn continuous_road_check_catches_rotating_trailer_corner_between_valid_endpoints() {
        use etsads_planning::geometry::{box_in_polygon, corners, footprints};
        let mut map = branch_region_mode(1000.0, true);
        let initial = TruckState {
            pose: Pose2 {
                x: 20.0,
                ..Default::default()
            },
            trailer_yaw: -0.5,
            ..Default::default()
        };
        let end = TruckState {
            trailer_yaw: 0.5,
            ..initial
        };
        let midpoint = TruckState {
            trailer_yaw: 0.0,
            ..initial
        };
        let geometry = etsads_msgs::VehicleGeometry::default();
        let min_x = |state| {
            footprints(state, geometry)
                .iter()
                .flat_map(|b| corners(*b))
                .map(|p| p.0)
                .fold(f64::INFINITY, f64::min)
        };
        let endpoints = min_x(initial).min(min_x(end));
        let middle = min_x(midpoint);
        assert!(middle < endpoints);
        let polygon = vec![
            ((middle + endpoints) / 2.0, -50.0),
            (100.0, -50.0),
            (100.0, 50.0),
            ((middle + endpoints) / 2.0, 50.0),
        ];
        assert!(
            footprints(initial, geometry)
                .iter()
                .chain(footprints(end, geometry).iter())
                .all(|b| box_in_polygon(*b, &polygon))
        );
        map.road_boundary.as_mut().unwrap().boundary.polygon = polygon;
        let mut runtime = road_runtime(map);
        runtime.check_road_execution(initial).unwrap();
        assert!(
            runtime
                .check_road_execution(end)
                .unwrap_err()
                .contains("observed tractor/trailer sweep")
        );
    }
    #[test]
    fn repeated_native_lane_cannot_overwrite_its_first_gps_exit() {
        let mut map = branch_region(300.0);
        let a = "100:road:0";
        let b = "100:road:1";
        let c = "200:road:0";
        {
            let lane = map.native.lanes.get_mut(a).unwrap();
            lane.successors = vec![b.into(), c.into()];
            lane.left = None;
            lane.can_change_left = false;
            let lane = map.native.lanes.get_mut(b).unwrap();
            lane.node_start = 2;
            lane.node_end = 1;
            lane.owner_uid = 101;
            lane.successors = vec![a.into()];
            lane.right = None;
            lane.can_change_right = false;
            lane.length_m = 340.0;
        }
        {
            let graph = &mut map.region.imported.graph;
            let lane = graph.lanes.get_mut(&1).unwrap();
            lane.successors = vec![2, 3];
            lane.left = None;
            lane.can_change_left = false;
            let lane = graph.lanes.get_mut(&2).unwrap();
            lane.successors = vec![1];
            lane.right = None;
            lane.can_change_right = false;
            lane.center = vec![
                Point3 {
                    x: 300.0,
                    y: 0.0,
                    z: 0.0,
                },
                Point3 {
                    x: 300.0,
                    y: 20.0,
                    z: 0.0,
                },
                Point3 {
                    x: 0.0,
                    y: 20.0,
                    z: 0.0,
                },
                Point3::default(),
            ];
            *graph = etsads_map::LaneGraph::new(graph.lanes.values().cloned().collect()).unwrap();
            map.region.imported.native_owners.insert(2, 101);
        }
        let document = serde_json::json!({"schema_version":1,"source_manifest_sha256":"synthetic","source_cache_key":"synthetic","topology_only":true,"nodes":[{"uid":1,"next":[2]},{"uid":2,"next":[1,3,4]},{"uid":3,"next":[]},{"uid":4,"next":[]}]});
        map.global = Arc::new(
            GlobalNativeRouteIndex::from_json(
                &document.to_string(),
                "synthetic",
                "synthetic",
                &map.provenance.source_id,
            )
            .unwrap(),
        );
        let mut runtime = Runtime::new(
            native_config(&map),
            crate::replay::low_speed_calibration(),
            Some(map.region.clone()),
        )
        .unwrap();
        runtime.install_native_region(map).unwrap();
        let state = TruckState {
            pose: Pose2 {
                x: 20.0,
                ..Default::default()
            },
            speed: 6.0,
            ..Default::default()
        };
        runtime.step(&frame(state, 0, &[1, 2, 1, 2, 3]));
        let output = runtime.step(&frame(state, 1, &[1, 2, 1, 2, 3]));
        assert_eq!(output.status, RuntimeStatus::Fault, "{}", output.reason);
        assert!(output.reason.contains("occurrence-aware"));
        assert!(output.trajectory.is_none() && !output.request_control);
    }
    #[test]
    fn live_road_requires_bound_host_evidence_and_cannot_expand_map_scope() {
        let mut map = branch_region_mode(1000.0, true);
        let mut config = native_config(&map);
        config.execution = ExecutionMode::Live;
        config.mode = DrivingMode::LaneFollow {
            route: vec![],
            maximum_speed_mps: 10.0,
        };
        let mut calibration = crate::replay::with_measured_envelope(crate::replay::calibration());
        calibration.dynamic_speed_range_mps = Some([5.0, 10.0]);
        assert!(
            Runtime::new(
                config.clone(),
                calibration.clone(),
                Some(map.region.clone())
            )
            .err()
            .unwrap()
            .to_string()
            .contains("host-verified")
        );
        let proof = VerifiedRoadOperation::from_host_verification(
            &config,
            &calibration,
            map.scope_digest().unwrap(),
            [7; 32],
        )
        .unwrap();
        let mut changed = calibration.clone();
        changed.delay_s = Some(0.2);
        assert!(
            Runtime::new_live_road(
                config.clone(),
                changed,
                Some(map.region.clone()),
                proof.clone(),
                None
            )
            .is_err()
        );
        let mut runtime =
            Runtime::new_live_road(config, calibration, Some(map.region.clone()), proof, None)
                .unwrap();
        runtime.install_native_region(map.clone()).unwrap();
        map.provenance.generation = 2;
        map.authorization.evidence_path = "different road measurements".into();
        map.region.policy.evidence_path = map.authorization.evidence_path.clone();
        assert!(
            runtime
                .install_native_region(map)
                .unwrap_err()
                .to_string()
                .contains("scope")
        );
    }
    #[test]
    fn consumed_gps_prefix_follows_observed_checkpoint_but_cannot_skip_an_exit() {
        for skip in [false, true] {
            let map = branch_region(300.0);
            let mut runtime = Runtime::new(
                native_config(&map),
                crate::replay::low_speed_calibration(),
                Some(map.region.clone()),
            )
            .unwrap();
            runtime.install_native_region(map).unwrap();
            let state = TruckState {
                pose: Pose2 {
                    x: 20.0,
                    ..Default::default()
                },
                speed: 6.0,
                ..Default::default()
            };
            runtime.step(&frame(state, 0, &[1, 2, 4]));
            runtime.step(&frame(state, 1, &[1, 2, 4]));
            let output = runtime.step(&frame(state, 2, if skip { &[4] } else { &[2, 4] }));
            if skip {
                assert_eq!(output.status, RuntimeStatus::Fault);
                assert!(output.reason.contains("ahead"));
            } else {
                assert_eq!(output.status, RuntimeStatus::Running, "{}", output.reason);
                assert_eq!(output.navigation.unwrap().consumed_prefix_nodes, 1);
            }
        }
    }
    #[test]
    fn road_mode_executes_above_commissioning_cap_but_never_beyond_calibration() {
        let outputs = road_operation().unwrap();
        assert!(
            outputs
                .iter()
                .all(|o| o.control_ego.speed > 30.0 / 3.6 && o.request_control)
        );
        assert!(
            outputs
                .iter()
                .flat_map(|o| o.trajectory.iter())
                .flat_map(|t| &t.points)
                .all(|p| p.speed <= 12.0 + 1e-6)
        );
        let map = branch_region_mode(1000.0, true);
        let mut config = native_config(&map);
        config.mode = DrivingMode::LaneFollow {
            route: vec![],
            maximum_speed_mps: 12.0,
        };
        let calibration = crate::replay::low_speed_calibration();
        assert!(
            Runtime::new(
                config.clone(),
                calibration.clone(),
                Some(map.region.clone())
            )
            .err()
            .unwrap()
            .to_string()
            .contains("dynamic speed")
        );
        let mut calibration = calibration;
        calibration.dynamic_speed_range_mps = Some([0.0, 10.0]);
        assert!(
            Runtime::new(config, calibration, Some(map.region))
                .err()
                .unwrap()
                .to_string()
                .contains("outside")
        );
    }
    #[test]
    fn native_snapshots_bind_branches_rebind_ids_and_stop_before_missed_exit() {
        let report = run_native_navigation_regression().unwrap();
        let old = report.cross_region[0].navigation.as_ref().unwrap();
        let new = report.cross_region[1].navigation.as_ref().unwrap();
        assert_eq!(old.corridor_source_ids, vec!["m"]);
        assert_eq!(new.corridor_source_ids, vec!["m", "z"]);
        assert_eq!(old.corridor_runtime_ids, vec![2]);
        assert_eq!(new.corridor_runtime_ids, vec![1, 2]);
        assert_eq!(old.next_boundary_checkpoint, Some(4));
        assert_eq!(new.next_boundary_checkpoint, None);
        assert!(
            (report.lane_change_final.pose.y - 4.5).abs() < 0.3,
            "{:?}",
            report.lane_change_final
        );
        assert_eq!(report.lane_change_constraint_violations, 0);
        assert!(
            report
                .lane_change
                .iter()
                .filter_map(|o| o.navigation.as_ref())
                .all(|n| !n.corridor_source_ids.contains(&"200:road:0".into()))
        );
        assert!(report.unknown_uid_reason.contains("999"));
        assert!(report.reroute_reason.contains("rerouted"));
        assert!(
            report.deadline_hold_final.pose.x < 45.0
                && report.deadline_hold_final.speed.abs() < 0.1,
            "{:?}",
            report.deadline_hold_final
        );
    }
    #[test]
    fn missing_annotations_boundaries_permissions_and_route_observation_fail_closed() {
        for kind in 0..4 {
            let mut map = branch_region(300.0);
            if kind == 0 {
                map.traffic =
                    TrafficContext::from_json(r#"{"schema_version":1,"lanes":[]}"#).unwrap();
            }
            if kind == 1 {
                map.native
                    .lanes
                    .get_mut("100:road:0")
                    .unwrap()
                    .physical_boundaries_verified = false;
            }
            if kind == 2 {
                map.native
                    .lanes
                    .get_mut("100:road:0")
                    .unwrap()
                    .can_change_left = false;
                map.region
                    .imported
                    .graph
                    .lanes
                    .get_mut(&1)
                    .unwrap()
                    .can_change_left = false;
            }
            let mut runtime = Runtime::new(
                native_config(&map),
                crate::replay::low_speed_calibration(),
                Some(map.region.clone()),
            )
            .unwrap();
            runtime.install_native_region(map).unwrap();
            let state = TruckState {
                pose: Pose2 {
                    x: 20.0,
                    ..Default::default()
                },
                speed: 6.0,
                ..Default::default()
            };
            runtime.step(&frame(state, 0, &[1, 2, 4]));
            let mut snapshot = frame(state, 1, &[1, 2, 4]);
            if kind == 3 {
                snapshot.extension.as_mut().unwrap().available &= !16;
            }
            let output = runtime.step(&snapshot);
            assert_eq!(
                output.status,
                RuntimeStatus::Fault,
                "{kind}:{}",
                output.reason
            );
            assert!(!output.request_control);
        }
    }
}
