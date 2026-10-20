//! Deterministic integration scenarios that drive the same Runtime used by the
//! live CLI. These fixtures provide synthetic evidence only.
use super::*;
use etsads_control::identification::{LongitudinalModel, SteeringRow};
use etsads_msgs::{
    Scene, VehicleGeometry,
    protocol::{ConfigBlock, TelemetryFrame},
};
use std::collections::BTreeMap;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReplayCaseReport {
    pub stage: String,
    pub name: String,
    pub passed: bool,
    pub duration_s: f64,
    pub distance_m: f64,
    pub final_pose: Pose2,
    pub target_error_m: f64,
    pub constraint_violations: usize,
    pub interaction_pulses: usize,
    pub gear_changes: usize,
    pub fault: Option<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RuntimeReplaySuite {
    pub evidence: String,
    pub all_passed: bool,
    pub cases: Vec<ReplayCaseReport>,
}
/// Replay the actual runtime through moving vehicle dynamics and synthetic SDK
/// signals/events. No shared memory is opened and no game inputs are written.
pub fn run_replay_suite() -> RuntimeReplaySuite {
    let mut cases = Vec::new();
    for kind in 0..4 {
        let (cfg, region) = if kind < 3 {
            let (c, r) = controlled_fixture(kind == 2, kind == 0, kind == 1);
            (c, Some(r))
        } else {
            let mut c = config(ExecutionMode::Replay);
            c.mode = DrivingMode::Parking {
                mission: parking_mission::ParkingMission {
                    target: parking_mission::ParkingTarget {
                        source_id: "synthetic trailer axle docking target".into(),
                        pose: Pose2 {
                            x: -18.0,
                            y: 1.0,
                            yaw: 0.0,
                        },
                        reference: parking_mission::SlotReference::TrailerAxle,
                    },
                    boundary: parking_mission::VerifiedBoundary {
                        polygon: vec![(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)],
                        physical_boundaries_verified: true,
                        evidence_path: "synthetic fixture boundary".into(),
                    },
                    static_obstacles: vec![
                        etsads_msgs::OrientedBox {
                            pose: Pose2 {
                                x: -24.0,
                                y: -2.2,
                                yaw: 0.0,
                            },
                            length: 14.0,
                            width: 2.5,
                        },
                        etsads_msgs::OrientedBox {
                            pose: Pose2 {
                                x: -24.0,
                                y: 4.5,
                                yaw: 0.0,
                            },
                            length: 14.0,
                            width: 2.5,
                        },
                    ],
                    max_expansions: 10000,
                    maximum_duration_s: 120.0,
                    recovery: Default::default(),
                },
            };
            (c, None)
        };
        let mut calibration = low_speed_calibration();
        calibration.evidence = "synthetic fixture; not live calibration".into();
        let mut runtime = Runtime::new(cfg, calibration, region)
            .expect("built-in replay configuration must validate");
        let mut state = TruckState {
            pose: Pose2 {
                x: if kind == 2 { 20.0 } else { 0.0 },
                ..Default::default()
            },
            speed: if kind == 3 { 0.0 } else { 6.0 },
            ..Default::default()
        };
        let mut activation = None;
        let mut pulses = 0;
        let mut paid = false;
        let mut stopped = false;
        let mut indicated = false;
        let mut completed = false;
        let mut gear = if kind == 3 { 0 } else { 1 };
        let mut shifts = 0;
        let mut violations = 0;
        let mut fault = None;
        let mut distance = 0.0;
        let mut steps = 0;
        for index in 0..if kind == 3 {
            6000
        } else if kind == 1 {
            1500
        } else {
            1200
        } {
            steps = index + 1;
            let open = activation.is_some_and(|frame| index > frame + 50);
            let signal = match kind {
                0 => Some(etsads_game::ets2la::Signal {
                    position: [60.0, 0.0, -3.0],
                    sector: [0, 0],
                    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                    kind: 1,
                    time_remaining: 30.0,
                    state: if index < 750 { 2 } else { 8 },
                    id: 99,
                }),
                1 => Some(etsads_game::ets2la::Signal {
                    position: [40.0, 0.0, 0.0],
                    sector: [0, 0],
                    quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                    kind: 2,
                    time_remaining: 0.0,
                    state: if open { 3 } else { 1 },
                    id: 101,
                }),
                _ => None,
            };
            let pay_now = kind == 1 && !paid && state.pose.x > 42.0;
            if pay_now {
                paid = true;
            }
            let mut frame = simulated_frame(state, index, signal, pay_now);
            frame.raw.gear = gear;
            let output = runtime.step(&frame);
            if output.status == RuntimeStatus::Fault {
                fault = Some(output.reason);
                break;
            }
            if output.status == RuntimeStatus::Completed {
                completed = true;
                break;
            }
            if output.actuation.activate {
                pulses += 1;
                activation = Some(index);
                if state.speed.abs() >= 0.1 {
                    violations += 1;
                }
            }
            if let Some(direction) = output.intent.gear_request {
                if state.speed.abs() >= 0.05 {
                    violations += 1;
                }
                gear = if direction == Direction::Reverse {
                    -1
                } else {
                    1
                };
                shifts += 1;
            }
            indicated |= output.actuation.indicator == 1;
            if kind == 0 && index < 750 {
                stopped |= state.speed < 0.1;
                if state.pose.x >= 60.0 - (3.8 * 0.35 + 6.2 / 2.0) - 3.0 + 0.5 {
                    violations += 1;
                }
            }
            if kind == 1 && !open && state.pose.x >= 40.0 - (3.8 * 0.35 + 6.2 / 2.0) - 3.0 + 0.6 {
                violations += 1;
            }
            if kind == 2 && index < 150 && state.pose.y.abs() >= 0.1 {
                violations += 1;
            }
            let command = output.actuation;
            let sign = if gear < 0 { -1.0 } else { 1.0 };
            let acceleration = sign * command.throttle * 1.2
                - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
            match state.step(
                VehicleGeometry::default(),
                command.steer * 0.6,
                acceleration,
                0.02,
            ) {
                Ok(next) => {
                    distance += state.pose.distance(next.pose);
                    state = next;
                }
                Err(e) => {
                    fault = Some(e.to_string());
                    break;
                }
            }
        }
        let target_error = if kind == 2 {
            (state.pose.y - 4.5).abs()
        } else if kind == 3 {
            state
                .trailer_axle(VehicleGeometry::default())
                .distance(Pose2 {
                    x: -18.0,
                    y: 1.0,
                    yaw: 0.0,
                })
        } else {
            0.0
        };
        let goal = match kind {
            0 => stopped && state.pose.x > 65.0,
            1 => paid && pulses == 1 && state.pose.x > 45.0,
            2 => indicated && target_error < 0.5,
            _ => completed && shifts >= 2 && target_error < 0.5,
        };
        cases.push(ReplayCaseReport {
            stage: match kind {
                0 => "P3",
                1 | 2 => "P2",
                _ => "P4",
            }
            .into(),
            name: match kind {
                0 => "Runtime red/green stop-line closed loop",
                1 => "Runtime toll gate and paid-event closed loop",
                2 => "Runtime signalled lane-change closed loop",
                _ => "Runtime articulated parking and gear-acknowledgement closed loop",
            }
            .into(),
            passed: goal && violations == 0 && fault.is_none(),
            duration_s: steps as f64 * 0.02,
            distance_m: distance,
            final_pose: state.pose,
            target_error_m: target_error,
            constraint_violations: violations,
            interaction_pulses: pulses,
            gear_changes: shifts,
            fault,
        });
    }
    RuntimeReplaySuite{evidence:"synthetic SDK-message replay and vehicle dynamics; no live ETS2 exit criterion is inferred".into(),all_passed:cases.iter().all(|c|c.passed),cases}
}
pub(crate) fn measured() -> ConfigBlock {
    let mut c = ConfigBlock {
        wheel_count: 4,
        ..Default::default()
    };
    c.wheel_positions[0] = [-1.0, 0.0, -1.9];
    c.wheel_positions[1] = [1.0, 0.0, -1.9];
    c.wheel_positions[2] = [-1.0, 0.0, 1.9];
    c.wheel_positions[3] = [1.0, 0.0, 1.9];
    c.wheel_radius[..4].fill(0.5);
    c.wheel_steerable[..2].fill(1);
    c.hook_position[2] = 2.3;
    c
}
pub(crate) fn calibration() -> VehicleCalibration {
    VehicleCalibration {
        schema_version: 1,
        vehicle_id: "unit-test-truck".into(),
        source_run: "synthetic unit fixture".into(),
        evidence: "recorded game telemetry".into(),
        geometry: VehicleGeometry::default(),
        geometry_fingerprint: Some(geometry_fingerprint(&measured()).unwrap()),
        envelope_measurement: None,
        front_bumper: Some(etsads_control::identification::FrontBumperMeasurement {
            front_from_rear_axle_m: VehicleGeometry::default().wheelbase * 0.35
                + VehicleGeometry::default().tractor_length / 2.0,
            geometry_fingerprint: geometry_fingerprint(&measured()).unwrap(),
            evidence_path: "synthetic model exterior, not installed-game evidence".into(),
        }),
        steering_axis_source: "semantic_applied".into(),
        reference: Some(
            etsads_control::identification::reference_from_config(
                &measured(),
                &[2, 3],
                "synthetic fixture rear-wheel indices",
            )
            .unwrap(),
        ),
        conditions: Some(etsads_control::identification::CalibrationConditions {
            trailer_count: 1,
            cargo_mass_range_kg: [0.0, 100.0],
            evidence_path: "synthetic fixture attachment/load".into(),
        }),
        sample_count: 10000,
        steering_rows: vec![
            SteeringRow {
                speed: 5.0,
                samples: vec![(-1.0, -0.15), (0.0, 0.0), (1.0, 0.15)],
                counts: vec![100; 3],
                rms_curvature: 0.0,
            },
            SteeringRow {
                speed: 10.0,
                samples: vec![(-1.0, -0.15), (0.0, 0.0), (1.0, 0.15)],
                counts: vec![100; 3],
                rms_curvature: 0.0,
            },
        ],
        delay_s: Some(0.1),
        steering_tau_s: Some(0.1),
        delay_correlation: Some(1.0),
        longitudinal: Some(LongitudinalModel {
            coefficients: [-0.1, 1.2, -4.0, 0.0, 0.0],
            training_rmse: 0.0,
            validation_rmse: 0.0,
            sample_count: 10000,
        }),
        accepted_for_control: true,
        dynamic_speed_range_mps: None,
        issues: vec![],
    }
}
#[cfg(test)]
pub(crate) fn with_measured_envelope(mut calibration: VehicleCalibration) -> VehicleCalibration {
    let envelope = etsads_msgs::MeasuredEnvelope {
        tractor_front_from_rear_axle_m: 4.5,
        tractor_rear_overhang_m: 1.7,
        trailer: calibration
            .geometry
            .has_trailer
            .then_some(etsads_msgs::TrailerEnvelope {
                front_from_hitch_m: 1.2,
                rear_from_axle_m: 4.4,
            }),
    };
    calibration.geometry.measured_envelope = Some(envelope);
    calibration.front_bumper = None;
    calibration.envelope_measurement = Some(etsads_control::identification::EnvelopeMeasurement {
        geometry_fingerprint: calibration.geometry_fingerprint.clone().unwrap(),
        envelope,
        evidence_path: "synthetic test exterior model; not installed-game measurement".into(),
    });
    calibration
}
pub(crate) fn config(execution: ExecutionMode) -> RuntimeConfig {
    RuntimeConfig {
        forward_lateral: Default::default(),
        route_source: Default::default(),
        execution,
        vehicle_id: "unit-test-truck".into(),
        vehicle_configuration_verified: true,
        mode: DrivingMode::CommissioningCircle {
            radius_m: 50.0,
            speed_mps: 6.0,
            clockwise: false,
            verified_clear_area: vec![
                (-200.0, -200.0),
                (200.0, -200.0),
                (200.0, 200.0),
                (-200.0, 200.0),
            ],
        },
    }
}
pub(crate) fn snapshot(frame: u64) -> GameSnapshot {
    let ego = EgoState {
        header: Header {
            frame_id: frame,
            sim_time_us: frame * 16667,
            seq: frame,
        },
        speed: 6.0,
        ..Default::default()
    };
    let mut raw = TelemetryFrame {
        qpc_frequency: etsads_shm::qpc_frequency(),
        navigation_speed_limit_mps: 20.0,
        ..Default::default()
    };
    raw.header.qpc = etsads_shm::qpc();
    raw.trailers[0].connected = 1;
    raw.orientation_turns[0] = 0.75;
    raw.header.frame_id = frame;
    GameSnapshot {
        ego,
        raw,
        extension: None,
        config: Some(measured()),
        events: vec![],
        scene: Scene {
            ego,
            ..Default::default()
        },
        local_origin: [0.0; 3],
        origin_generation: 1,
        session_start_qpc: 1,
    }
}
pub(crate) fn low_speed_calibration() -> VehicleCalibration {
    let mut cal = calibration();
    cal.reference = None;
    cal.geometry_fingerprint = None;
    cal.longitudinal.as_mut().unwrap().coefficients = [0.0, 1.2, -4.0, 0.0, 0.0];
    let g = cal.geometry;
    cal.steering_rows = [0.0, 10.0, 20.0]
        .into_iter()
        .map(|speed| SteeringRow {
            speed,
            samples: (-20..=20)
                .map(|i| {
                    let axis = i as f64 / 20.0;
                    (axis, (axis * g.max_steer).tan() / g.wheelbase)
                })
                .collect(),
            counts: vec![100; 41],
            rms_curvature: 0.0,
        })
        .collect();
    cal
}
pub(crate) fn controlled_fixture(
    lane_change: bool,
    signal: bool,
    toll: bool,
) -> (RuntimeConfig, RegionGraph) {
    use controlled_route::{ControlledRouteMission, LaneChangeMission, TollStop};
    use etsads_map::{
        Lane,
        importer::{ImportedGraph, RegionPolicy},
        traffic::TrafficContext,
    };
    use parking_mission::VerifiedBoundary;
    let lane = |id, y| Lane {
        id,
        center: vec![
            Point3 { x: 0.0, y, z: 0.0 },
            Point3 {
                x: 300.0,
                y,
                z: 0.0,
            },
        ],
        width: 4.5,
        speed_limit: 8.0,
        successors: vec![],
        predecessors: vec![],
        left: if id == 1 && lane_change {
            Some(2)
        } else {
            None
        },
        right: if id == 2 { Some(1) } else { None },
        can_change_left: id == 1 && lane_change,
        can_change_right: id == 2,
        toll_cost: 0.0,
        difficulty_cost: 0.0,
    };
    let mut lanes = vec![lane(1, 0.0)];
    if lane_change {
        lanes.push(lane(2, 4.5));
    }
    let graph = LaneGraph::new(lanes).unwrap();
    let mut source_ids: BTreeMap<u64, String> = [(1, "fixture:road:0".into())].into();
    if lane_change {
        source_ids.insert(2, "fixture:road:1".into());
    }
    let mut document = serde_json::json!({"schema_version":1,"lanes":source_ids.values().map(|id|serde_json::json!({"id":id})).collect::<Vec<_>>(),"conflicts":[],"lane_controls":[],"signals":[],"signal_lanes":[],"stop_lines":[]});
    if signal {
        document["lane_controls"] = serde_json::json!([{"lane_id":"fixture:road:0","traffic_rule":"signal","control_kind":"signal","priority_modifier":null,"additive_priority":false,"signal_group_id":"signal:0","evidence_source":"synthetic"}]);
        document["signals"] = serde_json::json!([{"id":"signal:0","owner_uid":1,"semaphore_id":0,"type":"TrafficLight","x":60.0,"y":3.0,"z":0.0,"yaw":0.0,"profile":"fixture","cycle_delay_s":0.0,"interval_0":30.0,"interval_1":2.0,"interval_2":30.0,"interval_3":2.0}]);
        document["signal_lanes"] =
            serde_json::json!([{"signal_id":"signal:0","lane_id":"fixture:road:0"}]);
        document["stop_lines"] = serde_json::json!([{"id":"stop:0","lane_id":"fixture:road:0","signal_id":"signal:0","rule":"signal","x":60.0,"y":0.0,"z":0.0,"geometry_status":"synthetic_verified","control_kind":"signal"}]);
    }
    let traffic = TrafficContext::from_json(&document.to_string()).unwrap();
    let policy = RegionPolicy {
        region_id: "controlled-fixture".into(),
        evidence_path: "synthetic replay".into(),
        verified_lane_ids: source_ids.values().cloned().collect(),
        allow_navigation_width: false,
        require_fresh_telemetry_speed_limit: true,
        maximum_speed_mps: 8.0,
        maximum_lateral_error_m: 0.8,
    };
    let region = RegionGraph {
        imported: ImportedGraph {
            graph,
            runtime_to_source: source_ids,
            native_owners: [(1, 1), (2, 2)].into(),
        },
        policy,
        physical_boundaries_verified: true,
    };
    let mission = ControlledRouteMission {
        route: if lane_change { vec![1, 2] } else { vec![1] },
        maximum_speed_mps: 6.0,
        boundary: VerifiedBoundary {
            polygon: vec![(-30.0, -6.0), (330.0, -6.0), (330.0, 11.0), (-30.0, 11.0)],
            physical_boundaries_verified: true,
            evidence_path: "synthetic replay polygon".into(),
        },
        traffic,
        lane_changes: if lane_change {
            vec![LaneChangeMission {
                latest_start_s: None,
                from_lane: 1,
                to_lane: 2,
                earliest_s: 10.0,
                deadline_s: 150.0,
            }]
        } else {
            vec![]
        },
        tolls: if toll {
            vec![TollStop {
                gate: None,
                id: "fixture-toll".into(),
                lane_id: 1,
                barrier_position: Point3 {
                    x: 40.0,
                    y: 0.0,
                    z: 0.0,
                },
                stop_s: 40.0,
            }]
        } else {
            vec![]
        },
        live_manoeuvres_verified: false,
        roundabout_exits: vec![],
        control_evidence_path: "synthetic replay annotations".into(),
    };
    let mut cfg = config(ExecutionMode::Replay);
    cfg.mode = DrivingMode::ControlledRoute { mission };
    (cfg, region)
}
pub(crate) fn simulated_frame(
    state: TruckState,
    index: u64,
    signal: Option<etsads_game::ets2la::Signal>,
    paid: bool,
) -> GameSnapshot {
    let mut frame = snapshot(index);
    frame.ego = EgoState {
        header: Header {
            frame_id: index,
            sim_time_us: index * 20000,
            seq: index,
        },
        pose: state.pose,
        speed: state.speed,
        wheel_angle: state.steer,
        trailer_yaw: state.trailer_yaw,
        yaw_rate: state.speed * state.steer.tan() / 3.8,
        ..Default::default()
    };
    frame.scene.ego = frame.ego;
    frame.raw.position_scs = [state.pose.x, 0.0, -state.pose.y];
    frame.raw.orientation_turns[0] = ((state.pose.yaw - std::f64::consts::FRAC_PI_2)
        / std::f64::consts::TAU)
        .rem_euclid(1.0) as f32;
    frame.raw.speed_mps = state.speed as f32;
    frame.raw.wheel_steering[..2].fill((state.steer / std::f64::consts::TAU) as f32);
    let axle = state.trailer_axle(VehicleGeometry::default());
    frame.raw.trailers[0].position_scs = [axle.x, 0.0, -axle.y];
    frame.raw.trailers[0].orientation_turns[0] = ((state.trailer_yaw - std::f64::consts::FRAC_PI_2)
        / std::f64::consts::TAU)
        .rem_euclid(1.0) as f32;
    frame.raw.gear = 1;
    let mut extension = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
    extension.available = 10;
    extension.header.frame_id = index;
    if let Some(signal) = signal {
        extension.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
    }
    frame.extension = Some(extension);
    if paid {
        let mut event = etsads_msgs::protocol::GameEvent::default();
        event.header.frame_id = index;
        let name = b"player.tollgate.paid";
        event.name[..name.len()].copy_from_slice(name);
        frame.events.push(event);
    }
    frame
}
#[cfg(test)]
pub(crate) fn integrate_command(state: TruckState, command: Actuation) -> TruckState {
    state
        .step(
            etsads_msgs::VehicleGeometry::default(),
            command.steer * 0.6,
            (command.throttle * 1.2 - command.brake * 4.0).max(-state.speed / 0.02),
            0.02,
        )
        .unwrap()
}
