//! Production Runtime planning regression; all data is synthetic and no IPC is
//! opened. The diagnostics are the same fields recorded by the driving host.
use super::*;

#[test]
fn both_runtime_route_planners_use_temporal_velocity_fit_not_raw_acceleration() {
    for controlled in [false, true] {
        let (mut config, region) = crate::replay::controlled_fixture(false, false, false);
        if !controlled {
            config.mode = DrivingMode::LaneFollow {
                route: vec![1],
                maximum_speed_mps: 6.0,
            };
        }
        let mut runtime = Runtime::new(config, crate::replay::calibration(), Some(region)).unwrap();
        let mut last = None;
        for n in 0..=65 {
            let t = n as f64 * 0.02;
            let state = TruckState {
                pose: Pose2 {
                    x: 20.0 + 6.0 * t,
                    y: 0.0,
                    yaw: 0.0,
                },
                trailer_yaw: 0.0,
                speed: 6.0,
                steer: 0.0,
            };
            let mut frame = crate::replay::simulated_frame(state, n, None, false);
            frame.scene.agents.push(Agent {
                id: 7,
                pose: Pose2 {
                    x: 80.0 + 5.0 * t + 0.4 * t * t,
                    y: 0.0,
                    yaw: 0.0,
                },
                z: 0.0,
                speed: 5.0 + 0.8 * t,
                acceleration: -20.0,
                length: 4.0,
                width: 2.0,
                lane_id: None,
                trailers: vec![],
            });
            let output = runtime.step(&frame);
            assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
            if output.planned_this_frame {
                assert!(output.trajectory.is_some());
                let diagnostic = output.prediction.unwrap();
                assert_eq!(diagnostic.frame_id, output.header.frame_id);
                last = Some(diagnostic);
            }
        }
        let last = last.unwrap();
        let actor = &last.agents[0];
        assert_eq!(actor.id, 7);
        assert!(actor.acceleration_fitted && actor.history_span_s > 0.8);
        assert!((actor.fitted_acceleration_mps2 - 0.8).abs() < 1e-8);
        assert_eq!(actor.lane_id, Some(1));
        assert!(!actor.explicit_turn_hint && !actor.target_route_hint);
        assert!(actor.velocity_samples <= 128);
        runtime.reset();
        assert_eq!(runtime.prediction.track_count(), 0);
    }
}

#[test]
fn runtime_predictions_keep_other_agent_fork_outside_the_ego_committed_route() {
    let (mut config, mut region) = crate::replay::controlled_fixture(false, false, false);
    let base = region.imported.graph.lanes[&1].clone();
    let mut incoming = base.clone();
    incoming.center[1].x = 100.0;
    incoming.successors = vec![2, 3];
    let branch = |id, y| {
        let mut lane = base.clone();
        lane.id = id;
        lane.predecessors = vec![1];
        lane.center = vec![
            Point3 {
                x: 100.0,
                y: 0.0,
                z: 0.0,
            },
            Point3 {
                x: 180.0,
                y,
                z: 0.0,
            },
        ];
        lane
    };
    region.imported.graph =
        LaneGraph::new(vec![incoming, branch(2, -40.0), branch(3, 40.0)]).unwrap();
    region.imported.runtime_to_source = [
        (1, "fixture:road:0".into()),
        (2, "fixture:road:right".into()),
        (3, "fixture:road:left".into()),
    ]
    .into();
    region.policy.verified_lane_ids = region
        .imported
        .runtime_to_source
        .values()
        .cloned()
        .collect();
    config.mode = DrivingMode::LaneFollow {
        route: vec![1, 2],
        maximum_speed_mps: 6.0,
    };
    let mut runtime = Runtime::new(config, crate::replay::calibration(), Some(region)).unwrap();
    assert_eq!(
        runtime.planning_graph.as_ref().unwrap().lanes[&1].successors,
        vec![2]
    );
    let mut recorded = None;
    for n in 0..=16 {
        let t = n as f64 * 0.02;
        let state = TruckState {
            pose: Pose2 {
                x: 20.0 + 6.0 * t,
                y: 0.0,
                yaw: 0.0,
            },
            trailer_yaw: 0.0,
            speed: 6.0,
            steer: 0.0,
        };
        let mut snapshot = crate::replay::simulated_frame(state, n, None, false);
        snapshot.scene.agents.push(Agent {
            id: 15,
            pose: Pose2 {
                x: 95.0 + 5.0 * t,
                y: 0.2 * t,
                yaw: 0.1 * t,
            },
            z: 0.0,
            speed: 5.0,
            acceleration: 0.0,
            length: 4.0,
            width: 2.0,
            lane_id: None,
            trailers: vec![],
        });
        let output = runtime.step(&snapshot);
        assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
        if output.prediction.is_some() {
            recorded = output.prediction;
        }
    }
    let report = recorded.unwrap();
    let target = &report.agents[0];
    assert_eq!(
        target.mode_probabilities.len(),
        2,
        "the other car's left branch must not disappear with the ego corridor pruning"
    );
    assert!(target.yaw_rate_hint > 0.05);
    assert!(target.mode_probabilities.iter().all(|p| *p > 0.0));
}
