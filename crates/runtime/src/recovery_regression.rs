//! Cross-module parking recovery regressions with finite gear-acknowledgement
//! delay, actuator lag and explicit synthetic articulation disturbances.
use crate::{
    DrivingMode, ExecutionMode, Runtime, RuntimeConfig, RuntimeOutput, RuntimeStatus,
    parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary},
    parking_recovery::ParkingRecoveryPhase,
};
use etsads_control::TruckState;
use etsads_msgs::{Direction, Pose2, Trajectory, frames::wrap_angle};
use etsads_planning::geometry::{footprints, swept_in_polygon};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecoveryScenario {
    SuccessfulPullUp,
    NoSpace,
    MissingForwardAcknowledgement,
    AttemptLimit,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecoverySample {
    pub time_s: f64,
    pub state: TruckState,
    pub measured_gear: i32,
    pub output: RuntimeOutput,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecoveryRegression {
    pub evidence: String,
    pub scenario: RecoveryScenario,
    pub configuration: RuntimeConfig,
    pub passed: bool,
    pub completed: bool,
    pub failure: Option<String>,
    pub disturbances: usize,
    pub recovery_attempts: u8,
    pub gear_requests: Vec<(f64, Direction, f64)>,
    pub gear_acknowledgements: Vec<(f64, Direction)>,
    pub observed_phases: Vec<ParkingRecoveryPhase>,
    pub constraint_violations: usize,
    pub final_position_error_m: f64,
    pub final_heading_error_rad: f64,
    pub final_speed_mps: f64,
    pub samples: Vec<RecoverySample>,
    pub trajectories: Vec<Trajectory>,
}
pub fn run_recovery(scenario: RecoveryScenario) -> Result<RecoveryRegression, String> {
    let mut configuration = crate::replay::config(ExecutionMode::Replay);
    let target = Pose2 {
        x: -20.0,
        y: 0.0,
        yaw: 0.0,
    };
    let front = if matches!(scenario, RecoveryScenario::NoSpace) {
        5.0
    } else {
        40.0
    };
    let boundary = vec![(-60.0, -25.0), (front, -25.0), (front, 25.0), (-60.0, 25.0)];
    let mut recovery = crate::parking_recovery::ParkingRecoveryConfig::default();
    if matches!(scenario, RecoveryScenario::AttemptLimit) {
        recovery.maximum_attempts = 1;
    }
    configuration.mode = DrivingMode::Parking {
        mission: ParkingMission {
            target: ParkingTarget {
                source_id: "synthetic-recovery-slot".into(),
                pose: target,
                reference: SlotReference::TrailerAxle,
            },
            boundary: VerifiedBoundary {
                polygon: boundary.clone(),
                physical_boundaries_verified: true,
                evidence_path: "synthetic polygon fixture; not game boundary evidence".into(),
            },
            static_obstacles: vec![],
            max_expansions: 100_000,
            maximum_duration_s: 180.0,
            recovery,
        },
    };
    // Parse the actual serializable mission contract used by CLI/Replay.
    let configuration: RuntimeConfig =
        serde_json::from_str(&serde_json::to_string(&configuration).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    let calibration = crate::replay::low_speed_calibration();
    let geometry = calibration.geometry;
    let mut runtime =
        Runtime::new(configuration.clone(), calibration, None).map_err(|e| e.to_string())?;
    let mut state = TruckState::default();
    let mut gear = 0;
    let mut pending: Option<(Direction, u64)> = None;
    let mut resumed = false;
    let mut report=RecoveryRegression{evidence:"OFFLINE synthetic Runtime parking recovery; RK4 articulated model, 80 ms steering lag, 100 ms gear acknowledgement; not game acceptance".into(),scenario,configuration,
        passed:false,completed:false,failure:None,disturbances:0,recovery_attempts:0,gear_requests:vec![],gear_acknowledgements:vec![],observed_phases:vec![],constraint_violations:0,
        final_position_error_m:0.0,final_heading_error_rad:0.0,final_speed_mps:0.0,samples:vec![],trajectories:vec![]};
    for frame in 0..9000_u64 {
        if let Some((direction, at)) = pending
            && frame >= at
            && state.speed.abs() < 0.05
            && !(matches!(scenario, RecoveryScenario::MissingForwardAcknowledgement)
                && direction == Direction::Forward
                && report.disturbances > 0)
        {
            gear = if direction == Direction::Forward {
                1
            } else {
                -1
            };
            pending = None;
            report
                .gear_acknowledgements
                .push((frame as f64 * 0.02, direction));
        }
        if gear < 0
            && state.speed < -0.15
            && ((frame >= 100 && report.disturbances == 0)
                || (matches!(scenario, RecoveryScenario::AttemptLimit)
                    && resumed
                    && report.disturbances == 1))
        {
            state.pose.yaw = wrap_angle(state.pose.yaw + 10_f64.to_radians());
            state.trailer_yaw = wrap_angle(state.trailer_yaw - 10_f64.to_radians());
            report.disturbances += 1;
        }
        let mut snapshot = crate::replay::simulated_frame(state, frame, None, false);
        snapshot.raw.gear = gear;
        let output = runtime.step(&snapshot);
        if let Some(diagnostic) = &output.intent.parking_recovery {
            report.recovery_attempts = report.recovery_attempts.max(diagnostic.attempt);
            if !report.observed_phases.contains(&diagnostic.phase) {
                report.observed_phases.push(diagnostic.phase);
            }
            resumed |= diagnostic.phase == ParkingRecoveryPhase::Resuming;
        }
        if let Some(direction) = output.intent.gear_request {
            report
                .gear_requests
                .push((frame as f64 * 0.02, direction, state.speed));
            if state.speed.abs() > 0.05 || pending.is_some() {
                report.constraint_violations += 1;
            }
            pending = Some((direction, frame + 5));
        }
        if let Some(trajectory) = &output.trajectory {
            report.trajectories.push(trajectory.clone());
        }
        let terminal = matches!(
            output.status,
            RuntimeStatus::Completed | RuntimeStatus::Fault
        );
        if output.status == RuntimeStatus::Completed {
            report.completed = true;
        }
        if output.status == RuntimeStatus::Fault {
            report.failure = Some(output.reason.clone());
        }
        let command = output.actuation;
        report.samples.push(RecoverySample {
            time_s: frame as f64 * 0.02,
            state,
            measured_gear: gear,
            output,
        });
        if terminal {
            break;
        }
        let sign = if gear < 0 { -1.0 } else { 1.0 };
        let acceleration = sign * command.throttle * 1.2
            - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
        let steering = state.steer
            + (command.steer * geometry.max_steer - state.steer) * (1.0 - (-0.02_f64 / 0.08).exp());
        let next = state
            .step(geometry, steering, acceleration, 0.02)
            .map_err(|e| e.to_string())?;
        if footprints(state, geometry)
            .into_iter()
            .zip(footprints(next, geometry))
            .any(|(a, b)| !swept_in_polygon(a, b, &boundary))
        {
            report.constraint_violations += 1;
        }
        state = next;
    }
    let axle = state.trailer_axle(geometry);
    report.final_position_error_m = axle.distance(target);
    report.final_heading_error_rad = wrap_angle(axle.yaw - target.yaw).abs();
    report.final_speed_mps = state.speed;
    report.passed = report.constraint_violations == 0
        && report.disturbances > 0
        && match scenario {
            RecoveryScenario::SuccessfulPullUp => {
                report.completed
                    && report.failure.is_none()
                    && report.recovery_attempts == 1
                    && report
                        .observed_phases
                        .contains(&ParkingRecoveryPhase::PullForward)
                    && report
                        .observed_phases
                        .contains(&ParkingRecoveryPhase::Replanning)
                    && report
                        .observed_phases
                        .contains(&ParkingRecoveryPhase::Resuming)
                    && report.final_position_error_m < 0.5
                    && report.final_heading_error_rad < 3_f64.to_radians()
            }
            RecoveryScenario::NoSpace => {
                report
                    .failure
                    .as_ref()
                    .is_some_and(|e| e.contains("no collision-free forward recovery"))
                    && state.speed.abs() < 0.05
            }
            RecoveryScenario::MissingForwardAcknowledgement => {
                report
                    .failure
                    .as_ref()
                    .is_some_and(|e| e.contains("not acknowledged"))
                    && state.speed.abs() < 0.05
            }
            RecoveryScenario::AttemptLimit => {
                report.disturbances == 2
                    && report.recovery_attempts == 1
                    && report
                        .failure
                        .as_ref()
                        .is_some_and(|e| e.contains("attempt limit exhausted"))
                    && state.speed.abs() < 0.05
            }
        };
    Ok(report)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn runtime_recovers_articulation_then_replans_and_completes_reverse_docking() {
        let report = run_recovery(RecoveryScenario::SuccessfulPullUp).unwrap();
        assert!(
            report.passed,
            "failure={:?} phases={:?} violations={} target={} heading={} speed={}",
            report.failure,
            report.observed_phases,
            report.constraint_violations,
            report.final_position_error_m,
            report.final_heading_error_rad,
            report.final_speed_mps
        );
    }
    #[test]
    fn recovery_limits_stop_without_unconfirmed_or_infeasible_motion() {
        for scenario in [
            RecoveryScenario::NoSpace,
            RecoveryScenario::MissingForwardAcknowledgement,
            RecoveryScenario::AttemptLimit,
        ] {
            let report = run_recovery(scenario).unwrap();
            assert!(
                report.passed,
                "scenario={scenario:?} failure={:?} phases={:?} violations={} speed={}",
                report.failure,
                report.observed_phases,
                report.constraint_violations,
                report.final_speed_mps
            );
        }
    }
}
