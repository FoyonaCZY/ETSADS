//! Bounded forward pull-up recovery. All candidate motion is swept against the
//! same measured tractor/trailer geometry and verified yard boundary.
use crate::parking_mission::{
    PlanningIntent, VerifiedBoundary, hold_trajectory, segment_trajectory,
};
use etsads_control::TruckState;
use etsads_msgs::{Direction, EgoState, OrientedBox, Pose2, Trajectory, VehicleGeometry};
use etsads_planning::{
    geometry::{box_in_polygon, footprints, overlaps, swept_collision, swept_in_polygon},
    parking::ParkingPoint,
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct ParkingRecoveryConfig {
    pub maximum_attempts: u8,
    pub maximum_distance_m: f64,
    pub maximum_duration_s: f64,
    pub speed_mps: f64,
    pub target_articulation_rad: f64,
    pub minimum_pull_distance_m: f64,
    pub steering_rate_rad_s: f64,
}
impl Default for ParkingRecoveryConfig {
    fn default() -> Self {
        Self {
            maximum_attempts: 2,
            maximum_distance_m: 15.0,
            maximum_duration_s: 45.0,
            speed_mps: 0.6,
            target_articulation_rad: 5_f64.to_radians(),
            minimum_pull_distance_m: 1.0,
            steering_rate_rad_s: 0.3,
        }
    }
}
impl ParkingRecoveryConfig {
    pub fn valid(self) -> bool {
        self.maximum_attempts <= 3
            && self.maximum_distance_m.is_finite()
            && (1.0..=30.0).contains(&self.maximum_distance_m)
            && self.maximum_duration_s.is_finite()
            && (5.0..=120.0).contains(&self.maximum_duration_s)
            && self.speed_mps.is_finite()
            && (0.2..=3.0 / 3.6).contains(&self.speed_mps)
            && self.target_articulation_rad.is_finite()
            && (1_f64.to_radians()..=10_f64.to_radians()).contains(&self.target_articulation_rad)
            && self.minimum_pull_distance_m.is_finite()
            && self.minimum_pull_distance_m >= 0.5
            && self.minimum_pull_distance_m <= self.maximum_distance_m
            && self.steering_rate_rad_s.is_finite()
            && (0.1..=0.6).contains(&self.steering_rate_rad_s)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParkingRecoveryPhase {
    Braking,
    AwaitingForwardGear,
    PullForward,
    StoppingAfterPull,
    Replanning,
    Resuming,
    StoppingFailed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingRecoveryDiagnostic {
    pub frame_id: u64,
    pub phase: ParkingRecoveryPhase,
    pub attempt: u8,
    pub travelled_m: f64,
    pub articulation_rad: f64,
    pub target_articulation_rad: f64,
    pub failure: Option<String>,
}
pub(crate) enum RecoveryProgress {
    Control(Trajectory, PlanningIntent),
    ReadyToReplan,
}
pub(crate) struct RecoveryContext<'a> {
    pub ego: EgoState,
    pub gear: i32,
    pub geometry: VehicleGeometry,
    pub boundary: &'a VerifiedBoundary,
    pub occupancy: &'a [OrientedBox],
}
pub(crate) struct RecoveryMachine {
    config: ParkingRecoveryConfig,
    phase: ParkingRecoveryPhase,
    attempt: u8,
    started: f64,
    travelled: f64,
    last_pose: Pose2,
    path: Vec<ParkingPoint>,
    gear_sent: Option<f64>,
    failure: Option<String>,
}
impl RecoveryMachine {
    pub fn failed(config: ParkingRecoveryConfig, ego: EgoState, attempt: u8, reason: &str) -> Self {
        let mut machine = Self::new(config, ego, attempt);
        machine.stop_failure(reason);
        machine
    }
    pub fn new(config: ParkingRecoveryConfig, ego: EgoState, attempt: u8) -> Self {
        Self {
            config,
            phase: ParkingRecoveryPhase::Braking,
            attempt,
            started: ego.header.sim_time_us as f64 / 1e6,
            travelled: 0.0,
            last_pose: ego.pose,
            path: vec![],
            gear_sent: None,
            failure: None,
        }
    }
    pub fn diagnostic(&self, ego: EgoState) -> ParkingRecoveryDiagnostic {
        ParkingRecoveryDiagnostic {
            frame_id: ego.header.frame_id,
            phase: self.phase,
            attempt: self.attempt,
            travelled_m: self.travelled,
            articulation_rad: TruckState::from(ego).articulation(),
            target_articulation_rad: self.config.target_articulation_rad,
            failure: self.failure.clone(),
        }
    }
    fn hold(&self, ego: EgoState, request: Option<Direction>, reason: &str) -> RecoveryProgress {
        RecoveryProgress::Control(
            hold_trajectory(ego),
            PlanningIntent {
                hold_brake: true,
                gear_request: request,
                parking_recovery: Some(self.diagnostic(ego)),
                reason: reason.into(),
                ..Default::default()
            },
        )
    }
    fn stop_failure(&mut self, reason: &str) {
        self.phase = ParkingRecoveryPhase::StoppingFailed;
        self.failure = Some(reason.into());
    }
    pub fn update(&mut self, context: RecoveryContext<'_>) -> Result<RecoveryProgress, String> {
        let RecoveryContext {
            ego,
            gear,
            geometry,
            boundary,
            occupancy,
        } = context;
        let now = ego.header.sim_time_us as f64 / 1e6;
        let state = TruckState::from(ego);
        self.travelled += self.last_pose.distance(ego.pose);
        self.last_pose = ego.pose;
        if now - self.started > self.config.maximum_duration_s {
            self.stop_failure("parking recovery duration budget exhausted");
        }
        if self.travelled > self.config.maximum_distance_m {
            self.stop_failure("parking recovery travelled-distance budget exhausted");
        }
        if state.articulation().abs() >= geometry.max_articulation {
            self.stop_failure("parking recovery cannot cross the hard articulation limit");
        }
        if self.phase == ParkingRecoveryPhase::PullForward && ego.speed > 3.0 / 3.6 {
            self.stop_failure("forward recovery exceeded the 3 km/h physical speed limit");
        }
        if self.phase == ParkingRecoveryPhase::PullForward
            && self.config.maximum_distance_m - self.travelled
                < ego.speed.powi(2) / (2.0 * 0.6) + 0.15
        {
            self.stop_failure("forward recovery distance budget cannot accommodate a safe stop");
        }
        match self.phase {
            ParkingRecoveryPhase::StoppingFailed => {
                if ego.speed.abs() < 0.05 {
                    return Err(self
                        .failure
                        .clone()
                        .unwrap_or_else(|| "parking recovery stopped".into()));
                }
                Ok(self.hold(
                    ego,
                    None,
                    "braking to a stop after recovery became infeasible",
                ))
            }
            ParkingRecoveryPhase::Braking => {
                if ego.speed.abs() >= 0.05 {
                    return Ok(self.hold(
                        ego,
                        None,
                        "stop before planning forward articulation recovery",
                    ));
                }
                self.path = match pull_up(state, geometry, self.config, boundary, occupancy) {
                    Some(path) => path,
                    None => {
                        self.stop_failure("no collision-free forward recovery fits the verified boundary and distance budget");
                        return Err(self.failure.clone().unwrap());
                    }
                };
                self.phase = ParkingRecoveryPhase::AwaitingForwardGear;
                let request = if gear > 0 {
                    None
                } else {
                    self.gear_sent = Some(now);
                    Some(Direction::Forward)
                };
                Ok(self.hold(
                    ego,
                    request,
                    "waiting for measured forward gear before recovery",
                ))
            }
            ParkingRecoveryPhase::AwaitingForwardGear => {
                if !remaining_safe(state, &self.path, geometry, boundary, occupancy) {
                    self.stop_failure(
                        "forward recovery path became occupied before gear acknowledgement",
                    );
                    return self.update(RecoveryContext {
                        ego,
                        gear,
                        geometry,
                        boundary,
                        occupancy,
                    });
                }
                if gear > 0 {
                    self.phase = ParkingRecoveryPhase::PullForward;
                } else {
                    if self.gear_sent.is_some_and(|sent| now - sent > 3.0) {
                        self.stop_failure(
                            "forward recovery gear was not acknowledged within 3 seconds",
                        );
                        return self.update(RecoveryContext {
                            ego,
                            gear,
                            geometry,
                            boundary,
                            occupancy,
                        });
                    }
                    return Ok(self.hold(
                        ego,
                        None,
                        "waiting for measured forward gear before recovery",
                    ));
                }
                Ok(RecoveryProgress::Control(
                    segment_trajectory(
                        &self.path,
                        Direction::Forward,
                        geometry,
                        ego.header,
                        self.config.speed_mps,
                    ),
                    PlanningIntent {
                        parking_recovery: Some(self.diagnostic(ego)),
                        reason: "executing verified forward pull-up recovery".into(),
                        ..Default::default()
                    },
                ))
            }
            ParkingRecoveryPhase::PullForward => {
                if !remaining_safe(state, &self.path, geometry, boundary, occupancy) {
                    self.stop_failure(
                        "forward recovery path became occupied or left verified bounds",
                    );
                    return self.update(RecoveryContext {
                        ego,
                        gear,
                        geometry,
                        boundary,
                        occupancy,
                    });
                }
                let end = self.path.last().unwrap().state;
                if state.pose.distance(end.pose) < 0.25
                    || (self.travelled >= self.config.minimum_pull_distance_m
                        && state.articulation().abs() <= self.config.target_articulation_rad)
                {
                    self.phase = ParkingRecoveryPhase::StoppingAfterPull;
                    return Ok(self.hold(
                        ego,
                        None,
                        "stop after straightening before parking replan",
                    ));
                }
                if gear <= 0 {
                    self.stop_failure("forward gear changed during pull-up recovery");
                    return self.update(RecoveryContext {
                        ego,
                        gear,
                        geometry,
                        boundary,
                        occupancy,
                    });
                }
                Ok(RecoveryProgress::Control(
                    segment_trajectory(
                        &self.path,
                        Direction::Forward,
                        geometry,
                        ego.header,
                        self.config.speed_mps,
                    ),
                    PlanningIntent {
                        parking_recovery: Some(self.diagnostic(ego)),
                        reason: "executing verified forward pull-up recovery".into(),
                        ..Default::default()
                    },
                ))
            }
            ParkingRecoveryPhase::StoppingAfterPull => {
                if ego.speed.abs() >= 0.05 {
                    return Ok(self.hold(
                        ego,
                        None,
                        "stop after straightening before parking replan",
                    ));
                }
                if state.articulation().abs() > self.config.target_articulation_rad {
                    self.stop_failure("forward recovery stopped without reaching the configured articulation target");
                    return Err(self.failure.clone().unwrap());
                }
                self.phase = ParkingRecoveryPhase::Replanning;
                Ok(self.hold(
                    ego,
                    None,
                    "straightening complete; parking replan starts from measured stopped state",
                ))
            }
            ParkingRecoveryPhase::Replanning => Ok(RecoveryProgress::ReadyToReplan),
            ParkingRecoveryPhase::Resuming => Err("invalid internal recovery phase".into()),
        }
    }
}
fn safe_transition(
    from: TruckState,
    to: TruckState,
    geometry: VehicleGeometry,
    boundary: &VerifiedBoundary,
    occupancy: &[OrientedBox],
) -> bool {
    let before = footprints(from, geometry);
    let after = footprints(to, geometry);
    before.iter().zip(after).all(|(a, b)| {
        box_in_polygon(b, &boundary.polygon)
            && swept_in_polygon(*a, b, &boundary.polygon)
            && !occupancy
                .iter()
                .any(|o| overlaps(b, *o, 0.5) || swept_collision(*a, b, *o, 0.5))
    })
}
fn remaining_safe(
    state: TruckState,
    path: &[ParkingPoint],
    geometry: VehicleGeometry,
    boundary: &VerifiedBoundary,
    occupancy: &[OrientedBox],
) -> bool {
    let Some(nearest) = path
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.state
                .pose
                .distance(state.pose)
                .total_cmp(&b.state.pose.distance(state.pose))
        })
        .map(|(i, _)| i)
    else {
        return false;
    };
    let mut previous = state;
    for point in path.iter().skip(nearest) {
        if !safe_transition(previous, point.state, geometry, boundary, occupancy) {
            return false;
        }
        previous = point.state;
    }
    true
}
fn pull_up(
    start: TruckState,
    geometry: VehicleGeometry,
    config: ParkingRecoveryConfig,
    boundary: &VerifiedBoundary,
    occupancy: &[OrientedBox],
) -> Option<Vec<ParkingPoint>> {
    if !geometry.has_trailer || !geometry.valid() || !config.valid() {
        return None;
    }
    let mut best: Option<Vec<ParkingPoint>> = None;
    for gain in [0.2, 0.3, 0.4] {
        let mut state = TruckState {
            speed: config.speed_mps,
            ..start
        };
        let mut path = vec![ParkingPoint {
            state: start,
            direction: Direction::Forward,
        }];
        let mut distance = 0.0;
        let brake_clearance = config.speed_mps.powi(2) / (2.0 * 0.6) + 0.15;
        let mut completed_at = None;
        while distance + config.speed_mps * 0.1 <= config.maximum_distance_m {
            let phi = state.articulation();
            if distance >= config.minimum_pull_distance_m
                && phi.abs() <= config.target_articulation_rad
                && completed_at.is_none()
            {
                completed_at = Some(distance);
            }
            let denominator = 1.0 + geometry.hitch_offset * phi.cos() / geometry.trailer_wheelbase;
            if denominator.abs() < 1e-6 {
                break;
            }
            let desired = (geometry.wheelbase
                * (phi.sin() / geometry.trailer_wheelbase - gain * phi)
                / denominator)
                .atan()
                .clamp(-geometry.max_steer, geometry.max_steer);
            let steer = desired.clamp(
                state.steer - config.steering_rate_rad_s * 0.1,
                state.steer + config.steering_rate_rad_s * 0.1,
            );
            let Ok(next) = state.step(geometry, steer, 0.0, 0.1) else {
                break;
            };
            if !safe_transition(state, next, geometry, boundary, occupancy) {
                break;
            }
            distance += state.pose.distance(next.pose);
            state = next;
            path.push(ParkingPoint {
                state,
                direction: Direction::Forward,
            });
            if completed_at.is_some_and(|finished| distance - finished >= brake_clearance) {
                if best
                    .as_ref()
                    .is_none_or(|current| path.len() < current.len())
                {
                    best = Some(path);
                }
                break;
            }
        }
    }
    best
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn new_occupancy_during_pull_up_latches_braking_until_stopped() {
        let geometry = VehicleGeometry::default();
        let mut ego = EgoState {
            trailer_yaw: -20_f64.to_radians(),
            ..Default::default()
        };
        let boundary = VerifiedBoundary {
            polygon: vec![
                (-100.0, -100.0),
                (100.0, -100.0),
                (100.0, 100.0),
                (-100.0, 100.0),
            ],
            physical_boundaries_verified: true,
            evidence_path: "synthetic-test".into(),
        };
        let mut machine = RecoveryMachine::new(ParkingRecoveryConfig::default(), ego, 1);
        let first = machine
            .update(RecoveryContext {
                ego,
                gear: -1,
                geometry,
                boundary: &boundary,
                occupancy: &[],
            })
            .unwrap();
        let RecoveryProgress::Control(_, intent) = first else {
            panic!("no gear request")
        };
        assert_eq!(intent.gear_request, Some(Direction::Forward));
        ego.header.sim_time_us = 100_000;
        machine
            .update(RecoveryContext {
                ego,
                gear: 1,
                geometry,
                boundary: &boundary,
                occupancy: &[],
            })
            .unwrap();
        assert_eq!(machine.phase, ParkingRecoveryPhase::PullForward);
        let end = footprints(machine.path.last().unwrap().state, geometry)[0];
        let obstacle = OrientedBox {
            pose: Pose2 {
                x: end.pose.x + (end.length / 2.0 - 0.2) * end.pose.yaw.cos(),
                y: end.pose.y + (end.length / 2.0 - 0.2) * end.pose.yaw.sin(),
                yaw: 0.0,
            },
            length: 0.2,
            width: 0.2,
        };
        ego.header.sim_time_us = 200_000;
        ego.speed = 0.4;
        let stopped = machine
            .update(RecoveryContext {
                ego,
                gear: 1,
                geometry,
                boundary: &boundary,
                occupancy: &[obstacle],
            })
            .unwrap();
        let RecoveryProgress::Control(_, intent) = stopped else {
            panic!("expected a bounded stop")
        };
        assert!(intent.hold_brake);
        assert_eq!(
            intent.parking_recovery.unwrap().phase,
            ParkingRecoveryPhase::StoppingFailed
        );
        ego.header.sim_time_us = 300_000;
        ego.speed = 0.0;
        let error = machine
            .update(RecoveryContext {
                ego,
                gear: 1,
                geometry,
                boundary: &boundary,
                occupancy: &[],
            })
            .err()
            .unwrap();
        assert!(error.contains("occupied"));
    }
}
