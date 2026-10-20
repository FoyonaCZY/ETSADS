//! Reproducible offline evaluation. All metrics in this crate are simulation
//! metrics and cannot establish ETS2 on-road acceptance.
pub mod parking;
pub mod recording;
use etsads_control::{Controller, ControllerConfig, TruckState};
use etsads_msgs::{
    Actuation, BehaviorKind, Direction, EgoState, Header, OrientedBox, Pose2, TrajPoint,
    Trajectory, VehicleGeometry, frames::wrap_angle,
};
use etsads_planning::{
    PlanningError,
    behavior::{SignalDecision, TollBooth, TollPhase, signal_decision},
    geometry::{footprints, overlaps},
    speed::IdmConfig,
};
use serde::{Deserialize, Serialize};
#[derive(Debug, thiserror::Error)]
pub enum SimError {
    #[error("invalid simulation config")]
    Invalid,
    #[error(transparent)]
    Control(#[from] etsads_control::ControlError),
    #[error(transparent)]
    Planning(#[from] PlanningError),
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct HighwayConfig {
    pub duration_s: f64,
    pub dt: f64,
    pub radius_m: f64,
    pub target_speed_mps: f64,
    pub initial_offset_m: f64,
    pub lead_vehicle: bool,
}
impl Default for HighwayConfig {
    fn default() -> Self {
        Self {
            duration_s: 120.0,
            dt: 0.02,
            radius_m: 350.0,
            target_speed_mps: 18.0,
            initial_offset_m: 0.4,
            lead_vehicle: false,
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct SimSample {
    pub time_s: f64,
    pub ego: Pose2,
    pub trailer: Pose2,
    pub speed_mps: f64,
    pub reference: Pose2,
    pub lateral_error_m: f64,
    pub command: Actuation,
    pub lead: Option<Pose2>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimulationReport {
    pub scenario: String,
    pub evidence: String,
    pub duration_s: f64,
    pub distance_m: f64,
    pub rms_lateral_error_m: f64,
    pub max_lateral_error_m: f64,
    pub min_headway_s: Option<f64>,
    pub max_lateral_acceleration_mps2: f64,
    pub max_jerk_mps3: f64,
    pub collision_count: u32,
    pub interventions: u32,
    pub success: bool,
    pub samples: Vec<SimSample>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AcceptanceCheck {
    pub stage: String,
    pub name: String,
    pub passed: bool,
    pub measured: f64,
    pub threshold: f64,
    pub unit: String,
    pub evidence: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SuiteReport {
    pub schema_version: u32,
    pub evidence: String,
    pub all_passed: bool,
    pub checks: Vec<AcceptanceCheck>,
    pub scenarios: Vec<SimulationReport>,
    pub parking_attempts: usize,
    pub parking_successes: usize,
    pub parking_closed_loop: parking::ParkingSuite,
    pub runtime_replay: etsads_runtime::replay::RuntimeReplaySuite,
}
fn circle_pose(progress: f64, radius: f64) -> Pose2 {
    let a = progress / radius;
    Pose2 {
        x: radius * a.sin(),
        y: radius * (1.0 - a.cos()),
        yaw: wrap_angle(a),
    }
}
fn reference(
    ego: TruckState,
    radius: f64,
    target: f64,
    acceleration: f64,
    header: Header,
) -> Trajectory {
    let angle = (ego.pose.x).atan2(radius - ego.pose.y);
    let progress = angle * radius;
    let v = target.max(0.05);
    let points = (0..=120)
        .map(|i| {
            let t = i as f64 * 0.1;
            TrajPoint {
                t,
                pose: circle_pose(progress + v * t, radius),
                speed: target,
                acceleration,
                curvature: 1.0 / radius,
                articulation: 0.0,
            }
        })
        .collect();
    Trajectory {
        header,
        points,
        direction: Direction::Forward,
        behavior: BehaviorKind::LaneFollow,
    }
}
pub fn run_highway(config: HighwayConfig) -> Result<SimulationReport, SimError> {
    let c = config;
    if ![
        c.duration_s,
        c.dt,
        c.radius_m,
        c.target_speed_mps,
        c.initial_offset_m,
    ]
    .iter()
    .all(|v| v.is_finite())
        || !(1.0..=3600.0).contains(&c.duration_s)
        || !(0.005..=0.1).contains(&c.dt)
        || c.radius_m < 60.0
        || c.radius_m > 10000.0
        || c.target_speed_mps <= 0.0
        || c.target_speed_mps > 35.0
        || c.initial_offset_m.abs() > 2.0
    {
        return Err(SimError::Invalid);
    }
    let geometry = VehicleGeometry::default();
    let mut controller = Controller::new(ControllerConfig {
        delay: 0.12,
        ..Default::default()
    })?;
    let mut state = TruckState {
        pose: Pose2 {
            y: c.initial_offset_m,
            ..Default::default()
        },
        speed: c.target_speed_mps,
        ..Default::default()
    };
    let mut lead_progress = 100.0;
    let mut lead_speed = c.target_speed_mps;
    let mut ego_progress = 0.0;
    let mut last_angle = 0.0;
    let mut last_acceleration = 0.0;
    let mut actuator_steer = 0.0;
    let mut sum_error2 = 0.0;
    let mut max_error: f64 = 0.0;
    let mut max_lateral: f64 = 0.0;
    let mut max_jerk: f64 = 0.0;
    let mut distance = 0.0;
    let mut min_headway = f64::INFINITY;
    let mut collisions = 0;
    let mut collided = false;
    let mut samples = Vec::new();
    let steps = (c.duration_s / c.dt).ceil() as usize;
    for i in 0..steps {
        let t = i as f64 * c.dt;
        let angle = state.pose.x.atan2(c.radius_m - state.pose.y);
        ego_progress += wrap_angle(angle - last_angle) * c.radius_m;
        last_angle = angle;
        let gap = lead_progress - ego_progress - 12.0;
        let lead = if c.lead_vehicle {
            Some((gap, lead_speed))
        } else {
            None
        };
        let idm = IdmConfig {
            desired_speed: c.target_speed_mps,
            ..Default::default()
        };
        let desired_accel = if c.lead_vehicle {
            idm.acceleration(state.speed, lead)?
        } else {
            0.0
        };
        let target_speed = if c.lead_vehicle {
            (state.speed + desired_accel).clamp(0.0, c.target_speed_mps)
        } else {
            c.target_speed_mps
        };
        let header = Header {
            sim_time_us: (t * 1e6).round() as u64,
            frame_id: i as u64,
            seq: i as u64,
        };
        let trajectory = reference(state, c.radius_m, target_speed, desired_accel, header);
        let ego = EgoState {
            header,
            pose: state.pose,
            speed: state.speed,
            acceleration: last_acceleration,
            yaw_rate: state.speed * state.steer.tan() / geometry.wheelbase,
            wheel_angle: state.steer,
            trailer_yaw: state.trailer_yaw,
            ..Default::default()
        };
        let command = controller.track(ego, &trajectory, c.dt)?;
        if !command.valid() {
            return Err(SimError::Invalid);
        }
        let requested_accel = command.throttle * 1.2 - command.brake * 4.0;
        let acceleration = requested_accel.max(-state.speed / c.dt);
        actuator_steer +=
            (command.steer * geometry.max_steer - actuator_steer) * (1.0 - (-c.dt / 0.12).exp());
        let next = state.step(geometry, actuator_steer, acceleration, c.dt)?;
        distance += state.pose.distance(next.pose);
        state = next;
        let error = c.radius_m - (state.pose.x).hypot(state.pose.y - c.radius_m);
        sum_error2 += error * error;
        max_error = max_error.max(error.abs());
        max_lateral = max_lateral
            .max((state.speed.powi(2) * actuator_steer.tan() / geometry.wheelbase).abs());
        if i > 0 {
            max_jerk = max_jerk.max(((acceleration - last_acceleration) / c.dt).abs());
        }
        last_acceleration = acceleration;
        let lead_pose = if c.lead_vehicle {
            let desired = if (25.0..40.0).contains(&t) {
                0.0
            } else if t >= 40.0 {
                c.target_speed_mps * 0.8
            } else {
                c.target_speed_mps * 0.7
            };
            let lead_acceleration = ((desired - lead_speed) * 0.6).clamp(-1.5, 1.0);
            lead_speed = (lead_speed + lead_acceleration * c.dt).max(0.0);
            lead_progress += lead_speed * c.dt;
            let p = circle_pose(lead_progress, c.radius_m);
            let obstacle = OrientedBox {
                pose: p,
                length: 6.0,
                width: 2.5,
            };
            let collision = footprints(state, geometry)
                .iter()
                .any(|b| overlaps(*b, obstacle, 0.0));
            if collision && !collided {
                collisions += 1;
            }
            collided = collision;
            if state.speed > 2.0 {
                min_headway = min_headway.min(gap / state.speed);
            }
            Some(p)
        } else {
            None
        };
        if i % ((0.1 / c.dt).round() as usize).max(1) == 0 {
            samples.push(SimSample {
                time_s: t,
                ego: state.pose,
                trailer: state.trailer_axle(geometry),
                speed_mps: state.speed,
                reference: circle_pose(ego_progress, c.radius_m),
                lateral_error_m: error,
                command,
                lead: lead_pose,
            });
        }
    }
    let rms = (sum_error2 / steps as f64).sqrt();
    Ok(SimulationReport {
        scenario: if c.lead_vehicle {
            "circle_acc_stop_and_go"
        } else {
            "circle_lane_keep"
        }
        .into(),
        evidence: "deterministic synthetic articulated model; not ETS2 road evidence".into(),
        duration_s: steps as f64 * c.dt,
        distance_m: distance,
        rms_lateral_error_m: rms,
        max_lateral_error_m: max_error,
        min_headway_s: if min_headway.is_finite() {
            Some(min_headway)
        } else {
            None
        },
        max_lateral_acceleration_mps2: max_lateral,
        max_jerk_mps3: max_jerk,
        collision_count: collisions,
        interventions: 0,
        success: rms < 0.3 && collisions == 0,
        samples,
    })
}
pub fn run_suite() -> Result<SuiteReport, SimError> {
    let lane = run_highway(HighwayConfig::default())?;
    let acc = run_highway(HighwayConfig {
        lead_vehicle: true,
        ..Default::default()
    })?;
    let mut checks = Vec::new();
    let mut add =
        |stage: &str, name: &str, passed: bool, measured: f64, threshold: f64, unit: &str| {
            checks.push(AcceptanceCheck {
                stage: stage.into(),
                name: name.into(),
                passed,
                measured,
                threshold,
                unit: unit.into(),
                evidence: "synthetic offline regression; does not satisfy live-game exit gate"
                    .into(),
            })
        };
    add(
        "P0",
        "articulated circle tracking",
        lane.rms_lateral_error_m < 0.5,
        lane.rms_lateral_error_m,
        0.5,
        "m RMS",
    );
    add(
        "P1",
        "lane tracking accuracy",
        lane.rms_lateral_error_m < 0.3,
        lane.rms_lateral_error_m,
        0.3,
        "m RMS",
    );
    add(
        "P1",
        "ACC collision count",
        acc.collision_count == 0,
        acc.collision_count as f64,
        0.0,
        "collisions",
    );
    if let Some(headway) = acc.min_headway_s {
        add(
            "P1",
            "ACC minimum moving headway",
            headway >= 1.8,
            headway,
            1.8,
            "s",
        );
    }
    let mut toll = TollBooth::default();
    let mut toll_actions = 0;
    let mut last = TollPhase::Approach;
    for i in 0..100 {
        let time = i as f64 * 0.1;
        let a = toll.update(
            time,
            if time < 5.0 { 3.0 } else { -3.0 },
            0.0,
            if time < 3.0 {
                etsads_msgs::BarrierState::Closed
            } else {
                etsads_msgs::BarrierState::Open
            },
            time > 6.0,
        )?;
        toll_actions += usize::from(a.activate);
        last = a.phase;
    }
    add(
        "P2",
        "toll interaction and paid-event state machine",
        last == TollPhase::Complete && toll_actions == 1,
        toll_actions as f64,
        1.0,
        "activation pulses",
    );
    let decision = signal_decision(etsads_msgs::SignalState::Red, 100.0, 15.0, 0.0, 8.0)?;
    add(
        "P3",
        "red signal stop decision",
        decision == SignalDecision::Stop,
        if decision == SignalDecision::Stop {
            1.0
        } else {
            0.0
        },
        1.0,
        "pass",
    );
    let parking_closed_loop = parking::run_parking_suite();
    let parking_attempts = parking_closed_loop.attempts;
    let parking_successes = parking_closed_loop.successes;
    let rate = parking_closed_loop.success_rate;
    add(
        "P4",
        "obstacle docking with gear changes and actuator lag (offline)",
        rate >= 0.9,
        rate,
        0.9,
        "fraction of 50 synthetic closed-loop trials",
    );
    let runtime_replay = etsads_runtime::replay::run_replay_suite();
    for case in &runtime_replay.cases {
        add(
            &case.stage,
            &case.name,
            case.passed,
            if case.passed { 1.0 } else { 0.0 },
            1.0,
            "completed constrained runtime replay",
        );
    }
    Ok(SuiteReport{schema_version:1,evidence:"OFFLINE ONLY: no game fines, live intervention distance or hard-slot delivery success measured".into(),all_passed:checks.iter().all(|c|c.passed),checks,scenarios:vec![lane,acc],parking_attempts,parking_successes,parking_closed_loop,runtime_replay})
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn closed_loop_lane_tracking() {
        let report = run_highway(HighwayConfig {
            duration_s: 60.0,
            ..Default::default()
        })
        .unwrap();
        assert!(
            report.rms_lateral_error_m < 0.3,
            "RMS {}",
            report.rms_lateral_error_m
        );
        assert!(report.max_lateral_acceleration_mps2 < 1.5);
        assert_eq!(report.collision_count, 0);
    }
    #[test]
    fn repeatability() {
        let c = HighwayConfig {
            duration_s: 3.0,
            ..Default::default()
        };
        let a = run_highway(c).unwrap();
        let b = run_highway(c).unwrap();
        assert_eq!(a.rms_lateral_error_m, b.rms_lateral_error_m);
        assert_eq!(a.distance_m, b.distance_m);
    }
    #[test]
    fn acc_stop_and_go_no_collision() {
        let r = run_highway(HighwayConfig {
            duration_s: 80.0,
            lead_vehicle: true,
            ..Default::default()
        })
        .unwrap();
        assert_eq!(r.collision_count, 0);
        assert!(
            r.min_headway_s.unwrap() > 1.8,
            "headway {:?}",
            r.min_headway_s
        );
    }
    #[test]
    fn config_rejects_nan() {
        assert!(
            run_highway(HighwayConfig {
                dt: f64::NAN,
                ..Default::default()
            })
            .is_err()
        );
    }
}
