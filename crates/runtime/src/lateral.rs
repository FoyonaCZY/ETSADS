//! Stateful forward control selection and honest per-frame MPC execution evidence.
use etsads_control::{
    Controller,
    mpc::{LateralMpc, MpcState},
};
use etsads_msgs::{Actuation, EgoState, Pose2, Trajectory, frames::wrap_angle};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ForwardLateralMode {
    #[default]
    PurePursuit,
    LtvMpc {
        lateral_acceleration_limit_mps2: f64,
        maximum_solve_time_ms: f64,
    },
}
impl ForwardLateralMode {
    pub fn valid(self) -> bool {
        match self {
            Self::PurePursuit => true,
            Self::LtvMpc {
                lateral_acceleration_limit_mps2: a,
                maximum_solve_time_ms: t,
            } => {
                a.is_finite()
                    && (0.2..=1.8).contains(&a)
                    && t.is_finite()
                    && (0.1..=50.0).contains(&t)
            }
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutedLateralController {
    PurePursuit,
    LtvMpc,
    ArticulatedLqr,
    Hold,
    MpcFailed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LateralControlDiagnostic {
    pub frame_id: u64,
    pub executed: ExecutedLateralController,
    pub qp_attempted: bool,
    pub qp_solved: bool,
    pub solve_seconds: Option<f64>,
    pub linearization_warm_started: bool,
    pub solver_warm_started: bool,
    pub requested_wheel_angle_rad: Option<f64>,
    pub failure: Option<String>,
}
impl LateralControlDiagnostic {
    pub(crate) fn direct(frame_id: u64, executed: ExecutedLateralController) -> Self {
        Self {
            frame_id,
            executed,
            qp_attempted: false,
            qp_solved: false,
            solve_seconds: None,
            linearization_warm_started: false,
            solver_warm_started: false,
            requested_wheel_angle_rad: None,
            failure: None,
        }
    }
}
#[derive(Default)]
pub(crate) struct MpcTracker {
    rates: Vec<f64>,
    last_time: Option<u64>,
}
impl MpcTracker {
    pub fn reset(&mut self) {
        self.rates.clear();
        self.last_time = None;
    }
    pub fn track(
        &mut self,
        controller: &mut Controller,
        ego: EgoState,
        trajectory: &Trajectory,
        dt: f64,
        limit: f64,
        budget_ms: f64,
    ) -> Result<(Actuation, LateralControlDiagnostic), LateralControlDiagnostic> {
        let mut diagnostic = LateralControlDiagnostic::direct(
            ego.header.frame_id,
            ExecutedLateralController::MpcFailed,
        );
        let fail = |mut evidence: LateralControlDiagnostic, message: String| {
            evidence.failure = Some(message);
            evidence
        };
        if !ego.valid() || !dt.is_finite() || !(0.0..=0.1).contains(&dt) || dt == 0.0 {
            self.reset();
            return Err(fail(
                diagnostic,
                "invalid MPC physical interval/state".into(),
            ));
        }
        if let Err(error) = trajectory.validate() {
            self.reset();
            return Err(fail(diagnostic, error));
        }
        let delay = controller.config.delay;
        let angle = ego.pose.yaw + ego.yaw_rate * delay;
        let pose = if ego.yaw_rate.abs() > 1e-6 {
            Pose2 {
                x: ego.pose.x + ego.speed / ego.yaw_rate * (angle.sin() - ego.pose.yaw.sin()),
                y: ego.pose.y + ego.speed / ego.yaw_rate * (ego.pose.yaw.cos() - angle.cos()),
                yaw: wrap_angle(angle),
            }
        } else {
            Pose2 {
                x: ego.pose.x + ego.speed * delay * ego.pose.yaw.cos(),
                y: ego.pose.y + ego.speed * delay * ego.pose.yaw.sin(),
                yaw: ego.pose.yaw,
            }
        };
        let (reference, t) = nearest_reference(trajectory, pose);
        let error = -(pose.x - reference.x) * reference.yaw.sin()
            + (pose.y - reference.y) * reference.yaw.cos();
        let curvatures: Vec<_> = (0..20)
            .map(|i| curvature_at(trajectory, t + i as f64 * 0.1))
            .collect();
        let warm = self
            .last_time
            .and_then(|last| ego.header.sim_time_us.checked_sub(last))
            .filter(|elapsed| *elapsed > 0 && *elapsed <= 100_000)
            .filter(|_| self.rates.len() == 20);
        let nominal = warm.map(|elapsed| {
            let shift = elapsed as f64 / 100_000.0;
            (0..20)
                .map(|i| {
                    let at = i as f64 + shift;
                    let lo = (at.floor() as usize).min(19);
                    let hi = (lo + 1).min(19);
                    (self.rates[lo] + (self.rates[hi] - self.rates[lo]) * at.fract())
                        .clamp(-controller.config.steer_rate, controller.config.steer_rate)
                })
                .collect::<Vec<_>>()
        });
        diagnostic.linearization_warm_started = nominal.is_some();
        diagnostic.qp_attempted = true;
        let solver = LateralMpc {
            geometry: controller.config.geometry,
            dt: 0.1,
            steer_rate_limit: controller.config.steer_rate,
            lateral_acceleration_limit: limit,
        };
        let started = std::time::Instant::now();
        let result = solver.solve_with_nominal(
            MpcState {
                lateral_error: error,
                heading_error: wrap_angle(pose.yaw - reference.yaw),
                steer: ego.wheel_angle,
                speed: ego.speed.max(0.0),
            },
            &curvatures,
            nominal.as_deref(),
        );
        diagnostic.solve_seconds = Some(started.elapsed().as_secs_f64());
        let solution = match result {
            Ok(s) => s,
            Err(e) => {
                self.reset();
                return Err(fail(diagnostic, e.to_string()));
            }
        };
        diagnostic.qp_solved = true;
        if diagnostic.solve_seconds.unwrap() * 1000.0 > budget_ms {
            self.reset();
            return Err(fail(
                diagnostic,
                "MPC exceeded configured solve-time budget".into(),
            ));
        }
        let steer_limit = controller.config.geometry.max_steer.min(
            (limit * controller.config.geometry.wheelbase / ego.speed.powi(2).max(0.1)).atan(),
        );
        if ego.wheel_angle.abs() > steer_limit + controller.config.steer_rate * dt + 1e-6 {
            self.reset();
            return Err(fail(
                diagnostic,
                "MPC cannot reach lateral-acceleration bound within this physical step".into(),
            ));
        }
        let steer = (ego.wheel_angle
            + solution.rates[0].clamp(-controller.config.steer_rate, controller.config.steer_rate)
                * dt)
            .clamp(-steer_limit, steer_limit);
        let command = match controller.track_with_lateral_steer(ego, trajectory, dt, steer) {
            Ok(c) => c,
            Err(e) => {
                self.reset();
                return Err(fail(diagnostic, e.to_string()));
            }
        };
        self.rates = solution.rates;
        self.last_time = Some(ego.header.sim_time_us);
        diagnostic.executed = ExecutedLateralController::LtvMpc;
        diagnostic.requested_wheel_angle_rad = Some(steer);
        Ok((command, diagnostic))
    }
}
fn nearest_reference(trajectory: &Trajectory, pose: Pose2) -> (Pose2, f64) {
    let mut result = (trajectory.points[0].pose, trajectory.points[0].t);
    let mut distance = f64::INFINITY;
    for pair in trajectory.points.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let dx = b.pose.x - a.pose.x;
        let dy = b.pose.y - a.pose.y;
        let fraction = (((pose.x - a.pose.x) * dx + (pose.y - a.pose.y) * dy)
            / (dx * dx + dy * dy).max(1e-12))
        .clamp(0.0, 1.0);
        let reference = Pose2 {
            x: a.pose.x + fraction * dx,
            y: a.pose.y + fraction * dy,
            yaw: wrap_angle(a.pose.yaw + fraction * wrap_angle(b.pose.yaw - a.pose.yaw)),
        };
        let d = pose.distance(reference);
        if d < distance {
            distance = d;
            result = (reference, a.t + fraction * (b.t - a.t));
        }
    }
    result
}
fn curvature_at(trajectory: &Trajectory, t: f64) -> f64 {
    for pair in trajectory.points.windows(2) {
        if t <= pair[1].t {
            let a = ((t - pair[0].t) / (pair[1].t - pair[0].t)).clamp(0.0, 1.0);
            return pair[0].curvature + a * (pair[1].curvature - pair[0].curvature);
        }
    }
    trajectory.points.last().unwrap().curvature
}
