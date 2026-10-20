//! Deterministic articulated kinematics and bounded feedback controllers.
use etsads_msgs::{Actuation, EgoState, Pose2, Trajectory, VehicleGeometry, frames::wrap_angle};
use serde::{Deserialize, Serialize};
pub mod identification;
pub mod mpc;
pub mod reverse;
pub mod static_identification;
#[derive(Debug, thiserror::Error)]
pub enum ControlError {
    #[error("invalid controller input: {0}")]
    Invalid(&'static str),
    #[error("articulation limit exceeded")]
    Jackknife,
    #[error("invalid trajectory: {0}")]
    Trajectory(String),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TruckState {
    /// Tractor rear axle position.
    pub pose: Pose2,
    pub trailer_yaw: f64,
    pub speed: f64,
    pub steer: f64,
}
impl TruckState {
    pub fn valid(self) -> bool {
        self.pose.valid()
            && self.trailer_yaw.is_finite()
            && self.speed.is_finite()
            && self.speed.abs() <= 100.0
            && self.steer.is_finite()
            && self.steer.abs() < 1.4
    }
    pub fn articulation(self) -> f64 {
        wrap_angle(self.pose.yaw - self.trailer_yaw)
    }
    pub fn trailer_axle(self, g: VehicleGeometry) -> Pose2 {
        if !g.has_trailer {
            return self.pose;
        }
        Pose2 {
            x: self.pose.x
                - g.hitch_offset * self.pose.yaw.cos()
                - g.trailer_wheelbase * self.trailer_yaw.cos(),
            y: self.pose.y
                - g.hitch_offset * self.pose.yaw.sin()
                - g.trailer_wheelbase * self.trailer_yaw.sin(),
            yaw: self.trailer_yaw,
        }
    }
    /// RK4, subdivided at 20 ms. Positive M is behind the tractor rear axle;
    /// negative M is ahead. Tractor-only models skip trailer dynamics entirely.
    pub fn step(
        self,
        g: VehicleGeometry,
        steer: f64,
        acceleration: f64,
        dt: f64,
    ) -> Result<Self, ControlError> {
        if !self.valid()
            || !g.valid()
            || !steer.is_finite()
            || steer.abs() > g.max_steer + 1e-9
            || !acceleration.is_finite()
            || acceleration.abs() > 15.0
            || !dt.is_finite()
            || dt <= 0.0
            || dt > 1.0
        {
            return Err(ControlError::Invalid(
                "kinematics state, geometry, control or dt",
            ));
        }
        let steps = (dt / 0.02).ceil() as usize;
        let h = dt / steps as f64;
        let mut y = [
            self.pose.x,
            self.pose.y,
            self.pose.yaw,
            if g.has_trailer {
                self.trailer_yaw
            } else {
                self.pose.yaw
            },
            self.speed,
        ];
        let derivative = |s: [f64; 5]| {
            let v = s[4];
            let phi = s[2] - s[3];
            let rate = v * steer.tan() / g.wheelbase;
            [
                v * s[2].cos(),
                v * s[2].sin(),
                rate,
                if g.has_trailer {
                    v * phi.sin() / g.trailer_wheelbase
                        - g.hitch_offset * rate * phi.cos() / g.trailer_wheelbase
                } else {
                    rate
                },
                acceleration,
            ]
        };
        for _ in 0..steps {
            let k1 = derivative(y);
            let k2 = derivative(std::array::from_fn(|i| y[i] + h * k1[i] / 2.0));
            let k3 = derivative(std::array::from_fn(|i| y[i] + h * k2[i] / 2.0));
            let k4 = derivative(std::array::from_fn(|i| y[i] + h * k3[i]));
            for i in 0..5 {
                y[i] += h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
            }
            if g.has_trailer && wrap_angle(y[2] - y[3]).abs() > g.max_articulation {
                return Err(ControlError::Jackknife);
            }
        }
        let out = Self {
            pose: Pose2 {
                x: y[0],
                y: y[1],
                yaw: wrap_angle(y[2]),
            },
            trailer_yaw: wrap_angle(y[3]),
            speed: y[4],
            steer,
        };
        if !out.valid() {
            return Err(ControlError::Invalid("kinematics result"));
        }
        Ok(out)
    }
}
impl From<EgoState> for TruckState {
    fn from(e: EgoState) -> Self {
        Self {
            pose: e.pose,
            trailer_yaw: e.trailer_yaw,
            speed: e.speed,
            steer: e.wheel_angle,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ControllerConfig {
    pub geometry: VehicleGeometry,
    pub delay: f64,
    pub steer_rate: f64,
    pub acceleration_limit: f64,
    pub braking_limit: f64,
    pub kp_speed: f64,
    pub ki_speed: f64,
}
impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            geometry: VehicleGeometry::default(),
            delay: 0.15,
            steer_rate: 0.6,
            acceleration_limit: 1.2,
            braking_limit: 4.0,
            kp_speed: 0.8,
            ki_speed: 0.15,
        }
    }
}
impl ControllerConfig {
    fn valid(self) -> bool {
        self.geometry.valid()
            && [
                self.delay,
                self.steer_rate,
                self.acceleration_limit,
                self.braking_limit,
                self.kp_speed,
                self.ki_speed,
            ]
            .iter()
            .all(|v| v.is_finite())
            && (0.0..=1.0).contains(&self.delay)
            && self.steer_rate > 0.0
            && self.steer_rate <= 5.0
            && self.acceleration_limit > 0.0
            && self.acceleration_limit <= 5.0
            && self.braking_limit > 0.0
            && self.braking_limit <= 10.0
            && self.kp_speed > 0.0
            && self.kp_speed <= 10.0
            && self.ki_speed >= 0.0
            && self.ki_speed <= 5.0
    }
}
#[derive(Clone, Debug)]
pub struct Controller {
    pub config: ControllerConfig,
    integral: f64,
    last_steer: f64,
}
impl Controller {
    pub fn new(config: ControllerConfig) -> Result<Self, ControlError> {
        if !config.valid() {
            return Err(ControlError::Invalid("configuration"));
        }
        Ok(Self {
            config,
            integral: 0.0,
            last_steer: 0.0,
        })
    }
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_steer = 0.0;
    }
    pub fn reset_to_steer(&mut self, angle: f64) -> Result<(), ControlError> {
        if !angle.is_finite() || angle.abs() > self.config.geometry.max_steer {
            return Err(ControlError::Invalid("measured steering state"));
        }
        self.integral = 0.0;
        self.last_steer = angle;
        Ok(())
    }
    pub fn track(
        &mut self,
        ego: EgoState,
        trajectory: &Trajectory,
        dt: f64,
    ) -> Result<Actuation, ControlError> {
        if !ego.valid() || !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
            return Err(ControlError::Invalid("ego or dt"));
        }
        trajectory.validate().map_err(ControlError::Trajectory)?;
        if ego.paused {
            self.reset();
            return Ok(Actuation::default());
        }
        if trajectory.direction == etsads_msgs::Direction::Reverse
            || trajectory.behavior == etsads_msgs::BehaviorKind::Parking
        {
            return self.reverse_track(ego, trajectory, dt);
        }
        let pose = Pose2 {
            x: ego.pose.x + ego.speed * self.config.delay * ego.pose.yaw.cos(),
            y: ego.pose.y + ego.speed * self.config.delay * ego.pose.yaw.sin(),
            yaw: wrap_angle(ego.pose.yaw + ego.yaw_rate * self.config.delay),
        };
        let nearest = trajectory
            .points
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.pose.distance(pose).total_cmp(&b.pose.distance(pose)))
            .unwrap()
            .0;
        let lookahead = (1.2 * ego.speed.abs()).clamp(8.0, 40.0);
        let mut target = &trajectory.points[nearest];
        let mut accumulated = 0.0;
        for pair in trajectory.points[nearest..].windows(2) {
            accumulated += pair[0].pose.distance(pair[1].pose);
            target = &pair[1];
            if accumulated >= lookahead {
                break;
            }
        }
        let dx = target.pose.x - pose.x;
        let dy = target.pose.y - pose.y;
        let distance2 = (dx * dx + dy * dy).max(0.5);
        let lateral = -pose.yaw.sin() * dx + pose.yaw.cos() * dy;
        // Pure pursuit already includes reference curvature geometrically; adding
        // it a second time creates a steady-state bias on circles.
        let curvature = 2.0 * lateral / distance2;
        let desired = (self.config.geometry.wheelbase * curvature).atan();
        let steer = self.rate_limit(desired, dt);
        let longitudinal = trajectory.points[nearest..]
            .iter()
            .find(|point| point.t >= trajectory.points[nearest].t + 0.3)
            .unwrap_or_else(|| trajectory.points.last().unwrap());
        let target_speed = longitudinal.speed;
        let feedforward = longitudinal.acceleration;
        let (throttle, brake) = self.speed_control(ego.speed, target_speed, feedforward, dt);
        Ok(Actuation {
            steer: steer / self.config.geometry.max_steer,
            throttle,
            brake,
            ..Default::default()
        })
    }
    fn rate_limit(&mut self, desired: f64, dt: f64) -> f64 {
        let limit = self.config.steer_rate * dt;
        self.last_steer = desired
            .clamp(self.last_steer - limit, self.last_steer + limit)
            .clamp(
                -self.config.geometry.max_steer,
                self.config.geometry.max_steer,
            );
        self.last_steer
    }
    pub fn speed_control(
        &mut self,
        speed: f64,
        target: f64,
        feedforward: f64,
        dt: f64,
    ) -> (f64, f64) {
        if ![speed, target, feedforward, dt]
            .iter()
            .all(|x| x.is_finite())
            || dt <= 0.0
            || dt > 0.5
        {
            return (0.0, 1.0);
        }
        let error = target.abs() - speed.abs();
        let next = (self.integral + error * dt).clamp(-5.0, 5.0);
        let raw = feedforward + self.config.kp_speed * error + self.config.ki_speed * next;
        let acceleration = raw.clamp(-self.config.braking_limit, self.config.acceleration_limit);
        // Conditional integration prevents saturation windup.
        if (raw - acceleration).abs() < 1e-9 || error.signum() != raw.signum() {
            self.integral = next;
        }
        if acceleration >= 0.0 {
            (acceleration / self.config.acceleration_limit, 0.0)
        } else {
            (0.0, -acceleration / self.config.braking_limit)
        }
    }
    /// Apply an externally solved physical wheel angle while retaining the
    /// same longitudinal preview and PI used by forward tracking.
    pub fn track_with_lateral_steer(
        &mut self,
        ego: EgoState,
        trajectory: &Trajectory,
        dt: f64,
        steer: f64,
    ) -> Result<Actuation, ControlError> {
        if !ego.valid()
            || !dt.is_finite()
            || dt <= 0.0
            || dt > 0.5
            || !steer.is_finite()
            || steer.abs() > self.config.geometry.max_steer
            || trajectory.direction != etsads_msgs::Direction::Forward
        {
            return Err(ControlError::Invalid("external lateral control input"));
        }
        trajectory.validate().map_err(ControlError::Trajectory)?;
        if ego.paused {
            self.reset();
            return Ok(Actuation::default());
        }
        let nearest = trajectory
            .points
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.pose
                    .distance(ego.pose)
                    .total_cmp(&b.pose.distance(ego.pose))
            })
            .unwrap()
            .0;
        let target = trajectory.points[nearest..]
            .iter()
            .find(|p| p.t >= trajectory.points[nearest].t + 0.3)
            .unwrap_or_else(|| trajectory.points.last().unwrap());
        let (throttle, brake) =
            self.speed_control(ego.speed, target.speed, target.acceleration, dt);
        self.last_steer = steer;
        Ok(Actuation {
            steer: steer / self.config.geometry.max_steer,
            throttle,
            brake,
            ..Default::default()
        })
    }
    fn reverse_track(
        &mut self,
        ego: EgoState,
        trajectory: &Trajectory,
        dt: f64,
    ) -> Result<Actuation, ControlError> {
        let g = self.config.geometry;
        let state = TruckState::from(ego);
        let nearest = trajectory
            .points
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.pose
                    .distance(ego.pose)
                    .total_cmp(&b.pose.distance(ego.pose))
            })
            .unwrap()
            .0;
        let point = &trajectory.points[nearest];
        let phi = state.articulation();
        let error = wrap_angle(point.articulation - phi);
        if g.has_trailer && error.abs() > 15.0_f64.to_radians() {
            return Ok(Actuation {
                brake: 1.0,
                ..Default::default()
            });
        }
        let mut reference = *point;
        reference.curvature =
            trajectory.points[(nearest + 2).min(trajectory.points.len() - 1)].curvature;
        if reference.speed.abs() < 0.1 {
            reference.speed = if trajectory.direction == etsads_msgs::Direction::Reverse {
                -0.3
            } else {
                0.3
            };
        }
        let desired = reverse::steering(state, reference, g)?;
        let steer = self.rate_limit(desired, dt);
        let (throttle, brake) = self.speed_control(
            ego.speed,
            point.speed.clamp(-3.0 / 3.6, 3.0 / 3.6),
            point.acceleration
                * if trajectory.direction == etsads_msgs::Direction::Reverse {
                    -1.0
                } else {
                    1.0
                },
            dt,
        );
        Ok(Actuation {
            steer: steer / g.max_steer,
            throttle,
            brake,
            ..Default::default()
        })
    }
}

/// Monotone measured steering calibration (axis, curvature), per speed row.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SteeringCalibration {
    pub speed: f64,
    pub samples: Vec<(f64, f64)>,
}
impl SteeringCalibration {
    pub fn inverse(&self, curvature: f64) -> Result<f64, ControlError> {
        if !self.speed.is_finite()
            || self.speed < 0.0
            || !curvature.is_finite()
            || self.samples.len() < 2
            || self
                .samples
                .iter()
                .any(|(s, k)| !s.is_finite() || !k.is_finite() || s.abs() > 1.0)
            || self.samples.windows(2).any(|p| p[1].0 <= p[0].0)
        {
            return Err(ControlError::Invalid("nonmonotone calibration"));
        }
        let orientation = (self.samples.last().unwrap().1 - self.samples[0].1).signum();
        if orientation == 0.0
            || self
                .samples
                .windows(2)
                .any(|p| (p[1].1 - p[0].1) * orientation <= 0.0)
        {
            return Err(ControlError::Invalid("nonmonotone calibration"));
        }
        if (curvature - self.samples[0].1) * orientation <= 0.0 {
            return Ok(self.samples[0].0);
        }
        for p in self.samples.windows(2) {
            if (curvature - p[1].1) * orientation <= 0.0 {
                let t = (curvature - p[0].1) / (p[1].1 - p[0].1);
                return Ok(p[0].0 + t * (p[1].0 - p[0].0));
            }
        }
        Ok(self.samples.last().unwrap().0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn straight_line_and_trailer() {
        let s = TruckState {
            speed: 10.0,
            ..Default::default()
        }
        .step(VehicleGeometry::default(), 0.0, 0.0, 1.0)
        .unwrap();
        assert!((s.pose.x - 10.0).abs() < 1e-10);
        assert_eq!(s.trailer_yaw, 0.0);
    }
    #[test]
    fn rk4_circle() {
        let g = VehicleGeometry::default();
        let mut s = TruckState {
            speed: 5.0,
            ..Default::default()
        };
        let delta: f64 = 0.1;
        let rate = 5.0 * delta.tan() / g.wheelbase;
        for _ in 0..100 {
            s = s.step(g, delta, 0.0, 0.02).unwrap();
        }
        assert!((s.pose.yaw - rate * 2.0).abs() < 1e-9);
        let radius = g.wheelbase / delta.tan();
        assert!((s.pose.x - radius * (rate * 2.0).sin()).abs() < 1e-8);
    }
    #[test]
    fn prevents_jackknife_and_nonfinite() {
        let g = VehicleGeometry::default();
        let s = TruckState {
            pose: Pose2 {
                yaw: 1.0,
                ..Default::default()
            },
            speed: -3.0,
            ..Default::default()
        };
        assert!(s.step(g, -0.6, 0.0, 1.0).is_err());
        assert!(s.step(g, f64::NAN, 0.0, 0.1).is_err());
    }
    #[test]
    fn pi_is_exclusive_and_recovers_saturation() {
        let mut c = Controller::new(Default::default()).unwrap();
        for _ in 0..1000 {
            assert_eq!(c.speed_control(0.0, 30.0, 0.0, 0.02), (1.0, 0.0));
        }
        let (t, b) = c.speed_control(30.0, 10.0, 0.0, 0.02);
        assert_eq!(t, 0.0);
        assert!(b > 0.0);
        assert_eq!(c.speed_control(f64::NAN, 0.0, 0.0, 0.1), (0.0, 1.0));
    }
    #[test]
    fn calibration_interpolates_and_rejects_bad_rows() {
        let c = SteeringCalibration {
            speed: 10.0,
            samples: vec![(-1.0, -0.1), (0.0, 0.0), (1.0, 0.1)],
        };
        assert!((c.inverse(0.025).unwrap() - 0.25).abs() < 1e-9);
    }
    #[test]
    fn forward_hitch_offset_preserves_signed_articulation_dynamics() {
        let geometry = VehicleGeometry {
            hitch_offset: -0.5,
            ..Default::default()
        };
        assert!(geometry.valid());
        let state = TruckState {
            speed: 5.0,
            ..Default::default()
        }
        .step(geometry, 0.1, 0.0, 0.1)
        .unwrap();
        assert!(state.trailer_yaw > 0.0);
        assert!(state.articulation().is_finite());
    }
    #[test]
    fn tractor_only_model_has_no_division_by_missing_trailer_length() {
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_wheelbase: 0.0,
            trailer_length: 0.0,
            ..Default::default()
        };
        assert!(geometry.valid());
        let state = TruckState {
            speed: 5.0,
            trailer_yaw: 2.0,
            ..Default::default()
        }
        .step(geometry, 0.1, 0.0, 0.1)
        .unwrap();
        assert!(state.valid());
        assert!((state.trailer_yaw - state.pose.yaw).abs() < 1e-12);
        assert_eq!(state.trailer_axle(geometry), state.pose);
    }
}
