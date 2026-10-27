//! The truck as the game drives it, fitted to recorded drives: the bridge's
//! input slew, the game's steering (absolute or relative), a kinematic
//! bicycle and an automatic gearbox with shift gaps.
use super::super::core::SteeringMap;
use etsads_msgs::{Pose2, frames::wrap_angle};
use serde::{Deserialize, Serialize};

/// Measured parameters. Defaults are fitted to r15–r17 (the user's 4×2
/// tractor without a trailer, relative steering at 0.4).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Profile {
    /// Rear axle to front axle (m): yaw rate over speed·tan(mean wheel
    /// angle) gave 3.77–3.79 from 0 to 15 m/s, no understeer.
    pub wheelbase_m: f64,
    /// Rear axle ahead of the SDK reference (m, negative: behind).
    pub rear_forward_m: f64,
    /// Relative steering: turning rate per unit input at a standstill (1/s);
    /// `None` steers absolutely.
    pub relative_rate_per_s: Option<f64>,
    /// Relative rate lost per m/s of speed (share).
    pub relative_falloff: f64,
    /// Lowest share of the relative rate left at speed.
    pub relative_min_factor: f64,
    /// Return toward the centre per m/s of speed (1/s per m/s).
    pub relative_centering: f64,
    /// An absolutely steered wheel follows the input with this lag (s).
    pub absolute_lag_s: f64,
    /// The bridge slews the steering input at the gate's rate (1/s).
    pub bridge_steer_rate: f64,
    /// Engine speed per m/s in each gear, lowest first (rpm·s/m).
    pub gear_rpm_per_mps: Vec<f64>,
    pub idle_rpm: f64,
    /// The starting gear's clutch slips below this engine speed.
    pub slip_rpm: f64,
    /// Upshift at this engine speed at full throttle, and at
    /// `upshift_light_rpm` with the throttle released (r17 shifted up two
    /// gears at 1200 rpm on lifting off).
    pub upshift_rpm: f64,
    pub upshift_light_rpm: f64,
    /// Downshift below this engine speed with the throttle released, and
    /// below `downshift_full_rpm` at full throttle.
    pub downshift_rpm: f64,
    pub downshift_full_rpm: f64,
    /// Traction is interrupted for this long while shifting (s).
    pub upshift_s: f64,
    pub downshift_s: f64,
    /// Full-throttle acceleration (m/s²) = `traction` · (ratio/100)^`traction_exponent`
    /// · torque share at the engine speed.
    pub traction: f64,
    pub traction_exponent: f64,
    /// Throttle response: traction grows with throttle^`throttle_curve`.
    pub throttle_curve: f64,
    /// Share of peak torque at idle.
    pub idle_torque: f64,
    /// Pedal response lags (s).
    pub throttle_lag_s: f64,
    pub brake_lag_s: f64,
    /// Deceleration at full brake (m/s²) and how it grows with the brake:
    /// `brake_per_unit` · brake^`brake_exponent`. Logged drives braked
    /// 1.3 m/s² at 0.05 but only 5 m/s² at 0.55: a light brake bites hard.
    pub brake_per_unit: f64,
    pub brake_exponent: f64,
    /// Rolling resistance (m/s²) and aerodynamic drag (1/m).
    pub rolling: f64,
    pub drag: f64,
    /// Engine braking in gear without throttle (m/s² per 1000 rpm).
    pub engine_brake: f64,
    /// The game's wheel angle over the steering table's at the same
    /// position: 1 when the table is exact (model error studies).
    pub steer_gain: f64,
}
impl Default for Profile {
    fn default() -> Self {
        Self {
            wheelbase_m: 3.78,
            rear_forward_m: -2.02,
            relative_rate_per_s: Some(0.4),
            relative_falloff: super::super::io::RELATIVE_SPEED_FALLOFF,
            relative_min_factor: 0.5,
            relative_centering: super::super::io::RELATIVE_CENTERING,
            absolute_lag_s: 0.1,
            bridge_steer_rate: 0.8,
            // r15–r17: engine speed over road speed clusters 1.28 apart.
            gear_rpm_per_mps: (0..12).map(|k| 460.0 / 1.28_f64.powi(k)).collect(),
            idle_rpm: 550.0,
            slip_rpm: 800.0,
            // Fitted by `sim-fit --fit` to r15–r17 (3 s windows).
            upshift_rpm: 1630.0,
            upshift_light_rpm: 1360.0,
            downshift_rpm: 600.0,
            downshift_full_rpm: 850.0,
            upshift_s: 0.45,
            downshift_s: 0.26,
            traction: 1.38,
            traction_exponent: 0.96,
            throttle_curve: 0.66,
            idle_torque: 0.56,
            throttle_lag_s: 0.19,
            brake_lag_s: 0.28,
            brake_per_unit: 14.2,
            brake_exponent: 1.0,
            rolling: 0.16,
            drag: 0.0006,
            engine_brake: 0.33,
            steer_gain: 1.0,
        }
    }
}
impl Profile {
    /// The parameters scaled by random factors within ±`amount` (some
    /// by more or less, as uncertain as their fits): a truck that differs
    /// from the model, to check the controller does not depend on an exact
    /// match.
    pub fn varied(&self, seed: u64, amount: f64) -> Self {
        let mut rng = super::traffic::Rng::new(seed);
        let mut f = |weight: f64| 1.0 + weight * amount * rng.range(-1.0, 1.0);
        Self {
            wheelbase_m: self.wheelbase_m * f(0.2),
            relative_rate_per_s: self.relative_rate_per_s.map(|r| r * f(1.0)),
            relative_falloff: self.relative_falloff * f(1.5),
            relative_centering: self.relative_centering * f(2.0),
            absolute_lag_s: self.absolute_lag_s * f(2.0),
            bridge_steer_rate: self.bridge_steer_rate * f(1.0),
            traction: self.traction * f(1.0),
            throttle_lag_s: self.throttle_lag_s * f(2.0),
            brake_lag_s: self.brake_lag_s * f(2.0),
            brake_per_unit: self.brake_per_unit * f(1.0),
            rolling: self.rolling * f(2.0),
            engine_brake: self.engine_brake * f(2.0),
            steer_gain: self.steer_gain * f(0.5),
            ..self.clone()
        }
    }
}
/// The bridge's inputs as the game applies them.
#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    /// Steering input: an absolute position, or with relative steering a
    /// turning rate (positive: the SDK's effective steering falls).
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
    /// Requested clutch; the game's automatic gearbox ignores it.
    pub clutch: f64,
}
/// A semi-trailer on the tractor's fifth wheel. Kinematic: its axle
/// group does not slip sideways, so in a turn it runs inside the tractor.
#[derive(Clone, Copy, Debug)]
pub struct Trailer {
    pub yaw: f64,
    /// Fifth wheel ahead of the tractor's rear axle (m).
    pub hitch_m: f64,
    /// Kingpin to the axle group (m).
    pub wheelbase_m: f64,
    /// Kingpin to the front and to the rear end (m), and width.
    pub front_m: f64,
    pub rear_m: f64,
    pub width: f64,
}
impl Trailer {
    /// A 13.6 m curtain-sider in line with the tractor.
    pub fn semi(yaw: f64) -> Self {
        Self {
            yaw,
            hitch_m: 0.4,
            wheelbase_m: 8.0,
            front_m: 1.4,
            rear_m: 13.0,
            width: 2.55,
        }
    }
    fn kingpin(&self, rear: Pose2) -> (f64, f64) {
        (
            rear.x + self.hitch_m * rear.yaw.cos(),
            rear.y + self.hitch_m * rear.yaw.sin(),
        )
    }
    /// Centre of the axle group.
    pub fn axle(&self, rear: Pose2) -> (f64, f64) {
        let (x, y) = self.kingpin(rear);
        (
            x - self.wheelbase_m * self.yaw.cos(),
            y - self.wheelbase_m * self.yaw.sin(),
        )
    }
    /// Its box: centre x, y, yaw, half length, half width.
    pub fn body(&self, rear: Pose2) -> [f64; 5] {
        let (x, y) = self.kingpin(rear);
        let mid = 0.5 * (self.front_m - self.rear_m);
        [
            x + mid * self.yaw.cos(),
            y + mid * self.yaw.sin(),
            self.yaw,
            0.5 * (self.front_m + self.rear_m),
            0.5 * self.width,
        ]
    }
    /// One step behind a tractor whose rear axle moves at `speed` and
    /// turns at `yaw_rate`, now heading `yaw`.
    fn follow(&mut self, yaw: f64, speed: f64, yaw_rate: f64, dt: f64) {
        let d = wrap_angle(yaw - self.yaw);
        let rate = (speed * d.sin() + self.hitch_m * yaw_rate * d.cos()) / self.wheelbase_m;
        self.yaw = wrap_angle(self.yaw + rate * dt);
    }
}
#[derive(Clone, Debug)]
pub struct Truck {
    /// Rear-axle pose (ENU).
    pub rear: Pose2,
    pub z: f64,
    /// Forward speed (m/s), never negative.
    pub speed: f64,
    /// Steering position (the absolute input it equals; the SDK's effective
    /// steering is its negative).
    pub position: f64,
    /// Steering input after the bridge's slew.
    pub applied: f64,
    pub gear: usize,
    /// Remaining traction gap of a shift in progress (s), and its target.
    shift: Option<(f64, usize)>,
    throttle: f64,
    brake: f64,
    pub rpm: f64,
    /// Mean front wheel angle (rad).
    pub wheel: f64,
    pub yaw_rate: f64,
    pub acceleration: f64,
    pub trailer: Option<Trailer>,
}
impl Truck {
    pub fn new(rear: Pose2, z: f64, speed: f64, position: f64) -> Self {
        Self {
            rear,
            z,
            speed: speed.max(0.0),
            position,
            applied: 0.0,
            gear: 0,
            shift: None,
            throttle: 0.0,
            brake: 0.0,
            rpm: 550.0,
            wheel: 0.0,
            yaw_rate: 0.0,
            acceleration: 0.0,
            trailer: None,
        }
    }
    /// With a semi-trailer in line behind it.
    pub fn hauling(mut self, trailer: bool) -> Self {
        self.trailer = trailer.then(|| Trailer::semi(self.rear.yaw));
        self
    }
    /// The SDK reference pose (the cab position the telemetry reports).
    pub fn sdk(&self, profile: &Profile) -> Pose2 {
        Pose2 {
            x: self.rear.x - profile.rear_forward_m * self.rear.yaw.cos(),
            y: self.rear.y - profile.rear_forward_m * self.rear.yaw.sin(),
            yaw: self.rear.yaw,
        }
    }
    /// The SDK's effective steering.
    pub fn effective(&self) -> f64 {
        -self.position
    }
    /// The gear whose engine speed suits `speed` best (after a teleport).
    pub fn select_gear(&mut self, profile: &Profile) {
        let ratios = &profile.gear_rpm_per_mps;
        self.gear = (0..ratios.len())
            .find(|&k| ratios[k] * self.speed < profile.upshift_rpm - 100.0)
            .unwrap_or(ratios.len() - 1);
        self.shift = None;
    }
    pub fn steer(&mut self, input: f64, profile: &Profile, map: &SteeringMap, dt: f64) {
        let step = profile.bridge_steer_rate * dt;
        self.applied = input
            .clamp(-1.0, 1.0)
            .clamp(self.applied - step, self.applied + step);
        let speed = self.speed.abs();
        self.position = match profile.relative_rate_per_s {
            Some(rate) => {
                let factor =
                    (1.0 - profile.relative_falloff * speed).max(profile.relative_min_factor);
                self.position
                    + factor
                        * (rate * self.applied - profile.relative_centering * speed * self.position)
                        * dt
            }
            None => {
                self.position
                    + (self.applied - self.position) * (dt / profile.absolute_lag_s.max(dt))
            }
        }
        .clamp(-1.0, 1.0);
        self.wheel = map.angle(self.position) * profile.steer_gain;
    }
    /// One frame of `dt` seconds on a road rising at `grade` (m/m).
    pub fn step(
        &mut self,
        input: Input,
        grade: f64,
        profile: &Profile,
        map: &SteeringMap,
        dt: f64,
    ) {
        self.steer(input.steer, profile, map, dt);
        self.longitudinal(input, grade, profile, dt);
        self.yaw_rate = self.speed * self.wheel.tan() / profile.wheelbase_m;
        let heading = self.rear.yaw + 0.5 * self.yaw_rate * dt;
        self.rear.x += self.speed * heading.cos() * dt;
        self.rear.y += self.speed * heading.sin() * dt;
        self.rear.yaw = wrap_angle(self.rear.yaw + self.yaw_rate * dt);
        if let Some(trailer) = &mut self.trailer {
            trailer.follow(self.rear.yaw, self.speed, self.yaw_rate, dt);
        }
    }
    pub fn longitudinal(&mut self, input: Input, grade: f64, profile: &Profile, dt: f64) {
        let lag = |state: f64, target: f64, tau: f64| state + (target - state) * (dt / tau.max(dt));
        self.throttle = lag(
            self.throttle,
            input.throttle.clamp(0.0, 1.0),
            profile.throttle_lag_s,
        );
        self.brake = lag(self.brake, input.brake.clamp(0.0, 1.0), profile.brake_lag_s);
        let ratios = &profile.gear_rpm_per_mps;
        // The gearbox: up above the shift speed, down below it; a stopped
        // truck starts again in the lowest gear. Traction pauses meanwhile.
        if let Some((left, to)) = &mut self.shift {
            *left -= dt;
            if *left <= 0.0 {
                self.gear = *to;
                self.shift = None;
            }
        } else if self.speed < 0.3 {
            self.gear = 0;
        } else {
            // Shift points move up with the throttle; a shift goes to the
            // gear that lands inside the band, skipping gears as the game
            // does (r17: 2 → 4 lifting off, 6 → 3 on the throttle at 17 km/h).
            let share = self.throttle.clamp(0.0, 1.0);
            let up = profile.upshift_light_rpm
                + (profile.upshift_rpm - profile.upshift_light_rpm) * share;
            let down = profile.downshift_rpm
                + (profile.downshift_full_rpm - profile.downshift_rpm) * share;
            let rpm = ratios[self.gear] * self.speed;
            let fits = |k: usize| {
                let r = ratios[k] * self.speed;
                r <= up - 100.0 && r >= down + 100.0
            };
            if rpm > up && self.gear + 1 < ratios.len() {
                let to = (self.gear + 1..ratios.len())
                    .rev()
                    .find(|&k| fits(k))
                    .unwrap_or(self.gear + 1);
                self.shift = Some((profile.upshift_s, to));
            } else if rpm < down && self.gear > 0 {
                let to = (0..self.gear).rev().find(|&k| fits(k)).unwrap_or(0);
                self.shift = Some((profile.downshift_s, to));
            }
        }
        // The automatic gearbox works its own clutch: over r15–r17 the
        // game's effective clutch stayed 0 whenever the bridge pressed it.
        let engaged = self.shift.is_none();
        let road_rpm = ratios[self.gear] * self.speed;
        self.rpm = if !engaged {
            self.rpm + (profile.idle_rpm - self.rpm) * (dt / 0.3).min(1.0)
        } else if self.gear == 0 && road_rpm < profile.slip_rpm {
            // The clutch slips: the engine runs at its take-up speed.
            if self.throttle > 0.02 {
                profile.slip_rpm
            } else {
                profile.idle_rpm.max(road_rpm)
            }
        } else {
            road_rpm.max(profile.idle_rpm)
        };
        let traction = if engaged {
            profile.traction
                * (ratios[self.gear] / 100.0).powf(profile.traction_exponent)
                * torque_share(self.rpm, profile.idle_torque)
                * self.throttle.powf(profile.throttle_curve)
        } else {
            0.0
        };
        let engine_brake = if engaged && self.throttle < 0.02 && road_rpm > profile.idle_rpm {
            profile.engine_brake * road_rpm / 1000.0
        } else {
            0.0
        };
        let resistance = if self.speed > 0.05 {
            profile.rolling + profile.drag * self.speed * self.speed + engine_brake
        } else {
            0.0
        };
        let braking = profile.brake_per_unit * self.brake.max(0.0).powf(profile.brake_exponent);
        let gravity = 9.81 * grade / (1.0 + grade * grade).sqrt();
        let mut a = traction - resistance - gravity;
        // Brakes and resistance only hold or slow; they never push back.
        if self.speed <= 0.05 && a <= braking && a < 0.0 && braking > 0.0 {
            a = 0.0;
        } else {
            a -= braking;
        }
        self.speed = (self.speed + a * dt).max(0.0);
        self.acceleration = a;
    }
}
/// Share of peak torque at `rpm`: a flat plateau from 1000 to 1500 rpm,
/// less at idle and toward the top.
pub fn torque_share(rpm: f64, idle: f64) -> f64 {
    if rpm < 1000.0 {
        idle + (1.0 - idle) * ((rpm - 550.0) / 450.0).clamp(0.0, 1.0)
    } else if rpm <= 1500.0 {
        1.0
    } else {
        (1.0 - 0.4 * (rpm - 1500.0) / 700.0).max(0.4)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn map() -> SteeringMap {
        SteeringMap::from_points("test", vec![(0.715, -1.0), (0.0, 0.0), (-0.715, 1.0)])
    }
    #[test]
    fn relative_wheel_turns_more_slowly_at_speed_and_returns() {
        let profile = Profile::default();
        let map = map();
        let mut slow = Truck::new(Pose2::default(), 0.0, 0.0, 0.0);
        let mut fast = Truck::new(Pose2::default(), 0.0, 13.9, 0.0);
        for _ in 0..60 {
            slow.steer(1.0, &profile, &map, 1.0 / 60.0);
            fast.steer(1.0, &profile, &map, 1.0 / 60.0);
        }
        assert!(slow.position > fast.position && fast.position > 0.0);
        // The bridge slews the input back to zero within 1.25 s.
        for _ in 0..90 {
            fast.steer(0.0, &profile, &map, 1.0 / 60.0);
        }
        let held = fast.position;
        for _ in 0..60 {
            fast.steer(0.0, &profile, &map, 1.0 / 60.0);
        }
        assert!(fast.position < held, "the wheel returns toward the centre");
        assert!(
            slow.wheel < 0.0,
            "a positive position turns the wheel right"
        );
    }
    #[test]
    fn full_throttle_pulls_away_shifting_up_and_brakes_stop() {
        let profile = Profile::default();
        let mut truck = Truck::new(Pose2::default(), 0.0, 0.0, 0.0);
        let input = Input {
            throttle: 1.0,
            ..Default::default()
        };
        let mut shifts = 0;
        let mut gear = 0;
        let mut seconds = 0.0;
        while truck.speed < 50.0 / 3.6 && seconds < 60.0 {
            truck.longitudinal(input, 0.0, &profile, 1.0 / 60.0);
            if truck.gear != gear {
                shifts += 1;
                gear = truck.gear;
            }
            seconds += 1.0 / 60.0;
        }
        // The solo tractor: 0–50 km/h at full throttle in 8–20 s.
        assert!((8.0..20.0).contains(&seconds), "{seconds:.1} s to 50 km/h");
        assert!(
            shifts >= 2 && truck.gear >= 4,
            "{shifts} shifts to gear {}",
            truck.gear
        );
        let brake = Input {
            brake: 0.5,
            ..Default::default()
        };
        for _ in 0..(10.0 * 60.0) as usize {
            truck.longitudinal(brake, 0.0, &profile, 1.0 / 60.0);
        }
        assert_eq!(truck.speed, 0.0);
    }
    #[test]
    fn steering_table_inverts() {
        let map = SteeringMap::from_points(
            "test",
            vec![
                (0.715, -1.0),
                (0.35, -0.5),
                (0.0, 0.0),
                (-0.35, 0.5),
                (-0.715, 1.0),
            ],
        );
        for axis in [-1.0, -0.7, -0.2, 0.0, 0.3, 0.9] {
            let angle = map.angle(axis);
            assert!((map.axis(angle).unwrap() - axis).abs() < 1e-9);
        }
        assert_eq!(map.angle(2.0), -0.715);
    }
    #[test]
    fn a_trailer_runs_inside_the_tractor_on_a_circle() {
        // Rear axle on a 15 m circle, anticlockwise about the origin.
        let (radius, speed, dt) = (15.0, 3.0, 0.01);
        let mut trailer = Trailer::semi(std::f64::consts::FRAC_PI_2);
        let mut angle = 0.0_f64;
        for _ in 0..20_000 {
            angle += speed / radius * dt;
            trailer.follow(
                angle + std::f64::consts::FRAC_PI_2,
                speed,
                speed / radius,
                dt,
            );
        }
        let rear = Pose2 {
            x: radius * angle.cos(),
            y: radius * angle.sin(),
            yaw: angle + std::f64::consts::FRAC_PI_2,
        };
        let (x, y) = trailer.axle(rear);
        let kingpin = radius.hypot(trailer.hitch_m);
        let expected = (kingpin * kingpin - trailer.wheelbase_m.powi(2)).sqrt();
        assert!(
            (x.hypot(y) - expected).abs() < 0.05,
            "{} vs {expected}",
            x.hypot(y)
        );
        // About 2 m inside the tractor's rear axle.
        assert!(radius - x.hypot(y) > 1.9);
    }
}
