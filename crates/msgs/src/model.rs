//! Validated, serializable ENU contracts. Distances are metres, angles radians,
//! speeds metres/second and timestamps simulation microseconds.
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Point3 {
    pub fn valid(self) -> bool {
        [self.x, self.y, self.z]
            .iter()
            .all(|v| v.is_finite() && v.abs() < 1e8)
    }
    pub fn distance_xy(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}
impl Pose2 {
    pub fn valid(self) -> bool {
        self.x.is_finite()
            && self.y.is_finite()
            && self.yaw.is_finite()
            && self.x.abs() < 1e8
            && self.y.abs() < 1e8
    }
    pub fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Header {
    pub sim_time_us: u64,
    pub frame_id: u64,
    pub seq: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrailerEnvelope {
    /// Body front ahead of the hitch, along the trailer heading.
    pub front_from_hitch_m: f64,
    /// Body rear behind the equivalent trailer axle.
    pub rear_from_axle_m: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeasuredEnvelope {
    pub tractor_front_from_rear_axle_m: f64,
    pub tractor_rear_overhang_m: f64,
    pub trailer: Option<TrailerEnvelope>,
}
impl MeasuredEnvelope {
    pub fn valid_for(self, geometry: VehicleGeometry) -> bool {
        let front = self.tractor_front_from_rear_axle_m;
        let rear = self.tractor_rear_overhang_m;
        front.is_finite()
            && rear.is_finite()
            && front >= geometry.wheelbase
            && rear >= 0.0
            && (front + rear - geometry.tractor_length).abs() <= 1e-6
            && match (self.trailer, geometry.has_trailer) {
                (Some(trailer), true) => {
                    trailer.front_from_hitch_m.is_finite()
                        && trailer.rear_from_axle_m.is_finite()
                        && trailer.front_from_hitch_m >= 0.0
                        && trailer.rear_from_axle_m >= 0.0
                        && (trailer.front_from_hitch_m
                            + geometry.trailer_wheelbase
                            + trailer.rear_from_axle_m
                            - geometry.trailer_length)
                            .abs()
                            <= 1e-6
                }
                (None, false) => true,
                _ => false,
            }
    }
    pub fn tractor_center_from_rear_axle_m(self) -> f64 {
        (self.tractor_front_from_rear_axle_m - self.tractor_rear_overhang_m) / 2.0
    }
}
impl TrailerEnvelope {
    pub fn center_from_hitch_m(self, trailer_wheelbase: f64) -> f64 {
        (self.front_from_hitch_m - trailer_wheelbase - self.rear_from_axle_m) / 2.0
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VehicleGeometry {
    /// Explicit measured configuration. No serde default: omitted is unknown.
    pub has_trailer: bool,
    pub wheelbase: f64,
    pub hitch_offset: f64,
    pub trailer_wheelbase: f64,
    pub tractor_length: f64,
    pub trailer_length: f64,
    pub width: f64,
    pub max_steer: f64,
    pub max_articulation: f64,
    /// Explicit exterior measurements. Legacy recordings omit this field;
    /// their inferred offsets are synthetic/offline fallback, never live proof.
    #[serde(default)]
    pub measured_envelope: Option<MeasuredEnvelope>,
}
impl Default for VehicleGeometry {
    fn default() -> Self {
        Self {
            has_trailer: true,
            wheelbase: 3.8,
            hitch_offset: 0.4,
            trailer_wheelbase: 8.0,
            tractor_length: 6.2,
            trailer_length: 13.6,
            width: 2.55,
            max_steer: 0.6,
            max_articulation: std::f64::consts::PI / 3.0,
            measured_envelope: None,
        }
    }
}
impl VehicleGeometry {
    pub fn valid(self) -> bool {
        [self.wheelbase, self.tractor_length, self.width]
            .iter()
            .all(|v| v.is_finite() && *v > 0.1 && *v < 50.0)
            && self.trailer_wheelbase.is_finite()
            && self.trailer_length.is_finite()
            && if self.has_trailer {
                self.trailer_wheelbase > 0.1
                    && self.trailer_wheelbase < 50.0
                    && self.trailer_length > 0.1
                    && self.trailer_length < 50.0
            } else {
                self.trailer_wheelbase == 0.0 && self.trailer_length == 0.0
            }
            && self.hitch_offset.is_finite()
            && self.hitch_offset.abs() < self.wheelbase
            && self.max_steer.is_finite()
            && self.max_steer > 0.0
            && self.max_steer < 1.3
            && self.max_articulation.is_finite()
            && self.max_articulation > 0.0
            && self.max_articulation <= std::f64::consts::FRAC_PI_2
            && self
                .measured_envelope
                .is_none_or(|envelope| envelope.valid_for(self))
    }
    /// Shared geometry for every planner footprint, sweep and optimiser.
    /// None in `measured_envelope` retains the original synthetic model exactly;
    /// callers authorising live manoeuvres must require measured evidence first.
    pub fn resolved_envelope(self) -> MeasuredEnvelope {
        self.measured_envelope.unwrap_or_else(|| MeasuredEnvelope {
            tractor_front_from_rear_axle_m: self.wheelbase * 0.35 + self.tractor_length / 2.0,
            tractor_rear_overhang_m: self.tractor_length / 2.0 - self.wheelbase * 0.35,
            trailer: self.has_trailer.then_some(TrailerEnvelope {
                front_from_hitch_m: 1.0,
                rear_from_axle_m: self.trailer_length - 1.0 - self.trailer_wheelbase,
            }),
        })
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EgoState {
    pub header: Header,
    pub pose: Pose2,
    pub z: f64,
    pub speed: f64,
    pub acceleration: f64,
    pub yaw_rate: f64,
    pub wheel_angle: f64,
    pub trailer_yaw: f64,
    pub paused: bool,
}
impl EgoState {
    pub fn valid(self) -> bool {
        self.pose.valid()
            && [
                self.z,
                self.speed,
                self.acceleration,
                self.yaw_rate,
                self.wheel_angle,
                self.trailer_yaw,
            ]
            .iter()
            .all(|x| x.is_finite())
            && self.speed.abs() <= 100.0
            && self.acceleration.abs() <= 30.0
            && self.wheel_angle.abs() < 1.4
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OrientedBox {
    pub pose: Pose2,
    pub length: f64,
    pub width: f64,
}
impl OrientedBox {
    pub fn valid(self) -> bool {
        self.pose.valid()
            && self.length.is_finite()
            && self.width.is_finite()
            && self.length > 0.0
            && self.width > 0.0
            && self.length < 100.0
            && self.width < 100.0
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Agent {
    pub id: u64,
    pub pose: Pose2,
    pub z: f64,
    pub speed: f64,
    pub acceleration: f64,
    pub length: f64,
    pub width: f64,
    pub lane_id: Option<u64>,
    pub trailers: Vec<OrientedBox>,
}
impl Agent {
    pub fn valid(&self) -> bool {
        self.pose.valid()
            && [
                self.z,
                self.speed,
                self.acceleration,
                self.length,
                self.width,
            ]
            .iter()
            .all(|x| x.is_finite())
            && self.speed.abs() <= 100.0
            && self.acceleration.abs() <= 30.0
            && self.length > 0.0
            && self.length < 50.0
            && self.width > 0.0
            && self.width < 20.0
            && self.trailers.len() <= 3
            && self.trailers.iter().all(|b| b.valid())
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction {
    #[default]
    Forward,
    Reverse,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum BehaviorKind {
    #[default]
    LaneFollow,
    LaneChangeLeft,
    LaneChangeRight,
    Merge,
    Exit,
    TollBooth,
    Stop,
    Yield,
    Roundabout,
    Parking,
    EmergencyStop,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TrajPoint {
    pub t: f64,
    pub pose: Pose2,
    pub speed: f64,
    pub acceleration: f64,
    pub curvature: f64,
    pub articulation: f64,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Trajectory {
    pub header: Header,
    pub points: Vec<TrajPoint>,
    pub direction: Direction,
    pub behavior: BehaviorKind,
}
impl Trajectory {
    pub fn validate(&self) -> Result<(), String> {
        if self.points.len() < 2 || self.points.len() > 100_000 {
            return Err("trajectory needs 2..100000 points".into());
        }
        let mut last = -1.0;
        for p in &self.points {
            if !p.pose.valid()
                || ![p.t, p.speed, p.acceleration, p.curvature, p.articulation]
                    .iter()
                    .all(|v| v.is_finite())
                || p.t < 0.0
                || p.t <= last
                || p.speed.abs() > 100.0
                || p.acceleration.abs() > 30.0
                || p.curvature.abs() > 2.0
                || p.articulation.abs() > std::f64::consts::FRAC_PI_2
            {
                return Err("nonfinite, unordered or physically invalid trajectory point".into());
            }
            if (self.direction == Direction::Forward && p.speed < -1e-6)
                || (self.direction == Direction::Reverse && p.speed > 1e-6)
            {
                return Err("trajectory direction disagrees with speed".into());
            }
            last = p.t;
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Actuation {
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
    pub retarder: u8,
    pub indicator: i8,
    pub activate: bool,
}
impl Actuation {
    pub fn valid(self) -> bool {
        self.steer.is_finite()
            && self.throttle.is_finite()
            && self.brake.is_finite()
            && self.steer.abs() <= 1.0
            && (0.0..=1.0).contains(&self.throttle)
            && (0.0..=1.0).contains(&self.brake)
            && (-1..=1).contains(&self.indicator)
            && !(self.throttle > 0.0 && self.brake > 0.0)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalState {
    Off,
    Red,
    AmberToRed,
    AmberToGreen,
    Green,
    Flashing,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BarrierState {
    Closing,
    Closed,
    Opening,
    Open,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Scene {
    pub ego: EgoState,
    pub agents: Vec<Agent>,
    pub lane_id: Option<u64>,
    pub speed_limit: f64,
    pub route: Vec<u64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn trajectory_rejects_nan_and_reverse_mismatch() {
        let mut t = Trajectory {
            points: vec![
                TrajPoint::default(),
                TrajPoint {
                    t: 1.0,
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        assert!(t.validate().is_ok());
        t.points[1].speed = -1.0;
        assert!(t.validate().is_err());
        t.direction = Direction::Reverse;
        assert!(t.validate().is_ok());
        t.points[1].pose.x = f64::NAN;
        assert!(t.validate().is_err());
    }
}
