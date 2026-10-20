//! Independent articulated swept-envelope AEB. This module intentionally has no
//! dependency on control, planning, the fused Scene, or their geometric code.
use crate::{Obstacle, read_f32};
use etsads_msgs::{
    Point3, Pose2, VehicleGeometry,
    frames::{heading_to_yaw, placement_offset_to_enu, wrap_angle},
    protocol::{ConfigBlock, ExtensionFrame, TelemetryFrame},
    vehicle::geometry_fingerprint,
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrailerSafetyGeometry {
    pub front_from_hitch_m: f64,
    pub rear_from_axle_m: f64,
    pub sdk_to_axle_scs: Point3,
    pub axle_consistency_tolerance_m: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SafetyGeometryProfile {
    pub schema_version: u32,
    pub validated_for_reverse: bool,
    pub evidence_path: String,
    pub geometry_fingerprint: String,
    pub configuration_sequence: u64,
    pub cargo_mass_range_kg: [f64; 2],
    /// Measured bottom/top relative to axle height. None never excludes a
    /// potential obstacle merely because its centre is on another level.
    #[serde(default)]
    pub vertical_extent_from_rear_m: Option<[f64; 2]>,
    pub rear_wheel_indices: Vec<u32>,
    pub sdk_to_rear_axle_scs: Point3,
    pub geometry: VehicleGeometry,
    pub tractor_front_from_rear_axle_m: f64,
    pub tractor_rear_overhang_m: f64,
    pub trailer: Option<TrailerSafetyGeometry>,
}
impl SafetyGeometryProfile {
    /// Compare the planner's explicit measured envelope with these independent
    /// body measurements. This performs no shared collision computation and
    /// does not replace `validate` against the actual raw vehicle configuration.
    pub fn validate_planner_envelope(&self, planning: VehicleGeometry) -> Result<(), &'static str> {
        if !planning.valid() || !self.geometry.valid() {
            return Err("invalid planner or independent safety geometry");
        }
        let envelope = planning.measured_envelope.ok_or("planner lacks measured body overhangs; synthetic offsets cannot authorize live parking")?;
        let mut planner_model = planning;
        let mut independent_model = self.geometry;
        planner_model.measured_envelope = None;
        independent_model.measured_envelope = None;
        if planner_model != independent_model {
            return Err(
                "planner kinematics, dimensions or articulation limits differ from independent safety model",
            );
        }
        if !self.tractor_front_from_rear_axle_m.is_finite()
            || !self.tractor_rear_overhang_m.is_finite()
            || self.tractor_front_from_rear_axle_m < self.geometry.wheelbase
            || self.tractor_rear_overhang_m < 0.0
            || envelope.tractor_front_from_rear_axle_m < self.tractor_front_from_rear_axle_m
            || envelope.tractor_rear_overhang_m < self.tractor_rear_overhang_m
        {
            return Err(
                "measured planner tractor envelope does not contain independent front/rear overhangs",
            );
        }
        match (&self.trailer, envelope.trailer) {
            (Some(independent), Some(planned)) => {
                if !independent.front_from_hitch_m.is_finite()
                    || !independent.rear_from_axle_m.is_finite()
                    || independent.front_from_hitch_m < 0.0
                    || independent.rear_from_axle_m < 0.0
                    || planned.front_from_hitch_m < independent.front_from_hitch_m
                    || planned.rear_from_axle_m < independent.rear_from_axle_m
                {
                    return Err(
                        "measured planner trailer envelope does not contain independent hitch/axle overhangs",
                    );
                }
            }
            (None, None) => {}
            _ => return Err("planner and independent profile disagree about attached body count"),
        }
        Ok(())
    }
    pub fn validate(&self, config: &ConfigBlock) -> Result<(), &'static str> {
        let [min_mass, max_mass] = self.cargo_mass_range_kg;
        if let Some([bottom, top]) = self.vertical_extent_from_rear_m
            && (!bottom.is_finite()
                || !top.is_finite()
                || bottom > 0.0
                || bottom < -5.0
                || top < 0.5
                || top > 15.0
                || top <= bottom)
        {
            return Err("invalid verified vertical envelope");
        }
        if !min_mass.is_finite()
            || !max_mass.is_finite()
            || min_mass < 0.0
            || max_mass < min_mass
            || max_mass > 200_000.0
            || !config.cargo_mass_kg.is_finite()
            || !(min_mass..=max_mass).contains(&(config.cargo_mass_kg as f64))
        {
            return Err("cargo mass lies outside independent safety-profile validation range");
        }
        if self.schema_version != 1
            || !self.validated_for_reverse
            || self.evidence_path.is_empty()
            || !self.geometry.valid()
            || !self.sdk_to_rear_axle_scs.valid()
            || self.configuration_sequence == 0
            || self.configuration_sequence != config.config_seq
            || self.geometry_fingerprint != geometry_fingerprint(config)?
        {
            return Err("unverified safety geometry or configuration mismatch");
        }
        if self.geometry.has_trailer != self.trailer.is_some() {
            return Err(
                "safety model must explicitly match attached-trailer versus tractor-only geometry",
            );
        }
        if self.rear_wheel_indices.len() < 2 || self.rear_wheel_indices.len() > 16 {
            return Err("missing independent rear-wheel selection");
        }
        let mut seen = [false; 16];
        let mut mean = Point3::default();
        for &index in &self.rear_wheel_indices {
            let i = index as usize;
            if index >= config.wheel_count || i >= 16 || seen[i] || config.wheel_steerable[i] != 0 {
                return Err("invalid rear-wheel selection");
            }
            seen[i] = true;
            let p = config.wheel_positions[i];
            mean.x += p[0] as f64;
            mean.y += p[1] as f64;
            mean.z += p[2] as f64;
        }
        let n = self.rear_wheel_indices.len() as f64;
        mean.x /= n;
        mean.y /= n;
        mean.z /= n;
        if mean.distance_xy(self.sdk_to_rear_axle_scs) > 1e-5
            || (mean.z - self.sdk_to_rear_axle_scs.z).abs() > 1e-5
            || mean.x.abs() > 0.15
        {
            return Err("rear-axle reference disagrees with raw wheels");
        }
        let front: Vec<_> = (0..config.wheel_count as usize)
            .filter(|i| config.wheel_steerable[*i] != 0)
            .collect();
        if front.len() < 2 {
            return Err("front steering axle unavailable");
        }
        let front_z = front
            .iter()
            .map(|i| config.wheel_positions[*i][2] as f64)
            .sum::<f64>()
            / front.len() as f64;
        if (mean.z - front_z - self.geometry.wheelbase).abs() > 0.2
            || (config.hook_position[2] as f64 - mean.z - self.geometry.hitch_offset).abs() > 0.15
        {
            return Err("wheelbase or signed hitch offset mismatch");
        }
        let f = self.tractor_front_from_rear_axle_m;
        let rear = self.tractor_rear_overhang_m;
        if !f.is_finite()
            || !rear.is_finite()
            || f < self.geometry.wheelbase
            || rear <= 0.0
            || f + rear < self.geometry.tractor_length - 0.1
            || f + rear > self.geometry.tractor_length + 2.0
        {
            return Err("tractor safety envelope is missing or too small");
        }
        if let Some(trailer) = &self.trailer {
            let front = trailer.front_from_hitch_m;
            let rear = trailer.rear_from_axle_m;
            let length = front + self.geometry.trailer_wheelbase + rear;
            if !front.is_finite()
                || !rear.is_finite()
                || front < 0.0
                || front > 5.0
                || rear < 0.0
                || rear > 15.0
                || length < self.geometry.trailer_length - 0.1
                || length > self.geometry.trailer_length + 2.0
                || !trailer.sdk_to_axle_scs.valid()
                || !trailer.axle_consistency_tolerance_m.is_finite()
                || !(0.05..=0.5).contains(&trailer.axle_consistency_tolerance_m)
            {
                return Err("trailer safety envelope/reference is unverified");
            }
        }
        Ok(())
    }
}
#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    pub(crate) fn fixture(
        with_trailer: bool,
    ) -> (
        TelemetryFrame,
        ConfigBlock,
        SafetyGeometryProfile,
        Box<ExtensionFrame>,
    ) {
        let mut config = ConfigBlock {
            config_seq: 1,
            wheel_count: 4,
            ..Default::default()
        };
        config.wheel_positions[0] = [-1.0, 0.5, -3.8];
        config.wheel_positions[1] = [1.0, 0.5, -3.8];
        config.wheel_positions[2] = [-1.0, 0.5, 0.0];
        config.wheel_positions[3] = [1.0, 0.5, 0.0];
        config.wheel_radius[..4].fill(0.5);
        config.wheel_steerable[..2].fill(1);
        config.hook_position = [0.0, 1.0, -0.5];
        let geometry = VehicleGeometry {
            has_trailer: with_trailer,
            trailer_wheelbase: if with_trailer { 8.0 } else { 0.0 },
            trailer_length: if with_trailer { 13.6 } else { 0.0 },
            hitch_offset: -0.5,
            ..Default::default()
        };
        let profile = SafetyGeometryProfile {
            schema_version: 1,
            validated_for_reverse: true,
            evidence_path: "synthetic independent safety fixture".into(),
            geometry_fingerprint: geometry_fingerprint(&config).unwrap(),
            configuration_sequence: 1,
            cargo_mass_range_kg: [0.0, 100.0],
            vertical_extent_from_rear_m: Some([-0.5, 3.5]),
            rear_wheel_indices: vec![2, 3],
            sdk_to_rear_axle_scs: Point3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            geometry,
            tractor_front_from_rear_axle_m: 4.5,
            tractor_rear_overhang_m: 1.7,
            trailer: with_trailer.then_some(TrailerSafetyGeometry {
                front_from_hitch_m: 1.0,
                rear_from_axle_m: 4.6,
                sdk_to_axle_scs: Point3::default(),
                axle_consistency_tolerance_m: 0.2,
            }),
        };
        let mut ego = TelemetryFrame {
            qpc_frequency: 1000,
            gear: -1,
            speed_mps: -0.8,
            ..Default::default()
        };
        ego.header.frame_id = 10;
        ego.header.qpc = 1000;
        ego.header.sim_time_us = 1000;
        if with_trailer {
            ego.trailers[0].connected = 1;
            ego.trailers[0].position_scs = [0.0, 0.5, 7.5];
        }
        let mut extension = Box::<ExtensionFrame>::default();
        extension.available = 6;
        extension.header.frame_id = 10;
        extension.header.qpc = 999;
        (ego, config, profile, extension)
    }
    fn write_box(bytes: &mut [u8], position: [f32; 3], quaternion: [f32; 4], size: [f32; 3]) {
        let values = [
            position[0],
            position[1],
            position[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
            size[0],
            size[1],
            size[2],
        ];
        for (i, value) in values.iter().enumerate() {
            bytes[i * 4..i * 4 + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    #[test]
    fn sees_obstacle_behind_real_trailer_with_signed_forward_hitch() {
        let (ego, config, profile, mut ext) = fixture(true);
        write_box(
            &mut ext.parked[..43],
            [0.0, 0.5, 14.0],
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 3.0, 2.0],
        );
        ext.parked[40] = 1;
        assert!(crate::primary_obstacle(&ego, &ext).unwrap().is_none());
        let obstacle = swept_obstacle(&ego, &ext, &config, &profile)
            .unwrap()
            .unwrap();
        assert!(obstacle.distance_m < 1.0);
        assert!(obstacle.closing_speed_mps > 0.0);
    }
    #[test]
    fn rearward_opposing_vehicle_is_predicted_in_time() {
        let (ego, config, profile, mut ext) = fixture(true);
        write_box(
            &mut ext.traffic[..174],
            [0.0, 0.5, 20.0],
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 3.0, 4.0],
        );
        ext.traffic[40..44].copy_from_slice(&10.0_f32.to_le_bytes());
        ext.traffic[50] = 1;
        let obstacle = swept_obstacle(&ego, &ext, &config, &profile)
            .unwrap()
            .unwrap();
        assert!(obstacle.distance_m / obstacle.closing_speed_mps < 1.0);
    }
    #[test]
    fn bobtail_front_corner_swing_is_guarded_while_reversing() {
        let (mut ego, config, profile, mut ext) = fixture(false);
        ego.wheel_steering[..2].fill((0.5 / std::f64::consts::TAU) as f32);
        write_box(
            &mut ext.parked[..43],
            [2.1, 0.5, -3.5],
            [1.0, 0.0, 0.0, 0.0],
            [0.8, 1.0, 1.0],
        );
        ext.parked[40] = 1;
        assert!(
            swept_obstacle(&ego, &ext, &config, &profile)
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn continuous_motion_catches_crossing_between_50ms_samples() {
        let (ego, config, profile, mut ext) = fixture(false);
        let half = std::f32::consts::FRAC_1_SQRT_2;
        write_box(
            &mut ext.traffic[..174],
            [-1.75, 0.5, 0.0],
            [half, 0.0, -half, 0.0],
            [0.2, 1.0, 0.2],
        );
        ext.traffic[40..44].copy_from_slice(&70.0_f32.to_le_bytes());
        ext.traffic[50] = 1;
        let obstacle = swept_obstacle(&ego, &ext, &config, &profile)
            .unwrap()
            .unwrap();
        assert_eq!(obstacle.distance_m, 0.0);
    }
    #[test]
    fn unknown_trailer_or_changed_load_does_not_reuse_profile() {
        let (mut ego, mut config, profile, ext) = fixture(true);
        ego.trailers[0].position_scs[2] += 2.0;
        assert!(swept_obstacle(&ego, &ext, &config, &profile).is_err());
        config.cargo_mass_kg = 20000.0;
        assert!(profile.validate(&config).is_err());
    }
    #[test]
    fn overpass_is_not_a_rear_obstacle_and_future_capture_is_rejected() {
        let (ego, config, profile, mut ext) = fixture(true);
        write_box(
            &mut ext.parked[..43],
            [0.0, 10.0, 14.0],
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 3.0, 2.0],
        );
        ext.parked[40] = 1;
        assert!(
            swept_obstacle(&ego, &ext, &config, &profile)
                .unwrap()
                .is_none()
        );
        ext.header.qpc = 1001;
        assert!(swept_obstacle(&ego, &ext, &config, &profile).is_err());
    }
    #[test]
    fn equal_length_planner_body_must_contain_independent_measured_overhangs() {
        let (_, config, profile, _) = fixture(true);
        profile.validate(&config).unwrap();
        assert!(profile.validate_planner_envelope(profile.geometry).is_err());
        let envelope = etsads_msgs::MeasuredEnvelope {
            tractor_front_from_rear_axle_m: 4.5,
            tractor_rear_overhang_m: 1.7,
            trailer: Some(etsads_msgs::TrailerEnvelope {
                front_from_hitch_m: 1.0,
                rear_from_axle_m: 4.6,
            }),
        };
        let geometry = VehicleGeometry {
            measured_envelope: Some(envelope),
            ..profile.geometry
        };
        profile.validate_planner_envelope(geometry).unwrap();
        for (front, rear) in [(5.0, 1.2), (4.0, 2.2)] {
            let changed = VehicleGeometry {
                measured_envelope: Some(etsads_msgs::MeasuredEnvelope {
                    tractor_front_from_rear_axle_m: front,
                    tractor_rear_overhang_m: rear,
                    ..envelope
                }),
                ..geometry
            };
            assert!(changed.valid());
            assert!(profile.validate_planner_envelope(changed).is_err());
        }
        let changed = VehicleGeometry {
            measured_envelope: Some(etsads_msgs::MeasuredEnvelope {
                trailer: Some(etsads_msgs::TrailerEnvelope {
                    front_from_hitch_m: 2.0,
                    rear_from_axle_m: 3.6,
                }),
                ..envelope
            }),
            ..geometry
        };
        assert!(changed.valid());
        assert!(profile.validate_planner_envelope(changed).is_err());
    }
    #[test]
    fn tall_vertical_overlap_is_not_filtered_by_center_height() {
        let (ego, config, mut profile, mut ext) = fixture(true);
        write_box(
            &mut ext.parked[..43],
            [0.0, 3.6, 14.0],
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 6.0, 2.0],
        );
        ext.parked[40] = 1;
        assert!(
            swept_obstacle(&ego, &ext, &config, &profile)
                .unwrap()
                .is_some()
        );
        profile.vertical_extent_from_rear_m = None;
        write_box(
            &mut ext.parked[..43],
            [0.0, 10.0, 14.0],
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 3.0, 2.0],
        );
        assert!(
            swept_obstacle(&ego, &ext, &config, &profile)
                .unwrap()
                .is_some()
        );
    }
}
#[cfg(test)]
#[test]
fn tilted_long_body_uses_projected_vertical_and_planar_extent() {
    let (ego, config, profile, mut ext) = tests::fixture(true);
    let half = std::f32::consts::FRAC_PI_8;
    let values = [
        0.0_f32,
        8.0,
        20.0,
        half.cos(),
        half.sin(),
        0.0,
        0.0,
        2.0,
        1.0,
        20.0,
    ];
    for (i, value) in values.iter().enumerate() {
        ext.parked[4 * i..4 * i + 4].copy_from_slice(&value.to_le_bytes());
    }
    ext.parked[40] = 1;
    assert!(
        swept_obstacle(&ego, &ext, &config, &profile)
            .unwrap()
            .is_some()
    );
}
#[derive(Clone, Copy)]
struct Shape {
    pose: Pose2,
    length: f64,
    width: f64,
}
#[derive(Clone, Copy)]
struct Body {
    shape: Shape,
    velocity: [f64; 2],
    vertical_center: f64,
    vertical_half_height: f64,
}
#[derive(Clone, Copy)]
struct State {
    rear: Pose2,
    trailer_yaw: Option<f64>,
}
fn shape_corners(shape: Shape) -> [(f64, f64); 4] {
    let (c, s) = (shape.pose.yaw.cos(), shape.pose.yaw.sin());
    [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)].map(|(x, y)| {
        (
            shape.pose.x + x * shape.length * c / 2.0 - y * shape.width * s / 2.0,
            shape.pose.y + x * shape.length * s / 2.0 + y * shape.width * c / 2.0,
        )
    })
}
fn intersects(a: Shape, b: Shape, margin: f64) -> bool {
    let a = Shape {
        length: a.length + 2.0 * margin,
        width: a.width + 2.0 * margin,
        ..a
    };
    let ac = shape_corners(a);
    let bc = shape_corners(b);
    for angle in [
        a.pose.yaw,
        a.pose.yaw + std::f64::consts::FRAC_PI_2,
        b.pose.yaw,
        b.pose.yaw + std::f64::consts::FRAC_PI_2,
    ] {
        let (c, s) = (angle.cos(), angle.sin());
        let bounds = |corners: [(f64, f64); 4]| {
            corners
                .into_iter()
                .map(|(x, y)| x * c + y * s)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                })
        };
        let (a0, a1) = bounds(ac);
        let (b0, b1) = bounds(bc);
        if a1 < b0 || b1 < a0 {
            return false;
        }
    }
    true
}
fn at(shape: Shape, to: Shape, t: f64) -> Shape {
    Shape {
        pose: Pose2 {
            x: shape.pose.x + (to.pose.x - shape.pose.x) * t,
            y: shape.pose.y + (to.pose.y - shape.pose.y) * t,
            yaw: shape.pose.yaw + wrap_angle(to.pose.yaw - shape.pose.yaw) * t,
        },
        ..shape
    }
}
fn motion_bound(a: Shape, b: Shape) -> f64 {
    a.pose.distance(b.pose)
        + a.length.hypot(a.width) / 2.0 * wrap_angle(b.pose.yaw - a.pose.yaw).abs()
}
fn moving_sweep(a: Shape, next_a: Shape, b: Shape, next_b: Shape) -> bool {
    let travel_a = motion_bound(a, next_a);
    let travel_b = motion_bound(b, next_b);
    let n = ((travel_a + travel_b) / 0.15).ceil().clamp(1.0, 2000.0) as usize;
    for i in 0..n {
        let t = (i as f64 + 0.5) / n as f64;
        let bound = (travel_a + travel_b) / (2.0 * n as f64);
        if intersects(at(a, next_a, t), at(b, next_b, t), 0.35 + bound) {
            return true;
        }
    }
    false
}
fn shapes(state: State, profile: &SafetyGeometryProfile) -> Vec<Shape> {
    let forward = profile.tractor_front_from_rear_axle_m;
    let rear = profile.tractor_rear_overhang_m;
    let offset = (forward - rear) / 2.0;
    let mut result = vec![Shape {
        pose: Pose2 {
            x: state.rear.x + offset * state.rear.yaw.cos(),
            y: state.rear.y + offset * state.rear.yaw.sin(),
            yaw: state.rear.yaw,
        },
        length: forward + rear,
        width: profile.geometry.width,
    }];
    if let (Some(yaw), Some(trailer)) = (state.trailer_yaw, &profile.trailer) {
        let hitch_x = state.rear.x - profile.geometry.hitch_offset * state.rear.yaw.cos();
        let hitch_y = state.rear.y - profile.geometry.hitch_offset * state.rear.yaw.sin();
        let back = profile.geometry.trailer_wheelbase + trailer.rear_from_axle_m;
        let offset = (trailer.front_from_hitch_m - back) / 2.0;
        result.push(Shape {
            pose: Pose2 {
                x: hitch_x + offset * yaw.cos(),
                y: hitch_y + offset * yaw.sin(),
                yaw,
            },
            length: trailer.front_from_hitch_m + back,
            width: profile.geometry.width,
        });
    }
    result
}
fn advance(
    state: State,
    speed: f64,
    steer: f64,
    dt: f64,
    geometry: VehicleGeometry,
) -> Result<State, &'static str> {
    let derivative = |s: State| {
        let yaw_rate = speed * steer.tan() / geometry.wheelbase;
        let trailer_rate = s.trailer_yaw.map(|yaw| {
            let phi = wrap_angle(s.rear.yaw - yaw);
            speed * phi.sin() / geometry.trailer_wheelbase
                - geometry.hitch_offset * yaw_rate * phi.cos() / geometry.trailer_wheelbase
        });
        (
            speed * s.rear.yaw.cos(),
            speed * s.rear.yaw.sin(),
            yaw_rate,
            trailer_rate,
        )
    };
    let (dx, dy, dtheta, dtrailer) = derivative(state);
    let mid = State {
        rear: Pose2 {
            x: state.rear.x + dx * dt / 2.0,
            y: state.rear.y + dy * dt / 2.0,
            yaw: state.rear.yaw + dtheta * dt / 2.0,
        },
        trailer_yaw: state
            .trailer_yaw
            .zip(dtrailer)
            .map(|(a, b)| a + b * dt / 2.0),
    };
    let (dx, dy, dtheta, dtrailer) = derivative(mid);
    let next = State {
        rear: Pose2 {
            x: state.rear.x + dx * dt,
            y: state.rear.y + dy * dt,
            yaw: wrap_angle(state.rear.yaw + dtheta * dt),
        },
        trailer_yaw: state
            .trailer_yaw
            .zip(dtrailer)
            .map(|(a, b)| wrap_angle(a + b * dt)),
    };
    if next
        .trailer_yaw
        .is_some_and(|yaw| wrap_angle(next.rear.yaw - yaw).abs() > geometry.max_articulation)
    {
        return Err("reverse sweep exceeds articulation limit");
    }
    Ok(next)
}
fn decode(data: &[u8], speed: f64) -> Result<Body, &'static str> {
    let values: [f64; 10] = std::array::from_fn(|i| read_f32(data, 4 * i) as f64);
    if values.iter().any(|v| !v.is_finite())
        || values[..3].iter().any(|v| v.abs() > 1e7)
        || !speed.is_finite()
        || speed.abs() > 70.0
    {
        return Err("invalid raw obstacle");
    }
    let [x, y, z, w, qx, qy, qz, width, height, length] = values;
    let norm = w * w + qx * qx + qy * qy + qz * qz;
    if !(0.8..=1.2).contains(&norm)
        || !(0.1..20.0).contains(&width)
        || !(0.1..30.0).contains(&height)
        || !(0.1..100.0).contains(&length)
    {
        return Err("invalid raw obstacle shape");
    }
    let fx = -2.0 * (qx * qz + w * qy) / norm;
    let fy = 1.0 - 2.0 * (qx * qx + qy * qy) / norm;
    if fx.hypot(fy) < 0.5 {
        return Err("invalid raw obstacle planar orientation");
    }
    let yaw = fy.atan2(fx);
    let (c, s) = (yaw.cos(), yaw.sin());
    let right = [
        1.0 - 2.0 * (qy * qy + qz * qz) / norm,
        -2.0 * (qx * qz - w * qy) / norm,
    ];
    let up = [
        2.0 * (qx * qy - w * qz) / norm,
        -2.0 * (qy * qz + w * qx) / norm,
    ];
    let projected_length = length * (fx * c + fy * s).abs()
        + width * (right[0] * c + right[1] * s).abs()
        + height * (up[0] * c + up[1] * s).abs();
    let projected_width = length * (-fx * s + fy * c).abs()
        + width * (-right[0] * s + right[1] * c).abs()
        + height * (-up[0] * s + up[1] * c).abs();
    let vertical_half = width / 2.0 * (2.0 * (qx * qy + w * qz) / norm).abs()
        + height / 2.0 * (1.0 - 2.0 * (qx * qx + qz * qz) / norm).abs()
        + length / 2.0 * (2.0 * (qy * qz - w * qx) / norm).abs();
    Ok(Body {
        shape: Shape {
            pose: Pose2 { x, y: -z, yaw },
            length: projected_length,
            width: projected_width,
        },
        velocity: [speed * fx, speed * fy],
        vertical_center: y,
        vertical_half_height: vertical_half,
    })
}
fn obstacle_bodies(extension: &ExtensionFrame) -> Result<Vec<Body>, &'static str> {
    let mut result = Vec::new();
    for bytes in extension.traffic.as_chunks::<174>().0 {
        let id = i16::from_le_bytes([bytes[50], bytes[51]]);
        if id < 0 || bytes.iter().all(|b| *b == 0) {
            continue;
        }
        let speed = read_f32(bytes, 40) as f64;
        result.push(decode(bytes, speed)?);
        let count = i16::from_le_bytes([bytes[48], bytes[49]]);
        if !(0..=3).contains(&count) {
            return Err("invalid raw traffic trailer count");
        }
        for trailer in bytes[54..].as_chunks::<40>().0.iter().take(count as usize) {
            result.push(decode(trailer, speed)?);
        }
    }
    for bytes in extension.parked.as_chunks::<43>().0 {
        let id = i16::from_le_bytes([bytes[40], bytes[41]]);
        if id < 0 || bytes.iter().all(|b| *b == 0) {
            continue;
        }
        result.push(decode(bytes, 0.0)?);
    }
    Ok(result)
}
pub fn swept_obstacle(
    ego: &TelemetryFrame,
    extension: &ExtensionFrame,
    config: &ConfigBlock,
    profile: &SafetyGeometryProfile,
) -> Result<Option<Obstacle>, &'static str> {
    profile.validate(config)?;
    if extension.available & 6 != 6 {
        return Err("independent traffic and parked-object channels are required");
    }
    if ego.qpc_frequency == 0
        || ego.header.qpc == 0
        || extension.header.qpc == 0
        || extension.header.qpc > ego.header.qpc
        || ego.header.qpc.abs_diff(extension.header.qpc) as u128 * 1000
            > ego.qpc_frequency as u128 * 200
        || extension.header.frame_id > ego.header.frame_id
        || ego.header.frame_id - extension.header.frame_id > 3
    {
        return Err("raw obstacle snapshot is not frame/clock aligned");
    }
    let position = placement_offset_to_enu(
        Point3 {
            x: ego.position_scs[0],
            y: ego.position_scs[1],
            z: ego.position_scs[2],
        },
        ego.orientation_turns.map(|v| v as f64),
        profile.sdk_to_rear_axle_scs,
    )?;
    let yaw = heading_to_yaw(ego.orientation_turns[0] as f64);
    if !ego.speed_mps.is_finite() || ego.speed_mps.abs() > 70.0 {
        return Err("invalid speed");
    }
    let attached: Vec<_> = ego.trailers.iter().filter(|t| t.connected != 0).collect();
    if attached.len() > 1 || attached.is_empty() != profile.trailer.is_none() {
        return Err("attached trailer does not match independent safety profile");
    }
    let trailer_yaw = if let Some(trailer) = attached.first() {
        let yaw = heading_to_yaw(trailer.orientation_turns[0] as f64);
        let reference = profile.trailer.as_ref().unwrap();
        let p = placement_offset_to_enu(
            Point3 {
                x: trailer.position_scs[0],
                y: trailer.position_scs[1],
                z: trailer.position_scs[2],
            },
            trailer.orientation_turns.map(|v| v as f64),
            reference.sdk_to_axle_scs,
        )?;
        let expected = Point3 {
            x: position.x
                - profile.geometry.hitch_offset
                    * (heading_to_yaw(ego.orientation_turns[0] as f64)).cos()
                - profile.geometry.trailer_wheelbase * yaw.cos(),
            y: position.y
                - profile.geometry.hitch_offset
                    * (heading_to_yaw(ego.orientation_turns[0] as f64)).sin()
                - profile.geometry.trailer_wheelbase * yaw.sin(),
            z: position.z,
        };
        if p.distance_xy(expected) > reference.axle_consistency_tolerance_m
            || (p.z - expected.z).abs() > 3.0
        {
            return Err("raw trailer reference disagrees with hitch/axle geometry");
        }
        Some(yaw)
    } else {
        None
    };
    let mut steer = 0.0;
    let mut steer_count = 0;
    for i in 0..config.wheel_count as usize {
        if config.wheel_steerable[i] != 0 {
            let angle = ego.wheel_steering[i] as f64 * std::f64::consts::TAU;
            if !angle.is_finite() || angle.abs() > profile.geometry.max_steer + 0.05 {
                return Err("invalid measured wheel steering");
            }
            steer += angle;
            steer_count += 1;
        }
    }
    if steer_count == 0 {
        return Err("wheel steering unavailable");
    }
    steer /= steer_count as f64;
    let reverse = ego.gear < 0 || ego.speed_mps < -0.02;
    let probe_speed = (ego.speed_mps.abs() as f64).max(if reverse { 3.0 / 3.6 } else { 0.5 });
    if reverse && probe_speed > 2.0 {
        return Err("reverse speed is outside the independently validated low-speed sweep");
    }
    let signed_speed = if reverse { -probe_speed } else { probe_speed };
    let bodies = obstacle_bodies(extension)?;
    let mut tilt = 0.0_f64;
    for orientation in std::iter::once(ego.orientation_turns)
        .chain(attached.iter().map(|trailer| trailer.orientation_turns))
    {
        if orientation.iter().any(|a| !a.is_finite())
            || orientation[1].abs() > 0.125
            || orientation[2].abs() > 0.125
        {
            return Err("orientation exceeds the verified planar safety model");
        }
        let pitch = orientation[1] as f64 * std::f64::consts::TAU;
        let roll = orientation[2] as f64 * std::f64::consts::TAU;
        let reach = profile.tractor_front_from_rear_axle_m
            + profile.tractor_rear_overhang_m
            + profile.geometry.trailer_length
            + profile.geometry.hitch_offset.abs();
        tilt =
            tilt.max(pitch.sin().abs() * reach + roll.sin().abs() * profile.geometry.width / 2.0);
    }
    let bodies: Vec<_> = bodies
        .into_iter()
        .filter(|body| {
            profile
                .vertical_extent_from_rear_m
                .is_none_or(|[bottom, top]| {
                    body.vertical_center + body.vertical_half_height >= position.z + bottom - tilt
                        && body.vertical_center - body.vertical_half_height
                            <= position.z + top + tilt
                })
        })
        .collect();
    let mut state = State {
        rear: Pose2 {
            x: position.x,
            y: position.y,
            yaw,
        },
        trailer_yaw,
    };
    let mut previous = shapes(state, profile);
    for body in &bodies {
        if previous
            .iter()
            .any(|shape| intersects(*shape, body.shape, 0.35))
        {
            return Ok(Some(Obstacle {
                distance_m: 0.0,
                closing_speed_mps: probe_speed as f32,
            }));
        }
    }
    let dt = 0.05;
    let horizon = if reverse {
        3.0
    } else {
        (probe_speed / 4.0 + 0.3).clamp(2.0, 6.0)
    };
    let steps = (horizon / dt).ceil() as usize;
    for i in 1..=steps {
        let t = i as f64 * dt;
        let next = advance(state, signed_speed, steer, dt, profile.geometry)?;
        let next_shapes = shapes(next, profile);
        for body in &bodies {
            let translated = |time: f64| Shape {
                pose: Pose2 {
                    x: body.shape.pose.x + body.velocity[0] * time,
                    y: body.shape.pose.y + body.velocity[1] * time,
                    ..body.shape.pose
                },
                ..body.shape
            };
            let before = translated(t - dt);
            let after = translated(t);
            if previous
                .iter()
                .zip(&next_shapes)
                .any(|(a, b)| moving_sweep(*a, *b, before, after))
            {
                return Ok(Some(Obstacle {
                    distance_m: (probe_speed * (t - dt).max(0.0)) as f32,
                    closing_speed_mps: probe_speed as f32,
                }));
            }
        }
        previous = next_shapes;
        state = next;
    }
    Ok(None)
}
