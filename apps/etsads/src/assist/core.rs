//! Operator-requested, short low-speed assistance. This geometry-only view
//! never changes a map's driving approval or creates a calibration certificate.
use anyhow::{Context, ensure};
use etsads_control::static_identification::StaticSteeringAnalysis;
use etsads_map::{importer::ExportedMap, polyline_tangent, project_polyline};
use etsads_msgs::{Point3, Pose2, frames::wrap_angle, protocol::ConfigBlock};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path};

pub const MAX_SPEED: f64 = 90.0 / 3.6;
pub const MAX_SECONDS: u64 = 3600;
/// Steering is limited so that curvature x speed² stays below this (m/s²).
const MAX_LATERAL_ACCELERATION: f64 = 2.5;
/// How the curvature feed-forward reads the road: `lag_s` ahead to cover
/// the steering lag, averaged over `half_m` + 0.25 s of travel each way.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct FeedForward {
    pub(super) lag_s: f64,
    pub(super) half_m: f64,
    /// How fast the steering can change the curvature (1/m per s), when it
    /// is slow enough to start a change early: relative steering unwinds a
    /// junction turn over a couple of seconds.
    pub(super) curvature_rate: Option<f64>,
}
impl FeedForward {
    /// Relative steering turning the wheel at `rate_per_s` (position per
    /// second): the curvature rate it can sustain with room to correct.
    pub(super) const fn relative(rate_per_s: f64) -> Self {
        Self {
            curvature_rate: Some(relative_curvature_rate(rate_per_s)),
            ..RELATIVE_FEED_FORWARD
        }
    }
}
/// r11 trace: the wheel follows the command about 0.4–0.6 s later.
pub(super) const ABSOLUTE_FEED_FORWARD: FeedForward = FeedForward {
    lag_s: 0.5,
    half_m: 1.5,
    curvature_rate: None,
};
/// With relative steering the servo holds the shaped position within a few
/// thousandths once it allows for the wheel's return; only the shaping's
/// ramp is left to cover. r14's 1.0 s (fitted without the return) led
/// bends by about 9 m at 31 km/h: in early, out early and 1.3 m wide (r15).
/// The wider average keeps f32 map noise off the slowly steered wheel.
const RELATIVE_FEED_FORWARD: FeedForward = FeedForward {
    lag_s: 0.2,
    half_m: 2.5,
    curvature_rate: None,
};
/// How fast relative steering turning the wheel at `rate_per_s` (position
/// per second) is planned to change the curvature (1/m per s).
pub(super) const fn relative_curvature_rate(rate_per_s: f64) -> f64 {
    RELATIVE_CURVATURE_RATE_SHARE * rate_per_s * TYPICAL_FULL_LOCK_RAD / TYPICAL_WHEELBASE_M
}
/// Share of a relatively steered wheel's turning rate the curvature
/// feed-forward plans with, and the planning vehicle (full lock and
/// wheelbase) that turns it into a curvature rate.
const RELATIVE_CURVATURE_RATE_SHARE: f64 = 0.7;
const TYPICAL_FULL_LOCK_RAD: f64 = 0.7;
const TYPICAL_WHEELBASE_M: f64 = 3.8;
/// The wheel takes about this long to get a change going (s).
const RELATIVE_RAMP_S: f64 = 0.5;
/// A curvature the steering cannot reach in time from `near`: within the
/// band every curvature ahead allows, given it changes at `rate` (1/m per
/// s) at `speed`. A junction turn ending in a straight needs the wheel
/// unwound before the straight; r16 unwound 0.36 from there at 11 km/h,
/// ran 1.5 m wide and swayed.
fn reachable_curvature(road: &Road, station: f64, speed: f64, near: f64, rate: f64) -> f64 {
    let speed = speed.max(1.0);
    let horizon = (speed * 8.0).clamp(10.0, 80.0).min(road.length() - station);
    let (mut low, mut high) = (f64::NEG_INFINITY, f64::INFINITY);
    let mut d = 1.0;
    while d <= horizon {
        let allowance =
            rate * super::io::relative_speed_factor(speed) * (d / speed - RELATIVE_RAMP_S).max(0.0);
        let curvature = road.mean_curvature(station + d, 3.0);
        low = low.max(curvature - allowance);
        high = high.min(curvature + allowance);
        d += 1.0;
    }
    if low > high {
        (low + high) / 2.0
    } else {
        near.clamp(low, high)
    }
}
/// Lateral-error trim: wheel angle per metre-second of integrated error,
/// its leak, and the integral bound (at most 0.032 rad of wheel).
const LATERAL_TRIM_GAIN: f64 = 0.004;
const LATERAL_LEAK_PER_S: f64 = 0.01;
const LATERAL_INTEGRAL_MAX: f64 = 8.0;
/// Beyond this offset from the locked path the truck stops instead of
/// correcting (r10: 1.0 m released it inside a roundabout).
pub const LOCKED_MAX_LATERAL_M: f64 = 1.6;
/// From this offset the truck slows down and warns while it corrects.
pub const DRIFT_WARN_M: f64 = 0.6;
/// Smoothing of the pursuit target's rate for relative steering (s).
const RELATIVE_RATE_SMOOTHING_S: f64 = 0.2;
/// Half-length of the chords path headings are read over (m).
const HEADING_BASE_M: f64 = 1.5;
/// Largest lead of a relatively steered command over the measured wheel.
const RELATIVE_LEAD: f64 = 0.08;

#[derive(Deserialize)]
pub struct SteeringReport {
    schema_version: u32,
    report_kind: String,
    recording_source: String,
    complete_recording: bool,
    frame_gaps: u64,
    geometry_fingerprint: Option<String>,
    analysis: StaticSteeringAnalysis,
}
#[derive(Clone)]
pub struct SteeringMap {
    pub fingerprint: String,
    points: Vec<(f64, f64)>, // measured mean wheel angle, semantic input
}
impl SteeringMap {
    pub fn read(path: &Path) -> anyhow::Result<Self> {
        let bytes = super::bounded_read(path, 128 * 1024)?;
        let report: SteeringReport = serde_json::from_slice(&bytes)?;
        ensure!(
            report.schema_version == 1
                && report.report_kind
                    == "static_zero_throttle_wheel_angle_scan_not_driving_calibration"
                && report.recording_source == "game"
                && report.complete_recording
                && report.frame_gaps == 0
                && report.analysis.stationary_verified
                && report.analysis.zero_throttle_verified
                && report.analysis.monotone_response
                && report.analysis.invalid_samples == 0
                && report.analysis.issues.is_empty(),
            "需要完整、无缺帧的实测静态转向报告"
        );
        let mut points = Vec::new();
        for point in &report.analysis.points {
            ensure!(
                point.samples >= 20
                    && point.axis.is_finite()
                    && point.axis.abs() <= 1.0
                    && !point.mean_wheel_angles_rad.is_empty()
                    && point
                        .mean_wheel_angles_rad
                        .iter()
                        .all(|v| v.is_finite() && v.abs() < 1.0)
                    && point
                        .standard_deviation_rad
                        .iter()
                        .all(|v| v.is_finite() && *v < 0.015),
                "转向样本不稳定或超出范围"
            );
            let angle = point.mean_wheel_angles_rad.iter().sum::<f64>()
                / point.mean_wheel_angles_rad.len() as f64;
            points.push((angle, point.axis));
        }
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        ensure!(
            points.len() >= 4 && points[0].0 < -0.05 && points.last().unwrap().0 > 0.05,
            "转向报告需要包含左右两个方向和至少四个稳态点"
        );
        ensure!(
            points.windows(2).all(|p| p[1].0 - p[0].0 > 1e-4)
                && (points.windows(2).all(|p| p[1].1 > p[0].1)
                    || points.windows(2).all(|p| p[1].1 < p[0].1)),
            "转向表不单调"
        );
        Ok(Self {
            fingerprint: report
                .geometry_fingerprint
                .context("转向报告缺少车辆指纹")?,
            points,
        })
    }
    /// A table from (mean wheel angle, input) points, for simulation tests.
    #[cfg(test)]
    pub(super) fn from_points(fingerprint: &str, mut points: Vec<(f64, f64)>) -> Self {
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self {
            fingerprint: fingerprint.into(),
            points,
        }
    }
    /// The calibrated wheel-angle range (rad).
    pub fn range(&self) -> (f64, f64) {
        (self.points[0].0, self.points.last().map_or(0.0, |p| p.0))
    }
    pub fn axis(&self, angle: f64) -> anyhow::Result<f64> {
        ensure!(
            angle.is_finite()
                && angle >= self.points[0].0
                && angle <= self.points.last().unwrap().0,
            "所需转角超出已测范围，请手动驾驶"
        );
        let p = self.points.windows(2).find(|p| angle <= p[1].0).unwrap();
        Ok(p[0].1 + (angle - p[0].0) / (p[1].0 - p[0].0) * (p[1].1 - p[0].1))
    }
    /// The mean wheel angle the game turns to at `axis` (the inverse of
    /// `axis`), held at the measured ends.
    pub fn angle(&self, axis: f64) -> f64 {
        let (first, last) = (self.points[0], self.points[self.points.len() - 1]);
        let (low, high) = if first.1 < last.1 {
            (first, last)
        } else {
            (last, first)
        };
        if axis <= low.1 {
            return low.0;
        }
        if axis >= high.1 {
            return high.0;
        }
        let p = self
            .points
            .windows(2)
            .find(|p| (p[0].1 - axis) * (p[1].1 - axis) <= 0.0)
            .unwrap_or(&self.points[..2]);
        p[0].0 + (axis - p[0].1) / (p[1].1 - p[0].1) * (p[1].0 - p[0].0)
    }
    pub fn vehicle(&self, config: &ConfigBlock) -> anyhow::Result<Axles> {
        let fingerprint =
            etsads_msgs::vehicle::geometry_fingerprint(config).map_err(anyhow::Error::msg)?;
        ensure!(
            fingerprint == self.fingerprint,
            "当前车辆与已测转向报告不匹配"
        );
        Ok(vehicle_geometry(config)?.0)
    }
}
/// Rear-axle reference and front steered wheels of a tractor. A tandem or
/// tag axle counts toward the rear reference (its midpoint is the equivalent
/// rear axle); a steered tag axle behind the front axle is not a front wheel.
pub fn vehicle_geometry(config: &ConfigBlock) -> anyhow::Result<(Axles, Vec<usize>)> {
    let count = (config.wheel_count as usize).min(16);
    let forward = |i: usize| -(config.wheel_positions[i][2] as f64);
    let front_most = (0..count)
        .filter(|i| config.wheel_steerable[*i] != 0)
        .map(forward)
        .fold(f64::NEG_INFINITY, f64::max);
    let front: Vec<usize> = (0..count)
        .filter(|i| config.wheel_steerable[*i] != 0 && forward(*i) > front_most - 0.5)
        .collect();
    let rear: Vec<usize> = (0..count)
        .filter(|i| config.wheel_steerable[*i] == 0)
        .collect();
    ensure!(
        front.len() == 2 && rear.len() >= 2 && rear.len().is_multiple_of(2),
        "仅支持前轴转向的牵引车头（4×2、6×2、6×4 等）"
    );
    let mean = |set: &[usize], f: &dyn Fn(usize) -> f64| {
        set.iter().map(|i| f(*i)).sum::<f64>() / set.len() as f64
    };
    let rear_forward = mean(&rear, &forward);
    let rear_left = mean(&rear, &|i| -(config.wheel_positions[i][0] as f64));
    let wheelbase = mean(&front, &forward) - rear_forward;
    ensure!(
        (2.5..=6.5).contains(&wheelbase) && rear_left.abs() < 0.2,
        "车辆轴距/后轴参考点不支持"
    );
    Ok((
        Axles {
            rear_forward,
            rear_left,
            wheelbase,
        },
        front,
    ))
}
/// Every measured steering table found, selected by the current truck's
/// geometry fingerprint, so changing trucks needs no manual configuration.
#[derive(Clone, Default)]
pub struct SteeringLibrary {
    maps: Vec<SteeringMap>,
    pub sources: Vec<std::path::PathBuf>,
}
impl SteeringLibrary {
    /// All valid reports in `dir` (one per truck) plus `extra` if given.
    /// Unreadable or invalid files are skipped, never trusted.
    pub fn load(dir: &Path, extra: Option<&Path>) -> Self {
        let mut library = Self::default();
        let mut paths: Vec<_> = std::fs::read_dir(dir)
            .map(|entries| {
                entries
                    .filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| p.extension().is_some_and(|x| x == "json"))
                    .collect()
            })
            .unwrap_or_default();
        paths.sort();
        paths.extend(extra.map(Path::to_path_buf));
        for path in paths {
            if let Ok(map) = SteeringMap::read(&path)
                && !library
                    .maps
                    .iter()
                    .any(|m| m.fingerprint == map.fingerprint)
            {
                library.maps.push(map);
                library.sources.push(path);
            }
        }
        library
    }
    pub fn len(&self) -> usize {
        self.maps.len()
    }
    pub fn is_empty(&self) -> bool {
        self.maps.is_empty()
    }
    pub fn vehicle(&self, config: &ConfigBlock) -> anyhow::Result<(Axles, &SteeringMap)> {
        let fingerprint =
            etsads_msgs::vehicle::geometry_fingerprint(config).map_err(anyhow::Error::msg)?;
        let map = self
            .maps
            .iter()
            .find(|m| m.fingerprint == fingerprint)
            .context(STEERING_NOT_CALIBRATED)?;
        Ok((vehicle_geometry(config)?.0, map))
    }
}
pub const STEERING_NOT_CALIBRATED: &str =
    "这辆车还没有标定转向：停稳并拉起驻车制动后，在面板点“标定转向”（约半分钟）";
#[derive(Clone, Copy)]
pub struct Axles {
    pub rear_forward: f64,
    pub rear_left: f64,
    pub wheelbase: f64,
}
impl Axles {
    pub fn rear_pose(self, sdk: Pose2) -> Pose2 {
        Pose2 {
            x: sdk.x + self.rear_forward * sdk.yaw.cos() - self.rear_left * sdk.yaw.sin(),
            y: sdk.y + self.rear_forward * sdk.yaw.sin() + self.rear_left * sdk.yaw.cos(),
            yaw: sdk.yaw,
        }
    }
}

pub struct Road {
    pub id: String,
    pub(super) points: Vec<Point3>,
    stations: Vec<f64>,
    bounds: [f64; 4],
}
impl Road {
    pub(super) fn path(id: String, points: Vec<Point3>) -> anyhow::Result<Self> {
        ensure!(
            points.len() >= 2 && points.iter().all(|p| p.valid()),
            "无效导航几何"
        );
        let mut stations = vec![0.0];
        for p in points.windows(2) {
            let d = p[0].distance_xy(p[1]);
            ensure!(d > 1e-6, "导航包含重复点");
            stations.push(stations.last().unwrap() + d);
        }
        let bounds = points.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |b, p| [b[0].min(p.x), b[1].min(p.y), b[2].max(p.x), b[3].max(p.y)],
        );
        Ok(Self {
            id,
            points,
            stations,
            bounds,
        })
    }
    pub fn length(&self) -> f64 {
        *self.stations.last().unwrap()
    }
    /// Whether (x, y) lies within `margin` of this road's bounding box.
    pub(super) fn near(&self, x: f64, y: f64, margin: f64) -> bool {
        x >= self.bounds[0] - margin
            && x <= self.bounds[2] + margin
            && y >= self.bounds[1] - margin
            && y <= self.bounds[3] + margin
    }
    /// Mean curvature (1/m) over `center ± half`, from the heading change.
    /// Headings are chords over ±1.5 m: map points are f32, about 8 mm
    /// steps at 110 km from the origin, so 1 m segment headings scatter by
    /// ±0.008 rad and the feed-forward swayed the wheel (r15). The chords
    /// sit inside the window so it still spans `center ± half` (with the
    /// half metre a segment heading spans), as the steering lag was tuned.
    pub(super) fn mean_curvature(&self, center: f64, half: f64) -> f64 {
        let inset = (HEADING_BASE_M - 0.5).min(half - 0.5).max(0.0);
        let a = (center - half + inset).clamp(0.0, self.length());
        let b = (center + half - inset).clamp(0.0, self.length());
        if b - a < 1.0 {
            return 0.0;
        }
        match (
            self.chord_heading(a, HEADING_BASE_M),
            self.chord_heading(b, HEADING_BASE_M),
        ) {
            (Some(ha), Some(hb)) => wrap_angle(hb - ha) / (b - a),
            _ => 0.0,
        }
    }
    /// Direction of the chord from `s - base` to `s + base` (clamped to
    /// the path).
    fn chord_heading(&self, s: f64, base: f64) -> Option<f64> {
        let (p, _) = self.sample((s - base).max(0.0))?;
        let (q, _) = self.sample((s + base).min(self.length()))?;
        (p.distance_xy(q) > 0.5).then(|| (q.y - p.y).atan2(q.x - p.x))
    }
    /// Signed curvature (left positive) of the circle through the path
    /// points at `s - half`, `s` and `s + half` (Menger curvature). Positions are smooth across lane
    /// joints where per-segment headings jump by a few hundredths of a
    /// radian, which as a 4 m heading difference read as a 100 m bend.
    pub(super) fn chord_curvature(&self, s: f64, half: f64) -> f64 {
        let a = (s - half).clamp(0.0, self.length());
        let c = (s + half).clamp(0.0, self.length());
        if c - a < 2.0 {
            return 0.0;
        }
        match (self.sample(a), self.sample((a + c) / 2.0), self.sample(c)) {
            (Some((p, _)), Some((q, _)), Some((r, _))) => {
                let cross = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
                let product = p.distance_xy(q) * q.distance_xy(r) * r.distance_xy(p);
                if product < 1e-6 {
                    0.0
                } else {
                    2.0 * cross / product
                }
            }
            _ => 0.0,
        }
    }
    pub(super) fn window(&self, start: f64, end: f64) -> (&[Point3], f64) {
        let first = self
            .stations
            .partition_point(|s| *s <= start)
            .saturating_sub(1)
            .min(self.points.len() - 2);
        let last = self
            .stations
            .partition_point(|s| *s < end)
            .min(self.points.len() - 1)
            .max(first + 1);
        (&self.points[first..=last], self.stations[first])
    }
    pub(super) fn sample(&self, s: f64) -> Option<(Point3, f64)> {
        if s < 0.0 || s > self.length() {
            return None;
        }
        let i = self
            .stations
            .partition_point(|v| *v <= s)
            .saturating_sub(1)
            .min(self.points.len() - 2);
        let t = (s - self.stations[i]) / (self.stations[i + 1] - self.stations[i]);
        let a = self.points[i];
        let b = self.points[i + 1];
        Some((
            Point3 {
                x: a.x + t * (b.x - a.x),
                y: a.y + t * (b.y - a.y),
                z: a.z + t * (b.z - a.z),
            },
            polyline_tangent(&self.points, i).ok()?,
        ))
    }
}
#[derive(Default)]
pub struct Roads {
    pub(super) roads: HashMap<String, Road>,
    pub(super) feed_forward: FeedForward,
}
/// A curve from a truck standing off the lane network (a yard, a fuel
/// station, a parking lot) onto the start of a lane ahead of it.
pub struct LeadIn {
    pub state: RoadState,
    /// From the truck's rear axle to the lane point at `state.station_m`.
    pub points: Vec<Point3>,
    pub length_m: f64,
}
/// Tightest lead-in curve allowed (radius 9 m at walking pace).
const LEAD_IN_MAX_CURVATURE: f64 = 1.0 / 9.0;
impl Roads {
    /// The best lane to join when the truck is not on any lane.
    #[cfg(test)]
    pub fn lead_in(&self, pose: Pose2, z: f64) -> Option<LeadIn> {
        self.lead_ins(pose, z).into_iter().next()
    }
    /// Lanes to join when the truck is not on any lane, best first, one
    /// joining point per lane: ahead of it (6–45 m), roughly in its
    /// direction or turning onto a crossing road (≤ 100°), and reachable by
    /// a smooth curve. The route decides which.
    pub fn lead_ins(&self, pose: Pose2, z: f64) -> Vec<LeadIn> {
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
        let mut found = Vec::<(f64, LeadIn)>::new();
        for road in self.roads.values() {
            if !road.near(pose.x, pose.y, 50.0) {
                continue;
            }
            let mut best = None::<(f64, LeadIn)>;
            let mut station = 0.0;
            while station <= road.length() - 4.0 {
                let here = station;
                station += 2.0;
                let Some((p, heading)) = road.sample(here) else {
                    continue;
                };
                let (dx, dy) = (p.x - pose.x, p.y - pose.y);
                let forward = dx * c + dy * s;
                let lateral = -dx * s + dy * c;
                let turn = wrap_angle(heading - pose.yaw);
                // Nearly straight on, or a turn onto a crossing road toward the
                // side the lane lies on (a yard exit; L12 faced the road at
                // 90° and found no lane to join). The Bézier check below keeps
                // the radius at 9 m or more.
                let joinable = if turn.abs() <= 0.6 {
                    lateral.abs() <= 0.6 * forward
                } else {
                    turn.abs() <= 1.75
                        && lateral * turn > 0.0
                        && lateral.abs() <= 1.5 * forward + 2.0
                };
                if !(6.0..=45.0).contains(&forward) || !joinable || (p.z - z).abs() > 2.0 {
                    continue;
                }
                let score = forward + 4.0 * lateral.abs() + 8.0 * turn.abs();
                if best.as_ref().is_some_and(|(b, _)| *b <= score) {
                    continue;
                }
                let Some(points) = bezier(pose, z, p, heading) else {
                    continue;
                };
                let length_m = points.windows(2).map(|w| w[0].distance_xy(w[1])).sum();
                best = Some((
                    score,
                    LeadIn {
                        state: RoadState {
                            lane: road.id.clone(),
                            station_m: here,
                            lateral_m: 0.0,
                            heading_error_rad: 0.0,
                            remaining_m: road.length() - here,
                            curvature: 0.0,
                            preview: p,
                            curvature_correction: 0.0,
                        },
                        points,
                        length_m,
                    },
                ));
            }
            found.extend(best);
        }
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        found.into_iter().map(|f| f.1).collect()
    }
}
/// Cubic Bézier from the truck pose to `end` with heading `end_heading`,
/// sampled about every metre; `None` if it bends tighter than allowed.
fn bezier(pose: Pose2, z: f64, end: Point3, end_heading: f64) -> Option<Vec<Point3>> {
    let distance = (end.x - pose.x).hypot(end.y - pose.y);
    // Control lengths: a third of the chord for a nearly straight join; for
    // a turn, 0.55 of each leg to where the two tangents meet, which traces
    // a near-circular arc (a third of the chord bent a 90° yard exit at 1/7.7).
    let (h0, h1) = (pose.yaw.sin_cos(), end_heading.sin_cos());
    let (h0, h1) = ((h0.1, h0.0), (h1.1, h1.0));
    let cross = h0.0 * h1.1 - h0.1 * h1.0;
    let (d0, d1) = if wrap_angle(end_heading - pose.yaw).abs() > 0.6 && cross.abs() > 1e-6 {
        let (rx, ry) = (end.x - pose.x, end.y - pose.y);
        let a = (rx * h1.1 - ry * h1.0) / cross;
        let b = (h0.0 * ry - h0.1 * rx) / cross;
        if a <= 0.0 || b <= 0.0 {
            return None;
        }
        (0.55 * a, 0.55 * b)
    } else {
        (distance / 3.0, distance / 3.0)
    };
    let p0 = (pose.x, pose.y);
    let p1 = (pose.x + d0 * h0.0, pose.y + d0 * h0.1);
    let p2 = (end.x - d1 * h1.0, end.y - d1 * h1.1);
    let p3 = (end.x, end.y);
    let n = (distance.ceil() as usize).max(4);
    let points: Vec<Point3> = (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let u = 1.0 - t;
            let b = |a: f64, b: f64, c: f64, d: f64| {
                u * u * u * a + 3.0 * u * u * t * b + 3.0 * u * t * t * c + t * t * t * d
            };
            Point3 {
                x: b(p0.0, p1.0, p2.0, p3.0),
                y: b(p0.1, p1.1, p2.1, p3.1),
                z: z + (end.z - z) * t,
            }
        })
        .collect();
    let headings: Vec<f64> = points
        .windows(2)
        .map(|w| (w[1].y - w[0].y).atan2(w[1].x - w[0].x))
        .collect();
    let tight = points.windows(3).zip(headings.windows(2)).any(|(w, h)| {
        let step = (w[0].distance_xy(w[1]) + w[1].distance_xy(w[2])) / 2.0;
        wrap_angle(h[1] - h[0]).abs() / step.max(1e-3) > LEAD_IN_MAX_CURVATURE
    });
    (!tight).then_some(points)
}
#[derive(Clone, Debug, Serialize)]
pub struct RoadState {
    pub lane: String,
    pub station_m: f64,
    pub lateral_m: f64,
    pub heading_error_rad: f64,
    pub remaining_m: f64,
    pub curvature: f64,
    pub preview: Point3,
    /// Path curvature near the truck minus the chord curvature to `preview`
    /// (1/m); added to the pure-pursuit command.
    pub curvature_correction: f64,
}
impl RoadState {
    /// The same lane followed `left_m` to the left of its centre (negative:
    /// to the right): the preview point moves sideways and the offset is
    /// measured from the shifted line.
    pub fn offset(&self, pose: Pose2, left_m: f64) -> Self {
        if left_m == 0.0 || !left_m.is_finite() {
            return self.clone();
        }
        let heading = (self.preview.y - pose.y).atan2(self.preview.x - pose.x);
        Self {
            lateral_m: self.lateral_m - left_m,
            preview: Point3 {
                x: self.preview.x - left_m * heading.sin(),
                y: self.preview.y + left_m * heading.cos(),
                z: self.preview.z,
            },
            ..self.clone()
        }
    }
}
impl Roads {
    pub fn from_export(map: ExportedMap) -> anyhow::Result<Self> {
        ensure!(
            map.schema_version == 1 && map.source == "installed_game_archives",
            "仅支持本机游戏归档导出的地图"
        );
        let mut roads = HashMap::new();
        for lane in map.lanes {
            // Lanes closed only to AI trucks are real roads the player may
            // drive (service areas); the route decides whether to use them.
            if !lane.centerline_validated
                || lane.truck_access.is_none()
                || lane.center.len() < 2
                || lane.center.iter().any(|p| !p.valid())
            {
                continue;
            }
            let mut stations = vec![0.0];
            let mut valid = true;
            for pair in lane.center.windows(2) {
                let length = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
                if !length.is_finite() || length < 1e-6 {
                    valid = false;
                    break;
                }
                stations.push(stations.last().unwrap() + length);
            }
            if !valid || *stations.last().unwrap() < 0.5 {
                continue;
            }
            let bounds = lane.center.iter().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |b, p| [b[0].min(p.x), b[1].min(p.y), b[2].max(p.x), b[3].max(p.y)],
            );
            roads.insert(
                lane.id.clone(),
                Road {
                    id: lane.id,
                    points: lane.center,
                    stations,
                    bounds,
                },
            );
        }
        ensure!(!roads.is_empty(), "附近地图没有可用于短时辅助的连续道路");
        Ok(Self {
            roads,
            feed_forward: ABSOLUTE_FEED_FORWARD,
        })
    }
    pub fn observe(
        &self,
        pose: Pose2,
        z: f64,
        speed: f64,
        locked: Option<&str>,
    ) -> anyhow::Result<RoadState> {
        self.observe_near(pose, z, speed, locked, None)
    }
    /// As `observe`, projecting only onto the stretch from 20 m behind to
    /// 40 m ahead of station `near`: a route through a service area and back
    /// across the same junction crosses itself, and the nearest point was on
    /// the later pass, at right angles (L18, L22 stopped there).
    pub fn observe_near(
        &self,
        pose: Pose2,
        z: f64,
        speed: f64,
        locked: Option<&str>,
        near: Option<f64>,
    ) -> anyhow::Result<RoadState> {
        ensure!(
            pose.valid() && z.is_finite() && speed.is_finite(),
            "车辆位置无效"
        );
        let lookahead = (6.0 + speed * 1.0).clamp(6.0, 32.0);
        let mut candidates = Vec::new();
        for road in self.roads.values() {
            if locked.is_some_and(|id| id != road.id)
                || pose.x < road.bounds[0] - 3.0
                || pose.x > road.bounds[2] + 3.0
                || pose.y < road.bounds[1] - 3.0
                || pose.y > road.bounds[3] + 3.0
            {
                continue;
            }
            let (slice, offset) = match near {
                Some(s) => road.window((s - 20.0).max(0.0), s + 40.0),
                None => (road.points.as_slice(), 0.0),
            };
            let Ok((mut p, _)) = project_polyline(
                0,
                slice,
                Point3 {
                    x: pose.x,
                    y: pose.y,
                    z,
                },
            ) else {
                continue;
            };
            p.s += offset;
            let heading = wrap_angle(p.heading - pose.yaw);
            // An engaged truck keeps its locked path while it corrects a drift;
            // free matching stays strict so engagement starts centred. Within
            // its station window a locked path cannot be confused with a road
            // above or below, and exported heights can be off on crests (r13:
            // 1.7 m on a climbing rural road, flat in the map).
            let (max_distance, max_heading, max_height) = if locked.is_some() && near.is_some() {
                (LOCKED_MAX_LATERAL_M + 0.4, 0.5, 3.5)
            } else if locked.is_some() {
                (LOCKED_MAX_LATERAL_M + 0.4, 0.5, 2.5)
            } else {
                (2.0, 0.26, 2.5)
            };
            if p.distance > max_distance
                || (p.z - z).abs() > max_height
                || heading.abs() > max_heading
            {
                continue;
            }
            let remaining = road.length() - p.s;
            // A free match must have road ahead; a locked one may be just
            // short of its lane's end inside a junction (rolling extension).
            if remaining < 2.0 && locked.is_none() {
                continue;
            }
            let Some((preview, preview_heading)) =
                road.sample((p.s + lookahead).min(road.length()))
            else {
                continue;
            };
            let curvature = wrap_angle(preview_heading - p.heading) / lookahead;
            // On the path, pure pursuit commands the chord curvature toward the
            // preview point, which anticipates a change of curvature by about
            // half the lookahead: entering a 14 m roundabout after a right-hand
            // flare it cut 1 m inside (r10). Replace the path's share of that
            // command by the curvature just ahead of the truck; the share that
            // depends on the truck's own offset stays pure pursuit.
            let Some((origin, _)) = road.sample(p.s) else {
                continue;
            };
            let (cx, cy) = (preview.x - origin.x, preview.y - origin.y);
            let chord_length = cx.hypot(cy);
            // The path heading over a chord, not the f32-noisy segment.
            let tangent = road.chord_heading(p.s, HEADING_BASE_M).unwrap_or(p.heading);
            let chord = if chord_length > 1.0 {
                2.0 * wrap_angle(cy.atan2(cx) - tangent).sin() / chord_length
            } else {
                0.0
            };
            let near = road.mean_curvature(
                p.s + speed.max(0.0) * self.feed_forward.lag_s,
                self.feed_forward.half_m + 0.25 * speed.abs(),
            );
            let near = match self.feed_forward.curvature_rate {
                Some(rate) => reachable_curvature(road, p.s, speed, near, rate),
                None => near,
            };
            let curvature_correction = (near - chord).clamp(-0.1, 0.1);
            candidates.push((
                p.distance.powi(2) + heading.powi(2) * 16.0,
                RoadState {
                    lane: road.id.clone(),
                    station_m: p.s,
                    lateral_m: p.d,
                    heading_error_rad: heading,
                    remaining_m: remaining,
                    curvature,
                    preview,
                    curvature_correction,
                },
            ));
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        let best = candidates
            .first()
            .context("没有匹配到同向车道，请先手动驶入道路中央")?;
        ensure!(
            locked.is_some() || candidates.get(1).is_none_or(|next| next.0 - best.0 > 0.5),
            "车道识别不确定，请保持在车道中央"
        );
        ensure!(
            best.1.lateral_m.abs()
                <= if locked.is_some() {
                    LOCKED_MAX_LATERAL_M
                } else {
                    0.55
                },
            "偏离车道中心，请手动修正"
        );
        Ok(best.1.clone())
    }
}

/// With relative steering the wheel catches up slowly: while it is well
/// short of where the controller wants it, or the truck is off centre, the
/// speed is held. r14 sped from 6 to 16 km/h out of a bend 0.6 m off and
/// swung ±1 m; capping at the current speed let it creep up 8 → 26 km/h.
/// The cap only falls during a hold: r16 latched 16 km/h going into a
/// junction turn, slowed to 9 for the turn and sped back up to 18 on the
/// exit while the wheel was still unwinding, and swayed ±1.5 m.
#[derive(Debug, Default)]
pub struct RelativeHold {
    held: Option<f64>,
}
impl RelativeHold {
    /// The speed cap (m/s) for this step, if holding.
    pub fn cap(&mut self, lag: f64, lateral_m: f64, speed: f64) -> Option<f64> {
        let holding = match self.held {
            None => lag > RELATIVE_CATCH_UP || lateral_m.abs() >= RELATIVE_HOLD_OFFSET_M,
            Some(_) => {
                lag > RELATIVE_CATCH_UP / 2.0 || lateral_m.abs() >= RELATIVE_RELEASE_OFFSET_M
            }
        };
        self.held = holding.then(|| {
            let speed = speed.max(RELATIVE_CREEP);
            self.held.map_or(speed, |held| held.min(speed))
        });
        self.held
    }
}
/// Steering lag (position) and offset that start a hold; walking pace is
/// always allowed.
const RELATIVE_CATCH_UP: f64 = 0.08;
const RELATIVE_HOLD_OFFSET_M: f64 = 0.35;
/// The hold ends once the truck is back within this offset.
const RELATIVE_RELEASE_OFFSET_M: f64 = 0.2;
const RELATIVE_CREEP: f64 = 5.0 / 3.6;
/// mvp-r7: brake 1.0 took 14.9 → 1.1 km/h in about 0.43 s (≈9 m/s²); r8:
/// brake 0.5 stopped 5.5 km/h at ≈4 m/s². Planned braking maps through this
/// gain and never exceeds `MAX_PLANNED_BRAKE`; harder braking is reserved for
/// the independent supervisor's AEB.
const BRAKE_DECELERATION_PER_UNIT: f64 = 8.5;
const MAX_PLANNED_BRAKE: f64 = 0.6;
const BRAKE_RISE_PER_S: f64 = 2.0;
const BRAKE_FALL_PER_S: f64 = 1.0;
const THROTTLE_RISE_PER_S: f64 = 0.6;
const MAX_THROTTLE: f64 = 1.0;
/// Planned deceleration (m/s²) and overspeed (m/s) left to coasting.
const COAST_DECELERATION: f64 = 0.5;
const OVERSPEED_DEADBAND: f64 = 0.4;
/// Integral authority 0.08 × 5 = 0.4 throttle for grade and load.
const THROTTLE_INTEGRAL_MAX: f64 = 5.0;
/// Below this speed the brake eases toward `0.2 + 0.15 v`, as a driver lifts
/// off before the stop; a hard stop pitches the cab and the wheel speed
/// rebounds negative (mvp-r7 exit at −0.22 m/s).
const SOFT_STOP_SPEED: f64 = 2.0;
const STOPPED_SPEED: f64 = 0.05;
const HOLD_BRAKE: f64 = 0.3;
/// Standstill must be settled and the restart request sustained this long.
const RESTART_DWELL_S: f64 = 1.0;
/// From standstill the planner caps its target at one second of acceleration
/// (1.0 m/s), so the threshold must sit clearly below that (r9 deadlock).
const RESTART_TARGET: f64 = 0.5;
/// Throttle stays available below this speed during a coasting approach.
const CREEP_SPEED: f64 = 1.5;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub enum Longitudinal {
    Driving,
    /// Standstill held on the service brake until a settled, sustained restart.
    Holding {
        settled_s: f64,
        restart_s: f64,
    },
}

pub struct Feedback {
    integral: f64,
    /// Leaky integral of the lateral error (m·s), trimming a steady steering
    /// bias such as road camber or a neutral offset (r11: 0.6 m drift on a
    /// straight road with the wheel already slightly right).
    lateral_integral: f64,
    last_axis: f64,
    throttle: f64,
    brake: f64,
    pub longitudinal: Longitudinal,
    /// Final approach to a planned stop: never throttle above creep speed;
    /// an over-braked truck coasts instead of re-accelerating (r9).
    pub coast: bool,
    axles: Axles,
    steering: SteeringMap,
    /// With relative steering the wheel turns at this rate per unit input;
    /// the position is shaped within rate and acceleration limits so the
    /// servo can follow it.
    relative_rate: Option<f64>,
    axis_rate: f64,
    /// The unshaped position and its smoothed rate.
    desired: Option<f64>,
    desired_rate: f64,
    /// The measured steering position of this frame (relative steering).
    measured: Option<f64>,
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Output {
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
}
impl Feedback {
    pub fn new(axles: Axles, steering: SteeringMap) -> Self {
        Self {
            integral: 0.0,
            lateral_integral: 0.0,
            last_axis: 0.0,
            throttle: 0.0,
            brake: 0.0,
            longitudinal: Longitudinal::Driving,
            coast: false,
            axles,
            steering,
            relative_rate: None,
            axis_rate: 0.0,
            desired: None,
            desired_rate: 0.0,
            measured: None,
        }
    }
    /// Start from the measured steering position (with relative steering the
    /// wheel stays wherever the driver left it) and, for relative steering,
    /// shape the position to what its turning rate can follow.
    pub fn with_steering(mut self, position: Option<f64>, relative_rate: Option<f64>) -> Self {
        self.last_axis = position.unwrap_or(0.0).clamp(-1.0, 1.0);
        self.relative_rate = relative_rate;
        self
    }
    /// `planned` is the speed planner's acceleration (m/s²); its braking is a
    /// feed-forward deceleration, overspeed adds proportional braking.
    pub fn step(
        &mut self,
        road: &RoadState,
        pose: Pose2,
        speed: f64,
        target: f64,
        planned: f64,
        dt: f64,
    ) -> anyhow::Result<Output> {
        ensure!(
            speed.is_finite()
                && target.is_finite()
                && planned.is_finite()
                && (0.0..=MAX_SPEED).contains(&target)
                && (0.001..=0.1).contains(&dt),
            "控制时钟或速度异常"
        );
        let dx = road.preview.x - pose.x;
        let dy = road.preview.y - pose.y;
        let distance = dx.hypot(dy);
        ensure!(distance >= 2.0, "预瞄点过近");
        let alpha = wrap_angle(dy.atan2(dx) - pose.yaw);
        // A large correction is steered at the limit, not refused: the drift
        // monitor (slow down from 0.6 m, stop from 1.6 m) guards the outcome.
        // r11 refused 0.45 rad at a junction exit with only 0.7 m of error.
        ensure!(alpha.abs() < 1.2, "所需修正过大，请手动接管");
        let angle = (self.axles.wheelbase
            * (2.0 * alpha.sin() / distance + road.curvature_correction))
            .atan();
        // Pure pursuit has no integral action, so a steady bias leaves a
        // steady offset that grows with the lookahead. Trim it slowly; the
        // integral leaks so a transient (a bend, a lane change) fades out.
        if speed > 5.0 {
            self.lateral_integral = (self.lateral_integral * (1.0 - LATERAL_LEAK_PER_S * dt)
                + road.lateral_m * dt)
                .clamp(-LATERAL_INTEGRAL_MAX, LATERAL_INTEGRAL_MAX);
        }
        let angle = angle - LATERAL_TRIM_GAIN * self.lateral_integral;
        // The measured map (r9: up to ±0.72 rad mean wheel angle) is bounded
        // by a speed-dependent lateral-acceleration envelope; beyond it the
        // wheel is held at the envelope and the speed planner slows down.
        let limit =
            (self.axles.wheelbase * MAX_LATERAL_ACCELERATION / speed.abs().max(1.0).powi(2)).atan();
        // Beyond the calibrated range the wheel is held at its edge, never
        // extrapolated: r13 stopped on a 5 m yard exit that needed more.
        let (low, high) = self.steering.range();
        let desired = self
            .steering
            .axis(angle.clamp(-limit, limit).clamp(low, high))?;
        // Axis slew: 0.5/s (validated up to 42 km/h) from 10 m/s; a roundabout
        // entry at walking pace needs the wheel turned twice as fast.
        let slew = (5.0 / speed.abs().max(0.1)).clamp(0.5, 1.0);
        let axis = if let Some(rate_per_s) = self.relative_rate {
            let (max_rate, max_accel) = super::io::relative_limits(
                rate_per_s,
                speed,
                desired.abs().max(self.last_axis.abs()),
            );
            // The pursuit target moves with the road; its smoothed rate lets
            // the shaped position follow a steady turn-in without lagging.
            let rate = self.desired.map_or(0.0, |d| (desired - d) / dt);
            self.desired = Some(desired);
            self.desired_rate += (rate.clamp(-max_rate, max_rate) - self.desired_rate)
                * (dt / RELATIVE_RATE_SMOOTHING_S).min(1.0);
            // The smoothed rate lags a target that stops: carried on, it
            // swung the wheel past the target (r16). Only the share the
            // target is still moving at counts.
            let goal_rate = if rate * self.desired_rate > 0.0 {
                self.desired_rate.signum() * self.desired_rate.abs().min(rate.abs())
            } else {
                0.0
            };
            let (axis, rate) = super::io::shape_steering(
                self.last_axis,
                self.axis_rate,
                desired,
                goal_rate,
                (max_rate.min(slew), max_accel),
                dt,
            );
            self.axis_rate = rate;
            // The command leads the measured wheel by at most RELATIVE_LEAD:
            // run further ahead, the pursuit saw the truck not turning as
            // commanded, steered on and swayed ±1.3 m at 15 km/h
            // (simulation; r16 swayed ±1.5 m the same way).
            match self.measured {
                Some(measured) if (axis - measured).abs() > RELATIVE_LEAD => {
                    let held = axis.clamp(measured - RELATIVE_LEAD, measured + RELATIVE_LEAD);
                    self.axis_rate = (held - self.last_axis) / dt;
                    held
                }
                _ => axis,
            }
        } else {
            desired.clamp(self.last_axis - slew * dt, self.last_axis + slew * dt)
        };
        self.last_axis = axis;
        let (throttle, brake) = self.pedals(speed, target, planned, dt);
        Ok(Output {
            steer: axis,
            throttle,
            brake,
        })
    }
    /// The steering position measured this frame, which a relatively
    /// steered command may lead only a little.
    pub fn observe_position(&mut self, position: Option<f64>) {
        self.measured = position.filter(|p| p.is_finite());
    }
    /// The last steering axis sent, held during a minimal-risk stop.
    pub fn held_axis(&self) -> f64 {
        self.last_axis
    }
    /// How far the shaped steering position is behind the controller's
    /// (relative steering only; 0 otherwise).
    pub fn steering_lag(&self) -> f64 {
        self.desired.map_or(0.0, |d| (d - self.last_axis).abs())
    }
    /// The larger of the shaping lag and how far the measured steering
    /// `position` is behind the last command: accelerating out of a bend the
    /// game turned the wheel too slowly for the servo, and the truck swayed
    /// into a safety stop at 33 km/h (simulation).
    pub fn tracking_lag(&self, position: Option<f64>) -> f64 {
        self.steering_lag()
            .max(position.map_or(0.0, |p| (p - self.last_axis).abs()))
    }
    fn pedals(&mut self, speed: f64, target: f64, planned: f64, dt: f64) -> (f64, f64) {
        let error = target - speed;
        let mut brake = 0.0_f64;
        // Lifting off gives about 0.3 m/s² of drag: light deceleration and a
        // small overspeed are coasted. Following a leader, braking at
        // −0.2 m/s² or 0.15 m/s over cycled the throttle 0 ↔ 0.5 every
        // second (L4).
        if planned < -COAST_DECELERATION {
            brake = (-planned - COAST_DECELERATION * 0.6) / BRAKE_DECELERATION_PER_UNIT;
        }
        if error < -OVERSPEED_DEADBAND {
            brake = brake.max((-error - OVERSPEED_DEADBAND * 0.5) * 0.16);
        }
        if target < 0.1 && speed > STOPPED_SPEED {
            brake = brake.max(1.0 / BRAKE_DECELERATION_PER_UNIT);
        }
        brake = brake.min(MAX_PLANNED_BRAKE);
        if brake > 0.0 && speed < SOFT_STOP_SPEED {
            brake = brake.min(0.2 + 0.15 * speed.max(0.0));
        }
        let stopping = brake > 0.0 || target < RESTART_TARGET;
        self.longitudinal = match self.longitudinal {
            Longitudinal::Driving if speed.abs() < STOPPED_SPEED && stopping => {
                Longitudinal::Holding {
                    settled_s: 0.0,
                    restart_s: 0.0,
                }
            }
            Longitudinal::Holding {
                settled_s,
                restart_s,
            } => {
                let settled_s = if speed.abs() < STOPPED_SPEED {
                    settled_s + dt
                } else {
                    0.0
                };
                let restart_s = if brake == 0.0 && target >= RESTART_TARGET {
                    restart_s + dt
                } else {
                    0.0
                };
                if settled_s >= RESTART_DWELL_S && restart_s >= RESTART_DWELL_S {
                    Longitudinal::Driving
                } else {
                    Longitudinal::Holding {
                        settled_s,
                        restart_s,
                    }
                }
            }
            state => state,
        };
        let mut throttle = 0.0;
        if matches!(self.longitudinal, Longitudinal::Holding { .. }) {
            brake = brake.max(HOLD_BRAKE);
            self.integral = 0.0;
        } else if brake == 0.0 {
            // Conditional integration: only near the target, so a long
            // acceleration does not wind up into an overshoot past the limit.
            if error.abs() < 1.5 {
                self.integral = (self.integral + error * dt)
                    .clamp(-THROTTLE_INTEGRAL_MAX, THROTTLE_INTEGRAL_MAX);
            }
            // The cruise feed-forward rises with speed (drag); the integral
            // carries grade and load. r11 capped the throttle at 0.5 with a
            // 0.05 integral authority: loaded uphill it never reached the
            // limit and every restart in town crawled.
            throttle = (0.08 + 0.006 * speed.max(0.0) + 0.30 * error + 0.08 * self.integral)
                .clamp(0.0, MAX_THROTTLE);
        }
        if self.coast && speed > CREEP_SPEED {
            throttle = 0.0;
            self.integral = 0.0;
        }
        // Brake rises fast enough for planned stops and falls smoothly; no
        // throttle is applied until the brake is fully released.
        self.brake = brake.clamp(
            self.brake - BRAKE_FALL_PER_S * dt,
            self.brake + BRAKE_RISE_PER_S * dt,
        );
        if self.brake > 0.005 {
            throttle = 0.0;
        } else {
            self.brake = 0.0;
        }
        self.throttle = throttle.min(self.throttle + THROTTLE_RISE_PER_S * dt);
        (self.throttle, self.brake)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn calibration() -> SteeringMap {
        SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.175, 0.25), (-0.07, 0.1), (0.07, -0.1), (0.175, -0.25)],
        }
    }
    fn roads() -> Roads {
        Roads::from_export(ExportedMap {
            schema_version: 1,
            source: "installed_game_archives".into(),
            accepted_for_autonomous_driving: false,
            lanes: vec![etsads_map::importer::ExportLane {
                id: "1:road:right:0".into(),
                center: vec![
                    Point3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    Point3 {
                        x: 300.0,
                        y: 0.0,
                        z: 0.0,
                    },
                ],
                centerline_validated: true,
                truck_access: Some(true),
                ..Default::default()
            }],
        })
        .unwrap()
    }
    #[test]
    fn unknown_width_is_not_forged_and_offroad_intersection_end_are_rejected() {
        let map = roads();
        assert!(
            map.observe(
                Pose2 {
                    x: 20.0,
                    y: 0.2,
                    yaw: 0.0
                },
                0.0,
                5.0,
                None
            )
            .is_ok()
        );
        assert!(
            map.observe(
                Pose2 {
                    x: 20.0,
                    y: 3.0,
                    yaw: 0.0
                },
                0.0,
                5.0,
                None
            )
            .is_err()
        );
        assert!(
            map.observe(
                Pose2 {
                    x: 299.0,
                    y: 0.0,
                    yaw: 0.0
                },
                0.0,
                5.0,
                None
            )
            .is_err()
        );
        assert!(
            map.observe(
                Pose2 {
                    x: 20.0,
                    y: 0.0,
                    yaw: std::f64::consts::PI
                },
                0.0,
                5.0,
                None
            )
            .is_err()
        );
    }
    #[test]
    fn measured_negative_axis_polarity_corrects_lateral_error_without_extrapolation() {
        let steering = calibration();
        assert!(steering.axis(0.3).is_err());
        let pose = Pose2 {
            x: 20.0,
            y: 0.4,
            yaw: 0.0,
        };
        let road = roads().observe(pose, 0.0, 5.0, None).unwrap();
        let out = Feedback::new(
            Axles {
                wheelbase: 3.76,
                rear_forward: 0.0,
                rear_left: 0.0,
            },
            steering,
        )
        .step(&road, pose, 5.0, 5.0, 0.0, 0.02)
        .unwrap();
        assert!(out.steer > 0.0 && out.steer <= 0.02); // right wheel angle uses positive semantic input; one 1.0/s slew step
        assert_eq!(out.brake, 0.0);
    }
    #[test]
    fn closed_loop_reduces_offset_and_overspeed_brakes_without_throttle() {
        let map = roads();
        let steering = calibration();
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        let mut feedback = Feedback::new(axles, steering);
        let mut pose = Pose2 {
            x: 20.0,
            y: 0.4,
            yaw: 0.0,
        };
        for _ in 0..500 {
            let road = map.observe(pose, 0.0, 5.0, Some("1:road:right:0")).unwrap();
            let out = feedback.step(&road, pose, 5.0, 5.0, 0.0, 0.02).unwrap();
            let angle = -out.steer * 0.7;
            pose.x += 5.0 * pose.yaw.cos() * 0.02;
            pose.y += 5.0 * pose.yaw.sin() * 0.02;
            pose.yaw += 5.0 * angle.tan() / axles.wheelbase * 0.02;
        }
        assert!(pose.y.abs() < 0.05, "offset={}", pose.y);
        let road = map.observe(pose, 0.0, 8.0, None).unwrap();
        let out = feedback.step(&road, pose, 8.0, 5.0, 0.0, 0.02).unwrap();
        assert_eq!(out.throttle, 0.0);
        assert!(out.brake > 0.0);
    }
    #[test]
    fn steering_envelope_allows_tight_turns_only_at_low_speed() {
        // Full-range map in the shape of the r9 scan (mean wheel angle).
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![
                (-0.715, 1.0),
                (-0.175, 0.25),
                (-0.07, 0.1),
                (0.07, -0.1),
                (0.175, -0.25),
                (0.715, -1.0),
            ],
        };
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        // A 20 m-radius left turn starting at the truck.
        let pose = Pose2 {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        let a: f64 = 8.0 / 20.0;
        let road = RoadState {
            lane: "gps".into(),
            station_m: 0.0,
            lateral_m: 0.0,
            heading_error_rad: 0.0,
            remaining_m: 100.0,
            curvature: 0.05,
            preview: Point3 {
                x: 20.0 * a.sin(),
                y: 20.0 * (1.0 - a.cos()),
                z: 0.0,
            },
            curvature_correction: 0.0,
        };
        let mut slow = Feedback::new(axles, wide.clone());
        slow.last_axis = -0.4;
        let out = slow.step(&road, pose, 2.0, 2.0, 0.0, 0.02).unwrap();
        assert!(
            out.steer < -0.3,
            "full map is usable at walking pace: {}",
            out.steer
        );
        // At 72 km/h the same turn is held at the lateral-acceleration
        // envelope (about 0.024 rad of wheel), never at full lock.
        let mut fast = Feedback::new(axles, wide);
        fast.last_axis = -0.4;
        let out = fast.step(&road, pose, 20.0, 20.0, 0.0, 0.02).unwrap();
        assert!(out.steer > -0.4 && out.steer.abs() < 0.4, "{}", out.steer);
        for _ in 0..200 {
            fast.step(&road, pose, 20.0, 20.0, 0.0, 0.02).unwrap();
        }
        let held = fast.step(&road, pose, 20.0, 20.0, 0.0, 0.02).unwrap();
        assert!(
            held.steer.abs() < 0.05,
            "envelope-limited axis {}",
            held.steer
        );
    }
    /// r10: a right-hand entry flare straight into a 14 m-radius roundabout,
    /// driven at 15 km/h behind a 0.25 s steering lag.
    const RELATIVE_SIM_KMH: f64 = 5.0;
    fn roundabout_peak_offset(feed_forward: bool, relative: bool) -> f64 {
        let mut points = Vec::new();
        let (mut x, mut y, mut heading) = (-40.0_f64, 0.0_f64, 0.0_f64);
        let curvature = |s: f64| match s {
            s if s < 40.0 => 0.0,
            s if s < 40.0 + 20.0 * 0.45 => -1.0 / 20.0,
            s if s < 49.0 + 14.0 * std::f64::consts::PI => 1.0 / 14.0,
            _ => 0.0,
        };
        for i in 0..130 {
            points.push(Point3 { x, y, z: 0.0 });
            heading += curvature(i as f64 + 0.5);
            x += heading.cos();
            y += heading.sin();
        }
        let map = Roads {
            feed_forward: if relative {
                FeedForward::relative(0.4)
            } else {
                ABSOLUTE_FEED_FORWARD
            },
            roads: HashMap::from([("gps".into(), Road::path("gps".into(), points).unwrap())]),
        };
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.715, 1.0), (0.715, -1.0)],
        };
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        let mut feedback =
            Feedback::new(axles, wide).with_steering(Some(0.0), relative.then_some(0.4));
        let mut servo = super::super::io::SteeringServo::new(if relative {
            crate::setup::SteeringMode::Relative { rate_per_s: 0.4 }
        } else {
            crate::setup::SteeringMode::Absolute
        });
        let mut truck = RelativeTruck::new(Pose2 {
            x: -30.0,
            y: 0.0,
            yaw: 0.0,
        });
        let start = std::time::Instant::now();
        // Relative steering turns the wheel at about 0.29 rad/s at most; the
        // speed planner slows such a reversal to walking pace.
        let (speed, dt) = (if relative { RELATIVE_SIM_KMH } else { 15.0 } / 3.6, 0.02);
        let mut wheel = 0.0_f64;
        let mut peak = 0.0_f64;
        for i in 0..2500 {
            let now = start + std::time::Duration::from_secs_f64(i as f64 * 0.02);
            servo.observe(-truck.position, speed, now);
            let pose = truck.pose;
            let Ok(mut road) = map.observe(pose, 0.0, speed, Some("gps")) else {
                return f64::INFINITY;
            };
            if road.remaining_m < 25.0 {
                break;
            }
            peak = peak.max(road.lateral_m.abs());
            if !feed_forward {
                road.curvature_correction = 0.0;
            }
            let out = feedback.step(&road, pose, speed, speed, 0.0, dt).unwrap();
            let input = servo.input(out.steer, now);
            if relative {
                truck.step(input, speed, axles.wheelbase, dt);
            } else {
                // Absolute steering: the wheel follows the input about 0.5 s
                // later.
                truck.position = input.clamp(-1.0, 1.0);
                wheel += (-truck.position * 0.715 - wheel) * dt / 0.5;
                truck.pose.x += speed * truck.pose.yaw.cos() * dt;
                truck.pose.y += speed * truck.pose.yaw.sin() * dt;
                truck.pose.yaw += speed * wheel.tan() / axles.wheelbase * dt;
            }
        }
        peak
    }
    #[test]
    fn lead_in_joins_a_lane_ahead_from_a_yard_but_not_one_behind() {
        let map = roads();
        // Parked 6 m beside the lane, angled 25° toward it.
        let pose = Pose2 {
            x: 40.0,
            y: 6.0,
            yaw: -0.44,
        };
        assert!(map.observe(pose, 0.0, 0.0, None).is_err());
        let lead = map.lead_in(pose, 0.0).expect("lane ahead is joinable");
        assert_eq!(lead.state.lane, "1:road:right:0");
        assert!(lead.state.station_m > 46.0 && lead.state.station_m < 80.0);
        let first = lead.points[0];
        let last = *lead.points.last().unwrap();
        assert!((first.x - 40.0).abs() < 1e-9 && (first.y - 6.0).abs() < 1e-9);
        assert!(last.y.abs() < 1e-6, "ends on the lane: {last:?}");
        // Facing away from the lane direction: no lead-in.
        let reversed = Pose2 {
            yaw: std::f64::consts::PI,
            ..pose
        };
        assert!(map.lead_in(reversed, 0.0).is_none());
        // Facing the road at 90° from a yard exit 12 m short of it (L12):
        // join by a right turn onto the lane, downstream of the truck.
        let exit = Pose2 {
            x: 40.0,
            y: -12.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let lead = map.lead_in(exit, 0.0).expect("crossing road is joinable");
        assert!(lead.state.station_m > 46.0, "{}", lead.state.station_m);
        assert!(lead.points.iter().all(|p| p.x >= 40.0 - 1e-9));
    }
    #[test]
    fn lateral_trim_removes_a_steady_steering_bias_at_speed() {
        let map = Roads {
            feed_forward: ABSOLUTE_FEED_FORWARD,
            roads: HashMap::from([(
                "gps".into(),
                Road::path(
                    "gps".into(),
                    vec![
                        Point3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        Point3 {
                            x: 3000.0,
                            y: 0.0,
                            z: 0.0,
                        },
                    ],
                )
                .unwrap(),
            )]),
        };
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.715, 1.0), (0.715, -1.0)],
        };
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        // r11: the road pulls left by about 0.02 rad of wheel at 45 km/h.
        let run = |trim: bool| {
            let mut feedback = Feedback::new(axles, wide.clone());
            let mut pose = Pose2 {
                x: 10.0,
                y: 0.0,
                yaw: 0.0,
            };
            let (speed, dt, bias) = (12.5, 0.02, 0.02);
            let mut wheel = 0.0_f64;
            let mut last = 0.0;
            for _ in 0..(120.0 / dt) as usize {
                let road = map.observe(pose, 0.0, speed, Some("gps")).unwrap();
                if !trim {
                    feedback.lateral_integral = 0.0;
                }
                let out = feedback.step(&road, pose, speed, speed, 0.0, dt).unwrap();
                wheel += (-out.steer * 0.715 - wheel) * dt / 0.5;
                pose.x += speed * pose.yaw.cos() * dt;
                pose.y += speed * pose.yaw.sin() * dt;
                pose.yaw += speed * (wheel + bias).tan() / axles.wheelbase * dt;
                last = road.lateral_m;
            }
            last
        };
        let untrimmed = run(false);
        let trimmed = run(true);
        assert!(
            untrimmed > 0.3,
            "pursuit alone keeps an offset: {untrimmed}"
        );
        assert!(
            trimmed.abs() < 0.1,
            "trimmed offset {trimmed} (untrimmed {untrimmed})"
        );
    }
    #[test]
    fn curvature_feed_forward_holds_the_lane_through_a_roundabout_entry() {
        let pursuit = roundabout_peak_offset(false, false);
        let combined = roundabout_peak_offset(true, false);
        assert!(pursuit > 0.6, "pure pursuit alone cut inside: {pursuit}");
        assert!(
            combined < 0.3,
            "feed-forward peak offset {combined} (pursuit {pursuit})"
        );
        eprintln!("roundabout peak offset: pursuit {pursuit:.2} m, feed-forward {combined:.2} m");
    }
    /// Peak offset over the last 20 s after starting 0.8 m off a straight
    /// lane at `kmh`, with relative steering, and how often it crossed the
    /// centre (r14 oscillated ±1 m at 15 km/h after a junction).
    fn relative_offset_recovery(kmh: f64) -> (f64, usize) {
        let map = Roads {
            feed_forward: FeedForward::relative(0.4),
            roads: HashMap::from([(
                "gps".into(),
                Road::path(
                    "gps".into(),
                    vec![
                        Point3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        Point3 {
                            x: 1000.0,
                            y: 0.0,
                            z: 0.0,
                        },
                    ],
                )
                .unwrap(),
            )]),
        };
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.715, 1.0), (0.715, -1.0)],
        };
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        let mut feedback = Feedback::new(axles, wide).with_steering(Some(0.0), Some(0.4));
        let mut servo =
            super::super::io::SteeringServo::new(crate::setup::SteeringMode::Relative {
                rate_per_s: 0.4,
            });
        let mut truck = RelativeTruck::new(Pose2 {
            x: 10.0,
            y: 0.8,
            yaw: 0.0,
        });
        let (speed, dt) = (kmh / 3.6, 0.02);
        let start = std::time::Instant::now();
        let (mut peak, mut crossings, mut last) = (0.0_f64, 0, 0.8_f64);
        for i in 0..(40.0 / dt) as usize {
            let now = start + std::time::Duration::from_secs_f64(i as f64 * dt);
            servo.observe(-truck.position, speed, now);
            let road = map.observe(truck.pose, 0.0, speed, Some("gps")).unwrap();
            feedback.observe_position(Some(truck.position));
            let out = feedback
                .step(&road, truck.pose, speed, speed, 0.0, dt)
                .unwrap();
            let input = servo.input(out.steer, now);
            truck.step(input, speed, axles.wheelbase, dt);
            if i as f64 * dt > 20.0 {
                peak = peak.max(road.lateral_m.abs());
            }
            if road.lateral_m.signum() != last.signum() && road.lateral_m.abs() > 0.05 {
                crossings += 1;
                last = road.lateral_m;
            }
        }
        (peak, crossings)
    }
    #[test]
    fn relative_steering_recovers_an_offset_without_oscillating() {
        for kmh in [8.0, 15.0, 30.0, 60.0] {
            let (peak, crossings) = relative_offset_recovery(kmh);
            eprintln!("relative {kmh} km/h: late peak {peak:.2} m, {crossings} crossings");
            assert!(peak < 0.1, "{kmh} km/h still {peak:.2} m off");
            assert!(
                crossings <= 2,
                "{kmh} km/h crossed the centre {crossings} times"
            );
        }
    }
    #[test]
    fn an_offset_lane_moves_the_preview_sideways_and_measures_from_it() {
        let road = roads()
            .observe(
                Pose2 {
                    x: 30.0,
                    y: 0.2,
                    yaw: 0.0,
                },
                0.0,
                5.0,
                None,
            )
            .unwrap();
        let pose = Pose2 {
            x: 30.0,
            y: 0.2,
            yaw: 0.0,
        };
        // Half a metre to the right of the lane centre.
        let shifted = road.offset(pose, -0.5);
        assert!((shifted.lateral_m - (road.lateral_m + 0.5)).abs() < 1e-9);
        assert!((shifted.preview.y - (road.preview.y - 0.5)).abs() < 0.01);
        assert!((shifted.preview.x - road.preview.x).abs() < 0.05);
        assert_eq!(road.offset(pose, 0.0).lateral_m, road.lateral_m);
    }
    /// r15 field road: 100 m straight, a 1.43 rad left bend whose curvature
    /// grows from 0.0035 to 0.0165 /m over 143 m and ends abruptly, then
    /// straight. Map coordinates are f32 near (24500, 111100): about 8 mm
    /// steps, so 1 m segment headings scatter by ±0.008 rad.
    fn field_bend() -> Vec<Point3> {
        let mut points = Vec::new();
        let (mut x, mut y, mut heading) = (0.0_f64, 0.0_f64, -1.79_f64);
        let curvature = |s: f64| match s - 100.0 {
            c if (0.0..143.0).contains(&c) => 0.0035 + 0.013 * c / 143.0,
            _ => 0.0,
        };
        for i in 0..400 {
            points.push(Point3 {
                x: (24500.0 + x) as f32 as f64,
                y: (111100.0 + y) as f32 as f64,
                z: 0.0,
            });
            heading += curvature(i as f64 + 0.5);
            x += heading.cos();
            y += heading.sin();
        }
        points
    }
    /// The game with relative steering as measured in r15 and r17: the
    /// position moves at 0.4/s per unit input (slewed by the bridge at
    /// 0.8/s) and returns toward the centre at `centering`/s per m/s of
    /// speed, both slower at speed; the wheel follows the position at once.
    struct RelativeTruck {
        position: f64,
        applied: f64,
        centering: f64,
        pose: Pose2,
    }
    impl RelativeTruck {
        fn new(pose: Pose2) -> Self {
            Self {
                position: 0.0,
                applied: 0.0,
                centering: super::super::io::RELATIVE_CENTERING,
                pose,
            }
        }
        fn step(&mut self, input: f64, speed: f64, wheelbase: f64, dt: f64) {
            self.applied = input.clamp(self.applied - 0.8 * dt, self.applied + 0.8 * dt);
            let factor = super::super::io::relative_speed_factor(speed);
            self.position = (self.position
                + factor * (0.4 * self.applied - self.centering * speed * self.position) * dt)
                .clamp(-1.0, 1.0);
            let wheel = -self.position * 0.715;
            self.pose.x += speed * self.pose.yaw.cos() * dt;
            self.pose.y += speed * self.pose.yaw.sin() * dt;
            self.pose.yaw += speed * wheel.tan() / wheelbase * dt;
        }
    }
    /// Drives `points` with relative steering at `kmh`, the game returning
    /// the wheel at `centering`; returns the peak offset and how often the
    /// steering position reversed by more than 0.01 (the wheel swaying).
    fn relative_field_drive(points: Vec<Point3>, kmh: f64, centering: f64) -> (f64, usize) {
        let map = Roads {
            feed_forward: FeedForward::relative(0.4),
            roads: HashMap::from([("gps".into(), Road::path("gps".into(), points).unwrap())]),
        };
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: 0.0,
            rear_left: 0.0,
        };
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.715, 1.0), (0.715, -1.0)],
        };
        let mut feedback = Feedback::new(axles, wide).with_steering(Some(0.0), Some(0.4));
        let mut servo =
            super::super::io::SteeringServo::new(crate::setup::SteeringMode::Relative {
                rate_per_s: 0.4,
            });
        let (start, heading) = map.roads["gps"].sample(10.0).unwrap();
        let mut truck = RelativeTruck {
            centering,
            ..RelativeTruck::new(Pose2 {
                x: start.x,
                y: start.y,
                yaw: heading,
            })
        };
        let (speed, dt) = (kmh / 3.6, 1.0 / 60.0);
        let begin = std::time::Instant::now();
        let (mut peak, mut reversals) = (0.0_f64, 0);
        let (mut extreme, mut rising) = (0.0_f64, true);
        for i in 0..(120.0 / dt) as usize {
            let now = begin + std::time::Duration::from_secs_f64(i as f64 * dt);
            servo.observe(-truck.position, speed, now);
            let Ok(road) = map.observe(truck.pose, 0.0, speed, Some("gps")) else {
                return (f64::INFINITY, reversals);
            };
            if road.remaining_m < 30.0 {
                break;
            }
            peak = peak.max(road.lateral_m.abs());
            feedback.observe_position(Some(truck.position));
            let out = feedback
                .step(&road, truck.pose, speed, speed, 0.0, dt)
                .unwrap();
            truck.step(servo.input(out.steer, now), speed, axles.wheelbase, dt);
            let p = truck.position;
            if rising && p < extreme - 0.01 || !rising && p > extreme + 0.01 {
                reversals += 1;
                rising = !rising;
                extreme = p;
            } else if rising && p > extreme || !rising && p < extreme {
                extreme = p;
            }
        }
        (peak, reversals)
    }
    /// Rows of numbers from a recorded fixture, skipping `#` comments.
    fn fixture(text: &str) -> Vec<Vec<f64>> {
        text.lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| l.split_whitespace().map(|v| v.parse().unwrap()).collect())
            .collect()
    }
    /// Replays a recorded drive with relative steering from its state at the
    /// first row: the route path as driven, the speed the planner chose
    /// (held as in the assist loop) and the game modelled as measured.
    /// The speed is capped at `max_kmh` where the planner now plans slower.
    /// Returns the peak offset.
    fn replay_relative(path: &str, drive: &str, max_kmh: f64) -> f64 {
        let points = fixture(path)
            .iter()
            .map(|p| Point3 {
                x: p[0],
                y: p[1],
                z: 0.0,
            })
            .collect();
        let log = fixture(drive);
        let map = Roads {
            feed_forward: FeedForward::relative(0.4),
            roads: HashMap::from([("gps".into(), Road::path("gps".into(), points).unwrap())]),
        };
        // The calibrated truck of r15/r16: rear axle 2.02 m behind the
        // SDK position.
        let axles = Axles {
            wheelbase: 3.76,
            rear_forward: -2.02,
            rear_left: 0.0,
        };
        let wide = SteeringMap {
            fingerprint: String::new(),
            points: vec![(-0.715, 1.0), (0.715, -1.0)],
        };
        let first = &log[0];
        let mut feedback = Feedback::new(axles, wide).with_steering(Some(first[5]), Some(0.4));
        let mut servo =
            super::super::io::SteeringServo::new(crate::setup::SteeringMode::Relative {
                rate_per_s: 0.4,
            });
        let mut truck = RelativeTruck {
            position: first[5],
            applied: first[7],
            ..RelativeTruck::new(axles.rear_pose(Pose2 {
                x: first[1],
                y: first[2],
                yaw: first[3],
            }))
        };
        let planned = |t: f64| {
            log.iter()
                .find(|r| r[0] >= t)
                .unwrap_or(&log[log.len() - 1])[4]
                .min(max_kmh / 3.6)
        };
        let dt = 1.0 / 60.0;
        let begin = std::time::Instant::now();
        let mut hold = RelativeHold::default();
        let (mut t, mut speed, mut peak) = (first[0], first[4], 0.0_f64);
        for i in 0.. {
            if t >= log[log.len() - 1][0] {
                break;
            }
            let now = begin + std::time::Duration::from_secs_f64(i as f64 * dt);
            servo.observe(-truck.position, speed, now);
            let Ok(road) = map.observe(truck.pose, 0.0, speed, Some("gps")) else {
                return f64::INFINITY;
            };
            peak = peak.max(road.lateral_m.abs());
            let next = (speed + (planned(t) - speed).clamp(-2.5 * dt, 1.5 * dt)).max(0.0);
            speed = hold
                .cap(feedback.steering_lag(), road.lateral_m, speed)
                .map_or(next, |held| next.min(held).max(speed.min(held)));
            feedback.observe_position(Some(truck.position));
            let out = feedback
                .step(&road, truck.pose, speed, speed, 0.0, dt)
                .unwrap();
            truck.step(servo.input(out.steer, now), speed, axles.wheelbase, dt);
            t += dt;
        }
        peak
    }
    /// r16 left a 90° junction turn (radius 11 m, ending abruptly) at 10–14
    /// km/h with the wheel still unwinding, sped up to 18 km/h and swayed
    /// ±1.5 m until the safety stop. Replayed: the wheel starts unwinding
    /// before the straight, the shaping stops on its target and the hold
    /// keeps the speed down.
    #[test]
    fn relative_steering_leaves_the_r16_junction_without_swaying() {
        let peak = replay_relative(
            include_str!("../../testdata/r16-junction-path.txt"),
            include_str!("../../testdata/r16-junction-drive.txt"),
            f64::INFINITY,
        );
        eprintln!("r16 junction replay peak offset {peak:.2} m");
        assert!(peak < 0.5, "r16 junction replay peak offset {peak:.2} m");
    }
    /// r15 drifted 1.3 m off the end of this bend at 31 km/h and swung 1.6 m
    /// back; the replay starts 0.34 m off as recorded.
    #[test]
    fn relative_steering_leaves_the_r15_bend_without_swaying() {
        let peak = replay_relative(
            include_str!("../../testdata/r15-bend-path.txt"),
            include_str!("../../testdata/r15-bend-drive.txt"),
            f64::INFINITY,
        );
        eprintln!("r15 bend replay peak offset {peak:.2} m");
        assert!(peak < 0.5, "r15 bend replay peak offset {peak:.2} m");
    }
    /// r17 took a 12 m left kink between a right bend and the straight, where
    /// a left-turn lane opens before a junction, at 47 km/h: the wheel, which
    /// the game turns more slowly at speed, reversed late and the truck
    /// drifted 1.6 m into the safety stop (replayed at the recorded speed it
    /// loses the lane). At the 34 km/h the planner now plans it holds.
    #[test]
    fn relative_steering_follows_the_r17_lane_split() {
        let peak = replay_relative(
            include_str!("../../testdata/r17-split-path.txt"),
            include_str!("../../testdata/r17-split-drive.txt"),
            34.0,
        );
        eprintln!("r17 split replay peak offset {peak:.2} m");
        assert!(peak < 0.5, "r17 split replay peak offset {peak:.2} m");
    }
    /// r17 sped from 23 to 32 km/h into this right bend after a junction and
    /// ran 1.3 m wide. Replayed with the wheel slower at speed, the servo that
    /// planned with the standstill rate lost the lane.
    #[test]
    fn relative_steering_follows_the_r17_bend() {
        let peak = replay_relative(
            include_str!("../../testdata/r17-bend-path.txt"),
            include_str!("../../testdata/r17-bend-drive.txt"),
            f64::INFINITY,
        );
        eprintln!("r17 bend replay peak offset {peak:.2} m");
        assert!(peak < 0.5, "r17 bend replay peak offset {peak:.2} m");
    }
    #[test]
    fn relative_hold_never_raises_its_cap() {
        let mut hold = RelativeHold::default();
        assert_eq!(hold.cap(0.0, 0.0, 5.0), None);
        assert_eq!(hold.cap(0.2, 0.0, 4.0), Some(4.0));
        // Slowing for a turn lowers the cap; speeding up does not raise it.
        assert_eq!(hold.cap(0.2, 0.0, 2.5), Some(2.5));
        assert_eq!(hold.cap(0.2, 0.0, 5.0), Some(2.5));
        // Walking pace is always allowed.
        assert_eq!(hold.cap(0.2, 0.0, 0.5), Some(RELATIVE_CREEP));
        // Released only once caught up and back near the centre.
        assert!(hold.cap(0.05, 0.1, 3.0).is_some());
        assert!(hold.cap(0.03, 0.25, 3.0).is_some());
        assert_eq!(hold.cap(0.03, 0.1, 3.0), None);
    }
    /// r15 swayed and drifted 1.3 m off this bend's abrupt end at 31 km/h,
    /// then overshot 1.6 m the other way: the servo ignored the wheel's
    /// return, the feed-forward led by a second and read f32-noisy segment
    /// headings. Robust to the return rate being 30 % off.
    #[test]
    fn relative_steering_follows_the_field_bend() {
        for centering in [0.07, 0.10, 0.13] {
            for kmh in [15.0, 30.0, 35.0] {
                let (peak, reversals) = relative_field_drive(field_bend(), kmh, centering);
                eprintln!(
                    "field bend {kmh} km/h, return {centering}: peak {peak:.2} m, {reversals} reversals"
                );
                // The planner takes the abrupt end at about 30 km/h. Held at 40
                // the wheel, which the game turns more slowly at speed (r17),
                // falls behind: 0.3–0.4 m.
                assert!(peak < 0.3, "{kmh} km/h peak {peak:.2} m");
                assert!(reversals <= 6, "{kmh} km/h swayed {reversals} times");
            }
        }
    }
    #[test]
    fn relative_steering_holds_the_lane_through_a_roundabout_entry() {
        let relative = roundabout_peak_offset(true, true);
        eprintln!("roundabout peak offset with relative steering: {relative:.2} m");
        assert!(relative < 0.4, "relative steering peak offset {relative}");
    }
    fn pedal_feedback() -> Feedback {
        Feedback::new(
            Axles {
                wheelbase: 3.76,
                rear_forward: 0.0,
                rear_left: 0.0,
            },
            calibration(),
        )
    }
    /// Longitudinal plant: brake gain from mvp-r7 (≈9 m/s² per unit) behind a
    /// 0.15 s air-brake lag, 2 m/s² per unit of throttle.
    struct Truck {
        speed: f64,
        brake_state: f64,
    }
    impl Truck {
        fn step(&mut self, out: (f64, f64), dt: f64) -> f64 {
            self.brake_state += (out.1 - self.brake_state) * (dt / 0.15);
            let mut a = 2.0 * out.0 - 9.0 * self.brake_state;
            if self.speed <= 0.0 && a < 0.0 {
                a = 0.0;
                self.speed = 0.0;
            }
            self.speed = (self.speed + a * dt).max(0.0);
            a
        }
    }
    #[test]
    fn planned_stop_is_gentle_and_restart_waits_for_settled_standstill() {
        let mut f = pedal_feedback();
        let mut truck = Truck {
            speed: 15.0 / 3.6,
            brake_state: 0.0,
        };
        let dt = 1.0 / 60.0;
        let mut peak_deceleration: f64 = 0.0;
        // The mvp-r7 phantom: IDM saturates at −4 m/s² with a zero gap.
        for _ in 0..240 {
            let out = f.pedals(truck.speed, 0.0, -4.0, dt);
            assert!(out.1 <= MAX_PLANNED_BRAKE + 1e-9 && (out.0 == 0.0 || out.1 == 0.0));
            peak_deceleration = peak_deceleration.max(-truck.step(out, dt));
        }
        assert!(peak_deceleration < 5.5, "peak {peak_deceleration}");
        assert_eq!(truck.speed, 0.0);
        assert!(matches!(f.longitudinal, Longitudinal::Holding { .. }));
        // The leader disappears: hold the brake, then release before throttle.
        let mut first_throttle = None;
        for i in 0..240 {
            // The planner caps its target at speed + 1 s of acceleration.
            let out = f.pedals(truck.speed, (truck.speed + 1.0).min(5.0), 0.5, dt);
            if out.0 > 0.0 && first_throttle.is_none() {
                first_throttle = Some(i as f64 * dt);
            }
            assert!(out.0 == 0.0 || out.1 == 0.0);
            assert!(out.0 <= 0.5 * (i as f64 * dt) + 1e-9);
            truck.step(out, dt);
        }
        assert!(
            first_throttle.is_some_and(|t| t >= RESTART_DWELL_S),
            "{first_throttle:?}"
        );
        assert!(truck.speed > 0.5);
    }
    #[test]
    fn loaded_truck_reaches_the_town_limit_uphill_without_overshoot() {
        // 40 t on a 3 % grade: full throttle gives about 1 m/s² at low speed,
        // rolling and air drag take their share, the grade 0.3 m/s².
        let mut f = pedal_feedback();
        let dt = 1.0 / 60.0;
        let target = 50.0 / 3.6 - 0.3;
        let mut speed: f64 = 0.0;
        let mut reached = None;
        let mut peak: f64 = 0.0;
        for i in 0..(150 * 60) {
            let (throttle, brake) = f.pedals(speed, target, 0.3, dt);
            let a = 1.0 * throttle - 9.0 * brake - 0.1 - 0.0006 * speed * speed - 0.3;
            speed = (speed + a * dt).max(0.0);
            if reached.is_none() && speed >= target - 0.5 {
                reached = Some(i as f64 * dt);
            }
            peak = peak.max(speed);
        }
        let reached = reached.expect("never reached the limit");
        assert!(reached < 45.0, "took {reached:.1} s");
        assert!(peak < target + 0.5, "overshoot to {:.2} km/h", peak * 3.6);
        assert!(
            (speed - target).abs() < 0.3,
            "settled at {:.2}",
            speed * 3.6
        );
    }
    #[test]
    fn brief_planner_braking_spike_is_rate_limited_and_released_smoothly() {
        let mut f = pedal_feedback();
        let dt = 1.0 / 60.0;
        for _ in 0..60 {
            f.pedals(8.0, 8.0, 0.0, dt);
        }
        let mut peak: f64 = 0.0;
        for _ in 0..6 {
            peak = peak.max(f.pedals(8.0, 8.0, -4.0, dt).1);
        }
        assert!(peak <= 6.0 * BRAKE_RISE_PER_S * dt + 1e-9);
        let mut previous = peak;
        for _ in 0..30 {
            let (throttle, brake) = f.pedals(8.0, 8.0, 0.0, dt);
            assert!(brake <= previous + 1e-12 && previous - brake <= BRAKE_FALL_PER_S * dt + 1e-9);
            assert!(throttle == 0.0 || brake == 0.0);
            previous = brake;
        }
        assert_eq!(f.longitudinal, Longitudinal::Driving);
    }
}
