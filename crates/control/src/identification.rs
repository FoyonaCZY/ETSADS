//! Evidence-based identification. Missing excitation produces an unaccepted
//! calibration, never a guessed steering sign or actuator model.
use crate::{ControlError, SteeringCalibration};
use etsads_msgs::{Point3, VehicleGeometry};
use serde::{Deserialize, Serialize};

use std::collections::BTreeMap;
/// Hash only physical configuration; job identity, timestamps and sequence
/// counters cannot invalidate an otherwise identical truck geometry.
pub fn geometry_fingerprint(
    config: &etsads_msgs::protocol::ConfigBlock,
) -> Result<String, ControlError> {
    etsads_msgs::vehicle::geometry_fingerprint(config).map_err(ControlError::Invalid)
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleReference {
    pub rear_wheel_indices: Vec<u32>,
    pub sdk_to_rear_axle_scs: Point3,
    pub evidence_path: String,
    pub validated: bool,
}
/// Rear-wheel selection is explicit because lifted/steerable axles cannot be
/// inferred reliably from the current configuration wire schema.
pub fn reference_from_config(
    config: &etsads_msgs::protocol::ConfigBlock,
    indices: &[u32],
    evidence_path: &str,
) -> Result<VehicleReference, ControlError> {
    geometry_fingerprint(config)?;
    if indices.len() < 2 || indices.len() > 16 || evidence_path.is_empty() {
        return Err(ControlError::Invalid(
            "explicit rear wheel indices and reference evidence required",
        ));
    }
    let mut seen = std::collections::HashSet::new();
    let mut offset = Point3::default();
    for index in indices {
        if *index >= config.wheel_count
            || !seen.insert(*index)
            || config.wheel_steerable[*index as usize] != 0
        {
            return Err(ControlError::Invalid(
                "rear wheel index is missing, repeated or steerable",
            ));
        }
        let p = config.wheel_positions[*index as usize];
        offset.x += p[0] as f64;
        offset.y += p[1] as f64;
        offset.z += p[2] as f64;
    }
    let n = indices.len() as f64;
    offset.x /= n;
    offset.y /= n;
    offset.z /= n;
    if offset.x.abs() > 0.15 {
        return Err(ControlError::Invalid(
            "rear wheel selection is not laterally symmetric",
        ));
    }
    Ok(VehicleReference {
        rear_wheel_indices: indices.to_vec(),
        sdk_to_rear_axle_scs: offset,
        evidence_path: evidence_path.into(),
        validated: true,
    })
}
pub fn validate_reference_geometry(
    config: &etsads_msgs::protocol::ConfigBlock,
    reference: &VehicleReference,
    geometry: VehicleGeometry,
) -> Result<(), ControlError> {
    if !reference.validated || !reference.sdk_to_rear_axle_scs.valid() || !geometry.valid() {
        return Err(ControlError::Invalid("vehicle reference not validated"));
    }
    let measured = reference_from_config(
        config,
        &reference.rear_wheel_indices,
        &reference.evidence_path,
    )?;
    if measured
        .sdk_to_rear_axle_scs
        .distance_xy(reference.sdk_to_rear_axle_scs)
        > 1e-5
        || (measured.sdk_to_rear_axle_scs.z - reference.sdk_to_rear_axle_scs.z).abs() > 1e-5
    {
        return Err(ControlError::Invalid(
            "stored rear axle offset disagrees with current wheel positions",
        ));
    }
    let front: Vec<_> = (0..config.wheel_count as usize)
        .filter(|i| config.wheel_steerable[*i] != 0)
        .collect();
    if front.len() < 2 {
        return Err(ControlError::Invalid(
            "front steering axle geometry unavailable",
        ));
    }
    let front_z = front
        .iter()
        .map(|i| config.wheel_positions[*i][2] as f64)
        .sum::<f64>()
        / front.len() as f64;
    let wheelbase = measured.sdk_to_rear_axle_scs.z - front_z;
    let hitch_offset = config.hook_position[2] as f64 - measured.sdk_to_rear_axle_scs.z;
    if (geometry.wheelbase - wheelbase).abs() > 0.2
        || (geometry.hitch_offset - hitch_offset).abs() > 0.15
    {
        return Err(ControlError::Invalid(
            "wheelbase or signed hitch offset disagrees with selected axles",
        ));
    }
    Ok(())
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct IdentificationSample {
    pub time: f64,
    pub speed: f64,
    pub steering_axis: f64,
    pub throttle: f64,
    pub brake: f64,
    pub curvature: f64,
    pub acceleration: f64,
    pub grade: f64,
}
impl IdentificationSample {
    fn valid(self) -> bool {
        [
            self.time,
            self.speed,
            self.steering_axis,
            self.throttle,
            self.brake,
            self.curvature,
            self.acceleration,
            self.grade,
        ]
        .iter()
        .all(|v| v.is_finite())
            && self.time >= 0.0
            && (0.0..=60.0).contains(&self.speed)
            && self.steering_axis.abs() <= 1.0
            && (0.0..=1.0).contains(&self.throttle)
            && (0.0..=1.0).contains(&self.brake)
            && self.curvature.abs() < 0.5
            && self.acceleration.abs() < 15.0
            && self.grade.abs() < 0.5
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SteeringRow {
    pub speed: f64,
    pub samples: Vec<(f64, f64)>,
    pub counts: Vec<usize>,
    pub rms_curvature: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct LongitudinalModel {
    /// a = bias + throttle*t + brake*b + speed*v + sin(grade)*g.
    pub coefficients: [f64; 5],
    pub training_rmse: f64,
    pub validation_rmse: f64,
    pub sample_count: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CalibrationConditions {
    pub trailer_count: u8,
    pub cargo_mass_range_kg: [f64; 2],
    pub evidence_path: String,
}
impl CalibrationConditions {
    pub fn valid(&self, geometry: VehicleGeometry) -> bool {
        let [min, max] = self.cargo_mass_range_kg;
        self.trailer_count <= 1
            && geometry.has_trailer == (self.trailer_count == 1)
            && min.is_finite()
            && max.is_finite()
            && min >= 0.0
            && max >= min
            && max <= 200_000.0
            && !self.evidence_path.is_empty()
    }
    pub fn validate_observation(
        &self,
        geometry: VehicleGeometry,
        telemetry: &etsads_msgs::protocol::TelemetryFrame,
        config: &etsads_msgs::protocol::ConfigBlock,
    ) -> Result<(), ControlError> {
        if !self.valid(geometry) {
            return Err(ControlError::Invalid(
                "unverified trailer/load applicability",
            ));
        }
        let count = telemetry
            .trailers
            .iter()
            .filter(|t| t.connected != 0)
            .count();
        if count != self.trailer_count as usize {
            return Err(ControlError::Invalid(
                "attached-trailer count differs from calibrated model",
            ));
        }
        let [min, max] = self.cargo_mass_range_kg;
        if !config.cargo_mass_kg.is_finite()
            || !(min..=max).contains(&(config.cargo_mass_kg as f64))
        {
            return Err(ControlError::Invalid(
                "cargo mass is outside calibrated load range",
            ));
        }
        Ok(())
    }
}
/// Independent exterior measurement: SDK wheel/hitch positions do not locate
/// the front bumper. This evidence is optional and never inferred by sysid.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrontBumperMeasurement {
    pub front_from_rear_axle_m: f64,
    pub geometry_fingerprint: String,
    pub evidence_path: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EnvelopeMeasurement {
    pub geometry_fingerprint: String,
    pub envelope: etsads_msgs::MeasuredEnvelope,
    pub evidence_path: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleCalibration {
    pub schema_version: u32,
    pub vehicle_id: String,
    pub source_run: String,
    pub evidence: String,
    pub geometry: VehicleGeometry,
    #[serde(default)]
    pub geometry_fingerprint: Option<String>,
    #[serde(default)]
    pub steering_axis_source: String,
    #[serde(default)]
    pub reference: Option<VehicleReference>,
    #[serde(default)]
    pub conditions: Option<CalibrationConditions>,
    #[serde(default)]
    pub front_bumper: Option<FrontBumperMeasurement>,
    #[serde(default)]
    pub envelope_measurement: Option<EnvelopeMeasurement>,
    pub sample_count: usize,
    pub steering_rows: Vec<SteeringRow>,
    pub delay_s: Option<f64>,
    pub steering_tau_s: Option<f64>,
    pub delay_correlation: Option<f64>,
    pub longitudinal: Option<LongitudinalModel>,
    /// Observed longitudinal sample range intersected with identified steering
    /// rows. Legacy/static data has no road-speed applicability declaration.
    #[serde(default)]
    pub dynamic_speed_range_mps: Option<[f64; 2]>,
    pub accepted_for_control: bool,
    pub issues: Vec<String>,
}
impl VehicleCalibration {
    pub fn road_speed_range(&self) -> Result<[f64; 2], ControlError> {
        let [min, max] = self.dynamic_speed_range_mps.ok_or(ControlError::Invalid(
            "road mode requires recorded dynamic speed applicability",
        ))?;
        if !min.is_finite()
            || !max.is_finite()
            || min < 0.0
            || max <= min
            || max > 60.0
            || !self.accepted_for_control
            || self.longitudinal.is_none_or(|model| {
                model.coefficients.iter().any(|v| !v.is_finite())
                    || !model.training_rmse.is_finite()
                    || model.training_rmse < 0.0
                    || !model.validation_rmse.is_finite()
                    || model.validation_rmse < 0.0
                    || model.sample_count == 0
            })
            || self.steering_rows.iter().any(|row| !row.speed.is_finite())
            || self
                .steering_rows
                .windows(2)
                .any(|pair| pair[0].speed >= pair[1].speed)
            || self.steering_rows.first().is_none_or(|r| min < r.speed)
            || self.steering_rows.last().is_none_or(|r| max > r.speed)
        {
            return Err(ControlError::Invalid(
                "road speed applicability exceeds identified dynamics or steering rows",
            ));
        }
        Ok([min, max])
    }
    pub fn validate_measured_envelope(
        &self,
    ) -> Result<etsads_msgs::MeasuredEnvelope, ControlError> {
        let envelope = self.geometry.measured_envelope.ok_or(ControlError::Invalid(
            "missing measured tractor front/rear overhangs and attached-trailer hitch/axle body offsets"))?;
        if !self.geometry.valid() || !envelope.valid_for(self.geometry) {
            return Err(ControlError::Invalid(
                "measured body offsets disagree with total lengths or axle configuration",
            ));
        }
        let measurement = self.envelope_measurement.as_ref().ok_or(ControlError::Invalid(
            "missing exterior-envelope measurement evidence; wheel positions alone do not measure body overhangs"))?;
        if measurement.evidence_path.trim().is_empty()
            || self.geometry_fingerprint.as_ref() != Some(&measurement.geometry_fingerprint)
            || measurement.envelope != envelope
        {
            return Err(ControlError::Invalid(
                "exterior-envelope evidence does not bind these measured offsets and truck geometry",
            ));
        }
        if self.front_bumper.as_ref().is_some_and(|front| {
            !front.front_from_rear_axle_m.is_finite()
                || (front.front_from_rear_axle_m - envelope.tractor_front_from_rear_axle_m).abs()
                    > 1e-6
                || front.geometry_fingerprint != measurement.geometry_fingerprint
        }) {
            return Err(ControlError::Invalid(
                "front bumper evidence conflicts with the measured body envelope",
            ));
        }
        Ok(envelope)
    }
    pub fn measured_front_bumper(&self) -> Option<f64> {
        if self.geometry.measured_envelope.is_some() {
            return self
                .validate_measured_envelope()
                .ok()
                .map(|e| e.tractor_front_from_rear_axle_m);
        }
        let measurement = self.front_bumper.as_ref()?;
        let offset = measurement.front_from_rear_axle_m;
        (self.geometry.valid()
            && offset.is_finite()
            && offset >= self.geometry.wheelbase
            && offset <= self.geometry.tractor_length
            && !measurement.evidence_path.is_empty()
            && self.geometry_fingerprint.as_ref() == Some(&measurement.geometry_fingerprint))
        .then_some(offset)
    }
    pub fn steering_axis(&self, curvature: f64, speed: f64) -> Result<f64, ControlError> {
        if !self.accepted_for_control
            || !curvature.is_finite()
            || !speed.is_finite()
            || self.steering_rows.is_empty()
        {
            return Err(ControlError::Invalid("unaccepted vehicle calibration"));
        }
        let first = &self.steering_rows[0];
        let last = self.steering_rows.last().unwrap();
        if speed < first.speed - 1.0 || speed > last.speed + 1.0 {
            return Err(ControlError::Invalid("speed outside calibrated range"));
        }
        let inverse = |row: &SteeringRow| -> Result<f64, ControlError> {
            let (min, max) = row
                .samples
                .iter()
                .map(|(_, k)| *k)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), k| {
                    (lo.min(k), hi.max(k))
                });
            if curvature < min || curvature > max {
                return Err(ControlError::Invalid("curvature outside calibrated range"));
            }
            SteeringCalibration {
                speed: row.speed,
                samples: row.samples.clone(),
            }
            .inverse(curvature)
        };
        if speed <= first.speed {
            return inverse(first);
        }
        for rows in self.steering_rows.windows(2) {
            if speed <= rows[1].speed {
                let t = (speed - rows[0].speed) / (rows[1].speed - rows[0].speed);
                return Ok(inverse(&rows[0])? * (1.0 - t) + inverse(&rows[1])? * t);
            }
        }
        inverse(last)
    }
    pub fn pedals(
        &self,
        acceleration: f64,
        speed: f64,
        grade: f64,
    ) -> Result<(f64, f64), ControlError> {
        if !self.accepted_for_control || ![acceleration, speed, grade].iter().all(|x| x.is_finite())
        {
            return Err(ControlError::Invalid("unaccepted longitudinal calibration"));
        }
        let c = self
            .longitudinal
            .ok_or(ControlError::Invalid("missing longitudinal model"))?
            .coefficients;
        let residual = acceleration - c[0] - c[3] * speed - c[4] * grade.sin();
        if residual >= 0.0 {
            Ok(((residual / c[1]).clamp(0.0, 1.0), 0.0))
        } else {
            Ok((0.0, (residual / c[2]).clamp(0.0, 1.0)))
        }
    }
}
fn solve<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for i in 0..N {
        let pivot = (i..N).max_by(|j, k| a[*j][i].abs().total_cmp(&a[*k][i].abs()))?;
        if a[pivot][i].abs() < 1e-10 {
            return None;
        }
        a.swap(i, pivot);
        b.swap(i, pivot);
        let divisor = a[i][i];
        for value in &mut a[i][i..] {
            *value /= divisor;
        }
        b[i] /= divisor;
        for row in 0..N {
            if row == i {
                continue;
            }
            let factor = a[row][i];
            let pivot_row = a[i];
            for (value, pivot) in a[row][i..].iter_mut().zip(&pivot_row[i..]) {
                *value -= factor * pivot;
            }
            b[row] -= factor * b[i];
        }
    }
    Some(b)
}
fn fit_longitudinal(samples: &[IdentificationSample]) -> Option<LongitudinalModel> {
    let active: Vec<_> = samples.iter().filter(|s| s.speed > 1.0).collect();
    if active.len() < 100 {
        return None;
    }
    let span = |get: fn(&IdentificationSample) -> f64| {
        let (min, max) = active
            .iter()
            .map(|s| get(s))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
        max - min
    };
    if span(|s| s.throttle) < 0.2 || span(|s| s.brake) < 0.2 || span(|s| s.speed) < 2.0 {
        return None;
    }
    let mut gram = [[0.0; 5]; 5];
    let mut rhs = [0.0; 5];
    for (i, s) in active.iter().enumerate() {
        if i % 5 == 0 {
            continue;
        }
        let f = [1.0, s.throttle, s.brake, s.speed, s.grade.sin()];
        for j in 0..5 {
            rhs[j] += f[j] * s.acceleration;
            for k in 0..5 {
                gram[j][k] += f[j] * f[k];
            }
        }
    } // Grade may be constant on a flat test area; identify the four observable terms.
    let coefficients = if let Some(c) = solve(gram, rhs) {
        c
    } else {
        let a = std::array::from_fn(|i| std::array::from_fn(|j| gram[i][j]));
        let b = std::array::from_fn(|i| rhs[i]);
        let c: [f64; 4] = solve(a, b)?;
        [c[0], c[1], c[2], c[3], 0.0]
    };
    if coefficients[1] <= 0.0
        || coefficients[2] >= 0.0
        || coefficients.iter().any(|c| !c.is_finite())
    {
        return None;
    }
    let mut errors = [0.0; 2];
    let mut counts = [0; 2];
    for (i, s) in active.iter().enumerate() {
        let f = [1.0, s.throttle, s.brake, s.speed, s.grade.sin()];
        let predicted = (0..5).map(|j| f[j] * coefficients[j]).sum::<f64>();
        let split = usize::from(i % 5 == 0);
        errors[split] += (predicted - s.acceleration).powi(2);
        counts[split] += 1;
    }
    Some(LongitudinalModel {
        coefficients,
        training_rmse: (errors[0] / counts[0] as f64).sqrt(),
        validation_rmse: (errors[1] / counts[1] as f64).sqrt(),
        sample_count: active.len(),
    })
}
/// ARX delay search on uniformly resampled data. For each candidate pure delay,
/// fit y[k]=a*y[k-1]+b*u[k-d]+c and report tau=-dt/ln(a).
pub fn estimate_steering_delay(samples: &[IdentificationSample]) -> Option<(f64, f64, f64)> {
    if samples.len() < 100 {
        return None;
    }
    let mut intervals: Vec<_> = samples.windows(2).map(|p| p[1].time - p[0].time).collect();
    intervals.sort_by(f64::total_cmp);
    let dt = intervals[intervals.len() / 2];
    if !(0.005..=0.2).contains(&dt) {
        return None;
    }
    let input_span = samples
        .iter()
        .map(|s| s.steering_axis)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        });
    if input_span.1 - input_span.0 < 0.2 {
        return None;
    }
    let max_lag = (1.0 / dt).ceil() as usize;
    let mut best = None;
    for lag in 0..=max_lag {
        let mut gram = [[0.0; 3]; 3];
        let mut rhs = [0.0; 3];
        let mut count = 0;
        for i in max_lag + 1..samples.len() {
            if (samples[i].time - samples[i - 1].time - dt).abs() > dt * 0.1
                || samples[i].speed < 2.0
                || samples[i].acceleration.abs() > 0.5
            {
                continue;
            }
            let feature = [
                samples[i - 1].curvature,
                samples[i - lag].steering_axis,
                1.0,
            ];
            for j in 0..3 {
                rhs[j] += feature[j] * samples[i].curvature;
                for k in 0..3 {
                    gram[j][k] += feature[j] * feature[k];
                }
            }
            count += 1;
        }
        if count < 100 {
            continue;
        }
        let Some(coeff) = solve(gram, rhs) else {
            continue;
        };
        if coeff[0] <= 0.0 || coeff[0] >= 1.0 || coeff[1].abs() < 1e-8 {
            continue;
        }
        let mut residual = 0.0;
        let mut signal = 0.0;
        for i in max_lag + 1..samples.len() {
            let predicted = coeff[0] * samples[i - 1].curvature
                + coeff[1] * samples[i - lag].steering_axis
                + coeff[2];
            residual += (samples[i].curvature - predicted).powi(2);
            signal += samples[i].curvature.powi(2);
        }
        if best.is_none_or(|(score, _, _, _)| residual < score) {
            let quality = 1.0 - residual / signal.max(1e-12);
            best = Some((residual, lag as f64 * dt, -dt / coeff[0].ln(), quality));
        }
    }
    best.map(|(_, delay, tau, quality)| (delay, tau, quality))
}
pub fn identify(
    samples: &[IdentificationSample],
    vehicle_id: &str,
    source_run: &str,
    geometry: VehicleGeometry,
    real_recording: bool,
) -> Result<VehicleCalibration, ControlError> {
    if samples.len() < 2
        || samples.len() > 5_000_000
        || samples.iter().any(|s| !s.valid())
        || samples.windows(2).any(|p| p[1].time <= p[0].time)
        || !geometry.valid()
        || vehicle_id.is_empty()
    {
        return Err(ControlError::Invalid(
            "identification data, geometry or identity",
        ));
    }
    let mut issues = Vec::new();
    let mut groups: BTreeMap<(usize, i32), Vec<f64>> = BTreeMap::new();
    let speeds = [20.0 / 3.6, 40.0 / 3.6, 60.0 / 3.6, 80.0 / 3.6];
    let mut settled_since = samples[0].time;
    for pair in samples.windows(2) {
        let s = pair[1];
        let dt = s.time - pair[0].time;
        if dt > 0.1 || (s.steering_axis - pair[0].steering_axis).abs() / dt > 0.03 {
            settled_since = s.time;
        }
        if s.time - settled_since < 1.0 || s.acceleration.abs() > 0.25 {
            continue;
        }
        if let Some((i, _)) = speeds
            .iter()
            .enumerate()
            .filter(|(_, v)| (s.speed - **v).abs() < 0.8)
            .min_by(|a, b| (s.speed - *a.1).abs().total_cmp(&(s.speed - *b.1).abs()))
        {
            groups
                .entry((i, (s.steering_axis / 0.025).round() as i32))
                .or_default()
                .push(s.curvature);
        }
    }
    let mut rows = Vec::new();
    let mut sign = None;
    for (i, speed) in speeds.iter().enumerate() {
        let mut values = Vec::new();
        let mut counts = Vec::new();
        let mut error = 0.0;
        let mut total = 0;
        for ((index, axis), curvatures) in &groups {
            if *index != i || curvatures.len() < 20 {
                continue;
            }
            let mean = curvatures.iter().sum::<f64>() / curvatures.len() as f64;
            error += curvatures.iter().map(|v| (v - mean).powi(2)).sum::<f64>();
            total += curvatures.len();
            values.push((*axis as f64 * 0.025, mean));
            counts.push(curvatures.len());
        }
        if values.len() < 5
            || values.first().is_none_or(|v| v.0 > -0.1)
            || values.last().is_none_or(|v| v.0 < 0.1)
        {
            issues.push(format!(
                "insufficient steady steering excitation at {:.0} km/h",
                speed * 3.6
            ));
            continue;
        }
        let orientation = (values.last().unwrap().1 - values[0].1).signum();
        if orientation == 0.0
            || values
                .windows(2)
                .any(|p| (p[1].1 - p[0].1) * orientation <= 0.0)
        {
            issues.push(format!(
                "nonmonotone steering response at {:.0} km/h",
                speed * 3.6
            ));
            continue;
        }
        if sign.is_some_and(|previous| previous != orientation) {
            issues.push("steering direction differs between speed bins".into());
        }
        sign = Some(orientation);
        rows.push(SteeringRow {
            speed: *speed,
            samples: values,
            counts,
            rms_curvature: (error / total as f64).sqrt(),
        });
    }
    let delay = estimate_steering_delay(samples);
    if delay.is_none_or(|(_, tau, q)| !tau.is_finite() || tau > 3.0 || q < 0.8) {
        issues.push("steering delay/lag is unidentifiable or has poor fit".into());
    }
    let longitudinal = fit_longitudinal(samples);
    if longitudinal.is_none_or(|m| m.validation_rmse > 0.5) {
        issues.push(
            "longitudinal model needs independent throttle/brake excitation or lower residual"
                .into(),
        );
    }
    if !real_recording {
        issues.push("synthetic recording cannot authorize game control".into());
    }
    let accepted_for_control = issues.is_empty();
    let observed = samples
        .iter()
        .filter(|s| s.speed > 1.0)
        .map(|s| s.speed)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), speed| {
            (lo.min(speed), hi.max(speed))
        });
    let dynamic_speed_range_mps = rows.first().zip(rows.last()).and_then(|(first, last)| {
        let min = observed.0.max(first.speed);
        let max = observed.1.min(last.speed);
        (longitudinal.is_some() && min.is_finite() && max > min).then_some([min, max])
    });
    Ok(VehicleCalibration {
        schema_version: 1,
        vehicle_id: vehicle_id.into(),
        source_run: source_run.into(),
        evidence: if real_recording {
            "recorded game telemetry"
        } else {
            "synthetic/test data"
        }
        .into(),
        geometry,
        geometry_fingerprint: None,
        steering_axis_source: "unspecified".into(),
        reference: None,
        conditions: None,
        front_bumper: None,
        envelope_measurement: None,
        sample_count: samples.len(),
        steering_rows: rows,
        delay_s: delay.map(|d| d.0),
        steering_tau_s: delay.map(|d| d.1),
        delay_correlation: delay.map(|d| d.2),
        longitudinal,
        dynamic_speed_range_mps,
        accepted_for_control,
        issues,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn physical_fingerprint_ignores_job_changes_but_detects_wheel_changes() {
        let mut config = etsads_msgs::protocol::ConfigBlock {
            wheel_count: 4,
            ..Default::default()
        };
        config.wheel_radius[..4].fill(0.5);
        let first = geometry_fingerprint(&config).unwrap();
        config.config_seq = 100;
        config.cargo_mass_kg = 20000.0;
        assert_eq!(geometry_fingerprint(&config).unwrap(), first);
        config.wheel_positions[2][2] = 3.0;
        assert_ne!(geometry_fingerprint(&config).unwrap(), first);
    }
    #[test]
    fn decreasing_axis_mapping_is_supported() {
        let row = SteeringCalibration {
            speed: 10.0,
            samples: vec![(-1.0, 0.1), (0.0, 0.0), (1.0, -0.1)],
        };
        assert!((row.inverse(0.05).unwrap() + 0.5).abs() < 1e-12);
    }
    #[test]
    fn arx_identifies_known_delay_and_lag() {
        let dt: f64 = 0.02;
        let delay = 8;
        let tau: f64 = 0.2;
        let mut y = 0.0;
        let mut samples = Vec::new();
        for i in 0..2000 {
            let input = if (i / 83) % 2 == 0 { 0.3 } else { -0.3 };
            let delayed = if i >= delay {
                if ((i - delay) / 83) % 2 == 0 {
                    0.3
                } else {
                    -0.3
                }
            } else {
                0.0
            };
            let a = (-dt / tau).exp();
            y = a * y + (1.0 - a) * 0.1 * delayed;
            samples.push(IdentificationSample {
                time: i as f64 * dt,
                speed: 10.0,
                steering_axis: input,
                throttle: 0.0,
                brake: 0.0,
                curvature: y,
                acceleration: 0.0,
                grade: 0.0,
            });
        }
        let (d, t, q) = estimate_steering_delay(&samples).unwrap();
        assert!((d - delay as f64 * dt).abs() < dt);
        assert!((t - tau).abs() < 0.01);
        assert!(q > 0.99);
    }
    #[test]
    fn longitudinal_fit_recovers_independent_effects() {
        let samples: Vec<_> = (0..1000)
            .map(|i| {
                let throttle = (i % 7) as f64 / 7.0;
                let brake = (i % 11) as f64 / 11.0;
                let speed = 5.0 + (i % 31) as f64;
                IdentificationSample {
                    time: i as f64 * 0.02,
                    speed,
                    steering_axis: 0.0,
                    throttle,
                    brake,
                    curvature: 0.0,
                    acceleration: -0.1 + 1.5 * throttle - 4.0 * brake - 0.01 * speed,
                    grade: 0.0,
                }
            })
            .collect();
        let m = fit_longitudinal(&samples).unwrap();
        assert!((m.coefficients[1] - 1.5).abs() < 1e-8);
        assert!((m.coefficients[2] + 4.0).abs() < 1e-8);
        assert!(m.validation_rmse < 1e-8);
    }
}
