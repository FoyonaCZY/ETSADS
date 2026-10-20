//! Stationary semantic-axis -> measured wheel-angle analysis. This is not a
//! velocity/curvature or longitudinal calibration and cannot authorize driving.
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
#[derive(Clone, Debug)]
pub struct StaticSteeringSample {
    pub time_s: f64,
    pub axis: f64,
    pub applied_throttle: f64,
    pub effective_throttle: f64,
    pub speed_mps: f64,
    pub paused: bool,
    pub wheel_angles_rad: Vec<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StaticSteeringPoint {
    pub axis: f64,
    pub samples: usize,
    pub mean_wheel_angles_rad: Vec<f64>,
    pub standard_deviation_rad: Vec<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StaticSteeringAnalysis {
    pub driving_authorized: bool,
    pub samples: usize,
    pub settled_samples: usize,
    pub excluded_samples: usize,
    pub invalid_samples: usize,
    pub nonzero_throttle_samples: usize,
    pub moving_samples: usize,
    pub zero_throttle_verified: bool,
    pub stationary_verified: bool,
    pub monotone_response: bool,
    pub measured_axis_polarity: Option<i8>,
    pub neutral_wheel_angle_rad: Option<f64>,
    pub points: Vec<StaticSteeringPoint>,
    pub missing_dynamic_evidence: Vec<String>,
    pub issues: Vec<String>,
}
pub fn analyze(
    samples: &[StaticSteeringSample],
    settle_seconds: f64,
) -> Result<StaticSteeringAnalysis, &'static str> {
    if !settle_seconds.is_finite() || !(0.2..=5.0).contains(&settle_seconds) {
        return Err("settle time must be 0.2..5 seconds");
    }
    let mut groups: BTreeMap<i32, Vec<&StaticSteeringSample>> = BTreeMap::new();
    let mut excluded = 0;
    let mut invalid = 0;
    let mut nonzero = 0;
    let mut moving = 0;
    let mut settled_since = 0.0;
    let mut last: Option<&StaticSteeringSample> = None;
    let mut wheel_count = None;
    for sample in samples {
        let valid = [
            sample.time_s,
            sample.axis,
            sample.applied_throttle,
            sample.effective_throttle,
            sample.speed_mps,
        ]
        .iter()
        .all(|v| v.is_finite())
            && sample.axis.abs() <= 1.0
            && (0.0..=1.0).contains(&sample.applied_throttle)
            && (0.0..=1.0).contains(&sample.effective_throttle)
            && sample.time_s >= 0.0
            && !sample.wheel_angles_rad.is_empty()
            && sample.wheel_angles_rad.len() <= 16
            && sample
                .wheel_angles_rad
                .iter()
                .all(|v| v.is_finite() && v.abs() < std::f64::consts::FRAC_PI_2)
            && wheel_count.is_none_or(|count| count == sample.wheel_angles_rad.len());
        if !valid {
            invalid += 1;
            excluded += 1;
            last = None;
            continue;
        }
        wheel_count.get_or_insert(sample.wheel_angles_rad.len());
        let throttle = sample.applied_throttle > 0.001 || sample.effective_throttle > 0.01;
        let motion = sample.speed_mps.abs() > 0.05;
        nonzero += usize::from(throttle);
        moving += usize::from(motion);
        if throttle || motion || sample.paused {
            excluded += 1;
            last = None;
            continue;
        }
        if let Some(previous) = last {
            let dt = sample.time_s - previous.time_s;
            if dt <= 0.0 {
                invalid += 1;
                excluded += 1;
                last = None;
                continue;
            }
            if dt > 0.1 || (sample.axis - previous.axis).abs() > 0.002 {
                settled_since = sample.time_s;
            }
        } else {
            settled_since = sample.time_s;
        }
        last = Some(sample);
        if sample.time_s - settled_since < settle_seconds {
            excluded += 1;
            continue;
        }
        groups
            .entry((sample.axis / 0.005).round() as i32)
            .or_default()
            .push(sample);
    }
    let mut points = Vec::new();
    let mut settled_samples = 0;
    for (axis, values) in groups {
        if values.len() < 10 {
            excluded += values.len();
            continue;
        }
        let n = values.len();
        let wheels = values[0].wheel_angles_rad.len();
        let mean: Vec<_> = (0..wheels)
            .map(|wheel| {
                values
                    .iter()
                    .map(|sample| sample.wheel_angles_rad[wheel])
                    .sum::<f64>()
                    / n as f64
            })
            .collect();
        let standard_deviation_rad = (0..wheels)
            .map(|wheel| {
                (values
                    .iter()
                    .map(|sample| (sample.wheel_angles_rad[wheel] - mean[wheel]).powi(2))
                    .sum::<f64>()
                    / n as f64)
                    .sqrt()
            })
            .collect();
        settled_samples += n;
        points.push(StaticSteeringPoint {
            axis: axis as f64 * 0.005,
            samples: n,
            mean_wheel_angles_rad: mean,
            standard_deviation_rad,
        });
    }
    let mean = |point: &StaticSteeringPoint| {
        point.mean_wheel_angles_rad.iter().sum::<f64>() / point.mean_wheel_angles_rad.len() as f64
    };
    let span = points
        .first()
        .zip(points.last())
        .map(|(a, b)| (b.axis - a.axis, mean(b) - mean(a)));
    let polarity = span
        .filter(|(axis, response)| *axis >= 0.1 && response.abs() >= 0.01)
        .map(|(_, response)| if response > 0.0 { 1 } else { -1 });
    let monotone = polarity.is_some_and(|sign| {
        points
            .windows(2)
            .all(|pair| (mean(&pair[1]) - mean(&pair[0])) * sign as f64 >= -0.001)
    });
    let neutral = points
        .iter()
        .find(|point| point.axis.abs() < 0.0025)
        .map(mean);
    let mut issues = Vec::new();
    if points.len() < 3 {
        issues.push(
            "fewer than three settled axis levels; collect positive, neutral and negative holds"
                .into(),
        );
    }
    if polarity.is_none() {
        issues.push("no identifiable bidirectional wheel-angle response".into());
    }
    if !monotone {
        issues.push("wheel response is not confirmed monotone".into());
    }
    if nonzero > 0 {
        issues.push(format!(
            "{nonzero} samples had nonzero applied/effective throttle and were excluded"
        ));
    }
    if moving > 0 {
        issues.push(format!("{moving} moving samples were excluded"));
    }
    Ok(StaticSteeringAnalysis {
        driving_authorized: false,
        samples: samples.len(),
        settled_samples,
        excluded_samples: excluded,
        invalid_samples: invalid,
        nonzero_throttle_samples: nonzero,
        moving_samples: moving,
        zero_throttle_verified: !samples.is_empty() && nonzero == 0 && invalid == 0,
        stationary_verified: !samples.is_empty() && moving == 0 && invalid == 0,
        monotone_response: monotone,
        measured_axis_polarity: polarity,
        neutral_wheel_angle_rad: neutral,
        points,
        missing_dynamic_evidence: vec![
            "speed-dependent steering curvature at 20/40/60/80 km/h".into(),
            "moving steering delay and yaw-response lag".into(),
            "independent throttle/brake/coast/grade longitudinal response".into(),
            "live closed-loop tracking acceptance".into(),
        ],
        issues,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn static_scan_recovers_negative_polarity_but_never_authorizes_driving() {
        let mut samples = Vec::new();
        for (i, axis) in [-0.2, -0.1, 0.0, 0.1, 0.2].iter().enumerate() {
            for step in 0..150 {
                samples.push(StaticSteeringSample {
                    time_s: i as f64 * 3.0 + step as f64 * 0.02,
                    axis: *axis,
                    applied_throttle: 0.0,
                    effective_throttle: 0.0,
                    speed_mps: 0.0,
                    paused: false,
                    wheel_angles_rad: vec![-axis * 0.6, -axis * 0.55],
                });
            }
        }
        let report = analyze(&samples, 0.75).unwrap();
        assert_eq!(report.measured_axis_polarity, Some(-1));
        assert!(
            report.monotone_response && report.stationary_verified && report.zero_throttle_verified
        );
        assert_eq!(report.points.len(), 5);
        assert!(!report.driving_authorized);
        assert!(!report.missing_dynamic_evidence.is_empty());
    }
    #[test]
    fn moving_or_nonzero_throttle_scan_is_not_called_static() {
        let samples = vec![StaticSteeringSample {
            time_s: 1.0,
            axis: 0.2,
            applied_throttle: 0.1,
            effective_throttle: 0.1,
            speed_mps: 2.0,
            paused: false,
            wheel_angles_rad: vec![0.1, 0.1],
        }];
        let report = analyze(&samples, 0.75).unwrap();
        assert!(!report.zero_throttle_verified && !report.stationary_verified);
        assert_eq!(report.settled_samples, 0);
        assert!(!report.driving_authorized);
    }
}
