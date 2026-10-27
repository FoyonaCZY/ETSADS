//! How closely the simulated truck follows recorded drives: every engaged
//! stretch of an assist log is replayed open loop from its recorded state
//! with the inputs the game applied, and the prediction compared after a
//! fixed horizon. The longitudinal parameters can be fitted the same way.
use super::super::core::SteeringMap;
use super::plant::{Input, Profile, Truck};
use etsads_msgs::{Pose2, frames::wrap_angle};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// One logged frame (10 Hz while engaged).
#[derive(Clone, Debug, Deserialize)]
struct Line {
    mode: Option<String>,
    source_qpc: u64,
    position_enu: [f64; 3],
    heading_rad: f64,
    applied_axes: Option<[f32; 4]>,
    effective: [f32; 4],
    speed_kmh: f64,
    rpm: f32,
}
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    pub speed: f64,
    /// Steering position (negative effective steering).
    pub position: f64,
    pub input: Input,
    pub rpm: f64,
}
const QPC_HZ: f64 = 1e7;
/// Samples further apart than this end a stretch (steady cruise is logged
/// at 2 Hz, too coarse to know the inputs in between).
const MAX_GAP_S: f64 = 0.15;
/// Engaged stretches of every log in `dir`, at least `min_s` long.
pub fn stretches(dir: &Path, min_s: f64) -> anyhow::Result<Vec<Vec<Sample>>> {
    let mut out = Vec::new();
    let mut paths: Vec<_> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "jsonl"))
        .collect();
    paths.sort();
    for path in paths {
        let text = std::fs::read_to_string(&path)?;
        let mut current: Vec<Sample> = Vec::new();
        let mut flush = |current: &mut Vec<Sample>| {
            if current.len() >= 2 && current[current.len() - 1].t - current[0].t >= min_s {
                out.push(std::mem::take(current));
            } else {
                current.clear();
            }
        };
        for line in text.lines() {
            let Ok(line) = serde_json::from_str::<Line>(line) else {
                continue;
            };
            let active = line.mode.as_deref().is_some_and(|m| {
                matches!(m, "Engaged" | "TakeoverSoon" | "SafetyBrake" | "Stopping")
            });
            let Some(axes) = line.applied_axes.filter(|_| active) else {
                flush(&mut current);
                continue;
            };
            let sample = Sample {
                t: line.source_qpc as f64 / QPC_HZ,
                x: line.position_enu[0],
                y: line.position_enu[1],
                z: line.position_enu[2],
                yaw: line.heading_rad,
                speed: line.speed_kmh / 3.6,
                position: -line.effective[0] as f64,
                input: Input {
                    steer: axes[0] as f64,
                    throttle: axes[1] as f64,
                    brake: axes[2] as f64,
                    clutch: axes[3] as f64,
                },
                rpm: line.rpm as f64,
            };
            // A paused game keeps logging the frozen frame: the truck stands
            // still at the speed it had, and the model drove on for 25 min
            // of one stretch (3 s lateral error 0.41 m RMS against 0.1).
            let frozen =
                |p: &Sample| sample.speed > 0.5 && (sample.x - p.x).hypot(sample.y - p.y) < 1e-3;
            if current.last().is_some_and(frozen) {
                flush(&mut current);
                continue;
            }
            if current
                .last()
                .is_some_and(|p| !(0.02..=MAX_GAP_S).contains(&(sample.t - p.t)))
            {
                flush(&mut current);
            }
            current.push(sample);
        }
        flush(&mut current);
    }
    Ok(out)
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct Fidelity {
    pub stretches: usize,
    pub windows: usize,
    pub horizon_s: f64,
    /// RMS over windows of the error after the horizon.
    pub steering_position_rms: f64,
    pub heading_rms_rad: f64,
    /// Sideways (across the recorded heading) and along-track position.
    pub lateral_rms_m: f64,
    pub lateral_p95_m: f64,
    pub speed_rms_mps: f64,
    pub speed_p95_mps: f64,
}
/// Rear-axle pose of a logged SDK pose.
fn rear(profile: &Profile, s: &Sample) -> Pose2 {
    Pose2 {
        x: s.x + profile.rear_forward_m * s.yaw.cos(),
        y: s.y + profile.rear_forward_m * s.yaw.sin(),
        yaw: s.yaw,
    }
}
/// Replay windows of `horizon_s` from every 1 s of each stretch. Lateral
/// windows use the recorded speed so steering is judged alone; speed
/// windows use the recorded pedals and the grade the truck drove.
pub fn evaluate(
    stretches: &[Vec<Sample>],
    profile: &Profile,
    map: &SteeringMap,
    horizon_s: f64,
) -> Fidelity {
    let dt = 1.0 / 60.0;
    let (mut position, mut heading, mut lateral, mut speed) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for stretch in stretches {
        let mut start = 0;
        while start < stretch.len() {
            let first = stretch[start];
            let Some(end) = (start..stretch.len()).find(|&i| stretch[i].t - first.t >= horizon_s)
            else {
                break;
            };
            let mut truck = Truck::new(rear(profile, &first), first.z, first.speed, first.position);
            truck.applied = first.input.steer;
            truck.rpm = first.rpm;
            truck.gear = gear_of(profile, first.speed, first.rpm);
            let mut free = truck.clone();
            let mut t = first.t;
            let mut i = start;
            while t < stretch[end].t - 1e-9 {
                while i + 1 < end && stretch[i + 1].t <= t {
                    i += 1;
                }
                let (a, b) = (stretch[i], stretch[i + 1]);
                // Speed as recorded, interpolated, for the lateral replay.
                let share = ((t - a.t) / (b.t - a.t)).clamp(0.0, 1.0);
                truck.speed = a.speed + share * (b.speed - a.speed);
                // The logged inputs are already slewed by the bridge.
                let mut lateral_profile = profile.clone();
                lateral_profile.bridge_steer_rate = 1e3;
                truck.steer(a.input.steer, &lateral_profile, map, dt);
                truck.yaw_rate = truck.speed * truck.wheel.tan() / profile.wheelbase_m;
                let mid = truck.rear.yaw + 0.5 * truck.yaw_rate * dt;
                truck.rear.x += truck.speed * mid.cos() * dt;
                truck.rear.y += truck.speed * mid.sin() * dt;
                truck.rear.yaw = wrap_angle(truck.rear.yaw + truck.yaw_rate * dt);
                let ds = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
                let grade = if ds > 0.3 { (b.z - a.z) / ds } else { 0.0 };
                free.longitudinal(a.input, grade, profile, dt);
                t += dt;
            }
            let last = stretch[end];
            let target = rear(profile, &last);
            let (dx, dy) = (truck.rear.x - target.x, truck.rear.y - target.y);
            position.push(truck.position - last.position);
            heading.push(wrap_angle(truck.rear.yaw - target.yaw));
            lateral.push(-dx * target.yaw.sin() + dy * target.yaw.cos());
            speed.push(free.speed - last.speed);
            // Next window one second on.
            start = (start..stretch.len())
                .find(|&i| stretch[i].t - first.t >= 1.0)
                .unwrap_or(stretch.len());
        }
    }
    let rms = |v: &[f64]| (v.iter().map(|e| e * e).sum::<f64>() / v.len().max(1) as f64).sqrt();
    let p95 = |v: &[f64]| {
        let mut a: Vec<f64> = v.iter().map(|e| e.abs()).collect();
        a.sort_by(f64::total_cmp);
        a.get((a.len() as f64 * 0.95) as usize)
            .copied()
            .unwrap_or(0.0)
    };
    Fidelity {
        stretches: stretches.len(),
        windows: position.len(),
        horizon_s,
        steering_position_rms: rms(&position),
        heading_rms_rad: rms(&heading),
        lateral_rms_m: rms(&lateral),
        lateral_p95_m: p95(&lateral),
        speed_rms_mps: rms(&speed),
        speed_p95_mps: p95(&speed),
    }
}
/// The gear a logged engine speed and road speed match.
fn gear_of(profile: &Profile, speed: f64, rpm: f64) -> usize {
    if speed < 1.0 {
        return 0;
    }
    let ratio = rpm / speed;
    (0..profile.gear_rpm_per_mps.len())
        .min_by(|&a, &b| {
            (profile.gear_rpm_per_mps[a] / ratio)
                .ln()
                .abs()
                .total_cmp(&(profile.gear_rpm_per_mps[b] / ratio).ln().abs())
        })
        .unwrap_or(0)
}
/// The longitudinal parameters `fit` adjusts, as a vector.
fn get(p: &Profile) -> Vec<f64> {
    vec![
        p.traction,
        p.traction_exponent,
        p.throttle_lag_s,
        p.brake_lag_s,
        p.brake_per_unit,
        p.rolling,
        p.engine_brake,
        p.upshift_s,
        p.upshift_rpm,
        p.upshift_light_rpm,
        p.downshift_rpm,
        p.throttle_curve,
        p.idle_torque,
        p.downshift_full_rpm,
        p.downshift_s,
        p.brake_exponent,
    ]
}
fn set(p: &Profile, v: &[f64]) -> Profile {
    Profile {
        traction: v[0].clamp(0.1, 20.0),
        traction_exponent: v[1].clamp(0.0, 2.0),
        throttle_lag_s: v[2].clamp(0.02, 1.0),
        brake_lag_s: v[3].clamp(0.02, 1.0),
        brake_per_unit: v[4].clamp(2.0, 20.0),
        rolling: v[5].clamp(0.0, 1.0),
        engine_brake: v[6].clamp(0.0, 2.0),
        upshift_s: v[7].clamp(0.0, 2.0),
        upshift_rpm: v[8].clamp(1100.0, 2000.0),
        upshift_light_rpm: v[9].clamp(900.0, 1800.0),
        downshift_rpm: v[10].clamp(600.0, 1100.0),
        throttle_curve: v[11].clamp(0.5, 3.0),
        idle_torque: v[12].clamp(0.1, 1.0),
        downshift_full_rpm: v[13].clamp(700.0, 1300.0),
        downshift_s: v[14].clamp(0.0, 1.5),
        brake_exponent: v[15].clamp(0.3, 1.5),
        ..p.clone()
    }
}
/// Nelder–Mead on the speed error over `horizon_s` windows.
pub fn fit_longitudinal(
    stretches: &[Vec<Sample>],
    start: &Profile,
    map: &SteeringMap,
    horizon_s: f64,
    iterations: usize,
) -> Profile {
    let cost = |v: &[f64]| evaluate(stretches, &set(start, v), map, horizon_s).speed_rms_mps;
    let x0 = get(&set(start, &get(start)));
    let n = x0.len();
    let mut simplex: Vec<(Vec<f64>, f64)> = (0..=n)
        .map(|i| {
            let mut x = x0.clone();
            if i > 0 {
                x[i - 1] *= 1.2;
            }
            let c = cost(&x);
            (x, c)
        })
        .collect();
    for _ in 0..iterations {
        simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
        let centroid: Vec<f64> = (0..n)
            .map(|j| simplex[..n].iter().map(|s| s.0[j]).sum::<f64>() / n as f64)
            .collect();
        let toward = |k: f64| -> Vec<f64> {
            (0..n)
                .map(|j| centroid[j] + k * (simplex[n].0[j] - centroid[j]))
                .collect()
        };
        let reflected = toward(-1.0);
        let r = cost(&reflected);
        if r < simplex[0].1 {
            let expanded = toward(-2.0);
            let e = cost(&expanded);
            simplex[n] = if e < r { (expanded, e) } else { (reflected, r) };
        } else if r < simplex[n - 1].1 {
            simplex[n] = (reflected, r);
        } else {
            let contracted = toward(0.5);
            let c = cost(&contracted);
            if c < simplex[n].1 {
                simplex[n] = (contracted, c);
            } else {
                let best = simplex[0].0.clone();
                for s in simplex.iter_mut().skip(1) {
                    s.0 = (0..n).map(|j| best[j] + 0.5 * (s.0[j] - best[j])).collect();
                    s.1 = cost(&s.0);
                }
            }
        }
    }
    simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
    set(start, &simplex[0].0)
}
/// Speed replay of one stretch, restarted from the record every
/// `horizon_s`: time, recorded and predicted speed and engine speed, the
/// pedals and the predicted gear (diagnostics).
pub fn speed_trace(stretch: &[Sample], profile: &Profile, horizon_s: f64) -> Vec<[f64; 8]> {
    let dt = 1.0 / 60.0;
    let mut out = Vec::new();
    let mut truck = None::<(Truck, f64)>;
    let mut t = stretch[0].t;
    let mut i = 0;
    while i + 1 < stretch.len() {
        while i + 1 < stretch.len() && stretch[i + 1].t <= t {
            i += 1;
            let s = stretch[i];
            if truck
                .as_ref()
                .is_none_or(|(_, since)| s.t - since >= horizon_s)
            {
                let mut fresh = Truck::new(rear(profile, &s), s.z, s.speed, s.position);
                fresh.rpm = s.rpm;
                fresh.gear = gear_of(profile, s.speed, s.rpm);
                truck = Some((fresh, s.t));
            }
            let (truck, _) = truck.as_ref().expect("set above");
            out.push([
                s.t - stretch[0].t,
                s.speed,
                truck.speed,
                s.input.throttle,
                s.input.brake,
                s.rpm,
                truck.rpm,
                truck.gear as f64,
            ]);
        }
        if i + 1 >= stretch.len() {
            break;
        }
        let (a, b) = (stretch[i], stretch[i + 1]);
        let ds = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
        let grade = if ds > 0.3 { (b.z - a.z) / ds } else { 0.0 };
        if let Some((truck, _)) = truck.as_mut() {
            truck.longitudinal(a.input, grade, profile, dt);
        }
        t += dt;
    }
    out
}
