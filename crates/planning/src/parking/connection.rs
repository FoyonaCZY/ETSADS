//! Reeds–Shepp analytic candidates replayed with the actual steering-rate and
//! articulated RK4 model. A car endpoint is never accepted as a trailer goal.
use super::{
    ParkingRequest, done,
    reeds_shepp::{self, Path, Turn},
    safe,
};
use etsads_control::TruckState;
use etsads_msgs::{Pose2, frames::wrap_angle};
pub(super) struct Connection {
    pub states: Vec<TruckState>,
    pub corrected: bool,
    pub curved: bool,
    pub cusps: usize,
    pub trailer_reference: bool,
}
pub(super) fn tractor_goal(r: &ParkingRequest<'_>) -> Pose2 {
    let offset = if r.geometry.has_trailer {
        r.geometry.hitch_offset + r.geometry.trailer_wheelbase
    } else {
        0.0
    };
    Pose2 {
        x: r.target.x + offset * r.target.yaw.cos(),
        y: r.target.y + offset * r.target.yaw.sin(),
        yaw: r.target.yaw,
    }
}
fn residual(state: TruckState, r: &ParkingRequest<'_>) -> [f64; 4] {
    let axle = state.trailer_axle(r.geometry);
    [
        axle.x - r.target.x,
        axle.y - r.target.y,
        wrap_angle(axle.yaw - r.target.yaw) * r.geometry.wheelbase.max(4.0),
        if r.geometry.has_trailer {
            state.articulation() * 2.0
        } else {
            0.0
        },
    ]
}
fn norm(v: [f64; 4]) -> f64 {
    v.iter().map(|x| x * x).sum()
}
fn rollout(
    start: TruckState,
    path: &Path,
    lengths: &[f64],
    r: &ParkingRequest<'_>,
    trailer_reference: bool,
    cancellation: Option<&std::sync::atomic::AtomicBool>,
) -> Option<Vec<TruckState>> {
    if lengths.iter().any(|v| !v.is_finite() || v.abs() > 60.0)
        || lengths.iter().map(|v| v.abs()).sum::<f64>() > 100.0
    {
        return None;
    }
    let mut state = start;
    let mut states = vec![];
    for (segment, length) in path.segments.iter().zip(lengths) {
        if length.abs() < 1e-6 {
            continue;
        }
        let curvature = segment.turn.curvature(path.radius);
        let steps = (length.abs() / 0.2).ceil() as usize;
        let dt = length.abs() / steps as f64;
        state.speed = length.signum();
        for _ in 0..steps {
            if super::cancelled(cancellation) {
                return None;
            }
            let desired = if trailer_reference {
                let u = r.geometry.trailer_wheelbase * curvature;
                let desired_phi = u.atan()
                    + (r.geometry.hitch_offset * curvature / (1.0 + u * u).sqrt())
                        .clamp(-1.0, 1.0)
                        .asin();
                if desired_phi.abs() > r.geometry.max_articulation - 0.02 {
                    return None;
                }
                let phi = state.articulation();
                let desired_rate = 1.5 * wrap_angle(desired_phi - phi);
                (r.geometry.wheelbase
                    * (desired_rate / state.speed + phi.sin() / r.geometry.trailer_wheelbase)
                    / (1.0 + r.geometry.hitch_offset * phi.cos() / r.geometry.trailer_wheelbase))
                    .atan()
                    .clamp(-r.geometry.max_steer, r.geometry.max_steer)
            } else {
                (r.geometry.wheelbase * curvature).atan()
            };
            let steer = desired.clamp(state.steer - 0.3 * dt, state.steer + 0.3 * dt);
            state = state.step(r.geometry, steer, 0.0, dt).ok()?;
            states.push(state);
        }
    }
    (!states.is_empty()).then_some(states)
}
fn solve(mut matrix: [[f64; 5]; 5], mut rhs: [f64; 5], n: usize) -> Option<[f64; 5]> {
    for i in 0..n {
        let pivot = (i..n).max_by(|&a, &b| matrix[a][i].abs().total_cmp(&matrix[b][i].abs()))?;
        if matrix[pivot][i].abs() < 1e-12 {
            return None;
        }
        matrix.swap(i, pivot);
        rhs.swap(i, pivot);
        let d = matrix[i][i];
        for value in &mut matrix[i][i..n] {
            *value /= d;
        }
        rhs[i] /= d;
        for row in 0..n {
            if row == i {
                continue;
            }
            let k = matrix[row][i];
            let pivot_row = matrix[i];
            for (value, pivot) in matrix[row][i..n].iter_mut().zip(&pivot_row[i..n]) {
                *value -= k * pivot;
            }
            rhs[row] -= k * rhs[i];
        }
    }
    rhs.iter().all(|v| v.is_finite()).then_some(rhs)
}
fn refine(
    start: TruckState,
    path: &Path,
    r: &ParkingRequest<'_>,
    trailer_reference: bool,
    iterations: usize,
    cancellation: Option<&std::sync::atomic::AtomicBool>,
) -> Option<(Vec<TruckState>, bool)> {
    let mut lengths: Vec<_> = path.segments.iter().map(|s| s.length).collect();
    let mut states = rollout(start, path, &lengths, r, trailer_reference, cancellation)?;
    for iteration in 0..=iterations {
        if super::cancelled(cancellation) {
            return None;
        }
        let final_state = *states.last()?;
        if done(final_state, r) {
            let mut previous = start;
            for state in &states {
                if super::cancelled(cancellation) {
                    return None;
                }
                if !safe(previous, *state, r) {
                    return None;
                }
                previous = *state;
            }
            return Some((states, iteration > 0));
        }
        if iteration == iterations {
            break;
        }
        let base = residual(final_state, r);
        let mut jacobian = [[0.0; 5]; 4];
        let n = lengths.len();
        for j in 0..n {
            let mut shifted = lengths.clone();
            let epsilon = 0.03 * path.segments[j].length.signum();
            shifted[j] += epsilon;
            let probe = rollout(start, path, &shifted, r, trailer_reference, cancellation)?;
            let value = residual(*probe.last()?, r);
            for k in 0..4 {
                jacobian[k][j] = (value[k] - base[k]) / epsilon;
            }
        }
        let mut matrix = [[0.0; 5]; 5];
        let mut rhs = [0.0; 5];
        for i in 0..n {
            rhs[i] = -(0..4).map(|k| jacobian[k][i] * base[k]).sum::<f64>();
            for j in 0..n {
                matrix[i][j] = (0..4).map(|k| jacobian[k][i] * jacobian[k][j]).sum::<f64>()
                    + if i == j { 0.01 } else { 0.0 };
            }
        }
        let delta = solve(matrix, rhs, n)?;
        let mut accepted = None;
        for fraction in [1.0, 0.5, 0.25, 0.125] {
            let candidate: Vec<_> = lengths
                .iter()
                .enumerate()
                .map(|(j, l)| {
                    let sign = path.segments[j].length.signum();
                    let value = *l + fraction * delta[j].clamp(-5.0, 5.0);
                    sign * (value * sign).max(0.0)
                })
                .collect();
            if let Some(trial) =
                rollout(start, path, &candidate, r, trailer_reference, cancellation)
                && norm(residual(*trial.last()?, r)) < norm(base) - 1e-8
            {
                accepted = Some((candidate, trial));
                break;
            }
        }
        let (next_lengths, next_states) = accepted?;
        lengths = next_lengths;
        states = next_states;
    }
    None
}
pub(super) fn connect(
    start: TruckState,
    r: &ParkingRequest<'_>,
    attempts: &mut usize,
    allow_correction: bool,
    cancellation: Option<&std::sync::atomic::AtomicBool>,
) -> Option<Connection> {
    let goal = tractor_goal(r);
    if start.pose.distance(goal) > 35.0 {
        return None;
    }
    let radius = r.geometry.wheelbase / r.geometry.max_steer.tan();
    for trailer_reference in [false, true] {
        if trailer_reference && !r.geometry.has_trailer {
            continue;
        }
        if trailer_reference
            && (start.trailer_axle(r.geometry).distance(r.target) > 15.0
                || wrap_angle(start.trailer_yaw - r.target.yaw).abs() > 0.8)
        {
            continue;
        }
        let (reference, goal, radius) = if trailer_reference {
            let phi = (r.geometry.max_articulation * 0.75).min(0.65);
            (
                start.trailer_axle(r.geometry),
                r.target,
                (r.geometry.trailer_wheelbase * phi.cos() + r.geometry.hitch_offset.abs())
                    / phi.sin(),
            )
        } else {
            (start.pose, goal, radius)
        };
        for factor in [1.0, 1.8, 3.0] {
            if trailer_reference && factor > 2.0 {
                continue;
            }
            let candidates = reeds_shepp::candidates(reference, goal, radius * factor).ok()?;
            for (index, path) in candidates
                .into_iter()
                .filter(|p| p.length() <= 85.0)
                .take(10)
                .enumerate()
            {
                *attempts += 1;
                if super::cancelled(cancellation) {
                    return None;
                }
                let iterations = if allow_correction && index < 2 { 8 } else { 0 };
                if let Some((states, corrected)) =
                    refine(start, &path, r, trailer_reference, iterations, cancellation)
                {
                    let curved = states.iter().any(|s| s.steer.abs() > 1e-3)
                        && path.segments.iter().any(|s| s.turn != Turn::Straight);
                    let cusps = states
                        .windows(2)
                        .filter(|p| p[0].speed.signum() != p[1].speed.signum())
                        .count();
                    return Some(Connection {
                        states,
                        corrected,
                        curved,
                        cusps,
                        trailer_reference,
                    });
                }
            }
        }
    }
    None
}
