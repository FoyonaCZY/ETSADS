//! Iterative LQR refinement with nonlinear articulated dynamics, numerical
//! linearization, backward Riccati recursion and a feasibility line search.
use crate::{
    PlanningError,
    geometry::{box_in_polygon, corners, footprints, overlaps},
};
use etsads_control::TruckState;
use etsads_msgs::{OrientedBox, Pose2, VehicleGeometry, frames::wrap_angle};
type State = [f64; 6];
type Mat6 = [[f64; 6]; 6];
#[derive(Clone, Copy, Debug)]
pub struct Input {
    pub acceleration: f64,
    pub steer_rate: f64,
}
#[derive(Clone, Debug)]
pub struct RefinementRequest<'a> {
    pub initial: TruckState,
    pub reference: &'a [TruckState],
    pub initial_inputs: &'a [Input],
    pub geometry: VehicleGeometry,
    pub drivable: &'a [(f64, f64)],
    pub obstacles: &'a [OrientedBox],
    pub dt: f64,
    pub max_iterations: usize,
}
#[derive(Clone, Debug)]
pub struct RefinedPath {
    pub states: Vec<TruckState>,
    pub inputs: Vec<Input>,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub converged: bool,
}
fn encode(s: TruckState) -> State {
    [
        s.pose.x,
        s.pose.y,
        s.pose.yaw,
        s.trailer_yaw,
        s.speed,
        s.steer,
    ]
}
fn decode(s: State) -> TruckState {
    TruckState {
        pose: Pose2 {
            x: s[0],
            y: s[1],
            yaw: s[2],
        },
        trailer_yaw: s[3],
        speed: s[4],
        steer: s[5],
    }
}
fn difference(a: State, b: State) -> State {
    let mut d = std::array::from_fn(|i| a[i] - b[i]);
    d[2] = wrap_angle(d[2]);
    d[3] = wrap_angle(d[3]);
    d
}
fn dynamics(x: State, u: [f64; 2], r: &RefinementRequest<'_>) -> Option<State> {
    let midpoint = x[5] + r.dt * u[1] / 2.0;
    let final_steer = x[5] + r.dt * u[1];
    if u[0].abs() > 4.0 + 1e-8
        || u[1].abs() > 0.6 + 1e-8
        || final_steer.abs() >= r.geometry.max_steer
    {
        return None;
    }
    let mut next = decode(x).step(r.geometry, midpoint, u[0], r.dt).ok()?;
    next.steer = final_steer;
    Some(encode(next))
}
fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let t =
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / (dx * dx + dy * dy).max(1e-20)).clamp(0.0, 1.0);
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}
fn cost(x: State, reference: State, terminal: bool, r: &RefinementRequest<'_>) -> f64 {
    let state = decode(x);
    if !state.valid()
        || (r.geometry.has_trailer
            && state.articulation().abs() >= r.geometry.max_articulation - 1e-6)
        || state.steer.abs() >= r.geometry.max_steer - 1e-6
    {
        return f64::INFINITY;
    }
    let error = difference(x, reference);
    let mut weights = if terminal {
        [30.0, 30.0, 20.0, 20.0, 5.0, 0.5]
    } else {
        [1.0, 1.0, 3.0, 3.0, 0.5, 0.1]
    };
    if !r.geometry.has_trailer {
        weights[3] = 0.0;
    }
    let mut value = (0..6)
        .map(|i| weights[i] * error[i] * error[i])
        .sum::<f64>();
    let phi = state.articulation();
    if r.geometry.has_trailer {
        value -= 0.02 * (r.geometry.max_articulation.powi(2) - phi * phi).ln();
    }
    value -= 0.02 * (r.geometry.max_steer.powi(2) - state.steer * state.steer).ln();
    for b in footprints(state, r.geometry) {
        if !box_in_polygon(b, r.drivable) || r.obstacles.iter().any(|o| overlaps(b, *o, 0.2)) {
            return f64::INFINITY;
        }
        for corner in corners(b) {
            let distance = (0..r.drivable.len())
                .map(|i| {
                    segment_distance(
                        corner,
                        r.drivable[i],
                        r.drivable[(i + 1) % r.drivable.len()],
                    )
                })
                .fold(f64::INFINITY, f64::min);
            if distance <= 0.15 {
                return f64::INFINITY;
            }
            value -= 0.01 * (distance - 0.15).ln();
        }
    }
    value
}
fn cost_derivatives(
    x: State,
    reference: State,
    terminal: bool,
    r: &RefinementRequest<'_>,
) -> Option<(State, Mat6)> {
    let center = cost(x, reference, terminal, r);
    if !center.is_finite() {
        return None;
    }
    let eps = [1e-3, 1e-3, 1e-4, 1e-4, 1e-3, 1e-4];
    let mut gradient = [0.0; 6];
    let mut hessian = [[0.0; 6]; 6];
    for i in 0..6 {
        let mut plus = x;
        plus[i] += eps[i];
        let mut minus = x;
        minus[i] -= eps[i];
        let fp = cost(plus, reference, terminal, r);
        let fm = cost(minus, reference, terminal, r);
        if !fp.is_finite() || !fm.is_finite() {
            return None;
        }
        gradient[i] = (fp - fm) / (2.0 * eps[i]);
        hessian[i][i] = (fp - 2.0 * center + fm) / eps[i].powi(2);
        for j in 0..i {
            let mut pp = x;
            pp[i] += eps[i];
            pp[j] += eps[j];
            let mut pm = x;
            pm[i] += eps[i];
            pm[j] -= eps[j];
            let mut mp = x;
            mp[i] -= eps[i];
            mp[j] += eps[j];
            let mut mm = x;
            mm[i] -= eps[i];
            mm[j] -= eps[j];
            let cross = (cost(pp, reference, terminal, r)
                - cost(pm, reference, terminal, r)
                - cost(mp, reference, terminal, r)
                + cost(mm, reference, terminal, r))
                / (4.0 * eps[i] * eps[j]);
            if !cross.is_finite() {
                return None;
            }
            hessian[i][j] = cross;
            hessian[j][i] = cross;
        }
    }
    Some((gradient, hessian))
}
fn linearize(x: State, u: [f64; 2], r: &RefinementRequest<'_>) -> Option<(Mat6, [[f64; 2]; 6])> {
    let mut a = [[0.0; 6]; 6];
    let mut b = [[0.0; 2]; 6];
    for col in 0..6 {
        let eps = 1e-5;
        let mut xp = x;
        let mut xm = x;
        xp[col] += eps;
        xm[col] -= eps;
        let delta = difference(dynamics(xp, u, r)?, dynamics(xm, u, r)?);
        for row in 0..6 {
            a[row][col] = delta[row] / (2.0 * eps);
        }
    }
    for col in 0..2 {
        let eps = 1e-5;
        let mut up = u;
        let mut um = u;
        up[col] += eps;
        um[col] -= eps;
        let delta = difference(dynamics(x, up, r)?, dynamics(x, um, r)?);
        for row in 0..6 {
            b[row][col] = delta[row] / (2.0 * eps);
        }
    }
    Some((a, b))
}
fn rollout(
    initial: State,
    inputs: &[[f64; 2]],
    references: &[State],
    r: &RefinementRequest<'_>,
) -> Option<(Vec<State>, f64)> {
    let mut states = vec![initial];
    let mut total = 0.0;
    for (i, u) in inputs.iter().enumerate() {
        let state = *states.last()?;
        total += cost(state, references[i], false, r) + 0.2 * u[0] * u[0] + 0.5 * u[1] * u[1];
        let next = dynamics(state, *u, r)?;
        let a = footprints(decode(state), r.geometry);
        let b = footprints(decode(next), r.geometry);
        if r.obstacles
            .iter()
            .any(|o| (0..a.len()).any(|j| crate::geometry::swept_collision(a[j], b[j], *o, 0.2)))
        {
            return None;
        }
        states.push(next);
    }
    total += cost(*states.last()?, *references.last()?, true, r);
    if total.is_finite() {
        Some((states, total))
    } else {
        None
    }
}
pub fn refine(r: RefinementRequest<'_>) -> Result<RefinedPath, PlanningError> {
    let n = r.initial_inputs.len();
    if !r.initial.valid()
        || !r.geometry.valid()
        || !(3..=100).contains(&n)
        || r.reference.len() != n + 1
        || r.reference.iter().any(|s| !s.valid())
        || r.initial_inputs.iter().any(|u| {
            !u.acceleration.is_finite()
                || !u.steer_rate.is_finite()
                || u.acceleration.abs() > 3.99
                || u.steer_rate.abs() > 0.599
        })
        || !r.dt.is_finite()
        || !(0.02..=0.3).contains(&r.dt)
        || r.max_iterations == 0
        || r.max_iterations > 100
        || r.drivable.len() < 3
        || r.drivable
            .iter()
            .any(|(x, y)| !x.is_finite() || !y.is_finite())
        || r.obstacles.iter().any(|b| !b.valid())
    {
        return Err(PlanningError::Invalid("iLQR input"));
    }
    let references: Vec<_> = r.reference.iter().copied().map(encode).collect();
    let mut inputs: Vec<_> = r
        .initial_inputs
        .iter()
        .map(|u| [u.acceleration, u.steer_rate])
        .collect();
    let (mut states, mut objective) =
        rollout(encode(r.initial), &inputs, &references, &r).ok_or(PlanningError::NoPath)?;
    let initial_cost = objective;
    let mut regularization = 1e-3;
    let mut converged = false;
    let mut iterations = 0;
    for iteration in 0..r.max_iterations {
        iterations = iteration + 1;
        let (mut vx, mut vxx) = cost_derivatives(states[n], references[n], true, &r)
            .ok_or(PlanningError::Invalid("iLQR terminal derivatives"))?;
        let mut gains = vec![[[0.0; 6]; 2]; n];
        let mut feedforward = vec![[0.0; 2]; n];
        let mut backward_valid = true;
        for t in (0..n).rev() {
            let Some((a, b)) = linearize(states[t], inputs[t], &r) else {
                backward_valid = false;
                break;
            };
            let Some((lx, lxx)) = cost_derivatives(states[t], references[t], false, &r) else {
                backward_valid = false;
                break;
            };
            let mut qx = lx;
            let mut qu = [0.4 * inputs[t][0], inputs[t][1]];
            let mut qxx = lxx;
            let mut quu = [[0.4 + regularization, 0.0], [0.0, 1.0 + regularization]];
            let mut qux = [[0.0; 6]; 2];
            for i in 0..6 {
                qx[i] += (0..6).map(|j| a[j][i] * vx[j]).sum::<f64>();
                for j in 0..6 {
                    qxx[i][j] += (0..6)
                        .flat_map(|k| (0..6).map(move |l| (k, l)))
                        .map(|(k, l)| a[k][i] * vxx[k][l] * a[l][j])
                        .sum::<f64>();
                }
            }
            for i in 0..2 {
                qu[i] += (0..6).map(|j| b[j][i] * vx[j]).sum::<f64>();
                for j in 0..2 {
                    quu[i][j] += (0..6)
                        .flat_map(|k| (0..6).map(move |l| (k, l)))
                        .map(|(k, l)| b[k][i] * vxx[k][l] * b[l][j])
                        .sum::<f64>();
                }
                for j in 0..6 {
                    qux[i][j] = (0..6)
                        .flat_map(|k| (0..6).map(move |l| (k, l)))
                        .map(|(k, l)| b[k][i] * vxx[k][l] * a[l][j])
                        .sum::<f64>();
                }
            }
            let determinant = quu[0][0] * quu[1][1] - quu[0][1] * quu[1][0];
            if quu[0][0] <= 0.0 || determinant <= 1e-12 || !determinant.is_finite() {
                backward_valid = false;
                break;
            }
            let inverse = [
                [quu[1][1] / determinant, -quu[0][1] / determinant],
                [-quu[1][0] / determinant, quu[0][0] / determinant],
            ];
            for i in 0..2 {
                feedforward[t][i] = -(0..2).map(|j| inverse[i][j] * qu[j]).sum::<f64>();
                for j in 0..6 {
                    gains[t][i][j] = -(0..2).map(|k| inverse[i][k] * qux[k][j]).sum::<f64>();
                }
            }
            let k = feedforward[t];
            let gain = gains[t];
            for i in 0..6 {
                vx[i] = qx[i]
                    + (0..2)
                        .map(|u| {
                            gain[u][i] * qu[u]
                                + qux[u][i] * k[u]
                                + (0..2).map(|v| gain[u][i] * quu[u][v] * k[v]).sum::<f64>()
                        })
                        .sum::<f64>();
                for j in 0..6 {
                    vxx[i][j] = qxx[i][j]
                        + (0..2)
                            .map(|u| {
                                gain[u][i] * qux[u][j]
                                    + qux[u][i] * gain[u][j]
                                    + (0..2)
                                        .map(|v| gain[u][i] * quu[u][v] * gain[v][j])
                                        .sum::<f64>()
                            })
                            .sum::<f64>();
                }
            }
            let previous = vxx;
            for (i, row) in vxx.iter_mut().enumerate() {
                for (j, value) in row.iter_mut().enumerate() {
                    *value = (previous[i][j] + previous[j][i]) / 2.0;
                }
            }
        }
        if !backward_valid {
            regularization *= 10.0;
            if regularization > 1e9 {
                break;
            }
            continue;
        }
        let mut improved = None;
        for alpha in [1.0, 0.5, 0.25, 0.1, 0.05, 0.01] {
            let mut proposed = Vec::with_capacity(n);
            let mut state = states[0];
            let mut valid = true;
            for t in 0..n {
                let dx = difference(state, states[t]);
                let control: [f64; 2] = std::array::from_fn(|i| {
                    inputs[t][i]
                        + alpha * feedforward[t][i]
                        + (0..6).map(|j| gains[t][i][j] * dx[j]).sum::<f64>()
                });
                let control = [
                    control[0].clamp(-3.99, 3.99),
                    control[1].clamp(-0.599, 0.599),
                ];
                let Some(next) = dynamics(state, control, &r) else {
                    valid = false;
                    break;
                };
                proposed.push(control);
                state = next;
            }
            if valid
                && let Some((new_states, new_cost)) = rollout(states[0], &proposed, &references, &r)
                && new_cost < objective
            {
                improved = Some((proposed, new_states, new_cost));
                break;
            }
        }
        if let Some((new_inputs, new_states, new_cost)) = improved {
            let relative = (objective - new_cost) / (1.0 + objective.abs());
            inputs = new_inputs;
            states = new_states;
            objective = new_cost;
            regularization = (regularization / 3.0).max(1e-8);
            if relative < 1e-5 {
                converged = true;
                break;
            }
        } else {
            regularization *= 10.0;
            if feedforward
                .iter()
                .flat_map(|k| k.iter())
                .all(|k| k.abs() < 1e-5)
            {
                converged = true;
                break;
            }
            if regularization > 1e9 {
                break;
            }
        }
    }
    Ok(RefinedPath {
        states: states.into_iter().map(decode).collect(),
        inputs: inputs
            .into_iter()
            .map(|u| Input {
                acceleration: u[0],
                steer_rate: u[1],
            })
            .collect(),
        initial_cost,
        final_cost: objective,
        iterations,
        converged,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn refinement_reduces_tracking_cost_and_keeps_envelopes_inside() {
        let initial = TruckState {
            speed: 2.0,
            ..Default::default()
        };
        let reference: Vec<_> = (0..=30)
            .map(|i| TruckState {
                pose: Pose2 {
                    x: i as f64 * 0.2,
                    y: 0.5,
                    yaw: 0.0,
                },
                speed: 2.0,
                ..Default::default()
            })
            .collect();
        let inputs = vec![
            Input {
                acceleration: 0.0,
                steer_rate: 0.0
            };
            30
        ];
        let polygon = [(-30.0, -10.0), (30.0, -10.0), (30.0, 10.0), (-30.0, 10.0)];
        let r = refine(RefinementRequest {
            initial,
            reference: &reference,
            initial_inputs: &inputs,
            geometry: VehicleGeometry::default(),
            drivable: &polygon,
            obstacles: &[],
            dt: 0.1,
            max_iterations: 20,
        })
        .unwrap();
        assert!(
            r.final_cost < r.initial_cost * 0.9,
            "{} -> {}",
            r.initial_cost,
            r.final_cost
        );
        assert!(
            r.states
                .iter()
                .all(|s| s.articulation().abs() < std::f64::consts::PI / 3.0)
        );
    }
}
