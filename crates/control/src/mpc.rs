//! Condensed LTV-MPC QP, solved by Clarabel. State is lateral error,
//! heading error and wheel steer; input is wheel steer rate.
use crate::ControlError;
use clarabel::{algebra::CscMatrix, solver::*};
use etsads_msgs::VehicleGeometry;
#[derive(Clone, Copy, Debug)]
pub struct MpcState {
    pub lateral_error: f64,
    pub heading_error: f64,
    pub steer: f64,
    pub speed: f64,
}
#[derive(Clone, Debug)]
pub struct MpcSolution {
    pub steer: f64,
    pub rates: Vec<f64>,
    pub predicted_errors: Vec<[f64; 3]>,
    pub solve_seconds: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct LateralMpc {
    pub geometry: VehicleGeometry,
    pub dt: f64,
    pub steer_rate_limit: f64,
    pub lateral_acceleration_limit: f64,
}
impl Default for LateralMpc {
    fn default() -> Self {
        Self {
            geometry: VehicleGeometry::default(),
            dt: 0.1,
            steer_rate_limit: 0.6,
            lateral_acceleration_limit: 1.2,
        }
    }
}
fn csc(matrix: &[Vec<f64>]) -> CscMatrix<f64> {
    let rows = matrix.len();
    let cols = matrix[0].len();
    let mut colptr = vec![0];
    let mut rowval = Vec::new();
    let mut nzval = Vec::new();
    for col in 0..cols {
        for (row, values) in matrix.iter().enumerate() {
            if values[col].abs() > 1e-14 {
                rowval.push(row);
                nzval.push(values[col]);
            }
        }
        colptr.push(nzval.len());
    }
    CscMatrix::new(rows, cols, colptr, rowval, nzval)
}
impl LateralMpc {
    pub fn solve(self, state: MpcState, curvatures: &[f64]) -> Result<MpcSolution, ControlError> {
        self.solve_with_nominal(state, curvatures, None)
    }
    /// Reuse a shifted preceding rate trajectory as the nonlinear steering
    /// linearisation point. Clarabel itself still computes its default start;
    /// this is not a primal/dual solver warm start.
    pub fn solve_with_nominal(
        self,
        state: MpcState,
        curvatures: &[f64],
        nominal_rates: Option<&[f64]>,
    ) -> Result<MpcSolution, ControlError> {
        if !self.geometry.valid()
            || ![
                self.dt,
                self.steer_rate_limit,
                self.lateral_acceleration_limit,
                state.lateral_error,
                state.heading_error,
                state.steer,
                state.speed,
            ]
            .iter()
            .all(|v| v.is_finite())
            || !(0.02..=0.2).contains(&self.dt)
            || self.steer_rate_limit <= 0.0
            || self.steer_rate_limit > 3.0
            || self.lateral_acceleration_limit <= 0.0
            || self.lateral_acceleration_limit > 4.0
            || state.speed < 0.0
            || state.speed > 50.0
            || state.steer.abs() > self.geometry.max_steer
            || curvatures.len() < 5
            || curvatures.len() > 40
            || curvatures.iter().any(|k| !k.is_finite() || k.abs() > 0.5)
            || nominal_rates.is_some_and(|rates| {
                rates.len() != curvatures.len()
                    || rates
                        .iter()
                        .any(|v| !v.is_finite() || v.abs() > self.steer_rate_limit + 1e-8)
            })
        {
            return Err(ControlError::Invalid("MPC state or horizon"));
        }
        let start = std::time::Instant::now();
        let n = curvatures.len();
        let steer_limit = self.geometry.max_steer.min(
            (self.lateral_acceleration_limit * self.geometry.wheelbase
                / state.speed.powi(2).max(0.1))
            .atan(),
        );
        if state.steer.abs() > steer_limit + self.steer_rate_limit * self.dt {
            return Err(ControlError::Invalid(
                "MPC initial steer cannot reach lateral acceleration bound",
            ));
        }
        let mut hessian = vec![vec![0.0; n]; n];
        let mut linear = vec![0.0; n];
        let mut base = [state.lateral_error, state.heading_error, state.steer];
        let mut sensitivity = vec![[0.0; 3]; n];
        let mut bases = Vec::new();
        let mut responses = Vec::new();
        let mut nominal_steer = state.steer;
        for (step, &kappa) in curvatures.iter().enumerate() {
            let reference = (self.geometry.wheelbase * kappa).atan();
            let linearisation = if nominal_rates.is_some() {
                nominal_steer
            } else {
                reference
            };
            let gain = state.speed / self.geometry.wheelbase / linearisation.cos().powi(2);
            let affine = state.speed / self.geometry.wheelbase * linearisation.tan()
                - gain * linearisation
                - state.speed * kappa;
            let previous = base;
            base = [
                previous[0] + self.dt * state.speed * previous[1],
                previous[1] + self.dt * (gain * previous[2] + affine),
                previous[2],
            ];
            for (column, s) in sensitivity.iter_mut().enumerate() {
                let old = *s;
                *s = [
                    old[0] + self.dt * state.speed * old[1],
                    old[1] + self.dt * gain * old[2],
                    old[2] + if column == step { self.dt } else { 0.0 },
                ];
            }
            let terminal = if step == n - 1 { 3.0 } else { 1.0 };
            let weights = [5.0 * terminal, 8.0 * terminal, 0.3];
            let error = [base[0], base[1], base[2] - reference];
            for i in 0..n {
                linear[i] += 2.0
                    * (0..3)
                        .map(|j| weights[j] * sensitivity[i][j] * error[j])
                        .sum::<f64>();
                for j in i..n {
                    hessian[i][j] += 2.0
                        * (0..3)
                            .map(|k| weights[k] * sensitivity[i][k] * sensitivity[j][k])
                            .sum::<f64>();
                }
            }
            bases.push(base);
            responses.push(sensitivity.clone());
            if let Some(rates) = nominal_rates {
                nominal_steer =
                    (nominal_steer + self.dt * rates[step]).clamp(-steer_limit, steer_limit);
            }
        }
        for (i, row) in hessian.iter_mut().enumerate() {
            row[i] += 0.4;
        }
        let mut constraints = vec![vec![0.0; n]; 4 * n];
        let mut bounds = vec![0.0; 4 * n];
        for i in 0..n {
            constraints[i][i] = 1.0;
            bounds[i] = self.steer_rate_limit;
            constraints[n + i][i] = -1.0;
            bounds[n + i] = self.steer_rate_limit;
            constraints[2 * n + i][..=i].fill(self.dt);
            constraints[3 * n + i][..=i].fill(-self.dt);
            bounds[2 * n + i] = steer_limit - state.steer;
            bounds[3 * n + i] = steer_limit + state.steer;
        }
        let p = csc(&hessian);
        let a = csc(&constraints);
        let cones = [NonnegativeConeT(4 * n)];
        let settings = DefaultSettingsBuilder::default()
            .verbose(false)
            .max_iter(60)
            .tol_gap_abs(1e-7)
            .tol_feas(1e-7)
            .build()
            .map_err(|_| ControlError::Invalid("MPC solver configuration"))?;
        let mut solver = DefaultSolver::new(&p, &linear, &a, &bounds, &cones, settings)
            .map_err(|_| ControlError::Invalid("MPC QP setup"))?;
        solver.solve();
        if !matches!(
            solver.solution.status,
            SolverStatus::Solved | SolverStatus::AlmostSolved
        ) {
            return Err(ControlError::Invalid("MPC solver did not converge"));
        }
        let rates = solver.solution.x;
        for (row, bound) in constraints.iter().zip(&bounds) {
            let value = row.iter().zip(&rates).map(|(a, b)| a * b).sum::<f64>();
            if !value.is_finite() || value > *bound + 1e-5 {
                return Err(ControlError::Invalid("MPC constraint residual"));
            }
        }
        let predicted_errors = bases
            .iter()
            .zip(responses)
            .map(|(base, sensitivity)| {
                std::array::from_fn(|j| {
                    base[j]
                        + sensitivity
                            .iter()
                            .zip(&rates)
                            .map(|(s, u)| s[j] * u)
                            .sum::<f64>()
                })
            })
            .collect();
        let steer = state.steer + self.dt * rates[0];
        Ok(MpcSolution {
            steer,
            rates,
            predicted_errors,
            solve_seconds: start.elapsed().as_secs_f64(),
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn corrects_lateral_error_and_obeys_all_constraints() {
        let m = LateralMpc::default();
        let solution = m
            .solve(
                MpcState {
                    lateral_error: 0.4,
                    heading_error: 0.0,
                    steer: 0.0,
                    speed: 15.0,
                },
                &[0.0; 20],
            )
            .unwrap();
        assert!(solution.steer < 0.0);
        assert!(
            solution
                .rates
                .iter()
                .all(|r| r.abs() <= m.steer_rate_limit + 1e-5)
        );
        assert!(solution.predicted_errors.last().unwrap()[0].abs() < 0.4);
        for x in solution.predicted_errors {
            assert!(15.0_f64.powi(2) * x[2].tan().abs() / m.geometry.wheelbase <= 1.2 + 1e-5);
        }
    }
    #[test]
    fn tracks_curvature_equilibrium() {
        let m = LateralMpc::default();
        let k = 1.0 / 400.0;
        let delta = (m.geometry.wheelbase * k).atan();
        let s = m
            .solve(
                MpcState {
                    lateral_error: 0.0,
                    heading_error: 0.0,
                    steer: delta,
                    speed: 15.0,
                },
                &[k; 20],
            )
            .unwrap();
        assert!((s.steer - delta).abs() < 1e-5);
    }
}
