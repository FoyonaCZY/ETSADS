//! Local LQR stabilizer for reverse articulated tracking. Trailer axle lateral
//! error is included in the quadratic cost through its exact linearized lever arms.
use crate::{ControlError, TruckState};
use etsads_msgs::{TrajPoint, VehicleGeometry, frames::wrap_angle};
pub fn steering(
    state: TruckState,
    reference: TrajPoint,
    g: VehicleGeometry,
) -> Result<f64, ControlError> {
    if !state.valid()
        || !g.valid()
        || !reference.pose.valid()
        || ![reference.curvature, reference.articulation, reference.speed]
            .iter()
            .all(|v| v.is_finite())
    {
        return Err(ControlError::Invalid("reverse LQR input"));
    }
    let v = if reference.speed < 0.0 {
        reference.speed.min(-0.3)
    } else {
        reference.speed.max(0.3)
    };
    let delta = (reference.curvature * g.wheelbase).atan();
    let phi = reference.articulation;
    let dt = 0.2;
    let coupling = if g.has_trailer {
        v * phi.cos() / g.trailer_wheelbase
            + v * g.hitch_offset * phi.sin() * delta.tan() / (g.wheelbase * g.trailer_wheelbase)
    } else {
        0.0
    };
    let a = [
        [1.0, v * dt, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, coupling * dt, 1.0 - coupling * dt],
    ];
    let b = [
        0.0,
        v * dt / (g.wheelbase * delta.cos().powi(2)),
        if g.has_trailer {
            -v * g.hitch_offset * phi.cos() * dt
                / (g.wheelbase * g.trailer_wheelbase * delta.cos().powi(2))
        } else {
            0.0
        },
    ];
    let trailer_error = if g.has_trailer {
        [1.0, -g.hitch_offset, -g.trailer_wheelbase]
    } else {
        [1.0, 0.0, 0.0]
    };
    let mut q = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            q[i][j] = trailer_error[i] * trailer_error[j];
        }
    }
    q[0][0] += 0.2;
    q[1][1] += if g.has_trailer { 0.5 } else { 6.0 };
    q[2][2] += if g.has_trailer { 8.0 } else { 0.0 };
    let r = 2.0;
    let mut p = q;
    let mut gain = [0.0; 3];
    for _ in 0..300 {
        let pb: [f64; 3] = std::array::from_fn(|i| (0..3).map(|j| p[i][j] * b[j]).sum());
        let denominator = r + (0..3).map(|i| b[i] * pb[i]).sum::<f64>();
        if !denominator.is_finite() || denominator <= 0.0 {
            return Err(ControlError::Invalid("reverse LQR Riccati equation"));
        }
        for i in 0..3 {
            gain[i] = (0..3)
                .map(|k| (0..3).map(|j| b[j] * p[j][k]).sum::<f64>() * a[k][i])
                .sum::<f64>()
                / denominator;
        }
        let mut next = q;
        for i in 0..3 {
            for j in 0..3 {
                next[i][j] += (0..3)
                    .map(|k| (0..3).map(|l| a[k][i] * p[k][l] * a[l][j]).sum::<f64>())
                    .sum::<f64>()
                    - denominator * gain[i] * gain[j];
            }
        }
        let difference = (0..3)
            .map(|i| (0..3).map(|j| (next[i][j] - p[i][j]).abs()).sum::<f64>())
            .sum::<f64>();
        p = next;
        if difference < 1e-7 {
            break;
        }
    }
    let lateral = -(state.pose.x - reference.pose.x) * reference.pose.yaw.sin()
        + (state.pose.y - reference.pose.y) * reference.pose.yaw.cos();
    let error = [
        lateral,
        wrap_angle(state.pose.yaw - reference.pose.yaw),
        wrap_angle(state.trailer_yaw - (reference.pose.yaw - reference.articulation)),
    ];
    let steer = delta - (0..3).map(|i| gain[i] * error[i]).sum::<f64>();
    if !steer.is_finite() {
        return Err(ControlError::Invalid("reverse LQR output"));
    }
    Ok(steer.clamp(-g.max_steer, g.max_steer))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reverse_lqr_stabilizes_articulation() {
        let g = VehicleGeometry::default();
        let mut state = TruckState {
            speed: -0.8,
            trailer_yaw: 0.03,
            ..Default::default()
        };
        for i in 0..1500 {
            let reference = TrajPoint {
                pose: etsads_msgs::Pose2 {
                    x: -(i as f64) * 0.8 * 0.02,
                    y: 0.0,
                    yaw: 0.0,
                },
                speed: -0.8,
                ..Default::default()
            };
            let delta = steering(state, reference, g).unwrap();
            state = state.step(g, delta, 0.0, 0.02).unwrap();
        }
        assert!(
            state.trailer_axle(g).y.abs() < 0.1,
            "{}",
            state.trailer_axle(g).y
        );
        assert!(state.articulation().abs() < 0.03);
    }
}
