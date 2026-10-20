//! Explicit camera-convention diagnostics. No default convention is certified.
//! The optional `diagnostics` binary reads immutable recordings only; it does
//! not connect to game mappings, inject input, or modify bridge protocols.
use etsads_game::{camera::Convention, ets2la::Camera};
use glam::{Mat4, Quat, Vec3, Vec4};
use serde::{Deserialize, Serialize};
pub mod live_diagnostics;

#[cfg(feature = "rerun")]
pub mod media;
#[cfg(feature = "rerun")]
pub mod rerun_export;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Candidate {
    pub id: usize,
    pub transpose: bool,
    pub quaternion_world_to_camera: bool,
    pub positive_z_forward: bool,
}
impl Candidate {
    pub fn all() -> [Self; 8] {
        std::array::from_fn(|id| {
            let convention = Convention::candidates()[id];
            Self {
                id,
                transpose: convention.transpose,
                quaternion_world_to_camera: convention.quaternion_world_to_camera,
                positive_z_forward: convention.positive_z_forward,
            }
        })
    }
    pub fn convention(self) -> Convention {
        Convention {
            transpose: self.transpose,
            quaternion_world_to_camera: self.quaternion_world_to_camera,
            positive_z_forward: self.positive_z_forward,
        }
    }
}

/// Directional vanishing point; unlike finite-point projection, a vanishing
/// point behind the focal plane remains mathematically meaningful.
pub fn vanishing_point(
    camera: Camera,
    direction_scs: [f32; 3],
    width: u32,
    height: u32,
    candidate: Candidate,
) -> Option<[f64; 2]> {
    let q = camera.quaternion_wxyz;
    if !etsads_game::ets2la::valid_quaternion(q) {
        return None;
    }
    let rotation = Quat::from_xyzw(q[1], q[2], q[3], q[0]).normalize();
    let direction = Vec3::from_array(direction_scs);
    let mut local = if candidate.quaternion_world_to_camera {
        rotation * direction
    } else {
        rotation.conjugate() * direction
    };
    if candidate.positive_z_forward {
        local.z = -local.z;
    }
    let coefficients = camera.projection;
    let mut projection = Mat4::from_cols_array(&coefficients);
    if candidate.transpose {
        projection = projection.transpose();
    }
    let clip = projection * Vec4::new(local.x, local.y, local.z, 0.0);
    if !clip.is_finite() || clip.w.abs() < 1e-7 {
        return None;
    }
    Some([
        (clip.x as f64 / clip.w as f64 + 1.0) * width as f64 / 2.0,
        (1.0 - clip.y as f64 / clip.w as f64) * height as f64 / 2.0,
    ])
}

/// Error at the endpoints of an observed vertical image edge relative to the
/// ray through its midpoint and the predicted world-up vanishing point.
pub fn vertical_edge_error(edge: [[f64; 2]; 2], vanishing: [f64; 2]) -> Option<f64> {
    let midpoint = [
        (edge[0][0] + edge[1][0]) / 2.0,
        (edge[0][1] + edge[1][1]) / 2.0,
    ];
    let d = [vanishing[0] - midpoint[0], vanishing[1] - midpoint[1]];
    let length = d[0].hypot(d[1]);
    if !length.is_finite() || length < 1e-9 {
        return None;
    }
    Some(
        edge.iter()
            .map(|point| {
                ((point[0] - midpoint[0]) * d[1] - (point[1] - midpoint[1]) * d[0]).abs() / length
            })
            .sum::<f64>()
            / 2.0,
    )
}

/// Effective vertical FOV implied by the projection's scale and homogeneous w.
/// A 4:3 reference FOV is reported separately as an inference, not as a hidden
/// assertion about the game's slider semantics.
pub fn implied_fov(camera: Camera, candidate: Candidate) -> Option<(f64, f64)> {
    let coefficients = camera.projection;
    let mut projection = Mat4::from_cols_array(&coefficients);
    if candidate.transpose {
        projection = projection.transpose();
    }
    let clip = projection * Vec4::new(0.0, 1.0, -1.0, 1.0);
    if !clip.is_finite() || clip.w.abs() < 1e-9 {
        return None;
    }
    let fy = (clip.y as f64 / clip.w as f64).abs();
    Some((
        2.0 * (1.0 / fy).atan().to_degrees(),
        2.0 * ((4.0 / 3.0) / fy).atan().to_degrees(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use bytemuck::Zeroable;
    #[test]
    fn observed_line_score_distinguishes_opposite_vanishing_points() {
        let edge = [[100.0, 100.0], [110.0, 200.0]];
        assert!(vertical_edge_error(edge, [200.0, 1100.0]).unwrap() < 1e-9);
        assert!(vertical_edge_error(edge, [200.0, -900.0]).unwrap() > 5.0);
    }
    #[test]
    fn fov_respects_homogeneous_projection_scale() {
        let mut camera = Camera::zeroed();
        camera.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        camera.projection =
            Mat4::perspective_rh_gl(60_f32.to_radians(), 16.0 / 9.0, 0.1, 1000.0).to_cols_array();
        let (vertical, _) = implied_fov(camera, Candidate::all()[0]).unwrap();
        assert!((vertical - 60.0).abs() < 0.001);
        assert_eq!(Candidate::all().len(), 8);
    }

    #[test]
    fn recorded_front_interior_direction_constraints_regress() {
        let report: serde_json::Value =
            serde_json::from_str(include_str!("../evidence/interior-report.json")).unwrap();
        let frames = report["frames"].as_array().unwrap();
        assert_eq!(frames.len(), 3);
        for frame in frames {
            let mut camera = Camera::zeroed();
            camera.fov = frame["camera"]["fov"].as_f64().unwrap() as f32;
            camera.quaternion_wxyz = std::array::from_fn(|i| {
                frame["camera"]["quaternion_wxyz"][i].as_f64().unwrap() as f32
            });
            camera.projection =
                std::array::from_fn(|i| frame["camera"]["projection"][i].as_f64().unwrap() as f32);
            let held: Vec<[[f64; 2]; 2]> = frame["vertical_annotations"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|edge| edge["held_out"] == true)
                .map(|edge| serde_json::from_value(edge["pixels"].clone()).unwrap())
                .collect();
            let residual = |id| {
                let vp = vanishing_point(camera, [0.0, 1.0, 0.0], 1920, 1080, Candidate::all()[id])
                    .unwrap();
                (held
                    .iter()
                    .map(|edge| vertical_edge_error(*edge, vp).unwrap().powi(2))
                    .sum::<f64>()
                    / held.len() as f64)
                    .sqrt()
            };
            assert!(residual(1) < 1.5);
            assert!(residual(0) > 4.0);
            assert!(residual(3) > 15.0);
            // World-up alone does not disambiguate C7 from C1 within annotation uncertainty.
            assert!(residual(7) < 1.5);
            let heading =
                frame["telemetry_orientation_turns"][0].as_f64().unwrap() * std::f64::consts::TAU;
            let forward = [-20.0 * heading.sin(), 0.0, -20.0 * heading.cos()];
            assert!(
                etsads_game::camera::project(
                    camera,
                    forward,
                    1920,
                    1080,
                    Candidate::all()[1].convention()
                )
                .is_some()
            );
            assert!(
                etsads_game::camera::project(
                    camera,
                    forward,
                    1920,
                    1080,
                    Candidate::all()[7].convention()
                )
                .is_none()
            );
        }
    }
}
