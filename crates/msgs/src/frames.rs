//! Single source of truth for SCS to ENU frame conversion.
use crate::model::Point3;
pub fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
pub fn scs_to_enu(p: Point3) -> Point3 {
    Point3 {
        x: p.x,
        y: -p.z,
        z: p.y,
    }
}
pub fn enu_to_scs(p: Point3) -> Point3 {
    Point3 {
        x: p.x,
        y: p.z,
        z: -p.y,
    }
}
pub fn heading_to_yaw(heading: f64) -> f64 {
    wrap_angle(heading * std::f64::consts::TAU + std::f64::consts::FRAC_PI_2)
}
pub fn scs_local_to_body(p: Point3) -> Point3 {
    Point3 {
        x: -p.z,
        y: -p.x,
        z: p.y,
    }
}
/// Official SDK telemetry_position example convention: roll Z, pitch X,
/// heading Y, then translate by world placement. No axle-origin assumption.
pub fn rotate_scs_vector(turns: [f64; 3], v: Point3) -> Result<Point3, &'static str> {
    if !v.valid() || turns.iter().any(|a| !a.is_finite()) {
        return Err("invalid placement rotation");
    }
    let (h, p, r) = (
        turns[0] * std::f64::consts::TAU,
        turns[1] * std::f64::consts::TAU,
        turns[2] * std::f64::consts::TAU,
    );
    let x = v.x * r.cos() - v.y * r.sin();
    let y = v.x * r.sin() + v.y * r.cos();
    let z = v.z;
    let py = y * p.cos() - z * p.sin();
    let pz = y * p.sin() + z * p.cos();
    Ok(Point3 {
        x: x * h.cos() + pz * h.sin(),
        y: py,
        z: -x * h.sin() + pz * h.cos(),
    })
}
pub fn placement_offset_to_enu(
    position_scs: Point3,
    turns: [f64; 3],
    offset_scs: Point3,
) -> Result<Point3, &'static str> {
    if !position_scs.valid() {
        return Err("invalid world placement");
    }
    let offset = rotate_scs_vector(turns, offset_scs)?;
    Ok(scs_to_enu(Point3 {
        x: position_scs.x + offset.x,
        y: position_scs.y + offset.y,
        z: position_scs.z + offset.z,
    }))
}
/// Quaternion components in [w,x,y,z] order. Coordinate change M R M^T.
pub fn scs_quaternion_to_enu(q: [f64; 4]) -> Result<[f64; 4], &'static str> {
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    if !n.is_finite() || n < 1e-8 {
        return Err("invalid quaternion");
    }
    Ok([q[0] / n, q[1] / n, -q[3] / n, q[2] / n])
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cardinal_headings() {
        for (h, want) in [
            (0.0, std::f64::consts::FRAC_PI_2),
            (0.25, -std::f64::consts::PI),
            (0.5, -std::f64::consts::FRAC_PI_2),
            (0.75, 0.0),
        ] {
            assert!(wrap_angle(heading_to_yaw(h) - want).abs() < 1e-12);
        }
    }
    #[test]
    fn roundtrip_large_coordinates() {
        let p = Point3 {
            x: 150000.12345,
            y: 40.0,
            z: -79000.9999,
        };
        assert_eq!(enu_to_scs(scs_to_enu(p)), p);
    }
    #[test]
    fn local_front_is_body_forward() {
        assert_eq!(
            scs_local_to_body(Point3 {
                x: 0.0,
                y: 0.0,
                z: -1.0
            }),
            Point3 {
                x: 1.0,
                y: 0.0,
                z: 0.0
            }
        );
    }
    #[test]
    fn quaternion_axis_change() {
        assert_eq!(
            scs_quaternion_to_enu([0.0, 0.0, 1.0, 0.0]).unwrap(),
            [0.0, 0.0, -0.0, 1.0]
        );
        assert!(scs_quaternion_to_enu([0.0; 4]).is_err());
    }
    #[test]
    fn rear_axle_offset_follows_official_sdk_rotation_order() {
        let north = placement_offset_to_enu(
            Point3 {
                x: 100.0,
                y: 10.0,
                z: 200.0,
            },
            [0.0, 0.0, 0.0],
            Point3 {
                x: 0.0,
                y: 0.0,
                z: 2.0,
            },
        )
        .unwrap();
        assert_eq!(
            north,
            Point3 {
                x: 100.0,
                y: -202.0,
                z: 10.0
            }
        );
        let west = placement_offset_to_enu(
            Point3::default(),
            [0.25, 0.0, 0.0],
            Point3 {
                x: 0.0,
                y: 0.0,
                z: 2.0,
            },
        )
        .unwrap();
        assert!((west.x - 2.0).abs() < 1e-12 && west.y.abs() < 1e-12);
        let pitch = rotate_scs_vector(
            [0.0, 0.25, 0.0],
            Point3 {
                x: 0.0,
                y: 0.0,
                z: -2.0,
            },
        )
        .unwrap();
        assert!((pitch.y - 2.0).abs() < 1e-12 && pitch.z.abs() < 1e-12);
    }
}
