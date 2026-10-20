use etsads_control::TruckState;
use etsads_msgs::{OrientedBox, Pose2, VehicleGeometry, frames::wrap_angle};
pub fn corners(b: OrientedBox) -> [(f64, f64); 4] {
    let (c, s) = (b.pose.yaw.cos(), b.pose.yaw.sin());
    [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)].map(|(u, v)| {
        (
            b.pose.x + c * u * b.length / 2.0 - s * v * b.width / 2.0,
            b.pose.y + s * u * b.length / 2.0 + c * v * b.width / 2.0,
        )
    })
}
/// Separating-axis theorem; touching counts as collision. Invalid boxes collide.
pub fn overlaps(a: OrientedBox, b: OrientedBox, margin: f64) -> bool {
    if !a.valid() || !b.valid() || !margin.is_finite() || margin < 0.0 {
        return true;
    }
    let a = OrientedBox {
        length: a.length + 2.0 * margin,
        width: a.width + 2.0 * margin,
        ..a
    };
    let ac = corners(a);
    let bc = corners(b);
    for angle in [
        a.pose.yaw,
        a.pose.yaw + std::f64::consts::FRAC_PI_2,
        b.pose.yaw,
        b.pose.yaw + std::f64::consts::FRAC_PI_2,
    ] {
        let (c, s) = (angle.cos(), angle.sin());
        let range = |points: [(f64, f64); 4]| {
            points
                .into_iter()
                .map(|(x, y)| x * c + y * s)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                })
        };
        let (amin, amax) = range(ac);
        let (bmin, bmax) = range(bc);
        if amax < bmin || bmax < amin {
            return false;
        }
    }
    true
}
pub fn footprints(state: TruckState, g: VehicleGeometry) -> Vec<OrientedBox> {
    let envelope = g.resolved_envelope();
    let hitch = Pose2 {
        x: state.pose.x - g.hitch_offset * state.pose.yaw.cos(),
        y: state.pose.y - g.hitch_offset * state.pose.yaw.sin(),
        yaw: state.trailer_yaw,
    };
    let tractor_offset = envelope.tractor_center_from_rear_axle_m();
    let mut boxes = vec![OrientedBox {
        pose: Pose2 {
            x: state.pose.x + tractor_offset * state.pose.yaw.cos(),
            y: state.pose.y + tractor_offset * state.pose.yaw.sin(),
            yaw: state.pose.yaw,
        },
        length: g.tractor_length,
        width: g.width,
    }];
    if let Some(trailer) = envelope.trailer {
        let offset = trailer.center_from_hitch_m(g.trailer_wheelbase);
        boxes.push(OrientedBox {
            pose: Pose2 {
                x: hitch.x + offset * hitch.yaw.cos(),
                y: hitch.y + offset * hitch.yaw.sin(),
                yaw: hitch.yaw,
            },
            length: g.trailer_length,
            width: g.width,
        });
    }
    boxes
}
/// Conservative swept OBB. Each midpoint box is expanded by a bound on the
/// motion of every corner, so obstacles cannot tunnel between samples.
pub fn swept_collision(
    from: OrientedBox,
    to: OrientedBox,
    obstacle: OrientedBox,
    margin: f64,
) -> bool {
    if !from.valid() || !to.valid() || !obstacle.valid() {
        return true;
    }
    let travel = from.pose.distance(to.pose);
    let angle = wrap_angle(to.pose.yaw - from.pose.yaw);
    let radius = from.length.hypot(from.width).max(to.length.hypot(to.width)) / 2.0;
    let divisions = ((travel + angle.abs() * radius) / 0.25)
        .ceil()
        .clamp(1.0, 10000.0) as usize;
    for i in 0..divisions {
        let t = (i as f64 + 0.5) / divisions as f64;
        let bound = (travel + angle.abs() * radius) / (2.0 * divisions as f64);
        let b = OrientedBox {
            pose: Pose2 {
                x: from.pose.x + t * (to.pose.x - from.pose.x),
                y: from.pose.y + t * (to.pose.y - from.pose.y),
                yaw: from.pose.yaw + t * angle,
            },
            length: from.length.max(to.length),
            width: from.width.max(to.width),
        };
        if overlaps(b, obstacle, margin + bound) {
            return true;
        }
    }
    false
}
/// Conservative relative sweep for two moving, rotating rectangles. Endpoint
/// checks alone can miss a fast vehicle crossing between planner samples.
pub fn moving_swept_collision(
    a: OrientedBox,
    next_a: OrientedBox,
    b: OrientedBox,
    next_b: OrientedBox,
    margin: f64,
) -> bool {
    if ![a, next_a, b, next_b].iter().all(|body| body.valid())
        || !margin.is_finite()
        || margin < 0.0
    {
        return true;
    }
    let travel = |from: OrientedBox, to: OrientedBox| {
        from.pose.distance(to.pose)
            + from.length.hypot(from.width) / 2.0 * wrap_angle(to.pose.yaw - from.pose.yaw).abs()
    };
    let total = travel(a, next_a) + travel(b, next_b);
    let n = (total / 0.2).ceil().clamp(1.0, 10000.0) as usize;
    let interpolate = |from: OrientedBox, to: OrientedBox, t: f64| OrientedBox {
        pose: Pose2 {
            x: from.pose.x + t * (to.pose.x - from.pose.x),
            y: from.pose.y + t * (to.pose.y - from.pose.y),
            yaw: from.pose.yaw + t * wrap_angle(to.pose.yaw - from.pose.yaw),
        },
        length: from.length.max(to.length),
        width: from.width.max(to.width),
    };
    for i in 0..n {
        let t = (i as f64 + 0.5) / n as f64;
        if overlaps(
            interpolate(a, next_a, t),
            interpolate(b, next_b, t),
            margin + total / (2.0 * n as f64),
        ) {
            return true;
        }
    }
    false
}
pub fn point_in_polygon(p: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3
        || ![p.0, p.1].iter().all(|x| x.is_finite())
        || polygon
            .iter()
            .any(|(x, y)| !x.is_finite() || !y.is_finite())
    {
        return false;
    }
    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
    }
    inside
}
pub fn box_in_polygon(b: OrientedBox, polygon: &[(f64, f64)]) -> bool {
    if !b.valid() {
        return false;
    }
    let cs = corners(b);
    if !cs.iter().all(|p| point_in_polygon(*p, polygon)) {
        return false;
    }
    // Exact edge intersection catches even arbitrarily narrow concave notches;
    // fixed-distance edge sampling cannot provide this guarantee.
    for i in 0..4 {
        let a = cs[i];
        let end = cs[(i + 1) % 4];
        for j in 0..polygon.len() {
            if segments_intersect(a, end, polygon[j], polygon[(j + 1) % polygon.len()]) {
                return false;
            }
        }
    }
    true
}
fn segments_intersect(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let cross = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
        (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
    };
    let ab_c = cross(a, b, c);
    let ab_d = cross(a, b, d);
    let cd_a = cross(c, d, a);
    let cd_b = cross(c, d, b);
    if ab_c * ab_d < 0.0 && cd_a * cd_b < 0.0 {
        return true;
    }
    let on = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
        r.0 >= p.0.min(q.0) - 1e-10
            && r.0 <= p.0.max(q.0) + 1e-10
            && r.1 >= p.1.min(q.1) - 1e-10
            && r.1 <= p.1.max(q.1) + 1e-10
    };
    (ab_c.abs() < 1e-10 && on(a, b, c))
        || (ab_d.abs() < 1e-10 && on(a, b, d))
        || (cd_a.abs() < 1e-10 && on(c, d, a))
        || (cd_b.abs() < 1e-10 && on(c, d, b))
}
/// Conservative containment of the rigid-body sweep between two poses.
pub fn swept_in_polygon(from: OrientedBox, to: OrientedBox, polygon: &[(f64, f64)]) -> bool {
    if !from.valid() || !to.valid() {
        return false;
    }
    let yaw = wrap_angle(to.pose.yaw - from.pose.yaw);
    let travel = from.pose.distance(to.pose);
    let radius = from.length.hypot(from.width).max(to.length.hypot(to.width)) / 2.0;
    let subdivisions = ((travel + yaw.abs() * radius) / 0.2)
        .ceil()
        .clamp(1.0, 10000.0) as usize;
    for i in 0..subdivisions {
        let t = (i as f64 + 0.5) / subdivisions as f64;
        let bound = (travel + yaw.abs() * radius) / (2.0 * subdivisions as f64);
        let b = OrientedBox {
            pose: Pose2 {
                x: from.pose.x + t * (to.pose.x - from.pose.x),
                y: from.pose.y + t * (to.pose.y - from.pose.y),
                yaw: from.pose.yaw + t * yaw,
            },
            length: from.length.max(to.length) + 2.0 * bound,
            width: from.width.max(to.width) + 2.0 * bound,
        };
        if !box_in_polygon(b, polygon) {
            return false;
        }
    }
    true
}
#[cfg(test)]
mod tests {
    use super::*;
    fn b(x: f64, y: f64, yaw: f64) -> OrientedBox {
        OrientedBox {
            pose: Pose2 { x, y, yaw },
            length: 4.0,
            width: 2.0,
        }
    }
    #[test]
    fn sat_rotation_and_touch() {
        assert!(overlaps(b(0.0, 0.0, 0.0), b(4.0, 0.0, 0.0), 0.0));
        assert!(!overlaps(b(0.0, 0.0, 0.0), b(4.1, 0.0, 0.0), 0.0));
        assert!(overlaps(b(0.0, 0.0, 0.8), b(1.0, 1.0, -0.6), 0.0));
    }
    #[test]
    fn sweep_catches_tunneling() {
        let wall = OrientedBox {
            length: 0.1,
            width: 10.0,
            ..b(5.0, 0.0, 0.0)
        };
        assert!(!overlaps(b(0.0, 0.0, 0.0), wall, 0.0));
        assert!(!overlaps(b(10.0, 0.0, 0.0), wall, 0.0));
        assert!(swept_collision(
            b(0.0, 0.0, 0.0),
            b(10.0, 0.0, 0.0),
            wall,
            0.0
        ));
    }
    #[test]
    fn narrow_concave_notch_is_not_missed_by_containment() {
        let polygon = [
            (-5.0, -5.0),
            (5.0, -5.0),
            (5.0, 5.0),
            (0.04, 5.0),
            (0.04, -0.5),
            (0.03, -0.5),
            (0.03, 5.0),
            (-5.0, 5.0),
        ];
        assert!(!box_in_polygon(b(0.0, 0.0, 0.0), &polygon));
    }
    #[test]
    fn tractor_only_envelope_has_no_phantom_trailer() {
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_length: 0.0,
            trailer_wheelbase: 0.0,
            ..Default::default()
        };
        let boxes = footprints(TruckState::default(), geometry);
        assert_eq!(boxes.len(), 1);
        let polygon = [(-3.0, -2.0), (6.0, -2.0), (6.0, 2.0), (-3.0, 2.0)];
        assert!(box_in_polygon(boxes[0], &polygon));
        assert!(
            !footprints(TruckState::default(), VehicleGeometry::default())
                .iter()
                .all(|b| box_in_polygon(*b, &polygon))
        );
    }
    fn measured_geometry(front: f64, trailer_front: f64, has_trailer: bool) -> VehicleGeometry {
        VehicleGeometry {
            has_trailer,
            hitch_offset: -0.5,
            trailer_length: if has_trailer { 13.6 } else { 0.0 },
            trailer_wheelbase: if has_trailer { 8.0 } else { 0.0 },
            measured_envelope: Some(etsads_msgs::MeasuredEnvelope {
                tractor_front_from_rear_axle_m: front,
                tractor_rear_overhang_m: 6.2 - front,
                trailer: has_trailer.then_some(etsads_msgs::TrailerEnvelope {
                    front_from_hitch_m: trailer_front,
                    rear_from_axle_m: 13.6 - 8.0 - trailer_front,
                }),
            }),
            ..Default::default()
        }
    }
    #[test]
    fn equal_total_lengths_with_different_overhangs_change_collision_and_boundary_results() {
        let a = measured_geometry(4.5, 1.0, false);
        let b = measured_geometry(5.3, 1.0, false);
        assert!(a.valid() && b.valid());
        assert_eq!(a.tractor_length, b.tractor_length);
        let state = TruckState::default();
        let box_a = footprints(state, a)[0];
        let box_b = footprints(state, b)[0];
        let post = OrientedBox {
            pose: Pose2 {
                x: 5.1,
                y: 1.0,
                yaw: 0.0,
            },
            length: 0.1,
            width: 0.1,
        };
        assert!(!overlaps(box_a, post, 0.0));
        assert!(overlaps(box_b, post, 0.0));
        let area = [(-5.0, -3.0), (5.0, -3.0), (5.0, 3.0), (-5.0, 3.0)];
        assert!(box_in_polygon(box_a, &area));
        assert!(!box_in_polygon(box_b, &area));
        for (geometry, allowed) in [(a, true), (b, false)] {
            let request = crate::parking::ParkingRequest {
                start: state,
                target: Pose2 {
                    x: -1.0,
                    y: 0.0,
                    yaw: 0.0,
                },
                geometry,
                obstacles: &[],
                drivable: &area,
                max_expansions: 1000,
                position_tolerance: 0.25,
                angle_tolerance: 0.03,
            };
            assert_eq!(crate::parking::plan(request).is_ok(), allowed);
            let reference = [state; 4];
            let inputs = [crate::ilqr::Input {
                acceleration: 0.0,
                steer_rate: 0.0,
            }; 3];
            let request = crate::ilqr::RefinementRequest {
                initial: state,
                reference: &reference,
                initial_inputs: &inputs,
                geometry,
                drivable: &area,
                obstacles: &[],
                dt: 0.1,
                max_iterations: 1,
            };
            assert_eq!(crate::ilqr::refine(request).is_ok(), allowed);
        }
    }
    #[test]
    fn articulated_measured_front_corner_is_swept_about_the_signed_hitch() {
        let a = measured_geometry(4.5, 0.5, true);
        let b = measured_geometry(4.5, 2.0, true);
        assert!(a.valid() && b.valid());
        assert_eq!(a.trailer_length, b.trailer_length);
        let from = TruckState {
            trailer_yaw: 35_f64.to_radians(),
            ..Default::default()
        };
        let to = TruckState {
            trailer_yaw: 55_f64.to_radians(),
            ..Default::default()
        };
        let yaw = 45_f64.to_radians();
        let obstacle = OrientedBox {
            pose: Pose2 {
                x: 0.5 + 1.8 * yaw.cos() - yaw.sin(),
                y: 1.8 * yaw.sin() + yaw.cos(),
                yaw: 0.0,
            },
            length: 0.1,
            width: 0.1,
        };
        assert!(!swept_collision(
            footprints(from, a)[1],
            footprints(to, a)[1],
            obstacle,
            0.0
        ));
        assert!(swept_collision(
            footprints(from, b)[1],
            footprints(to, b)[1],
            obstacle,
            0.0
        ));
        let middle = TruckState {
            trailer_yaw: yaw,
            ..Default::default()
        };
        let shape = footprints(middle, b)[1];
        let axle = middle.trailer_axle(b);
        let rear = b
            .measured_envelope
            .unwrap()
            .trailer
            .unwrap()
            .rear_from_axle_m;
        let body_rear = Pose2 {
            x: shape.pose.x - shape.length / 2.0 * yaw.cos(),
            y: shape.pose.y - shape.length / 2.0 * yaw.sin(),
            yaw,
        };
        assert!(
            body_rear.distance(Pose2 {
                x: axle.x - rear * yaw.cos(),
                y: axle.y - rear * yaw.sin(),
                yaw
            }) < 1e-10
        );
        let moving = TruckState {
            speed: -0.5,
            ..middle
        };
        assert_eq!(
            moving.step(a, 0.1, 0.0, 0.02).unwrap(),
            moving.step(b, 0.1, 0.0, 0.02).unwrap(),
            "body offsets must not move the kinematic axle/hitch reference"
        );
    }
    #[test]
    fn two_moving_boxes_cannot_tunnel_through_each_other() {
        let a = OrientedBox {
            length: 0.2,
            width: 0.2,
            ..b(-1.0, 0.0, 0.0)
        };
        let next_a = OrientedBox {
            pose: Pose2 { x: 1.0, ..a.pose },
            ..a
        };
        let other = OrientedBox {
            length: 0.2,
            width: 0.2,
            ..b(0.0, -1.0, std::f64::consts::FRAC_PI_2)
        };
        let next_other = OrientedBox {
            pose: Pose2 {
                y: 1.0,
                ..other.pose
            },
            ..other
        };
        assert!(!overlaps(a, other, 0.0) && !overlaps(next_a, next_other, 0.0));
        assert!(moving_swept_collision(a, next_a, other, next_other, 0.0));
    }
}
