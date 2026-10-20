//! Exact obstacle-free Reeds–Shepp word families, in signed metres.
//! Formulae adapted from OMPL's ReedsSheppStateSpace (Mark Moll), BSD-3-Clause,
//! Copyright (c) 2010 Rice University. See vendor/licenses/OMPL-ReedsShepp.txt.
//! https://github.com/ompl/ompl/blob/main/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp
use etsads_msgs::{Pose2, frames::wrap_angle};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, PI};
const ZERO: f64 = 10.0 * f64::EPSILON;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Turn {
    Left,
    Straight,
    Right,
}
impl Turn {
    pub fn curvature(self, radius: f64) -> f64 {
        match self {
            Self::Left => 1.0 / radius,
            Self::Right => -1.0 / radius,
            Self::Straight => 0.0,
        }
    }
    fn reflected(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
            Self::Straight => Self::Straight,
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Segment {
    pub turn: Turn,
    pub length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Path {
    pub radius: f64,
    pub segments: Vec<Segment>,
}
impl Path {
    pub fn length(&self) -> f64 {
        self.segments.iter().map(|s| s.length.abs()).sum()
    }
    pub fn endpoint(&self, start: Pose2) -> Pose2 {
        self.segments.iter().fold(start, |pose, s| {
            advance(pose, s.turn, s.length, self.radius)
        })
    }
}
pub fn advance(p: Pose2, turn: Turn, distance: f64, radius: f64) -> Pose2 {
    let k = turn.curvature(radius);
    let yaw = p.yaw + k * distance;
    if k.abs() < 1e-15 {
        Pose2 {
            x: p.x + distance * p.yaw.cos(),
            y: p.y + distance * p.yaw.sin(),
            yaw: p.yaw,
        }
    } else {
        Pose2 {
            x: p.x + (yaw.sin() - p.yaw.sin()) / k,
            y: p.y + (-yaw.cos() + p.yaw.cos()) / k,
            yaw: wrap_angle(yaw),
        }
    }
}
type Triple = Option<(f64, f64, f64)>;
fn tau_omega(u: f64, v: f64, xi: f64, eta: f64, phi: f64) -> (f64, f64) {
    let delta = wrap_angle(u - v);
    let a = u.sin() - delta.sin();
    let b = u.cos() - delta.cos() - 1.0;
    let t1 = (eta * a - xi * b).atan2(xi * a + eta * b);
    let t2 = 2.0 * (delta.cos() - v.cos() - u.cos()) + 3.0;
    let tau = wrap_angle(t1 + if t2 < 0.0 { PI } else { 0.0 });
    (tau, wrap_angle(tau - u + v - phi))
}
fn lsl(x: f64, y: f64, phi: f64) -> Triple {
    let a = x - phi.sin();
    let b = y - 1.0 + phi.cos();
    let u = a.hypot(b);
    let t = b.atan2(a);
    let v = wrap_angle(phi - t);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}
fn lsr(x: f64, y: f64, phi: f64) -> Triple {
    let a = x + phi.sin();
    let b = y - 1.0 - phi.cos();
    let q = a * a + b * b;
    if q < 4.0 {
        return None;
    }
    let u = (q - 4.0).sqrt();
    let t = wrap_angle(b.atan2(a) + 2.0_f64.atan2(u));
    let v = wrap_angle(t - phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}
fn lrl(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x - phi.sin();
    let eta = y - 1.0 + phi.cos();
    let rho = xi.hypot(eta);
    if rho > 4.0 {
        return None;
    }
    let u = -2.0 * (rho * 0.25).clamp(-1.0, 1.0).asin();
    let t = wrap_angle(eta.atan2(xi) + 0.5 * u + PI);
    let v = wrap_angle(phi - t + u);
    (t >= -ZERO && u <= ZERO).then_some((t, u, v))
}
fn lr_lr(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x + phi.sin();
    let eta = y - 1.0 - phi.cos();
    let rho = (2.0 + xi.hypot(eta)) * 0.25;
    if rho > 1.0 {
        return None;
    }
    let u = rho.clamp(-1.0, 1.0).acos();
    let (t, v) = tau_omega(u, -u, xi, eta, phi);
    (t >= -ZERO && v <= ZERO).then_some((t, u, v))
}
fn l_rlr(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x + phi.sin();
    let eta = y - 1.0 - phi.cos();
    let rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if !(0.0..=1.0).contains(&rho) {
        return None;
    }
    let u = -rho.acos();
    if u < -FRAC_PI_2 {
        return None;
    }
    let (t, v) = tau_omega(u, u, xi, eta, phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}
fn lrsl(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x - phi.sin();
    let eta = y - 1.0 + phi.cos();
    let rho = xi.hypot(eta);
    if rho < 2.0 {
        return None;
    }
    let r = (rho * rho - 4.0).sqrt();
    let u = 2.0 - r;
    let t = wrap_angle(eta.atan2(xi) + r.atan2(-2.0));
    let v = wrap_angle(phi - FRAC_PI_2 - t);
    (t >= -ZERO && u <= ZERO && v <= ZERO).then_some((t, u, v))
}
fn lrsr(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x + phi.sin();
    let eta = y - 1.0 - phi.cos();
    let rho = xi.hypot(eta);
    if rho < 2.0 {
        return None;
    }
    let t = xi.atan2(-eta);
    let u = 2.0 - rho;
    let v = wrap_angle(t + FRAC_PI_2 - phi);
    (t >= -ZERO && u <= ZERO && v <= ZERO).then_some((t, u, v))
}
fn lrslr(x: f64, y: f64, phi: f64) -> Triple {
    let xi = x + phi.sin();
    let eta = y - 1.0 - phi.cos();
    let rho = xi.hypot(eta);
    if rho < 2.0 {
        return None;
    }
    let u = 4.0 - (rho * rho - 4.0).sqrt();
    if u > ZERO {
        return None;
    }
    let t = wrap_angle(((4.0 - u) * xi - 2.0 * eta).atan2(-2.0 * xi + (u - 4.0) * eta));
    let v = wrap_angle(t - phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}
fn word(kind: usize, x: f64, y: f64, phi: f64) -> Option<Vec<Segment>> {
    use Turn::{Left as L, Right as R, Straight as S};
    let (t, u, v) = family(kind, x, y, phi)?;
    let terms = match kind {
        0 => vec![(L, t), (S, u), (L, v)],
        1 => vec![(L, t), (S, u), (R, v)],
        2 => vec![(L, t), (R, u), (L, v)],
        3 => vec![(L, t), (R, u), (L, -u), (R, v)],
        4 => vec![(L, t), (R, u), (L, u), (R, v)],
        5 => vec![(L, t), (R, -FRAC_PI_2), (S, u), (L, v)],
        6 => vec![(L, t), (R, -FRAC_PI_2), (S, u), (R, v)],
        7 => vec![(L, t), (R, -FRAC_PI_2), (S, u), (L, -FRAC_PI_2), (R, v)],
        _ => unreachable!(),
    };
    Some(
        terms
            .into_iter()
            .map(|(turn, length)| Segment { turn, length })
            .collect(),
    )
}
fn family(kind: usize, x: f64, y: f64, phi: f64) -> Triple {
    match kind {
        0 => lsl(x, y, phi),
        1 => lsr(x, y, phi),
        2 => lrl(x, y, phi),
        3 => lr_lr(x, y, phi),
        4 => l_rlr(x, y, phi),
        5 => lrsl(x, y, phi),
        6 => lrsr(x, y, phi),
        7 => lrslr(x, y, phi),
        _ => None,
    }
}
/// All feasible canonical words, sorted by obstacle-free geometric length.
/// The caller must still validate obstacles, steering rate and articulation.
pub fn candidates(
    start: Pose2,
    goal: Pose2,
    radius: f64,
) -> Result<Vec<Path>, crate::PlanningError> {
    if !start.valid() || !goal.valid() || !radius.is_finite() || radius <= 0.0 || radius > 1e6 {
        return Err(crate::PlanningError::Invalid("Reeds-Shepp geometry"));
    }
    let dx = (goal.x - start.x) / radius;
    let dy = (goal.y - start.y) / radius;
    let x = start.yaw.cos() * dx + start.yaw.sin() * dy;
    let y = -start.yaw.sin() * dx + start.yaw.cos() * dy;
    let phi = wrap_angle(goal.yaw - start.yaw);
    if ![x, y, phi].iter().all(|v| v.is_finite()) {
        return Err(crate::PlanningError::Invalid("Reeds-Shepp coordinates"));
    }
    if x.hypot(y) < 1e-12 && phi.abs() < 1e-12 {
        return Ok(vec![Path {
            radius,
            segments: vec![],
        }]);
    }
    let mut paths = vec![];
    for backward in [false, true] {
        let (bx, by) = if backward {
            (x * phi.cos() + y * phi.sin(), x * phi.sin() - y * phi.cos())
        } else {
            (x, y)
        };
        for timeflip in [false, true] {
            for reflected in [false, true] {
                let tx = if timeflip { -bx } else { bx };
                let ty = if reflected { -by } else { by };
                let tp = if timeflip ^ reflected { -phi } else { phi };
                for kind in 0..8 {
                    if backward && ![2, 5, 6].contains(&kind) {
                        continue;
                    }
                    let Some(mut segments) = word(kind, tx, ty, tp) else {
                        continue;
                    };
                    if backward {
                        segments.reverse();
                    }
                    for segment in &mut segments {
                        segment.length *= radius * if timeflip { -1.0 } else { 1.0 };
                        if reflected {
                            segment.turn = segment.turn.reflected();
                        }
                    }
                    segments.retain(|s| s.length.abs() > 1e-12);
                    let path = Path { radius, segments };
                    let end = path.endpoint(start);
                    if !path.length().is_finite()
                        || end.distance(goal) > 1e-6 * radius.max(1.0)
                        || wrap_angle(end.yaw - goal.yaw).abs() > 1e-7
                    {
                        continue;
                    }
                    paths.push(path);
                }
            }
        }
    }
    paths.sort_by(|a, b| a.length().total_cmp(&b.length()));
    let mut unique: Vec<Path> = Vec::new();
    for path in paths {
        if !unique.iter().any(|other| {
            other.segments.len() == path.segments.len()
                && other
                    .segments
                    .iter()
                    .zip(&path.segments)
                    .all(|(a, b)| a.turn == b.turn && (a.length - b.length).abs() < 1e-8)
        }) {
            unique.push(path);
        }
    }
    let paths = unique;
    if paths.is_empty() {
        return Err(crate::PlanningError::NoPath);
    }
    Ok(paths)
}
pub fn distance(start: Pose2, goal: Pose2, radius: f64) -> Result<f64, crate::PlanningError> {
    if !start.valid() || !goal.valid() || !radius.is_finite() || radius <= 0.0 || radius > 1e6 {
        return Err(crate::PlanningError::Invalid("Reeds-Shepp geometry"));
    }
    let dx = (goal.x - start.x) / radius;
    let dy = (goal.y - start.y) / radius;
    let x = start.yaw.cos() * dx + start.yaw.sin() * dy;
    let y = -start.yaw.sin() * dx + start.yaw.cos() * dy;
    let phi = wrap_angle(goal.yaw - start.yaw);
    let mut best = f64::INFINITY;
    for backward in [false, true] {
        let (bx, by) = if backward {
            (x * phi.cos() + y * phi.sin(), x * phi.sin() - y * phi.cos())
        } else {
            (x, y)
        };
        for timeflip in [false, true] {
            for reflected in [false, true] {
                let tx = if timeflip { -bx } else { bx };
                let ty = if reflected { -by } else { by };
                let tp = if timeflip ^ reflected { -phi } else { phi };
                for kind in 0..8 {
                    if backward && ![2, 5, 6].contains(&kind) {
                        continue;
                    }
                    if let Some((t, u, v)) = family(kind, tx, ty, tp) {
                        let length = t.abs()
                            + v.abs()
                            + u.abs() * if kind == 3 || kind == 4 { 2.0 } else { 1.0 }
                            + match kind {
                                5 | 6 => FRAC_PI_2,
                                7 => PI,
                                _ => 0.0,
                            };
                        best = best.min(length);
                    }
                }
            }
        }
    }
    let result = best * radius;
    if result.is_finite() {
        Ok(result)
    } else {
        Err(crate::PlanningError::NoPath)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matches_independently_compiled_ompl_distance_reference() {
        for row in include_str!("../../testdata/reeds-shepp-ompl.csv").lines() {
            let values: Vec<f64> = row.split(',').map(|v| v.parse().unwrap()).collect();
            let actual = distance(
                Pose2::default(),
                Pose2 {
                    x: values[0],
                    y: values[1],
                    yaw: values[2],
                },
                1.0,
            )
            .unwrap();
            assert!((actual - values[3]).abs() < 1e-9, "{row}: {actual}");
        }
        let quarter = Pose2 {
            x: 4.0,
            y: 4.0,
            yaw: FRAC_PI_2,
        };
        assert!((distance(Pose2::default(), quarter, 4.0).unwrap() - 2.0 * PI).abs() < 1e-9);
        assert!(
            (distance(
                Pose2::default(),
                Pose2 {
                    yaw: PI,
                    ..Default::default()
                },
                1.0
            )
            .unwrap()
                - PI)
                .abs()
                < 1e-9
        );
    }
    #[test]
    fn shortest_paths_reach_pose_preserve_symmetries_and_scale() {
        for i in 0..250 {
            let p = Pose2 {
                x: ((i * 73 % 101) as f64 - 50.0) / 7.0,
                y: ((i * 37 % 103) as f64 - 51.0) / 8.0,
                yaw: (i as f64 * 0.37).sin() * PI,
            };
            let a = Pose2::default();
            let d = distance(a, p, 1.0).unwrap();
            assert!((d - candidates(a, p, 1.0).unwrap()[0].length()).abs() < 1e-7);
            assert!(d + 1e-7 >= p.distance(a));
            assert!((d - distance(p, a, 1.0).unwrap()).abs() < 1e-7);
            let reflect = Pose2 {
                x: p.x,
                y: -p.y,
                yaw: -p.yaw,
            };
            assert!((d - distance(a, reflect, 1.0).unwrap()).abs() < 1e-7);
            let scaled = Pose2 {
                x: p.x * 3.0,
                y: p.y * 3.0,
                yaw: p.yaw,
            };
            assert!((3.0 * d - distance(a, scaled, 3.0).unwrap()).abs() < 1e-6);
            for path in candidates(a, p, 1.0).unwrap() {
                let end = path.endpoint(a);
                assert!(end.distance(p) < 1e-6);
                assert!(wrap_angle(end.yaw - p.yaw).abs() < 1e-7);
            }
        }
        assert!(
            (distance(
                Pose2::default(),
                Pose2 {
                    x: -8.0,
                    ..Default::default()
                },
                4.0
            )
            .unwrap()
                - 8.0)
                .abs()
                < 1e-8
        );
    }
    #[test]
    fn reverse_cusps_are_required_for_sideways_goal_and_curves_are_not_straight_stubs() {
        let paths = candidates(
            Pose2::default(),
            Pose2 {
                x: 0.0,
                y: 2.0,
                yaw: 0.0,
            },
            4.0,
        )
        .unwrap();
        let p = &paths[0];
        assert!(p.segments.iter().any(|s| s.length < 0.0));
        assert!(p.segments.iter().any(|s| s.length > 0.0));
        assert!(p.segments.iter().any(|s| s.turn != Turn::Straight));
        assert!(
            distance(
                Pose2::default(),
                Pose2 {
                    yaw: f64::NAN,
                    ..Default::default()
                },
                4.0
            )
            .is_err()
        );
    }
}
