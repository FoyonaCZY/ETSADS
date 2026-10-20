//! Quintic lateral and quartic longitudinal Frenet sampling with full truck
//! envelopes. Candidates are rejected on dynamic, geometric or data violations.
use crate::{
    PlanningError,
    geometry::{footprints, overlaps, swept_collision},
};
use etsads_control::TruckState;
use etsads_map::LaneGraph;
use etsads_msgs::{
    BehaviorKind, Direction, EgoState, OrientedBox, Point3, Pose2, TrajPoint, Trajectory,
    VehicleGeometry, frames::wrap_angle,
};
use etsads_prediction::PredictedAgent;

#[derive(Clone, Copy, Debug)]
pub struct Quintic {
    c: [f64; 6],
}
impl Quintic {
    pub fn zero_derivative(start: f64, end: f64, t: f64) -> Result<Self, PlanningError> {
        Self::boundary(start, 0.0, 0.0, end, t)
    }
    pub fn boundary(
        start: f64,
        velocity: f64,
        acceleration: f64,
        end: f64,
        t: f64,
    ) -> Result<Self, PlanningError> {
        if ![start, velocity, acceleration, end, t]
            .iter()
            .all(|v| v.is_finite())
            || t <= 0.0
        {
            return Err(PlanningError::Invalid("quintic boundary"));
        }
        let d = end - start - velocity * t - 0.5 * acceleration * t * t;
        let dv = -velocity - acceleration * t;
        let da = -acceleration;
        Ok(Self {
            c: [
                start,
                velocity,
                acceleration / 2.0,
                (10.0 * d - 4.0 * dv * t + 0.5 * da * t * t) / t.powi(3),
                (-15.0 * d + 7.0 * dv * t - da * t * t) / t.powi(4),
                (6.0 * d - 3.0 * dv * t + 0.5 * da * t * t) / t.powi(5),
            ],
        })
    }
    pub fn evaluate(self, t: f64) -> (f64, f64, f64, f64) {
        let c = self.c;
        (
            c[0] + c[1] * t
                + c[2] * t.powi(2)
                + c[3] * t.powi(3)
                + c[4] * t.powi(4)
                + c[5] * t.powi(5),
            c[1] + 2.0 * c[2] * t
                + 3.0 * c[3] * t.powi(2)
                + 4.0 * c[4] * t.powi(3)
                + 5.0 * c[5] * t.powi(4),
            2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t.powi(2) + 20.0 * c[5] * t.powi(3),
            6.0 * c[3] + 24.0 * c[4] * t + 60.0 * c[5] * t.powi(2),
        )
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Quartic {
    c: [f64; 5],
}
impl Quartic {
    pub fn speed_boundary(
        s: f64,
        v: f64,
        a: f64,
        end_speed: f64,
        t: f64,
    ) -> Result<Self, PlanningError> {
        if ![s, v, a, end_speed, t].iter().all(|v| v.is_finite())
            || t <= 0.0
            || v < 0.0
            || end_speed < 0.0
        {
            return Err(PlanningError::Invalid("quartic boundary"));
        }
        let dv = end_speed - v - a * t;
        let da = -a;
        let c3 = dv / t.powi(2) - da / (3.0 * t);
        let c4 = da / (4.0 * t.powi(2)) - dv / (2.0 * t.powi(3));
        Ok(Self {
            c: [s, v, a / 2.0, c3, c4],
        })
    }
    pub fn evaluate(self, t: f64) -> (f64, f64, f64, f64) {
        let c = self.c;
        (
            c[0] + c[1] * t + c[2] * t * t + c[3] * t.powi(3) + c[4] * t.powi(4),
            c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t + 4.0 * c[4] * t.powi(3),
            2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t * t,
            6.0 * c[3] + 24.0 * c[4] * t,
        )
    }
}
#[derive(Clone, Debug)]
pub struct FrenetRequest<'a> {
    pub graph: &'a LaneGraph,
    pub lane_id: u64,
    pub ego: EgoState,
    pub target_speed: f64,
    pub lateral_targets: &'a [f64],
    pub predictions: &'a [PredictedAgent],
    pub obstacles: &'a [OrientedBox],
    pub geometry: VehicleGeometry,
    pub lateral_acceleration_limit: f64,
}
pub fn plan(request: FrenetRequest<'_>) -> Result<Trajectory, PlanningError> {
    let r = request;
    if !r.ego.valid()
        || !r.geometry.valid()
        || !r.target_speed.is_finite()
        || !(0.0..=45.0).contains(&r.target_speed)
        || r.lateral_targets.is_empty()
        || r.lateral_targets.len() > 9
        || r.lateral_targets
            .iter()
            .any(|d| !d.is_finite() || d.abs() > 15.0)
        || !r.lateral_acceleration_limit.is_finite()
        || r.lateral_acceleration_limit <= 0.0
        || r.obstacles.iter().any(|o| !o.valid())
    {
        return Err(PlanningError::Invalid("Frenet request"));
    }
    for agent in r.predictions {
        if !agent.length.is_finite()
            || !agent.width.is_finite()
            || agent.length <= 0.0
            || agent.width <= 0.0
            || agent.modes.is_empty()
        {
            return Err(PlanningError::Invalid("prediction geometry"));
        }
        let mut probability = 0.0;
        for mode in &agent.modes {
            probability += mode.probability;
            if !mode.probability.is_finite()
                || mode.probability <= 0.0
                || mode.points.len() < 2
                || mode.points.iter().any(|p| {
                    !p.pose.valid() || !p.t.is_finite() || !p.speed.is_finite() || p.t < 0.0
                })
                || mode.points.windows(2).any(|p| p[0].t >= p[1].t)
            {
                return Err(PlanningError::Invalid("prediction mode"));
            }
        }
        if (probability - 1.0).abs() > 1e-6 {
            return Err(PlanningError::Invalid("prediction probabilities"));
        }
    }
    let projection = r.graph.project(
        r.lane_id,
        Point3 {
            x: r.ego.pose.x,
            y: r.ego.pose.y,
            z: r.ego.z,
        },
    )?;
    let mut best = None;
    for &target_d in r.lateral_targets {
        for horizon in [4.0, 5.0, 6.0, 8.0] {
            // A trajectory cannot outlive its traffic prediction coverage.
            if r.predictions
                .iter()
                .flat_map(|a| &a.modes)
                .any(|m| m.points[0].t > 0.0 || m.points.last().unwrap().t < horizon)
            {
                continue;
            }
            for speed_delta in [0.0, -5.0 / 3.6, 5.0 / 3.6, -10.0 / 3.6, 10.0 / 3.6] {
                let target_v = (r.target_speed + speed_delta).clamp(0.0, r.target_speed);
                let heading_error = wrap_angle(r.ego.pose.yaw - projection.heading);
                let lateral = Quintic::boundary(
                    projection.d,
                    r.ego.speed * heading_error.sin(),
                    r.ego.acceleration * heading_error.sin()
                        + r.ego.speed * r.ego.yaw_rate * heading_error.cos(),
                    target_d,
                    horizon,
                )?;
                let longitudinal = Quartic::speed_boundary(
                    projection.s,
                    (r.ego.speed * heading_error.cos()).max(0.0),
                    (r.ego.acceleration * heading_error.cos()
                        - r.ego.speed * r.ego.yaw_rate * heading_error.sin())
                    .clamp(-3.0, 1.0),
                    target_v,
                    horizon,
                )?;
                let n = (horizon / 0.1).round() as usize;
                let mut points = Vec::with_capacity(n + 1);
                let mut score = 0.0;
                let mut valid = true;
                let mut trailer_yaw = r.ego.trailer_yaw;
                let mut previous_boxes = None;
                for i in 0..=n {
                    let t = i as f64 * 0.1;
                    let (s, v, a, j) = longitudinal.evaluate(t);
                    let (d, dd, ddd, dj) = lateral.evaluate(t);
                    if v < 0.0 || a.abs() > 4.0 || ddd.abs() > r.lateral_acceleration_limit {
                        valid = false;
                        break;
                    }
                    let (id, local_s) =
                        r.graph
                            .advance(r.lane_id, projection.s, (s - projection.s).max(0.0))?;
                    let lane = &r.graph.lanes[&id];
                    let (pos, heading) = lane
                        .sample(local_s)
                        .ok_or(PlanningError::Invalid("reference sampling"))?;
                    if v > 0.2 && local_s >= lane.length() - 1e-6 && lane.successors.is_empty() {
                        valid = false;
                        break;
                    }
                    let yaw = wrap_angle(heading + dd.atan2(v.max(0.01)));
                    let pose = Pose2 {
                        x: pos.x - d * heading.sin(),
                        y: pos.y + d * heading.cos(),
                        yaw,
                    };
                    let curvature = if let Some(p) = points.last() {
                        let p: &TrajPoint = p;
                        wrap_angle(yaw - p.pose.yaw) / pose.distance(p.pose).max(0.01)
                    } else {
                        0.0
                    };
                    if curvature.abs() * v * v > r.lateral_acceleration_limit
                        || (curvature * r.geometry.wheelbase).atan().abs() > r.geometry.max_steer
                    {
                        valid = false;
                        break;
                    }
                    if !r.geometry.has_trailer {
                        trailer_yaw = yaw;
                    } else if i > 0 {
                        let phi = wrap_angle(yaw - trailer_yaw);
                        trailer_yaw = wrap_angle(
                            trailer_yaw
                                + 0.1
                                    * (v * phi.sin() / r.geometry.trailer_wheelbase
                                        - v * r.geometry.hitch_offset * phi.cos() * curvature
                                            / r.geometry.trailer_wheelbase),
                        );
                    }
                    let articulation = wrap_angle(yaw - trailer_yaw);
                    if articulation.abs() > r.geometry.max_articulation {
                        valid = false;
                        break;
                    }
                    let boxes = footprints(
                        TruckState {
                            pose,
                            trailer_yaw,
                            speed: v,
                            steer: (curvature * r.geometry.wheelbase).atan(),
                        },
                        r.geometry,
                    );
                    for &b in &boxes {
                        if r.obstacles.iter().any(|o| overlaps(b, *o, 0.5)) {
                            valid = false;
                            break;
                        }
                    }
                    if let Some(previous) = previous_boxes {
                        let previous: Vec<OrientedBox> = previous;
                        for j in 0..boxes.len() {
                            if r.obstacles
                                .iter()
                                .any(|o| swept_collision(previous[j], boxes[j], *o, 0.5))
                            {
                                valid = false;
                                break;
                            }
                        }
                    }
                    for agent in r.predictions {
                        for mode in &agent.modes {
                            for pair in mode
                                .points
                                .windows(2)
                                .filter(|p| p[0].t <= t + 0.3 && p[1].t >= t - 0.3)
                            {
                                let from = OrientedBox {
                                    pose: pair[0].pose,
                                    length: agent.length,
                                    width: agent.width,
                                };
                                let to = OrientedBox {
                                    pose: pair[1].pose,
                                    length: agent.length,
                                    width: agent.width,
                                };
                                if boxes.iter().any(|b| swept_collision(from, to, *b, 0.5)) {
                                    valid = false;
                                }
                            }
                            for predicted in
                                mode.points.iter().filter(|p| (p.t - t).abs() <= 0.3 + 1e-9)
                            {
                                let obstacle = OrientedBox {
                                    pose: predicted.pose,
                                    length: agent.length,
                                    width: agent.width,
                                };
                                if boxes.iter().any(|b| overlaps(*b, obstacle, 0.5)) {
                                    valid = false;
                                }
                                let distance = pose.distance(predicted.pose);
                                score += mode.probability / (distance + 1.0);
                            }
                        }
                    }
                    if !valid {
                        break;
                    }
                    score +=
                        0.1 * (j * j + dj * dj + 0.1 * d * d + 0.1 * (r.target_speed - v).powi(2));
                    points.push(TrajPoint {
                        t,
                        pose,
                        speed: v,
                        acceleration: a,
                        curvature,
                        articulation,
                    });
                    previous_boxes = Some(boxes);
                }
                if valid {
                    score += 3.0 * (target_d - projection.d).abs() + 0.1 * horizon;
                    let trajectory = Trajectory {
                        header: r.ego.header,
                        points,
                        direction: Direction::Forward,
                        behavior: if (target_d - projection.d).abs() > 1.0 {
                            if target_d > projection.d {
                                BehaviorKind::LaneChangeLeft
                            } else {
                                BehaviorKind::LaneChangeRight
                            }
                        } else {
                            BehaviorKind::LaneFollow
                        },
                    };
                    if trajectory.validate().is_ok()
                        && best
                            .as_ref()
                            .is_none_or(|(cost, _): &(f64, Trajectory)| score < *cost)
                    {
                        best = Some((score, trajectory));
                    }
                }
            }
        }
    }
    best.map(|(_, t)| t).ok_or(PlanningError::NoPath)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn quintic_boundaries() {
        let q = Quintic::zero_derivative(2.0, 5.0, 4.0).unwrap();
        let (p, v, a, _) = q.evaluate(4.0);
        assert!((p - 5.0).abs() < 1e-10);
        assert!(v.abs() < 1e-10 && a.abs() < 1e-10);
    }
    #[test]
    fn replanning_preserves_measured_lateral_velocity_and_acceleration() {
        let q = Quintic::boundary(-3.0, 0.5, 0.1, 0.0, 5.0).unwrap();
        let (p, v, a, _) = q.evaluate(0.0);
        assert_eq!(p, -3.0);
        assert_eq!(v, 0.5);
        assert_eq!(a, 0.1);
        let (p, v, a, _) = q.evaluate(5.0);
        assert!(p.abs() < 1e-9 && v.abs() < 1e-9 && a.abs() < 1e-9);
    }
    #[test]
    fn quartic_boundaries() {
        let q = Quartic::speed_boundary(10.0, 5.0, 0.5, 9.0, 4.0).unwrap();
        let (_, v, a, _) = q.evaluate(4.0);
        assert!((v - 9.0).abs() < 1e-10);
        assert!(a.abs() < 1e-10);
    }
    #[test]
    fn frenet_rejects_obstacle_and_missing_prediction_coverage() {
        let graph = LaneGraph::new(vec![etsads_map::Lane {
            id: 1,
            center: vec![
                Point3::default(),
                Point3 {
                    x: 500.0,
                    ..Default::default()
                },
            ],
            width: 3.7,
            speed_limit: 25.0,
            successors: vec![],
            predecessors: vec![],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        }])
        .unwrap();
        let ego = EgoState {
            speed: 5.0,
            ..Default::default()
        };
        let geometry = VehicleGeometry::default();
        let request = FrenetRequest {
            graph: &graph,
            lane_id: 1,
            ego,
            target_speed: 5.0,
            lateral_targets: &[0.0],
            predictions: &[],
            obstacles: &[],
            geometry,
            lateral_acceleration_limit: 1.2,
        };
        assert!(plan(request.clone()).is_ok());
        let obstacle = OrientedBox {
            pose: Pose2 {
                x: 8.0,
                y: 0.0,
                yaw: 0.0,
            },
            length: 2.0,
            width: 4.0,
        };
        assert!(
            plan(FrenetRequest {
                obstacles: &[obstacle],
                ..request.clone()
            })
            .is_err()
        );
        let prediction = PredictedAgent {
            id: 1,
            length: 4.0,
            width: 2.0,
            modes: vec![etsads_prediction::PredictionMode {
                probability: 1.0,
                points: vec![
                    etsads_prediction::PredictedPoint {
                        t: 0.0,
                        pose: Pose2 {
                            x: 100.0,
                            ..Default::default()
                        },
                        speed: 0.0,
                    },
                    etsads_prediction::PredictedPoint {
                        t: 2.0,
                        pose: Pose2 {
                            x: 100.0,
                            ..Default::default()
                        },
                        speed: 0.0,
                    },
                ],
            }],
        };
        assert!(
            plan(FrenetRequest {
                predictions: &[prediction],
                ..request
            })
            .is_err()
        );
    }
}
