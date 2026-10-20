//! Bounded constant-acceleration prediction on a lane graph, including forks.
pub mod tracking;
use etsads_map::LaneGraph;
use etsads_msgs::{Agent, Point3, Pose2};
use serde::{Deserialize, Serialize};
pub use tracking::{
    AgentHint, PredictionBatch, PredictionContext, PredictionDiagnostics, Predictor, TurnHint,
};
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct PredictedPoint {
    pub t: f64,
    pub pose: Pose2,
    pub speed: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PredictionMode {
    pub probability: f64,
    pub points: Vec<PredictedPoint>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PredictedAgent {
    pub id: u64,
    pub length: f64,
    pub width: f64,
    pub modes: Vec<PredictionMode>,
}
#[derive(Debug, thiserror::Error)]
pub enum PredictionError {
    #[error("invalid agent or horizon")]
    Invalid,
    #[error("prediction track or branch budget exceeded")]
    Budget,
    #[error(transparent)]
    Map(#[from] etsads_map::MapError),
}
pub fn predict(
    graph: &LaneGraph,
    agent: &Agent,
    horizon: f64,
    dt: f64,
) -> Result<PredictedAgent, PredictionError> {
    if !agent.valid()
        || !(0.0..=70.0).contains(&agent.speed)
        || ![horizon, dt].iter().all(|v| v.is_finite())
        || horizon <= 0.0
        || horizon > 10.0
        || dt < 0.02
        || dt > horizon
    {
        return Err(PredictionError::Invalid);
    }
    let projection = if let Some(id) = agent.lane_id {
        Some(graph.project(
            id,
            Point3 {
                x: agent.pose.x,
                y: agent.pose.y,
                z: agent.z,
            },
        )?)
    } else {
        graph
            .nearest_lanes(
                Point3 {
                    x: agent.pose.x,
                    y: agent.pose.y,
                    z: agent.z,
                },
                5.0,
            )
            .into_iter()
            .filter(|p| {
                (p.z - agent.z).abs() < 3.0
                    && etsads_msgs::frames::wrap_angle(p.heading - agent.pose.yaw).abs() < 0.6
            })
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
    };
    let n = (horizon / dt).ceil() as usize;
    let acceleration = agent.acceleration.clamp(-3.0, 1.0);
    let initial_speed = agent.speed.max(0.0);
    let motion = |t: f64| {
        let effective = if acceleration < 0.0 {
            t.min(initial_speed / -acceleration)
        } else if acceleration > 0.0 {
            t.min((70.0 - initial_speed) / acceleration)
        } else {
            t
        };
        (
            initial_speed * effective
                + 0.5 * acceleration * effective * effective
                + if acceleration > 0.0 {
                    (t - effective) * 70.0
                } else {
                    0.0
                },
            (initial_speed + acceleration * t).clamp(0.0, 70.0),
        )
    };
    let mut modes = Vec::new();
    if let Some(p) = projection {
        let reachable = motion(horizon).0;
        let mut routes = Vec::new();
        let mut pending = vec![(vec![p.lane_id], graph.lanes[&p.lane_id].length() - p.s)];
        while let Some((path, distance)) = pending.pop() {
            let lane = &graph.lanes[path.last().unwrap()];
            if distance < reachable && !lane.successors.is_empty() && path.len() >= 16 {
                return Err(PredictionError::Budget);
            }
            if distance >= reachable || lane.successors.is_empty() {
                routes.push(path);
            } else {
                let mut next = lane.successors.clone();
                next.sort_unstable();
                if routes.len() + pending.len() + next.len() > 16 {
                    return Err(PredictionError::Budget);
                }
                for id in next.into_iter().rev() {
                    let mut branch = path.clone();
                    branch.push(id);
                    pending.push((branch, distance + graph.lanes[&id].length()));
                }
            }
        }
        let probability = 1.0 / routes.len() as f64;
        for route in routes {
            let mut points = Vec::with_capacity(n + 1);
            for i in 0..=n {
                let t = (i as f64 * dt).min(horizon);
                let (distance, speed) = motion(t);
                let mut s = p.s + distance;
                let mut lane = route[0];
                for (id_index, id) in route.iter().enumerate() {
                    lane = *id;
                    let len = graph.lanes[id].length();
                    if s <= len || id_index == route.len() - 1 {
                        break;
                    }
                    s -= len;
                }
                let l = &graph.lanes[&lane];
                let reached_end = s > l.length();
                let (pos, yaw) = l.sample(s).ok_or(PredictionError::Invalid)?;
                points.push(PredictedPoint {
                    t,
                    pose: Pose2 {
                        x: pos.x,
                        y: pos.y,
                        yaw,
                    },
                    speed: if reached_end { 0.0 } else { speed },
                });
            }
            modes.push(PredictionMode {
                probability,
                points,
            });
        }
    } else {
        let points = (0..=n)
            .map(|i| {
                let t = (i as f64 * dt).min(horizon);
                let (distance, speed) = motion(t);
                PredictedPoint {
                    t,
                    pose: Pose2 {
                        x: agent.pose.x + distance * agent.pose.yaw.cos(),
                        y: agent.pose.y + distance * agent.pose.yaw.sin(),
                        yaw: agent.pose.yaw,
                    },
                    speed,
                }
            })
            .collect();
        modes.push(PredictionMode {
            probability: 1.0,
            points,
        });
    }
    Ok(PredictedAgent {
        id: agent.id,
        length: agent.length,
        width: agent.width,
        modes,
    })
}
/// Continuous interpolation of one predicted mode; no extrapolation beyond
/// the validated horizon. Collision guards retain every nonzero mode.
pub fn sample_mode(mode: &PredictionMode, t: f64) -> Option<Pose2> {
    let first = mode.points.first()?;
    let last = mode.points.last()?;
    if !t.is_finite() || t < first.t || t > last.t + 1e-9 {
        return None;
    }
    let upper = mode.points.partition_point(|p| p.t < t);
    if upper == 0 {
        return Some(first.pose);
    }
    if upper >= mode.points.len() {
        return Some(last.pose);
    }
    let a = mode.points[upper - 1];
    let b = mode.points[upper];
    let f = (t - a.t) / (b.t - a.t);
    Some(Pose2 {
        x: a.pose.x + (b.pose.x - a.pose.x) * f,
        y: a.pose.y + (b.pose.y - a.pose.y) * f,
        yaw: etsads_msgs::frames::wrap_angle(
            a.pose.yaw + etsads_msgs::frames::wrap_angle(b.pose.yaw - a.pose.yaw) * f,
        ),
    })
}
/// Least-squares slope from a one-second history; timestamps must increase.
pub fn estimate_acceleration(samples: &[(f64, f64)]) -> Result<f64, PredictionError> {
    if samples.len() < 2
        || samples
            .iter()
            .any(|(t, v)| !t.is_finite() || !v.is_finite() || v.abs() > 100.0)
        || samples.windows(2).any(|p| p[0].0 >= p[1].0)
    {
        return Err(PredictionError::Invalid);
    }
    let now = samples.last().unwrap().0;
    let filtered: Vec<_> = samples.iter().filter(|(t, _)| now - t <= 1.0).collect();
    if filtered.len() < 2 {
        return Err(PredictionError::Invalid);
    }
    let mean_t = filtered.iter().map(|p| p.0 - now).sum::<f64>() / filtered.len() as f64;
    let mean_v = filtered.iter().map(|p| p.1).sum::<f64>() / filtered.len() as f64;
    let denom = filtered
        .iter()
        .map(|p| (p.0 - now - mean_t).powi(2))
        .sum::<f64>();
    if denom < 1e-12 {
        return Err(PredictionError::Invalid);
    }
    Ok((filtered
        .iter()
        .map(|p| (p.0 - now - mean_t) * (p.1 - mean_v))
        .sum::<f64>()
        / denom)
        .clamp(-3.0, 1.0))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn acceleration_fit_is_translation_invariant() {
        let s = vec![(1000000.0, 10.0), (1000000.5, 9.0), (1000001.0, 8.0)];
        assert!((estimate_acceleration(&s).unwrap() + 2.0).abs() < 1e-8);
        assert!(estimate_acceleration(&[(0.0, 0.0), (0.0, 1.0)]).is_err());
    }
    fn fork_graph() -> LaneGraph {
        let lane = |id, center, successors| etsads_map::Lane {
            id,
            center,
            width: 3.7,
            speed_limit: 20.0,
            successors,
            predecessors: vec![],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        };
        LaneGraph::new(vec![
            lane(
                1,
                vec![
                    Point3::default(),
                    Point3 {
                        x: 10.0,
                        ..Default::default()
                    },
                ],
                vec![2, 3],
            ),
            lane(
                2,
                vec![
                    Point3 {
                        x: 10.0,
                        ..Default::default()
                    },
                    Point3 {
                        x: 30.0,
                        y: 10.0,
                        z: 0.0,
                    },
                ],
                vec![],
            ),
            lane(
                3,
                vec![
                    Point3 {
                        x: 10.0,
                        ..Default::default()
                    },
                    Point3 {
                        x: 30.0,
                        y: -10.0,
                        z: 0.0,
                    },
                ],
                vec![],
            ),
        ])
        .unwrap()
    }
    fn car() -> Agent {
        Agent {
            id: 1,
            pose: Pose2 {
                x: 9.0,
                ..Default::default()
            },
            z: 0.0,
            speed: 5.0,
            acceleration: 0.0,
            length: 4.0,
            width: 2.0,
            lane_id: Some(1),
            trailers: vec![],
        }
    }
    #[test]
    fn branching_predictions_preserve_probability_and_diverge() {
        let prediction = predict(&fork_graph(), &car(), 4.0, 0.2).unwrap();
        assert_eq!(prediction.modes.len(), 2);
        assert!((prediction.modes.iter().map(|m| m.probability).sum::<f64>() - 1.0).abs() < 1e-12);
        assert!(
            prediction.modes[0].points.last().unwrap().pose.y
                * prediction.modes[1].points.last().unwrap().pose.y
                < 0.0
        );
    }
    #[test]
    fn braking_prediction_stops_without_reversing() {
        let mut agent = car();
        agent.pose.x = 0.0;
        agent.speed = 2.0;
        agent.acceleration = -3.0;
        let prediction = predict(&fork_graph(), &agent, 4.0, 0.2).unwrap();
        let points = &prediction.modes[0].points;
        assert!(points.windows(2).all(|p| p[1].pose.x >= p[0].pose.x));
        assert_eq!(points.last().unwrap().speed, 0.0);
        assert!((points.last().unwrap().pose.x - 2.0 / 3.0).abs() < 1e-9);
    }
}
