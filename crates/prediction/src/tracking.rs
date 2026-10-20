//! Per-agent temporal prediction. Native traffic currently exposes neither
//! indicator lamps nor another vehicle's navigation route: absent hints stay
//! unknown, and the ego route must never be substituted for a target route.
use crate::{PredictedAgent, PredictionError, estimate_acceleration, predict};
use etsads_map::{
    LaneGraph,
    matcher::{LaneState, Matcher},
};
use etsads_msgs::{Agent, Point3, Pose2, frames::wrap_angle};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

const MAX_TRACKS: usize = 800; // 200 traffic vehicles, each with at most 3 trailers.
const MAX_SAMPLES: usize = 128;
const HISTORY_SECONDS: f64 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TurnHint {
    Left,
    Right,
    Straight,
}
#[derive(Clone, Debug, Default)]
pub struct AgentHint {
    pub turn: Option<TurnHint>,
    /// Only an independently observed target route, never the ego GPS route.
    pub route_lanes: Vec<u64>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PredictionContext {
    pub session_start_qpc: u64,
    pub map_generation: u64,
    pub frame_id: u64,
    pub sim_time_us: u64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AgentPredictionDiagnostic {
    pub id: u64,
    pub lane_id: Option<u64>,
    pub lane_confidence: Option<f64>,
    pub fitted_acceleration_mps2: f64,
    pub velocity_samples: usize,
    pub history_span_s: f64,
    pub acceleration_fitted: bool,
    pub yaw_rate_hint: f64,
    pub lateral_velocity_hint: f64,
    pub explicit_turn_hint: bool,
    pub target_route_hint: bool,
    pub mode_probabilities: Vec<f64>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PredictionDiagnostics {
    pub frame_id: u64,
    pub session_start_qpc: u64,
    pub map_generation: u64,
    pub reset: bool,
    pub agents: Vec<AgentPredictionDiagnostic>,
}
pub struct PredictionBatch {
    pub agents: Vec<PredictedAgent>,
    pub diagnostics: PredictionDiagnostics,
}
struct Observation {
    time: f64,
    pose: Pose2,
    speed: f64,
}
#[derive(Default)]
struct Track {
    matcher: Matcher,
    samples: VecDeque<Observation>,
    lane: Option<LaneState>,
    last_hmm_time: Option<f64>,
    dimensions: Option<(f64, f64)>,
}
#[derive(Default)]
pub struct Predictor {
    tracks: BTreeMap<u64, Track>,
    context: Option<PredictionContext>,
}
impl Predictor {
    pub fn reset(&mut self) {
        self.tracks.clear();
        self.context = None;
    }
    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }
    pub fn predict(
        &mut self,
        graph: &LaneGraph,
        agents: &[Agent],
        context: PredictionContext,
        hints: &BTreeMap<u64, AgentHint>,
    ) -> Result<PredictionBatch, PredictionError> {
        if agents.len() > MAX_TRACKS {
            return Err(PredictionError::Budget);
        }
        let mut ids = BTreeSet::new();
        if agents
            .iter()
            .any(|a| !a.valid() || !(0.0..=70.0).contains(&a.speed) || !ids.insert(a.id))
        {
            self.reset();
            return Err(PredictionError::Invalid);
        }
        let reset = self.context.is_some_and(|old| {
            old.session_start_qpc != context.session_start_qpc
                || old.map_generation != context.map_generation
                || context.frame_id <= old.frame_id
                || context.sim_time_us <= old.sim_time_us
                || context.sim_time_us - old.sim_time_us > 500_000
        });
        if reset {
            self.reset();
        }
        let time = context.sim_time_us as f64 / 1e6;
        self.tracks.retain(|_, track| {
            track
                .samples
                .back()
                .is_some_and(|s| time - s.time <= HISTORY_SECONDS)
        });
        if self.tracks.keys().filter(|id| !ids.contains(id)).count() + agents.len() > MAX_TRACKS {
            self.tracks.retain(|id, _| ids.contains(id));
        }
        let mut result = PredictionBatch {
            agents: Vec::with_capacity(agents.len()),
            diagnostics: PredictionDiagnostics {
                frame_id: context.frame_id,
                session_start_qpc: context.session_start_qpc,
                map_generation: context.map_generation,
                reset,
                agents: Vec::with_capacity(agents.len()),
            },
        };
        for agent in agents {
            let hint = hints.get(&agent.id);
            if hint.is_some_and(|hint| {
                hint.route_lanes.len() > 64
                    || hint
                        .route_lanes
                        .iter()
                        .any(|id| !graph.lanes.contains_key(id))
            }) {
                return Err(PredictionError::Invalid);
            }
            let track = self.tracks.entry(agent.id).or_default();
            if track.samples.back().is_some_and(|old| {
                let dt = time - old.time;
                dt <= 0.0
                    || dt > 0.5
                    || agent.pose.distance(old.pose) > (old.speed * dt + 3.0).max(10.0)
            }) || track.dimensions.is_some_and(|(l, w)| {
                (l - agent.length).abs() > 0.5 || (w - agent.width).abs() > 0.3
            }) {
                *track = Track::default();
            }
            track.dimensions = Some((agent.length, agent.width));
            track.samples.push_back(Observation {
                time,
                pose: agent.pose,
                speed: agent.speed,
            });
            while track.samples.len() > MAX_SAMPLES
                || track
                    .samples
                    .front()
                    .is_some_and(|s| time - s.time > HISTORY_SECONDS)
            {
                track.samples.pop_front();
            }
            // HMM has its own 2 s history. Bound its update rate to 50 Hz even
            // if a caller supplies faster telemetry; projection below is fresh.
            if track
                .last_hmm_time
                .is_none_or(|old| time - old >= 0.02 - 1e-9)
            {
                if graph
                    .nearest_lanes(
                        Point3 {
                            x: agent.pose.x,
                            y: agent.pose.y,
                            z: agent.z,
                        },
                        5.0,
                    )
                    .len()
                    > 32
                {
                    return Err(PredictionError::Budget);
                }
                track.lane = track
                    .matcher
                    .update(graph, agent.pose, agent.z, agent.speed, time);
                track.last_hmm_time = Some(time);
            }
            let span = time - track.samples.front().unwrap().time;
            let fit = if track.samples.len() >= 2 && span >= 0.05 {
                let samples: Vec<_> = track.samples.iter().map(|s| (s.time, s.speed)).collect();
                Some(estimate_acceleration(&samples)?)
            } else {
                None
            };
            // Until enough distinct observations exist, retain the source's
            // bounded braking/acceleration observation. Do not invent zero
            // acceleration for a newly seen vehicle already braking.
            let acceleration = fit.unwrap_or_else(|| agent.acceleration.clamp(-3.0, 1.0));
            let heading = track
                .lane
                .and_then(|l| {
                    graph
                        .project(
                            l.lane_id,
                            Point3 {
                                x: agent.pose.x,
                                y: agent.pose.y,
                                z: agent.z,
                            },
                        )
                        .ok()
                })
                .map_or(agent.pose.yaw, |p| p.heading);
            let trend = track
                .samples
                .iter()
                .rev()
                .find(|s| time - s.time >= 0.15 && time - s.time <= 0.6);
            let (yaw_rate, lateral) = trend.map_or((0.0, 0.0), |old| {
                let dt = time - old.time;
                (
                    (wrap_angle(agent.pose.yaw - old.pose.yaw) / dt).clamp(-0.6, 0.6),
                    ((-(agent.pose.x - old.pose.x) * heading.sin()
                        + (agent.pose.y - old.pose.y) * heading.cos())
                        / dt)
                        .clamp(-3.0, 3.0),
                )
            });
            let mut target = agent.clone();
            target.acceleration = acceleration;
            target.lane_id = track.lane.map(|l| l.lane_id);
            let mut prediction = predict(graph, &target, 4.0, 0.2)?;
            let mut weights = Vec::with_capacity(prediction.modes.len());
            for mode in &mut prediction.modes {
                let start = mode.points[0].pose;
                let dx = agent.pose.x - start.x;
                let dy = agent.pose.y - start.y;
                for point in &mut mode.points {
                    let blend = (1.0 - point.t / 2.0).max(0.0);
                    point.pose.x += dx * blend;
                    point.pose.y += dy * blend;
                    if point.t == 0.0 {
                        point.pose = agent.pose;
                    }
                }
                let end = mode.points.last().unwrap().pose;
                let turn = wrap_angle(end.yaw - heading);
                let anticipated = wrap_angle(agent.pose.yaw + yaw_rate);
                let mut log_weight =
                    -0.5 * (wrap_angle(end.yaw - anticipated) / 0.5).powi(2) + lateral * turn.sin();
                if let Some(hint) = hint {
                    if let Some(direction) = hint.turn {
                        let agrees = match direction {
                            TurnHint::Left => turn > 0.12,
                            TurnHint::Right => turn < -0.12,
                            TurnHint::Straight => turn.abs() <= 0.12,
                        };
                        log_weight += if agrees { 1.4 } else { -0.7 };
                    }
                    if !hint.route_lanes.is_empty() {
                        let on_route = hint.route_lanes.iter().any(|id| {
                            graph
                                .project(
                                    *id,
                                    Point3 {
                                        x: end.x,
                                        y: end.y,
                                        z: agent.z,
                                    },
                                )
                                .is_ok_and(|p| {
                                    p.distance < 2.5 && wrap_angle(p.heading - end.yaw).abs() < 0.6
                                })
                        });
                        log_weight += if on_route { 1.1 } else { -0.4 };
                    }
                }
                weights.push(log_weight);
            }
            let max = weights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let weights: Vec<_> = weights.iter().map(|w| (w - max).exp().max(0.01)).collect();
            let total: f64 = weights.iter().sum();
            for (mode, weight) in prediction.modes.iter_mut().zip(weights) {
                mode.probability = weight / total;
            }
            result.diagnostics.agents.push(AgentPredictionDiagnostic {
                id: agent.id,
                lane_id: track.lane.map(|l| l.lane_id),
                lane_confidence: track.lane.map(|l| l.confidence),
                fitted_acceleration_mps2: acceleration,
                velocity_samples: track.samples.len(),
                history_span_s: span,
                acceleration_fitted: fit.is_some(),
                yaw_rate_hint: yaw_rate,
                lateral_velocity_hint: lateral,
                explicit_turn_hint: hint.is_some_and(|h| h.turn.is_some()),
                target_route_hint: hint.is_some_and(|h| !h.route_lanes.is_empty()),
                mode_probabilities: prediction.modes.iter().map(|m| m.probability).collect(),
            });
            result.agents.push(prediction);
        }
        self.context = Some(context);
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lane(id: u64, start: (f64, f64), end: (f64, f64), successors: Vec<u64>) -> etsads_map::Lane {
        etsads_map::Lane {
            id,
            center: vec![
                Point3 {
                    x: start.0,
                    y: start.1,
                    z: 0.0,
                },
                Point3 {
                    x: end.0,
                    y: end.1,
                    z: 0.0,
                },
            ],
            width: 3.7,
            speed_limit: 25.0,
            successors,
            predecessors: vec![],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        }
    }
    fn agent() -> Agent {
        Agent {
            id: 1,
            pose: Pose2 {
                x: 1.0,
                y: 0.0,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 5.0,
            acceleration: -20.0,
            length: 4.0,
            width: 2.0,
            lane_id: None,
            trailers: vec![],
        }
    }
    fn context(frame: u64) -> PredictionContext {
        PredictionContext {
            session_start_qpc: 9,
            map_generation: 1,
            frame_id: frame,
            sim_time_us: frame * 100_000,
        }
    }
    #[test]
    fn separate_hmm_retains_connected_lane_through_nearest_lane_noise_and_crossing() {
        let graph = LaneGraph::new(vec![
            lane(1, (0.0, 0.0), (100.0, 0.0), vec![]),
            lane(2, (0.0, 0.8), (100.0, 0.8), vec![]),
            lane(3, (10.0, -50.0), (10.0, 50.0), vec![]),
        ])
        .unwrap();
        let mut tracker = Predictor::default();
        let mut car = agent();
        for n in 1..=6 {
            car.pose.x = 7.0 + n as f64 * 0.5;
            car.pose.y = if n == 6 { 0.5 } else { 0.0 };
            car.lane_id = Some(2); // An external label is not a substitute for temporal matching.
            let batch = tracker
                .predict(&graph, &[car.clone()], context(n), &BTreeMap::new())
                .unwrap();
            assert_eq!(batch.diagnostics.agents[0].lane_id, Some(1));
        }
        assert_eq!(tracker.track_count(), 1);
    }
    #[test]
    fn one_second_fit_is_used_and_sessions_maps_clock_and_reused_ids_reset_history() {
        let graph = LaneGraph::new(vec![lane(1, (0.0, 0.0), (1000.0, 0.0), vec![])]).unwrap();
        let mut tracker = Predictor::default();
        let mut car = agent();
        let mut batch = None;
        for n in 0..=10 {
            let t = n as f64 * 0.1;
            car.speed = 5.0 + 0.8 * t;
            car.pose.x = 1.0 + 5.0 * t + 0.4 * t * t;
            batch = Some(
                tracker
                    .predict(&graph, &[car.clone()], context(n), &BTreeMap::new())
                    .unwrap(),
            );
        }
        let batch = batch.unwrap();
        let diagnostic = &batch.diagnostics.agents[0];
        assert!(diagnostic.acceleration_fitted);
        assert!((diagnostic.fitted_acceleration_mps2 - 0.8).abs() < 1e-9);
        assert!((batch.agents[0].modes[0].points.last().unwrap().speed - 9.0).abs() < 1e-9);
        assert!(diagnostic.velocity_samples <= MAX_SAMPLES);
        for changed in [
            PredictionContext {
                session_start_qpc: 10,
                ..context(11)
            },
            PredictionContext {
                session_start_qpc: 10,
                map_generation: 2,
                ..context(12)
            },
            PredictionContext {
                session_start_qpc: 10,
                map_generation: 2,
                ..context(2)
            },
        ] {
            let b = tracker
                .predict(&graph, &[car.clone()], changed, &BTreeMap::new())
                .unwrap();
            assert!(b.diagnostics.reset);
            assert_eq!(b.diagnostics.agents[0].velocity_samples, 1);
        }
        car.pose.x += 50.0;
        let b = tracker
            .predict(
                &graph,
                &[car],
                PredictionContext {
                    session_start_qpc: 10,
                    map_generation: 2,
                    ..context(3)
                },
                &BTreeMap::new(),
            )
            .unwrap();
        assert_eq!(b.diagnostics.agents[0].velocity_samples, 1);
    }
    #[test]
    fn branch_trends_and_explicit_target_hints_change_probabilities_without_dropping_modes() {
        let graph = LaneGraph::new(vec![
            lane(1, (0.0, 0.0), (10.0, 0.0), vec![2, 3]),
            lane(2, (10.0, 0.0), (40.0, 15.0), vec![]),
            lane(3, (10.0, 0.0), (40.0, -15.0), vec![]),
        ])
        .unwrap();
        let mut tracker = Predictor::default();
        let mut car = agent();
        let mut last = None;
        for n in 0..=5 {
            car.pose = Pose2 {
                x: 5.0 + n as f64 * 0.5,
                y: n as f64 * 0.04,
                yaw: n as f64 * 0.024,
            };
            last = Some(
                tracker
                    .predict(&graph, &[car.clone()], context(n), &BTreeMap::new())
                    .unwrap(),
            );
        }
        let batch = last.unwrap();
        let probability = |p: &PredictedAgent, left: bool| {
            p.modes
                .iter()
                .find(|m| (m.points.last().unwrap().pose.y > 0.0) == left)
                .unwrap()
                .probability
        };
        assert!(probability(&batch.agents[0], true) > probability(&batch.agents[0], false));
        assert!(!batch.diagnostics.agents[0].explicit_turn_hint);
        let hints = BTreeMap::from([(
            1,
            AgentHint {
                turn: Some(TurnHint::Right),
                route_lanes: vec![3],
            },
        )]);
        let mut explicit = Predictor::default();
        car.pose = Pose2 {
            x: 8.0,
            y: 0.0,
            yaw: 0.0,
        };
        let b = explicit
            .predict(&graph, &[car], context(1), &hints)
            .unwrap();
        assert!(probability(&b.agents[0], false) > probability(&b.agents[0], true));
        assert_eq!(b.agents[0].modes.len(), 2);
        assert!((b.agents[0].modes.iter().map(|m| m.probability).sum::<f64>() - 1.0).abs() < 1e-12);
        assert!(b.agents[0].modes.iter().all(|m| m.probability > 0.0));
    }
    #[test]
    fn invalid_duplicate_agents_and_track_budget_fail_closed() {
        let graph = LaneGraph::new(vec![lane(1, (0.0, 0.0), (100.0, 0.0), vec![])]).unwrap();
        let mut tracker = Predictor::default();
        assert!(
            tracker
                .predict(&graph, &[agent(), agent()], context(1), &BTreeMap::new())
                .is_err()
        );
        assert!(
            tracker
                .predict(
                    &graph,
                    &vec![agent(); MAX_TRACKS + 1],
                    context(1),
                    &BTreeMap::new()
                )
                .is_err()
        );
        let mut lanes = vec![lane(1, (0.0, 0.0), (2.0, 0.0), (2..=18).collect())];
        for id in 2..=18 {
            lanes.push(lane(id, (2.0, 0.0), (30.0, id as f64 - 10.0), vec![]));
        }
        let graph = LaneGraph::new(lanes).unwrap();
        let mut car = agent();
        car.acceleration = 0.0;
        assert!(
            matches!(
                tracker.predict(&graph, &[car], context(1), &BTreeMap::new()),
                Err(PredictionError::Budget)
            ),
            "unrepresented fork modes must fail rather than silently disappear"
        );
    }
}
