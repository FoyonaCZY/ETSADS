//! Route behaviour orchestration for signals, priority, deliberate lane changes
//! and toll interactions. All permissions come from explicit map annotations.
use crate::parking_mission::{PlanningIntent, VerifiedBoundary};
use etsads_map::{
    LaneGraph,
    matcher::LaneState,
    traffic::{
        ControlKind, RightOfWay, SignalDecision, SignalHoldReason, SignalObservation,
        TrafficContext,
    },
};
use etsads_msgs::{
    Agent, BarrierState, BehaviorKind, EgoState, Point3,
    frames::wrap_angle,
    protocol::{ExtensionFrame, GameEvent},
};
use etsads_planning::{
    behavior::{GateInput, GateMachine, GatePhase, must_yield},
    speed::lane_change_safe,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
#[path = "gates.rs"]
pub mod gates;
pub use gates::{GateDiagnostic, GatePlan};
#[cfg(test)]
#[path = "gate_tests.rs"]
mod gate_tests;
#[derive(Clone, Debug)]
struct TrackedGate {
    plan: GatePlan,
    machine: GateMachine,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LaneChangeMission {
    pub from_lane: u64,
    pub to_lane: u64,
    pub earliest_s: f64,
    pub deadline_s: f64,
    /// Native corridor start deadline, distinct from completion at the exit.
    #[serde(default)]
    pub latest_start_s: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TollStop {
    pub id: String,
    pub lane_id: u64,
    pub barrier_position: Point3,
    pub stop_s: f64,
    #[serde(default)]
    pub gate: Option<GatePlan>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoundaboutExit {
    pub lane_id: u64,
    pub exit_s: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlledRouteMission {
    pub route: Vec<u64>,
    pub maximum_speed_mps: f64,
    pub boundary: VerifiedBoundary,
    pub traffic: TrafficContext,
    pub lane_changes: Vec<LaneChangeMission>,
    pub tolls: Vec<TollStop>,
    #[serde(default)]
    pub roundabout_exits: Vec<RoundaboutExit>,
    pub live_manoeuvres_verified: bool,
    pub control_evidence_path: String,
}
#[derive(Clone, Debug)]
pub(crate) struct RouteDirective {
    pub reference_lane: u64,
    pub speed_limit: f64,
    pub stop_distance: Option<f64>,
    pub behavior: BehaviorKind,
    pub intent: PlanningIntent,
}
#[derive(Clone, Copy, Debug)]
struct ChangeState {
    from: u64,
    to: u64,
    since: f64,
    executing: bool,
}
pub(crate) struct RouteEngine {
    mission: ControlledRouteMission,
    source_ids: BTreeMap<u64, String>,
    gates: HashMap<String, TrackedGate>,
    geometry: etsads_msgs::VehicleGeometry,
    interactions_authorized: bool,
    stopped_at: HashMap<String, f64>,
    stop_completed: HashSet<String>,
    change: Option<ChangeState>,
    change_completed: HashSet<(u64, u64)>,
    start_frame: Option<u64>,
    clearance_length_m: f64,
    front_offset_m: f64,
}
impl RouteEngine {
    /// Rebind an unchanged source-scoped mission after regional numeric IDs
    /// change, preserving stop/toll progress and completed lateral transitions.
    pub(crate) fn rebind(
        &mut self,
        mission: ControlledRouteMission,
        graph: &LaneGraph,
        source_ids: BTreeMap<u64, String>,
        geometry: etsads_msgs::VehicleGeometry,
    ) -> Result<(), String> {
        if self.change.is_some() {
            return Err("map replacement must wait for the active lane change to finish".into());
        }
        let mut next = Self::new(
            mission,
            graph,
            source_ids,
            geometry,
            self.interactions_authorized,
        )?;
        for (id, old) in std::mem::take(&mut self.gates) {
            if let Some(new) = next.gates.get_mut(&id) {
                if !old.plan.same_semantics(&new.plan) {
                    return Err("regional replacement changed active gate semantics".into());
                }
                if old.machine.committed() {
                    *new = old;
                } else {
                    new.machine = old.machine;
                }
            } else if old.machine.committed() {
                next.gates.insert(id, old);
            }
        }
        if next.gates.len() > 256 {
            return Err("gate progress retention budget exceeded".into());
        }
        next.stopped_at = std::mem::take(&mut self.stopped_at);
        next.stop_completed = std::mem::take(&mut self.stop_completed);
        next.start_frame = self.start_frame;
        for (from, to) in &self.change_completed {
            let a = self.source_ids.get(from);
            let b = self.source_ids.get(to);
            if let (Some(a), Some(b)) = (a, b)
                && let (Some((&x, _)), Some((&y, _))) = (
                    next.source_ids.iter().find(|(_, s)| *s == a),
                    next.source_ids.iter().find(|(_, s)| *s == b),
                )
            {
                next.change_completed.insert((x, y));
            }
        }
        *self = next;
        Ok(())
    }
    pub(crate) fn new(
        mission: ControlledRouteMission,
        graph: &LaneGraph,
        source_ids: BTreeMap<u64, String>,
        geometry: etsads_msgs::VehicleGeometry,
        interactions_authorized: bool,
    ) -> Result<Self, String> {
        mission.traffic.validate().map_err(|e| e.to_string())?;
        if !geometry.valid() {
            return Err("invalid truck clearance geometry".into());
        }
        let sources: Vec<_> = mission
            .route
            .iter()
            .filter_map(|id| source_ids.get(id).cloned())
            .collect();
        if !mission.boundary.valid()
            || mission.control_evidence_path.is_empty()
            || mission.route.is_empty()
            || sources.len() != mission.route.len()
            || !mission.traffic.has_annotations_for_route(&sources)
            || !mission.maximum_speed_mps.is_finite()
            || mission.maximum_speed_mps <= 0.0
            || mission.maximum_speed_mps > 25.0
        {
            return Err("controlled route requires verified bounds, complete traffic annotations and explicit speed limit".into());
        }
        for source in &sources {
            if source.contains(":prefab:")
                && !mission
                    .traffic
                    .lane_controls
                    .iter()
                    .any(|c| c.lane_id == *source && c.control_kind != ControlKind::Unknown)
            {
                return Err("prefab route lacks an explicit known TrafficRule annotation".into());
            }
        }
        for change in &mission.lane_changes {
            if change
                .latest_start_s
                .is_some_and(|s| !s.is_finite() || s < change.earliest_s || s >= change.deadline_s)
            {
                return Err("invalid mandatory lane-change start deadline".into());
            }
            let from = graph
                .lanes
                .get(&change.from_lane)
                .ok_or("lane-change source missing")?;
            if !graph.lanes.contains_key(&change.to_lane)
                || !((from.left == Some(change.to_lane) && from.can_change_left)
                    || (from.right == Some(change.to_lane) && from.can_change_right))
                || !change.earliest_s.is_finite()
                || !change.deadline_s.is_finite()
                || change.earliest_s < 0.0
                || change.deadline_s <= change.earliest_s
                || change.deadline_s > from.length()
            {
                return Err("lane change lacks legal graph adjacency or valid deadline".into());
            }
        }
        for pair in mission.route.windows(2) {
            if !graph
                .lanes
                .get(&pair[0])
                .is_some_and(|lane| lane.successors.contains(&pair[1]))
                && !mission
                    .lane_changes
                    .iter()
                    .any(|c| c.from_lane == pair[0] && c.to_lane == pair[1])
            {
                return Err("controlled route has a disconnected or unapproved lane change".into());
            }
        }
        let mut gates = HashMap::new();
        for toll in &mission.tolls {
            if toll.id.is_empty()
                || !toll.barrier_position.valid()
                || !toll.stop_s.is_finite()
                || toll.stop_s < 0.0
                || !graph.lanes.contains_key(&toll.lane_id)
                || toll.stop_s > graph.lanes[&toll.lane_id].length()
            {
                return Err("invalid toll location".into());
            }
            if (graph
                .project(toll.lane_id, toll.barrier_position)
                .map_err(|e| e.to_string())?
                .s
                - toll.stop_s)
                .abs()
                > 1.5
            {
                return Err("gate station disagrees with its world position".into());
            }
            let source = source_ids
                .get(&toll.lane_id)
                .ok_or("gate has no stable source lane ID")?;
            let plan = toll.gate.clone().map_or_else(
                || GatePlan::legacy(toll, graph, source, &mission.traffic),
                Ok,
            )?;
            plan.validate()?;
            plan.validate_signal_semantics(&mission.traffic)?;
            if plan.source_lane_id != *source {
                return Err("gate source lane binding mismatch".into());
            }
            let tracked = TrackedGate {
                machine: GateMachine::new(plan.mode, plan.requires_payment_event),
                plan,
            };
            if gates.insert(toll.id.clone(), tracked).is_some() {
                return Err("duplicate gate identity".into());
            }
        }
        if gates.len() > 128 {
            return Err("gate mission count budget exceeded".into());
        }
        for exit in &mission.roundabout_exits {
            if !exit.exit_s.is_finite()
                || exit.exit_s < 0.0
                || graph
                    .lanes
                    .get(&exit.lane_id)
                    .is_none_or(|lane| exit.exit_s > lane.length())
            {
                return Err("invalid roundabout exit annotation".into());
            }
        }
        Ok(Self {
            mission,
            source_ids,
            gates,
            geometry,
            interactions_authorized,
            stopped_at: HashMap::new(),
            stop_completed: HashSet::new(),
            change: None,
            change_completed: HashSet::new(),
            start_frame: None,
            clearance_length_m: geometry.tractor_length + geometry.trailer_length,
            front_offset_m: geometry.resolved_envelope().tractor_front_from_rear_axle_m,
        })
    }
    pub(crate) fn update(
        &mut self,
        graph: &LaneGraph,
        ego: EgoState,
        lane: LaneState,
        agents: &[Agent],
        extension: Option<&ExtensionFrame>,
        events: &[GameEvent],
    ) -> Result<RouteDirective, String> {
        let time = ego.header.sim_time_us as f64 / 1e6;
        let start = *self.start_frame.get_or_insert(ego.header.frame_id);
        let mut directive = RouteDirective {
            reference_lane: lane.lane_id,
            speed_limit: self.mission.maximum_speed_mps,
            stop_distance: None,
            behavior: BehaviorKind::LaneFollow,
            intent: PlanningIntent {
                reason: "controlled route lane following".into(),
                ..Default::default()
            },
        };
        if let Some(change) = self.change
            && change.executing
            && lane.lane_id == change.to
            && lane.d.abs() < 0.3
            && lane.heading_err.abs() < 0.1
        {
            self.change_completed.insert((change.from, change.to));
            self.change = None;
        }
        if self.change.is_none()
            && let Some(change) = self.mission.lane_changes.iter().find(|c| {
                c.from_lane == lane.lane_id
                    && lane.s >= c.earliest_s
                    && !self.change_completed.contains(&(c.from_lane, c.to_lane))
            })
        {
            self.change = Some(ChangeState {
                from: change.from_lane,
                to: change.to_lane,
                since: time,
                executing: false,
            });
        }
        if let Some(mut change) = self.change {
            let left = graph.lanes[&change.from].left == Some(change.to);
            directive.intent.indicator = if left { 1 } else { -1 };
            directive.intent.active_lane_change = true;
            let position = graph
                .project(
                    change.to,
                    Point3 {
                        x: ego.pose.x,
                        y: ego.pose.y,
                        z: ego.z,
                    },
                )
                .map_err(|e| e.to_string())?;
            let (front, rear) = adjacent_gaps(graph, change.to, position.s, agents);
            let safe = lane_change_safe(ego.speed.max(0.0), front, rear);
            let request = self
                .mission
                .lane_changes
                .iter()
                .find(|c| c.from_lane == change.from && c.to_lane == change.to)
                .unwrap();
            if !change.executing && request.latest_start_s.is_some_and(|s| lane.s >= s) {
                return Err(
                    "mandatory lane-change start deadline reached without a safe gap".into(),
                );
            }
            if !change.executing && time - change.since >= 3.0 && safe {
                change.executing = true;
                self.change = Some(change);
            }
            if change.executing {
                let request = self
                    .mission
                    .lane_changes
                    .iter()
                    .find(|c| c.from_lane == change.from && c.to_lane == change.to)
                    .unwrap();
                let source_progress = graph
                    .project(
                        change.from,
                        Point3 {
                            x: ego.pose.x,
                            y: ego.pose.y,
                            z: ego.z,
                        },
                    )
                    .map_err(|e| e.to_string())?
                    .s;
                if source_progress >= request.deadline_s {
                    return Err(
                        "mandatory lane-change deadline reached before stable completion".into(),
                    );
                }
                if !safe {
                    return Err("target lane gap became unsafe during lane change".into());
                }
                directive.reference_lane = change.to;
                directive.behavior = if left {
                    BehaviorKind::LaneChangeLeft
                } else {
                    BehaviorKind::LaneChangeRight
                };
                directive.intent.reason =
                    "Frenet lane change after three-second signal and gap validation".into();
            } else {
                let request = self
                    .mission
                    .lane_changes
                    .iter()
                    .find(|c| c.from_lane == change.from && c.to_lane == change.to)
                    .unwrap();
                let stop_before = request.latest_start_s.unwrap_or(request.deadline_s);
                if stop_before - lane.s < ego.speed.max(0.0) * 5.0 + 15.0 {
                    directive.stop_distance = Some((stop_before - lane.s - 5.0).max(0.0));
                }
                directive.intent.reason =
                    "signalling and waiting for front/rear lane-change gaps".into();
            }
        }
        if self.mission.roundabout_exits.iter().any(|exit| {
            exit.lane_id == lane.lane_id && (0.0..=30.0).contains(&(exit.exit_s - lane.s))
        }) {
            directive.intent.indicator = -1;
            directive.behavior = BehaviorKind::Roundabout;
            directive.intent.reason = "roundabout exit within 30 m; right indicator".into();
        }
        let route_index = self
            .mission
            .route
            .iter()
            .position(|id| *id == lane.lane_id)
            .ok_or("current lane outside controlled route")?;
        let mut offsets = HashMap::new();
        let mut offset = -lane.s;
        for id in &self.mission.route[route_index..] {
            offsets.insert(*id, offset);
            offset += graph.lanes[id].length();
            if offset > 500.0 {
                break;
            }
        }
        let route_sources: Vec<_> = self.mission.route[route_index..]
            .iter()
            .map(|id| self.source_ids[id].clone())
            .collect();
        let observed = signal_observations(extension, ego.header.frame_id);
        let controls = self.mission.traffic.upcoming_stop_controls(&route_sources);
        for control in controls {
            let Some((&id, &base)) = offsets
                .iter()
                .find(|(id, _)| self.source_ids[id] == control.lane_id)
            else {
                continue;
            };
            let projected = graph
                .project(id, control.position)
                .map_err(|e| e.to_string())?;
            let distance = base + projected.s - control.stop_before_m - self.front_offset_m - 3.0;
            if distance < -3.0 || distance > 300.0 {
                continue;
            }
            let key = format!(
                "{}:{:.3}:{:.3}",
                control.lane_id, control.position.x, control.position.y
            );
            let mut stop = control.requires_conservative_wait
                || matches!(control.priority, RightOfWay::Unknown);
            let mut reason = "unknown priority requires a conservative stop";
            if control.control_kind == ControlKind::Signal
                || control.priority == RightOfWay::SignalRequired
            {
                let handled_gate = control.signal_group_id.as_ref().is_some_and(|group| {
                    self.gates
                        .values()
                        .any(|gate| gate.plan.native_signal_group_id.as_ref() == Some(group))
                });
                if handled_gate {
                    // The gate machine handles OPEN/CLOSED, not the traffic
                    // light GREEN=8 convention. Conflict checks below remain.
                    stop = false;
                    reason = "native gate state handled independently";
                } else {
                    let decision = control
                        .signal_group_id
                        .as_ref()
                        .map(|group| {
                            self.mission
                                .traffic
                                .signal_decision(group, &observed, 1.5, 200_000)
                        })
                        .unwrap_or(SignalDecision::Hold(SignalHoldReason::MissingGroup));
                    stop = !decision.can_proceed();
                    reason = "traffic signal holds this route";
                    if matches!(
                        decision,
                        SignalDecision::Hold(SignalHoldReason::NotGreen(1))
                    ) && ego.speed * 0.3 + ego.speed * ego.speed / (2.0 * 1.5)
                        > distance.max(0.0)
                    {
                        stop = false;
                    }
                    if decision.can_proceed()
                        && let Some(group) = &control.signal_group_id
                        && let Some(remaining) = green_remaining(
                            &self.mission.traffic,
                            group,
                            extension,
                            ego.header.frame_id,
                        )
                    {
                        let clearance = (if control.lane_id.contains(":prefab:") {
                            graph.lanes[&id].length()
                        } else {
                            20.0
                        } + self.clearance_length_m)
                            / ego.speed.max(3.0);
                        let stopping = ego.speed * 0.3 + ego.speed * ego.speed / (2.0 * 1.5);
                        if remaining > 0.0 && remaining < clearance && stopping <= distance {
                            stop = true;
                            reason = "green time is insufficient to clear the conflict area";
                        }
                    }
                }
            }
            if control.control_kind == ControlKind::Stop && !self.stop_completed.contains(&key) {
                if distance <= 1.0 && ego.speed.abs() < 0.1 {
                    let since = self.stopped_at.entry(key.clone()).or_insert(time);
                    if time - *since >= 1.0 {
                        self.stop_completed.insert(key.clone());
                    } else {
                        stop = true;
                    }
                } else {
                    stop = true;
                }
                reason = "stop-sign dwell before conflict gap selection";
            }
            let mut arrivals = Vec::new();
            for source in &control.conflict_lane_ids {
                if let Some((&conflicting_id, _)) =
                    self.source_ids.iter().find(|(_, id)| *id == source)
                {
                    let conflict_s = graph
                        .project(conflicting_id, control.position)
                        .map_err(|e| e.to_string())?
                        .s;
                    for agent in agents {
                        let p = graph
                            .project(
                                conflicting_id,
                                Point3 {
                                    x: agent.pose.x,
                                    y: agent.pose.y,
                                    z: agent.z,
                                },
                            )
                            .map_err(|e| e.to_string())?;
                        if p.distance > graph.lanes[&conflicting_id].width / 2.0 + 0.3
                            || wrap_angle(agent.pose.yaw - p.heading).abs() > 0.6
                            || (p.z - agent.z).abs() > 3.0
                        {
                            continue;
                        }
                        let gap = conflict_s - p.s - agent.length / 2.0;
                        if gap < -agent.length {
                            continue;
                        }
                        arrivals.push(if gap <= 0.0 {
                            0.0
                        } else {
                            (agent.speed.max(0.0).powi(2) + 2.0 * gap).sqrt() - agent.speed.max(0.0)
                        });
                    }
                }
            }
            let current = &graph.lanes[&id];
            let turn = wrap_angle(
                current.sample(current.length()).unwrap().1 - current.sample(0.0).unwrap().1,
            );
            let ego_arrival = distance.max(0.0) / ego.speed.max(3.0);
            if must_yield(ego_arrival, &arrivals, turn > 0.35) {
                stop = true;
                reason = "conflicting traffic arrival gap is insufficient";
            }
            if !stop && control.priority == RightOfWay::ZipMerge {
                directive.behavior = BehaviorKind::Merge;
            }
            if stop {
                directive.stop_distance = Some(
                    directive
                        .stop_distance
                        .map_or(distance.max(0.0), |d| d.min(distance.max(0.0))),
                );
                directive.behavior = if control.control_kind == ControlKind::Signal
                    || control.control_kind == ControlKind::Stop
                {
                    BehaviorKind::Stop
                } else {
                    BehaviorKind::Yield
                };
                directive.intent.reason = reason.into();
            }
        }
        self.update_gates(ego, &offsets, extension, events, start, &mut directive)?;
        if directive
            .stop_distance
            .is_some_and(|distance| distance < 0.5)
            && ego.speed.abs() < 0.1
        {
            directive.intent.hold_brake = true;
        }
        Ok(directive)
    }
    fn update_gates(
        &mut self,
        ego: EgoState,
        offsets: &HashMap<u64, f64>,
        extension: Option<&ExtensionFrame>,
        events: &[GameEvent],
        start: u64,
        directive: &mut RouteDirective,
    ) -> Result<(), String> {
        let mut upcoming = Vec::new();
        for (id, gate) in &self.gates {
            if gate.machine.phase() == GatePhase::Complete {
                continue;
            }
            let (physical_distance, rear_cleared) = gate.plan.clearance(ego, self.geometry);
            let route_distance = self
                .mission
                .tolls
                .iter()
                .find(|t| &t.id == id)
                .and_then(|t| {
                    offsets
                        .get(&t.lane_id)
                        .map(|base| base + t.stop_s - self.front_offset_m)
                });
            let distance = if let Some(distance) = route_distance {
                distance
            } else if gate.machine.committed() {
                physical_distance
            } else {
                continue;
            };
            if distance > 500.0 && !gate.machine.committed() {
                continue;
            }
            let state = gate_barrier_state(
                extension,
                ego.header.frame_id,
                &gate.plan.observation_positions,
            );
            upcoming.push((id.clone(), distance, physical_distance, rear_cleared, state));
        }
        upcoming.sort_by(|a, b| a.1.abs().total_cmp(&b.1.abs()).then(a.0.cmp(&b.0)));
        let paid = events.iter().any(|e| {
            e.header.frame_id >= start
                && e.header.frame_id <= ego.header.frame_id
                && e.name.split(|b| *b == 0).next() == Some(b"player.tollgate.paid".as_slice())
        });
        // The SDK event has no native semaphore UID. Associate it only to one
        // spatially unambiguous committed interaction; never broadcast receipts.
        let mut pay_candidates: Vec<_> = upcoming
            .iter()
            .filter(|(id, _, physical, _, _)| {
                physical.abs() <= 15.0
                    && (self.gates[id].machine.committed()
                        || self.gates[id].plan.mode != gates::GateMode::Manual
                            && gate_barrier_state(
                                extension,
                                ego.header.frame_id,
                                &self.gates[id].plan.observation_positions,
                            ) == Some(BarrierState::Open))
            })
            .collect();
        pay_candidates.sort_by(|a, b| a.2.abs().total_cmp(&b.2.abs()).then(a.0.cmp(&b.0)));
        let pay_id = if paid
            && !pay_candidates.is_empty()
            && pay_candidates
                .get(1)
                .is_none_or(|b| (b.2.abs() - pay_candidates[0].2.abs()).abs() > 1.5)
        {
            Some(pay_candidates[0].0.as_str())
        } else {
            None
        };
        let mut activation_selected = false;
        for (id, distance, physical_distance, rear_cleared, state) in &upcoming {
            let gate = self.gates.get_mut(id).unwrap();
            let action = gate
                .machine
                .update(GateInput {
                    time_s: ego.header.sim_time_us as f64 / 1e6,
                    distance_m: *distance,
                    speed_mps: ego.speed.abs(),
                    barrier: *state,
                    rear_cleared: *rear_cleared,
                    activation_zone: (0.0..=3.5).contains(distance)
                        && (0.0..=5.0).contains(physical_distance),
                    paid_event: pay_id == Some(id.as_str()),
                    activation_authorized: self.interactions_authorized && !activation_selected,
                })
                .map_err(|e| e.to_string())?;
            activation_selected |= action.activate;
            let diagnostic = GateDiagnostic {
                id: id.clone(),
                source_lane_id: gate.plan.source_lane_id.clone(),
                native_signal_group_id: gate.plan.native_signal_group_id.clone(),
                mode: gate.plan.mode,
                barrier: *state,
                distance_m: *distance,
                physical_front_distance_m: *physical_distance,
                rear_cleared: *rear_cleared,
                payment_required: gate.plan.requires_payment_event,
                action: action.clone(),
                geometry_source: gate.plan.geometry_source.clone(),
                profile_evidence_ids: gate.plan.profile_evidence_ids.clone(),
                observation_positions: gate.plan.observation_positions.clone(),
            };
            if directive.intent.gate.is_none() || action.activate {
                directive.intent.gate = Some(Box::new(diagnostic));
            }
            if action.phase == GatePhase::Takeover {
                return Err(format!(
                    "gate {id} timed out, lacks a required receipt, or was crossed without an observed OPEN state"
                ));
            }
            if action.phase != GatePhase::Complete {
                directive.behavior = BehaviorKind::TollBooth;
                directive.intent.activate |= action.activate;
                directive.speed_limit = directive.speed_limit.min(action.target_speed);
                if action.target_speed == 0.0 {
                    directive.stop_distance = Some(0.0);
                } else if action.phase == GatePhase::Approach {
                    let stop = (*distance - 3.0).max(0.0);
                    directive.stop_distance =
                        Some(directive.stop_distance.map_or(stop, |old| old.min(stop)));
                }
                directive.intent.reason = format!(
                    "gate {id}: {:?} {:?}; payment observed={}",
                    gate.plan.mode, action.phase, action.payment_observed
                );
            }
        }
        Ok(())
    }
}
type Gap = Option<(f64, f64)>;
fn adjacent_gaps(graph: &LaneGraph, lane: u64, ego_s: f64, agents: &[Agent]) -> (Gap, Gap) {
    let mut front = None;
    let mut rear = None;
    for agent in agents {
        if let Ok(p) = graph.project(
            lane,
            Point3 {
                x: agent.pose.x,
                y: agent.pose.y,
                z: agent.z,
            },
        ) {
            if p.distance > graph.lanes[&lane].width / 2.0 + 0.3 || (p.z - agent.z).abs() > 3.0 {
                continue;
            }
            let delta = p.s - ego_s;
            let gap = delta.abs() - agent.length / 2.0 - 8.0;
            let slot = if delta >= 0.0 { &mut front } else { &mut rear };
            if slot.is_none_or(|(distance, _)| gap < distance) {
                *slot = Some((gap, agent.speed.max(0.0)));
            }
        }
    }
    (front, rear)
}
fn green_remaining(
    context: &TrafficContext,
    group: &str,
    extension: Option<&ExtensionFrame>,
    frame: u64,
) -> Option<f64> {
    let extension = extension.filter(|e| e.available & 8 != 0 && e.header.frame_id == frame)?;
    etsads_game::ets2la::records::<etsads_game::ets2la::Signal>(&extension.signals)
        .filter(|s| {
            s.kind == 1 && s.state == 8 && s.time_remaining.is_finite() && s.time_remaining >= 0.0
        })
        .filter_map(|s| {
            let p = etsads_game::ets2la::sector_world(s.position, s.sector);
            let p = Point3 {
                x: p[0],
                y: -p[2],
                z: p[1],
            };
            context
                .signals
                .iter()
                .filter(|head| head.id == group)
                .any(|head| head.position().distance_xy(p) < 1.5 && (head.z - p.z).abs() < 1.5)
                .then_some(s.time_remaining as f64)
        })
        .min_by(f64::total_cmp)
}
fn signal_observations(extension: Option<&ExtensionFrame>, frame: u64) -> Vec<SignalObservation> {
    let Some(extension) = extension.filter(|e| e.available & 8 != 0 && e.header.frame_id == frame)
    else {
        return vec![];
    };
    etsads_game::ets2la::records::<etsads_game::ets2la::Signal>(&extension.signals)
        .filter(|s| s.kind == 1)
        .filter_map(|s| {
            let p = etsads_game::ets2la::sector_world(s.position, s.sector);
            let position = Point3 {
                x: p[0],
                y: -p[2],
                z: p[1],
            };
            (position.valid() && s.time_remaining.is_finite() && s.time_remaining >= 0.0).then_some(
                SignalObservation {
                    position,
                    raw_state: s.state,
                    age_us: 0,
                },
            )
        })
        .collect()
}
fn barrier_state(
    extension: Option<&ExtensionFrame>,
    frame: u64,
    position: Point3,
) -> Option<BarrierState> {
    let extension = extension.filter(|e| e.available & 8 != 0 && e.header.frame_id == frame)?;
    let mut states = Vec::new();
    for signal in etsads_game::ets2la::records::<etsads_game::ets2la::Signal>(&extension.signals) {
        if signal.kind != 2 {
            continue;
        }
        let p = etsads_game::ets2la::sector_world(signal.position, signal.sector);
        let p = Point3 {
            x: p[0],
            y: -p[2],
            z: p[1],
        };
        if p.distance_xy(position) < 1.5 && (p.z - position.z).abs() < 1.5 {
            states.push(match signal.state {
                0 => BarrierState::Closing,
                1 => BarrierState::Closed,
                2 => BarrierState::Opening,
                3 => BarrierState::Open,
                _ => return None,
            });
        }
    }
    let first = *states.first()?;
    states.iter().all(|state| *state == first).then_some(first)
}
pub fn gate_barrier_state(
    extension: Option<&ExtensionFrame>,
    frame: u64,
    positions: &[Point3],
) -> Option<BarrierState> {
    let mut result = None;
    for position in positions {
        let state = barrier_state(extension, frame, *position)?;
        if result.is_some_and(|old| old != state) {
            return None;
        }
        result = Some(state);
    }
    result
}
