//! Source-bound gate mission derivation and runtime evidence. No permission is
//! created by a static candidate; the host must authenticate interaction input.
use super::TollStop;
use crate::navigation::SourceTollStop;
use etsads_control::TruckState;
use etsads_map::{
    LaneGraph, annotations::GateCandidate, corridor::BoundCorridor, traffic::SemaphoreKind,
};
use etsads_msgs::{BarrierState, EgoState, Point3, VehicleGeometry};
pub use etsads_planning::behavior::{GateAction, GateMode, GatePhase};
use etsads_planning::geometry::{corners, footprints};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GatePlane {
    pub position: Point3,
    pub heading: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GatePlan {
    pub mode: GateMode,
    pub native_signal_group_id: Option<String>,
    pub source_lane_id: String,
    pub observation_positions: Vec<Point3>,
    pub clearance_planes: Vec<GatePlane>,
    pub requires_payment_event: bool,
    pub geometry_source: String,
    pub profile_evidence_ids: Vec<String>,
    pub source_stop_target: Option<Point3>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GateDiagnostic {
    pub id: String,
    pub source_lane_id: String,
    pub native_signal_group_id: Option<String>,
    pub mode: GateMode,
    pub barrier: Option<BarrierState>,
    pub distance_m: f64,
    pub physical_front_distance_m: f64,
    pub rear_cleared: bool,
    pub payment_required: bool,
    pub action: GateAction,
    pub geometry_source: String,
    pub profile_evidence_ids: Vec<String>,
    pub observation_positions: Vec<Point3>,
}
impl GatePlan {
    pub fn validate_signal_semantics(
        &self,
        traffic: &etsads_map::traffic::TrafficContext,
    ) -> Result<(), String> {
        if let Some(group) = &self.native_signal_group_id {
            let signals: Vec<_> = traffic.signals.iter().filter(|s| &s.id == group).collect();
            if signals.is_empty() {
                return Err("gate lacks native signal group metadata".into());
            }
            for signal in signals {
                if signal.kind == SemaphoreKind::ModelOnly {
                    continue;
                }
                if gate_mode(signal.kind)? != self.mode {
                    return Err("gate group contains contradictory semantic types".into());
                }
            }
        }
        Ok(())
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.source_lane_id.is_empty()
            || self.geometry_source.is_empty()
            || self.observation_positions.is_empty()
            || self.observation_positions.len() > 32
            || self.clearance_planes.is_empty()
            || self.clearance_planes.len() > 32
            || self.observation_positions.iter().any(|p| !p.valid())
            || self
                .clearance_planes
                .iter()
                .any(|p| !p.position.valid() || !p.heading.is_finite())
            || self
                .profile_evidence_ids
                .iter()
                .any(|id| id.len() != 64 || !id.bytes().all(|c| c.is_ascii_hexdigit()))
            || self.source_stop_target.is_some_and(|p| !p.valid())
        {
            return Err("invalid source-bound gate plan".into());
        }
        Ok(())
    }
    pub fn legacy(
        toll: &TollStop,
        graph: &LaneGraph,
        source_lane: &str,
        traffic: &etsads_map::traffic::TrafficContext,
    ) -> Result<Self, String> {
        let lane = graph
            .lanes
            .get(&toll.lane_id)
            .ok_or("reviewed toll lane is missing")?;
        let (position, heading) = lane
            .sample(toll.stop_s)
            .ok_or("reviewed toll stop cannot be sampled")?;
        let mut plan = Self {
            mode: GateMode::Manual,
            native_signal_group_id: None,
            source_lane_id: source_lane.into(),
            observation_positions: vec![toll.barrier_position],
            clearance_planes: vec![GatePlane { position, heading }],
            requires_payment_event: true,
            geometry_source: "explicit_reviewed_toll_configuration".into(),
            profile_evidence_ids: vec![],
            source_stop_target: None,
        };
        let groups: BTreeSet<_> = traffic
            .signal_lanes
            .iter()
            .filter(|link| {
                link.lane_id == source_lane
                    && traffic.signals.iter().any(|s| {
                        s.id == link.signal_id
                            && s.position().distance_xy(toll.barrier_position) <= 1.5
                            && (s.z - toll.barrier_position.z).abs() <= 1.5
                    })
            })
            .map(|link| link.signal_id.clone())
            .collect();
        if groups.len() > 1 {
            return Err("reviewed gate position matches multiple native groups".into());
        }
        if let Some(group) = groups.into_iter().next() {
            let signals: Vec<_> = traffic
                .signals
                .iter()
                .filter(|s| s.id == group && s.kind != SemaphoreKind::ModelOnly)
                .collect();
            let first = signals
                .first()
                .ok_or("reviewed gate group has no controlling semaphore")?;
            let mode = gate_mode(first.kind)?;
            if signals.iter().any(|s| gate_mode(s.kind) != Ok(mode)) {
                return Err("reviewed gate group has unresolved/contradictory types".into());
            }
            plan.mode = mode;
            plan.native_signal_group_id = Some(group);
            plan.observation_positions = signals.iter().map(|s| s.position()).collect();
            plan.profile_evidence_ids = signals
                .iter()
                .filter_map(|s| s.profile_evidence_id.clone())
                .collect();
            plan.profile_evidence_ids.sort();
            plan.profile_evidence_ids.dedup();
        }
        Ok(plan)
    }
    /// Plane clearance uses every actual tractor/trailer corner, including
    /// measured overhangs. It remains valid after regional numeric IDs change.
    pub fn clearance(&self, ego: EgoState, geometry: VehicleGeometry) -> (f64, bool) {
        let state = TruckState {
            pose: ego.pose,
            trailer_yaw: ego.trailer_yaw,
            speed: ego.speed,
            steer: ego.wheel_angle,
        };
        let bodies = footprints(state, geometry);
        let mut front = f64::INFINITY;
        let mut cleared = true;
        for plane in &self.clearance_planes {
            let values = bodies.iter().flat_map(|b| corners(*b)).map(|(x, y)| {
                (x - plane.position.x) * plane.heading.cos()
                    + (y - plane.position.y) * plane.heading.sin()
            });
            let (minimum, maximum) = values
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                });
            front = front.min(-maximum);
            cleared &= minimum > 1.0;
        }
        (front, cleared)
    }
    pub fn same_semantics(&self, other: &Self) -> bool {
        self.mode == other.mode
            && self.native_signal_group_id == other.native_signal_group_id
            && self.requires_payment_event == other.requires_payment_event
            && self.observation_positions == other.observation_positions
            && self.profile_evidence_ids == other.profile_evidence_ids
    }
}
pub fn gate_mode(kind: SemaphoreKind) -> Result<GateMode, String> {
    match kind {
        SemaphoreKind::BarrierManualTimed => Ok(GateMode::Manual),
        SemaphoreKind::BarrierAutomatic => Ok(GateMode::Automatic),
        SemaphoreKind::BarrierDistanceActivated => Ok(GateMode::Distance),
        SemaphoreKind::BarrierGas => {
            Err("route enters an unsupported gas-station interaction; it is not a toll gate".into())
        }
        _ => Err("gate type is unresolved or is not a supported barrier".into()),
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceGatePreview {
    pub source_lane_id: String,
    pub native_signal_group_id: String,
    pub mode: GateMode,
    pub barrier_station_m: f64,
    pub projected_position: Point3,
    pub heading: f64,
    pub source_stop_target_station_m: Option<f64>,
    pub geometry_only: bool,
}
pub fn preview_source_candidate(
    candidate: &GateCandidate,
    center: &[Point3],
) -> Result<SourceGatePreview, String> {
    if !candidate.semantics_resolved
        || candidate.source_lane_id.is_empty()
        || candidate.native_signal_group_id.is_empty()
    {
        return Err("gate semantics or identity are unresolved".into());
    }
    let mode = gate_mode(candidate.kind)?;
    let (projection, point) =
        etsads_map::project_polyline(0, center, candidate.position).map_err(|e| e.to_string())?;
    let stop = candidate
        .stop_target
        .map(|p| etsads_map::project_polyline(0, center, p).map(|(p, _)| p.s))
        .transpose()
        .map_err(|e| e.to_string())?;
    Ok(SourceGatePreview {
        source_lane_id: candidate.source_lane_id.clone(),
        native_signal_group_id: candidate.native_signal_group_id.clone(),
        mode,
        barrier_station_m: projection.s,
        projected_position: point,
        heading: projection.heading,
        source_stop_target_station_m: stop,
        geometry_only: true,
    })
}

/// Select only gates on actually traversed portions of the GPS lane corridor.
/// Native signal-to-lane links select the channel; profile names never do.
pub fn derive_native_gates(
    graph: &LaneGraph,
    source_ids: &BTreeMap<u64, String>,
    corridor: &BoundCorridor,
    candidates: &[GateCandidate],
    reviewed: &[SourceTollStop],
) -> Result<Vec<TollStop>, String> {
    let reverse: BTreeMap<_, _> = source_ids.iter().map(|(id, s)| (s.as_str(), *id)).collect();
    type CandidateProjection<'a> = (&'a GateCandidate, u64, f64, f64, usize);
    let mut groups: BTreeMap<&str, Vec<CandidateProjection<'_>>> = BTreeMap::new();
    for candidate in candidates {
        let Some((route_index, step)) = corridor
            .native
            .steps
            .iter()
            .enumerate()
            .find(|(_, s)| s.source_lane_id == candidate.source_lane_id)
        else {
            continue;
        };
        let id = *reverse
            .get(candidate.source_lane_id.as_str())
            .ok_or("gate route lane missing from current graph")?;
        let projected = graph
            .project(id, candidate.position)
            .map_err(|e| e.to_string())?;
        if projected.s + 0.01 < step.entry_s_m || projected.s > step.exit_s_m + 0.01 {
            continue;
        }
        let preview = preview_source_candidate(candidate, &graph.lanes[&id].center)?;
        if (preview.barrier_station_m - projected.s).abs() > 1e-9 {
            return Err("source and runtime gate projections disagree".into());
        }
        if !candidate.semantics_resolved || candidate.native_signal_group_id.is_empty() {
            return Err("route gate has unresolved static semantics".into());
        }
        match candidate.kind {
            SemaphoreKind::BarrierManualTimed
            | SemaphoreKind::BarrierAutomatic
            | SemaphoreKind::BarrierDistanceActivated => {}
            SemaphoreKind::BarrierGas => {
                return Err(
                    "route enters an unsupported gas-station interaction; it is not a toll gate"
                        .into(),
                );
            }
            _ => return Err("route gate type is unknown or unsupported".into()),
        }
        groups
            .entry(&candidate.native_signal_group_id)
            .or_default()
            .push((candidate, id, projected.s, projected.distance, route_index));
    }
    if groups.len() > 128 {
        return Err("regional gate mission budget exceeded".into());
    }
    let mut output = Vec::new();
    let mut used_reviewed = BTreeSet::new();
    for (group, mut rows) in groups {
        rows.sort_by(|a, b| {
            a.3.total_cmp(&b.3)
                .then(a.4.cmp(&b.4))
                .then(a.0.source_lane_id.cmp(&b.0.source_lane_id))
        });
        let kind = rows[0].0.kind;
        if rows.iter().any(|r| r.0.kind != kind) {
            return Err("same native gate group declares contradictory gate modes".into());
        }
        let mode = gate_mode(kind)?;
        let matches: Vec<_> = reviewed
            .iter()
            .enumerate()
            .filter(|(_, r)| {
                rows.iter().any(|row| {
                    r.source_lane_id == row.0.source_lane_id
                        && r.barrier_position.distance_xy(row.0.position) <= 1.5
                        && (r.barrier_position.z - row.0.position.z).abs() <= 1.5
                })
            })
            .collect();
        if matches.len() > 1 {
            return Err("duplicate/ambiguous reviewed gate overrides".into());
        }
        let anchor_row = matches
            .first()
            .and_then(|(_, review)| {
                rows.iter()
                    .find(|row| row.0.source_lane_id == review.source_lane_id)
            })
            .copied()
            .unwrap_or(rows[0]);
        let (anchor, id, mut station, _, _) = anchor_row;
        let mut positions = rows.iter().map(|r| r.0.position).collect::<Vec<_>>();
        positions.sort_by(|a, b| {
            a.x.total_cmp(&b.x)
                .then(a.y.total_cmp(&b.y))
                .then(a.z.total_cmp(&b.z))
        });
        positions.dedup();
        let mut proof_ids = rows
            .iter()
            .filter_map(|r| r.0.profile_evidence_id.clone())
            .collect::<Vec<_>>();
        proof_ids.sort();
        proof_ids.dedup();
        let mut planes = Vec::new();
        for position in &positions {
            let p = graph.project(id, *position).map_err(|e| e.to_string())?;
            let (position, heading) = graph.lanes[&id]
                .sample(p.s)
                .ok_or("gate plane has no lane geometry")?;
            planes.push(GatePlane { position, heading });
        }
        let mut name = format!("native_gate:{group}");
        let mut requires_payment = false;
        if let Some((index, review)) = matches.first() {
            if !review.stop_s.is_finite()
                || review.stop_s < 0.0
                || review.stop_s > graph.lanes[&id].length()
                || review.id.is_empty()
                || (graph
                    .project(id, review.barrier_position)
                    .map_err(|e| e.to_string())?
                    .s
                    - review.stop_s)
                    .abs()
                    > 1.5
            {
                return Err("invalid reviewed gate reference".into());
            }
            station = review.stop_s;
            name = review.id.clone();
            requires_payment = true;
            used_reviewed.insert(*index);
        }
        let plan = GatePlan {
            mode,
            native_signal_group_id: Some(group.into()),
            source_lane_id: anchor.source_lane_id.clone(),
            observation_positions: positions,
            clearance_planes: planes,
            requires_payment_event: requires_payment,
            geometry_source:
                "native_semaphore_projected_to_controlled_lane; stop proxy retained separately"
                    .into(),
            profile_evidence_ids: proof_ids,
            source_stop_target: anchor.stop_target,
        };
        plan.validate()?;
        output.push(TollStop {
            id: name,
            lane_id: id,
            barrier_position: anchor.position,
            stop_s: station,
            gate: Some(plan),
        });
    }
    for (index, review) in reviewed.iter().enumerate() {
        if used_reviewed.contains(&index) {
            continue;
        }
        if let Some(id) = reverse.get(review.source_lane_id.as_str())
            && corridor.native.steps.iter().any(|s| {
                s.source_lane_id == review.source_lane_id
                    && review.stop_s >= s.entry_s_m - 0.01
                    && review.stop_s <= s.exit_s_m + 0.01
            })
        {
            // Legacy reviewed interactions remain usable. They never suppress
            // an unresolved native signal's separate conservative stop.
            output.push(TollStop {
                id: review.id.clone(),
                lane_id: *id,
                barrier_position: review.barrier_position,
                stop_s: review.stop_s,
                gate: None,
            });
        }
    }
    output.sort_by(|a, b| a.id.cmp(&b.id));
    if output.windows(2).any(|w| w[0].id == w[1].id) {
        return Err("duplicate gate mission identity".into());
    }
    Ok(output)
}
