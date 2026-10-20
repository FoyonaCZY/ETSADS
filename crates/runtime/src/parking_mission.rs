//! Hybrid-A* parking mission execution with explicit target reference semantics,
//! stopping before each direction change, gear acknowledgements and completion.
pub(crate) use crate::parking_certificate::ParkingAdoptionProof;
use crate::parking_certificate::adoption_proof;
pub use crate::parking_recovery::{
    ParkingRecoveryConfig, ParkingRecoveryDiagnostic, ParkingRecoveryPhase,
};
use crate::parking_recovery::{RecoveryContext, RecoveryMachine, RecoveryProgress};
use etsads_control::TruckState;
use etsads_msgs::{
    BehaviorKind, Direction, EgoState, Header, OrientedBox, Pose2, TrajPoint, Trajectory,
    VehicleGeometry, frames::wrap_angle,
};
use etsads_planning::{
    geometry::{box_in_polygon, footprints, overlaps},
    parking::{self, ParkingPoint, ParkingRequest},
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedBoundary {
    pub polygon: Vec<(f64, f64)>,
    pub physical_boundaries_verified: bool,
    pub evidence_path: String,
}
impl VerifiedBoundary {
    pub fn valid(&self) -> bool {
        self.physical_boundaries_verified
            && !self.evidence_path.is_empty()
            && self.polygon.len() >= 3
            && self.polygon.len() <= 10000
            && self
                .polygon
                .iter()
                .all(|(x, y)| x.is_finite() && y.is_finite())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SlotReference {
    TractorRearAxle,
    TrailerAxle,
    TrailerReference {
        axle_offset_forward_m: f64,
        axle_offset_left_m: f64,
        evidence_path: String,
    },
    Unknown,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingTarget {
    pub source_id: String,
    pub pose: Pose2,
    pub reference: SlotReference,
}
impl ParkingTarget {
    pub fn axle_pose(&self) -> Result<Pose2, String> {
        if !self.pose.valid() || self.source_id.is_empty() {
            return Err("invalid parking target identity or pose".into());
        }
        match &self.reference {
            SlotReference::TrailerAxle | SlotReference::TractorRearAxle => Ok(self.pose),
            SlotReference::TrailerReference {
                axle_offset_forward_m: x,
                axle_offset_left_m: y,
                evidence_path,
            } => {
                if !x.is_finite()
                    || !y.is_finite()
                    || x.abs() > 20.0
                    || y.abs() > 5.0
                    || evidence_path.is_empty()
                {
                    return Err("invalid/unverified trailer reference-to-axle offset".into());
                }
                Ok(Pose2 {
                    x: self.pose.x + x * self.pose.yaw.cos() - y * self.pose.yaw.sin(),
                    y: self.pose.y + x * self.pose.yaw.sin() + y * self.pose.yaw.cos(),
                    yaw: self.pose.yaw,
                })
            }
            SlotReference::Unknown => Err(
                "company spawn target reference is unknown; cannot plan a parking manoeuvre".into(),
            ),
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingMission {
    pub target: ParkingTarget,
    pub boundary: VerifiedBoundary,
    pub static_obstacles: Vec<OrientedBox>,
    pub max_expansions: usize,
    pub maximum_duration_s: f64,
    #[serde(default)]
    pub recovery: ParkingRecoveryConfig,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PlanningIntent {
    #[serde(default)]
    pub gate: Option<Box<crate::controlled_route::GateDiagnostic>>,
    pub indicator: i8,
    pub activate: bool,
    pub gear_request: Option<Direction>,
    pub hold_brake: bool,
    pub precise_stop: bool,
    pub completed: bool,
    pub active_lane_change: bool,
    pub reason: String,
    #[serde(default)]
    pub parking_recovery: Option<ParkingRecoveryDiagnostic>,
    #[serde(default)]
    pub parking_search: Option<Box<etsads_planning::parking::ParkingSearchStatistics>>,
}
pub(crate) struct ParkingEngine {
    mission: ParkingMission,
    geometry: VehicleGeometry,
    segments: Vec<(Direction, Vec<ParkingPoint>)>,
    segment: usize,
    progress: usize,
    progress_allowance: f64,
    progress_ego: Option<EgoState>,
    start_time: Option<f64>,
    gear_requested: Option<(Direction, f64)>,
    complete: bool,
    completed_by_delivery: bool,
    recovery: Option<RecoveryMachine>,
    recovery_attempts: u8,
    pub(crate) last_recovery_diagnostic: Option<ParkingRecoveryDiagnostic>,
    cancellation: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    search_statistics: Option<Box<etsads_planning::parking::ParkingSearchStatistics>>,
}
impl ParkingEngine {
    pub(crate) fn new(mission: ParkingMission, geometry: VehicleGeometry) -> Result<Self, String> {
        mission.target.axle_pose()?;
        if geometry.has_trailer
            == matches!(mission.target.reference, SlotReference::TractorRearAxle)
        {
            return Err("parking target reference does not match tractor-only versus attached-trailer model".into());
        }
        if !mission.boundary.valid()
            || !geometry.valid()
            || mission.static_obstacles.iter().any(|o| !o.valid())
            || mission.max_expansions == 0
            || mission.max_expansions > 1_000_000
            || !mission.maximum_duration_s.is_finite()
            || !(10.0..=600.0).contains(&mission.maximum_duration_s)
            || !mission.recovery.valid()
        {
            return Err("invalid parking mission bounds, obstacles or budget".into());
        }
        Ok(Self {
            mission,
            geometry,
            segments: vec![],
            segment: 0,
            progress: 0,
            progress_allowance: 0.5,
            progress_ego: None,
            start_time: None,
            gear_requested: None,
            complete: false,
            completed_by_delivery: false,
            recovery: None,
            recovery_attempts: 0,
            last_recovery_diagnostic: None,
            cancellation: None,
            search_statistics: None,
        })
    }
    pub(crate) fn set_cancellation(
        &mut self,
        cancellation: std::sync::Arc<std::sync::atomic::AtomicBool>,
    ) {
        self.cancellation = Some(cancellation);
    }
    pub(crate) fn search_statistics(
        &self,
    ) -> Option<Box<etsads_planning::parking::ParkingSearchStatistics>> {
        self.search_statistics.clone()
    }
    fn reset_progress(&mut self) {
        self.progress = 0;
        self.progress_allowance = 0.5;
        self.progress_ego = None;
    }
    /// Advance only along the current occurrence of a segment. A global nearest
    /// point can jump across a self-intersection or reintroduce an old prefix.
    /// The arc-length allowance comes from observed travel, never elapsed time.
    fn advance_progress(&mut self, ego: EgoState) {
        let Some((_, points)) = self.segments.get(self.segment) else {
            return;
        };
        if points.is_empty() {
            return;
        }
        if let Some(previous) = self.progress_ego {
            self.progress_allowance =
                (self.progress_allowance + 1.5 * previous.pose.distance(ego.pose)).min(3.0);
        }
        self.progress_ego = Some(ego);
        self.progress = self.progress.min(points.len() - 1);
        let mut distance = points[self.progress].state.pose.distance(ego.pose);
        let mut nearest = self.progress;
        let mut used = 0.0;
        let mut arc = 0.0;
        for index in self.progress + 1..points.len() {
            arc += points[index - 1]
                .state
                .pose
                .distance(points[index].state.pose);
            if arc > self.progress_allowance + 1e-9 {
                break;
            }
            let candidate = points[index].state.pose.distance(ego.pose);
            // Equal-distance crossings retain the earlier occurrence.
            if candidate + 1e-9 < distance {
                distance = candidate;
                nearest = index;
                used = arc;
            }
        }
        self.progress = nearest;
        self.progress_allowance = (self.progress_allowance - used).max(0.0);
    }
    /// A slow search result is a geometric candidate, never a current command.
    /// Clear old one-shot action timing, then validate the entire remaining
    /// path against the new request's actual obstacle observation.
    pub(crate) fn revalidate_for_adoption(
        &mut self,
        ego: EgoState,
        obstacles: &[OrientedBox],
        occupancy: &[OrientedBox],
    ) -> Result<ParkingAdoptionProof, String> {
        if self.recovery.is_some() {
            return Err("slow recovery result cannot be adopted as a searched parking path".into());
        }
        self.advance_progress(ego);
        let count: usize = self
            .segments
            .iter()
            .skip(self.segment)
            .enumerate()
            .map(|(index, (_, p))| {
                p.len()
                    .saturating_sub(if index == 0 { self.progress } else { 0 })
            })
            .sum();
        if count > 20_000 {
            return Err("parking adoption path exceeds 20000-state validation budget".into());
        }
        let states: Vec<_> = self
            .segments
            .iter()
            .skip(self.segment)
            .enumerate()
            .flat_map(|(index, (_, p))| {
                p.iter()
                    .skip(if index == 0 { self.progress } else { 0 })
                    .map(|p| p.state)
            })
            .collect();
        if states.is_empty() && !self.complete {
            return Err("slow search produced no cached parking path".into());
        }
        if self.complete && !self.completed_by_delivery {
            let axle = TruckState::from(ego).trailer_axle(self.geometry);
            let target = self.mission.target.axle_pose()?;
            if axle.distance(target) >= 0.5
                || wrap_angle(axle.yaw - target.yaw).abs() >= 3.0_f64.to_radians()
            {
                return Err("parking completion pose changed before fresh adoption".into());
            }
        }
        validate_adoption_path(
            ego,
            &states,
            self.geometry,
            &self.mission,
            obstacles,
            occupancy,
            self.cancellation.as_deref(),
        )?;
        self.gear_requested = None;
        adoption_proof(
            ego,
            &states,
            self.geometry,
            obstacles,
            occupancy,
            self.cancellation.as_deref(),
        )
    }
    pub(crate) fn update(
        &mut self,
        ego: EgoState,
        gear: i32,
        delivered: bool,
        obstacles: &[OrientedBox],
        predicted_occupancy: &[OrientedBox],
    ) -> Result<(Trajectory, PlanningIntent), String> {
        let time = ego.header.sim_time_us as f64 / 1e6;
        let start = *self.start_time.get_or_insert(time);
        if time - start > self.mission.maximum_duration_s {
            return Err("parking duration exceeded; stop and replan".into());
        }
        if delivered {
            self.complete = true;
            self.completed_by_delivery = true;
        }
        let state = TruckState::from(ego);
        let target = self.mission.target.axle_pose()?;
        let axle = state.trailer_axle(self.geometry);
        if self.complete {
            return Ok((
                hold_trajectory(ego),
                PlanningIntent {
                    completed: true,
                    hold_brake: true,
                    reason: "job.delivered or verified parking pose reached".into(),
                    ..Default::default()
                },
            ));
        }
        if footprints(state, self.geometry).iter().any(|b| {
            !box_in_polygon(*b, &self.mission.boundary.polygon)
                || obstacles
                    .iter()
                    .chain(&self.mission.static_obstacles)
                    .any(|o| overlaps(*b, *o, 0.0))
        }) {
            return Err(
                "parking truck envelope intersects an obstacle or leaves verified boundary".into(),
            );
        }
        if let Some(recovery) = &mut self.recovery {
            let mut occupancy = self.mission.static_obstacles.clone();
            occupancy.extend_from_slice(predicted_occupancy);
            let result = recovery.update(RecoveryContext {
                ego,
                gear,
                geometry: self.geometry,
                boundary: &self.mission.boundary,
                occupancy: &occupancy,
            });
            self.last_recovery_diagnostic = Some(recovery.diagnostic(ego));
            match result? {
                RecoveryProgress::Control(trajectory, intent) => return Ok((trajectory, intent)),
                RecoveryProgress::ReadyToReplan => {
                    self.recovery = None;
                    self.segments.clear();
                    self.segment = 0;
                    self.reset_progress();
                    self.gear_requested = None;
                    self.last_recovery_diagnostic.as_mut().unwrap().phase =
                        ParkingRecoveryPhase::Resuming;
                }
            }
        }
        if self.segments.is_empty() {
            if ego.speed.abs() > 0.05 {
                return Ok((
                    hold_trajectory(ego),
                    PlanningIntent {
                        hold_brake: true,
                        reason: "stop before parking search".into(),
                        ..Default::default()
                    },
                ));
            }
            if footprints(state, self.geometry)
                .iter()
                .any(|b| predicted_occupancy.iter().any(|o| overlaps(*b, *o, 0.5)))
            {
                return Ok((
                    hold_trajectory(ego),
                    PlanningIntent {
                        hold_brake: true,
                        reason: "wait for incoming obstacle before parking search".into(),
                        ..Default::default()
                    },
                ));
            }
            let mut combined = self.mission.static_obstacles.clone();
            combined.extend_from_slice(predicted_occupancy);
            let request = ParkingRequest {
                start: state,
                target,
                geometry: self.geometry,
                obstacles: &combined,
                drivable: &self.mission.boundary.polygon,
                max_expansions: self.mission.max_expansions,
                position_tolerance: 0.25,
                angle_tolerance: 1.5_f64.to_radians(),
            };
            self.search_statistics = None;
            let plan = if let Some(cancelled) = &self.cancellation {
                parking::plan_cancellable(request, cancelled)
            } else {
                parking::plan(request)
            }
            .map_err(|e| e.to_string())?;
            self.search_statistics = Some(Box::new(plan.search.clone()));
            for point in &plan.points[1..] {
                if self
                    .segments
                    .last()
                    .is_none_or(|(direction, _)| *direction != point.direction)
                {
                    let previous = self
                        .segments
                        .last()
                        .and_then(|(_, points)| points.last())
                        .copied()
                        .unwrap_or(plan.points[0]);
                    self.segments.push((point.direction, vec![previous]));
                }
                self.segments.last_mut().unwrap().1.push(*point);
            }
            if self.segments.is_empty() {
                self.complete = true;
                return self.update(ego, gear, delivered, obstacles, predicted_occupancy);
            }
        }
        self.advance_progress(ego);
        let (direction, points) = &self.segments[self.segment];
        let end = points.last().unwrap().state;
        let remaining_arc: f64 = points[self.progress..]
            .windows(2)
            .map(|pair| pair[0].state.pose.distance(pair[1].state.pose))
            .sum();
        if remaining_arc < 0.5 && state.pose.distance(end.pose) < 0.35 && ego.speed.abs() < 0.05 {
            if self.segment + 1 < self.segments.len() {
                self.segment += 1;
                self.reset_progress();
                self.gear_requested = None;
                return self.update(ego, gear, delivered, obstacles, predicted_occupancy);
            }
            if axle.distance(target) < 0.5
                && wrap_angle(axle.yaw - target.yaw).abs() < 3.0_f64.to_radians()
            {
                self.complete = true;
                return self.update(ego, gear, delivered, obstacles, predicted_occupancy);
            }
            return Err("parking stopped outside delivery tolerances; replan required".into());
        }
        let acknowledged = match direction {
            Direction::Forward => gear > 0,
            Direction::Reverse => gear < 0,
        };
        if !acknowledged {
            if ego.speed.abs() > 0.05 {
                return Ok((
                    hold_trajectory(ego),
                    PlanningIntent {
                        hold_brake: true,
                        reason: "braking to zero before gear change".into(),
                        ..Default::default()
                    },
                ));
            }
            let mut request = None;
            if self
                .gear_requested
                .is_none_or(|(desired, _)| desired != *direction)
            {
                self.gear_requested = Some((*direction, time));
                request = Some(*direction);
            } else if self
                .gear_requested
                .is_some_and(|(_, since)| time - since > 3.0)
            {
                return Err("parking gear change was not acknowledged within 3 seconds".into());
            }
            return Ok((
                hold_trajectory(ego),
                PlanningIntent {
                    gear_request: request,
                    hold_brake: true,
                    reason: "waiting for measured gear acknowledgement".into(),
                    ..Default::default()
                },
            ));
        }
        let trajectory = segment_trajectory(
            &points[self.progress.saturating_sub(1)..],
            *direction,
            self.geometry,
            ego.header,
            3.0 / 3.6,
        );
        let nearest = self.progress;
        if self.geometry.has_trailer
            && wrap_angle(points[nearest].state.articulation() - state.articulation()).abs()
                > 15.0_f64.to_radians()
        {
            if self.recovery_attempts >= self.mission.recovery.maximum_attempts {
                self.recovery = Some(RecoveryMachine::failed(
                    self.mission.recovery,
                    ego,
                    self.recovery_attempts,
                    "parking articulation recovery attempt limit exhausted; explicit operator review required",
                ));
            } else {
                self.recovery_attempts += 1;
                self.recovery = Some(RecoveryMachine::new(
                    self.mission.recovery,
                    ego,
                    self.recovery_attempts,
                ));
            }
            self.gear_requested = None;
            return self.update(ego, gear, delivered, obstacles, predicted_occupancy);
        }
        let mut previous = footprints(state, self.geometry);
        for point in points.iter().skip(nearest).take(15) {
            let current = footprints(point.state, self.geometry);
            if current
                .iter()
                .any(|b| predicted_occupancy.iter().any(|o| overlaps(*b, *o, 0.5)))
                || previous.iter().zip(&current).any(|(a, b)| {
                    predicted_occupancy
                        .iter()
                        .any(|o| etsads_planning::geometry::swept_collision(*a, *b, *o, 0.5))
                })
            {
                return Ok((
                    hold_trajectory(ego),
                    PlanningIntent {
                        hold_brake: true,
                        reason: "new obstacle in parking sweep".into(),
                        ..Default::default()
                    },
                ));
            }
            previous = current;
        }
        Ok((
            trajectory,
            PlanningIntent {
                parking_recovery: self.last_recovery_diagnostic.clone(),
                reason: format!(
                    "parking segment {}/{} {:?}",
                    self.segment + 1,
                    self.segments.len(),
                    direction
                ),
                ..Default::default()
            },
        ))
    }
}
/// Fresh validation on both the worker and controlling thread. Occupancy is
/// deliberately supplied by the current frame, not the original search input.
pub(crate) fn validate_adoption_path(
    ego: EgoState,
    states: &[TruckState],
    geometry: VehicleGeometry,
    mission: &ParkingMission,
    obstacles: &[OrientedBox],
    occupancy: &[OrientedBox],
    cancelled: Option<&std::sync::atomic::AtomicBool>,
) -> Result<(), String> {
    if ego.speed.abs() > 0.05 {
        return Err("parking adoption requires measured standstill".into());
    }
    if states.len() > 20_000
        || states.iter().any(|s| {
            !s.valid()
                || s.steer.abs() > geometry.max_steer
                || geometry.has_trailer && s.articulation().abs() > geometry.max_articulation
        })
    {
        return Err("parking adoption path budget or kinematics invalid".into());
    }
    let mut previous = footprints(TruckState::from(ego), geometry);
    for boxes in std::iter::once(previous.clone())
        .chain(states.iter().map(|state| footprints(*state, geometry)))
    {
        if cancelled.is_some_and(|c| c.load(std::sync::atomic::Ordering::Acquire)) {
            return Err("parking adoption cancelled".into());
        }
        if boxes.iter().any(|body| {
            !box_in_polygon(*body, &mission.boundary.polygon)
                || mission
                    .static_obstacles
                    .iter()
                    .chain(obstacles)
                    .chain(occupancy)
                    .any(|o| overlaps(*body, *o, 0.3))
        }) || previous.iter().zip(&boxes).any(|(a, b)| {
            !etsads_planning::geometry::swept_in_polygon(*a, *b, &mission.boundary.polygon)
                || mission
                    .static_obstacles
                    .iter()
                    .chain(obstacles)
                    .chain(occupancy)
                    .any(|o| etsads_planning::geometry::swept_collision(*a, *b, *o, 0.3))
        }) {
            return Err("fresh parking candidate sweep intersects an obstacle or boundary".into());
        }
        previous = boxes;
    }
    Ok(())
}
pub(crate) fn hold_trajectory(ego: EgoState) -> Trajectory {
    Trajectory {
        header: ego.header,
        points: vec![
            TrajPoint {
                pose: ego.pose,
                ..Default::default()
            },
            TrajPoint {
                t: 1.0,
                pose: ego.pose,
                ..Default::default()
            },
        ],
        direction: Direction::Forward,
        behavior: BehaviorKind::Stop,
    }
}
pub(crate) fn segment_trajectory(
    points: &[ParkingPoint],
    direction: Direction,
    geometry: VehicleGeometry,
    header: Header,
    maximum_speed: f64,
) -> Trajectory {
    let mut total = 0.0;
    let mut distances = vec![0.0];
    for pair in points.windows(2) {
        total += pair[0].state.pose.distance(pair[1].state.pose);
        distances.push(total);
    }
    let sign = if direction == Direction::Reverse {
        -1.0
    } else {
        1.0
    };
    Trajectory {
        header,
        direction,
        behavior: BehaviorKind::Parking,
        points: points
            .iter()
            .zip(distances)
            .map(|(p, s)| TrajPoint {
                t: s / maximum_speed,
                pose: p.state.pose,
                speed: sign * (2.0 * 0.6 * (total - s)).sqrt().min(maximum_speed),
                acceleration: if total - s < 0.7 { -0.8 * sign } else { 0.0 },
                curvature: p.state.steer.tan() / geometry.wheelbase,
                articulation: p.state.articulation(),
            })
            .collect(),
    }
}

#[cfg(test)]
mod cursor_tests {
    use super::*;
    fn engine() -> ParkingEngine {
        ParkingEngine::new(
            ParkingMission {
                target: ParkingTarget {
                    source_id: "synthetic-cursor".into(),
                    pose: Pose2 {
                        x: -18.0,
                        y: 0.0,
                        yaw: 0.0,
                    },
                    reference: SlotReference::TractorRearAxle,
                },
                boundary: VerifiedBoundary {
                    polygon: vec![
                        (-100.0, -100.0),
                        (100.0, -100.0),
                        (100.0, 100.0),
                        (-100.0, 100.0),
                    ],
                    physical_boundaries_verified: true,
                    evidence_path: "synthetic".into(),
                },
                static_obstacles: vec![],
                max_expansions: 1000,
                maximum_duration_s: 60.0,
                recovery: Default::default(),
            },
            VehicleGeometry {
                has_trailer: false,
                trailer_length: 0.0,
                trailer_wheelbase: 0.0,
                ..Default::default()
            },
        )
        .unwrap()
    }
    fn point(x: f64, y: f64) -> ParkingPoint {
        ParkingPoint {
            state: TruckState {
                pose: Pose2 { x, y, yaw: 0.0 },
                trailer_yaw: 0.0,
                speed: 0.0,
                steer: 0.0,
            },
            direction: Direction::Reverse,
        }
    }
    fn obstacle(x: f64) -> OrientedBox {
        OrientedBox {
            pose: Pose2 {
                x,
                y: 0.0,
                yaw: 0.0,
            },
            length: 1.0,
            width: 1.0,
        }
    }
    #[test]
    fn adoption_uses_observed_cursor_and_keeps_current_to_remaining_connection() {
        let mut engine = engine();
        engine.segments = vec![(
            Direction::Reverse,
            (0..=90).map(|i| point(-(i as f64) * 0.2, 0.0)).collect(),
        )];
        let mut ego = EgoState::default();
        for n in 0..=50 {
            ego.pose.x = -(n as f64) * 0.2;
            engine.advance_progress(ego);
        }
        assert_eq!(engine.progress, 50);
        // The body and remaining path are clear. A new object behind the
        // already executed prefix must not resurrect the old start segment.
        assert!(
            engine
                .revalidate_for_adoption(ego, &[obstacle(-2.0)], &[])
                .is_ok()
        );
        assert!(
            engine
                .revalidate_for_adoption(ego, &[obstacle(-15.0)], &[])
                .is_err()
        );
        // Displaced current pose must still be swept back to the first point.
        let first = engine.segments[0].1[engine.progress].state;
        ego.pose.y = 5.0;
        let connector_obstacle = OrientedBox {
            pose: Pose2 {
                x: -8.0,
                y: 2.5,
                yaw: 0.0,
            },
            length: 0.5,
            width: 0.5,
        };
        assert!(
            validate_adoption_path(
                ego,
                &[first],
                engine.geometry,
                &engine.mission,
                &[connector_obstacle],
                &[],
                None
            )
            .is_err()
        );
    }
    #[test]
    fn self_intersection_cannot_jump_to_a_future_occurrence_or_complete() {
        let mut engine = engine();
        engine.segments = vec![(
            Direction::Reverse,
            vec![
                point(0.0, 0.0),
                point(-0.2, 0.0),
                point(-1.0, 0.0),
                point(-1.0, 1.0),
                point(0.0, 1.0),
                point(0.0, 0.0),
            ],
        )];
        let mut ego = EgoState::default();
        engine.advance_progress(ego);
        assert_eq!(engine.progress, 0);
        for _ in 0..10 {
            engine.advance_progress(ego);
        }
        assert_eq!(
            engine.progress, 0,
            "time alone cannot advance path occurrence"
        );
        ego.pose.x = -0.2;
        engine.advance_progress(ego);
        assert_eq!(engine.progress, 1);
        ego.pose.x = 0.0;
        engine.advance_progress(ego);
        assert_eq!(
            engine.progress, 1,
            "cursor neither regresses nor jumps across loop"
        );
        let (_, intent) = engine
            .update(EgoState::default(), -1, false, &[], &[])
            .unwrap();
        assert!(!intent.completed);
    }
}
