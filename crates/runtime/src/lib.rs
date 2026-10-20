//! Driving-stack orchestration. Output is advisory until the independent
//! supervisor grants the hardware gate; this crate never opens that gate.
pub mod control_regression;
pub mod controlled_route;
pub mod following;
pub mod lateral;
pub mod live_experiment;
pub mod live_road;
pub use live_road::VerifiedRoadOperation;
pub mod navigation;
pub mod navigation_regression;
pub use lateral::{ExecutedLateralController, ForwardLateralMode, LateralControlDiagnostic};
pub use live_experiment::{LiveExperimentKind, VerifiedLiveExperiment};
pub use navigation::{NavigationDiagnostic, NavigationRegion, RouteSource};
mod parking_async;
mod parking_certificate;
pub mod parking_mission;
pub use parking_async::{ParkingPlanningDiagnostic, ParkingPlanningPhase};
mod parking_obstacles;
pub mod parking_recovery;
#[cfg(test)]
mod prediction_regression;
pub mod recovery_regression;
pub mod replay;
mod road_boundary;
use controlled_route::{ControlledRouteMission, RouteEngine};
use etsads_control::{
    Controller, ControllerConfig, TruckState,
    identification::{VehicleCalibration, geometry_fingerprint, validate_reference_geometry},
};
use etsads_game::adapter::GameSnapshot;
use etsads_map::{
    LaneGraph,
    importer::RegionGraph,
    matcher::{LaneState, Matcher},
};
use etsads_msgs::{
    Actuation, Agent, BehaviorKind, Direction, EgoState, Header, Point3, Pose2, TrajPoint,
    Trajectory, frames::wrap_angle,
};
use etsads_planning::{
    frenet::{self, FrenetRequest},
    geometry::{box_in_polygon, footprints},
    speed::{IdmConfig, curve_speed_limit},
};
use following::{FollowingDiagnostic, FollowingQuality, select_lead};
use parking_mission::{ParkingEngine, ParkingMission, PlanningIntent};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionMode {
    Live,
    Replay,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum DrivingMode {
    /// Explicit bounded commissioning area; it is not a navigation mode.
    CommissioningCircle {
        radius_m: f64,
        speed_mps: f64,
        clockwise: bool,
        verified_clear_area: Vec<(f64, f64)>,
    },
    /// Only centreline following and ACC. Turns, lane changes and parking are
    /// not enabled by this runtime capability.
    LaneFollow {
        route: Vec<u64>,
        maximum_speed_mps: f64,
    },
    Parking {
        mission: ParkingMission,
    },
    ControlledRoute {
        mission: ControlledRouteMission,
    },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RuntimeConfig {
    pub execution: ExecutionMode,
    pub vehicle_id: String,
    pub vehicle_configuration_verified: bool,
    pub mode: DrivingMode,
    #[serde(default)]
    pub forward_lateral: ForwardLateralMode,
    #[serde(default)]
    pub route_source: RouteSource,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RuntimeStatus {
    WaitingForSecondFrame,
    WaitingForTrajectory,
    WaitingForParkingPlan,
    Paused,
    Running,
    Fault,
    Completed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RuntimeOutput {
    pub header: Header,
    /// Normalized tractor rear-axle state used by matching, planning and control.
    pub control_ego: EgoState,
    pub request_control: bool,
    pub status: RuntimeStatus,
    pub reason: String,
    pub actuation: Actuation,
    pub lane: Option<LaneState>,
    pub trajectory: Option<Trajectory>,
    pub planned_this_frame: bool,
    pub planning_seconds: f64,
    pub supported_capabilities: Vec<String>,
    pub intent: PlanningIntent,
    #[serde(default)]
    pub following: Option<FollowingDiagnostic>,
    #[serde(default)]
    pub following_unavailable_reason: Option<String>,
    #[serde(default)]
    pub lateral_control: Option<LateralControlDiagnostic>,
    #[serde(default)]
    pub navigation: Option<NavigationDiagnostic>,
    /// Temporal traffic prediction used by this planning update. No target
    /// turn signal or route is fabricated when the source does not expose it.
    #[serde(default)]
    pub prediction: Option<etsads_prediction::PredictionDiagnostics>,
    #[serde(default)]
    pub parking_planning: Option<ParkingPlanningDiagnostic>,
}
#[derive(Debug, thiserror::Error)]
pub enum RuntimeError {
    #[error("runtime configuration: {0}")]
    Configuration(String),
}
struct PlanJob {
    generation: u64,
    snapshot: GameSnapshot,
    lane: Option<LaneState>,
    context: Option<PlanContext>,
    parking: Option<parking_async::ParkingJobTag>,
    cancellation: std::sync::Arc<std::sync::atomic::AtomicBool>,
    #[cfg(test)]
    delay: std::time::Duration,
}
#[derive(Clone)]
struct PlanContext {
    mode: DrivingMode,
    region: RegionGraph,
    terminal: Option<etsads_map::corridor::BoundCorridor>,
    road_boundary: Option<navigation::RoadBoundary>,
}
struct PlanResult {
    generation: u64,
    result: Result<Trajectory, String>,
    acceleration: f64,
    seconds: f64,
    intent: PlanningIntent,
    prediction: Option<etsads_prediction::PredictionDiagnostics>,
    parking: Option<parking_async::ParkingJobTag>,
    parking_path: Option<parking_mission::ParkingAdoptionProof>,
}
pub struct Runtime {
    config: RuntimeConfig,
    calibration: VehicleCalibration,
    region: Option<RegionGraph>,
    planning_graph: Option<LaneGraph>,
    controller: Controller,
    mpc: lateral::MpcTracker,
    lateral_diagnostic: Option<LateralControlDiagnostic>,
    matcher: Matcher,
    last_time: Option<u64>,
    last_frame: Option<u64>,
    last_plan: Option<u64>,
    latest: Option<Trajectory>,
    target_acceleration: f64,
    circle_center: Option<(f64, f64)>,
    fault: Option<String>,
    session_start_qpc: Option<u64>,
    planner_tx: Option<std::sync::mpsc::SyncSender<PlanJob>>,
    planner_rx: Option<std::sync::mpsc::Receiver<PlanResult>>,
    last_request: Option<u64>,
    first_request: Option<u64>,
    generation: u64,
    parking: Option<ParkingEngine>,
    intent: PlanningIntent,
    route_engine: Option<RouteEngine>,
    pending_events: Vec<etsads_msgs::protocol::GameEvent>,
    mission_frame: Option<u64>,
    navigation: Option<navigation::NativeNavigation>,
    native_terminal: Option<etsads_map::corridor::BoundCorridor>,
    planner_context_pending: bool,
    live_navigation_scope: Option<[u8; 32]>,
    live_road_scope: Option<[u8; 32]>,
    road_boundary: Option<navigation::RoadBoundary>,
    road_previous: Option<TruckState>,
    prediction: etsads_prediction::Predictor,
    prediction_diagnostic: Option<etsads_prediction::PredictionDiagnostics>,
    interactions_authorized: bool,
    parking_async: parking_async::ParkingAsyncState,
    planner_cancel: std::sync::Arc<std::sync::atomic::AtomicBool>,
    #[cfg(test)]
    parking_test_delay: std::time::Duration,
    #[cfg(test)]
    parking_test_validation_delay: std::time::Duration,
}
impl Runtime {
    pub fn new(
        config: RuntimeConfig,
        calibration: VehicleCalibration,
        region: Option<RegionGraph>,
    ) -> Result<Self, RuntimeError> {
        Self::new_internal(config, calibration, region, None, None)
    }
    /// Host-verified, supervised experiment entry; does not grant the safety
    /// gate, skip calibration checks or certify any development stage.
    pub fn new_live_experiment(
        config: RuntimeConfig,
        calibration: VehicleCalibration,
        region: Option<RegionGraph>,
        experiment: VerifiedLiveExperiment,
    ) -> Result<Self, RuntimeError> {
        experiment.validates(&config, &calibration, region.as_ref())?;
        Self::new_internal(config, calibration, region, Some(experiment), None)
    }
    pub fn new_live_road(
        config: RuntimeConfig,
        calibration: VehicleCalibration,
        region: Option<RegionGraph>,
        road: VerifiedRoadOperation,
        experiment: Option<VerifiedLiveExperiment>,
    ) -> Result<Self, RuntimeError> {
        road.validates(&config, &calibration)?;
        if let Some(experiment) = &experiment {
            experiment.validates(&config, &calibration, region.as_ref())?;
        }
        Self::new_internal(config, calibration, region, experiment, Some(road))
    }
    fn new_internal(
        config: RuntimeConfig,
        calibration: VehicleCalibration,
        region: Option<RegionGraph>,
        experiment: Option<VerifiedLiveExperiment>,
        road: Option<VerifiedRoadOperation>,
    ) -> Result<Self, RuntimeError> {
        let invalid = |s: &str| RuntimeError::Configuration(s.into());
        let interactions_authorized =
            config.execution == ExecutionMode::Replay || experiment.is_some();
        let native_gps = matches!(config.route_source, RouteSource::NativeGps(_));
        if native_gps
            && !matches!(
                config.mode,
                DrivingMode::LaneFollow { .. } | DrivingMode::ControlledRoute { .. }
            )
        {
            return Err(invalid(
                "native GPS route source is not valid for this driving mode",
            ));
        }
        if !config.forward_lateral.valid() {
            return Err(invalid("invalid forward lateral controller configuration"));
        }
        if let RouteSource::NativeGps(settings) = &config.route_source
            && settings.road_evidence.is_some()
        {
            if config.execution == ExecutionMode::Live && road.is_none() {
                return Err(invalid(
                    "live road operation requires host-verified map, calibration and road measurement evidence",
                ));
            }
            let range = calibration
                .road_speed_range()
                .map_err(|e| invalid(&e.to_string()))?;
            let requested = match &config.mode {
                DrivingMode::LaneFollow {
                    maximum_speed_mps, ..
                } => *maximum_speed_mps,
                DrivingMode::ControlledRoute { mission } => mission.maximum_speed_mps,
                _ => return Err(invalid("road-region mode requires a route")),
            };
            if !requested.is_finite() || requested < range[0] || requested > range[1] {
                return Err(invalid(
                    "road target speed is outside the recorded dynamic calibration range",
                ));
            }
        }
        if config.execution == ExecutionMode::Live {
            calibration
                .validate_measured_envelope()
                .map_err(|e| invalid(&e.to_string()))?;
        }
        if config.execution == ExecutionMode::Live
            && matches!(
                config.mode,
                DrivingMode::ControlledRoute { .. } | DrivingMode::Parking { .. }
            )
            && experiment.is_none()
        {
            return Err(invalid(
                "advanced live missions require the explicit host-verified experiment entry, source recording evidence and independent supervisor",
            ));
        }
        if !config.vehicle_configuration_verified
            || config.vehicle_id != calibration.vehicle_id
            || !calibration.accepted_for_control
            || calibration.schema_version != 1
            || !calibration.geometry.valid()
        {
            return Err(invalid(
                "vehicle identity, measured geometry and accepted calibration are mandatory",
            ));
        }
        if calibration
            .delay_s
            .is_none_or(|d| !d.is_finite() || !(0.0..=1.0).contains(&d))
            || calibration.longitudinal.is_none()
        {
            return Err(invalid(
                "missing identified steering delay or longitudinal model",
            ));
        }
        if config.execution == ExecutionMode::Live
            && calibration.evidence != "recorded game telemetry"
        {
            return Err(invalid(
                "live mode requires real recorded calibration evidence",
            ));
        }
        if config.execution == ExecutionMode::Live && calibration.geometry_fingerprint.is_none() {
            return Err(invalid(
                "live calibration lacks a measured /config geometry fingerprint",
            ));
        }
        if config.execution == ExecutionMode::Live
            && calibration.steering_axis_source != "semantic_applied"
        {
            return Err(invalid(
                "live steering calibration must use recorded applied semantic SDK axes",
            ));
        }
        if config.execution == ExecutionMode::Live
            && calibration.reference.as_ref().is_none_or(|r| {
                !r.validated || !r.sdk_to_rear_axle_scs.valid() || r.evidence_path.is_empty()
            })
        {
            return Err(invalid(
                "live control requires a measured SDK reference-to-rear-axle transform",
            ));
        }
        if config.execution == ExecutionMode::Live
            && calibration
                .conditions
                .as_ref()
                .is_none_or(|conditions| !conditions.valid(calibration.geometry))
        {
            return Err(invalid(
                "live control requires recorded trailer-count and cargo-load applicability",
            ));
        }
        let mut planning_graph = region.as_ref().map(|r| r.imported.graph.clone());
        let mut parking = None;
        let mut route_engine = None;
        match &config.mode {
            DrivingMode::CommissioningCircle {
                radius_m,
                speed_mps,
                verified_clear_area,
                ..
            } => {
                if !radius_m.is_finite()
                    || !speed_mps.is_finite()
                    || *radius_m < 15.0
                    || *radius_m > 2000.0
                    || *speed_mps <= 0.0
                    || *speed_mps > 30.0 / 3.6
                    || speed_mps * speed_mps / radius_m > 1.2
                    || verified_clear_area.len() < 3
                    || verified_clear_area
                        .iter()
                        .any(|(x, y)| !x.is_finite() || !y.is_finite())
                {
                    return Err(invalid(
                        "invalid commissioning circle or unverified test area",
                    ));
                }
            }
            DrivingMode::LaneFollow {
                route,
                maximum_speed_mps,
            } => {
                let Some(region) = &region else {
                    return Err(invalid("LaneFollow requires an explicitly verified region"));
                };
                if !maximum_speed_mps.is_finite()
                    || *maximum_speed_mps <= 0.0
                    || *maximum_speed_mps > region.policy.maximum_speed_mps
                    || (!native_gps && route.is_empty())
                {
                    return Err(invalid("route and explicit regional speed limit required"));
                }
                let graph = planning_graph.as_mut().unwrap();
                for id in route {
                    if !graph.lanes.contains_key(id) {
                        return Err(invalid("route leaves approved map region"));
                    }
                    if region
                        .imported
                        .runtime_to_source
                        .get(id)
                        .is_none_or(|source| !source.contains(":road:"))
                    {
                        return Err(invalid(
                            "LaneFollow cannot enter prefab junctions or unknown lane kinds; signal/priority integration is required",
                        ));
                    }
                }
                for pair in route.windows(2) {
                    if !graph.lanes[&pair[0]].successors.contains(&pair[1]) {
                        return Err(invalid(
                            "route requires an unsupported lane change or disconnected manoeuvre",
                        ));
                    }
                    graph.lanes.get_mut(&pair[0]).unwrap().successors = vec![pair[1]];
                }
                if let Some(last) = route.last() {
                    graph.lanes.get_mut(last).unwrap().successors.clear();
                }
            }
            DrivingMode::Parking { mission } => {
                parking = Some(
                    ParkingEngine::new(mission.clone(), calibration.geometry)
                        .map_err(|e| invalid(&e))?,
                );
            }
            DrivingMode::ControlledRoute { mission } => {
                let Some(region) = &region else {
                    return Err(invalid("controlled route requires a verified map region"));
                };
                if mission.maximum_speed_mps > region.policy.maximum_speed_mps {
                    return Err(invalid(
                        "controlled route speed exceeds regional permission",
                    ));
                }
                if config.execution == ExecutionMode::Live
                    && (!mission.live_manoeuvres_verified
                        || mission.control_evidence_path.is_empty()
                        || !mission.boundary.valid())
                {
                    return Err(invalid(
                        "live controlled route requires explicit manoeuvre and physical-boundary evidence",
                    ));
                }
                let graph = planning_graph.as_mut().unwrap();
                if !native_gps {
                    route_engine = Some(
                        RouteEngine::new(
                            mission.clone(),
                            graph,
                            region.imported.runtime_to_source.clone(),
                            calibration.geometry,
                            interactions_authorized,
                        )
                        .map_err(|e| invalid(&e))?,
                    );
                }
                for pair in mission.route.windows(2) {
                    if graph.lanes[&pair[0]].successors.contains(&pair[1]) {
                        graph.lanes.get_mut(&pair[0]).unwrap().successors = vec![pair[1]];
                    }
                }
                if let Some(last) = mission.route.last() {
                    graph.lanes.get_mut(last).unwrap().successors.clear();
                }
            }
        }
        let controller = Controller::new(ControllerConfig {
            geometry: calibration.geometry,
            delay: calibration.delay_s.unwrap(),
            ..Default::default()
        })
        .map_err(|e| invalid(&e.to_string()))?;
        let mut this = Self {
            config,
            calibration,
            region,
            planning_graph,
            controller,
            mpc: lateral::MpcTracker::default(),
            lateral_diagnostic: None,
            matcher: Matcher::default(),
            last_time: None,
            last_frame: None,
            last_plan: None,
            latest: None,
            target_acceleration: 0.0,
            circle_center: None,
            fault: None,
            session_start_qpc: None,
            planner_tx: None,
            planner_rx: None,
            last_request: None,
            first_request: None,
            generation: 0,
            parking,
            intent: PlanningIntent::default(),
            route_engine,
            pending_events: vec![],
            mission_frame: None,
            navigation: None,
            native_terminal: None,
            planner_context_pending: false,
            live_navigation_scope: experiment.as_ref().and_then(|e| e.navigation_scope()),
            live_road_scope: road.as_ref().map(|r| r.scope),
            road_boundary: None,
            road_previous: None,
            prediction: Default::default(),
            prediction_diagnostic: None,
            interactions_authorized,
            parking_async: Default::default(),
            planner_cancel: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            #[cfg(test)]
            parking_test_delay: std::time::Duration::ZERO,
            #[cfg(test)]
            parking_test_validation_delay: std::time::Duration::ZERO,
        };
        if this.config.execution == ExecutionMode::Live {
            let mut worker_config = this.config.clone();
            worker_config.execution = ExecutionMode::Replay;
            let mut worker =
                Self::new(worker_config, this.calibration.clone(), this.region.clone())?;
            let (tx, requests) = std::sync::mpsc::sync_channel::<PlanJob>(1);
            let (results, rx) = std::sync::mpsc::channel();
            std::thread::Builder::new()
                .name("etsads-planner".into())
                .spawn(move || {
                    let mut generation = 0;
                    while let Ok(job) = requests.recv() {
                        if job.generation != generation {
                            worker.prediction.reset();
                            worker.prediction_diagnostic = None;
                            if let Some(context) = job.context {
                                if let Err(error) = worker.apply_plan_context(context) {
                                    let _ = results.send(PlanResult {
                                        generation: job.generation,
                                        result: Err(error),
                                        acceleration: 0.0,
                                        seconds: 0.0,
                                        intent: PlanningIntent::default(),
                                        prediction: None,
                                        parking: job.parking,
                                        parking_path: None,
                                    });
                                    continue;
                                }
                            } else {
                                worker.reset();
                            }
                            generation = job.generation;
                            worker.generation = job.generation;
                        }
                        let start = std::time::Instant::now();
                        if let Some(engine) = &mut worker.parking {
                            engine.set_cancellation(job.cancellation.clone());
                        }
                        #[cfg(test)]
                        std::thread::sleep(job.delay);
                        let mut parking_path = None;
                        let result =
                            if job.cancellation.load(std::sync::atomic::Ordering::Acquire) {
                                Err("planner request cancelled".into())
                            } else {
                                (|| {
                                    if job.parking.is_some_and(|p| {
                                        p.phase == ParkingPlanningPhase::Revalidating
                                    }) {
                                        let obstacles = parking_obstacles::collect(&job.snapshot)?;
                                        parking_path = Some(
                                            worker
                                                .parking
                                                .as_mut()
                                                .ok_or("parking engine missing")?
                                                .revalidate_for_adoption(
                                                    job.snapshot.ego,
                                                    &obstacles.current,
                                                    &obstacles.occupancy,
                                                )?,
                                        );
                                    }
                                    worker.make_plan(&job.snapshot, job.lane)
                                })()
                            };
                        if results
                            .send(PlanResult {
                                generation,
                                result,
                                acceleration: worker.target_acceleration,
                                seconds: start.elapsed().as_secs_f64(),
                                intent: worker.intent.clone(),
                                prediction: worker.prediction_diagnostic.clone(),
                                parking: job.parking,
                                parking_path,
                            })
                            .is_err()
                        {
                            break;
                        }
                    }
                })
                .map_err(|e| invalid(&format!("planner thread: {e}")))?;
            this.planner_tx = Some(tx);
            this.planner_rx = Some(rx);
        }
        Ok(this)
    }
    /// A fault latches until the caller explicitly resets/rearms after review.
    /// This operation never arms the independent supervisor itself.
    pub fn reset(&mut self) {
        self.planner_cancel
            .store(true, std::sync::atomic::Ordering::Release);
        self.planner_cancel = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.parking_async = Default::default();
        self.prediction.reset();
        self.prediction_diagnostic = None;
        self.road_previous = None;
        if let Some(navigation) = &mut self.navigation {
            navigation.gps.clear();
            navigation.corridor = None;
            navigation.consumed = 0;
            navigation.corridor_consumed_base = 0;
            navigation.dirty = true;
            navigation.diagnostic = None;
        }
        self.mpc.reset();
        self.lateral_diagnostic = None;
        self.last_time = None;
        self.last_frame = None;
        self.last_plan = None;
        self.latest = None;
        self.fault = None;
        self.circle_center = None;
        self.session_start_qpc = None;
        self.last_request = None;
        self.first_request = None;
        self.generation = self.generation.wrapping_add(1);
        self.controller.reset();
        self.matcher.reset();
        self.intent = PlanningIntent::default();
        self.pending_events.clear();
        self.mission_frame = None;
        if let DrivingMode::ControlledRoute { mission } = &self.config.mode {
            self.route_engine = self.region.as_ref().and_then(|r| {
                RouteEngine::new(
                    mission.clone(),
                    &r.imported.graph,
                    r.imported.runtime_to_source.clone(),
                    self.calibration.geometry,
                    self.interactions_authorized,
                )
                .ok()
            });
        }
        if let DrivingMode::Parking { mission } = &self.config.mode {
            self.parking = ParkingEngine::new(mission.clone(), self.calibration.geometry).ok();
        }
    }
    fn stopped(
        &mut self,
        ego: EgoState,
        status: RuntimeStatus,
        reason: impl Into<String>,
    ) -> RuntimeOutput {
        let reason = reason.into();
        if let Some(diagnostic) = self.navigation.as_mut().and_then(|n| n.diagnostic.as_mut()) {
            diagnostic.frame_id = ego.header.frame_id;
            if status == RuntimeStatus::Fault {
                diagnostic.reason = reason.clone();
            }
        }
        self.latest = None;
        self.last_plan = None;
        self.controller.reset();
        self.mpc.reset();
        if status == RuntimeStatus::Fault {
            self.fault = Some(reason.clone());
        }
        if matches!(
            status,
            RuntimeStatus::Fault | RuntimeStatus::Paused | RuntimeStatus::Completed
        ) {
            self.planner_cancel
                .store(true, std::sync::atomic::Ordering::Release);
            self.parking_async = Default::default();
        }
        RuntimeOutput {
            header: ego.header,
            control_ego: ego,
            request_control: false,
            status,
            reason,
            actuation: Actuation::default(),
            lane: None,
            trajectory: None,
            planned_this_frame: false,
            planning_seconds: 0.0,
            supported_capabilities: self.capabilities(),
            intent: self.intent.clone(),
            following: None,
            lateral_control: self.lateral_diagnostic.clone(),
            navigation: self.navigation.as_ref().and_then(|n| n.diagnostic.clone()),
            prediction: None,
            parking_planning: None,
            following_unavailable_reason: Some(
                "runtime is not producing active forward following observations".into(),
            ),
        }
    }
    pub fn capabilities(&self) -> Vec<String> {
        let mut capabilities = match self.config.mode {
            DrivingMode::CommissioningCircle { .. } => vec!["P0 bounded fixed-circle commissioning".into()],
            DrivingMode::LaneFollow { .. } => vec!["P1 verified-region centreline following".into(),
                "P1 privileged-traffic ACC".into(), "physical road boundaries may be unknown; lane changes and intersections are disabled".into()],
            DrivingMode::Parking { .. } => vec!["P4 Hybrid A* parking with gear acknowledgement and articulated feedback".into(),
                if self.config.execution == ExecutionMode::Live { "supervised development experiment; independent rearward AEB and host evidence checks required; not stage acceptance".into() }
                else { "offline replay; no live parking acceptance implied".into() }],
            DrivingMode::ControlledRoute { .. } => vec!["P2 annotated route, gap-checked lane changes and toll interactions".into(),
                "P3 spatial signal association, stop/yield conflict rules and verified truck sweep".into()],
        };
        if self.calibration.geometry.measured_envelope.is_none() {
            capabilities
                .push("synthetic body-offset fallback; no measured exterior envelope".into());
        } else if self.config.execution == ExecutionMode::Replay {
            capabilities.push("explicit body offsets in replay; no live acceptance implied".into());
        }
        if let RouteSource::NativeGps(settings) = &self.config.route_source {
            capabilities.push(if settings.road_evidence.is_some() {
                "native GPS road-region operation with moving 2 km windows; measured map and recorded dynamic speed applicability required; no 100 km acceptance implied"
            } else {
                "native GPS corridor with moving 2 km source-bound windows; commissioning policy remains limited to 30 km/h"
            }.into());
        }
        capabilities
    }
    pub fn step(&mut self, snapshot: &GameSnapshot) -> RuntimeOutput {
        self.lateral_diagnostic = None;
        let normalized;
        let snapshot = if let Some(reference) = &self.calibration.reference {
            let Some(config) = snapshot.config.as_ref() else {
                return self.stopped(
                    snapshot.ego,
                    RuntimeStatus::Fault,
                    "vehicle reference requires /config",
                );
            };
            if let Err(e) =
                validate_reference_geometry(config, reference, self.calibration.geometry)
            {
                return self.stopped(snapshot.ego, RuntimeStatus::Fault, e.to_string());
            }
            let p = snapshot.raw.position_scs;
            let turns = snapshot.raw.orientation_turns.map(|v| v as f64);
            let position = match etsads_msgs::frames::placement_offset_to_enu(
                Point3 {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                },
                turns,
                reference.sdk_to_rear_axle_scs,
            ) {
                Ok(p) => p,
                Err(e) => return self.stopped(snapshot.ego, RuntimeStatus::Fault, e),
            };
            let mut ego = snapshot.ego;
            ego.pose.x = position.x;
            ego.pose.y = position.y;
            ego.z = position.z;
            let mut scene = snapshot.scene.clone();
            scene.ego = ego;
            normalized = GameSnapshot {
                ego,
                raw: snapshot.raw,
                extension: snapshot.extension.clone(),
                config: snapshot.config,
                events: snapshot.events.clone(),
                scene,
                local_origin: snapshot.local_origin,
                origin_generation: snapshot.origin_generation,
                session_start_qpc: snapshot.session_start_qpc,
            };
            &normalized
        } else {
            snapshot
        };
        let ego = snapshot.ego;
        if let Some(fault) = self.fault.clone() {
            return self.stopped(ego, RuntimeStatus::Fault, fault);
        }
        if self.intent.completed {
            return self.stopped(ego, RuntimeStatus::Completed, self.intent.reason.clone());
        }
        if !ego.valid()
            || (ego.speed < -0.1 && !matches!(self.config.mode, DrivingMode::Parking { .. }))
        {
            return self.stopped(ego, RuntimeStatus::Fault, "invalid ego state");
        }
        if let RouteSource::NativeGps(settings) = &self.config.route_source
            && settings.road_evidence.is_some()
        {
            match self.calibration.road_speed_range() {
                Ok([min, max]) if (min..=max).contains(&ego.speed) => {}
                _ => {
                    return self.stopped(
                        ego,
                        RuntimeStatus::Fault,
                        "road speed left the recorded dynamic calibration applicability",
                    );
                }
            }
        }
        if let Some(conditions) = &self.calibration.conditions {
            let Some(config) = snapshot.config.as_ref() else {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "calibrated load range requires current /config",
                );
            };
            if let Err(e) =
                conditions.validate_observation(self.calibration.geometry, &snapshot.raw, config)
            {
                return self.stopped(ego, RuntimeStatus::Fault, e.to_string());
            }
        }
        if self.config.execution == ExecutionMode::Live
            && snapshot
                .raw
                .trailers
                .iter()
                .filter(|t| t.connected != 0)
                .count()
                > 1
        {
            return self.stopped(
                ego,
                RuntimeStatus::Fault,
                "current calibrated model supports at most one attached trailer",
            );
        }
        if let Some(expected) = &self.calibration.geometry_fingerprint {
            let actual = snapshot
                .config
                .as_ref()
                .and_then(|c| geometry_fingerprint(c).ok());
            if actual.as_ref() != Some(expected) {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "current truck geometry differs from calibration or /config is unavailable",
                );
            }
        }
        if ego.paused {
            self.last_time = None;
            self.last_frame = None;
            return self.stopped(
                ego,
                RuntimeStatus::Paused,
                "game paused; all inputs released",
            );
        }
        if let Some(session) = self.session_start_qpc {
            if snapshot.session_start_qpc != session {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "game/plugin session changed; rearm required",
                );
            }
        } else {
            self.session_start_qpc = Some(snapshot.session_start_qpc);
        }
        if self.config.execution == ExecutionMode::Live {
            let raw = &snapshot.raw;
            let now = etsads_shm::qpc();
            if raw.qpc_frequency == 0
                || raw.header.qpc > now
                || now.saturating_sub(raw.header.qpc) > raw.qpc_frequency / 5
            {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "telemetry wall-clock age exceeds 200 ms",
                );
            }
        }
        let Some(previous) = self.last_time else {
            self.last_time = Some(ego.header.sim_time_us);
            self.last_frame = Some(ego.header.frame_id);
            return self.stopped(
                ego,
                RuntimeStatus::WaitingForSecondFrame,
                "waiting for measured physical-step interval",
            );
        };
        let elapsed = ego.header.sim_time_us.checked_sub(previous);
        if elapsed.is_none_or(|dt| dt == 0 || dt > 100_000)
            || self.last_frame.is_some_and(|id| ego.header.frame_id <= id)
        {
            return self.stopped(
                ego,
                RuntimeStatus::Fault,
                "frozen, regressed or missed physical frame",
            );
        }
        let dt = elapsed.unwrap() as f64 / 1e6;
        self.last_time = Some(ego.header.sim_time_us);
        self.last_frame = Some(ego.header.frame_id);
        if snapshot.scene.agents.len() > 200 || snapshot.scene.agents.iter().any(|a| !a.valid()) {
            return self.stopped(ego, RuntimeStatus::Fault, "invalid traffic scene");
        }
        if matches!(self.config.mode, DrivingMode::Parking { .. })
            && let Err(reason) = parking_obstacles::validate_source(snapshot, self.config.execution)
        {
            return self.stopped(ego, RuntimeStatus::Fault, reason);
        }
        self.pending_events.extend_from_slice(&snapshot.events);
        if self.pending_events.len() > 256 {
            return self.stopped(ego, RuntimeStatus::Fault, "planning event queue overflow");
        }
        let mut lane = None;
        if matches!(self.config.route_source, RouteSource::NativeGps(_)) {
            match self.update_native_route(snapshot) {
                Ok(found) => lane = Some(found),
                Err(reason) => return self.stopped(ego, RuntimeStatus::Fault, reason),
            }
        }
        if let Err(reason) = self.check_road_execution(TruckState::from(ego)) {
            return self.stopped(ego, RuntimeStatus::Fault, reason);
        }
        let controlled_route = matches!(self.config.mode, DrivingMode::ControlledRoute { .. });
        let route = match &self.config.mode {
            DrivingMode::LaneFollow { route, .. } => Some(route),
            DrivingMode::ControlledRoute { mission } => Some(&mission.route),
            _ => None,
        };
        if let Some(route) = route {
            if snapshot
                .extension
                .as_ref()
                .is_none_or(|e| e.available & 2 == 0 || e.header.frame_id != ego.header.frame_id)
            {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "ACC requires fresh privileged traffic; no blind fallback",
                );
            }
            let region = self.region.as_ref().unwrap();
            let found = lane.or_else(|| {
                self.matcher.update(
                    &region.imported.graph,
                    ego.pose,
                    ego.z,
                    ego.speed,
                    ego.header.sim_time_us as f64 / 1e6,
                )
            });
            if found.is_none_or(|m| {
                ((!self.intent.active_lane_change || !controlled_route)
                    && (m.confidence < 0.6 || m.d.abs() > region.policy.maximum_lateral_error_m))
                    || !route.contains(&m.lane_id)
            }) {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "lane match confidence, regional deviation or route guard failed",
                );
            }
            lane = found;
            let limit = snapshot.raw.navigation_speed_limit_mps as f64;
            if !limit.is_finite() || limit <= 0.0 || limit > 60.0 {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "fresh telemetry speed limit unavailable",
                );
            }
        }
        if let DrivingMode::ControlledRoute { mission } = &self.config.mode
            && footprints(TruckState::from(ego), self.calibration.geometry)
                .iter()
                .any(|b| !box_in_polygon(*b, &mission.boundary.polygon))
        {
            return self.stopped(
                ego,
                RuntimeStatus::Fault,
                "truck envelope left the controlled-route physical boundary",
            );
        }
        let mut planning_seconds = 0.0;
        let mut planned_this_frame = false;
        if self.config.execution == ExecutionMode::Live
            && matches!(self.config.mode, DrivingMode::Parking { .. })
        {
            match self.poll_live_parking(snapshot) {
                Ok(progress) => {
                    if let Some(waiting) = progress.waiting {
                        return self.parking_wait_output(ego, waiting);
                    }
                    planned_this_frame = progress.planned;
                    planning_seconds = progress.seconds;
                }
                Err(reason) => return self.stopped(ego, RuntimeStatus::Fault, reason),
            }
        } else if self.config.execution == ExecutionMode::Live {
            let received: Vec<_> = self.planner_rx.as_ref().unwrap().try_iter().collect();
            for completed in received {
                if completed.generation != self.generation {
                    continue;
                }
                match completed.result {
                    Ok(trajectory) => {
                        self.last_plan = Some(trajectory.header.sim_time_us);
                        self.latest = Some(trajectory);
                        self.target_acceleration = completed.acceleration;
                        self.intent = completed.intent;
                        self.prediction_diagnostic = completed.prediction;
                        planning_seconds = completed.seconds;
                        planned_this_frame = true;
                    }
                    Err(reason) => return self.stopped(ego, RuntimeStatus::Fault, reason),
                }
            }
            if self
                .last_request
                .is_none_or(|last| ego.header.sim_time_us.saturating_sub(last) >= 100_000)
            {
                let copy = GameSnapshot {
                    ego: snapshot.ego,
                    raw: snapshot.raw,
                    extension: snapshot.extension.clone(),
                    config: None,
                    events: self.pending_events.clone(),
                    scene: snapshot.scene.clone(),
                    local_origin: snapshot.local_origin,
                    origin_generation: snapshot.origin_generation,
                    session_start_qpc: snapshot.session_start_qpc,
                };
                match self.planner_tx.as_ref().unwrap().try_send(PlanJob {
                    generation: self.generation,
                    snapshot: copy,
                    lane,
                    context: self.planner_context_pending.then(|| PlanContext {
                        mode: self.config.mode.clone(),
                        region: self.region.as_ref().unwrap().clone(),
                        terminal: self.native_terminal.clone(),
                        road_boundary: self.road_boundary.clone(),
                    }),
                    parking: None,
                    cancellation: self.planner_cancel.clone(),
                    #[cfg(test)]
                    delay: std::time::Duration::ZERO,
                }) {
                    Ok(()) => {
                        self.last_request = Some(ego.header.sim_time_us);
                        self.first_request.get_or_insert(ego.header.sim_time_us);
                        self.pending_events.clear();
                        self.planner_context_pending = false;
                    }
                    Err(std::sync::mpsc::TrySendError::Full(_)) => {}
                    Err(_) => {
                        return self.stopped(
                            ego,
                            RuntimeStatus::Fault,
                            "planner worker disconnected",
                        );
                    }
                }
            }
        } else if self
            .last_plan
            .is_none_or(|last| ego.header.sim_time_us.saturating_sub(last) >= 100_000)
        {
            let start = std::time::Instant::now();
            let copy = GameSnapshot {
                ego: snapshot.ego,
                raw: snapshot.raw,
                extension: snapshot.extension.clone(),
                config: snapshot.config,
                events: self.pending_events.clone(),
                scene: snapshot.scene.clone(),
                local_origin: snapshot.local_origin,
                origin_generation: snapshot.origin_generation,
                session_start_qpc: snapshot.session_start_qpc,
            };
            match self.make_plan(&copy, lane) {
                Ok(trajectory) => {
                    self.latest = Some(trajectory);
                    self.last_plan = Some(ego.header.sim_time_us);
                    planning_seconds = start.elapsed().as_secs_f64();
                    planned_this_frame = true;
                    self.pending_events.clear();
                }
                Err(reason) => return self.stopped(ego, RuntimeStatus::Fault, reason),
            }
        }
        let Some(trajectory) = self.latest.as_ref() else {
            return if self
                .first_request
                .is_some_and(|first| ego.header.sim_time_us.saturating_sub(first) > 200_000)
            {
                self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "initial trajectory missed 200 ms deadline",
                )
            } else {
                self.stopped(
                    ego,
                    RuntimeStatus::WaitingForTrajectory,
                    "waiting for asynchronous planner; inputs released",
                )
            };
        };
        if self.intent.completed {
            return self.stopped(ego, RuntimeStatus::Completed, self.intent.reason.clone());
        }
        if ego
            .header
            .sim_time_us
            .saturating_sub(trajectory.header.sim_time_us)
            > 200_000
        {
            return self.stopped(ego, RuntimeStatus::Fault, "trajectory older than 200 ms");
        }
        if matches!(self.config.mode, DrivingMode::Parking { .. }) && !self.intent.hold_brake {
            let measured = match trajectory.direction {
                Direction::Forward => snapshot.raw.gear > 0,
                Direction::Reverse => snapshot.raw.gear < 0,
            };
            if !measured {
                return self.stopped(
                    ego,
                    RuntimeStatus::Fault,
                    "parking motion direction no longer matches the currently measured gear",
                );
            }
        }
        let mut command = if self.intent.hold_brake {
            self.mpc.reset();
            self.lateral_diagnostic = Some(LateralControlDiagnostic::direct(
                ego.header.frame_id,
                ExecutedLateralController::Hold,
            ));
            if let Err(e) = self.controller.reset_to_steer(ego.wheel_angle) {
                return self.stopped(ego, RuntimeStatus::Fault, e.to_string());
            }
            Actuation::default()
        } else if trajectory.direction == Direction::Forward
            && let ForwardLateralMode::LtvMpc {
                lateral_acceleration_limit_mps2,
                maximum_solve_time_ms,
            } = self.config.forward_lateral
        {
            match self.mpc.track(
                &mut self.controller,
                ego,
                trajectory,
                dt,
                lateral_acceleration_limit_mps2,
                maximum_solve_time_ms,
            ) {
                Ok((command, diagnostic)) => {
                    self.lateral_diagnostic = Some(diagnostic);
                    command
                }
                Err(diagnostic) => {
                    let reason = diagnostic
                        .failure
                        .clone()
                        .unwrap_or_else(|| "MPC failed".into());
                    self.lateral_diagnostic = Some(diagnostic);
                    return self.stopped(ego, RuntimeStatus::Fault, reason);
                }
            }
        } else {
            self.mpc.reset();
            self.lateral_diagnostic = Some(LateralControlDiagnostic::direct(
                ego.header.frame_id,
                if trajectory.direction == Direction::Reverse
                    || trajectory.behavior == BehaviorKind::Parking
                {
                    ExecutedLateralController::ArticulatedLqr
                } else {
                    ExecutedLateralController::PurePursuit
                },
            ));
            match self.controller.track(ego, trajectory, dt) {
                Ok(command) => command,
                Err(e) => return self.stopped(ego, RuntimeStatus::Fault, e.to_string()),
            }
        };
        if self.intent.hold_brake {
            command = Actuation {
                steer: (ego.wheel_angle / self.calibration.geometry.max_steer).clamp(-1.0, 1.0),
                brake: 0.4,
                ..Default::default()
            };
        }
        if self.intent.gear_request.is_some()
            && let Err(e) = self.controller.reset_to_steer(ego.wheel_angle)
        {
            return self.stopped(ego, RuntimeStatus::Fault, e.to_string());
        }
        let desired_curvature = (command.steer * self.calibration.geometry.max_steer).tan()
            / self.calibration.geometry.wheelbase;
        command.steer = match self
            .calibration
            .steering_axis(desired_curvature, ego.speed.abs())
        {
            Ok(axis) => axis,
            Err(_) if self.intent.hold_brake => 0.0,
            Err(e) => return self.stopped(ego, RuntimeStatus::Fault, e.to_string()),
        };
        let grade = snapshot.raw.orientation_turns[1] as f64 * std::f64::consts::TAU;
        let requested_acceleration = if matches!(self.config.mode, DrivingMode::Parking { .. }) {
            command.throttle * 1.2 - command.brake * 4.0
        } else if self.intent.precise_stop {
            // This is a position-feedback stopping controller: a=v²/(2d),
            // recomputed from measured distance and speed. Adding speed-preview
            // PI again brakes early and prevents reaching the toll activation zone.
            self.target_acceleration
        } else {
            self.target_acceleration
                .min(command.throttle * 1.2 - command.brake * 4.0)
        };
        match self
            .calibration
            .pedals(requested_acceleration, ego.speed.abs(), grade)
        {
            Ok((throttle, brake)) => {
                command.throttle = throttle;
                command.brake = brake;
            }
            Err(e) => return self.stopped(ego, RuntimeStatus::Fault, e.to_string()),
        }
        if self.intent.hold_brake {
            command.throttle = 0.0;
            command.brake = 0.4;
        }
        command.indicator = self.intent.indicator;
        command.activate = self.intent.activate;
        if !command.valid() {
            return self.stopped(ego, RuntimeStatus::Fault, "calibrated command is invalid");
        }
        RuntimeOutput {
            header: ego.header,
            control_ego: ego,
            request_control: true,
            status: RuntimeStatus::Running,
            reason: "supervisor permission is still required".into(),
            actuation: command,
            lateral_control: self.lateral_diagnostic.clone(),
            navigation: self.navigation.as_ref().and_then(|n| n.diagnostic.clone()),
            prediction: planned_this_frame.then(|| self.prediction_diagnostic.clone()).flatten(),
            parking_planning: None,
            following: self.following_diagnostic(snapshot, ego, lane),
            following_unavailable_reason: self.calibration.measured_front_bumper().is_none().then(|| "front bumper-to-rear-axle measurement and matching geometry evidence are absent or invalid".into()),
            lane,
            trajectory: if planned_this_frame {
                self.latest.clone()
            } else {
                None
            },
            planned_this_frame,
            planning_seconds,
            supported_capabilities: self.capabilities(),
            intent: {
                let intent = self.intent.clone();
                self.intent.activate = false;
                self.intent.gear_request = None;
                intent
            },
        }
    }
    fn front_bumper_for_control(&self) -> f64 {
        // A rear axle lies within the measured tractor body. If its precise
        // front offset is unavailable, total body length is a conservative
        // control bound, not a certified physical gap measurement.
        self.calibration
            .measured_front_bumper()
            .unwrap_or(self.calibration.geometry.tractor_length)
    }
    fn following_diagnostic(
        &self,
        snapshot: &GameSnapshot,
        ego: EgoState,
        lane: Option<LaneState>,
    ) -> Option<FollowingDiagnostic> {
        let front = self.calibration.measured_front_bumper()?;
        let route = match &self.config.mode {
            DrivingMode::LaneFollow { route, .. } => route,
            DrivingMode::ControlledRoute { mission } => &mission.route,
            _ => return None,
        };
        select_lead(
            self.planning_graph.as_ref()?,
            route,
            lane?,
            ego,
            &snapshot.scene.agents,
            front,
            self.calibration.geometry.width,
        )?
        .diagnostic(
            ego,
            if self.config.execution == ExecutionMode::Live {
                FollowingQuality::VerifiedGeometry
            } else {
                FollowingQuality::SyntheticReplay
            },
        )
    }
    fn plan_controlled(
        &mut self,
        snapshot: &GameSnapshot,
        lane: LaneState,
        mission: &ControlledRouteMission,
        predictions: &[etsads_prediction::PredictedAgent],
    ) -> Result<Trajectory, String> {
        let ego = snapshot.ego;
        let graph = self
            .planning_graph
            .as_ref()
            .ok_or("controlled route graph unavailable")?;
        let mut directive = self
            .route_engine
            .as_mut()
            .ok_or("controlled route engine unavailable")?
            .update(
                graph,
                ego,
                lane,
                &snapshot.scene.agents,
                snapshot.extension.as_deref(),
                &snapshot.events,
            )?;
        if let Some(distance) = self.native_stop_distance(lane) {
            directive.stop_distance = Some(
                directive
                    .stop_distance
                    .map_or(distance, |d| d.min(distance)),
            );
            if distance <= 0.2 && ego.speed.abs() < 0.1 {
                directive.intent.hold_brake = true;
                directive.intent.reason =
                    "holding before native corridor terminal; a verified continuation is required"
                        .into();
            }
        }
        self.intent = directive.intent;
        if self.intent.hold_brake {
            self.target_acceleration = -1.6;
            return Ok(parking_mission::hold_trajectory(ego));
        }
        let reference = graph
            .project(
                directive.reference_lane,
                Point3 {
                    x: ego.pose.x,
                    y: ego.pose.y,
                    z: ego.z,
                },
            )
            .map_err(|e| e.to_string())?;
        let desired = directive
            .speed_limit
            .min(snapshot.raw.navigation_speed_limit_mps as f64)
            .min(graph.lanes[&directive.reference_lane].speed_limit);
        if !desired.is_finite() || desired < 0.0 {
            return Err("invalid controlled-route speed constraint".into());
        }
        let lead = select_lead(
            graph,
            &mission.route,
            lane,
            ego,
            &snapshot.scene.agents,
            self.front_bumper_for_control(),
            self.calibration.geometry.width,
        );
        self.target_acceleration = IdmConfig {
            desired_speed: desired.max(0.1),
            ..Default::default()
        }
        .acceleration(
            ego.speed.max(0.0),
            lead.map(|observation| observation.idm()),
        )
        .map_err(|e| e.to_string())?;
        let stop_profile = directive
            .stop_distance
            .is_some_and(|distance| distance < ego.speed.max(1.0) * 4.0 + 3.0);
        self.intent.precise_stop = stop_profile;
        let mut trajectory = if stop_profile {
            let distance = directive.stop_distance.unwrap().max(0.1);
            let deceleration =
                (ego.speed.max(0.0).powi(2) / (2.0 * (distance - 0.15).max(0.1))).max(0.1);
            if deceleration > 4.0 {
                return Err("stop line cannot be reached within emergency braking envelope".into());
            }
            self.target_acceleration = -deceleration;
            let mut points = Vec::new();
            let mut trailer_yaw = ego.trailer_yaw;
            for i in 0..=40 {
                let t = i as f64 * 0.1;
                let effective = t.min(ego.speed.max(0.0) / deceleration);
                let travel =
                    ego.speed.max(0.0) * effective - 0.5 * deceleration * effective * effective;
                let (id, s) = graph
                    .advance(directive.reference_lane, reference.s, travel)
                    .map_err(|e| e.to_string())?;
                let (pos, yaw) = graph.lanes[&id]
                    .sample(s)
                    .ok_or("stop reference unavailable")?;
                let speed = (ego.speed - deceleration * t).max(0.0);
                let curvature = if let Some(previous) = points.last() {
                    let previous: &TrajPoint = previous;
                    wrap_angle(yaw - previous.pose.yaw)
                        / (pos.x - previous.pose.x)
                            .hypot(pos.y - previous.pose.y)
                            .max(0.01)
                } else {
                    0.0
                };
                if !self.calibration.geometry.has_trailer {
                    trailer_yaw = yaw;
                } else if i > 0 {
                    let phi = wrap_angle(yaw - trailer_yaw);
                    trailer_yaw = wrap_angle(
                        trailer_yaw
                            + 0.1
                                * (speed * phi.sin() / self.calibration.geometry.trailer_wheelbase
                                    - speed
                                        * self.calibration.geometry.hitch_offset
                                        * phi.cos()
                                        * curvature
                                        / self.calibration.geometry.trailer_wheelbase),
                    );
                }
                points.push(TrajPoint {
                    t,
                    pose: Pose2 {
                        x: pos.x,
                        y: pos.y,
                        yaw,
                    },
                    speed,
                    acceleration: if speed > 0.0 { -deceleration } else { 0.0 },
                    curvature,
                    articulation: wrap_angle(yaw - trailer_yaw),
                });
            }
            Trajectory {
                header: ego.header,
                points,
                direction: Direction::Forward,
                behavior: directive.behavior,
            }
        } else {
            let target = (ego.speed + self.target_acceleration * 4.0).clamp(0.0, desired);
            frenet::plan(FrenetRequest {
                graph,
                lane_id: directive.reference_lane,
                ego,
                target_speed: target,
                lateral_targets: &[0.0],
                predictions,
                obstacles: &[],
                geometry: self.calibration.geometry,
                lateral_acceleration_limit: 1.2,
            })
            .map_err(|e| e.to_string())?
        };
        trajectory.behavior = directive.behavior;
        if snapshot.scene.agents.is_empty()
            && self
                .region
                .as_ref()
                .and_then(|r| r.imported.runtime_to_source.get(&directive.reference_lane))
                .is_some_and(|id| id.contains(":prefab:"))
            && trajectory.points.len() >= 41
            && trajectory.behavior == BehaviorKind::LaneFollow
        {
            let geometry = self.calibration.geometry;
            let references: Vec<_> = trajectory
                .points
                .iter()
                .step_by(2)
                .take(21)
                .map(|p| TruckState {
                    pose: p.pose,
                    trailer_yaw: p.pose.yaw - p.articulation,
                    speed: p.speed,
                    steer: (p.curvature * geometry.wheelbase).atan(),
                })
                .collect();
            let inputs: Vec<_> = references
                .windows(2)
                .map(|p| etsads_planning::ilqr::Input {
                    acceleration: ((p[1].speed - p[0].speed) / 0.2).clamp(-3.9, 3.9),
                    steer_rate: ((p[1].steer - p[0].steer) / 0.2).clamp(-0.55, 0.55),
                })
                .collect();
            if let Ok(refined) =
                etsads_planning::ilqr::refine(etsads_planning::ilqr::RefinementRequest {
                    initial: TruckState::from(ego),
                    reference: &references,
                    initial_inputs: &inputs,
                    geometry,
                    drivable: &mission.boundary.polygon,
                    obstacles: &[],
                    dt: 0.2,
                    max_iterations: 5,
                })
            {
                let points = refined
                    .states
                    .iter()
                    .enumerate()
                    .map(|(i, s)| TrajPoint {
                        t: i as f64 * 0.2,
                        pose: s.pose,
                        speed: s.speed,
                        acceleration: refined.inputs.get(i).map_or(0.0, |u| u.acceleration),
                        curvature: s.steer.tan() / geometry.wheelbase,
                        articulation: s.articulation(),
                    })
                    .collect();
                let candidate = Trajectory {
                    points,
                    ..trajectory.clone()
                };
                if candidate.validate().is_ok() && refined.final_cost < refined.initial_cost {
                    trajectory = candidate;
                    self.intent
                        .reason
                        .push_str("; articulated iLQR refinement accepted");
                }
            }
        }
        trajectory.validate().map_err(|e| e.to_string())?;
        let mut previous: Option<Vec<etsads_msgs::OrientedBox>> = None;
        let mut previous_time = None;
        for point in &trajectory.points {
            let state = TruckState {
                pose: point.pose,
                trailer_yaw: point.pose.yaw - point.articulation,
                speed: point.speed,
                steer: (point.curvature * self.calibration.geometry.wheelbase).atan(),
            };
            let boxes = footprints(state, self.calibration.geometry);
            if boxes
                .iter()
                .any(|b| !box_in_polygon(*b, &mission.boundary.polygon))
            {
                return Err("planned truck sweep leaves verified physical boundary".into());
            }
            if let Some(previous) = &previous {
                for i in 0..boxes.len() {
                    if !etsads_planning::geometry::swept_in_polygon(
                        previous[i],
                        boxes[i],
                        &mission.boundary.polygon,
                    ) {
                        return Err(
                            "between-point truck sweep leaves verified physical boundary".into(),
                        );
                    }
                }
            }
            for agent in predictions {
                for mode in &agent.modes {
                    let obstacle = etsads_msgs::OrientedBox {
                        pose: etsads_prediction::sample_mode(mode, point.t)
                            .ok_or("traffic prediction horizon missing")?,
                        length: agent.length,
                        width: agent.width,
                    };
                    if boxes
                        .iter()
                        .any(|b| etsads_planning::geometry::overlaps(*b, obstacle, 0.5))
                    {
                        return Err("controlled trajectory conflicts with moving traffic".into());
                    }
                    if let (Some(before), Some(t)) = (&previous, previous_time) {
                        let before_obstacle = etsads_msgs::OrientedBox {
                            pose: etsads_prediction::sample_mode(mode, t)
                                .ok_or("traffic prediction horizon missing")?,
                            length: agent.length,
                            width: agent.width,
                        };
                        if before.iter().zip(&boxes).any(|(a, b)| {
                            etsads_planning::geometry::moving_swept_collision(
                                *a,
                                *b,
                                before_obstacle,
                                obstacle,
                                0.5,
                            )
                        }) {
                            return Err(
                                "continuous relative sweep conflicts with moving traffic".into()
                            );
                        }
                    }
                }
            }
            previous_time = Some(point.t);
            previous = Some(boxes);
        }
        Ok(trajectory)
    }
    fn make_plan(
        &mut self,
        snapshot: &GameSnapshot,
        lane: Option<LaneState>,
    ) -> Result<Trajectory, String> {
        let predictions = if matches!(
            self.config.mode,
            DrivingMode::LaneFollow { .. } | DrivingMode::ControlledRoute { .. }
        ) {
            // Other agents may take branches outside the ego corridor. Track
            // them on the complete loaded graph, not planning_graph's pruned exits.
            let graph = &self
                .region
                .as_ref()
                .ok_or("prediction region unavailable")?
                .imported
                .graph;
            let batch = self
                .prediction
                .predict(
                    graph,
                    &prediction_bodies(&snapshot.scene.agents),
                    etsads_prediction::PredictionContext {
                        session_start_qpc: snapshot.session_start_qpc,
                        map_generation: self.generation,
                        frame_id: snapshot.ego.header.frame_id,
                        sim_time_us: snapshot.ego.header.sim_time_us,
                    },
                    &std::collections::BTreeMap::new(),
                )
                .map_err(|error| error.to_string())?;
            self.prediction_diagnostic = Some(batch.diagnostics);
            batch.agents
        } else {
            Vec::new()
        };
        let trajectory = self.make_plan_inner(snapshot, lane, &predictions)?;
        self.check_road_trajectory(TruckState::from(snapshot.ego), &trajectory)?;
        Ok(trajectory)
    }
    fn make_plan_inner(
        &mut self,
        snapshot: &GameSnapshot,
        lane: Option<LaneState>,
        predictions: &[etsads_prediction::PredictedAgent],
    ) -> Result<Trajectory, String> {
        let ego = snapshot.ego;
        let mission_frame = *self.mission_frame.get_or_insert(ego.header.frame_id);
        self.intent = PlanningIntent::default();
        match self.config.mode.clone() {
            DrivingMode::ControlledRoute { mission } => self.plan_controlled(
                snapshot,
                lane.ok_or("controlled route has no lane match")?,
                &mission,
                predictions,
            ),
            DrivingMode::Parking { .. } => {
                let obstacles = parking_obstacles::collect(snapshot)?;
                let delivered = snapshot.events.iter().any(|e| {
                    e.header.frame_id >= mission_frame
                        && e.name.split(|b| *b == 0).next() == Some(b"job.delivered".as_slice())
                });
                let engine = self.parking.as_mut().ok_or("parking engine unavailable")?;
                let result = engine.update(
                    ego,
                    snapshot.raw.gear,
                    delivered,
                    &obstacles.current,
                    &obstacles.occupancy,
                );
                let mut recovery = engine.last_recovery_diagnostic.clone();
                let search_statistics = engine.search_statistics();
                if let Some(diagnostic) = &mut recovery {
                    diagnostic.frame_id = ego.header.frame_id;
                    diagnostic.articulation_rad = TruckState::from(ego).articulation();
                }
                let (trajectory, mut intent) = match result {
                    Ok(result) => result,
                    Err(error) => {
                        self.intent.parking_recovery = recovery;
                        self.intent.parking_search = search_statistics;
                        return Err(error);
                    }
                };
                if intent.parking_recovery.is_none() {
                    intent.parking_recovery = recovery;
                }
                intent.parking_search = search_statistics;
                self.intent = intent;
                Ok(trajectory)
            }
            DrivingMode::CommissioningCircle {
                radius_m,
                speed_mps,
                clockwise,
                verified_clear_area,
            } => {
                let sign = if clockwise { -1.0 } else { 1.0 };
                let (center_x, center_y) = *self.circle_center.get_or_insert((
                    ego.pose.x - sign * radius_m * ego.pose.yaw.sin(),
                    ego.pose.y + sign * radius_m * ego.pose.yaw.cos(),
                ));
                let radial = (ego.pose.y - center_y).atan2(ego.pose.x - center_x);
                let error = ((ego.pose.x - center_x).hypot(ego.pose.y - center_y) - radius_m).abs();
                if error > 1.0 {
                    return Err("commissioning circle deviation exceeds 1 m".into());
                }
                self.target_acceleration = (0.8 * (speed_mps - ego.speed)).clamp(-1.5, 1.0);
                let mut points = Vec::new();
                for i in 0..=100 {
                    let t = i as f64 * 0.1;
                    let angle = radial + sign * speed_mps * t / radius_m;
                    let pose = Pose2 {
                        x: center_x + radius_m * angle.cos(),
                        y: center_y + radius_m * angle.sin(),
                        yaw: wrap_angle(angle + sign * std::f64::consts::FRAC_PI_2),
                    };
                    let point = TrajPoint {
                        t,
                        pose,
                        speed: speed_mps,
                        acceleration: self.target_acceleration,
                        curvature: sign / radius_m,
                        articulation: 0.0,
                    };
                    let truck = TruckState {
                        pose,
                        trailer_yaw: pose.yaw,
                        speed: speed_mps,
                        steer: 0.0,
                    };
                    if footprints(truck, self.calibration.geometry)
                        .iter()
                        .any(|b| !box_in_polygon(*b, &verified_clear_area))
                    {
                        return Err("circle leaves verified clear test area".into());
                    }
                    points.push(point);
                }
                Ok(Trajectory {
                    header: ego.header,
                    points,
                    direction: Direction::Forward,
                    behavior: BehaviorKind::LaneFollow,
                })
            }
            DrivingMode::LaneFollow {
                route,
                maximum_speed_mps,
            } => {
                let lane = lane.ok_or("no lane match")?;
                let graph = self.planning_graph.as_ref().unwrap();
                let current = &graph.lanes[&lane.lane_id];
                let max_curvature = current
                    .center
                    .windows(3)
                    .map(|p| {
                        let a = (p[1].y - p[0].y).atan2(p[1].x - p[0].x);
                        let b = (p[2].y - p[1].y).atan2(p[2].x - p[1].x);
                        wrap_angle(b - a).abs()
                            / ((p[0].distance_xy(p[1]) + p[1].distance_xy(p[2])) / 2.0).max(0.01)
                    })
                    .fold(0.0_f64, f64::max);
                let desired = curve_speed_limit(
                    (snapshot.raw.navigation_speed_limit_mps as f64).min(maximum_speed_mps),
                    current.speed_limit,
                    max_curvature,
                    true,
                )
                .map_err(|e| e.to_string())?;
                let lead = select_lead(
                    graph,
                    &route,
                    lane,
                    ego,
                    &snapshot.scene.agents,
                    self.front_bumper_for_control(),
                    self.calibration.geometry.width,
                );
                self.target_acceleration = IdmConfig {
                    desired_speed: desired,
                    ..Default::default()
                }
                .acceleration(
                    ego.speed.max(0.0),
                    lead.map(|observation| observation.idm()),
                )
                .map_err(|e| e.to_string())?;
                if let Some(distance) = self.native_stop_distance(lane) {
                    if distance <= 0.2 && ego.speed.abs() < 0.1 {
                        self.intent.hold_brake = true;
                        self.intent.reason =
                            "holding before the verified native GPS corridor terminal".into();
                        self.target_acceleration = -1.6;
                        return Ok(parking_mission::hold_trajectory(ego));
                    }
                    if distance < 0.0 {
                        return Err("vehicle passed verified native GPS corridor terminal".into());
                    }
                    self.target_acceleration = self.target_acceleration.min(
                        IdmConfig {
                            desired_speed: desired,
                            ..Default::default()
                        }
                        .acceleration(ego.speed.max(0.0), Some((distance.max(0.01), 0.0)))
                        .map_err(|e| e.to_string())?,
                    );
                }
                let target = (ego.speed + self.target_acceleration * 4.0).clamp(0.0, desired);
                frenet::plan(FrenetRequest {
                    graph,
                    lane_id: lane.lane_id,
                    ego,
                    target_speed: target,
                    lateral_targets: &[0.0],
                    predictions,
                    obstacles: &[],
                    geometry: self.calibration.geometry,
                    lateral_acceleration_limit: 1.2,
                })
                .map_err(|e| e.to_string())
            }
        }
    }
}
fn prediction_bodies(agents: &[Agent]) -> Vec<Agent> {
    let mut bodies = Vec::new();
    for agent in agents {
        let mut tractor = agent.clone();
        tractor.trailers.clear();
        bodies.push(tractor);
        for (index, trailer) in agent.trailers.iter().enumerate() {
            bodies.push(Agent {
                id: agent.id.wrapping_add((index as u64 + 1) << 48),
                pose: trailer.pose,
                z: agent.z,
                speed: agent.speed,
                acceleration: agent.acceleration,
                length: trailer.length,
                width: trailer.width,
                lane_id: None,
                trailers: vec![],
            });
        }
    }
    bodies
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::replay::*;
    use etsads_control::identification::SteeringRow;

    #[test]
    fn measured_clock_controls_and_pause_releases() {
        let mut runtime = Runtime::new(config(ExecutionMode::Replay), calibration(), None).unwrap();
        assert_eq!(
            runtime.step(&snapshot(0)).status,
            RuntimeStatus::WaitingForSecondFrame
        );
        let result = runtime.step(&snapshot(1));
        assert_eq!(result.status, RuntimeStatus::Running, "{}", result.reason);
        assert!(result.actuation.valid() && result.request_control);
        assert!(result.trajectory.is_some());
        let mut pause = snapshot(2);
        pause.ego.paused = true;
        let result = runtime.step(&pause);
        assert_eq!(result.status, RuntimeStatus::Paused);
        assert!(!result.request_control);
        assert_eq!(result.actuation, Actuation::default());
    }
    #[test]
    fn stale_data_fault_latches_and_vehicle_change_is_detected() {
        let mut runtime = Runtime::new(config(ExecutionMode::Replay), calibration(), None).unwrap();
        runtime.step(&snapshot(0));
        assert_eq!(runtime.step(&snapshot(20)).status, RuntimeStatus::Fault);
        assert_eq!(runtime.step(&snapshot(21)).status, RuntimeStatus::Fault);
        runtime.reset();
        runtime.step(&snapshot(0));
        let mut changed = snapshot(1);
        changed.config.as_mut().unwrap().wheel_positions[0][2] -= 0.2;
        let output = runtime.step(&changed);
        assert_eq!(output.status, RuntimeStatus::Fault);
        assert!(output.reason.contains("geometry"));
    }
    #[test]
    fn live_worker_does_not_block_control_until_a_trajectory_exists() {
        let mut runtime = Runtime::new(
            config(ExecutionMode::Live),
            with_measured_envelope(calibration()),
            None,
        )
        .unwrap();
        runtime.step(&snapshot(0));
        let first = runtime.step(&snapshot(1));
        assert_eq!(first.status, RuntimeStatus::WaitingForTrajectory);
        assert!(!first.request_control);
        let mut running = false;
        for i in 2..10 {
            std::thread::sleep(std::time::Duration::from_millis(2));
            let output = runtime.step(&snapshot(i));
            if output.status == RuntimeStatus::Running {
                running = true;
                break;
            }
        }
        assert!(running);
    }
    #[test]
    fn live_rejects_unmeasured_calibration() {
        let mut cal = calibration();
        cal.geometry_fingerprint = None;
        assert!(Runtime::new(config(ExecutionMode::Live), cal, None).is_err());
    }
    #[test]
    fn verified_region_runs_acc_and_missing_traffic_faults() {
        use etsads_map::{
            Lane,
            importer::{ImportedGraph, RegionPolicy},
        };
        let graph = LaneGraph::new(vec![Lane {
            id: 1,
            center: vec![
                Point3::default(),
                Point3 {
                    x: 500.0,
                    ..Default::default()
                },
            ],
            width: 4.5,
            speed_limit: 8.0,
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
        let region = RegionGraph {
            imported: ImportedGraph {
                graph,
                runtime_to_source: [(1, "fixture:road:0".into())].into(),
                native_owners: [(1, 100)].into(),
            },
            policy: RegionPolicy {
                region_id: "unit-fixture".into(),
                evidence_path: "synthetic unit fixture".into(),
                verified_lane_ids: vec!["fixture:road:0".into()],
                allow_navigation_width: true,
                require_fresh_telemetry_speed_limit: true,
                maximum_speed_mps: 8.0,
                maximum_lateral_error_m: 0.6,
            },
            physical_boundaries_verified: false,
        };
        let mut cfg = config(ExecutionMode::Replay);
        cfg.mode = DrivingMode::LaneFollow {
            route: vec![1],
            maximum_speed_mps: 7.0,
        };
        let mut runtime = Runtime::new(cfg, calibration(), Some(region)).unwrap();
        runtime.step(&snapshot(0));
        let mut frame = snapshot(1);
        let mut extension = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        extension.available = 2;
        extension.header.frame_id = 1;
        frame.extension = Some(extension);
        let out = runtime.step(&frame);
        assert_eq!(out.status, RuntimeStatus::Running, "{}", out.reason);
        assert_eq!(out.lane.unwrap().lane_id, 1);
        assert!(out.actuation.throttle > 0.0);
        let out = runtime.step(&snapshot(2));
        assert_eq!(out.status, RuntimeStatus::Fault);
        assert!(out.reason.contains("traffic"));
    }
    #[test]
    fn parking_replay_closes_loop_with_gear_acknowledgement() {
        use parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary};
        let geometry = etsads_msgs::VehicleGeometry::default();
        let mut cal = calibration();
        cal.reference = None;
        cal.geometry_fingerprint = None;
        cal.longitudinal.as_mut().unwrap().coefficients = [0.0, 1.2, -4.0, 0.0, 0.0];
        let samples: Vec<_> = (-10..=10)
            .map(|i| {
                let axis = i as f64 / 10.0;
                (axis, (axis * geometry.max_steer).tan() / geometry.wheelbase)
            })
            .collect();
        cal.steering_rows = vec![
            SteeringRow {
                speed: 0.0,
                samples: samples.clone(),
                counts: vec![100; 21],
                rms_curvature: 0.0,
            },
            SteeringRow {
                speed: 2.0,
                samples,
                counts: vec![100; 21],
                rms_curvature: 0.0,
            },
        ];
        let mission = ParkingMission {
            target: ParkingTarget {
                source_id: "synthetic axle target".into(),
                pose: Pose2 {
                    x: -18.0,
                    y: 1.0,
                    yaw: 0.0,
                },
                reference: SlotReference::TrailerAxle,
            },
            boundary: VerifiedBoundary {
                polygon: vec![(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)],
                physical_boundaries_verified: true,
                evidence_path: "synthetic closed-loop fixture".into(),
            },
            static_obstacles: vec![
                etsads_msgs::OrientedBox {
                    pose: Pose2 {
                        x: -24.0,
                        y: -2.2,
                        yaw: 0.0,
                    },
                    length: 14.0,
                    width: 2.5,
                },
                etsads_msgs::OrientedBox {
                    pose: Pose2 {
                        x: -24.0,
                        y: 4.5,
                        yaw: 0.0,
                    },
                    length: 14.0,
                    width: 2.5,
                },
            ],
            max_expansions: 10000,
            maximum_duration_s: 120.0,
            recovery: Default::default(),
        };
        let mut cfg = config(ExecutionMode::Replay);
        cfg.mode = DrivingMode::Parking { mission };
        let mut runtime = Runtime::new(cfg, cal, None).unwrap();
        let mut state = TruckState::default();
        let mut gear = 0;
        let mut completed = false;
        let mut shifts = 0;
        for i in 0..6000 {
            let mut frame = snapshot(i);
            frame.ego = EgoState {
                header: Header {
                    frame_id: i,
                    sim_time_us: i * 20000,
                    seq: i,
                },
                pose: state.pose,
                speed: state.speed,
                wheel_angle: state.steer,
                trailer_yaw: state.trailer_yaw,
                yaw_rate: state.speed * state.steer.tan() / geometry.wheelbase,
                ..Default::default()
            };
            frame.scene.ego = frame.ego;
            frame.raw.gear = gear;
            let output = runtime.step(&frame);
            assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
            if output.status == RuntimeStatus::Completed {
                completed = true;
                break;
            }
            if let Some(direction) = output.intent.gear_request {
                assert!(state.speed.abs() < 0.05);
                gear = if direction == Direction::Reverse {
                    -1
                } else {
                    1
                };
                shifts += 1;
            }
            let command = output.actuation;
            let sign = if gear < 0 { -1.0 } else { 1.0 };
            let acceleration = sign * command.throttle * 1.2
                - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
            state = state
                .step(
                    geometry,
                    command.steer * geometry.max_steer,
                    acceleration,
                    0.02,
                )
                .unwrap();
        }
        assert!(
            completed,
            "parking did not complete; axle {:?}",
            state.trailer_axle(geometry)
        );
        assert!(shifts >= 2);
        assert!(
            state.trailer_axle(geometry).distance(Pose2 {
                x: -18.0,
                y: 1.0,
                yaw: 0.0
            }) < 0.5
        );
    }
    #[test]
    fn controlled_signal_stops_then_resumes_on_spatially_matched_green() {
        let (cfg, region) = controlled_fixture(false, true, false);
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let mut state = TruckState {
            speed: 6.0,
            ..Default::default()
        };
        let mut stopped = false;
        for i in 0..1200 {
            let signal = etsads_game::ets2la::Signal {
                position: [60.0, 0.0, -3.0],
                sector: [0, 0],
                quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                kind: 1,
                time_remaining: 30.0,
                state: if i < 750 { 2 } else { 8 },
                id: 99,
            };
            let output = runtime.step(&simulated_frame(state, i, Some(signal), false));
            assert_ne!(
                output.status,
                RuntimeStatus::Fault,
                "at {i}: {}",
                output.reason
            );
            state = integrate_command(state, output.actuation);
            if i < 750 {
                assert!(state.pose.x < 60.0 - (3.8 * 0.35 + 6.2 / 2.0) - 3.0 + 0.5);
                if state.speed < 0.1 {
                    stopped = true;
                }
            }
        }
        assert!(stopped);
        assert!(state.pose.x > 65.0, "{}", state.pose.x);
    }
    #[test]
    fn toll_replay_sends_one_pulse_waits_open_and_uses_paid_event() {
        let (cfg, region) = controlled_fixture(false, false, true);
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let mut state = TruckState {
            speed: 6.0,
            ..Default::default()
        };
        let mut activation = None;
        let mut count = 0;
        let mut paid = false;
        for i in 0..1500 {
            let open = activation.is_some_and(|frame| i > frame + 50);
            let signal = etsads_game::ets2la::Signal {
                position: [40.0, 0.0, 0.0],
                sector: [0, 0],
                quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                kind: 2,
                time_remaining: 0.0,
                state: if open { 3 } else { 1 },
                id: 101,
            };
            let pay_now = !paid && state.pose.x > 42.0;
            if pay_now {
                paid = true;
            }
            let output = runtime.step(&simulated_frame(state, i, Some(signal), pay_now));
            assert_ne!(
                output.status,
                RuntimeStatus::Fault,
                "at {i}: {}",
                output.reason
            );
            if output.actuation.activate {
                count += 1;
                activation = Some(i);
                assert!(state.speed < 0.1);
                assert!((40.0 - (3.8 * 0.35 + 6.2 / 2.0) - state.pose.x - 3.0).abs() < 0.6);
            }
            if !open {
                assert!(state.pose.x < 40.0 - (3.8 * 0.35 + 6.2 / 2.0) - 3.0 + 0.6);
            }
            state = integrate_command(state, output.actuation);
        }
        assert_eq!(count, 1);
        assert!(paid && state.pose.x > 45.0);
    }
    #[test]
    fn controlled_lane_change_waits_for_indicator_then_tracks_target_lane() {
        let (cfg, region) = controlled_fixture(true, false, false);
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let mut state = TruckState {
            pose: Pose2 {
                x: 20.0,
                ..Default::default()
            },
            speed: 6.0,
            ..Default::default()
        };
        let mut indicated = false;
        for i in 0..1200 {
            let output = runtime.step(&simulated_frame(state, i, None, false));
            assert_ne!(
                output.status,
                RuntimeStatus::Fault,
                "at {i} y={} : {}",
                state.pose.y,
                output.reason
            );
            if output.actuation.indicator == 1 {
                indicated = true;
            }
            if i < 150 {
                assert!(state.pose.y.abs() < 0.1);
            }
            state = integrate_command(state, output.actuation);
        }
        assert!(indicated);
        assert!((state.pose.y - 4.5).abs() < 0.5, "final y {}", state.pose.y);
    }
    #[test]
    fn curved_prefab_route_uses_articulated_optimizer_inside_boundaries() {
        let (mut cfg, mut region) = controlled_fixture(false, false, false);
        let source = "fixture:prefab:0".to_owned();
        region.imported.runtime_to_source.insert(1, source.clone());
        let mut lane = region.imported.graph.lanes[&1].clone();
        lane.center = (0..=60)
            .map(|i| {
                let angle = i as f64 * 0.02;
                Point3 {
                    x: 60.0 * angle.sin(),
                    y: 60.0 * (1.0 - angle.cos()),
                    z: 0.0,
                }
            })
            .collect();
        region.imported.graph = LaneGraph::new(vec![lane]).unwrap();
        let DrivingMode::ControlledRoute { mission } = &mut cfg.mode else {
            panic!()
        };
        mission.maximum_speed_mps = 3.0;
        mission.boundary.polygon = vec![
            (-30.0, -10.0),
            (100.0, -10.0),
            (100.0, 100.0),
            (-30.0, 100.0),
        ];
        mission.traffic.source_lane_ids = vec![source.clone()];
        mission.traffic.lane_controls = vec![etsads_map::traffic::LaneControl {
            lane_id: source,
            traffic_rule: "uncontrolled".into(),
            control_kind: etsads_map::traffic::ControlKind::Uncontrolled,
            priority_modifier: None,
            additive_priority: false,
            signal_group_id: None,
            evidence_source: "synthetic fixture".into(),
        }];
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let mut state = TruckState {
            speed: 3.0,
            ..Default::default()
        };
        let mut refined = false;
        for i in 0..300 {
            let output = runtime.step(&simulated_frame(state, i, None, false));
            assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
            refined |= output.intent.reason.contains("iLQR");
            state = integrate_command(state, output.actuation);
        }
        assert!(refined);
        assert!(state.pose.y > 1.0);
        assert!(state.articulation().abs() < std::f64::consts::PI / 3.0);
    }
    #[test]
    fn identified_bobtail_runs_and_changed_cargo_or_attachment_faults() {
        let mut cal = calibration();
        cal.geometry.has_trailer = false;
        cal.geometry.trailer_length = 0.0;
        cal.geometry.trailer_wheelbase = 0.0;
        cal.conditions.as_mut().unwrap().trailer_count = 0;
        cal = with_measured_envelope(cal);
        assert!(Runtime::new(config(ExecutionMode::Live), cal.clone(), None).is_ok());
        let mut runtime = Runtime::new(config(ExecutionMode::Replay), cal.clone(), None).unwrap();
        let mut first = snapshot(0);
        first.raw.trailers[0].connected = 0;
        runtime.step(&first);
        let mut next = snapshot(1);
        next.raw.trailers[0].connected = 0;
        let output = runtime.step(&next);
        assert_eq!(output.status, RuntimeStatus::Running, "{}", output.reason);
        assert_eq!(
            footprints(TruckState::from(output.control_ego), cal.geometry).len(),
            1
        );
        let mut loaded = snapshot(2);
        loaded.raw.trailers[0].connected = 0;
        loaded.config.as_mut().unwrap().cargo_mass_kg = 20000.0;
        let output = runtime.step(&loaded);
        assert_eq!(output.status, RuntimeStatus::Fault);
        assert!(output.reason.contains("cargo"));
        let mut second = Runtime::new(config(ExecutionMode::Replay), cal, None).unwrap();
        second.step(&first);
        let output = second.step(&snapshot(1));
        assert_eq!(output.status, RuntimeStatus::Fault);
        assert!(output.reason.contains("trailer"));
    }
    #[test]
    fn opposing_traffic_uses_selected_braking_trajectory_in_control() {
        let (cfg, region) = controlled_fixture(false, false, false);
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let state = TruckState {
            speed: 6.0,
            ..Default::default()
        };
        runtime.step(&simulated_frame(state, 0, None, false));
        let mut frame = simulated_frame(state, 1, None, false);
        frame.scene.agents.push(Agent {
            id: 10,
            pose: Pose2 {
                x: 50.0,
                y: 0.0,
                yaw: std::f64::consts::PI,
            },
            z: 0.0,
            speed: 6.0,
            acceleration: 0.0,
            length: 4.0,
            width: 2.0,
            lane_id: None,
            trailers: vec![],
        });
        let output = runtime.step(&frame);
        assert_eq!(output.status, RuntimeStatus::Running, "{}", output.reason);
        assert_eq!(output.actuation.throttle, 0.0);
        assert!(
            output.actuation.brake > 0.08,
            "brake {}",
            output.actuation.brake
        );
    }
    #[test]
    fn crossing_between_stop_trajectory_samples_is_not_missed() {
        let (cfg, region) = controlled_fixture(false, true, false);
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), Some(region)).unwrap();
        let state = TruckState {
            pose: Pose2 {
                x: 45.0,
                ..Default::default()
            },
            speed: 6.0,
            ..Default::default()
        };
        let signal = etsads_game::ets2la::Signal {
            position: [60.0, 0.0, -3.0],
            sector: [0, 0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            kind: 1,
            time_remaining: 30.0,
            state: 2,
            id: 99,
        };
        runtime.step(&simulated_frame(state, 0, Some(signal), false));
        let mut frame = simulated_frame(state, 1, Some(signal), false);
        frame.scene.agents.push(Agent {
            id: 11,
            pose: Pose2 {
                x: 48.0,
                y: -2.2,
                yaw: std::f64::consts::FRAC_PI_2,
            },
            z: 0.0,
            speed: 70.0,
            acceleration: 0.0,
            length: 0.2,
            width: 0.2,
            lane_id: None,
            trailers: vec![],
        });
        let output = runtime.step(&frame);
        assert_eq!(output.status, RuntimeStatus::Fault);
        assert!(
            output.reason.contains("continuous relative sweep"),
            "{}",
            output.reason
        );
        assert_eq!(output.actuation.throttle, 0.0);
    }
    #[test]
    fn parking_checks_trailer_boundary_and_allows_identified_bobtail() {
        use parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary};
        let mission = |reference| ParkingMission {
            target: ParkingTarget {
                source_id: "synthetic bounded target".into(),
                pose: Pose2 {
                    x: 5.0,
                    y: 0.0,
                    yaw: 0.0,
                },
                reference,
            },
            boundary: VerifiedBoundary {
                polygon: vec![(-6.0, -3.0), (20.0, -3.0), (20.0, 3.0), (-6.0, 3.0)],
                physical_boundaries_verified: true,
                evidence_path: "synthetic validated boundary".into(),
            },
            static_obstacles: vec![],
            max_expansions: 1000,
            maximum_duration_s: 60.0,
            recovery: Default::default(),
        };
        let mut cfg = config(ExecutionMode::Replay);
        cfg.mode = DrivingMode::Parking {
            mission: mission(SlotReference::TrailerAxle),
        };
        let mut runtime = Runtime::new(cfg, low_speed_calibration(), None).unwrap();
        runtime.step(&simulated_frame(TruckState::default(), 0, None, false));
        let out = runtime.step(&simulated_frame(TruckState::default(), 1, None, false));
        assert_eq!(out.status, RuntimeStatus::Fault);
        assert!(out.reason.contains("boundary"));
        let mut cal = low_speed_calibration();
        cal.geometry.has_trailer = false;
        cal.geometry.trailer_length = 0.0;
        cal.geometry.trailer_wheelbase = 0.0;
        cal.conditions.as_mut().unwrap().trailer_count = 0;
        let mut cfg = config(ExecutionMode::Replay);
        cfg.mode = DrivingMode::Parking {
            mission: mission(SlotReference::TractorRearAxle),
        };
        let mut runtime = Runtime::new(cfg, cal, None).unwrap();
        let mut first = simulated_frame(TruckState::default(), 0, None, false);
        first.raw.trailers[0].connected = 0;
        runtime.step(&first);
        let mut next = simulated_frame(TruckState::default(), 1, None, false);
        next.raw.trailers[0].connected = 0;
        let out = runtime.step(&next);
        assert_eq!(out.status, RuntimeStatus::Running, "{}", out.reason);
        assert!(out.actuation.throttle > 0.0);
    }
}
