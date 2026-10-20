//! Live orchestration. Real game snapshots are the only input, an independent
//! supervisor owns Gate, and a recording worker must remain healthy throughout.
mod actions;
pub mod experiment;
pub mod map_session;
mod observed_evidence;
use anyhow::{Context, ensure};
use etsads_control::identification::VehicleCalibration;
use etsads_game::adapter::{GameAdapter, GameSnapshot};
use etsads_map::importer::{ExportedMap, RegionPolicy, load_parquet_region};
use etsads_msgs::{Point3, protocol::*};
use etsads_recorder::{Record, RunMetadata};
use etsads_runtime::{
    DrivingMode, ExecutionMode, Runtime, RuntimeConfig, RuntimeOutput, RuntimeStatus,
};
use etsads_shm::{SharedCell, SingleWriterGuard, qpc, qpc_frequency};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, SyncSender, TrySendError},
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub struct DriveOptions {
    pub config: PathBuf,
    pub calibration: PathBuf,
    pub output: PathBuf,
    pub map: Option<PathBuf>,
    pub policy: Option<PathBuf>,
    pub region_radius_m: f64,
    pub seconds: u64,
    pub arm: bool,
    pub video: bool,
    pub experiment: Option<experiment::ExperimentKind>,
    pub experiment_evidence: Option<PathBuf>,
    pub safety_profile: Option<PathBuf>,
    pub road_evidence: Option<PathBuf>,
}
#[derive(Debug, Serialize)]
pub struct DriveReport {
    pub source: &'static str,
    pub output: PathBuf,
    pub stop_reason: String,
    pub frames: u64,
    pub active_frames: u64,
    pub arm_requested: bool,
    pub capabilities: Vec<String>,
    pub recording: RunMetadata,
}
#[derive(Clone, Debug)]
pub struct DriveProgress {
    /// Source time, not GUI receive time: a full queue must not refresh old state.
    pub produced_at: Instant,
    pub frame_id: u64,
    pub status: RuntimeStatus,
    pub reason: String,
    pub arm_requested: bool,
    pub control_sent: bool,
    pub speed_mps: f64,
}

fn read_config<T: DeserializeOwned>(path: &Path, toml: bool) -> anyhow::Result<T> {
    use std::io::Read;
    let mut text = String::new();
    std::fs::File::open(path)
        .with_context(|| format!("open {}", path.display()))?
        .take(8 * 1024 * 1024 + 1)
        .read_to_string(&mut text)?;
    ensure!(text.len() <= 8 * 1024 * 1024, "configuration exceeds 8 MiB");
    if toml {
        ::toml::from_str(&text).with_context(|| format!("parse TOML {}", path.display()))
    } else {
        serde_json::from_str(&text).with_context(|| format!("parse JSON {}", path.display()))
    }
}

/// RAII release is armed before recording starts, so every return path clears
/// owned command axes and requests supervisor disarm without writing Gate.
struct CommandSink {
    command: SharedCell<ControlCmd>,
    requests: SharedCell<SafetyRequest>,
    last: ControlCmd,
    actions: actions::ActionExecutor,
    stop: Arc<AtomicBool>,
}
impl CommandSink {
    fn connect(stop: Arc<AtomicBool>) -> anyhow::Result<Self> {
        let command = SharedCell::create(CONTROL_MAPPING, SCHEMA_VERSION)?;
        let requests = SharedCell::open(SAFETY_REQUEST_MAPPING, SCHEMA_VERSION)
            .context("start etsads-supervisor first")?;
        let previous: ControlCmd = command.latest().unwrap_or_default();
        let last = ControlCmd {
            seq: previous.seq,
            pulses: previous.pulses,
            ..Default::default()
        };
        Ok(Self {
            command,
            requests,
            last,
            actions: actions::ActionExecutor::default(),
            stop,
        })
    }
    fn request(&self, arm: bool) -> anyhow::Result<()> {
        let last = self.requests.latest().unwrap_or_default();
        if arm {
            self.check_running()?;
        }
        self.requests.publish(&SafetyRequest {
            seq: last.seq.saturating_add(1),
            write_qpc: qpc(),
            arm: arm as u32,
            reserved: 0,
        })?;
        if arm && self.stop.load(Ordering::Acquire) {
            self.request(false)?;
            anyhow::bail!("stop arrived during arm publication; disarm requested");
        }
        Ok(())
    }
    fn check_running(&self) -> anyhow::Result<()> {
        ensure!(
            !self.stop.load(Ordering::Acquire),
            "user stopped before command publication"
        );
        Ok(())
    }
    fn neutral(&mut self) -> anyhow::Result<()> {
        self.last.steer = 0.0;
        self.last.throttle = 0.0;
        self.last.brake = 0.0;
        self.last.clutch = 0.0;
        self.last.holds = 0;
        self.last.seq = self.last.seq.wrapping_add(1);
        self.last.write_qpc = qpc();
        self.command.publish(&self.last)?;
        Ok(())
    }
    fn publish(
        &mut self,
        output: &RuntimeOutput,
        snapshot: &GameSnapshot,
        kind: Option<experiment::ExperimentKind>,
        enabled: bool,
        granted: bool,
        mut trace: impl FnMut(&serde_json::Value) -> anyhow::Result<()>,
    ) -> anyhow::Result<()> {
        ensure!(
            output.actuation.valid(),
            "runtime produced invalid actuation"
        );
        let now = qpc();
        if enabled && granted && output.request_control && output.intent.activate {
            actions::validate_gate_activation(snapshot, output)?;
        }
        let batch = self
            .actions
            .prepare(kind, output, &snapshot.raw, enabled, granted, now)?;
        let mut command = self.last;
        command.source_frame_id = output.header.frame_id;
        if enabled && output.request_control && (kind.is_none() || granted) {
            command.steer = output.actuation.steer as f32;
            command.throttle = output.actuation.throttle as f32;
            command.brake = output.actuation.brake as f32;
            if batch.hold_brake {
                command.throttle = 0.0;
                command.brake = command.brake.max(0.4);
            }
        } else {
            command.steer = 0.0;
            command.throttle = 0.0;
            command.brake = 0.0;
        }
        for name in &batch.pulses {
            let index = etsads_game::input::PULSE_NAMES
                .iter()
                .position(|n| n == name)
                .context("semantic action not registered by bridge")?;
            command.pulses[index] = command.pulses[index].wrapping_add(1);
        }
        command.clutch = 0.0;
        command.holds = 0;
        command.seq = command.seq.wrapping_add(1);
        command.write_qpc = now;
        // Enqueue intent/feedback evidence before publishing any pulse. A full
        // trace queue therefore cannot produce an unrecorded semantic action.
        if !batch.events.is_empty() {
            trace(
                &serde_json::json!({"command_seq":command.seq,"write_qpc":now,"events":batch.events,"measured_gear":snapshot.raw.gear,"measured_lights":snapshot.raw.lights,"hold_brake":batch.hold_brake}),
            )?;
        }
        self.check_running()?;
        self.command.publish(&command)?;
        self.last = command;
        if self.stop.load(Ordering::Acquire) {
            self.release();
            anyhow::bail!("stop arrived during command publication; released");
        }
        Ok(())
    }
    fn release(&mut self) {
        let _ = self.neutral();
        let _ = self.request(false);
    }
    fn publish_parking_wait(
        &mut self,
        output: &RuntimeOutput,
        snapshot: &GameSnapshot,
        kind: Option<experiment::ExperimentKind>,
        already_armed: bool,
        granted: bool,
    ) -> anyhow::Result<()> {
        ensure!(
            kind == Some(experiment::ExperimentKind::Parking)
                && output.status == RuntimeStatus::WaitingForParkingPlan
                && output
                    .parking_planning
                    .as_ref()
                    .is_some_and(|d| d.timeout_seconds == 30.0
                        && d.waited_seconds >= 0.0
                        && d.waited_seconds < d.timeout_seconds)
                && !output.request_control
                && snapshot.ego.speed.abs() <= 0.05
                && snapshot.raw.speed_mps.is_finite()
                && snapshot.raw.speed_mps.abs() <= 0.05
                && output.header.frame_id == snapshot.ego.header.frame_id
                && output.actuation.valid()
                && output.actuation.throttle == 0.0
                && (0.4..=1.0).contains(&output.actuation.brake)
                && output.actuation.retarder == 0
                && !output.actuation.activate
                && output.actuation.indicator == 0
                && output.intent.hold_brake
                && output.intent.gear_request.is_none()
                && !output.intent.activate
                && output.intent.indicator == 0,
            "invalid stationary parking wait; motion or semantic actions are forbidden"
        );
        self.actions = Default::default(); // Never carry a deferred pulse across a slow candidate.
        self.last.source_frame_id = output.header.frame_id;
        self.last.steer = if already_armed && granted {
            output.actuation.steer as f32
        } else {
            0.0
        };
        self.last.brake = if already_armed && granted {
            output.actuation.brake as f32
        } else {
            0.0
        };
        self.last.throttle = 0.0;
        self.last.clutch = 0.0;
        self.last.holds = 0;
        self.last.seq = self.last.seq.wrapping_add(1);
        self.last.write_qpc = qpc();
        self.check_running()?;
        self.command.publish(&self.last)?;
        if self.stop.load(Ordering::Acquire) {
            self.release();
            anyhow::bail!("stop arrived during parking hold publication; released");
        }
        Ok(())
    }
}
impl Drop for CommandSink {
    fn drop(&mut self) {
        self.release();
    }
}

struct RecordingSession {
    stop: Arc<AtomicBool>,
    heartbeat: Arc<AtomicU64>,
    failure: Arc<Mutex<Option<String>>>,
    sender: Option<SyncSender<crate::ExtraRecord>>,
    ready: mpsc::Receiver<()>,
    worker: Option<thread::JoinHandle<anyhow::Result<RunMetadata>>>,
}
impl RecordingSession {
    fn start(options: &DriveOptions) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let heartbeat = Arc::new(AtomicU64::new(0));
        let failure = Arc::new(Mutex::new(None));
        let (tx, rx) = mpsc::sync_channel(64);
        let (ready_tx, ready) = mpsc::sync_channel(1);
        let hooks = crate::RecordingHooks {
            extra: rx,
            ready: ready_tx,
            heartbeat_qpc: heartbeat.clone(),
            failure_reason: failure.clone(),
        };
        let output = options.output.clone();
        let duration = options.seconds.saturating_add(15);
        let video = options.video;
        let worker_stop = stop.clone();
        let worker = thread::spawn(move || {
            crate::record_game_with_video_and_trace(
                &output,
                duration,
                &worker_stop,
                video,
                Some(hooks),
            )
        });
        Self {
            stop,
            heartbeat,
            failure,
            sender: Some(tx),
            ready,
            worker: Some(worker),
        }
    }
    fn await_ready(&self, stop: &AtomicBool) -> anyhow::Result<()> {
        let start = Instant::now();
        loop {
            if self.ready.try_recv().is_ok() {
                return Ok(());
            }
            ensure!(
                !stop.load(Ordering::Acquire),
                "cancelled before recorder ready"
            );
            ensure!(
                !self.worker.as_ref().unwrap().is_finished(),
                "mandatory recorder failed before readiness"
            );
            ensure!(
                start.elapsed() < Duration::from_secs(10),
                "mandatory recorder did not become ready within 10 seconds"
            );
            thread::sleep(Duration::from_millis(2));
        }
    }
    fn healthy(&self) -> anyhow::Result<()> {
        ensure!(
            !self.worker.as_ref().unwrap().is_finished(),
            "mandatory recorder worker stopped"
        );
        let heartbeat = self.heartbeat.load(Ordering::Acquire);
        ensure!(
            etsads_safety::fresh(qpc(), heartbeat, qpc_frequency(), 200),
            "mandatory recorder heartbeat exceeded 200 ms"
        );
        Ok(())
    }
    fn record<T: Serialize>(
        &self,
        topic: &str,
        value: &T,
        snapshot: &GameSnapshot,
    ) -> anyhow::Result<()> {
        let record = json_record(value, snapshot)?;
        self.send(crate::ExtraRecord {
            topic: topic.into(),
            record,
        })
    }
    fn send(&self, record: crate::ExtraRecord) -> anyhow::Result<()> {
        match self
            .sender
            .as_ref()
            .context("recording sender closed")?
            .try_send(record)
        {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => {
                anyhow::bail!("mandatory trace queue is full; stopping control")
            }
            Err(TrySendError::Disconnected(_)) => anyhow::bail!("mandatory recorder disconnected"),
        }
    }
    fn fail(&self, reason: String) {
        if let Ok(mut current) = self.failure.lock() {
            *current = Some(reason);
        }
    }
    fn finish(mut self) -> anyhow::Result<RunMetadata> {
        self.sender.take();
        self.stop.store(true, Ordering::Release);
        self.worker
            .take()
            .unwrap()
            .join()
            .map_err(|_| anyhow::anyhow!("mandatory recorder panicked"))?
    }
}
impl Drop for RecordingSession {
    fn drop(&mut self) {
        self.sender.take();
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
fn json_record<T: Serialize>(value: &T, snapshot: &GameSnapshot) -> anyhow::Result<Record> {
    let mut payload = serde_json::to_value(value)?;
    if let serde_json::Value::Object(object) = &mut payload {
        object.insert("encoding".into(), serde_json::json!("json"));
        object.insert("source".into(), serde_json::json!("game"));
    }
    Ok(Record {
        schema_version: SCHEMA_VERSION,
        frame_id: snapshot.raw.header.frame_id,
        sim_time_us: snapshot.raw.header.sim_time_us,
        qpc: snapshot.raw.header.qpc,
        raw_payload: serde_json::to_vec(&payload)?,
        ..Default::default()
    })
}

fn first_snapshot(adapter: &mut GameAdapter, stop: &AtomicBool) -> anyhow::Result<GameSnapshot> {
    let start = Instant::now();
    loop {
        ensure!(
            !stop.load(Ordering::Acquire),
            "cancelled during game connection"
        );
        if let Some(snapshot) = adapter.poll()? {
            ensure!(
                etsads_safety::fresh(qpc(), snapshot.raw.header.qpc, qpc_frequency(), 200),
                "game telemetry is stale"
            );
            return Ok(snapshot);
        }
        ensure!(
            start.elapsed() < Duration::from_secs(2),
            "no fresh game snapshot within two seconds"
        );
        thread::sleep(Duration::from_millis(2));
    }
}

pub fn run(options: DriveOptions, stop: Arc<AtomicBool>) -> anyhow::Result<DriveReport> {
    run_internal(options, stop, None)
}
/// Optional lossy UI observations; a blocked/disconnected UI never blocks the
/// control loop or changes arm, gate, recording or cancellation semantics.
pub fn run_with_progress(
    options: DriveOptions,
    stop: Arc<AtomicBool>,
    progress: SyncSender<DriveProgress>,
) -> anyhow::Result<DriveReport> {
    run_internal(options, stop, Some(progress))
}
fn run_internal(
    options: DriveOptions,
    stop: Arc<AtomicBool>,
    progress: Option<SyncSender<DriveProgress>>,
) -> anyhow::Result<DriveReport> {
    ensure!(
        (1..=86_400).contains(&options.seconds),
        "drive duration must be 1..86400 seconds"
    );
    ensure!(
        options.region_radius_m.is_finite() && (1.0..=5000.0).contains(&options.region_radius_m),
        "invalid map radius"
    );
    ensure!(
        options.map.is_some() == options.policy.is_some(),
        "map and policy must be supplied together"
    );
    let mut config: RuntimeConfig = read_config(&options.config, false)?;
    config.execution = ExecutionMode::Live;
    let road_mode = matches!(&config.route_source,etsads_runtime::RouteSource::NativeGps(settings) if settings.road_evidence.is_some());
    ensure!(
        road_mode == options.road_evidence.is_some(),
        "road operation requires --road-evidence with the matching road configuration; commissioning does not consume road evidence"
    );
    if let etsads_runtime::RouteSource::NativeGps(settings) = &config.route_source
        && let Some(evidence) = &settings.road_evidence
    {
        for (label, path) in [
            ("road centerline", &evidence.centerline_evidence_path),
            (
                "road physical boundaries",
                &evidence.physical_boundary_evidence_path,
            ),
            (
                "road vehicle access",
                &evidence.vehicle_access_evidence_path,
            ),
        ] {
            let path = Path::new(path);
            let resolved = if path.is_absolute() {
                path.to_owned()
            } else {
                options.config.parent().unwrap_or(Path::new(".")).join(path)
            };
            ensure!(
                resolved.is_file(),
                "{label} evidence file is missing: {}",
                resolved.display()
            );
        }
    }
    ensure!(
        matches!(
            (&config.mode, options.experiment),
            (
                DrivingMode::CommissioningCircle { .. } | DrivingMode::LaneFollow { .. },
                None
            ) | (
                DrivingMode::ControlledRoute { .. },
                Some(experiment::ExperimentKind::ControlledRoute)
            ) | (
                DrivingMode::Parking { .. },
                Some(experiment::ExperimentKind::Parking)
            )
        ),
        "advanced RuntimeConfig modes require the matching explicit --experiment controlled-route|parking; P0/P1 use no --experiment"
    );
    ensure!(
        options.experiment.is_some()
            || (options.experiment_evidence.is_none() && options.safety_profile.is_none()),
        "experiment evidence/profile options require explicit --experiment"
    );
    let calibration: VehicleCalibration = read_config(
        &options.calibration,
        options.calibration.extension().and_then(|x| x.to_str()) != Some("json"),
    )?;
    calibration.validate_measured_envelope()?;
    let calibration_directory = options.calibration.parent().unwrap_or(Path::new("."));
    let references = [
        calibration
            .reference
            .as_ref()
            .map(|r| ("rear axle reference", r.evidence_path.as_str())),
        calibration
            .conditions
            .as_ref()
            .map(|r| ("trailer/load applicability", r.evidence_path.as_str())),
        calibration
            .front_bumper
            .as_ref()
            .map(|r| ("front bumper measurement", r.evidence_path.as_str())),
        calibration
            .envelope_measurement
            .as_ref()
            .map(|r| ("measured envelope", r.evidence_path.as_str())),
    ];
    for (label, path) in references.into_iter().flatten() {
        let declared = Path::new(path);
        let evidence = if declared.is_absolute() {
            declared.to_owned()
        } else {
            calibration_directory.join(declared)
        };
        ensure!(
            evidence.is_file(),
            "{label} calibration evidence is missing: {}",
            evidence.display()
        );
    }
    let source_run = PathBuf::from(&calibration.source_run);
    let source_run = if source_run.exists() {
        source_run
    } else {
        options
            .calibration
            .parent()
            .unwrap_or(Path::new("."))
            .join(source_run)
    };
    let evidence: RunMetadata = read_config(&source_run.join("meta.json"), false)
        .context("live calibration must reference an existing game recording")?;
    ensure!(
        evidence.source == "game" && evidence.completed,
        "live calibration requires a completed real-game recording"
    );
    let game_directory = crate::find_game()
        .context("could not locate installed ETS2 for PE version verification")?;
    let game_version =
        etsads_game::version::file_version(&game_directory.join("bin/win_x64/eurotrucks2.exe"))
            .context("actual game PE version unavailable")?;
    ensure!(
        evidence.game_version == game_version,
        "calibration game version {} does not match installed {}",
        evidence.game_version,
        game_version
    );
    if !matches!(config.mode, DrivingMode::CommissioningCircle { .. }) {
        ensure!(
            etsads_game::version::extension_version_supported(&game_version),
            "ETS2LA extension is only verified for game 1.61.x"
        );
        let plugin = game_directory.join("bin/win_x64/plugins/ets2la_plugin.dll");
        ensure!(
            plugin_hash(&plugin)?.eq_ignore_ascii_case(
                "0E1893719F84F28079857451B82A22A263401D98B8FC299CD561EB7695BFA88F"
            ),
            "ETS2LA plugin differs from the pinned 1.61 build"
        );
        let map = options
            .map
            .as_ref()
            .context("route and parking runs require an exported map")?;
        let manifest_path = experiment::map_manifest_path(map);
        let manifest: serde_json::Value = read_config(&manifest_path, false)
            .context("map manifest must include verified PE game version")?;
        ensure!(
            manifest["game_version"].as_str() == Some(game_version.as_str()),
            "map manifest game_version is absent or differs from the installed game"
        );
    }
    let experiment = options
        .experiment
        .map(|_| experiment::verify(&options, &config, &calibration, &game_version))
        .transpose()?;
    let road_evidence = road_mode
        .then(|| experiment::verify_road_evidence(&options, &config, &calibration, &game_version))
        .transpose()?;
    let _writer = SingleWriterGuard::acquire("ETSADS_Control_Writer")?;
    let mut sink = CommandSink::connect(stop.clone())?;
    sink.release();
    let gate = SharedCell::<Gate>::open(GATE_MAPPING, SCHEMA_VERSION)?;
    let info = SharedCell::<BridgeInfo>::open(INFO_MAPPING, SCHEMA_VERSION)
        .context("bridge metadata unavailable; deploy current bridge")?;
    let identity = info
        .latest()
        .context("bridge metadata is not initialized")?;
    ensure!(
        identity.telemetry_api_version == 0x10001
            && identity.input_api_version == 0x10000
            && identity.process_id != 0,
        "live telemetry/input plugin is not active"
    );
    let applied = SharedCell::<AppliedControl>::open(APPLIED_MAPPING, SCHEMA_VERSION)?;
    let supervisor_status = if experiment.is_some() {
        Some(SharedCell::<SupervisorStatus>::open(SUPERVISOR_STATUS_MAPPING,SCHEMA_VERSION)
            .context("advanced experiment requires current independent supervisor status; start the current etsads-supervisor")?)
    } else {
        None
    };
    ensure!(
        gate.latest().is_some_and(|g| etsads_safety::fresh(
            qpc(),
            g.heartbeat_qpc,
            qpc_frequency(),
            200
        )),
        "independent supervisor heartbeat unavailable"
    );
    let mut adapter = GameAdapter::connect()?;
    adapter.require_extension = !matches!(config.mode, DrivingMode::CommissioningCircle { .. });
    let first = first_snapshot(&mut adapter, &stop)?;
    let loaded_controls = if options.arm {
        let mut controls = crate::loaded_controls::LoadedControls::connect("")?;
        controls.bind(&first.raw)?;
        Some(controls)
    } else {
        // Shadow runs never arm or emit nonzero axes/buttons. Unsupported
        // controls are still useful observations and must remain recordable.
        None
    };
    if let Some(evidence) = &experiment {
        let status = supervisor_status
            .as_ref()
            .unwrap()
            .latest()
            .context("independent supervisor has not published status")?;
        ensure!(
            status.process_id != identity.process_id && status.process_id != std::process::id(),
            "supervisor must run independently from game and driving host"
        );
        evidence.validate_supervisor(
            &status,
            first
                .config
                .as_ref()
                .context("experiment requires measured vehicle configuration")?,
            qpc(),
            qpc_frequency(),
        )?;
    }
    let mut map_session = None;
    let mut navigation_region = None;
    let region =
        if let (Some(map), Some(policy_path)) = (&options.map, &options.policy) {
            let mut policy: RegionPolicy = read_config(policy_path, false)?;
            let path = PathBuf::from(&policy.evidence_path);
            let path = if path.is_absolute() {
                path
            } else {
                policy_path.parent().unwrap_or(Path::new(".")).join(path)
            };
            ensure!(
                path.exists(),
                "regional verification evidence is missing: {}",
                path.display()
            );
            if matches!(config.route_source, etsads_runtime::RouteSource::Explicit) {
                policy.evidence_path = path.to_string_lossy().into_owned();
            }
            if matches!(
                config.route_source,
                etsads_runtime::RouteSource::NativeGps(_)
            ) {
                let (session, navigation) = map_session::MapSession::start(
                    map,
                    policy,
                    &config,
                    options.config.parent().unwrap_or(Path::new(".")),
                    &first,
                )?;
                let region = navigation.region.clone();
                map_session = Some(session);
                navigation_region = Some(navigation);
                Some(region)
            } else {
                let exported = if map.is_dir() {
                    load_parquet_region(
                        map,
                        Point3 {
                            x: first.ego.pose.x,
                            y: first.ego.pose.y,
                            z: first.ego.z,
                        },
                        options.region_radius_m,
                    )?
                } else {
                    ExportedMap::from_path(map)?
                };
                ensure!(
                    exported.source == "installed_game_archives",
                    "live map must come from installed game archives"
                );
                Some(exported.into_commissioning_region(
                    policy,
                    first.raw.navigation_speed_limit_mps as f64,
                )?)
            }
        } else {
            None
        };
    ensure!(
        !matches!(config.mode, DrivingMode::LaneFollow { .. }) || region.is_some(),
        "lane follow requires map and verified region policy"
    );
    let permission = if let Some(evidence) = &experiment {
        let kind = match evidence.document.kind {
            experiment::ExperimentKind::ControlledRoute => {
                etsads_runtime::LiveExperimentKind::ControlledRoute
            }
            experiment::ExperimentKind::Parking => etsads_runtime::LiveExperimentKind::Parking,
        };
        let mut permission = etsads_runtime::VerifiedLiveExperiment::from_host_verification(
            &config,
            &calibration,
            region.as_ref(),
            kind,
            evidence.digest,
            evidence.profile_digest,
        )?;
        if let Some(navigation) = &navigation_region {
            evidence.validate_navigation_scope(&config, navigation)?;
            permission = permission
                .with_native_scope(navigation.scope_digest().map_err(anyhow::Error::msg)?)?;
        }
        Some(permission)
    } else {
        None
    };
    let mut runtime = if let Some(evidence) = &road_evidence {
        let navigation = navigation_region
            .as_ref()
            .context("road mode requires an authenticated native map window")?;
        evidence.validate_navigation_scope(&config, navigation)?;
        let road = etsads_runtime::VerifiedRoadOperation::from_host_verification(
            &config,
            &calibration,
            navigation.scope_digest().map_err(anyhow::Error::msg)?,
            evidence.digest,
        )?;
        Runtime::new_live_road(
            config.clone(),
            calibration.clone(),
            region,
            road,
            permission,
        )?
    } else if let Some(permission) = permission {
        Runtime::new_live_experiment(config.clone(), calibration.clone(), region, permission)?
    } else {
        Runtime::new(config.clone(), calibration.clone(), region)?
    };
    let recorded_road_boundary = navigation_region
        .as_ref()
        .and_then(|n| n.road_boundary.clone());
    if let Some(navigation) = navigation_region {
        runtime.install_native_region(navigation)?;
    }
    let capabilities = runtime.capabilities();
    eprintln!(
        "Live capabilities: {}. arm requested={}",
        capabilities.join("; "),
        options.arm
    );
    let recording = RecordingSession::start(&options);
    let mut frames = 0;
    let mut active_frames = 0;
    let mut arm_requested = false;
    let mut last_header = first.raw.header;
    let result = (|| -> anyhow::Result<String> {
        recording.await_ready(&stop)?;
        recording.record("/runtime_config",&serde_json::json!({"runtime":config,"calibration":calibration,"arm_requested":options.arm,"video":options.video,"capabilities":capabilities}),&first)?;
        if let Some(evidence) = &experiment {
            recording.record("/experiment_evidence",&serde_json::json!({"document":evidence.document,"sha256":evidence.digest,"scope":"bounded development experiment only; not stage acceptance"}),&first)?;
        }
        if let Some(evidence) = &road_evidence {
            recording.record("/road_evidence",&serde_json::json!({"document":evidence.document,"sha256":evidence.digest,"phase_acceptance":false}),&first)?;
        }
        if let Some(boundary) = &recorded_road_boundary {
            recording.record("/road_boundary", boundary, &first)?;
        }
        let start = Instant::now();
        let mut last_snapshot = Instant::now();
        let mut arm_at = None;
        let mut gate_was_enabled = false;
        let mut last_progress = Instant::now() - Duration::from_millis(100);
        loop {
            if stop.load(Ordering::Acquire) {
                return Ok("Ctrl+C / user stop".into());
            }
            if start.elapsed().as_secs() >= options.seconds {
                return Ok("requested duration completed".into());
            }
            recording.healthy()?;
            ensure!(
                info.latest()
                    .is_some_and(|i| i.session_start_qpc == identity.session_start_qpc
                        && i.input_api_version != 0
                        && i.telemetry_api_version != 0),
                "SDK session changed or stopped"
            );
            let permission = crate::loaded_controls::coherent_read(|| gate.latest())
                .context("supervisor Gate unavailable")?;
            let input_applied = crate::loaded_controls::coherent_read(|| applied.latest());
            let now = qpc();
            ensure!(
                etsads_safety::fresh(now, permission.heartbeat_qpc, qpc_frequency(), 200),
                "supervisor heartbeat exceeded 200 ms"
            );
            if arm_requested {
                if permission.enabled == 1 {
                    gate_was_enabled = true;
                } else if gate_was_enabled {
                    return Ok("supervisor disarmed or manual takeover".into());
                } else {
                    ensure!(
                        arm_at.is_some_and(|at: Instant| at.elapsed() < Duration::from_millis(300)),
                        "supervisor did not grant the single arm request"
                    );
                }
                ensure!(
                    input_applied.is_some_and(|a| etsads_safety::fresh(
                        now,
                        a.applied_qpc,
                        qpc_frequency(),
                        200
                    )),
                    "game input callback heartbeat exceeded 200 ms"
                );
            } else {
                ensure!(
                    permission.enabled == 0,
                    "supervisor armed externally before this run's explicit request"
                );
            }
            let Some(snapshot) = adapter.poll()? else {
                ensure!(
                    last_snapshot.elapsed() < Duration::from_millis(200),
                    "physical snapshots stalled for 200 ms"
                );
                thread::sleep(Duration::from_millis(1));
                continue;
            };
            last_snapshot = Instant::now();
            ensure!(
                etsads_safety::fresh(qpc(), snapshot.raw.header.qpc, qpc_frequency(), 200),
                "physical snapshot is older than 200 ms"
            );
            if let Some(evidence) = &experiment {
                let status = supervisor_status
                    .as_ref()
                    .unwrap()
                    .latest()
                    .context("independent supervisor status unavailable")?;
                ensure!(
                    status.process_id != identity.process_id
                        && status.process_id != std::process::id(),
                    "supervisor independence changed"
                );
                evidence.validate_supervisor(
                    &status,
                    snapshot
                        .config
                        .as_ref()
                        .context("experiment vehicle configuration disappeared")?,
                    qpc(),
                    qpc_frequency(),
                )?;
            }
            frames += 1;
            last_header = snapshot.raw.header;
            if let Some(session) = &mut map_session
                && let Err(error) = session.poll(&snapshot, &mut runtime)
            {
                runtime.report_navigation_failure(format!("{error:#}"));
            }
            let output = runtime.step(&snapshot);
            if matches!(
                output.status,
                RuntimeStatus::Fault | RuntimeStatus::Paused | RuntimeStatus::Completed
            ) {
                sink.release();
            }
            recording.record("/scene", &snapshot.scene, &snapshot)?;
            recording.record("/stack", &output, &snapshot)?;
            if let Some(trajectory) = &output.trajectory {
                recording.record("/trajectory", trajectory, &snapshot)?;
            }
            let mut control_sent = false;
            // A profile file can lag the loaded game settings. Validate the
            // newest SDK controls generation before any running/hold command.
            if let Some(loaded_controls) = &loaded_controls
                && !matches!(
                    output.status,
                    RuntimeStatus::Fault | RuntimeStatus::Paused | RuntimeStatus::Completed
                )
            {
                loaded_controls.validate(&snapshot.raw)?;
            }
            if stop.load(Ordering::Acquire) {
                return Ok("Ctrl+C / user stop before publication".into());
            }
            match output.status {
                RuntimeStatus::Fault => anyhow::bail!("runtime fault: {}", output.reason),
                RuntimeStatus::Paused => return Ok("game paused; run stopped and disarmed".into()),
                RuntimeStatus::Completed => {
                    return Ok(format!("runtime completed: {}", output.reason));
                }
                RuntimeStatus::WaitingForSecondFrame | RuntimeStatus::WaitingForTrajectory => {
                    ensure!(
                        !arm_requested,
                        "runtime stopped requesting control after arming"
                    );
                    sink.publish(
                        &output,
                        &snapshot,
                        options.experiment,
                        false,
                        false,
                        |event| recording.record("/actuator_event", event, &snapshot),
                    )?;
                }
                RuntimeStatus::WaitingForParkingPlan => {
                    // Initial slow planning never requests arm. After a
                    // previously granted run, only a current brake hold is
                    // allowed; Gate/manual/recorder watchdogs above stay active.
                    sink.publish_parking_wait(
                        &output,
                        &snapshot,
                        options.experiment,
                        options.arm && arm_requested,
                        permission.enabled == 1,
                    )?;
                }
                RuntimeStatus::Running => {
                    ensure!(
                        output.request_control,
                        "running runtime did not request control"
                    );
                    sink.publish(
                        &output,
                        &snapshot,
                        options.experiment,
                        options.arm,
                        arm_requested && permission.enabled == 1,
                        |event| recording.record("/actuator_event", event, &snapshot),
                    )?;
                    control_sent = options.arm
                        && arm_requested
                        && permission.enabled == 1
                        && output.request_control;
                    if options.arm && !arm_requested {
                        if stop.load(Ordering::Acquire) {
                            return Ok("Ctrl+C / user stop before arm request".into());
                        }
                        sink.request(true)?;
                        arm_requested = true;
                        arm_at = Some(Instant::now());
                    }
                    if options.arm && permission.enabled == 1 {
                        active_frames += 1;
                    }
                }
            }
            if last_progress.elapsed() >= Duration::from_millis(100) {
                if let Some(progress) = &progress {
                    let _ = progress.try_send(DriveProgress {
                        produced_at: Instant::now(),
                        frame_id: snapshot.raw.header.frame_id,
                        status: output.status,
                        reason: output.reason.clone(),
                        arm_requested,
                        control_sent,
                        speed_mps: snapshot.ego.speed,
                    });
                }
                last_progress = Instant::now();
            }
        }
    })();
    sink.release();
    let stop_event = serde_json::json!({"encoding":"json","source":"game","stop_reason":result.as_ref().map(|s|s.clone()).unwrap_or_else(|e|format!("{e:#}")),"frames":frames,"active_frames":active_frames,"arm_requested":arm_requested});
    if let Err(error) = recording.send(crate::ExtraRecord {
        topic: "/drive_event".into(),
        record: Record {
            schema_version: SCHEMA_VERSION,
            frame_id: last_header.frame_id,
            sim_time_us: last_header.sim_time_us,
            qpc: qpc(),
            raw_payload: serde_json::to_vec(&stop_event)?,
            ..Default::default()
        },
    }) {
        recording.fail(format!("stop event could not be recorded: {error:#}"));
    }
    if let Err(error) = &result {
        recording.fail(format!("live control stopped: {error:#}"));
    }
    let recorded = recording.finish();
    let reason = match result {
        Ok(reason) => reason,
        Err(error) => {
            if let Err(record_error) = recorded {
                return Err(error.context(format!("recording finalization: {record_error:#}")));
            }
            return Err(error);
        }
    };
    let recording = recorded?;
    Ok(DriveReport {
        source: "game",
        output: options.output,
        stop_reason: reason,
        frames,
        active_frames,
        arm_requested,
        capabilities,
        recording,
    })
}

fn plugin_hash(path: &Path) -> anyhow::Result<String> {
    use sha2::{Digest, Sha256};
    use std::io::Read;
    let mut file = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut hash = Sha256::new();
    let mut bytes = [0u8; 65536];
    loop {
        let count = file.read(&mut bytes)?;
        if count == 0 {
            break;
        }
        hash.update(&bytes[..count]);
    }
    Ok(format!("{:x}", hash.finalize()))
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    fn isolated_sink() -> (CommandSink, SharedCell<ControlCmd>) {
        let name = format!("ETSADS_ExperimentTest_{}_{}", std::process::id(), qpc());
        let command = SharedCell::create(&name, SCHEMA_VERSION).unwrap();
        let requests = SharedCell::create(&format!("{name}_Request"), SCHEMA_VERSION).unwrap();
        let reader = SharedCell::open(&name, SCHEMA_VERSION).unwrap();
        (
            CommandSink {
                command,
                requests,
                last: ControlCmd::default(),
                actions: actions::ActionExecutor::default(),
                stop: Arc::new(AtomicBool::new(false)),
            },
            reader,
        )
    }
    #[test]
    fn stop_during_trace_prevents_pulse_publication_and_later_arm() {
        let (mut sink, reader) = isolated_sink();
        sink.neutral().unwrap();
        let before = reader.latest().unwrap();
        let mut output = actions::tests::output();
        output.intent.indicator = 1;
        output.actuation.indicator = 1;
        let snapshot = etsads_game::adapter::snapshot_from_recorded_frame(
            TelemetryFrame {
                qpc_frequency: qpc_frequency(),
                ..Default::default()
            },
            None,
            None,
        )
        .unwrap();
        let stop = sink.stop.clone();
        let result = sink.publish(
            &output,
            &snapshot,
            Some(experiment::ExperimentKind::ControlledRoute),
            true,
            true,
            |_| {
                stop.store(true, Ordering::Release);
                Ok(())
            },
        );
        assert!(result.unwrap_err().to_string().contains("stopped"));
        assert_eq!(reader.latest().unwrap().seq, before.seq);
        assert!(reader.latest().unwrap().pulses.iter().all(|n| *n == 0));
        assert!(sink.request(true).is_err());
        assert!(sink.requests.latest().is_none());
        drop(sink);
        assert_eq!(reader.latest().unwrap().throttle, 0.0);
    }
    #[test]
    fn isolated_parking_wait_never_arms_or_replays_actions_and_only_holds_existing_permission() {
        let (mut sink, reader) = isolated_sink();
        let mut output = actions::tests::output();
        output.status = RuntimeStatus::WaitingForParkingPlan;
        output.request_control = false;
        output.actuation.brake = 0.4;
        output.intent.hold_brake = true;
        output.parking_planning = Some(etsads_runtime::ParkingPlanningDiagnostic {
            phase: etsads_runtime::ParkingPlanningPhase::Searching,
            original_frame_id: 0,
            request_frame_id: 0,
            waited_seconds: 1.0,
            timeout_seconds: 30.0,
        });
        let raw = TelemetryFrame {
            qpc_frequency: qpc_frequency(),
            ..Default::default()
        };
        let snapshot = etsads_game::adapter::snapshot_from_recorded_frame(raw, None, None).unwrap();
        let kind = Some(experiment::ExperimentKind::Parking);
        sink.publish_parking_wait(&output, &snapshot, kind, false, false)
            .unwrap();
        assert_eq!(reader.latest().unwrap().brake, 0.0);
        assert!(
            sink.requests.latest().is_none(),
            "waiting must not create an arm request"
        );
        let mut old = actions::tests::output();
        old.intent.gear_request = Some(etsads_msgs::Direction::Reverse);
        old.intent.hold_brake = true;
        sink.actions
            .prepare(kind, &old, &snapshot.raw, true, false, qpc())
            .unwrap();
        sink.request(true).unwrap();
        let request = sink.requests.latest().unwrap();
        sink.publish_parking_wait(&output, &snapshot, kind, true, true)
            .unwrap();
        let command = reader.latest().unwrap();
        assert_eq!(command.throttle, 0.0);
        assert_eq!(command.brake, 0.4);
        assert_eq!(command.holds, 0);
        assert!(command.pulses.iter().all(|v| *v == 0));
        assert_eq!(sink.requests.latest().unwrap().seq, request.seq);
        assert!(
            sink.actions
                .prepare(kind, &output, &snapshot.raw, true, true, qpc())
                .unwrap()
                .pulses
                .is_empty(),
            "old gear request must not survive wait"
        );
        output.actuation.throttle = 0.01;
        assert!(
            sink.publish_parking_wait(&output, &snapshot, kind, true, true)
                .is_err()
        );
        output.actuation.throttle = 0.0;
        let mut moving = snapshot.clone();
        moving.raw.speed_mps = 0.2;
        moving.ego.speed = 0.2;
        assert!(
            sink.publish_parking_wait(&output, &moving, kind, true, true)
                .is_err()
        );
        assert!(
            sink.publish_parking_wait(&output, &snapshot, None, true, true)
                .is_err()
        );
        drop(sink);
        assert_eq!(reader.latest().unwrap().brake, 0.0);
    }
    #[test]
    fn isolated_pulse_publication_is_shadow_safe_and_fails_before_trace_loss() {
        let (mut sink, reader) = isolated_sink();
        let mut out = actions::tests::output();
        out.intent.indicator = 1;
        out.actuation.indicator = 1;
        let raw = TelemetryFrame {
            qpc_frequency: qpc_frequency(),
            ..Default::default()
        };
        let snapshot = etsads_game::adapter::snapshot_from_recorded_frame(raw, None, None).unwrap();
        sink.publish(
            &out,
            &snapshot,
            Some(experiment::ExperimentKind::ControlledRoute),
            false,
            false,
            |_| Ok(()),
        )
        .unwrap();
        assert!(reader.latest().unwrap().pulses.iter().all(|n| *n == 0));
        assert!(
            sink.publish(
                &out,
                &snapshot,
                Some(experiment::ExperimentKind::ControlledRoute),
                true,
                true,
                |_| anyhow::bail!("synthetic trace queue full")
            )
            .is_err()
        );
        assert!(reader.latest().unwrap().pulses.iter().all(|n| *n == 0));
        // A failed run is dropped; it must not publish its unrecorded counter.
        drop(sink);
        assert!(reader.latest().unwrap().pulses.iter().all(|n| *n == 0));
    }
    #[test]
    fn replay_parking_gear_event_reaches_isolated_ipc_once_and_waits_for_feedback() {
        use etsads_control::identification::{LongitudinalModel, SteeringRow};
        use etsads_msgs::{Pose2, VehicleGeometry};
        use etsads_runtime::parking_mission::{
            ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary,
        };
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_wheelbase: 0.0,
            trailer_length: 0.0,
            ..Default::default()
        };
        let calibration = VehicleCalibration {
            dynamic_speed_range_mps: None,
            schema_version: 1,
            vehicle_id: "synthetic-ipc-fixture".into(),
            source_run: "synthetic; no game recording".into(),
            evidence: "synthetic replay only".into(),
            geometry,
            geometry_fingerprint: None,
            steering_axis_source: "synthetic".into(),
            reference: None,
            conditions: None,
            front_bumper: None,
            envelope_measurement: None,
            sample_count: 100,
            steering_rows: [0.0, 10.0]
                .into_iter()
                .map(|speed| SteeringRow {
                    speed,
                    samples: vec![(-1.0, -0.2), (0.0, 0.0), (1.0, 0.2)],
                    counts: vec![100; 3],
                    rms_curvature: 0.0,
                })
                .collect(),
            delay_s: Some(0.1),
            steering_tau_s: Some(0.1),
            delay_correlation: Some(1.0),
            longitudinal: Some(LongitudinalModel {
                coefficients: [0.0, 1.2, -4.0, 0.0, 0.0],
                training_rmse: 0.0,
                validation_rmse: 0.0,
                sample_count: 100,
            }),
            accepted_for_control: true,
            issues: vec![],
        };
        let config = RuntimeConfig {
            forward_lateral: Default::default(),
            route_source: Default::default(),
            execution: ExecutionMode::Replay,
            vehicle_id: calibration.vehicle_id.clone(),
            vehicle_configuration_verified: true,
            mode: DrivingMode::Parking {
                mission: ParkingMission {
                    recovery: Default::default(),
                    target: ParkingTarget {
                        source_id: "synthetic rear axle target".into(),
                        pose: Pose2 {
                            x: -6.0,
                            y: 0.0,
                            yaw: 0.0,
                        },
                        reference: SlotReference::TractorRearAxle,
                    },
                    boundary: VerifiedBoundary {
                        polygon: vec![(-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0)],
                        physical_boundaries_verified: true,
                        evidence_path: "synthetic replay boundary".into(),
                    },
                    static_obstacles: vec![],
                    max_expansions: 10000,
                    maximum_duration_s: 30.0,
                },
            },
        };
        let mut runtime = Runtime::new(config, calibration, None).unwrap();
        let (mut sink, reader) = isolated_sink();
        let reverse_index = etsads_game::input::PULSE_NAMES
            .iter()
            .position(|n| *n == "gearreverse")
            .unwrap();
        let mut gear = 0;
        let mut acknowledged = false;
        let mut pulse_observed = false;
        for frame in 1..=12 {
            let mut raw = TelemetryFrame {
                qpc_frequency: qpc_frequency(),
                paused_simulation_time_us: frame * 16667,
                gear,
                ..Default::default()
            };
            raw.header = WireHeader {
                frame_id: frame,
                sim_time_us: frame * 16667,
                qpc: qpc(),
                ..Default::default()
            };
            raw.orientation_turns[0] = 0.75;
            let snapshot =
                etsads_game::adapter::snapshot_from_recorded_frame(raw, None, None).unwrap();
            let output = runtime.step(&snapshot);
            assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
            sink.publish(
                &output,
                &snapshot,
                Some(experiment::ExperimentKind::Parking),
                true,
                true,
                |event| {
                    acknowledged |= event["events"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .any(|e| e["kind"] == "gear_acknowledged");
                    Ok(())
                },
            )
            .unwrap();
            let command = reader.latest().unwrap();
            if command.pulses[reverse_index] > 0 {
                pulse_observed = true;
                gear = -1;
            }
            assert!(
                command.pulses[reverse_index] <= 1,
                "gear pulse must not repeat each physical frame"
            );
        }
        assert!(pulse_observed && acknowledged);
    }
    #[test]
    fn command_sink_drop_releases_axes_and_requests_disarm() {
        let name = format!("ETSADS_DriveTest_{}_{}", std::process::id(), qpc());
        let command = SharedCell::<ControlCmd>::create(&name, SCHEMA_VERSION).unwrap();
        let requests =
            SharedCell::<SafetyRequest>::create(&format!("{name}_Request"), SCHEMA_VERSION)
                .unwrap();
        let reader = SharedCell::<ControlCmd>::open(&name, SCHEMA_VERSION).unwrap();
        let request_reader =
            SharedCell::<SafetyRequest>::open(&format!("{name}_Request"), SCHEMA_VERSION).unwrap();
        {
            let mut sink = CommandSink {
                command,
                requests,
                last: ControlCmd {
                    throttle: 0.7,
                    source_frame_id: 7,
                    ..Default::default()
                },
                actions: actions::ActionExecutor::default(),
                stop: Arc::new(AtomicBool::new(false)),
            };
            sink.command.publish(&sink.last).unwrap();
            sink.request(true).unwrap();
            sink.neutral().unwrap();
            sink.last.throttle = 0.8;
            sink.command.publish(&sink.last).unwrap();
        }
        let final_command = reader.latest().unwrap();
        assert_eq!(
            (
                final_command.steer,
                final_command.throttle,
                final_command.brake,
                final_command.clutch
            ),
            (0.0, 0.0, 0.0, 0.0)
        );
        assert_eq!(request_reader.latest().unwrap().arm, 0);
    }
    #[test]
    fn bounded_recording_backpressure_is_a_stop_error() {
        let (tx, rx) = mpsc::sync_channel(1);
        let (ready_tx, ready) = mpsc::sync_channel(1);
        drop(ready_tx);
        let session = RecordingSession {
            stop: Arc::new(AtomicBool::new(false)),
            heartbeat: Arc::new(AtomicU64::new(qpc())),
            failure: Arc::new(Mutex::new(None)),
            sender: Some(tx),
            ready,
            worker: None,
        };
        session
            .send(crate::ExtraRecord {
                topic: "/stack".into(),
                record: Record::default(),
            })
            .unwrap();
        assert!(
            session
                .send(crate::ExtraRecord {
                    topic: "/stack".into(),
                    record: Record::default()
                })
                .is_err()
        );
        drop(rx);
        assert!(
            session
                .send(crate::ExtraRecord {
                    topic: "/stack".into(),
                    record: Record::default()
                })
                .is_err()
        );
    }
}
