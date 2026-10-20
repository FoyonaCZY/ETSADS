//! Shared application services for the native UI and command line.
pub mod alignment;
pub mod assist;
pub mod driving;
pub mod environment;
pub mod hud;
mod loaded_controls;
mod recording_loss;
pub mod setup;
mod sidecar_recording;
pub mod video;
use anyhow::{Context, ensure};
use etsads_msgs::protocol::*;
use etsads_recorder::{Record, Recorder, RunMetadata};
use etsads_shm::{SharedCell, SharedRing, qpc, qpc_frequency};
use std::{
    path::{Path, PathBuf},
    time::Duration,
};

pub fn find_game() -> Option<PathBuf> {
    if let Some(path) = std::env::var_os("ETSADS_GAME_DIR")
        .map(PathBuf::from)
        .filter(|p| p.join("bin/win_x64/eurotrucks2.exe").exists())
    {
        return Some(path);
    }
    let mut roots = vec![
        PathBuf::from("C:/Program Files (x86)/Steam"),
        PathBuf::from("D:/SteamLibrary"),
    ];
    // Steam may be installed anywhere; its own registry entry says where.
    if let Some(steam) = steam_path_from_registry() {
        roots.insert(0, steam);
    }
    let libraries = roots
        .iter()
        .find_map(|r| std::fs::read_to_string(r.join("steamapps/libraryfolders.vdf")).ok());
    if let Some(libraries) = libraries {
        for line in libraries.lines() {
            let parts: Vec<_> = line.split('"').collect();
            if parts.get(1) == Some(&"path")
                && let Some(path) = parts.get(3)
            {
                roots.push(PathBuf::from(path.replace("\\\\", "\\")));
            }
        }
    }
    roots
        .into_iter()
        .map(|p| p.join("steamapps/common/Euro Truck Simulator 2"))
        .find(|p| p.join("bin/win_x64/eurotrucks2.exe").exists())
}
fn steam_path_from_registry() -> Option<PathBuf> {
    let mut command = std::process::Command::new("reg");
    command.args(["query", r"HKCU\Software\Valve\Steam", "/v", "SteamPath"]);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x0800_0000);
    }
    let output = command.output().ok()?;
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .find(|l| l.contains("SteamPath"))
        .and_then(|l| l.split("REG_SZ").nth(1))
        .map(|p| PathBuf::from(p.trim()))
        .filter(|p| p.is_dir())
}
pub fn set_gate(arm: bool) -> anyhow::Result<()> {
    let request = SharedCell::<SafetyRequest>::open(SAFETY_REQUEST_MAPPING, SCHEMA_VERSION)
        .context("请先启动 etsads-supervisor 安全监督进程")?;
    request.publish(&SafetyRequest {
        seq: request.latest().map_or(1, |v| v.seq.saturating_add(1)),
        write_qpc: qpc(),
        arm: u32::from(arm),
        reserved: 0,
    })?;
    std::thread::sleep(Duration::from_millis(30));
    Ok(())
}
pub fn record_game(
    output: &Path,
    seconds: u64,
    stop: &std::sync::atomic::AtomicBool,
) -> anyhow::Result<RunMetadata> {
    record_game_with_video(output, seconds, stop, false)
}
pub fn record_game_with_video(
    output: &Path,
    seconds: u64,
    stop: &std::sync::atomic::AtomicBool,
    video: bool,
) -> anyhow::Result<RunMetadata> {
    record_game_with_video_and_trace(output, seconds, stop, video, None)
}

pub struct ExtraRecord {
    pub topic: String,
    pub record: Record,
}
pub struct RecordingHooks {
    pub extra: std::sync::mpsc::Receiver<ExtraRecord>,
    pub ready: std::sync::mpsc::SyncSender<()>,
    pub heartbeat_qpc: std::sync::Arc<std::sync::atomic::AtomicU64>,
    pub failure_reason: std::sync::Arc<std::sync::Mutex<Option<String>>>,
}
fn drain_extra(recorder: &mut Recorder, hooks: &Option<RecordingHooks>) -> anyhow::Result<()> {
    if let Some(hooks) = hooks {
        for extra in hooks.extra.try_iter() {
            ensure!(
                matches!(
                    extra.topic.as_str(),
                    "/scene" | "/trajectory" | "/stack" | "/runtime_config" | "/drive_event"
                ),
                "unsupported extra recording topic"
            );
            ensure!(
                extra.record.raw_payload.len() <= 8 * 1024 * 1024,
                "extra recording payload exceeds 8 MiB"
            );
            recorder.record(&extra.topic, &extra.record)?;
        }
    }
    Ok(())
}

pub fn record_game_with_video_and_trace(
    output: &Path,
    seconds: u64,
    stop: &std::sync::atomic::AtomicBool,
    video: bool,
    hooks: Option<RecordingHooks>,
) -> anyhow::Result<RunMetadata> {
    let telemetry =
        SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(TELEMETRY_MAPPING, SCHEMA_VERSION)
            .context("未连接遥测：先部署插件并进入游戏")?;
    let config = SharedCell::<ConfigBlock>::open(CONFIG_MAPPING, SCHEMA_VERSION).ok();
    let events = SharedRing::<GameEvent, EVENT_SLOTS>::open(EVENTS_MAPPING, SCHEMA_VERSION).ok();
    let extension = SharedRing::<ExtensionFrame, 4>::open(EXTENSION_MAPPING, SCHEMA_VERSION).ok();
    let command = SharedCell::<ControlCmd>::open(CONTROL_MAPPING, SCHEMA_VERSION).ok();
    let gate = SharedCell::<Gate>::open(GATE_MAPPING, SCHEMA_VERSION).ok();
    let supervisor_status =
        SharedCell::<SupervisorStatus>::open(SUPERVISOR_STATUS_MAPPING, SCHEMA_VERSION).ok();
    let applied = SharedCell::<AppliedControl>::open(APPLIED_MAPPING, SCHEMA_VERSION).ok();
    let input_diagnostics =
        SharedCell::<InputDiagnostics>::open(INPUT_DIAGNOSTICS_MAPPING, SCHEMA_VERSION).ok();
    let timing = SharedRing::<FrameTiming, 256>::open(FRAME_TIMING_MAPPING, SCHEMA_VERSION).ok();
    let input_actions =
        SharedRing::<InputActionFrame, 256>::open(INPUT_ACTIONS_MAPPING, SCHEMA_VERSION).ok();
    let info = SharedCell::<BridgeInfo>::open(INFO_MAPPING, SCHEMA_VERSION).ok();
    let counters =
        SharedCell::<VehicleCounters>::open(VEHICLE_COUNTERS_MAPPING, SCHEMA_VERSION).ok();
    let environment = environment::describe();
    let mut meta = RunMetadata::new(
        "game",
        environment["game_version"].as_str().unwrap_or("unverified"),
    );
    if let Some(commit) = environment["git_commit"].as_str() {
        meta.git_commit = commit.into();
    }
    meta.environment = environment;
    meta.notes.push(if video{"NVENC video requested; /video_index includes alignment error and dropped capture frames"}else{"Structured-only recording; video alignment acceptance requires separate capture evidence"}.into());
    if video {
        meta.notes.push("Video alignment uses 32-frame telemetry pre-roll, recorded before/after samples, a 40 ms bounded wait, and /video_alignment status; tail capture may remain explicitly unaligned".into());
    }
    let mut recorder = Recorder::create(output, meta)?;
    let session = info.as_ref().and_then(|i| i.latest());
    let mut sidecars = sidecar_recording::SidecarRecording::connect(
        hooks.is_some(),
        session.map_or(0, |i| i.session_start_qpc),
    );
    if let Some(info) = session {
        recorder.record(
            "/bridge_info",
            &Record::wire(&info, 0, 0, info.session_start_qpc),
        )?;
    }
    let start = std::time::Instant::now();
    let initial_latest = telemetry.latest_index();
    let mut telemetry_cursor = TelemetryCursor::new(
        initial_latest.saturating_sub(if video { 32 } else { 1 }),
        initial_latest,
        session.map(|s| s.session_start_qpc),
    );
    let mut alignment = alignment::VideoAligner::new(qpc_frequency())?;
    let mut event_index = events.as_ref().map_or(0, |e| e.latest_index());
    let mut event_loss =
        recording_loss::LossObserver::new("/events", EVENT_SLOTS as u64, event_index, event_index);
    let mut config_seq = 0;
    let mut extension_drain =
        ExtensionDrain::new(extension.as_ref(), session.map(|s| s.session_start_qpc));
    let mut counters_frame = 0;
    let mut timing_index = timing.as_ref().map_or(0, |t| t.latest_index());
    let mut timing_loss =
        recording_loss::LossObserver::new("/frame_timing", 256, timing_index, timing_index);
    let mut action_index = input_actions.as_ref().map_or(0, |r| r.latest_index());
    let mut action_loss =
        recording_loss::LossObserver::new("/input_actions", 256, action_index, action_index);
    let video = if video {
        match video::VideoSession::start(output) {
            Ok(video) => Some(video),
            Err(error) => {
                recorder.finish_incomplete(format!("video initialization failed: {error:#}"))?;
                return Err(error);
            }
        }
    } else {
        None
    };
    let mut ready_sent = false;
    let mut telemetry_written = false;
    let mut video_written = video.is_none();
    let recording_result = (|| -> anyhow::Result<()> {
        while start.elapsed().as_secs() < seconds
            && !stop.load(std::sync::atomic::Ordering::Relaxed)
        {
            let current = telemetry.latest().context("no telemetry frame available")?;
            ensure!(
                etsads_safety::fresh(qpc(), current.header.qpc, qpc_frequency(), 2000),
                "telemetry stopped; recording incomplete"
            );
            if let Some(initial) = session {
                ensure!(
                    info.as_ref()
                        .and_then(|i| i.latest())
                        .is_some_and(|i| i.session_start_qpc == initial.session_start_qpc),
                    "SDK session changed; start a new recording"
                );
            }
            let latest = telemetry.latest_index();
            let earliest = latest.saturating_sub(TELEMETRY_SLOTS as u64 - 1).max(1);
            if telemetry_cursor.loss.observe(
                &mut recorder,
                telemetry_cursor.index,
                latest,
                false,
                true,
                telemetry_cursor.session,
            )? > 0
            {
                ensure!(
                    hooks.is_none(),
                    "mandatory live recording lost telemetry frames"
                );
                telemetry_cursor.index = earliest - 1;
            }
            for index in telemetry_cursor.index + 1..=latest {
                if let Some(t) = telemetry.read(index) {
                    recorder.record("/telemetry", &Record::telemetry(&t))?;
                    alignment.note_telemetry(t);
                    telemetry_written = true;
                    if let Some(c) = command.as_ref().and_then(|r| r.latest()) {
                        recorder.record(
                            "/control_cmd",
                            &Record::wire(&c, t.header.frame_id, t.header.sim_time_us, c.write_qpc),
                        )?;
                    }
                    if let Some(g) = gate.as_ref().and_then(|r| r.latest()) {
                        recorder.record(
                            "/gate",
                            &Record::wire(
                                &g,
                                t.header.frame_id,
                                t.header.sim_time_us,
                                g.heartbeat_qpc,
                            ),
                        )?;
                    }
                    if let Some(a) = applied.as_ref().and_then(|r| r.latest()) {
                        recorder.record(
                            "/applied",
                            &Record::wire(
                                &a,
                                t.header.frame_id,
                                t.header.sim_time_us,
                                a.applied_qpc,
                            ),
                        )?;
                    }
                    if let Some(status) = supervisor_status.as_ref().and_then(|r| r.latest()) {
                        recorder.record(
                            "/supervisor_status",
                            &Record::wire(
                                &status,
                                t.header.frame_id,
                                t.header.sim_time_us,
                                status.heartbeat_qpc,
                            ),
                        )?;
                    }
                    if let Some(d) = input_diagnostics.as_ref().and_then(|r| r.latest()) {
                        recorder.record(
                            "/input_diagnostics",
                            &Record::wire(
                                &d,
                                t.header.frame_id,
                                t.header.sim_time_us,
                                d.last_end_qpc,
                            ),
                        )?;
                    }
                    if let Some(c) = config
                        .as_ref()
                        .and_then(|r| r.latest())
                        .filter(|c| c.config_seq != config_seq)
                    {
                        recorder.record(
                            "/config",
                            &Record::wire(
                                &c,
                                t.header.frame_id,
                                t.header.sim_time_us,
                                t.header.qpc,
                            ),
                        )?;
                        config_seq = c.config_seq;
                    }
                    telemetry_cursor.index = index;
                } else {
                    break;
                }
            }
            extension_drain.record_available(
                &mut recorder,
                extension.as_ref(),
                &mut alignment,
                false,
            )?;
            sidecars.drain(&mut recorder, &alignment, false)?;
            if let Some(c) = counters
                .as_ref()
                .and_then(|r| r.latest())
                .filter(|c| c.valid != 0 && c.header.frame_id != counters_frame)
            {
                recorder.record(
                    "/vehicle_counters",
                    &Record::wire(&c, c.header.frame_id, c.header.sim_time_us, c.header.qpc),
                )?;
                counters_frame = c.header.frame_id;
            }
            if let Some(events) = &events {
                let latest = events.latest_index();
                event_loss.observe(
                    &mut recorder,
                    event_index,
                    latest,
                    false,
                    false,
                    session.map(|s| s.session_start_qpc),
                )?;
                event_index = event_index.max(latest.saturating_sub(EVENT_SLOTS as u64));
                for i in event_index + 1..=latest {
                    if let Some(e) = events.read(i) {
                        recorder.record(
                            "/events",
                            &Record::wire(
                                &e,
                                e.header.frame_id,
                                e.header.sim_time_us,
                                e.header.qpc,
                            ),
                        )?;
                        event_index = i;
                    } else {
                        break;
                    }
                }
            }
            if let Some(timing) = &timing {
                let latest = timing.latest_index();
                let earliest = latest.saturating_sub(255).max(1);
                if timing_loss.observe(
                    &mut recorder,
                    timing_index,
                    latest,
                    false,
                    true,
                    session.map(|s| s.session_start_qpc),
                )? > 0
                {
                    ensure!(
                        hooks.is_none(),
                        "mandatory frame timing evidence overran its ring"
                    );
                    timing_index = earliest - 1;
                }
                for index in timing_index + 1..=latest {
                    let Some(t) = timing.read(index) else { break };
                    recorder.record(
                        "/frame_timing",
                        &Record::wire(&t, t.physics_frame_id, t.sim_time_us, t.qpc),
                    )?;
                    timing_index = index;
                }
            }
            if let Some(actions) = &input_actions {
                let latest = actions.latest_index();
                let earliest = latest.saturating_sub(255).max(1);
                if action_loss.observe(
                    &mut recorder,
                    action_index,
                    latest,
                    false,
                    true,
                    session.map(|s| s.session_start_qpc),
                )? > 0
                {
                    ensure!(
                        hooks.is_none(),
                        "mandatory SDK action evidence overran its ring"
                    );
                    action_index = earliest - 1;
                }
                for index in action_index + 1..=latest {
                    let Some(action) = actions.read(index) else {
                        break;
                    };
                    recorder.record(
                        "/input_actions",
                        &Record::wire(&action, action.source_frame_id, 0, action.completed_qpc),
                    )?;
                    action_index = index;
                }
            }
            if let Some(video) = &video {
                for index in video.indexes.try_iter() {
                    alignment.enqueue(index, qpc())?;
                }
                video_written |= alignment.flush(&mut recorder, qpc(), false)? > 0;
                anyhow::ensure!(
                    !video.is_finished(),
                    "video capture stopped early; run is incomplete"
                );
                let video_progress = video.progress_qpc();
                ensure!(
                    !video_written
                        || etsads_safety::fresh(qpc(), video_progress, qpc_frequency(), 1000),
                    "video encoder made no progress for 1 second"
                );
            }
            drain_extra(&mut recorder, &hooks)?;
            if let Some(hooks) = &hooks {
                hooks
                    .heartbeat_qpc
                    .store(qpc(), std::sync::atomic::Ordering::Release);
                if !ready_sent && telemetry_written && video_written {
                    hooks
                        .ready
                        .try_send(())
                        .context("control loop left before recorder became ready")?;
                    ready_sent = true;
                }
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        Ok(())
    })();
    let allow_tail_telemetry = recording_result.is_ok();
    let cleanup_result = (|| -> anyhow::Result<()> {
        if let Some(video) = video {
            video.request_stop();
            let mut write_error = None;
            let tail_deadline = std::time::Instant::now() + Duration::from_millis(40);
            while !video.is_finished() {
                for index in video.indexes.try_iter() {
                    if let Err(e) = alignment.enqueue(index, qpc()) {
                        write_error.get_or_insert(e);
                    }
                }
                if allow_tail_telemetry
                    && std::time::Instant::now() < tail_deadline
                    && let Err(e) = record_alignment_tail(
                        &mut recorder,
                        &telemetry,
                        &mut telemetry_cursor,
                        &mut alignment,
                        extension.as_ref(),
                        &mut extension_drain,
                        Some(&mut sidecars),
                    )
                {
                    write_error.get_or_insert(e);
                }
                if let Err(e) = alignment.flush(&mut recorder, qpc(), false) {
                    write_error.get_or_insert(e);
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            for index in video.indexes.try_iter() {
                if let Err(e) = alignment.enqueue(index, qpc()) {
                    write_error.get_or_insert(e);
                }
            }
            while allow_tail_telemetry
                && (alignment.has_pending() || alignment.has_pending_camera())
                && std::time::Instant::now() < tail_deadline
            {
                if let Err(e) = record_alignment_tail(
                    &mut recorder,
                    &telemetry,
                    &mut telemetry_cursor,
                    &mut alignment,
                    extension.as_ref(),
                    &mut extension_drain,
                    Some(&mut sidecars),
                ) {
                    write_error.get_or_insert(e);
                    break;
                }
                if let Err(e) = alignment.flush(&mut recorder, qpc(), false) {
                    write_error.get_or_insert(e);
                    break;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            if let Err(e) = alignment.flush(&mut recorder, qpc(), true) {
                write_error.get_or_insert(e);
            }
            let camera_summary = alignment.finish_camera_summary(
                extension.is_some(),
                extension_drain.overrun,
                extension_drain.unmatched,
            );
            if let Err(e) = recorder.record(
                "/video_camera_summary",
                &Record {
                    schema_version: SCHEMA_VERSION,
                    qpc: qpc(),
                    raw_payload: serde_json::to_vec(&camera_summary)?,
                    ..Default::default()
                },
            ) {
                write_error.get_or_insert(e);
            }
            let finished = video.finish();
            if let Some(e) = write_error {
                return Err(e);
            }
            finished?;
        }
        drain_extra(&mut recorder, &hooks)?;
        if let Some(reason) = hooks
            .as_ref()
            .and_then(|h| h.failure_reason.lock().ok().and_then(|r| r.clone()))
        {
            anyhow::bail!(reason)
        }
        Ok(())
    })();
    match recording_result.and(cleanup_result) {
        Ok(()) => recorder.finish(),
        Err(error) => {
            let reason = format!("recording interrupted: {error:#}");
            recorder
                .finish_incomplete(&reason)
                .context("could not finalize incomplete recording")?;
            Err(error)
        }
    }
}
struct TelemetryCursor {
    index: u64,
    loss: recording_loss::LossObserver,
    session: Option<u64>,
}
impl TelemetryCursor {
    fn new(index: u64, latest: u64, session: Option<u64>) -> Self {
        Self {
            index,
            loss: recording_loss::LossObserver::new(
                "/telemetry",
                TELEMETRY_SLOTS as u64,
                index,
                latest,
            ),
            session,
        }
    }
}
fn record_alignment_tail(
    recorder: &mut Recorder,
    telemetry: &SharedRing<TelemetryFrame, TELEMETRY_SLOTS>,
    previous: &mut TelemetryCursor,
    alignment: &mut alignment::VideoAligner,
    extension: Option<&SharedRing<ExtensionFrame, 4>>,
    extension_drain: &mut ExtensionDrain,
    sidecars: Option<&mut sidecar_recording::SidecarRecording>,
) -> anyhow::Result<()> {
    let latest = telemetry.latest_index();
    let earliest = latest.saturating_sub(TELEMETRY_SLOTS as u64 - 1).max(1);
    if previous.loss.observe(
        recorder,
        previous.index,
        latest,
        true,
        false,
        previous.session,
    )? > 0
    {
        previous.index = earliest - 1;
        anyhow::bail!("telemetry retention overrun while aligning video tail");
    }
    for index in previous.index + 1..=latest {
        let Some(frame) = telemetry.read(index) else {
            break;
        };
        recorder.record("/telemetry", &Record::telemetry(&frame))?;
        alignment.note_telemetry(frame);
        previous.index = index;
    }
    extension_drain.record_available(recorder, extension, alignment, true)?;
    if let Some(sidecars) = sidecars {
        sidecars.drain(recorder, alignment, true)?;
    }
    Ok(())
}

struct ExtensionDrain {
    index: u64,
    buffer: Box<ExtensionFrame>,
    overrun: u64,
    unmatched: u64,
    last_recorded_frame: u64,
    loss: recording_loss::LossObserver,
    session: Option<u64>,
}
impl ExtensionDrain {
    fn new(ring: Option<&SharedRing<ExtensionFrame, 4>>, session: Option<u64>) -> Self {
        let latest = ring.map_or(0, |r| r.latest_index());
        let index = latest.saturating_sub(4);
        Self {
            index,
            buffer: Box::default(),
            overrun: 0,
            unmatched: 0,
            last_recorded_frame: 0,
            loss: recording_loss::LossObserver::new("/extension", 4, index, latest),
            session,
        }
    }
    fn record_available(
        &mut self,
        recorder: &mut Recorder,
        ring: Option<&SharedRing<ExtensionFrame, 4>>,
        alignment: &mut alignment::VideoAligner,
        tail: bool,
    ) -> anyhow::Result<()> {
        let Some(ring) = ring else {
            return Ok(());
        };
        let latest = ring.latest_index();
        let earliest = latest.saturating_sub(3).max(1);
        let lost = self
            .loss
            .observe(recorder, self.index, latest, tail, false, self.session)?;
        if lost > 0 {
            self.overrun += lost;
            self.index = earliest - 1;
        }
        for index in self.index + 1..=latest {
            if !ring.read_into(index, &mut self.buffer) {
                break;
            }
            let e = &self.buffer;
            if e.header.frame_id <= self.last_recorded_frame {
                self.index = index;
                continue;
            }
            if alignment
                .latest_recorded_frame()
                .is_none_or(|last| e.header.frame_id > last)
            {
                break;
            }
            if let Some(t) = alignment.recorded_telemetry(e.header.frame_id) {
                ensure!(
                    t.header.qpc == e.header.qpc && t.header.sim_time_us == e.header.sim_time_us,
                    "extension disagrees with already recorded physical frame"
                );
                recorder.record(
                    "/extension",
                    &Record::wire(
                        e.as_ref(),
                        e.header.frame_id,
                        e.header.sim_time_us,
                        e.header.qpc,
                    ),
                )?;
                self.last_recorded_frame = e.header.frame_id;
                if e.available & 1 != 0 {
                    alignment.note_camera(e.header.frame_id);
                }
            } else {
                self.unmatched += 1;
            }
            self.index = index;
        }
        Ok(())
    }
}
pub fn manual_command(
    steer: f32,
    throttle: f32,
    brake: f32,
    seconds: f64,
    pulse: Option<usize>,
) -> anyhow::Result<()> {
    manual_command_with_arm(steer, throttle, brake, seconds, pulse, false)
}

#[derive(Default)]
struct InputWarmup {
    since: Option<std::time::Instant>,
    frames: u32,
    last_frame: u64,
}
impl InputWarmup {
    fn observe(&mut self, fresh: bool, frame: u64, now: std::time::Instant) -> bool {
        if !fresh {
            *self = Self::default();
            return false;
        }
        let since = *self.since.get_or_insert(now);
        if frame != self.last_frame {
            self.frames += 1;
            self.last_frame = frame;
        }
        self.frames >= 10 && now.saturating_duration_since(since) >= Duration::from_millis(300)
    }
}
/// Warm up on neutral commands before the optional, single supervisor arm
/// request. Once active, freshness remains a strict 200 ms fail-closed rule.
pub fn manual_command_with_arm(
    steer: f32,
    throttle: f32,
    brake: f32,
    seconds: f64,
    pulse: Option<usize>,
    arm: bool,
) -> anyhow::Result<()> {
    manual_command_with_stop(
        steer,
        throttle,
        brake,
        seconds,
        pulse,
        arm,
        &std::sync::atomic::AtomicBool::new(false),
    )
}

/// Cancellation exits through the neutral/disarm guard, like failures.
pub fn manual_command_with_stop(
    steer: f32,
    throttle: f32,
    brake: f32,
    seconds: f64,
    pulse: Option<usize>,
    arm: bool,
    stop: &std::sync::atomic::AtomicBool,
) -> anyhow::Result<()> {
    run_manual_command(
        ManualCommand {
            steer,
            throttle,
            brake,
            seconds,
            pulse,
            arm,
        },
        stop,
        "",
    )
}
struct ManualCommand {
    steer: f32,
    throttle: f32,
    brake: f32,
    seconds: f64,
    pulse: Option<usize>,
    arm: bool,
}
fn run_manual_command(
    options: ManualCommand,
    stop: &std::sync::atomic::AtomicBool,
    namespace: &str,
) -> anyhow::Result<()> {
    let ManualCommand {
        steer,
        throttle,
        brake,
        seconds,
        pulse,
        arm,
    } = options;
    let mapping = |name: &str| format!("{name}{namespace}");
    let check_stop = || -> anyhow::Result<()> {
        ensure!(
            !stop.load(std::sync::atomic::Ordering::Acquire),
            "manual input cancelled by user"
        );
        Ok(())
    };
    check_stop()?;
    ensure!(
        steer.is_finite()
            && steer.abs() <= 1.0
            && throttle.is_finite()
            && (0.0..=1.0).contains(&throttle)
            && brake.is_finite()
            && (0.0..=1.0).contains(&brake)
            && seconds.is_finite()
            && (0.01..=60.0).contains(&seconds),
        "invalid command values/duration"
    );
    ensure!(
        throttle == 0.0 || brake == 0.0,
        "throttle and brake are mutually exclusive"
    );
    if let Some(index) = pulse {
        ensure!(
            index < etsads_game::input::PULSE_NAMES.len(),
            "unknown pulse"
        );
    }
    let stationary_action = pulse.is_some_and(etsads_game::input::pulse_requires_stationary);
    let expected_pause = pulse.is_some_and(|i| {
        matches!(
            etsads_game::input::PULSE_NAMES.get(i),
            Some(&"pause" | &"menu")
        )
    });
    let needs_completed_edge = stationary_action || expected_pause;
    ensure!(
        !stationary_action || arm,
        "quickload/engine requires explicit --arm even if the supervisor was already armed"
    );
    let _writer = etsads_shm::SingleWriterGuard::acquire(&mapping("ETSADS_Control_Writer"))?;
    let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
        &mapping(TELEMETRY_MAPPING),
        SCHEMA_VERSION,
    )?;
    let command = SharedCell::<ControlCmd>::create(&mapping(CONTROL_MAPPING), SCHEMA_VERSION)?;
    let applied = SharedCell::<AppliedControl>::open(&mapping(APPLIED_MAPPING), SCHEMA_VERSION)
        .context("input callback is not connected")?;
    let gate = SharedCell::<Gate>::open(&mapping(GATE_MAPPING), SCHEMA_VERSION)
        .context("start etsads-supervisor first")?;
    let requests =
        SharedCell::<SafetyRequest>::open(&mapping(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
            .context("start current etsads-supervisor before a manual experiment")?;
    let completed_actions_ring = if needs_completed_edge {
        Some(
            SharedRing::<InputActionFrame, 256>::open(
                &mapping(INPUT_ACTIONS_MAPPING),
                SCHEMA_VERSION,
            )
            .context(
                "quickload/engine/menu/pause needs the current bridge with completed SDK action evidence",
            )?,
        )
    } else {
        None
    };
    let previous = command.latest().unwrap_or_default();
    let mut cmd = ControlCmd {
        seq: previous.seq,
        pulses: previous.pulses,
        ..Default::default()
    };
    struct Release<'a> {
        command: &'a SharedCell<ControlCmd>,
        requests: SharedCell<SafetyRequest>,
    }
    impl Drop for Release<'_> {
        fn drop(&mut self) {
            let last = self.command.latest().unwrap_or_default();
            let _ = self.command.publish(&ControlCmd {
                seq: last.seq.wrapping_add(1),
                write_qpc: qpc(),
                source_frame_id: last.source_frame_id,
                pulses: last.pulses,
                ..Default::default()
            });
            let seq = self
                .requests
                .latest()
                .map_or(1, |r| r.seq.saturating_add(1));
            let _ = self.requests.publish(&SafetyRequest {
                seq,
                write_qpc: qpc(),
                arm: 0,
                reserved: 0,
            });
        }
    }
    let release = Release {
        command: &command,
        requests,
    };
    let mut loaded_controls = loaded_controls::LoadedControls::connect(namespace)?;
    let mut warmup = InputWarmup::default();
    let warmup_start = std::time::Instant::now();
    eprintln!("等待连续稳定的遥测和输入回调；预热期间转向、油门、刹车均为零。");
    loop {
        check_stop()?;
        ensure!(
            warmup_start.elapsed() < Duration::from_secs(10),
            "input warmup did not obtain 300 ms / 10 fresh physical frames within 10 seconds; enter driving and keep the game active"
        );
        let t = telemetry.latest();
        let a = applied.latest();
        let now = qpc();
        if let Some(t) = t {
            loaded_controls.validate(&t)?;
            validate_stationary_pulse(pulse, t.speed_mps)?;
            cmd.source_frame_id = t.header.frame_id;
        }
        cmd.seq = cmd.seq.wrapping_add(1);
        cmd.write_qpc = now;
        command.publish(&cmd)?;
        let fresh = t.is_some_and(|t| {
            t.header.frame_id > 0
                && t.paused == 0
                && etsads_safety::fresh(now, t.header.qpc, qpc_frequency(), 100)
        }) && a
            .is_some_and(|a| etsads_safety::fresh(now, a.applied_qpc, qpc_frequency(), 100));
        if warmup.observe(
            fresh,
            t.map_or(0, |t| t.header.frame_id),
            std::time::Instant::now(),
        ) {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    check_stop()?;
    let initial = telemetry
        .latest()
        .context("no telemetry at controls binding")?;
    loaded_controls.bind(&initial)?;
    let stationary_steering =
        pulse.is_none() && throttle == 0.0 && steer != 0.0 && initial.speed_mps.abs() <= 0.05;
    if arm {
        let requests = &release.requests;
        let seq = requests.latest().map_or(1, |r| r.seq.saturating_add(1));
        requests.publish(&SafetyRequest {
            seq,
            write_qpc: qpc(),
            arm: 1,
            reserved: 0,
        })?;
    }
    let grant_start = std::time::Instant::now();
    loop {
        check_stop()?;
        let t = telemetry.latest().context("no telemetry after warmup")?;
        let g = loaded_controls::coherent_read(|| gate.latest())
            .context("supervisor heartbeat unavailable")?;
        let a = loaded_controls::coherent_read(|| applied.latest());
        // The observation time follows every producer snapshot. A producer may
        // otherwise publish between reading the clock and reading its payload.
        let now = qpc();
        loaded_controls.validate(&t)?;
        ensure!(
            t.paused == 0 && etsads_safety::fresh(now, t.header.qpc, qpc_frequency(), 200),
            "telemetry became stale/paused while waiting for the gate"
        );
        validate_stationary_pulse(pulse, t.speed_mps)?;
        ensure!(
            a.is_some_and(|a| etsads_safety::fresh(now, a.applied_qpc, qpc_frequency(), 200)),
            "input callback unavailable during grant: now={now}, applied_qpc={:?}, age_ms={:?}, future={}, telemetry_frame={}, gate_seq={}",
            a.map(|a| a.applied_qpc),
            a.map(|a| (now as i128 - a.applied_qpc as i128) as f64 * 1000.0
                / qpc_frequency().max(1) as f64),
            a.is_some_and(|a| a.applied_qpc > now),
            t.header.frame_id,
            g.seq
        );
        ensure!(
            etsads_safety::fresh(now, g.heartbeat_qpc, qpc_frequency(), 200),
            "supervisor heartbeat is stale"
        );
        if g.enabled == 1 {
            break;
        }
        ensure!(
            arm && grant_start.elapsed() < Duration::from_millis(300),
            "safety supervisor is disarmed or refused the single arm request; use --arm after confirming the test area"
        );
        cmd.seq = cmd.seq.wrapping_add(1);
        cmd.source_frame_id = t.header.frame_id;
        cmd.write_qpc = now;
        command.publish(&cmd)?;
        std::thread::sleep(Duration::from_millis(5));
    }
    cmd.steer = steer;
    cmd.throttle = throttle;
    cmd.brake = brake;
    if let Some(index) = pulse {
        cmd.pulses[index] = cmd.pulses[index].wrapping_add(1);
    }
    let first_seq = cmd.seq.wrapping_add(1);
    let mut accepted = false;
    let active_start = std::time::Instant::now();
    let action_start_qpc = qpc();
    let mut action_cursor = completed_actions_ring
        .as_ref()
        .map_or(0, |r| r.latest_index());
    let mut completed_edge_qpc = None;
    let mut last_frame = cmd.source_frame_id;
    while active_start.elapsed().as_secs_f64() < seconds {
        check_stop()?;
        let t = telemetry.latest().context("no telemetry frame")?;
        let g = loaded_controls::coherent_read(|| gate.latest());
        let a = loaded_controls::coherent_read(|| applied.latest());
        let mut completed_actions = Vec::new();
        if let Some(actions) = &completed_actions_ring {
            let latest = actions.latest_index();
            ensure!(
                latest.saturating_sub(action_cursor) <= 256,
                "requested action evidence overran its ring"
            );
            for index in action_cursor + 1..=latest {
                let Some(action) = actions.read(index) else {
                    break;
                };
                completed_actions.push(action);
                action_cursor = index;
            }
        }
        let now = qpc();
        if let Some(index) = pulse {
            for action in &completed_actions {
                if manual_action_ack(
                    action,
                    index,
                    first_seq,
                    action_start_qpc,
                    now,
                    qpc_frequency(),
                ) {
                    completed_edge_qpc = Some(action.completed_qpc);
                }
            }
        }
        let fresh_completed_edge = completed_edge_qpc
            .is_some_and(|at| etsads_safety::fresh(now, at, qpc_frequency(), 200));
        // The SDK action, telemetry and supervisor are separate mappings. An
        // acknowledged menu can pause after `t` was read but before Gate Off
        // is observed. Re-read only to confirm this already-completed request;
        // never publish another command or infer success from Gate Off alone.
        let observed_expected_pause = if fresh_completed_edge && expected_pause {
            let paused = if t.paused != 0 {
                Some(t)
            } else {
                loaded_controls::coherent_read(|| telemetry.latest())
            };
            if let Some(paused) = paused {
                loaded_controls.validate(&paused)?;
                paused.paused != 0
                    && paused.header.frame_id >= t.header.frame_id
                    && etsads_safety::fresh(qpc(), paused.header.qpc, qpc_frequency(), 200)
            } else {
                false
            }
        } else {
            false
        };
        if fresh_completed_edge && (stationary_action || observed_expected_pause) {
            // Loading may immediately pause/reset telemetry. A completed SDK
            // edge acknowledges the request, not a loaded save or a running engine.
            accepted = true;
            break;
        }
        ensure!(t.paused == 0, "game paused; experiment stopped");
        loaded_controls.validate(&t)?;
        if stationary_steering {
            loaded_controls::validate_stationary_steering(&initial, &t)?;
        }
        validate_stationary_pulse(pulse, t.speed_mps)?;
        ensure!(
            etsads_safety::fresh(now, t.header.qpc, qpc_frequency(), 200)
                && t.header.frame_id >= last_frame,
            "telemetry is stale or its frame counter restarted"
        );
        last_frame = t.header.frame_id;
        ensure!(
            g.is_some_and(|g| g.enabled == 1
                && etsads_safety::fresh(now, g.heartbeat_qpc, qpc_frequency(), 200)),
            "independent supervisor revoked permission or became stale: gate={:?}, heartbeat_qpc={:?}, now={now}, age_ms={:?}, telemetry_frame={}, command_seq={}",
            g.map(|v| v.enabled),
            g.map(|v| v.heartbeat_qpc),
            g.map(|v| (now as i128 - v.heartbeat_qpc as i128) as f64 * 1000.0
                / qpc_frequency().max(1) as f64),
            t.header.frame_id,
            cmd.seq
        );
        ensure!(
            a.is_some_and(|a| etsads_safety::fresh(now, a.applied_qpc, qpc_frequency(), 200)),
            "input callback unavailable: now={now}, applied_qpc={:?}, age_ms={:?}, future={}, telemetry_frame={}, gate_seq={:?}",
            a.map(|a| a.applied_qpc),
            a.map(|a| (now as i128 - a.applied_qpc as i128) as f64 * 1000.0
                / qpc_frequency().max(1) as f64),
            a.is_some_and(|a| a.applied_qpc > now),
            t.header.frame_id,
            g.map(|g| g.seq)
        );
        cmd.seq = cmd.seq.wrapping_add(1);
        cmd.write_qpc = now;
        cmd.source_frame_id = t.header.frame_id;
        command.publish(&cmd)?;
        accepted |= !needs_completed_edge
            && a.is_some_and(|a| {
                a.reason == etsads_game::input::REASON_ACTIVE
                    && a.applied_seq >= first_seq
                    && etsads_safety::fresh(now, a.applied_qpc, qpc_frequency(), 200)
            });
        std::thread::sleep(Duration::from_millis(5));
    }
    check_stop()?;
    ensure!(
        accepted,
        "the input DLL never acknowledged the requested command; quickload/engine/menu/pause require their completed SDK edge and menu/pause additionally require the expected pause"
    );
    Ok(())
}
fn manual_action_ack(
    action: &InputActionFrame,
    index: usize,
    first_seq: u64,
    started: u64,
    now: u64,
    frequency: u64,
) -> bool {
    index < 64
        && action.reason == etsads_game::input::REASON_ACTIVE
        && action.command_seq >= first_seq
        && action.prepared_qpc >= started
        && action.completed_qpc >= action.prepared_qpc
        && etsads_safety::fresh(now, action.completed_qpc, frequency, 200)
        && action.pulse_mask & (1u64 << index) != 0
}
fn validate_stationary_pulse(pulse: Option<usize>, speed: f32) -> anyhow::Result<()> {
    ensure!(
        !pulse.is_some_and(etsads_game::input::pulse_requires_stationary)
            || (speed.is_finite() && speed.abs() <= 0.05),
        "quickload/engine is permitted only at measured standstill (<=0.05 m/s); stop the vehicle before requesting this action"
    );
    Ok(())
}
pub fn analyze(run: &Path) -> anyhow::Result<serde_json::Value> {
    let mut steps = Vec::new();
    let mut paused_steps = Vec::new();
    let mut last: Option<TelemetryFrame> = None;
    let mut first_qpc = None;
    let mut last_qpc = 0;
    let mut frequency = 0;
    let mut max_speed = 0_f32;
    let mut inputs = [[f32::INFINITY, f32::NEG_INFINITY]; 4];
    let mut effective = inputs;
    let mut events = std::collections::BTreeMap::<String, u64>::new();
    let mut applied_reasons = std::collections::BTreeMap::<u32, u64>::new();
    let replay = etsads_recorder::replay(run, |topic, r| {
        match topic {
            "/telemetry" => {
                let t = r.decode_wire::<TelemetryFrame>()?;
                if let Some(p) = last
                    && t.header.sim_time_us >= p.header.sim_time_us
                {
                    steps.push(t.header.sim_time_us - p.header.sim_time_us);
                    paused_steps.push(
                        t.paused_simulation_time_us
                            .saturating_sub(p.paused_simulation_time_us),
                    );
                }
                first_qpc.get_or_insert(t.header.qpc);
                last_qpc = t.header.qpc;
                frequency = t.qpc_frequency;
                max_speed = max_speed.max(t.speed_mps);
                for i in 0..4 {
                    inputs[i][0] = inputs[i][0].min(t.input[i]);
                    inputs[i][1] = inputs[i][1].max(t.input[i]);
                    effective[i][0] = effective[i][0].min(t.effective[i]);
                    effective[i][1] = effective[i][1].max(t.effective[i]);
                }
                last = Some(t);
            }
            "/events" => {
                let e = r.decode_wire::<GameEvent>()?;
                let text = String::from_utf8_lossy(&e.name)
                    .trim_end_matches('\0')
                    .to_owned();
                *events.entry(text).or_default() += 1;
            }
            "/applied" => {
                let a = r.decode_wire::<AppliedControl>()?;
                *applied_reasons.entry(a.reason).or_default() += 1;
            }
            _ => {}
        }
        Ok(())
    })?;
    steps.sort_unstable();
    paused_steps.sort_unstable();
    let percentile = |v: &[u64], p: f64| -> Option<u64> {
        v.get(((v.len().saturating_sub(1)) as f64 * p).round() as usize)
            .copied()
    };
    Ok(
        serde_json::json!({"integrity":replay,"wall_duration_s":if frequency>0{Some(last_qpc.saturating_sub(first_qpc.unwrap_or(0))as f64/frequency as f64)}else{None},"simulation_step_us":{"min":steps.first(),"p50":percentile(&steps,0.5),"p95":percentile(&steps,0.95),"max":steps.last()},"paused_simulation_step_us":{"min":paused_steps.first(),"p50":percentile(&paused_steps,0.5),"max":paused_steps.last()},"max_speed_kmh":max_speed*3.6,"input_ranges":inputs,"effective_ranges":effective,"events":events,"applied_reason_counts":applied_reasons}),
    )
}

#[cfg(test)]
mod trace_recording_tests {
    use super::*;
    #[test]
    fn stationary_pulses_require_explicit_arm_and_reject_moving_or_unknown_speed() {
        for name in ["quickload", "engine"] {
            let index = etsads_game::input::PULSE_NAMES
                .iter()
                .position(|n| *n == name)
                .unwrap();
            let result = manual_command_with_arm(0.0, 0.0, 0.0, 0.3, Some(index), false);
            assert!(result.unwrap_err().to_string().contains("explicit --arm")); // Fails before opening any mapping.
            validate_stationary_pulse(Some(index), 0.0).unwrap();
            for speed in [0.051, -0.051, f32::NAN, f32::INFINITY] {
                assert!(validate_stationary_pulse(Some(index), speed).is_err());
            }
        }
        validate_stationary_pulse(None, 10.0).unwrap();
    }
    #[cfg(windows)]
    #[test]
    fn tail_records_delayed_same_frame_camera_once_and_counts_a_real_miss() {
        let namespace = format!("ETSADS_CameraTailTest_{}_{}", std::process::id(), qpc());
        let telemetry =
            SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(&namespace, SCHEMA_VERSION)
                .unwrap();
        let extension = SharedRing::<ExtensionFrame, 4>::create(
            &format!("{namespace}_Extension"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(&namespace);
        let mut recorder =
            Recorder::create(&path, RunMetadata::new("synthetic", "camera-tail-fixture")).unwrap();
        let mut alignment = alignment::VideoAligner::new(1000).unwrap();
        let mut drain = ExtensionDrain::new(Some(&extension), None);
        let mut previous = TelemetryCursor::new(0, telemetry.latest_index(), None);
        let mut t = TelemetryFrame {
            header: WireHeader {
                frame_id: 10,
                sim_time_us: 10000,
                qpc: 100,
                ..Default::default()
            },
            qpc_frequency: 1000,
            ..Default::default()
        };
        telemetry.publish(&t).unwrap();
        record_alignment_tail(
            &mut recorder,
            &telemetry,
            &mut previous,
            &mut alignment,
            Some(&extension),
            &mut drain,
            None,
        )
        .unwrap();
        alignment
            .enqueue(
                VideoIndex {
                    video_frame: 0,
                    capture_frame: 1,
                    capture_qpc: 100,
                    ..Default::default()
                },
                100,
            )
            .unwrap();
        alignment.flush(&mut recorder, 100, false).unwrap();
        assert!(alignment.has_pending_camera());
        // The next physical frame's start publishes the preceding camera.
        let mut e = Box::<ExtensionFrame>::default();
        e.header = t.header;
        e.available = 1;
        extension.publish(&e).unwrap();
        t.header.frame_id = 11;
        t.header.sim_time_us = 20000;
        t.header.qpc = 117;
        telemetry.publish(&t).unwrap();
        record_alignment_tail(
            &mut recorder,
            &telemetry,
            &mut previous,
            &mut alignment,
            Some(&extension),
            &mut drain,
            None,
        )
        .unwrap();
        assert!(!alignment.has_pending_camera());
        extension.publish(&e).unwrap(); // A duplicate physical frame is not re-recorded.
        record_alignment_tail(
            &mut recorder,
            &telemetry,
            &mut previous,
            &mut alignment,
            Some(&extension),
            &mut drain,
            None,
        )
        .unwrap();
        alignment
            .enqueue(
                VideoIndex {
                    video_frame: 1,
                    capture_frame: 2,
                    capture_qpc: 117,
                    ..Default::default()
                },
                117,
            )
            .unwrap();
        alignment.flush(&mut recorder, 117, false).unwrap();
        e.header.frame_id = 12;
        e.header.sim_time_us = 30000;
        e.header.qpc = 134;
        extension.publish(&e).unwrap();
        record_alignment_tail(
            &mut recorder,
            &telemetry,
            &mut previous,
            &mut alignment,
            Some(&extension),
            &mut drain,
            None,
        )
        .unwrap();
        let summary = alignment.finish_camera_summary(true, drain.overrun, drain.unmatched);
        assert_eq!(summary["video_frames_with_recorded_camera"], 1);
        assert_eq!(summary["video_frames_missing_recorded_camera"], 1);
        recorder.finish().unwrap();
        let mut recorded = Vec::new();
        etsads_recorder::replay(&path, |topic, record| {
            if topic == "/extension" {
                recorded.push(record.frame_id);
            }
            Ok(())
        })
        .unwrap();
        assert_eq!(recorded, vec![10]);
    }
    #[cfg(windows)]
    #[test]
    fn manual_cancellation_releases_at_warmup_grant_and_active_phases() {
        use std::sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        };
        for phase in 0..4 {
            // Never open the running game's namespace. The producer below is
            // an isolated synthetic SDK/supervisor fixture, not game evidence.
            let namespace = format!(
                "_ManualCancelTest_{}_{}_{}",
                std::process::id(),
                qpc(),
                phase
            );
            let name = |base: &str| format!("{base}{namespace}");
            let control_configuration = SharedRing::<ControlConfiguration, 64>::create(
                &name(CONTROL_CONFIGURATION_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let info =
                SharedCell::<BridgeInfo>::create(&name(INFO_MAPPING), SCHEMA_VERSION).unwrap();
            let session = qpc();
            info.publish(&BridgeInfo {
                session_start_qpc: session,
                telemetry_api_version: 0x10001,
                input_api_version: 0x10000,
                ..Default::default()
            })
            .unwrap();
            control_configuration
                .publish(&ControlConfiguration {
                    header: WireHeader {
                        qpc: session,
                        seq: 1,
                        ..Default::default()
                    },
                    session_start_qpc: session,
                    generation: 1,
                    effective_from_frame_id: 1,
                    shifter_type: SHIFTER_AUTOMATIC,
                    valid: 1,
                })
                .unwrap();
            let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
                &name(TELEMETRY_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let command =
                SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
            let gate = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
            let applied =
                SharedCell::<AppliedControl>::create(&name(APPLIED_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let requests =
                SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let command_observer =
                SharedCell::<ControlCmd>::open(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
            let request_observer =
                SharedCell::<SafetyRequest>::open(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let stop = Arc::new(AtomicBool::new(false));
            let fixture_stop = stop.clone();
            let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
            let fixture = std::thread::spawn(move || {
                let deadline = std::time::Instant::now() + Duration::from_secs(12);
                let mut frame = 0;
                loop {
                    frame += 1;
                    let cmd = command.latest().unwrap_or_default();
                    let request = requests.latest().unwrap_or_default();
                    let enabled = phase == 3 || (phase == 2 && request.arm == 1);
                    let now = qpc();
                    telemetry
                        .publish(&TelemetryFrame {
                            header: WireHeader {
                                frame_id: frame,
                                qpc: now,
                                ..Default::default()
                            },
                            qpc_frequency: qpc_frequency(),
                            ..Default::default()
                        })
                        .unwrap();
                    gate.publish(&Gate {
                        seq: frame,
                        heartbeat_qpc: now,
                        enabled: enabled as u32,
                        ..Default::default()
                    })
                    .unwrap();
                    applied
                        .publish(&AppliedControl {
                            applied_qpc: now,
                            frame_index: frame,
                            applied_seq: cmd.seq,
                            reason: if enabled { 0 } else { 1 },
                            ..Default::default()
                        })
                        .unwrap();
                    if frame == 1 {
                        ready_tx.send(()).unwrap();
                    }
                    let reached = match phase {
                        0 => cmd.seq > 0,
                        1 => request.arm == 1,
                        2 | 3 => cmd.steer == 0.3,
                        _ => unreachable!(),
                    };
                    if reached || std::time::Instant::now() > deadline {
                        fixture_stop.store(true, Ordering::Release);
                        return reached;
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
            });
            ready_rx.recv().unwrap();
            let result = run_manual_command(
                ManualCommand {
                    steer: 0.3,
                    throttle: 0.0,
                    brake: 0.0,
                    seconds: 10.0,
                    pulse: None,
                    arm: phase != 3,
                },
                &stop,
                &namespace,
            );
            assert!(
                fixture.join().unwrap(),
                "fixture did not reach phase {phase}: {result:?}"
            );
            assert!(
                result
                    .unwrap_err()
                    .to_string()
                    .contains("cancelled by user")
            );
            let released = command_observer.latest().unwrap();
            assert_eq!(
                [
                    released.steer,
                    released.throttle,
                    released.brake,
                    released.clutch
                ],
                [0.0; 4]
            );
            assert_eq!(request_observer.latest().unwrap().arm, 0);
            assert!(etsads_shm::SingleWriterGuard::acquire(&name("ETSADS_Control_Writer")).is_ok());
        }
    }
    #[test]
    fn input_warmup_requires_distinct_continuously_fresh_frames() {
        let mut warmup = InputWarmup::default();
        let start = std::time::Instant::now();
        for i in 0..20 {
            assert!(!warmup.observe(true, 1, start + Duration::from_millis(i * 30)));
        }
        assert!(!warmup.observe(false, 2, start + Duration::from_secs(1)));
        for i in 0..10 {
            assert!(!warmup.observe(true, i + 2, start + Duration::from_millis(1100 + i * 30)));
        }
        assert!(warmup.observe(true, 12, start + Duration::from_millis(1400)));
    }
    #[test]
    fn extra_trace_is_written_and_failure_stays_incomplete() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!("trace-{}-{}", std::process::id(), qpc()));
        let mut recorder =
            Recorder::create(&path, RunMetadata::new("synthetic", "fixture")).unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(2);
        let (ready, _) = std::sync::mpsc::sync_channel(1);
        let hooks = Some(RecordingHooks {
            extra: rx,
            ready,
            heartbeat_qpc: std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)),
            failure_reason: std::sync::Arc::new(std::sync::Mutex::new(None)),
        });
        tx.send(ExtraRecord {
            topic: "/stack".into(),
            record: Record {
                schema_version: SCHEMA_VERSION,
                frame_id: 42,
                raw_payload: br#"{"source":"synthetic","fixture":true}"#.to_vec(),
                ..Default::default()
            },
        })
        .unwrap();
        drain_extra(&mut recorder, &hooks).unwrap();
        let metadata = recorder
            .finish_incomplete("injected fixture failure")
            .unwrap();
        assert!(!metadata.completed);
        assert_eq!(metadata.source, "synthetic");
        let replay = etsads_recorder::replay(&path, |topic, record| {
            assert_eq!(topic, "/stack");
            assert_eq!(record.frame_id, 42);
            assert_eq!(
                serde_json::from_slice::<serde_json::Value>(&record.raw_payload)?["source"],
                "synthetic"
            );
            Ok(())
        })
        .unwrap();
        assert!(!replay.completed);
        assert_eq!(replay.messages, 1);
    }
}
