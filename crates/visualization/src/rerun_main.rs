//! Optional read-only Rerun CLI. No control mappings or game-input APIs are used.
use anyhow::{Context, ensure};
use clap::{Parser, Subcommand};
use etsads_game::{
    adapter::{GameAdapter, GameSnapshot},
    capture::CaptureReader,
};
use etsads_msgs::protocol::{
    APPLIED_MAPPING, AppliedControl, CONTROL_MAPPING, ControlCmd, SCHEMA_VERSION, TelemetryFrame,
    VideoIndex,
};
use etsads_visualization::live_diagnostics::{CameraHistory, CommandDelays, ImageMatchStatus};
use etsads_visualization::rerun_export::{CameraSample, MapContext, Sink, viewer_video_timestamps};
use rerun as rr;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::Read,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

#[derive(Parser)]
#[command(about = "Read-only ETSADS Rerun export/viewer stream; never sends game input")]
struct Args {
    #[command(subcommand)]
    command: Mode,
}
#[derive(Subcommand)]
enum Mode {
    Export {
        #[arg(long)]
        run: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long)]
        map: Option<PathBuf>,
        #[arg(long, default_value = "ffprobe")]
        ffprobe: String,
        /// Enable only the explicitly requested C1 interior diagnostic camera model.
        #[arg(long)]
        interior_c1: bool,
    },
    Live {
        #[arg(long, conflicts_with = "connect")]
        output: Option<PathBuf>,
        /// Connect to an already-running LOCAL Rerun viewer (default port 9876).
        #[arg(long)]
        connect: bool,
        #[arg(long, default_value_t = 30)]
        seconds: u64,
        #[arg(long)]
        map: Option<PathBuf>,
        /// Display raw images without a camera model unless explicitly enabled.
        #[arg(long)]
        interior_c1: bool,
        /// Disable image capture when only structured diagnostics are needed.
        #[arg(long)]
        no_video: bool,
    },
}
#[derive(Default, Serialize)]
struct ExportReport {
    schema_version: u32,
    rerun_version: String,
    source: String,
    source_completed: bool,
    game_version: String,
    output: String,
    source_sha256: BTreeMap<String, String>,
    telemetry_frames: u64,
    scene_frames: u64,
    agent_observations: u64,
    max_agents_in_frame: usize,
    route_frames_with_uids: u64,
    telemetry_without_extension: u64,
    extension_without_telemetry: u64,
    video_frames: u64,
    video_with_index_camera: u64,
    video_without_index_camera: u64,
    camera_capture_p95_ms: Option<f64>,
    camera_capture_max_ms: Option<f64>,
    camera_capture_over_16ms: u64,
    native_route_nodes_available: usize,
    camera_model: String,
    video_codec: Option<String>,
    ffprobe_start_pts: Option<i64>,
    ffprobe_time_base: Option<String>,
    viewer_pts_cross_checked: bool,
    missing: Vec<String>,
    notes: Vec<String>,
    command_delay_matched: u64,
    command_delay_unavailable: u64,
}
#[derive(Serialize)]
struct VideoAssociation {
    video_frame: u64,
    capture_frame: u64,
    capture_qpc: u64,
    indexed_physics_frame: u64,
    indexed_physics_qpc: u64,
    presentation_timestamp_ns: i64,
    camera_physics_frame: Option<u64>,
    camera_qpc: Option<u64>,
    camera_capture_delta_ms: Option<f64>,
    presentation_paused_sim_time_us: Option<u64>,
    scene_and_image_share_exposure_time: bool,
}
fn main() -> anyhow::Result<()> {
    // Fixed stack budget for large wire records in third-party APIs, independent
    // of Windows executable defaults. Wire extension decoding itself is boxed.
    std::thread::Builder::new()
        .name("rerun-export".into())
        .stack_size(8 * 1024 * 1024)
        .spawn(run)?
        .join()
        .map_err(|_| anyhow::anyhow!("Rerun worker panicked"))?
}
fn run() -> anyhow::Result<()> {
    match Args::parse().command {
        Mode::Export {
            run,
            output,
            map,
            ffprobe,
            interior_c1,
        } => export(&run, &output, map.as_deref(), &ffprobe, interior_c1),
        Mode::Live {
            output,
            connect,
            seconds,
            map,
            interior_c1,
            no_video,
        } => live(
            output.as_deref(),
            connect,
            seconds,
            map.as_deref(),
            interior_c1,
            !no_video,
        ),
    }
}
fn reserve_output(path: &Path) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .context("output exists; choose a new .rrd path")?;
    Ok(())
}
fn digest(path: &Path) -> anyhow::Result<String> {
    let mut file = fs::File::open(path)?;
    let mut hash = Sha256::new();
    let mut buffer = vec![0; 1024 * 1024];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hash.update(&buffer[..read]);
    }
    Ok(format!("{:x}", hash.finalize()))
}
fn map_from(path: Option<&Path>) -> anyhow::Result<MapContext> {
    path.map(MapContext::from_path)
        .transpose()
        .map(Option::unwrap_or_default)
}
const MAX_VIDEO_INDEXES: usize = 500_000;
const MAX_EMBEDDED_VIDEO_BYTES: u64 = 2 * 1024 * 1024 * 1024;
fn bounded_file(path: &Path, limit: u64) -> anyhow::Result<Vec<u8>> {
    let mut bytes = Vec::new();
    fs::File::open(path)?
        .take(limit + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() as u64 <= limit,
        "{} exceeds resource budget",
        path.display()
    );
    Ok(bytes)
}
fn export(
    run: &Path,
    output: &Path,
    map_path: Option<&Path>,
    ffprobe: &str,
    interior_c1: bool,
) -> anyhow::Result<()> {
    ensure!(
        fs::metadata(run.join("meta.json"))?.len() <= 8 * 1024 * 1024,
        "run metadata exceeds 8 MiB"
    );
    let metadata: serde_json::Value =
        serde_json::from_slice(&bounded_file(&run.join("meta.json"), 8 * 1024 * 1024)?)?;
    ensure!(
        metadata["source"] == "game",
        "this exporter accepts real-game recordings; simulation must remain separately labelled"
    );
    let map = map_from(map_path)?;
    let mut first = None;
    let mut indices = Vec::new();
    let mut frequency = 0;
    let replay = etsads_recorder::replay(run, |topic, record| {
        match topic {
            "/telemetry" => {
                let value = record.decode_wire::<TelemetryFrame>()?;
                ensure!(
                    value.qpc_frequency > 0 && (frequency == 0 || frequency == value.qpc_frequency),
                    "missing/changing QPC frequency"
                );
                frequency = value.qpc_frequency;
                first.get_or_insert(value);
            }
            "/video_index" => {
                ensure!(
                    indices.len() < MAX_VIDEO_INDEXES,
                    "video index budget exceeded (500000 frames); export a shorter recording"
                );
                indices.push(record.decode_wire::<VideoIndex>()?);
            }
            _ => {}
        }
        Ok(())
    })?;
    let first = first.context("no telemetry")?;
    let qpc_frequency = first.qpc_frequency;
    let first_qpc = first.header.qpc;
    indices.sort_by_key(|v| v.video_frame);
    let video_path = run.join("video-0000.mp4");
    let video = if video_path.is_file() {
        ensure!(
            fs::metadata(&video_path)?.len() <= MAX_EMBEDDED_VIDEO_BYTES,
            "embedded video exceeds 2 GiB; select a shorter recording or use live streaming; source data was not changed"
        );
        ensure!(
            !indices.is_empty(),
            "video exists but video_index is missing"
        );
        ensure!(
            indices
                .iter()
                .enumerate()
                .all(|(i, v)| v.video_frame == i as u64),
            "duplicate/gapped encoded video indices"
        );
        let info =
            etsads_visualization::media::probe_video(&video_path, ffprobe, MAX_VIDEO_INDEXES)?;
        let decoded_pts = info.timestamps_ns()?;
        let timestamps = viewer_video_timestamps(&video_path)?;
        ensure!(
            decoded_pts.len() == timestamps.len()
                && decoded_pts
                    .iter()
                    .zip(&timestamps)
                    .all(|(a, b)| a.abs_diff(*b) <= 1),
            "decoded ffprobe frame PTS disagree with the viewer's MP4 sample table"
        );
        ensure!(
            timestamps.len() == indices.len(),
            "video frame count {} disagrees with {} index records",
            timestamps.len(),
            indices.len()
        );
        Some((info, timestamps))
    } else {
        None
    };
    let mut report=ExportReport {schema_version:1,rerun_version:"0.38.1".into(),source:"game".into(),source_completed:replay.completed,
        game_version:metadata["game_version"].as_str().unwrap_or("unknown").into(),output:output.display().to_string(),
        native_route_nodes_available:map.nodes.len(),camera_model:if interior_c1 {"C1 explicitly selected; interior direction evidence only; absolute reprojection unverified"}else{"disabled; raw CameraState logged"}.into(),
        notes:vec!["Read-only source MCAP. ENU metres via GameAdapter; no driving acceptance asserted.".into(),
            "Elapsed timeline uses original QPC; simulation timeline can freeze while paused.".into(),
            "Video uses the recorded encoded frame index and actual rational MP4 PTS. Camera uses the exact indexed physics frame ID, never silent nearest-frame substitution.".into(),
            "A matching physical ID does not prove identical render exposure. Per-frame QPC residuals remain explicit; >16 ms is not accepted synchronization.".into(),
            "Traffic and parked native boxes use raw centers, dimensions, height and full quaternion; legacy Scene footprints remain separate. Ego has a reference point and symbolic heading only because its full exterior height is unmeasured.".into(),
            "Geometry, camera and image presentation share capture exposure QPC; original physical QPC is retained in each association. Telemetry curves retain physical sample time and paused simulation time.".into(),
            "Streaming snapshot lookahead is 64 frames; encoded index metadata is capped at 500000 frames, map JSON at 128 MiB and embedded MP4 at 2 GiB. SDK file queues are flushed every 256 physical frames.".into()],..Default::default()};
    for entry in fs::read_dir(run)? {
        let path = entry?.path();
        if path.extension().is_some_and(|x| x == "mcap" || x == "mp4")
            || path.file_name().is_some_and(|x| x == "meta.json")
        {
            report.source_sha256.insert(
                path.file_name().unwrap().to_string_lossy().into_owned(),
                digest(&path)?,
            );
        }
    }
    if let Some(path) = map_path {
        report.source_sha256.insert("map".into(), digest(path)?);
    }
    if map.nodes.is_empty() {
        report.missing.push(
            "Native node coordinates: route UIDs remain visible, unresolved geometry is omitted."
                .into(),
        );
    }
    if video.is_none() {
        report
            .missing
            .push("Video file absent: no camera image fabricated.".into());
    }
    report.missing.push("Planner lane trajectory is not present in this recording; native navigation route is a separate entity.".into());
    reserve_output(output)?;
    let rec = rr::RecordingStreamBuilder::new("etsads/read-only-audit").save(output)?;
    rec.log_static(
        "status/source",
        &rr::TextDocument::new(serde_json::to_string_pretty(&metadata)?),
    )?;
    let sink = Sink::new(rec, first_qpc, qpc_frequency, &map)?;
    let mut associations = Vec::new();
    let mut deltas = Vec::new();
    let mut rendered = vec![false; indices.len()];
    if let Some((info, _)) = &video {
        report.video_codec = Some(info.streams[0].codec_name.clone());
        report.ffprobe_start_pts = Some(info.frames[0].best_effort_timestamp);
        report.ffprobe_time_base = Some(info.streams[0].time_base.clone());
        report.viewer_pts_cross_checked = true;
        sink.recording.log_static(
            "video_asset",
            &rr::AssetVideo::from_file_contents(
                bounded_file(&video_path, MAX_EMBEDDED_VIDEO_BYTES)?,
                Some("video/mp4"),
            ),
        )?;
        sink.recording.flush_with_timeout(Duration::from_secs(30))?;
    }
    let joined = etsads_replay::ReplayAdapter::run(run, |frame| {
        let snapshot = frame.snapshot;
        if let Some((info, timestamps)) = &video {
            sink.telemetry_series(&snapshot)?;
            let resolution = [info.streams[0].width, info.streams[0].height];
            for video_frame in frame.video {
                let index = video_frame.index;
                let offset = usize::try_from(index.video_frame)?;
                let was_rendered = rendered
                    .get_mut(offset)
                    .context("video index exceeds encoded frame count")?;
                ensure!(!*was_rendered, "duplicate rendered video index");
                *was_rendered = true;
                let association = present_video(
                    &sink,
                    index,
                    Some(&snapshot),
                    timestamps[offset],
                    resolution,
                    interior_c1,
                )?;
                note_association(&mut report, &mut associations, &mut deltas, association);
            }
        } else {
            sink.telemetry(&snapshot)?;
            sink.scene(&snapshot)?;
        }
        report.telemetry_frames += 1;
        report.telemetry_without_extension += u64::from(snapshot.extension.is_none());
        report.scene_frames += u64::from(snapshot.extension.is_some());
        report.agent_observations += snapshot.scene.agents.len() as u64;
        report.max_agents_in_frame = report.max_agents_in_frame.max(snapshot.scene.agents.len());
        report.route_frames_with_uids += u64::from(!snapshot.scene.route.is_empty());
        if report.telemetry_frames.is_multiple_of(256) {
            sink.recording.flush_with_timeout(Duration::from_secs(30))?;
        }
        Ok(())
    })?;
    report.extension_without_telemetry = joined.associations.orphan_extensions;
    if let Some((info, timestamps)) = &video {
        let resolution = [info.streams[0].width, info.streams[0].height];
        for (offset, index) in indices
            .iter()
            .enumerate()
            .filter(|(offset, _)| !rendered[*offset])
        {
            let association = present_video(
                &sink,
                *index,
                None,
                timestamps[offset],
                resolution,
                interior_c1,
            )?;
            note_association(&mut report, &mut associations, &mut deltas, association);
        }
    }
    associations.sort_by_key(|association| association.video_frame);
    // Command diagnostics stream independently; a held command contributes one
    // first-application observation, not an ever-growing age distribution.
    let mut delays = CommandDelays::default();
    let mut physical_clock = None;
    etsads_recorder::replay(run, |topic, record| {
        match topic {
            "/telemetry" => {
                let raw: TelemetryFrame = record.decode_wire()?;
                physical_clock = Some((raw.header.frame_id, raw.paused_simulation_time_us));
            }
            "/control_cmd" => delays.command(record.decode_wire::<ControlCmd>()?),
            "/applied" => {
                if let Some(delay) =
                    delays.applied(record.decode_wire::<AppliedControl>()?, qpc_frequency)
                {
                    let paused_time = physical_clock
                        .filter(|(frame, _)| *frame == record.frame_id)
                        .map(|(_, time)| time);
                    sink.command_delay(&delay, record.frame_id, paused_time)?;
                    if delay.delay_ms.is_some() {
                        report.command_delay_matched += 1;
                    } else {
                        report.command_delay_unavailable += 1;
                    }
                    if (report.command_delay_matched + report.command_delay_unavailable)
                        .is_multiple_of(256)
                    {
                        sink.recording.flush_with_timeout(Duration::from_secs(30))?;
                    }
                }
            }
            _ => {}
        }
        Ok(())
    })?;
    deltas.sort_by(f64::total_cmp);
    report.camera_capture_p95_ms = deltas
        .get((deltas.len() as f64 * 0.95).floor() as usize)
        .copied();
    report.camera_capture_max_ms = deltas.last().copied();
    if report.route_frames_with_uids == 0 {
        report.missing.push("No native navigation route UIDs were present in any recorded scene; no route geometry is invented.".into());
    }
    if report.video_without_index_camera > 0 {
        report.missing.push(format!(
            "{} encoded video frame(s) have no CameraState for their indexed physics frame.",
            report.video_without_index_camera
        ));
    }
    sink.recording.log_static(
        "status/export",
        &rr::TextDocument::new(serde_json::to_string_pretty(&report)?),
    )?;
    sink.recording.flush_with_timeout(Duration::from_secs(30))?;
    fs::write(
        output.with_extension("report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    fs::write(
        output.with_extension("video-associations.json"),
        serde_json::to_vec(&associations)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn note_association(
    report: &mut ExportReport,
    associations: &mut Vec<VideoAssociation>,
    deltas: &mut Vec<f64>,
    value: VideoAssociation,
) {
    report.video_frames += 1;
    if let Some(delta) = value.camera_capture_delta_ms {
        report.video_with_index_camera += 1;
        deltas.push(delta.abs());
        report.camera_capture_over_16ms += u64::from(delta.abs() > 16.0);
    } else {
        report.video_without_index_camera += 1;
    }
    associations.push(value);
}

fn present_video(
    sink: &Sink,
    index: VideoIndex,
    snapshot: Option<&GameSnapshot>,
    pts: i64,
    resolution: [u32; 2],
    interior_c1: bool,
) -> anyhow::Result<VideoAssociation> {
    let paused_time = snapshot.map(|value| value.raw.paused_simulation_time_us);
    sink.time(
        index.capture_qpc,
        index.physics_frame,
        paused_time.unwrap_or(0),
    );
    if paused_time.is_none() {
        sink.recording.disable_timeline("simulation");
    }
    if let Some(snapshot) = snapshot {
        sink.telemetry_at(snapshot, index.capture_qpc)?;
        sink.scene_at(snapshot, index.capture_qpc)?;
    } else {
        for path in ["world/ego", "world/agents", "world/signals", "world/route"] {
            sink.recording.log(path, &rr::Clear::recursive())?;
        }
    }
    let camera = snapshot
        .and_then(|snapshot| snapshot.extension.as_ref())
        .filter(|ext| ext.available & 1 != 0)
        .map(|ext| CameraSample::new(ext.header, bytemuck::pod_read_unaligned(&ext.camera)));
    let delta = camera.as_ref().map(|camera| {
        (camera.qpc as i128 - index.capture_qpc as i128) as f64 * 1000.0 / sink.qpc_frequency as f64
    });
    let use_model = interior_c1 && delta.is_some_and(|delta| delta.abs() <= 16.0);
    if let Some(camera) = &camera {
        camera.log(&sink.recording, Some(resolution), use_model)?;
    } else {
        sink.recording.log(
            "status/camera",
            &rr::TextLog::new("missing CameraState for indexed physics frame"),
        )?;
    }
    let reference = rr::VideoFrameReference::new(rr::components::VideoTimestamp::from_nanos(pts))
        .with_video_reference("video_asset");
    // Keep a stable independent video view even while a diagnostic pinhole is
    // enabled; auto-layout can otherwise select an intentionally cleared tab.
    sink.recording.log("video/image", &reference)?;
    if use_model {
        sink.recording.log("world/camera/image", &reference)?;
    } else {
        sink.recording
            .log("world/camera", &rr::Clear::recursive())?;
    }
    let value = VideoAssociation {
        video_frame: index.video_frame,
        capture_frame: index.capture_frame,
        capture_qpc: index.capture_qpc,
        indexed_physics_frame: index.physics_frame,
        indexed_physics_qpc: index.physics_qpc,
        presentation_timestamp_ns: pts,
        camera_physics_frame: camera.as_ref().map(|camera| camera.physics_frame),
        camera_qpc: camera.as_ref().map(|camera| camera.qpc),
        camera_capture_delta_ms: delta,
        presentation_paused_sim_time_us: paused_time,
        scene_and_image_share_exposure_time: snapshot.is_some(),
    };
    sink.recording.log(
        "video/alignment",
        &rr::TextLog::new(serde_json::to_string(&value)?),
    )?;
    sink.recording.log(
        "video/encoded_frame",
        &rr::Scalars::new([index.video_frame as f64]),
    )?;
    if let Some(delta) = delta {
        sink.recording
            .log("video/camera_capture_delta_ms", &rr::Scalars::new([delta]))?;
    } else {
        sink.recording
            .log("video/camera_capture_delta_ms", &rr::Clear::recursive())?;
    }
    Ok(value)
}

fn live(
    output: Option<&Path>,
    connect: bool,
    seconds: u64,
    map_path: Option<&Path>,
    interior_c1: bool,
    video: bool,
) -> anyhow::Result<()> {
    ensure!((1..=3600).contains(&seconds), "seconds must be in 1..3600");
    ensure!(
        output.is_some() || connect,
        "choose --output or --connect (local viewer)"
    );
    let map = map_from(map_path)?;
    let mut adapter = GameAdapter::connect().context("game bridge unavailable")?;
    let builder = rr::RecordingStreamBuilder::new("etsads/live-read-only");
    let rec = if let Some(path) = output {
        reserve_output(path)?;
        builder.save(path)?
    } else {
        builder.connect_grpc_opts("rerun+http://127.0.0.1:9876/proxy")?
    };
    rec.log_static("status/source",&rr::TextDocument::new("Live read-only GameAdapter and capture reader. Exposure QPC selects bracketed historical physics/camera state within 16 ms; publication delay is recorded separately. Image mode publishes scene and camera from that same historical snapshot. C1 is opt-in and remains uncalibrated. SDK application delay is not effective actuator-response latency. No driving acceptance."))?;
    let mut sink = None;
    let start = Instant::now();
    let mut frames = 0;
    let mut history = CameraHistory::default();
    let mut capture = if video {
        CaptureReader::connect().ok()
    } else {
        None
    };
    let mut retry_capture = Instant::now();
    let mut image_due = Instant::now();
    let mut pending_image = None;
    let mut image_shape = None;
    let mut last_image_at = None;
    let mut images = 0u64;
    let mut image_status = BTreeMap::<String, u64>::new();
    let mut commands =
        etsads_shm::SharedCell::<ControlCmd>::open(CONTROL_MAPPING, SCHEMA_VERSION).ok();
    let mut applied =
        etsads_shm::SharedCell::<AppliedControl>::open(APPLIED_MAPPING, SCHEMA_VERSION).ok();
    let mut delays = CommandDelays::default();
    let mut last_header = None;
    let mut last_paused_time = None;
    while start.elapsed() < Duration::from_secs(seconds) {
        match adapter.poll() {
            Ok(Some(snapshot)) => {
                if sink.is_none() {
                    sink = Some(Sink::new(
                        rec.clone(),
                        snapshot.raw.header.qpc,
                        snapshot.raw.qpc_frequency,
                        &map,
                    )?);
                }
                let sink = sink.as_ref().unwrap();
                history.observe(&snapshot);
                last_header = Some(snapshot.raw.header);
                last_paused_time = Some(snapshot.raw.paused_simulation_time_us);
                if !video {
                    sink.telemetry(&snapshot)?;
                    sink.scene(&snapshot)?;
                } else {
                    sink.telemetry_series(&snapshot)?;
                }
                if !video
                    && let Some(ext) = &snapshot.extension
                    && ext.available & 1 != 0
                {
                    CameraSample::new(ext.header, bytemuck::pod_read_unaligned(&ext.camera)).log(
                        &sink.recording,
                        None,
                        false,
                    )?;
                }
                frames += 1;
                if frames % 128 == 0 {
                    rec.flush_with_timeout(Duration::from_secs(5))?;
                }
            }
            Ok(None) => std::thread::sleep(Duration::from_millis(5)),
            Err(error) => {
                rec.log("status/adapter_error", &rr::TextLog::new(error.to_string()))?;
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        if retry_capture <= Instant::now() {
            retry_capture = Instant::now() + Duration::from_secs(2);
            if video && capture.is_none() {
                capture = CaptureReader::connect().ok();
            }
            if commands.is_none() {
                commands = etsads_shm::SharedCell::open(CONTROL_MAPPING, SCHEMA_VERSION).ok();
            }
            if applied.is_none() {
                applied = etsads_shm::SharedCell::open(APPLIED_MAPPING, SCHEMA_VERSION).ok();
            }
        }
        if let Some(command) = commands.as_ref().and_then(|mapping| mapping.latest()) {
            delays.command(command);
        }
        if let (Some(sink), Some(header), Some(value)) = (
            &sink,
            last_header,
            applied.as_ref().and_then(|mapping| mapping.latest()),
        ) && let Some(delay) = delays.applied(value, sink.qpc_frequency)
        {
            sink.command_delay(&delay, header.frame_id, last_paused_time)?;
        }
        if video
            && pending_image.is_none()
            && Instant::now() >= image_due
            && let Some(reader) = &mut capture
        {
            ensure!(
                sink.as_ref()
                    .is_none_or(|sink| sink.qpc_frequency == reader.qpc_frequency),
                "capture and telemetry QPC frequencies disagree"
            );
            match reader.poll() {
                Ok(Some(frame)) => {
                    let shape = (frame.width, frame.height, frame.format);
                    if image_shape.is_some_and(|old| old != shape) {
                        history.clear();
                    }
                    image_shape = Some(shape);
                    pending_image = Some((frame, Instant::now()));
                }
                Ok(None) => {}
                Err(error) => {
                    rec.log("status/capture", &rr::TextLog::new(error.to_string()))?;
                    capture = None;
                }
            }
        }
        if let (Some(sink), Some((frame, arrived))) = (&sink, &pending_image) {
            let associated = history.associate(frame.timestamp_qpc, sink.qpc_frequency);
            let wait = associated.status == ImageMatchStatus::MissingFuture
                && arrived.elapsed() < Duration::from_millis(40);
            if !wait {
                *image_status
                    .entry(format!("{:?}", associated.status))
                    .or_default() += 1;
                let header = associated.header.unwrap_or_default();
                let paused_time = history
                    .snapshot(header.frame_id)
                    .map(|snapshot| snapshot.raw.paused_simulation_time_us);
                sink.time(
                    frame.timestamp_qpc,
                    header.frame_id,
                    paused_time.unwrap_or(0),
                );
                if paused_time.is_none() {
                    sink.recording.disable_timeline("simulation");
                }
                if let Some(header) = associated.header
                    && let Some(snapshot) = history.snapshot(header.frame_id)
                    && associated.status == ImageMatchStatus::Aligned
                {
                    sink.telemetry_at(snapshot, frame.timestamp_qpc)?;
                    sink.scene_at(snapshot, frame.timestamp_qpc)?;
                } else {
                    for path in [
                        "world/ego",
                        "world/agents",
                        "world/signals",
                        "world/route",
                        "world/camera",
                    ] {
                        rec.log(path, &rr::Clear::recursive())?;
                    }
                }
                let camera_model = interior_c1 && associated.camera.is_some();
                if let Some(camera) = associated.camera {
                    CameraSample::new(header, camera).log(
                        &rec,
                        Some([frame.width, frame.height]),
                        camera_model,
                    )?;
                }
                let image = etsads_visualization::media::encode_live_image(frame)?;
                rec.log("video/image", &image)?;
                if camera_model {
                    rec.log("world/camera/image", &image)?;
                } else {
                    rec.log("world/camera", &rr::Clear::recursive())?;
                }
                rec.log("video/alignment",&rr::TextLog::new(serde_json::json!({"capture_frame":frame.frame_index,
                    "exposure_qpc":frame.timestamp_qpc,"publication_qpc":frame.publish_qpc,
                    "readback_delay_ms":frame.publish_qpc.saturating_sub(frame.timestamp_qpc) as f64*1000.0/sink.qpc_frequency as f64,
                    "physics_frame":associated.header.map(|h|h.frame_id),"source_physics_qpc":associated.header.map(|h|h.qpc),
                    "source_sim_time_us":associated.header.map(|h|h.sim_time_us),"presentation_paused_sim_time_us":paused_time,
                    "status":associated.status,"camera_delta_ms":associated.delta_ms,
                    "camera_model":"explicit C1 diagnostic only; absolute calibration unverified"}).to_string()))?;
                images += 1;
                last_image_at = Some(Instant::now());
                pending_image = None;
                image_due = Instant::now() + Duration::from_millis(100);
            }
        }
        if last_image_at.is_some_and(|last: Instant| last.elapsed() > Duration::from_millis(500)) {
            if let Some(sink) = &sink {
                sink.time(
                    etsads_shm::qpc(),
                    last_header.map_or(0, |header| header.frame_id),
                    last_paused_time.unwrap_or(0),
                );
                if last_paused_time.is_none() {
                    sink.recording.disable_timeline("simulation");
                }
            }
            for path in [
                "world/ego",
                "world/agents",
                "world/signals",
                "world/route",
                "world/camera",
                "video/image",
            ] {
                rec.log(path, &rr::Clear::recursive())?;
            }
            rec.log(
                "status/capture",
                &rr::TextLog::new("capture stale for 500 ms; image and associated scene cleared"),
            )?;
            last_image_at = None;
        }
    }
    let report = serde_json::json!({"execution":"read_only_live_diagnostic","telemetry_frames":frames,"video_requested":video,
        "images":images,"image_association_status":image_status,"phase_acceptance":false,"control_written":false});
    rec.log_static(
        "status/live_summary",
        &rr::TextDocument::new(serde_json::to_string_pretty(&report)?),
    )?;
    rec.flush_with_timeout(Duration::from_secs(5))?;
    if let Some(path) = output {
        fs::write(
            path.with_extension("report.json"),
            serde_json::to_vec_pretty(&report)?,
        )?;
    }
    ensure!(frames > 0, "no valid game frames received");
    ensure!(
        !video || images > 0,
        "video requested but no capture images received"
    );
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn emitted_arrow_geometry_camera_and_video_share_exposure_time() {
        use etsads_msgs::protocol::{ExtensionFrame, WireHeader};
        let raw = TelemetryFrame {
            header: WireHeader {
                frame_id: 12,
                qpc: 120,
                sim_time_us: 99,
                ..Default::default()
            },
            paused_simulation_time_us: 7,
            qpc_frequency: 1000,
            ..Default::default()
        };
        let camera = etsads_game::ets2la::Camera {
            fov: 80.0,
            position: [0.0, 2.0, 0.0],
            sector: [0, 0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            projection: glam::Mat4::perspective_rh_gl(1.2, 1.6, 0.1, 1000.0)
                .transpose()
                .to_cols_array(),
            truck_position: [0.0; 3],
            truck_quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let mut extension = Box::<ExtensionFrame>::default();
        extension.header = raw.header;
        extension.available = 3;
        extension
            .camera
            .copy_from_slice(bytemuck::bytes_of(&camera));
        let mut actor: etsads_game::ets2la::Traffic = bytemuck::Zeroable::zeroed();
        actor.id = 42;
        actor.position = [1.0, 2.0, -10.0];
        actor.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        actor.size = [2.5, 3.0, 7.0];
        extension.traffic[..std::mem::size_of_val(&actor)]
            .copy_from_slice(bytemuck::bytes_of(&actor));
        let snapshot =
            etsads_game::adapter::snapshot_from_recorded_frame(raw, Some(extension), None).unwrap();
        let (recording, storage) =
            rr::RecordingStreamBuilder::new("synthetic-exposure-timeline-regression")
                .memory()
                .unwrap();
        let sink = Sink::new(recording, 100, 1000, &MapContext::default()).unwrap();
        sink.telemetry_series(&snapshot).unwrap();
        let index = VideoIndex {
            video_frame: 0,
            capture_frame: 1,
            capture_qpc: 119,
            physics_frame: 12,
            physics_qpc: 120,
            sim_time_us: 99,
            ..Default::default()
        };
        present_video(&sink, index, Some(&snapshot), 0, [320, 200], true).unwrap();
        sink.recording.flush_blocking().unwrap();
        let elapsed: rr::time::TimelineName = "elapsed".into();
        let simulation: rr::time::TimelineName = "simulation".into();
        let mut checked = std::collections::BTreeSet::new();
        for message in storage.take() {
            if let rr::log::LogMsg::ArrowMsg(_, arrow) = message {
                let chunk = rr::log::Chunk::from_arrow_msg(&arrow).unwrap();
                let path = chunk.entity_path().to_string();
                let path = path.trim_start_matches('/');
                if [
                    "world/ego/reference",
                    "world/agents/footprints",
                    "world/agents/centers",
                    "world/agents/native_boxes",
                    "world/agents/native_box_labels",
                    "world/camera",
                    "world/camera/image",
                    "video/image",
                ]
                .contains(&path)
                {
                    assert!(
                        chunk.timelines()[&elapsed]
                            .times_raw()
                            .iter()
                            .all(|time| *time == 19_000_000),
                        "wrong exposure timeline for {path}"
                    );
                    assert!(
                        chunk.timelines()[&simulation]
                            .times_raw()
                            .iter()
                            .all(|time| *time == 7000),
                        "raw simulation time leaked into paused timeline for {path}"
                    );
                    checked.insert(path.to_owned());
                }
            }
        }
        assert_eq!(
            checked.len(),
            8,
            "all geometry and image records must be inspected"
        );
    }

    #[test]
    fn media_file_reader_rejects_growth_beyond_its_explicit_budget() {
        let path = std::env::temp_dir().join(format!(
            "etsads-bounded-media-{}-{}",
            std::process::id(),
            etsads_shm::qpc()
        ));
        fs::write(&path, [1, 2, 3, 4, 5]).unwrap();
        assert!(bounded_file(&path, 4).is_err());
        assert_eq!(bounded_file(&path, 5).unwrap(), vec![1, 2, 3, 4, 5]);
        fs::remove_file(path).unwrap();
    }
}
