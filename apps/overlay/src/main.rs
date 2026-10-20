//! Independent diagnostic executable; no driving command client is linked here.
use anyhow::{Context, ensure};
use clap::{Args, Parser, Subcommand};
use etsads_game::{
    adapter::{GameAdapter, GameSnapshot},
    capture::CaptureReader,
    ets2la::Camera,
};
use etsads_msgs::protocol::{
    BridgeInfo, ExtensionFrame, INFO_MAPPING, OverlayFrame, SCHEMA_VERSION, TelemetryFrame,
    VideoIndex,
};
use etsads_overlay::{
    CaptureStamp, DiagnosticPublisher, FreshnessGate, ProjectionOptions, ProjectionOutput,
    TimingContext, disabled, generate, replay_image_aligned,
};
use etsads_recorder::{Record, Recorder, RunMetadata};
use etsads_shm::{SharedCell, qpc, qpc_frequency};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{BufWriter, Read, Write},
    path::{Path, PathBuf},
    process::Command,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

#[derive(Parser)]
#[command(
    about = "UNCALIBRATED overlay diagnostics. Explicit hypothesis, mandatory evidence, no driving-input client."
)]
struct Cli {
    #[command(subcommand)]
    mode: Mode,
}
#[derive(Subcommand)]
enum Mode {
    /// Validate an existing diagnostic MCAP; never opens game mappings.
    Verify {
        #[arg(long)]
        run: PathBuf,
    },
    Replay {
        #[arg(long)]
        run: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long, default_value = "ffprobe")]
        ffprobe: String,
        #[command(flatten)]
        projection: ProjectionArgs,
    },
    Live {
        #[arg(long)]
        output: PathBuf,
        #[arg(long,default_value_t=30,value_parser=clap::value_parser!(u64).range(1..=3600))]
        seconds: u64,
        #[command(flatten)]
        projection: ProjectionArgs,
    },
}
#[derive(Args)]
struct ProjectionArgs {
    /// Required: 0..7 are hypotheses, never a global calibration assertion.
    #[arg(long,value_parser=clap::value_parser!(u8).range(0..=7))]
    hypothesis: u8,
    #[arg(long, default_value_t = 200.0)]
    maximum_distance_m: f64,
    #[arg(long, default_value_t = 32)]
    maximum_boxes: usize,
    #[arg(long, default_value_t = 2.0)]
    line_width_px: f32,
}
impl ProjectionArgs {
    fn options(self) -> anyhow::Result<ProjectionOptions> {
        ProjectionOptions {
            hypothesis: self.hypothesis,
            maximum_distance_m: self.maximum_distance_m,
            maximum_boxes: self.maximum_boxes,
            line_width_px: self.line_width_px,
        }
        .validate()
    }
}
#[derive(Default, Serialize)]
struct Summary {
    fully_calibrated: bool,
    mode: String,
    telemetry_frames: u64,
    extension_frames: u64,
    capture_frames: u64,
    enabled_frames: u64,
    published_frames: u64,
    published_enabled_frames: u64,
    cleared_frames: u64,
    projected_actor_observations: u64,
    maximum_rectangles: u32,
    reasons: BTreeMap<String, u64>,
}
struct Audit {
    recorder: Recorder,
    ledger: BufWriter<fs::File>,
    output: PathBuf,
    summary: Summary,
}
impl Audit {
    fn create(
        output: &Path,
        mode: &str,
        patch: &str,
        options: ProjectionOptions,
        environment: serde_json::Value,
    ) -> anyhow::Result<Self> {
        if let Some(parent) = output.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::create_dir(output).context("--output must be a new directory")?;
        let mut meta = RunMetadata::new(
            if mode == "live" {
                "game"
            } else {
                "game_replay_diagnostic"
            },
            patch,
        );
        meta.notes = vec![
            "fully_calibrated=false; explicit projection hypothesis; visibility/occlusion unknown"
                .into(),
            "Only diagnostic OverlayFrame publication; no driving-input client".into(),
        ];
        meta.environment = serde_json::json!({"mode":mode,"projection":options,"fully_calibrated":false,"context":environment,
            "payloads":{"/telemetry":"TelemetryFrame ABI","/extension":"ExtensionFrame ABI","/camera":"ETS2LA Camera packed ABI","/overlay":"OverlayFrame ABI","/bridge_info":"BridgeInfo raw SDK versions, NOT game patch","/capture_metadata":"JSON","/overlay_status":"JSON"}});
        let recorder = Recorder::create(output, meta)?;
        let ledger = BufWriter::new(
            fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(output.join("evidence.jsonl"))?,
        );
        Ok(Self {
            recorder,
            ledger,
            output: output.into(),
            summary: Summary {
                mode: mode.into(),
                ..Default::default()
            },
        })
    }
    fn json(
        &mut self,
        topic: &str,
        value: impl Serialize,
        frame: u64,
        sim: u64,
        at: u64,
    ) -> anyhow::Result<()> {
        let payload = serde_json::to_vec(&value)?;
        self.recorder.record(
            topic,
            &Record {
                schema_version: SCHEMA_VERSION,
                frame_id: frame,
                sim_time_us: sim,
                qpc: at,
                raw_payload: payload.clone(),
                ..Default::default()
            },
        )?;
        self.ledger.write_all(&payload)?;
        self.ledger.write_all(b"\n")?;
        self.ledger.flush()?;
        Ok(())
    }
    fn source(&mut self, raw: &TelemetryFrame, ext: Option<&ExtensionFrame>) -> anyhow::Result<()> {
        self.recorder
            .record("/telemetry", &Record::telemetry(raw))?;
        self.summary.telemetry_frames += 1;
        if let Some(ext) = ext {
            self.recorder.record(
                "/extension",
                &Record::wire(
                    ext,
                    ext.header.frame_id,
                    ext.header.sim_time_us,
                    ext.header.qpc,
                ),
            )?;
            self.summary.extension_frames += 1;
            if ext.available & 1 != 0 {
                let camera = bytemuck::pod_read_unaligned::<Camera>(&ext.camera);
                self.recorder.record(
                    "/camera",
                    &Record::wire(
                        &camera,
                        ext.header.frame_id,
                        ext.header.sim_time_us,
                        ext.header.qpc,
                    ),
                )?;
            }
        }
        Ok(())
    }
    fn capture(
        &mut self,
        stamp: CaptureStamp,
        received: u64,
        frequency: u64,
        source: &str,
    ) -> anyhow::Result<()> {
        self.summary.capture_frames += 1;
        self.json("/capture_metadata",serde_json::json!({"event":"capture","source":source,"metadata":stamp,"received_qpc":received,"qpc_frequency":frequency}),0,0,received)
    }
    fn frame(&mut self, output: &ProjectionOutput, sim: u64) -> anyhow::Result<()> {
        self.recorder.record(
            "/overlay",
            &Record::wire(
                output.frame.as_ref(),
                output.frame.source_frame_id,
                sim,
                output.frame.write_qpc,
            ),
        )?;
        self.json(
            "/overlay_status",
            &output.evidence,
            output.frame.source_frame_id,
            sim,
            output.frame.write_qpc,
        )?;
        if output.frame.enabled == 1 {
            self.summary.enabled_frames += 1;
        } else {
            self.summary.cleared_frames += 1;
        }
        self.summary.projected_actor_observations += output.evidence.projected_actors.len() as u64;
        self.summary.maximum_rectangles = self.summary.maximum_rectangles.max(output.frame.count);
        *self
            .summary
            .reasons
            .entry(output.evidence.reason.clone())
            .or_default() += 1;
        Ok(())
    }
    fn clear(&mut self, reason: &str, frame: u64, sim: u64, at: u64) -> anyhow::Result<()> {
        self.recorder.record(
            "/overlay",
            &Record::wire(disabled(at, frame).as_ref(), frame, sim, at),
        )?;
        self.json("/overlay_status",serde_json::json!({"event":"clear","fully_calibrated":false,"reason":reason,"source_frame_id":frame,"qpc":at}),frame,sim,at)?;
        self.summary.cleared_frames += 1;
        *self.summary.reasons.entry(reason.into()).or_default() += 1;
        Ok(())
    }
    fn finish(mut self, result: anyhow::Result<()>) -> anyhow::Result<()> {
        if let Err(error) = &result {
            self.json("/overlay_status",serde_json::json!({"event":"error","fully_calibrated":false,"source":format!("{error:#}")}),0,0,qpc())?;
        }
        self.ledger.flush()?;
        self.ledger.get_ref().sync_all()?;
        fs::write(
            self.output.join("summary.json"),
            serde_json::to_vec_pretty(&self.summary)?,
        )?;
        println!("{}", serde_json::to_string_pretty(&self.summary)?);
        match result {
            Ok(()) => {
                self.recorder.finish()?;
                Ok(())
            }
            Err(error) => {
                self.recorder.finish_incomplete(format!("{error:#}"))?;
                Err(error)
            }
        }
    }
}
fn main() -> anyhow::Result<()> {
    std::thread::Builder::new()
        .name("diagnostic-overlay".into())
        .stack_size(8 * 1024 * 1024)
        .spawn(|| match Cli::parse().mode {
            Mode::Verify { run } => verify(&run),
            Mode::Replay {
                run,
                output,
                ffprobe,
                projection,
            } => replay(&run, &output, &ffprobe, projection.options()?),
            Mode::Live {
                output,
                seconds,
                projection,
            } => live(&output, seconds, projection.options()?),
        })?
        .join()
        .map_err(|_| anyhow::anyhow!("overlay diagnostic worker panicked"))?
}
fn verify(run: &Path) -> anyhow::Result<()> {
    let mut overlay_frames = 0_u64;
    let mut enabled_frames = 0_u64;
    let mut labelled_records = 0_u64;
    let mut maximum_rectangles = 0;
    let report = etsads_recorder::replay(run, |topic, record| {
        if topic == "/overlay" {
            let frame = record.decode_wire::<OverlayFrame>()?;
            ensure!(
                frame.enabled <= 1 && frame.count <= 256 && frame.write_qpc > 0,
                "invalid overlay header"
            );
            ensure!(
                frame.enabled != 0 || frame.count == 0,
                "disabled frame retains primitives"
            );
            for rect in frame.rectangles.iter().take(frame.count as usize) {
                let [l, t, r, b] = rect.bounds;
                ensure!(
                    rect.bounds.iter().all(|v| v.is_finite())
                        && 0. <= l
                        && l < r
                        && r <= 16384.
                        && 0. <= t
                        && t < b
                        && b <= 16384.,
                    "invalid overlay geometry"
                );
                ensure!(
                    rect.color_rgba
                        .iter()
                        .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
                    "invalid overlay color"
                );
            }
            overlay_frames += 1;
            enabled_frames += u64::from(frame.enabled == 1);
            maximum_rectangles = maximum_rectangles.max(frame.count);
        } else if topic == "/overlay_status" {
            let evidence: serde_json::Value = serde_json::from_slice(&record.raw_payload)?;
            ensure!(
                evidence["fully_calibrated"] == false,
                "diagnostic calibration claim must remain false"
            );
            if evidence["enabled"] == true {
                ensure!(
                    evidence["label"]
                        .as_str()
                        .is_some_and(|s| s.starts_with("UNCAL C")),
                    "enabled projection lacks uncalibrated label"
                );
                labelled_records += 1;
            }
        }
        Ok(())
    })?;
    ensure!(
        report.completed && overlay_frames > 0,
        "recording is incomplete or has no overlay frames"
    );
    ensure!(
        enabled_frames == labelled_records,
        "overlay frames and labelled evidence disagree"
    );
    let value = serde_json::json!({"source":report.source,"completed":report.completed,"messages":report.messages,"telemetry_frames":report.telemetry_frames,"nonmonotonic_telemetry":report.nonmonotonic,"overlay_frames":overlay_frames,"enabled_frames":enabled_frames,"labelled_records":labelled_records,"maximum_rectangles":maximum_rectangles,"fully_calibrated":false,"topics":report.topics,"verification":"wire and evidence invariants only; no game rendering acceptance"});
    fs::write(
        run.join("verification.json"),
        serde_json::to_vec_pretty(&value)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&value)?);
    Ok(())
}
fn hash(path: &Path) -> anyhow::Result<String> {
    let mut file = fs::File::open(path)?;
    let mut sha = Sha256::new();
    let mut bytes = vec![0; 1024 * 1024];
    loop {
        let n = file.read(&mut bytes)?;
        if n == 0 {
            break;
        }
        sha.update(&bytes[..n]);
    }
    Ok(format!("{:x}", sha.finalize()))
}
fn decode_extension(record: &Record) -> anyhow::Result<Box<ExtensionFrame>> {
    ensure!(
        record.raw_payload.len() == std::mem::size_of::<ExtensionFrame>(),
        "extension ABI mismatch"
    );
    let mut ext = etsads_shm::zeroed_box::<ExtensionFrame>();
    bytemuck::bytes_of_mut(ext.as_mut()).copy_from_slice(&record.raw_payload);
    Ok(ext)
}
fn replay(
    run: &Path,
    output: &Path,
    ffprobe: &str,
    options: ProjectionOptions,
) -> anyhow::Result<()> {
    let meta: serde_json::Value = serde_json::from_reader(fs::File::open(run.join("meta.json"))?)?;
    ensure!(
        meta["source"] == "game",
        "replay requires an original game recording"
    );
    let video = run.join("video-0000.mp4");
    ensure!(
        video.is_file(),
        "recorded video is required to establish viewport dimensions; no default resolution is assumed"
    );
    let probe = Command::new(ffprobe)
        .args([
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height",
            "-of",
            "json",
        ])
        .arg(&video)
        .output()?;
    ensure!(
        probe.status.success(),
        "ffprobe failed: {}",
        String::from_utf8_lossy(&probe.stderr)
    );
    let dimensions: serde_json::Value = serde_json::from_slice(&probe.stdout)?;
    let width = u32::try_from(
        dimensions["streams"][0]["width"]
            .as_u64()
            .context("missing video width")?,
    )?;
    let height = u32::try_from(
        dimensions["streams"][0]["height"]
            .as_u64()
            .context("missing video height")?,
    )?;
    let mut telemetry = BTreeMap::new();
    let mut indices = BTreeMap::<u64, Vec<VideoIndex>>::new();
    etsads_recorder::replay(run, |topic, record| {
        match topic {
            "/telemetry" => {
                let raw = record.decode_wire::<TelemetryFrame>()?;
                ensure!(
                    telemetry.insert(raw.header.frame_id, raw).is_none(),
                    "duplicate physical IDs: split sessions before replay"
                );
            }
            "/video_index" => {
                let index = record.decode_wire::<VideoIndex>()?;
                indices.entry(index.physics_frame).or_default().push(index);
            }
            _ => {}
        }
        Ok(())
    })?;
    ensure!(!indices.is_empty(), "recorded video_index missing");
    let mut sources = BTreeMap::new();
    for entry in fs::read_dir(run)? {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "mcap" || e == "mp4")
            || path.file_name().is_some_and(|e| e == "meta.json")
        {
            sources.insert(
                path.file_name().unwrap().to_string_lossy().into_owned(),
                hash(&path)?,
            );
        }
    }
    let mut audit = Audit::create(
        output,
        "replay",
        meta["game_version"].as_str().unwrap_or("unknown"),
        options,
        serde_json::json!({"source_run":run,"source_sha256":sources,"width":width,"height":height,"viewport_source":"ffprobe recorded MP4","availability_time_unknown":true,"capture_generation_unknown":true,"note":"Offline geometry only. Evaluation time=max(camera,telemetry,capture) is a replay assumption, not live freshness acceptance."}),
    )?;
    let result = (|| -> anyhow::Result<()> {
        let mut processed = BTreeSet::new();
        etsads_recorder::replay(run, |topic, record| {
            if topic == "/bridge_info" {
                audit.recorder.record(topic, record)?;
            }
            if topic != "/extension" {
                return Ok(());
            }
            let ext = decode_extension(record)?;
            let Some(frames) = indices.get(&ext.header.frame_id) else {
                return Ok(());
            };
            let Some(raw) = telemetry.get(&ext.header.frame_id) else {
                return Ok(());
            };
            audit.source(raw, Some(&ext))?;
            for index in frames {
                let stamp = CaptureStamp {
                    width,
                    height,
                    frame_index: index.capture_frame,
                    timestamp_qpc: index.capture_qpc,
                    publish_qpc: None,
                    generation: None,
                };
                let now = ext.header.qpc.max(raw.header.qpc).max(index.capture_qpc);
                audit.capture(
                    stamp,
                    now,
                    raw.qpc_frequency,
                    "recorded_video_index; publish/generation unknown",
                )?;
                if !replay_image_aligned(ext.header.qpc, index.capture_qpc, raw.qpc_frequency) {
                    audit.clear(
                        "replay_camera_capture_delta_exceeds_16ms",
                        raw.header.frame_id,
                        raw.header.sim_time_us,
                        now,
                    )?;
                } else {
                    let mut generated = generate(raw, Some(&ext), stamp, options, now)?;
                    generated.evidence.timing_context = TimingContext::ReplayImageAligned;
                    if audit.summary.enabled_frames.is_multiple_of(1200)
                        && generated.frame.enabled == 1
                    {
                        svg(
                            &output.join(format!("projection-{:06}.svg", index.video_frame)),
                            &generated.frame,
                            width,
                            height,
                        )?;
                    }
                    audit.frame(&generated, raw.header.sim_time_us)?;
                }
                ensure!(
                    processed.insert(index.video_frame),
                    "duplicate encoded video index"
                );
            }
            Ok(())
        })?;
        for index in indices.values().flatten() {
            if !processed.contains(&index.video_frame) {
                audit.clear(
                    "recorded_index_has_no_matching_telemetry_extension",
                    index.physics_frame,
                    index.sim_time_us,
                    index.capture_qpc,
                )?;
            }
        }
        ensure!(
            audit.summary.enabled_frames > 0,
            "no usable unpaused camera frames in replay"
        );
        Ok(())
    })();
    audit.finish(result)
}
fn svg(path: &Path, frame: &OverlayFrame, width: u32, height: u32) -> anyhow::Result<()> {
    let mut text = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {width} {height}\"><title>Uncalibrated projection primitives; no image or occlusion assertion</title><rect width=\"100%\" height=\"100%\" fill=\"#171b22\"/>"
    );
    for r in frame.rectangles.iter().take(frame.count as usize) {
        let [l, t, right, b] = r.bounds;
        let c = r.color_rgba.map(|v| (v.clamp(0., 1.) * 255.).round() as u8);
        text += &format!(
            "<rect x=\"{l}\" y=\"{t}\" width=\"{}\" height=\"{}\" fill=\"rgb({},{},{})\"/>",
            right - l,
            b - t,
            c[0],
            c[1],
            c[2]
        );
    }
    text += "</svg>";
    fs::write(path, text)?;
    Ok(())
}
#[derive(Default)]
struct LivePublicationState {
    enabled: bool,
    last_frame: Option<u64>,
    blocked_reason: Option<String>,
}
impl LivePublicationState {
    fn clear(
        &mut self,
        publisher: &DiagnosticPublisher,
        audit: &mut Audit,
        reason: &str,
        frame: u64,
        sim: u64,
    ) -> anyhow::Result<()> {
        if self.enabled || self.blocked_reason.as_deref() != Some(reason) {
            let at = qpc();
            publisher.publish(&disabled(at, frame))?;
            audit.clear(reason, frame, sim, at)?;
        }
        self.enabled = false;
        self.last_frame = None;
        self.blocked_reason = Some(reason.into());
        Ok(())
    }
}
fn live(output: &Path, seconds: u64, options: ProjectionOptions) -> anyhow::Result<()> {
    let mut audit = Audit::create(
        output,
        "live",
        "unknown; BridgeInfo contains raw SDK game versions, not PE patch",
        options,
        serde_json::json!({
            "seconds":seconds,"image_pixels_copied":false,"maximum_sample_age_ms":100,
            "maximum_metadata_publication_age_ms":100,"consumer_expiry_ms":200,
            "timing_context":"live_latest_for_presentation",
            "capture_role":"viewport_and_producer_activity_only",
            "capture_is_image_match":false,
            "note":"Capture exposure is an older staging readback. Only replay aligns a camera to that image within 16 ms."
        }),
    )?;
    let stop = Arc::new(AtomicBool::new(false));
    let stop_handler = stop.clone();
    let result = (|| -> anyhow::Result<()> {
        ctrlc::set_handler(move || stop_handler.store(true, Ordering::Relaxed))
            .context("could not install diagnostic shutdown handler")?;
        let publisher = DiagnosticPublisher::acquire_live()?;
        let mut published = LivePublicationState::default();
        published.clear(&publisher, &mut audit, "publisher_started_disabled", 0, 0)?;
        let mut adapter = GameAdapter::connect().context("telemetry bridge unavailable")?;
        let mut capture =
            CaptureReader::connect().context("capture metadata mapping unavailable")?;
        let bridge = SharedCell::<BridgeInfo>::open(INFO_MAPPING, SCHEMA_VERSION)
            .context("BridgeInfo mapping unavailable")?;
        let frequency = qpc_frequency();
        ensure!(
            capture.qpc_frequency == frequency,
            "capture and host clocks disagree"
        );

        // There is intentionally no historical camera queue in Live. The capture
        // publisher may expose an image from Present n-3; drawing is for a later
        // Present and must use the newest coherent camera/traffic snapshot.
        let mut latest_snapshot: Option<GameSnapshot> = None;
        let mut latest_capture: Option<CaptureStamp> = None;
        let mut gate = FreshnessGate::default();
        let mut session = None;
        let mut last_bridge_bytes = Vec::new();
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(seconds) && !stop.load(Ordering::Relaxed) {
            let info = bridge.latest();
            if let Some(info) = &info {
                if bytemuck::bytes_of(info) != last_bridge_bytes.as_slice() {
                    audit
                        .recorder
                        .record("/bridge_info", &Record::wire(info, 0, 0, qpc()))?;
                    last_bridge_bytes = bytemuck::bytes_of(info).to_vec();
                }
                if session != Some(info.session_start_qpc) {
                    published.clear(&publisher, &mut audit, "bridge_session_transition", 0, 0)?;
                    latest_snapshot = None;
                    latest_capture = None;
                    gate.reset();
                    session = Some(info.session_start_qpc);
                }
            } else {
                published.clear(&publisher, &mut audit, "bridge_info_missing", 0, 0)?;
                latest_snapshot = None;
                latest_capture = None;
                gate.reset();
            }

            match capture.poll_metadata() {
                Ok(Some(metadata)) => {
                    let stamp = CaptureStamp::from(metadata);
                    audit.capture(
                        stamp,
                        qpc(),
                        frequency,
                        "live viewport/producer metadata; NOT image alignment",
                    )?;
                    latest_capture = Some(stamp);
                }
                Ok(None) => {}
                Err(error) => {
                    latest_capture = None;
                    published.clear(
                        &publisher,
                        &mut audit,
                        &format!("capture_error:{error}"),
                        0,
                        0,
                    )?;
                }
            }
            match adapter.poll() {
                Ok(Some(snapshot)) => {
                    audit.source(&snapshot.raw, snapshot.extension.as_deref())?;
                    // A paused or telemetry-only latest snapshot replaces the
                    // previous camera as well; no fallback to an old active one.
                    latest_snapshot = Some(snapshot);
                }
                Ok(None) => {}
                Err(error) => {
                    latest_snapshot = None;
                    published.clear(
                        &publisher,
                        &mut audit,
                        &format!("adapter_error:{error}"),
                        0,
                        0,
                    )?;
                }
            }

            (|| -> anyhow::Result<()> {
                let Some(snapshot) = latest_snapshot.as_ref() else {
                    published.clear(&publisher, &mut audit, "latest_snapshot_missing", 0, 0)?;
                    return Ok(());
                };
                let Some(stamp) = latest_capture else {
                    published.clear(
                        &publisher,
                        &mut audit,
                        "capture_metadata_missing",
                        snapshot.raw.header.frame_id,
                        snapshot.raw.header.sim_time_us,
                    )?;
                    return Ok(());
                };
                if !info
                    .as_ref()
                    .is_some_and(|i| i.session_start_qpc == snapshot.session_start_qpc)
                {
                    published.clear(
                        &publisher,
                        &mut audit,
                        "snapshot_session_mismatch",
                        snapshot.raw.header.frame_id,
                        snapshot.raw.header.sim_time_us,
                    )?;
                    return Ok(());
                }
                let now = qpc();
                let timing = match gate.check(
                    &snapshot.raw,
                    snapshot.extension.as_deref(),
                    stamp,
                    info.as_ref(),
                    now,
                    frequency,
                ) {
                    Ok(timing) => timing,
                    Err(reason) => {
                        published.clear(
                            &publisher,
                            &mut audit,
                            reason,
                            snapshot.raw.header.frame_id,
                            snapshot.raw.header.sim_time_us,
                        )?;
                        return Ok(());
                    }
                };
                // Check freshness even for an already-published source frame,
                // but do not refresh its write_qpc indefinitely.
                if published.last_frame == Some(snapshot.raw.header.frame_id) {
                    return Ok(());
                }
                let mut generated = generate(
                    &snapshot.raw,
                    snapshot.extension.as_deref(),
                    stamp,
                    options,
                    now,
                )?;
                generated.evidence.timing_context = TimingContext::LiveLatestForPresentation;
                generated.evidence.live_timing = Some(timing);
                audit.frame(&generated, snapshot.raw.header.sim_time_us)?;
                let after_record = qpc();
                if (after_record - now) as u128 * 1000 > frequency as u128 * 50 {
                    published.clear(
                        &publisher,
                        &mut audit,
                        "recording_delay_exceeds_50ms",
                        snapshot.raw.header.frame_id,
                        snapshot.raw.header.sim_time_us,
                    )?;
                    return Ok(());
                }

                // Detect a resize/new capture generation that became available
                // while writing evidence, before publishing old-view coordinates.
                match capture.poll_metadata() {
                    Ok(Some(metadata)) => {
                        let latest = CaptureStamp::from(metadata);
                        audit.capture(
                            latest,
                            qpc(),
                            frequency,
                            "pre-publish viewport/producer metadata; NOT image alignment",
                        )?;
                        latest_capture = Some(latest);
                    }
                    Ok(None) => {}
                    Err(error) => {
                        latest_capture = None;
                        published.clear(
                            &publisher,
                            &mut audit,
                            &format!("pre_publish_capture_error:{error}"),
                            snapshot.raw.header.frame_id,
                            snapshot.raw.header.sim_time_us,
                        )?;
                        return Ok(());
                    }
                }
                let publish_stamp = latest_capture.expect("capture retained or updated above");
                let publish_info = bridge.latest();
                let publish_timing = match gate.check(
                    &snapshot.raw,
                    snapshot.extension.as_deref(),
                    publish_stamp,
                    publish_info.as_ref(),
                    qpc(),
                    frequency,
                ) {
                    Ok(timing) => timing,
                    Err(reason) => {
                        published.clear(
                            &publisher,
                            &mut audit,
                            &format!("pre_publish_recheck:{reason}"),
                            snapshot.raw.header.frame_id,
                            snapshot.raw.header.sim_time_us,
                        )?;
                        return Ok(());
                    }
                };
                publisher.publish(&generated.frame)?;
                audit.summary.published_frames += 1;
                audit.summary.published_enabled_frames += u64::from(generated.frame.enabled == 1);
                audit.json(
                    "/overlay_published",
                    serde_json::json!({
                        "event":"published","fully_calibrated":false,
                        "timing_context":"live_latest_for_presentation",
                        "source_frame_id":generated.frame.source_frame_id,
                        "write_qpc":generated.frame.write_qpc,"published_qpc":qpc(),
                        "enabled":generated.frame.enabled,"rectangles":generated.frame.count,
                        "capture_metadata":publish_stamp,"live_timing":publish_timing
                    }),
                    snapshot.raw.header.frame_id,
                    snapshot.raw.header.sim_time_us,
                    qpc(),
                )?;
                published.enabled = generated.frame.enabled == 1;
                published.last_frame = Some(snapshot.raw.header.frame_id);
                published.blocked_reason = None;
                Ok(())
            })()?;
            std::thread::sleep(Duration::from_millis(5));
        }
        published.clear(&publisher, &mut audit, "normal_exit_or_ctrl_c", 0, 0)?;
        ensure!(
            audit.summary.published_enabled_frames > 0,
            "no fresh unpaused camera frames were produced"
        );
        Ok(())
    })();
    audit.finish(result)
}
