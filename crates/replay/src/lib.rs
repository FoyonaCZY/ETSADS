//! Bounded, read-only reconstruction of recorded GameSnapshots.
//!
//! This module never opens game mappings. Telemetry is emitted exactly once in
//! physical-frame order, using the same unit/scene conversion as GameAdapter.
//! Delayed extensions, configuration observations, events and video associations
//! are joined before delivery. Missing observations remain explicit.
use anyhow::{Context, ensure};
use etsads_game::adapter::{GameSnapshot, snapshot_from_recorded_frame};
use etsads_msgs::protocol::*;
use etsads_recorder::{Record, ReplayReport};
use serde::Serialize;
use std::{collections::BTreeMap, path::Path};

const LOOKAHEAD_FRAMES: usize = 64;
const MAX_ASSOCIATIONS: usize = 4096;

/// The frame/QPC at which the recorder observed a legacy configuration. Old
/// ConfigBlock messages do not contain the original SDK configuration-event
/// timestamp, so this must not be presented as an exact configuration boundary.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Observation {
    pub frame_id: u64,
    pub qpc: u64,
}

pub struct RecordedVideo {
    pub index: VideoIndex,
    pub alignment: Option<VideoAlignment>,
}

pub struct ReplayFrame {
    pub control_configuration: Option<ControlConfiguration>,
    pub snapshot: GameSnapshot,
    pub config_observation: Option<Observation>,
    pub configuration: Option<Box<ConfigurationFrame>>,
    pub dynamics: Option<DynamicsFrame>,
    pub camera_view: Option<CameraViewFrame>,
    pub video: Vec<RecordedVideo>,
}

#[derive(Default, Debug, Serialize)]
pub struct AssociationReport {
    pub control_configuration_frames: u64,
    pub unknown_control_configuration_frames: u64,
    pub snapshots: u64,
    pub extension_frames: u64,
    pub missing_extension_frames: u64,
    pub missing_config_frames: u64,
    pub precise_configuration_frames: u64,
    pub dynamics_frames: u64,
    pub camera_view_frames: u64,
    pub events_delivered: u64,
    pub video_indices: u64,
    pub video_without_alignment: u64,
    pub video_without_telemetry: u64,
    pub orphan_extensions: u64,
    pub orphan_sidecars: u64,
    pub tail_events: u64,
    pub maximum_buffered_frames: usize,
}

#[derive(Serialize)]
pub struct ReplaySummary {
    #[serde(flatten)]
    pub integrity: ReplayReport,
    pub associations: AssociationReport,
    pub execution: &'static str,
    pub phase_acceptance: bool,
}

#[derive(Default)]
struct Pending {
    raw: Option<TelemetryFrame>,
    extension: Option<Box<ExtensionFrame>>,
    dynamics: Option<DynamicsFrame>,
    camera_view: Option<CameraViewFrame>,
    video: BTreeMap<u64, RecordedVideo>,
}

/// Record-order join with at most 64 physical frames of lookahead. Late records
/// outside this window are errors, never silently attached to a newer frame.
/// SDK-session changes require separate recordings, matching the live recorder.
pub struct ReplayAdapter {
    frames: BTreeMap<u64, Pending>,
    configs: BTreeMap<u64, (Observation, ConfigBlock)>,
    config: Option<(Observation, ConfigBlock)>,
    full_configs: BTreeMap<(u64, u64, u64), Box<ConfigurationFrame>>,
    full_config: Option<Box<ConfigurationFrame>>,
    control_configurations: BTreeMap<(u64, u64, u64), ControlConfiguration>,
    control_configuration: Option<ControlConfiguration>,
    last_control_configuration: Option<ControlConfiguration>,
    has_precise_configuration_stream: bool,
    events: BTreeMap<(u64, u64, u64), GameEvent>,
    video_alignment: BTreeMap<u64, (u64, u64, VideoAlignment)>,
    unassociated_video: BTreeMap<u64, VideoIndex>,
    info: Option<BridgeInfo>,
    last_raw: Option<TelemetryFrame>,
    last_emitted: Option<u64>,
    last_video: Option<u64>,
    origin: [f64; 3],
    origin_generation: u64,
    report: AssociationReport,
}

impl Default for ReplayAdapter {
    fn default() -> Self {
        Self {
            frames: BTreeMap::new(),
            configs: BTreeMap::new(),
            config: None,
            full_configs: BTreeMap::new(),
            full_config: None,
            control_configurations: BTreeMap::new(),
            control_configuration: None,
            last_control_configuration: None,
            has_precise_configuration_stream: false,
            events: BTreeMap::new(),
            video_alignment: BTreeMap::new(),
            unassociated_video: BTreeMap::new(),
            info: None,
            last_raw: None,
            last_emitted: None,
            last_video: None,
            origin: [0.0; 3],
            origin_generation: 0,
            report: AssociationReport::default(),
        }
    }
}

impl ReplayAdapter {
    /// Validates the complete closed recording before delivering observations.
    /// The second pass is streamed and bounded. Consumer output is offline and
    /// must remain labelled replay, even when the source recording is a game.
    pub fn run(
        path: &Path,
        mut receive: impl FnMut(ReplayFrame) -> anyhow::Result<()>,
    ) -> anyhow::Result<ReplaySummary> {
        let integrity = etsads_recorder::replay(path, |_, _| Ok(()))?;
        ensure!(
            integrity.completed,
            "recording is incomplete; inspect integrity separately"
        );
        ensure!(
            integrity.nonmonotonic == 0,
            "split nonmonotonic/SDK sessions before replay"
        );
        let mut adapter = Self {
            has_precise_configuration_stream: integrity.topics.contains_key("/config_full"),
            ..Self::default()
        };
        etsads_recorder::replay(path, |topic, record| {
            adapter.push(topic, record, &mut receive)
        })?;
        adapter.finish(&mut receive)?;
        Ok(ReplaySummary {
            integrity,
            associations: adapter.report,
            execution: "read-only recorded snapshot replay",
            phase_acceptance: false,
        })
    }

    fn not_late(&self, frame: u64) -> anyhow::Result<()> {
        ensure!(
            self.last_emitted.is_none_or(|previous| frame > previous),
            "record for physical frame {frame} arrived after that frame was emitted"
        );
        Ok(())
    }

    fn check_session_qpc(&self, qpc: u64) -> anyhow::Result<()> {
        ensure!(
            self.info.is_none_or(|info| qpc >= info.session_start_qpc),
            "record predates SDK session"
        );
        Ok(())
    }

    fn check_camera_session(&self, view: &CameraViewFrame) -> anyhow::Result<()> {
        ensure!(
            self.info
                .is_none_or(|info| view.session_start_qpc == info.session_start_qpc),
            "camera-view belongs to another SDK session"
        );
        Ok(())
    }

    /// Identity may arrive after retained records, but never after delivery.
    /// Apply its lower bound retroactively before any buffered state is used.
    fn check_buffered_session(&self) -> anyhow::Result<()> {
        for controls in self.control_configurations.values() {
            self.check_session_qpc(controls.header.qpc)?;
            ensure!(
                self.info
                    .is_none_or(|info| info.session_start_qpc == controls.session_start_qpc),
                "controls configuration belongs to another SDK session"
            );
        }
        for frame in self.frames.values() {
            for header in frame
                .raw
                .as_ref()
                .map(|value| value.header)
                .into_iter()
                .chain(frame.extension.as_ref().map(|value| value.header))
                .chain(frame.dynamics.as_ref().map(|value| value.header))
                .chain(frame.camera_view.as_ref().map(|value| value.header))
            {
                self.check_session_qpc(header.qpc)?;
            }
            if let Some(view) = &frame.camera_view {
                self.check_camera_session(view)?;
            }
        }
        for qpc in self
            .configs
            .values()
            .map(|(observation, _)| observation.qpc)
            .chain(self.full_configs.values().map(|value| value.header.qpc))
            .chain(self.events.values().map(|value| value.header.qpc))
        {
            self.check_session_qpc(qpc)?;
        }
        Ok(())
    }

    pub fn push(
        &mut self,
        topic: &str,
        record: &Record,
        receive: &mut impl FnMut(ReplayFrame) -> anyhow::Result<()>,
    ) -> anyhow::Result<()> {
        if matches!(
            topic,
            "/telemetry"
                | "/extension"
                | "/config"
                | "/configuration"
                | "/config_full"
                | "/control_configuration"
                | "/dynamics"
                | "/camera_view"
                | "/events"
        ) {
            self.check_session_qpc(record.qpc)?;
        }
        match topic {
            "/bridge_info" => {
                let info: BridgeInfo = record.decode_wire()?;
                ensure!(
                    self.info
                        .is_none_or(|old| old.session_start_qpc == info.session_start_qpc
                            && old.process_id == info.process_id),
                    "SDK session changed in recording"
                );
                ensure!(
                    self.last_emitted.is_none(),
                    "bridge identity arrived after replay began"
                );
                self.info = Some(info);
                self.check_buffered_session()?;
            }
            "/telemetry" => {
                let raw: TelemetryFrame = record.decode_wire()?;
                check_header(record, raw.header)?;
                self.not_late(raw.header.frame_id)?;
                if let Some(previous) = self.last_raw {
                    ensure!(
                        raw.header.frame_id > previous.header.frame_id
                            && raw.header.qpc > previous.header.qpc
                            && raw.header.sim_time_us >= previous.header.sim_time_us
                            && raw.qpc_frequency == previous.qpc_frequency,
                        "nonmonotonic telemetry or changing QPC frequency"
                    );
                }
                ensure!(
                    self.info
                        .is_none_or(|info| raw.header.qpc >= info.session_start_qpc),
                    "telemetry predates SDK session"
                );
                let slot = self.frames.entry(raw.header.frame_id).or_default();
                ensure!(slot.raw.is_none(), "duplicate telemetry");
                slot.raw = Some(raw);
                self.last_raw = Some(raw);
            }
            "/extension" => {
                ensure!(
                    record.raw_payload.len() == std::mem::size_of::<ExtensionFrame>(),
                    "extension layout mismatch"
                );
                let mut extension = Box::<ExtensionFrame>::default();
                bytemuck::bytes_of_mut(extension.as_mut()).copy_from_slice(&record.raw_payload);
                check_header(record, extension.header)?;
                self.not_late(extension.header.frame_id)?;
                let slot = self.frames.entry(extension.header.frame_id).or_default();
                ensure!(slot.extension.is_none(), "duplicate extension frame");
                slot.extension = Some(extension);
            }
            "/config" | "/configuration" => {
                self.not_late(record.frame_id)?;
                let config: ConfigBlock = record.decode_wire()?;
                let observation = Observation {
                    frame_id: record.frame_id,
                    qpc: record.qpc,
                };
                ensure!(
                    self.configs
                        .insert(record.frame_id, (observation, config))
                        .is_none(),
                    "multiple configuration observations for one physical frame"
                );
            }
            "/control_configuration" => {
                let value: ControlConfiguration = record.decode_wire()?;
                check_header(record, value.header)?;
                self.not_late(value.effective_from_frame_id)?;
                ensure!(
                    value.session_start_qpc > 0
                        && value.header.qpc >= value.session_start_qpc
                        && self
                            .info
                            .is_none_or(|info| info.session_start_qpc == value.session_start_qpc),
                    "controls configuration belongs to another SDK session"
                );
                ensure!(
                    value.header.seq == value.generation
                        && value.header.frame_id.checked_add(1)
                            == Some(value.effective_from_frame_id)
                        && ((value.valid == 0 && value.shifter_type == SHIFTER_UNKNOWN)
                            || (value.valid == CONTROL_CONFIGURATION_SHIFTER_VALID
                                && (SHIFTER_ARCADE..=SHIFTER_HSHIFTER)
                                    .contains(&value.shifter_type))),
                    "invalid controls configuration boundary/enum"
                );
                ensure!(
                    self.last_control_configuration
                        .is_none_or(|old| value.generation > old.generation
                            && value.header.qpc >= old.header.qpc
                            && value.effective_from_frame_id >= old.effective_from_frame_id
                            && value.session_start_qpc == old.session_start_qpc),
                    "controls generation/session changed nonmonotonically"
                );
                self.last_control_configuration = Some(value);
                self.control_configurations.insert(
                    (
                        value.effective_from_frame_id,
                        value.header.qpc,
                        value.generation,
                    ),
                    value,
                );
            }
            "/config_full" => {
                self.has_precise_configuration_stream = true;
                let full: ConfigurationFrame = record.decode_wire()?;
                check_header(record, full.header)?;
                self.not_late(full.effective_from_frame_id)?;
                ensure!(
                    full.effective_from_frame_id
                        == full
                            .header
                            .frame_id
                            .checked_add(1)
                            .context("configuration frame overflow")?
                        && full.config_seq == full.truck.config_seq,
                    "invalid precise configuration boundary/sequence"
                );
                ensure!(
                    self.full_configs
                        .insert(
                            (
                                full.effective_from_frame_id,
                                full.header.qpc,
                                full.config_seq
                            ),
                            Box::new(full)
                        )
                        .is_none(),
                    "duplicate precise configuration event"
                );
            }
            "/dynamics" => {
                let value: DynamicsFrame = record.decode_wire()?;
                check_header(record, value.header)?;
                self.not_late(value.header.frame_id)?;
                ensure!(
                    self.frames
                        .entry(value.header.frame_id)
                        .or_default()
                        .dynamics
                        .replace(value)
                        .is_none(),
                    "duplicate dynamics frame"
                );
            }
            "/camera_view" => {
                let value: CameraViewFrame = record.decode_wire()?;
                check_header(record, value.header)?;
                self.check_camera_session(&value)?;
                self.not_late(value.header.frame_id)?;
                ensure!(
                    self.frames
                        .entry(value.header.frame_id)
                        .or_default()
                        .camera_view
                        .replace(value)
                        .is_none(),
                    "duplicate camera-view frame"
                );
            }
            "/events" => {
                let event: GameEvent = record.decode_wire()?;
                check_header(record, event.header)?;
                self.not_late(event.header.frame_id)?;
                ensure!(
                    self.events
                        .insert(
                            (event.header.frame_id, event.header.qpc, event.header.seq),
                            event
                        )
                        .is_none(),
                    "duplicate gameplay event"
                );
            }
            "/video_index" => {
                let index: VideoIndex = record.decode_wire()?;
                ensure!(
                    index.physics_frame == record.frame_id
                        && index.capture_qpc == record.qpc
                        && index.sim_time_us == record.sim_time_us,
                    "video index envelope mismatch"
                );
                ensure!(
                    self.last_video.is_none_or(|last| index.video_frame > last),
                    "duplicate/nonmonotonic video index"
                );
                self.last_video = Some(index.video_frame);
                if index.physics_frame == 0 {
                    self.report.video_without_telemetry += 1;
                    self.unassociated_video.insert(index.video_frame, index);
                } else {
                    self.not_late(index.physics_frame)?;
                    self.frames
                        .entry(index.physics_frame)
                        .or_default()
                        .video
                        .insert(
                            index.video_frame,
                            RecordedVideo {
                                index,
                                alignment: None,
                            },
                        );
                }
                self.report.video_indices += 1;
            }
            "/video_alignment" => {
                let alignment: VideoAlignment = record.decode_wire()?;
                ensure!(
                    alignment.capture_qpc == record.qpc,
                    "video alignment envelope mismatch"
                );
                if record.frame_id != 0 {
                    self.not_late(record.frame_id)?;
                }
                ensure!(
                    alignment.status <= VIDEO_ALIGNMENT_NO_TELEMETRY,
                    "unknown video alignment status"
                );
                ensure!(
                    self.video_alignment
                        .insert(
                            alignment.video_frame,
                            (record.frame_id, record.sim_time_us, alignment)
                        )
                        .is_none(),
                    "duplicate video alignment"
                );
            }
            _ => {}
        }
        ensure!(
            self.events.len() <= MAX_ASSOCIATIONS
                && self.full_configs.len() <= MAX_ASSOCIATIONS
                && self.control_configurations.len() <= MAX_ASSOCIATIONS
                && self.configs.len() <= LOOKAHEAD_FRAMES + 1
                && self.video_alignment.len() <= MAX_ASSOCIATIONS
                && self.unassociated_video.len()
                    + self
                        .frames
                        .values()
                        .map(|frame| frame.video.len())
                        .sum::<usize>()
                    <= MAX_ASSOCIATIONS,
            "replay association budget exceeded"
        );
        ensure!(
            self.frames.len() <= LOOKAHEAD_FRAMES * 2,
            "too many orphan physical frames"
        );
        self.report.maximum_buffered_frames =
            self.report.maximum_buffered_frames.max(self.frames.len());
        while self.frames.len() > LOOKAHEAD_FRAMES {
            self.emit_first(receive)?;
        }
        Ok(())
    }

    pub fn finish(
        &mut self,
        receive: &mut impl FnMut(ReplayFrame) -> anyhow::Result<()>,
    ) -> anyhow::Result<()> {
        while !self.frames.is_empty() {
            self.emit_first(receive)?;
        }
        self.report.tail_events += self.events.len() as u64;
        self.events.clear();
        // Alignment for an explicitly unassociated (physics_frame=0) image is
        // retained in the raw recording, not attached to an invented snapshot.
        for (video_frame, index) in &self.unassociated_video {
            if let Some((frame, time, alignment)) = self.video_alignment.remove(video_frame) {
                check_video(index, frame, time, alignment)?;
            } else {
                self.report.video_without_alignment += 1;
            }
        }
        ensure!(
            self.video_alignment.is_empty(),
            "video alignment has no corresponding index"
        );
        self.unassociated_video.clear();
        Ok(())
    }

    fn emit_first(
        &mut self,
        receive: &mut impl FnMut(ReplayFrame) -> anyhow::Result<()>,
    ) -> anyhow::Result<()> {
        let (frame_id, mut pending) = self.frames.pop_first().context("empty replay queue")?;
        let Some(raw) = pending.raw else {
            self.report.orphan_extensions += u64::from(pending.extension.is_some());
            self.report.orphan_sidecars +=
                u64::from(pending.dynamics.is_some()) + u64::from(pending.camera_view.is_some());
            self.report.video_without_telemetry += pending.video.len() as u64;
            for video_frame in pending.video.keys() {
                self.video_alignment.remove(video_frame);
            }
            return Ok(());
        };
        while self
            .configs
            .first_key_value()
            .is_some_and(|(frame, (obs, _))| *frame <= frame_id && obs.qpc <= raw.header.qpc)
        {
            self.config = self.configs.pop_first().map(|(_, value)| value);
        }
        while self
            .full_configs
            .first_key_value()
            .is_some_and(|((frame, qpc, _), _)| *frame <= frame_id && *qpc <= raw.header.qpc)
        {
            self.full_config = self.full_configs.pop_first().map(|(_, value)| value);
        }
        // Controls mode follows its own SDK event boundary and generation.
        while self
            .control_configurations
            .first_key_value()
            .is_some_and(|((frame, qpc, _), _)| *frame <= frame_id && *qpc <= raw.header.qpc)
        {
            self.control_configuration = self
                .control_configurations
                .pop_first()
                .map(|(_, value)| value);
        }
        // A legacy sample can observe the newest configuration while draining
        // older telemetry. Its larger sequence must never backdate an exact
        // SDK event. If that newer event is absent entirely, state is unknown.
        let configuration = self
            .full_config
            .as_ref()
            .filter(|full| {
                self.config.is_none_or(|(_, legacy)| {
                    legacy.config_seq <= full.config_seq
                        || self
                            .full_configs
                            .values()
                            .any(|future| future.config_seq == legacy.config_seq)
                })
            })
            .cloned();
        let config = if self.has_precise_configuration_stream {
            configuration.as_ref().map(|full| full.truck)
        } else {
            self.config.map(|(_, value)| value)
        };
        if let Some(extension) = &pending.extension {
            ensure!(
                extension.header.frame_id == raw.header.frame_id
                    && extension.header.sim_time_us == raw.header.sim_time_us
                    && extension.header.qpc == raw.header.qpc,
                "extension belongs to another physical timestamp"
            );
        }
        let mut snapshot = snapshot_from_recorded_frame(raw, pending.extension.take(), config)?;
        for header in pending
            .dynamics
            .as_ref()
            .map(|v| v.header)
            .into_iter()
            .chain(pending.camera_view.as_ref().map(|v| v.header))
        {
            ensure!(
                header.frame_id == raw.header.frame_id
                    && header.sim_time_us == raw.header.sim_time_us
                    && header.qpc == raw.header.qpc,
                "sidecar belongs to another physical timestamp"
            );
        }
        snapshot.session_start_qpc = self.info.map_or(0, |info| info.session_start_qpc);
        let ready_events: Vec<_> = self
            .events
            .iter()
            .filter(|((frame, qpc, _), _)| *frame <= frame_id && *qpc <= raw.header.qpc)
            .map(|(key, _)| *key)
            .collect();
        for key in ready_events {
            snapshot.events.push(self.events.remove(&key).unwrap());
        }
        if self.origin_generation == 0
            || (snapshot.ego.pose.x - self.origin[0]).hypot(snapshot.ego.pose.y - self.origin[1])
                > 2000.0
        {
            self.origin = snapshot.local_origin;
            self.origin_generation += 1;
        }
        snapshot.local_origin = self.origin;
        snapshot.origin_generation = self.origin_generation;
        for (&video_frame, video) in &mut pending.video {
            ensure!(
                video.index.physics_qpc == raw.header.qpc
                    && video.index.sim_time_us == raw.header.sim_time_us,
                "video index points at a different physical timestamp"
            );
            if let Some((frame, time, alignment)) = self.video_alignment.remove(&video_frame) {
                check_video(&video.index, frame, time, alignment)?;
                video.alignment = Some(alignment);
            } else {
                self.report.video_without_alignment += 1;
            }
        }
        self.report.snapshots += 1;
        self.report.control_configuration_frames += u64::from(self.control_configuration.is_some());
        self.report.unknown_control_configuration_frames +=
            u64::from(self.control_configuration.is_none_or(|c| c.valid == 0));
        self.report.events_delivered += snapshot.events.len() as u64;
        self.report.extension_frames += u64::from(snapshot.extension.is_some());
        self.report.missing_extension_frames += u64::from(snapshot.extension.is_none());
        self.report.missing_config_frames += u64::from(snapshot.config.is_none());
        self.report.precise_configuration_frames += u64::from(configuration.is_some());
        self.report.dynamics_frames += u64::from(pending.dynamics.is_some());
        self.report.camera_view_frames += u64::from(pending.camera_view.is_some());
        self.last_emitted = Some(frame_id);
        receive(ReplayFrame {
            control_configuration: self.control_configuration,
            snapshot,
            config_observation: self.config.map(|(obs, _)| obs),
            configuration,
            dynamics: pending.dynamics,
            camera_view: pending.camera_view,
            video: pending.video.into_values().collect(),
        })
    }
}

fn check_header(record: &Record, header: WireHeader) -> anyhow::Result<()> {
    ensure!(
        record.frame_id == header.frame_id
            && record.sim_time_us == header.sim_time_us
            && record.qpc == header.qpc,
        "record envelope disagrees with raw header"
    );
    Ok(())
}

fn check_video(
    index: &VideoIndex,
    frame: u64,
    time: u64,
    alignment: VideoAlignment,
) -> anyhow::Result<()> {
    ensure!(
        frame == index.physics_frame
            && time == index.sim_time_us
            && alignment.video_frame == index.video_frame
            && alignment.capture_frame == index.capture_frame
            && alignment.capture_qpc == index.capture_qpc,
        "video alignment belongs to another physical frame/capture"
    );
    Ok(())
}
