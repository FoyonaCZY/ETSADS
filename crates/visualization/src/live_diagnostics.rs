//! Pure association helpers shared by the live viewer and deterministic tests.
//! Capture exposure QPC, not delayed publication QPC, selects historical state.
use etsads_game::{adapter::GameSnapshot, capture::Frame, ets2la};
use etsads_msgs::protocol::{AppliedControl, ControlCmd, WireHeader};
use serde::Serialize;
use std::collections::VecDeque;

struct CameraEntry {
    header: WireHeader,
    camera: Option<ets2la::Camera>,
    paused: bool,
    snapshot: GameSnapshot,
}
#[derive(Default)]
pub struct CameraHistory {
    entries: VecDeque<CameraEntry>,
    session: Option<u64>,
    frequency: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageMatchStatus {
    Aligned,
    MissingPast,
    MissingFuture,
    MissingCamera,
    InvalidCamera,
    OutsideTolerance,
    Paused,
    ClockMismatch,
}
pub struct ImageMatch {
    pub status: ImageMatchStatus,
    pub header: Option<WireHeader>,
    pub camera: Option<ets2la::Camera>,
    pub delta_ms: Option<f64>,
}
impl CameraHistory {
    pub fn clear(&mut self) {
        self.entries.clear();
    }
    pub fn observe(&mut self, snapshot: &GameSnapshot) {
        if self.session != Some(snapshot.session_start_qpc)
            || self.frequency != snapshot.raw.qpc_frequency
            || self.entries.back().is_some_and(|last| {
                snapshot.raw.header.frame_id <= last.header.frame_id
                    || snapshot.raw.header.qpc <= last.header.qpc
            })
        {
            self.clear();
        }
        self.session = Some(snapshot.session_start_qpc);
        self.frequency = snapshot.raw.qpc_frequency;
        let camera = snapshot
            .extension
            .as_ref()
            .filter(|ext| ext.available & 1 != 0)
            .map(|ext| bytemuck::pod_read_unaligned::<ets2la::Camera>(&ext.camera));
        self.entries.push_back(CameraEntry {
            header: snapshot.raw.header,
            camera,
            paused: snapshot.ego.paused,
            snapshot: snapshot.clone(),
        });
        if self.entries.len() > 64 {
            self.entries.pop_front();
        }
    }
    pub fn snapshot(&self, frame_id: u64) -> Option<&GameSnapshot> {
        self.entries
            .iter()
            .find(|entry| entry.header.frame_id == frame_id)
            .map(|entry| &entry.snapshot)
    }
    pub fn associate(&self, exposure_qpc: u64, frequency: u64) -> ImageMatch {
        let missing = |status| ImageMatch {
            status,
            header: None,
            camera: None,
            delta_ms: None,
        };
        if frequency == 0 || frequency != self.frequency {
            return missing(ImageMatchStatus::ClockMismatch);
        }
        let Some(past) = self
            .entries
            .iter()
            .rev()
            .find(|entry| entry.header.qpc <= exposure_qpc)
        else {
            return missing(ImageMatchStatus::MissingPast);
        };
        let Some(future) = self
            .entries
            .iter()
            .find(|entry| entry.header.qpc >= exposure_qpc)
        else {
            return missing(ImageMatchStatus::MissingFuture);
        };
        let selected =
            if exposure_qpc.abs_diff(past.header.qpc) <= exposure_qpc.abs_diff(future.header.qpc) {
                past
            } else {
                future
            };
        let delta_ms =
            (selected.header.qpc as i128 - exposure_qpc as i128) as f64 * 1000.0 / frequency as f64;
        let status = if selected.paused {
            ImageMatchStatus::Paused
        } else if delta_ms.abs() > 16.0 {
            ImageMatchStatus::OutsideTolerance
        } else if let Some(camera) = selected.camera {
            if camera
                .position
                .into_iter()
                .chain(camera.projection)
                .all(f32::is_finite)
                && ets2la::valid_quaternion(camera.quaternion_wxyz)
            {
                ImageMatchStatus::Aligned
            } else {
                ImageMatchStatus::InvalidCamera
            }
        } else {
            ImageMatchStatus::MissingCamera
        };
        ImageMatch {
            status,
            header: Some(selected.header),
            camera: if status == ImageMatchStatus::Aligned {
                selected.camera
            } else {
                None
            },
            delta_ms: Some(delta_ms),
        }
    }
}

/// Strip row padding and convert supported DXGI RGBA/BGRA/BGRX to RGBA.
pub fn rgba_pixels(frame: &Frame) -> Result<Vec<u8>, &'static str> {
    let row = (frame.width as usize)
        .checked_mul(4)
        .ok_or("image width overflow")?;
    let total = row
        .checked_mul(frame.height as usize)
        .ok_or("image size overflow")?;
    if row == 0
        || total == 0
        || total > 64 * 1024 * 1024
        || (frame.stride as usize) < row
        || (frame.stride as usize)
            .checked_mul(frame.height as usize)
            .is_none_or(|size| size > frame.data.len())
        || !matches!(frame.format, 27 | 28 | 29 | 87 | 88 | 90 | 91 | 92 | 93)
    {
        return Err("unsupported/invalid live image layout");
    }
    let bgra = matches!(frame.format, 87 | 88 | 90 | 91 | 92 | 93);
    let opaque = matches!(frame.format, 88 | 92 | 93);
    let mut rgba = Vec::with_capacity(total);
    for line in frame
        .data
        .chunks_exact(frame.stride as usize)
        .take(frame.height as usize)
    {
        for pixel in line[..row].as_chunks::<4>().0 {
            rgba.extend_from_slice(&if bgra {
                [
                    pixel[2],
                    pixel[1],
                    pixel[0],
                    if opaque { 255 } else { pixel[3] },
                ]
            } else {
                [pixel[0], pixel[1], pixel[2], pixel[3]]
            });
        }
    }
    Ok(rgba)
}

#[derive(Clone, Debug, Serialize)]
pub struct CommandDelay {
    pub sequence: u64,
    pub applied_qpc: u64,
    pub status: &'static str,
    pub delay_ms: Option<f64>,
}
#[derive(Default)]
pub struct CommandDelays {
    commands: VecDeque<ControlCmd>,
    last_applied: Option<(u64, u64, &'static str)>,
}
impl CommandDelays {
    pub fn command(&mut self, command: ControlCmd) {
        if self
            .commands
            .back()
            .is_some_and(|old| old.seq == command.seq && old.write_qpc == command.write_qpc)
        {
            return;
        }
        if self
            .commands
            .back()
            .is_some_and(|old| command.seq <= old.seq || command.write_qpc < old.write_qpc)
        {
            self.commands.clear();
            self.last_applied = None;
        }
        self.commands.push_back(command);
        if self.commands.len() > 512 {
            self.commands.pop_front();
        }
    }
    pub fn applied(&mut self, applied: AppliedControl, frequency: u64) -> Option<CommandDelay> {
        let command = self
            .commands
            .iter()
            .rev()
            .find(|command| command.seq == applied.applied_seq);
        let status = if applied.reason != 0 {
            "released_or_rejected"
        } else if frequency == 0 || applied.applied_qpc == 0 {
            "invalid_clock"
        } else if let Some(command) = command {
            if command.write_qpc > applied.applied_qpc {
                "noncausal_clock"
            } else if command.write_qpc == 0 {
                "invalid_clock"
            } else {
                "matched_sdk_application"
            }
        } else {
            "missing_command"
        };
        let identity = (
            applied.applied_seq,
            command.map_or(0, |value| value.write_qpc),
            status,
        );
        if self.last_applied == Some(identity) {
            return None;
        }
        self.last_applied = Some(identity);
        Some(CommandDelay {
            sequence: applied.applied_seq,
            applied_qpc: applied.applied_qpc,
            status,
            delay_ms: if status == "matched_sdk_application" {
                Some(
                    (applied.applied_qpc - command.unwrap().write_qpc) as f64 * 1000.0
                        / frequency as f64,
                )
            } else {
                None
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use etsads_game::adapter::snapshot_from_recorded_frame;
    use etsads_msgs::protocol::{ExtensionFrame, TelemetryFrame};
    fn snapshot(frame: u64, qpc: u64, x: f32) -> GameSnapshot {
        let raw = TelemetryFrame {
            header: WireHeader {
                frame_id: frame,
                qpc,
                sim_time_us: frame * 16_667,
                ..Default::default()
            },
            qpc_frequency: 1000,
            ..Default::default()
        };
        let camera = ets2la::Camera {
            fov: 80.0,
            position: [x, 0.0, 0.0],
            sector: [0, 0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            projection: [0.0; 16],
            truck_position: [0.0; 3],
            truck_quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let mut ext = Box::<ExtensionFrame>::default();
        ext.header = raw.header;
        ext.available = 1;
        ext.camera.copy_from_slice(bytemuck::bytes_of(&camera));
        snapshot_from_recorded_frame(raw, Some(ext), None).unwrap()
    }
    #[test]
    fn staged_capture_uses_exposure_history_and_never_newest_pose() {
        let mut history = CameraHistory::default();
        for frame in 1..=6 {
            history.observe(&snapshot(frame, frame * 20, frame as f32));
        }
        let matched = history.associate(41, 1000); // Published later, at ~120.
        assert_eq!(matched.status, ImageMatchStatus::Aligned);
        assert_eq!(matched.header.unwrap().frame_id, 2);
        let camera_position = matched.camera.unwrap().position;
        assert_eq!(camera_position[0], 2.0);
        assert_eq!(
            history.associate(121, 1000).status,
            ImageMatchStatus::MissingFuture
        );
        assert_eq!(
            history.associate(19, 1000).status,
            ImageMatchStatus::MissingPast
        );
        assert_eq!(
            history.associate(41, 2000).status,
            ImageMatchStatus::ClockMismatch
        );
        let mut next = snapshot(7, 140, 7.0);
        next.session_start_qpc = 99;
        history.observe(&next);
        assert_eq!(
            history.associate(41, 1000).status,
            ImageMatchStatus::MissingPast
        );
    }
    #[test]
    fn camera_missing_or_paused_does_not_reuse_a_nearby_valid_camera() {
        let mut history = CameraHistory::default();
        history.observe(&snapshot(1, 20, 1.0));
        let mut missing = snapshot(2, 40, 2.0);
        missing.extension = None;
        history.observe(&missing);
        history.observe(&snapshot(3, 60, 3.0));
        assert_eq!(
            history.associate(41, 1000).status,
            ImageMatchStatus::MissingCamera
        );
        let mut paused = snapshot(4, 80, 4.0);
        paused.ego.paused = true;
        history.observe(&paused);
        assert_eq!(history.associate(80, 1000).status, ImageMatchStatus::Paused);
        history.clear();
        assert_eq!(
            history.associate(80, 1000).status,
            ImageMatchStatus::MissingPast
        );
    }
    #[test]
    fn padded_bgra_and_opaque_bgrx_preserve_pixel_colors() {
        let mut frame = Frame {
            width: 1,
            height: 2,
            stride: 8,
            format: 87,
            frame_index: 1,
            timestamp_qpc: 1,
            publish_qpc: 3,
            data: vec![1, 2, 3, 4, 9, 9, 9, 9, 5, 6, 7, 8, 9, 9, 9, 9],
        };
        assert_eq!(rgba_pixels(&frame).unwrap(), vec![3, 2, 1, 4, 7, 6, 5, 8]);
        frame.format = 88;
        assert_eq!(
            rgba_pixels(&frame).unwrap(),
            vec![3, 2, 1, 255, 7, 6, 5, 255]
        );
        frame.stride = 3;
        assert!(rgba_pixels(&frame).is_err());
    }
    #[test]
    fn command_delay_requires_matching_causal_sequence_and_distinguishes_missing() {
        let mut delays = CommandDelays::default();
        let applied = AppliedControl {
            applied_seq: 7,
            applied_qpc: 150,
            ..Default::default()
        };
        assert_eq!(
            delays.applied(applied, 1000).unwrap().status,
            "missing_command"
        );
        delays.command(ControlCmd {
            seq: 7,
            write_qpc: 100,
            ..Default::default()
        });
        let mut applied = applied;
        applied.applied_qpc = 151;
        assert_eq!(delays.applied(applied, 1000).unwrap().delay_ms, Some(51.0));
        assert!(delays.applied(applied, 1000).is_none());
        applied.applied_qpc = 170;
        assert!(
            delays.applied(applied, 1000).is_none(),
            "held command age is not another latency sample"
        );
        applied.applied_qpc = 152;
        applied.reason = 1;
        assert!(delays.applied(applied, 1000).unwrap().delay_ms.is_none());
        delays.command(ControlCmd {
            seq: 1,
            write_qpc: 200,
            ..Default::default()
        });
        applied.applied_seq = 1;
        applied.applied_qpc = 190;
        applied.reason = 0;
        assert_eq!(
            delays.applied(applied, 1000).unwrap().status,
            "noncausal_clock"
        );
    }

    #[cfg(windows)]
    #[test]
    fn isolated_capture_ipc_retains_exposure_time_pixels_and_resize() {
        use etsads_game::capture::CaptureReader;
        use etsads_gfx::publisher::CapturePublisher;
        let unique = format!(
            "ETSADS_ViewerFixture_{}_{}",
            std::process::id(),
            etsads_shm::qpc()
        );
        let control = format!("{unique}_Control");
        let data = format!("{unique}_Data_");
        let mut publisher =
            CapturePublisher::create_named(&control, &data, &format!("{unique}_Writer")).unwrap();
        let mut reader = CaptureReader::connect_to(&control, &data).unwrap();
        let frequency = etsads_shm::qpc_frequency();
        let now = etsads_shm::qpc();
        let old = now - frequency / 10;
        let exposure = old + frequency / 1000;
        let mut history = CameraHistory::default();
        for (n, qpc) in [(1, old), (2, now - frequency / 20)] {
            let mut state = snapshot(n, qpc, n as f32);
            state.raw.qpc_frequency = frequency;
            history.observe(&state);
        }
        publisher
            .publish(2, 1, 87, 8, &[1, 2, 3, 255, 4, 5, 6, 255], exposure)
            .unwrap();
        let frame = reader.poll().unwrap().unwrap();
        assert_eq!(frame.timestamp_qpc, exposure);
        assert!(frame.publish_qpc > frame.timestamp_qpc);
        assert_eq!(
            rgba_pixels(&frame).unwrap(),
            vec![3, 2, 1, 255, 6, 5, 4, 255]
        );
        let matched = history.associate(frame.timestamp_qpc, reader.qpc_frequency);
        assert_eq!(matched.status, ImageMatchStatus::Aligned);
        assert_eq!(matched.header.unwrap().frame_id, 1);
        publisher
            .publish(3, 1, 87, 12, &[0; 12], etsads_shm::qpc())
            .unwrap();
        let resized = reader.poll().unwrap().unwrap();
        assert_eq!(resized.width, 3);
        history.clear();
        assert_eq!(
            history.associate(resized.timestamp_qpc, frequency).status,
            ImageMatchStatus::MissingPast
        );
    }
}
