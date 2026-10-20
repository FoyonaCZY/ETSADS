//! The encoder never waits for physics. The recorder holds only small indexes
//! until a recorded future physics sample exists, with a 40 ms hard bound.
use anyhow::ensure;
use etsads_msgs::protocol::*;
use etsads_recorder::{Record, Recorder};
use std::collections::{BTreeMap, VecDeque};

struct Pending {
    index: VideoIndex,
    received_qpc: u64,
}
pub struct VideoAligner {
    history: VecDeque<TelemetryFrame>,
    pending: VecDeque<Pending>,
    max_wait: u64,
    last_video: Option<u64>,
    camera_frames: VecDeque<u64>,
    camera_pending: BTreeMap<u64, u64>,
    camera_matched: u64,
    camera_missing: u64,
}
impl VideoAligner {
    pub fn new(frequency: u64) -> anyhow::Result<Self> {
        ensure!(frequency > 0, "invalid QPC frequency for alignment");
        Ok(Self {
            history: VecDeque::with_capacity(256),
            pending: VecDeque::with_capacity(256),
            max_wait: (frequency / 25).max(1),
            last_video: None,
            camera_frames: VecDeque::with_capacity(256),
            camera_pending: BTreeMap::new(),
            camera_matched: 0,
            camera_missing: 0,
        })
    }
    pub fn note_telemetry(&mut self, frame: TelemetryFrame) {
        if self.history.len() == 256 {
            self.history.pop_front();
        }
        self.history.push_back(frame);
        if let Some(oldest) = self.history.front() {
            while self
                .camera_pending
                .first_key_value()
                .is_some_and(|(id, _)| *id < oldest.header.frame_id)
            {
                self.camera_missing += self.camera_pending.pop_first().unwrap().1;
            }
        }
    }
    pub(crate) fn recorded_telemetry(&self, frame: u64) -> Option<&TelemetryFrame> {
        self.history.iter().find(|t| t.header.frame_id == frame)
    }
    pub(crate) fn latest_recorded_frame(&self) -> Option<u64> {
        self.history.back().map(|t| t.header.frame_id)
    }
    pub(crate) fn note_camera(&mut self, frame: u64) {
        if self.camera_frames.back() == Some(&frame) {
            return;
        }
        if self.camera_frames.len() == 256 {
            self.camera_frames.pop_front();
        }
        self.camera_frames.push_back(frame);
        self.camera_matched += self.camera_pending.remove(&frame).unwrap_or(0);
    }
    pub(crate) fn has_pending_camera(&self) -> bool {
        !self.camera_pending.is_empty()
    }
    pub(crate) fn finish_camera_summary(
        &mut self,
        connected: bool,
        overrun: u64,
        unmatched: u64,
    ) -> serde_json::Value {
        self.camera_missing += self.camera_pending.values().sum::<u64>();
        self.camera_pending.clear();
        serde_json::json!({"encoding":"json","source":"game","extension_mapping_connected":connected,"video_frames_with_recorded_camera":self.camera_matched,"video_frames_missing_recorded_camera":self.camera_missing,"extension_slots_overrun":overrun,"extension_frames_without_recorded_telemetry":unmatched,"scope":"raw same-physical-frame camera payload presence; not projection calibration"})
    }
    pub fn enqueue(&mut self, index: VideoIndex, now: u64) -> anyhow::Result<()> {
        ensure!(self.pending.len() < 256, "video alignment queue is full");
        ensure!(
            self.last_video.is_none_or(|last| index.video_frame > last),
            "video index regressed or repeated"
        );
        self.last_video = Some(index.video_frame);
        self.pending.push_back(Pending {
            index,
            received_qpc: now,
        });
        Ok(())
    }
    pub fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }
    fn ready(&mut self, now: u64, tail: bool) -> Vec<(VideoIndex, VideoAlignment)> {
        let mut ready = Vec::new();
        while let Some(front) = self.pending.front() {
            let capture = front.index.capture_qpc;
            let before = self
                .history
                .iter()
                .filter(|f| f.header.qpc <= capture)
                .max_by_key(|f| f.header.qpc);
            let after = self
                .history
                .iter()
                .filter(|f| f.header.qpc >= capture)
                .min_by_key(|f| f.header.qpc);
            let waited = now.saturating_sub(front.received_qpc);
            if after.is_none() && !tail && waited < self.max_wait {
                break;
            }
            let status = match (before, after) {
                (Some(_), Some(_)) => VIDEO_ALIGNMENT_BRACKETED,
                (None, Some(_)) => VIDEO_ALIGNMENT_MISSING_PAST,
                (Some(_), None) if tail => VIDEO_ALIGNMENT_TAIL,
                (Some(_), None) => VIDEO_ALIGNMENT_TIMEOUT,
                (None, None) => VIDEO_ALIGNMENT_NO_TELEMETRY,
            };
            let nearest = [before, after]
                .into_iter()
                .flatten()
                .min_by_key(|f| f.header.qpc.abs_diff(capture))
                .copied();
            let mut index = self.pending.pop_front().unwrap().index;
            if let Some(frame) = nearest {
                index.physics_frame = frame.header.frame_id;
                index.physics_qpc = frame.header.qpc;
                index.sim_time_us = frame.header.sim_time_us;
                index.alignment_error_qpc = frame.header.qpc.abs_diff(capture);
            } else {
                index.physics_frame = 0;
                index.physics_qpc = 0;
                index.alignment_error_qpc = u64::MAX;
            }
            if index.physics_frame == 0 {
                self.camera_missing += 1;
            } else if self.camera_frames.contains(&index.physics_frame) {
                self.camera_matched += 1;
            } else {
                *self.camera_pending.entry(index.physics_frame).or_default() += 1;
            }
            let evidence = VideoAlignment {
                video_frame: index.video_frame,
                capture_frame: index.capture_frame,
                capture_qpc: capture,
                recorded_qpc: now,
                wait_qpc: waited,
                status,
                reserved: 0,
            };
            ready.push((index, evidence));
        }
        ready
    }
    pub fn flush(
        &mut self,
        recorder: &mut Recorder,
        now: u64,
        tail: bool,
    ) -> anyhow::Result<usize> {
        let ready = self.ready(now, tail);
        for (index, evidence) in &ready {
            recorder.record(
                "/video_index",
                &Record::wire(
                    index,
                    index.physics_frame,
                    index.sim_time_us,
                    index.capture_qpc,
                ),
            )?;
            recorder.record(
                "/video_alignment",
                &Record::wire(
                    evidence,
                    index.physics_frame,
                    index.sim_time_us,
                    index.capture_qpc,
                ),
            )?;
        }
        Ok(ready.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(id: u64, qpc: u64) -> TelemetryFrame {
        TelemetryFrame {
            header: WireHeader {
                frame_id: id,
                qpc,
                sim_time_us: id * 16666,
                ..Default::default()
            },
            ..Default::default()
        }
    }
    #[test]
    fn waits_for_future_and_chooses_nearest_recorded_sample() {
        let mut a = VideoAligner::new(1000).unwrap();
        a.note_telemetry(frame(1, 100));
        a.enqueue(
            VideoIndex {
                capture_qpc: 112,
                ..Default::default()
            },
            115,
        )
        .unwrap();
        assert!(a.ready(139, false).is_empty());
        a.note_telemetry(frame(2, 120));
        let ready = a.ready(140, false);
        assert_eq!(ready[0].0.physics_frame, 2);
        assert_eq!(ready[0].0.alignment_error_qpc, 8);
        assert_eq!(ready[0].1.status, VIDEO_ALIGNMENT_BRACKETED);
    }
    #[test]
    fn timeout_and_tail_stay_explicit() {
        let mut a = VideoAligner::new(1000).unwrap();
        a.note_telemetry(frame(1, 100));
        a.enqueue(
            VideoIndex {
                capture_qpc: 112,
                ..Default::default()
            },
            115,
        )
        .unwrap();
        assert_eq!(a.ready(155, false)[0].1.status, VIDEO_ALIGNMENT_TIMEOUT);
        a.enqueue(
            VideoIndex {
                video_frame: 1,
                capture_qpc: 122,
                ..Default::default()
            },
            160,
        )
        .unwrap();
        assert_eq!(a.ready(161, true)[0].1.status, VIDEO_ALIGNMENT_TAIL);
    }
    #[test]
    fn bounded_queue_rejects_unrecorded_loss() {
        let mut a = VideoAligner::new(1000).unwrap();
        for i in 0..256 {
            a.enqueue(
                VideoIndex {
                    video_frame: i,
                    capture_qpc: 112,
                    ..Default::default()
                },
                115,
            )
            .unwrap();
        }
        assert!(
            a.enqueue(
                VideoIndex {
                    video_frame: 256,
                    ..Default::default()
                },
                115
            )
            .is_err()
        );
        let ready = a.ready(155, true);
        assert_eq!(ready.len(), 256);
        assert!(
            ready
                .iter()
                .all(|(_, e)| e.status == VIDEO_ALIGNMENT_NO_TELEMETRY)
        );
    }
}
