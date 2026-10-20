//! Bounded loss observations. Ring indices are not physical frame identifiers,
//! and the observation clock is not the timestamp of an overwritten sample.
use anyhow::ensure;
use serde::{Deserialize, Serialize};

pub const LOSS_TOPIC: &str = "/recording_loss";
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LossPhase {
    FirstDrain,
    Running,
    VideoTail,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RingLoss {
    pub topic: String,
    pub count: u64,
    pub initial_last_read: u64,
    pub initial_latest: u64,
    pub last_read: u64,
    pub earliest_available: u64,
    pub latest: u64,
    pub capacity: u64,
    pub cursor_initialized_qpc: u64,
    pub observed_qpc: u64,
    pub qpc_frequency: u64,
    pub session_start_qpc: Option<u64>,
    pub phase: LossPhase,
    /// Retains the old aggregate's coverage. Some streams previously had only
    /// separate counters (extension), or no aggregate accounting (events).
    pub counted_in_dropped_frames: bool,
}
impl RingLoss {
    pub(crate) fn validate(&self) -> anyhow::Result<()> {
        ensure!(
            self.topic.starts_with('/') && self.topic.len() <= 128,
            "invalid loss topic"
        );
        ensure!(
            self.capacity > 0 && self.capacity <= 65536 && self.latest > 0,
            "invalid loss ring capacity/index"
        );
        ensure!(
            self.initial_last_read <= self.initial_latest
                && self.initial_latest <= self.latest
                && self.initial_last_read <= self.last_read
                && self
                    .last_read
                    .checked_add(1)
                    .is_some_and(|next| next < self.earliest_available)
                && self.earliest_available == self.latest.saturating_sub(self.capacity - 1).max(1)
                && self.count == self.earliest_available - self.last_read - 1,
            "loss count or cursor range is inconsistent"
        );
        ensure!(
            self.cursor_initialized_qpc > 0
                && self.observed_qpc >= self.cursor_initialized_qpc
                && self.qpc_frequency > 0,
            "invalid loss observation clock"
        );
        ensure!(
            self.session_start_qpc
                .is_none_or(|s| s > 0 && s <= self.observed_qpc),
            "invalid loss session"
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LOSS_TOPIC, Recorder, RunMetadata};
    #[test]
    fn bounded_observations_keep_legacy_total_and_do_not_invent_sample_timestamps() {
        let dir = std::env::temp_dir().join(format!(
            "etsads-loss-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer =
            Recorder::create(&dir, RunMetadata::new("synthetic", "loss diagnostics")).unwrap();
        writer.add_dropped(7); // Existing callers and old files retain this aggregate API.
        let mut loss = RingLoss {
            topic: "/telemetry".into(),
            count: 2,
            initial_last_read: 100,
            initial_latest: 100,
            last_read: 100,
            earliest_available: 103,
            latest: 166,
            capacity: 64,
            cursor_initialized_qpc: 10,
            observed_qpc: 20,
            qpc_frequency: 1000,
            session_start_qpc: None,
            phase: LossPhase::Running,
            counted_in_dropped_frames: true,
        };
        writer.record_ring_loss(&loss).unwrap();
        loss.topic = "/extension".into();
        loss.counted_in_dropped_frames = false;
        writer.record_ring_loss(&loss).unwrap();
        let mut invalid = loss.clone();
        invalid.topic = "/".repeat(129);
        assert!(writer.record_ring_loss(&invalid).is_err());
        invalid = loss;
        invalid.count += 1;
        assert!(writer.record_ring_loss(&invalid).is_err());
        let meta = writer.finish().unwrap();
        assert_eq!(meta.dropped_frames, 9);
        assert_eq!(meta.messages, 2);
        let mut count = 0;
        crate::replay(&dir, |topic, record| {
            assert_eq!(topic, LOSS_TOPIC);
            assert_eq!(
                (record.frame_id, record.sim_time_us, record.qpc),
                (0, 0, 20)
            );
            let event: serde_json::Value = serde_json::from_slice(&record.raw_payload)?;
            assert!(record.raw_payload.len() < 4096);
            assert!(
                event["lost_physical_frame_range"].is_null()
                    && event["lost_sample_qpc_range"].is_null()
            );
            assert_eq!(
                event["lost_ring_index_range"],
                serde_json::json!([101, 102])
            );
            count += 1;
            Ok(())
        })
        .unwrap();
        assert_eq!(count, 2);
        let raw: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dir.join("meta.json")).unwrap()).unwrap();
        assert!(
            raw.get("losses").is_none(),
            "loss history streams to MCAP, not unbounded metadata"
        );
    }
}
