//! Constant-space per-stream cursor diagnostics. First-drain loss is distinct
//! from an overwrite after recording has begun or during bounded video tail.
use etsads_recorder::{LossPhase, Recorder, RingLoss};
use etsads_shm::{qpc, qpc_frequency};

pub(crate) struct LossObserver {
    topic: &'static str,
    capacity: u64,
    initial_last_read: u64,
    initial_latest: u64,
    initialized_qpc: u64,
    first_drain: bool,
}
impl LossObserver {
    pub fn new(
        topic: &'static str,
        capacity: u64,
        initial_last_read: u64,
        initial_latest: u64,
    ) -> Self {
        assert!((1..=65536).contains(&capacity));
        Self {
            topic,
            capacity,
            initial_last_read,
            initial_latest,
            initialized_qpc: qpc(),
            first_drain: true,
        }
    }
    /// Returns the exact missed slot count for this cursor snapshot. Caller
    /// advances its existing cursor only after recording this observation.
    pub fn observe(
        &mut self,
        recorder: &mut Recorder,
        last_read: u64,
        latest: u64,
        tail: bool,
        counted_in_dropped_frames: bool,
        session_start_qpc: Option<u64>,
    ) -> anyhow::Result<u64> {
        let phase = if tail {
            LossPhase::VideoTail
        } else if self.first_drain {
            LossPhase::FirstDrain
        } else {
            LossPhase::Running
        };
        self.first_drain = false;
        let earliest = latest.saturating_sub(self.capacity - 1).max(1);
        let count = earliest.saturating_sub(last_read.saturating_add(1));
        if count > 0 {
            recorder.record_ring_loss(&RingLoss {
                topic: self.topic.into(),
                count,
                initial_last_read: self.initial_last_read,
                initial_latest: self.initial_latest,
                last_read,
                earliest_available: earliest,
                latest,
                capacity: self.capacity,
                cursor_initialized_qpc: self.initialized_qpc,
                observed_qpc: qpc(),
                qpc_frequency: qpc_frequency(),
                phase,
                counted_in_dropped_frames,
                session_start_qpc,
            })?;
        }
        Ok(count)
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    use etsads_msgs::protocol::*;
    use etsads_recorder::{LOSS_TOPIC, Record, RunMetadata};
    use etsads_shm::SharedRing;
    fn directory(label: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!(
                "record-loss-{label}-{}-{}",
                std::process::id(),
                qpc()
            ))
    }
    fn losses(path: &std::path::Path) -> Vec<serde_json::Value> {
        let mut result = Vec::new();
        etsads_recorder::replay(path, |topic, record| {
            if topic == LOSS_TOPIC {
                result.push(serde_json::from_slice(&record.raw_payload)?);
            }
            Ok(())
        })
        .unwrap();
        result
    }
    #[test]
    fn timing_actions_and_events_overruns_keep_topic_and_legacy_accounting() {
        let path = directory("topics");
        let mut recorder = Recorder::create(
            &path,
            RunMetadata::new("synthetic", "source cursor diagnostics"),
        )
        .unwrap();
        for (topic, aggregate) in [
            ("/frame_timing", true),
            ("/input_actions", true),
            ("/events", false),
        ] {
            let mut observer = LossObserver::new(topic, 256, 100, 100);
            assert_eq!(
                observer
                    .observe(&mut recorder, 100, 100, false, aggregate, None)
                    .unwrap(),
                0
            );
            assert_eq!(
                observer
                    .observe(&mut recorder, 100, 400, false, aggregate, None)
                    .unwrap(),
                44
            );
            assert_eq!(
                observer
                    .observe(&mut recorder, 400, 400, false, aggregate, None)
                    .unwrap(),
                0
            );
        }
        let meta = recorder.finish().unwrap();
        assert_eq!(meta.dropped_frames, 88);
        let observations = losses(&path);
        assert_eq!(observations.len(), 3);
        for (i, topic) in ["/frame_timing", "/input_actions", "/events"]
            .into_iter()
            .enumerate()
        {
            let event = &observations[i];
            assert_eq!(event["observation"]["topic"], topic);
            assert_eq!(
                event["lost_ring_index_range"],
                serde_json::json!([101, 144])
            );
            assert_eq!(event["observation"]["phase"], "running");
            assert!(event["lost_sample_qpc_range"].is_null());
        }
        assert_eq!(
            observations[2]["observation"]["counted_in_dropped_frames"],
            false
        );
    }
    #[test]
    fn extension_overrun_has_a_stream_event_even_without_video_summary() {
        let path = directory("extension");
        let ring = SharedRing::<ExtensionFrame, 4>::create(
            &format!("ETSADS_LossExt_{}_{}", std::process::id(), qpc()),
            SCHEMA_VERSION,
        )
        .unwrap();
        let mut extension = Box::<ExtensionFrame>::default();
        for frame in 1..=4 {
            extension.header.frame_id = frame;
            extension.header.qpc = frame * 100;
            ring.publish(&extension).unwrap();
        }
        let mut drain = crate::ExtensionDrain::new(Some(&ring), None);
        for frame in 5..=6 {
            extension.header.frame_id = frame;
            extension.header.qpc = frame * 100;
            ring.publish(&extension).unwrap();
        }
        let mut recorder =
            Recorder::create(&path, RunMetadata::new("synthetic", "no video")).unwrap();
        let mut alignment = crate::alignment::VideoAligner::new(1000).unwrap();
        for frame in 3..=6 {
            let t = TelemetryFrame {
                header: WireHeader {
                    frame_id: frame,
                    qpc: frame * 100,
                    ..Default::default()
                },
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&t))
                .unwrap();
            alignment.note_telemetry(t);
        }
        drain
            .record_available(&mut recorder, Some(&ring), &mut alignment, false)
            .unwrap();
        drain
            .record_available(&mut recorder, Some(&ring), &mut alignment, false)
            .unwrap();
        assert_eq!(drain.overrun, 2);
        assert_eq!(recorder.finish().unwrap().dropped_frames, 0);
        let events = losses(&path);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0]["observation"]["topic"], "/extension");
        assert_eq!(
            events[0]["lost_ring_index_range"],
            serde_json::json!([1, 2])
        );
        assert_eq!(events[0]["observation"]["counted_in_dropped_frames"], false);
    }
    #[test]
    fn video_tail_overrun_is_located_before_incomplete_and_does_not_change_old_total() {
        let path = directory("tail");
        let ring = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
            &format!("ETSADS_LossTail_{}_{}", std::process::id(), qpc()),
            SCHEMA_VERSION,
        )
        .unwrap();
        let mut cursor = crate::TelemetryCursor::new(0, 0, None);
        for n in 1..=70 {
            ring.publish(&TelemetryFrame {
                header: WireHeader {
                    frame_id: n,
                    qpc: n * 100,
                    ..Default::default()
                },
                ..Default::default()
            })
            .unwrap();
        }
        let mut recorder =
            Recorder::create(&path, RunMetadata::new("synthetic", "tail overrun")).unwrap();
        let mut align = crate::alignment::VideoAligner::new(1000).unwrap();
        let mut extension = crate::ExtensionDrain::new(None, None);
        let error = crate::record_alignment_tail(
            &mut recorder,
            &ring,
            &mut cursor,
            &mut align,
            None,
            &mut extension,
            None,
        )
        .unwrap_err();
        assert!(error.to_string().contains("overrun"));
        // Cleanup retry can retain remaining data, without duplicating the loss.
        crate::record_alignment_tail(
            &mut recorder,
            &ring,
            &mut cursor,
            &mut align,
            None,
            &mut extension,
            None,
        )
        .unwrap();
        let meta = recorder.finish_incomplete(error.to_string()).unwrap();
        assert!(!meta.completed);
        assert_eq!(meta.dropped_frames, 0);
        let events = losses(&path);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0]["observation"]["phase"], "video_tail");
        assert_eq!(
            events[0]["lost_ring_index_range"],
            serde_json::json!([1, 6])
        );
    }
}
