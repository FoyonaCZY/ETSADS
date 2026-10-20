//! Preserve optional new bridge streams without changing legacy MCAP channels.
use crate::alignment::VideoAligner;
use anyhow::ensure;
use etsads_msgs::protocol::*;
use etsads_recorder::{Record, Recorder};
use etsads_shm::SharedRing;

struct Stream<T: bytemuck::Pod> {
    ring: Option<SharedRing<T, 64>>,
    cursor: u64,
    buffer: Box<T>,
    topic: &'static str,
    header: fn(&T) -> WireHeader,
    physical: bool,
    loss: crate::recording_loss::LossObserver,
}
impl<T: bytemuck::Pod> Stream<T> {
    fn new(
        mapping: &str,
        topic: &'static str,
        header: fn(&T) -> WireHeader,
        physical: bool,
    ) -> Self {
        let ring = SharedRing::open(mapping, SCHEMA_VERSION).ok();
        Self::with_ring(ring, topic, header, physical)
    }
    fn with_ring(
        ring: Option<SharedRing<T, 64>>,
        topic: &'static str,
        header: fn(&T) -> WireHeader,
        physical: bool,
    ) -> Self {
        let latest = ring.as_ref().map_or(0, |r| r.latest_index());
        let cursor = latest.saturating_sub(64);
        Self {
            cursor,
            ring,
            buffer: etsads_shm::zeroed_box(),
            topic,
            header,
            physical,
            loss: crate::recording_loss::LossObserver::new(topic, 64, cursor, latest),
        }
    }
    fn drain(
        &mut self,
        recorder: &mut Recorder,
        alignment: &VideoAligner,
        mandatory: bool,
        session: u64,
        tail: bool,
    ) -> anyhow::Result<()> {
        let Some(ring) = &self.ring else {
            return Ok(());
        };
        let latest = ring.latest_index();
        let earliest = latest.saturating_sub(63).max(1);
        if self.loss.observe(
            recorder,
            self.cursor,
            latest,
            tail,
            true,
            (session > 0).then_some(session),
        )? > 0
        {
            self.cursor = earliest - 1;
            ensure!(!mandatory, "mandatory {} sidecar ring overrun", self.topic);
        }
        for index in self.cursor + 1..=latest {
            if !ring.read_into(index, &mut self.buffer) {
                break;
            }
            let header = (self.header)(&self.buffer);
            if header.qpc < session {
                self.cursor = index;
                continue;
            }
            if self.physical {
                if alignment
                    .latest_recorded_frame()
                    .is_none_or(|last| header.frame_id > last)
                {
                    break;
                }
                let Some(t) = alignment.recorded_telemetry(header.frame_id) else {
                    self.cursor = index;
                    continue;
                };
                ensure!(
                    header.qpc == t.header.qpc && header.sim_time_us == t.header.sim_time_us,
                    "{} does not match its recorded physical frame",
                    self.topic
                );
            }
            recorder.record(
                self.topic,
                &Record::wire(
                    self.buffer.as_ref(),
                    header.frame_id,
                    header.sim_time_us,
                    header.qpc,
                ),
            )?;
            self.cursor = index;
        }
        Ok(())
    }
}
pub(crate) struct SidecarRecording {
    controls: Stream<ControlConfiguration>,
    dynamics: Stream<DynamicsFrame>,
    configuration: Stream<ConfigurationFrame>,
    camera_view: Stream<CameraViewFrame>,
    mandatory: bool,
    session: u64,
}
impl SidecarRecording {
    pub fn connect(mandatory: bool, session: u64) -> Self {
        Self {
            controls: Stream::new(
                CONTROL_CONFIGURATION_MAPPING,
                "/control_configuration",
                |v| v.header,
                false,
            ),
            dynamics: Stream::new(DYNAMICS_MAPPING, "/dynamics", |v| v.header, true),
            configuration: Stream::new(CONFIG_FULL_MAPPING, "/config_full", |v| v.header, false),
            camera_view: Stream::new(CAMERA_VIEW_MAPPING, "/camera_view", |v| v.header, true),
            mandatory,
            session,
        }
    }
    pub fn drain(
        &mut self,
        recorder: &mut Recorder,
        alignment: &VideoAligner,
        tail: bool,
    ) -> anyhow::Result<()> {
        self.controls
            .drain(recorder, alignment, self.mandatory, self.session, tail)?;
        self.configuration
            .drain(recorder, alignment, self.mandatory, self.session, tail)?;
        self.dynamics
            .drain(recorder, alignment, self.mandatory, self.session, tail)?;
        self.camera_view
            .drain(recorder, alignment, self.mandatory, self.session, tail)
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    #[test]
    fn sidecar_startup_window_and_running_overrun_have_distinct_streamed_ranges() {
        let suffix = format!("{}_{}", std::process::id(), etsads_shm::qpc());
        let ring = SharedRing::<DynamicsFrame, 64>::create(
            &format!("ETSADS_LossSidecar_{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let frame = |n| DynamicsFrame {
            header: WireHeader {
                frame_id: n,
                sim_time_us: n * 1000,
                qpc: n * 100,
                ..Default::default()
            },
            ..Default::default()
        };
        for n in 1..=64 {
            ring.publish(&frame(n)).unwrap();
        }
        let mut stream = Stream::with_ring(Some(ring), "/dynamics", |d| d.header, true);
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!("loss-sidecar-{suffix}"));
        let mut recorder = Recorder::create(
            &path,
            etsads_recorder::RunMetadata::new(
                "synthetic",
                "startup loss is not an internal recorded gap",
            ),
        )
        .unwrap();
        let mut alignment = VideoAligner::new(1000).unwrap();
        for n in 33..=65 {
            let t = TelemetryFrame {
                header: frame(n).header,
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&t))
                .unwrap();
            alignment.note_telemetry(t);
        }
        stream.ring.as_ref().unwrap().publish(&frame(65)).unwrap();
        stream
            .drain(&mut recorder, &alignment, false, 0, false)
            .unwrap();
        stream
            .drain(&mut recorder, &alignment, false, 0, false)
            .unwrap(); // no repeat count
        // Future physical samples defer, rather than being called overwritten.
        stream.ring.as_ref().unwrap().publish(&frame(66)).unwrap();
        stream
            .drain(&mut recorder, &alignment, false, 0, false)
            .unwrap();
        assert_eq!(stream.cursor, 65);
        let t = TelemetryFrame {
            header: frame(66).header,
            ..Default::default()
        };
        recorder
            .record("/telemetry", &Record::telemetry(&t))
            .unwrap();
        alignment.note_telemetry(t);
        stream
            .drain(&mut recorder, &alignment, false, 0, false)
            .unwrap();
        for n in 67..=132 {
            stream.ring.as_ref().unwrap().publish(&frame(n)).unwrap();
            let t = TelemetryFrame {
                header: frame(n).header,
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&t))
                .unwrap();
            alignment.note_telemetry(t);
        }
        stream
            .drain(&mut recorder, &alignment, false, 0, false)
            .unwrap();
        let meta = recorder.finish().unwrap();
        assert_eq!(meta.dropped_frames, 3);
        let mut losses = Vec::new();
        let report = etsads_recorder::replay(&path, |topic, record| {
            if topic == etsads_recorder::LOSS_TOPIC {
                losses.push(serde_json::from_slice::<serde_json::Value>(
                    &record.raw_payload,
                )?);
            }
            Ok(())
        })
        .unwrap();
        assert_eq!(report.frame_gaps, 0);
        assert_eq!(losses.len(), 2);
        assert_eq!(losses[0]["observation"]["topic"], "/dynamics");
        assert_eq!(losses[0]["observation"]["phase"], "first_drain");
        assert_eq!(losses[0]["observation"]["initial_latest"], 64);
        assert_eq!(
            losses[0]["lost_ring_index_range"],
            serde_json::json!([1, 1])
        );
        assert_eq!(losses[1]["observation"]["phase"], "running");
        assert_eq!(
            losses[1]["lost_ring_index_range"],
            serde_json::json!([67, 68])
        );
    }
    #[test]
    fn mandatory_sidecar_loss_is_recorded_before_fail_closed_finalization() {
        let suffix = format!("{}_{}", std::process::id(), etsads_shm::qpc());
        let ring = SharedRing::<ControlConfiguration, 64>::create(
            &format!("ETSADS_LossMandatory_{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let mut stream =
            Stream::with_ring(Some(ring), "/control_configuration", |c| c.header, false);
        for _ in 0..65 {
            stream
                .ring
                .as_ref()
                .unwrap()
                .publish(&ControlConfiguration::default())
                .unwrap();
        }
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!("loss-mandatory-{suffix}"));
        let mut recorder = Recorder::create(
            &path,
            etsads_recorder::RunMetadata::new("synthetic", "mandatory overrun"),
        )
        .unwrap();
        let error = stream
            .drain(
                &mut recorder,
                &VideoAligner::new(1000).unwrap(),
                true,
                0,
                false,
            )
            .unwrap_err();
        assert!(error.to_string().contains("overrun"));
        let meta = recorder.finish_incomplete(error.to_string()).unwrap();
        assert!(!meta.completed);
        assert_eq!(meta.dropped_frames, 1);
        let report = etsads_recorder::replay(&path, |topic, record| {
            assert_eq!(topic, etsads_recorder::LOSS_TOPIC);
            let v: serde_json::Value = serde_json::from_slice(&record.raw_payload)?;
            assert_eq!(v["observation"]["count"], 1);
            Ok(())
        })
        .unwrap();
        assert_eq!(report.messages, 1);
    }
    #[test]
    fn sidecars_record_exact_frames_event_time_and_old_runs_stay_readable() {
        let suffix = format!("{}_{}", std::process::id(), etsads_shm::qpc());
        let dynamics = SharedRing::<DynamicsFrame, 64>::create(
            &format!("ETSADS_SidecarTest_D_{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let configuration = SharedRing::<ConfigurationFrame, 64>::create(
            &format!("ETSADS_SidecarTest_C_{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let views = SharedRing::<CameraViewFrame, 64>::create(
            &format!("ETSADS_SidecarTest_V_{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let mut recorder_streams = SidecarRecording {
            controls: Stream::with_ring(
                Some(
                    SharedRing::create(&format!("ETSADS_ControlSidecar_{suffix}"), SCHEMA_VERSION)
                        .unwrap(),
                ),
                "/control_configuration",
                |c| c.header,
                false,
            ),
            dynamics: Stream::with_ring(Some(dynamics), "/dynamics", |d| d.header, true),
            configuration: Stream::with_ring(
                Some(configuration),
                "/config_full",
                |c| c.header,
                false,
            ),
            camera_view: Stream::with_ring(Some(views), "/camera_view", |v| v.header, true),
            mandatory: true,
            session: 1,
        };
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!("sidecars-{suffix}"));
        let mut recorder = Recorder::create(
            &path,
            etsads_recorder::RunMetadata::new("synthetic", "sidecar fixture"),
        )
        .unwrap();
        let t = TelemetryFrame {
            header: WireHeader {
                frame_id: 2,
                qpc: 200,
                sim_time_us: 16667,
                ..Default::default()
            },
            qpc_frequency: 1000,
            ..Default::default()
        };
        recorder
            .record("/telemetry", &Record::telemetry(&t))
            .unwrap();
        let mut alignment = VideoAligner::new(1000).unwrap();
        alignment.note_telemetry(t);
        recorder_streams
            .controls
            .ring
            .as_ref()
            .unwrap()
            .publish(&ControlConfiguration {
                header: WireHeader {
                    frame_id: 1,
                    qpc: 150,
                    seq: 1,
                    ..Default::default()
                },
                session_start_qpc: 1,
                generation: 1,
                effective_from_frame_id: 2,
                shifter_type: SHIFTER_AUTOMATIC,
                valid: 1,
            })
            .unwrap();
        let config = ConfigurationFrame {
            header: WireHeader {
                frame_id: 1,
                qpc: 150,
                ..Default::default()
            },
            effective_from_frame_id: 2,
            config_seq: 9,
            truck: ConfigBlock {
                config_seq: 9,
                ..Default::default()
            },
            ..Default::default()
        };
        recorder_streams
            .configuration
            .ring
            .as_ref()
            .unwrap()
            .publish(&config)
            .unwrap();
        recorder_streams
            .dynamics
            .ring
            .as_ref()
            .unwrap()
            .publish(&DynamicsFrame {
                header: t.header,
                angular_acceleration_local: [0.0, 0.25, 0.0],
                valid: DYNAMICS_ANGULAR_ACCELERATION,
                ..Default::default()
            })
            .unwrap();
        recorder_streams
            .drain(&mut recorder, &alignment, false)
            .unwrap();
        recorder_streams
            .camera_view
            .ring
            .as_ref()
            .unwrap()
            .publish(&CameraViewFrame {
                header: t.header,
                session_start_qpc: 1,
                flags: CAMERA_VIEW_MISSING_CONFIG,
                selected_hypothesis: u32::MAX,
                ..Default::default()
            })
            .unwrap();
        recorder_streams
            .drain(&mut recorder, &alignment, false)
            .unwrap();
        recorder_streams
            .drain(&mut recorder, &alignment, false)
            .unwrap();
        recorder.finish().unwrap();
        let mut seen = std::collections::BTreeSet::new();
        let replay = etsads_recorder::replay(&path, |topic, record| {
            assert!(seen.insert(topic.to_owned()));
            match topic {
                "/config_full" => {
                    let value: ConfigurationFrame = record.decode_wire()?;
                    assert_eq!(value.header.qpc, 150);
                    assert_eq!(value.effective_from_frame_id, 2);
                }
                "/dynamics" => assert_eq!(record.decode_wire::<DynamicsFrame>()?.header.qpc, 200),
                "/camera_view" => {
                    assert_eq!(record.decode_wire::<CameraViewFrame>()?.header.frame_id, 2)
                }
                _ => {}
            }
            Ok(())
        })
        .unwrap();
        assert_eq!(replay.messages, 5);
        let joined = etsads_replay::ReplayAdapter::run(&path, |frame| {
            assert_eq!(frame.snapshot.raw.header.frame_id, 2);
            assert_eq!(frame.configuration.as_ref().unwrap().config_seq, 9);
            assert!(frame.control_configuration.unwrap().safe_shifter());
            assert_eq!(frame.control_configuration.unwrap().header.qpc, 150);
            assert_eq!(frame.dynamics.unwrap().angular_acceleration_local[1], 0.25);
            assert_eq!(frame.camera_view.unwrap().selected_hypothesis, u32::MAX);
            Ok(())
        })
        .unwrap();
        assert_eq!(joined.associations.snapshots, 1);
        // A legacy stream needs no fabricated replacement for new sidecars.
        let legacy = path.with_extension("legacy");
        let mut recorder = Recorder::create(
            &legacy,
            etsads_recorder::RunMetadata::new("synthetic", "old schema fixture"),
        )
        .unwrap();
        recorder
            .record("/telemetry", &Record::telemetry(&t))
            .unwrap();
        recorder.finish().unwrap();
        let old = etsads_recorder::replay(&legacy, |topic, record| {
            assert_eq!(topic, "/telemetry");
            assert_eq!(record.decode_wire::<TelemetryFrame>()?.header.frame_id, 2);
            Ok(())
        })
        .unwrap();
        assert_eq!(old.messages, 1);
        etsads_replay::ReplayAdapter::run(&legacy, |frame| {
            assert!(frame.configuration.is_none());
            assert!(frame.control_configuration.is_none());
            assert!(frame.dynamics.is_none());
            assert!(frame.camera_view.is_none());
            Ok(())
        })
        .unwrap();
    }

    #[test]
    fn recorded_config_full_boundary_overrides_later_legacy_sampling() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!(
                "configuration-boundary-{}_{}",
                std::process::id(),
                etsads_shm::qpc()
            ));
        let mut recorder = Recorder::create(
            &path,
            etsads_recorder::RunMetadata::new("synthetic", "configuration observation boundary"),
        )
        .unwrap();
        let earlier = ConfigurationFrame {
            header: WireHeader {
                qpc: 50,
                ..Default::default()
            },
            effective_from_frame_id: 1,
            config_seq: 1,
            truck: ConfigBlock {
                config_seq: 1,
                ..Default::default()
            },
            ..Default::default()
        };
        let later = ConfigurationFrame {
            header: WireHeader {
                frame_id: 1,
                qpc: 150,
                ..Default::default()
            },
            effective_from_frame_id: 2,
            config_seq: 2,
            truck: ConfigBlock {
                config_seq: 2,
                ..Default::default()
            },
            ..Default::default()
        };
        // A loop observed the latest ConfigBlock while draining an older frame.
        // Only ConfigFull carries the actual SDK callback/effective boundary.
        recorder
            .record("/config", &Record::wire(&later.truck, 1, 1000, 100))
            .unwrap();
        for frame in 1..=2 {
            let t = TelemetryFrame {
                header: WireHeader {
                    frame_id: frame,
                    sim_time_us: frame * 1000,
                    qpc: frame * 100,
                    ..Default::default()
                },
                qpc_frequency: 1000,
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&t))
                .unwrap();
        }
        for config in [earlier, later] {
            recorder
                .record(
                    "/config_full",
                    &Record::wire(
                        &config,
                        config.header.frame_id,
                        config.header.sim_time_us,
                        config.header.qpc,
                    ),
                )
                .unwrap();
        }
        recorder.finish().unwrap();
        let mut frames = 0;
        etsads_replay::ReplayAdapter::run(&path, |frame| {
            frames += 1;
            assert_eq!(frame.configuration.as_ref().unwrap().config_seq, frames);
            assert_eq!(frame.snapshot.config.unwrap().config_seq, frames);
            Ok(())
        })
        .unwrap();
        assert_eq!(frames, 2);
    }
}
