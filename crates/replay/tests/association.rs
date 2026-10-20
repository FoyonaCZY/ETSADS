use etsads_msgs::protocol::*;
use etsads_recorder::{Record, Recorder, RunMetadata};
use etsads_replay::ReplayAdapter;

fn telemetry(frame: u64) -> TelemetryFrame {
    TelemetryFrame {
        header: WireHeader {
            frame_id: frame,
            sim_time_us: frame * 16_666,
            qpc: frame * 166_666,
            ..Default::default()
        },
        paused_simulation_time_us: frame * 16_666,
        qpc_frequency: 10_000_000,
        position_scs: [frame as f64, 3.0, -10.0],
        ..Default::default()
    }
}
fn record<T: bytemuck::Pod>(value: &T, header: WireHeader) -> Record {
    Record::wire(value, header.frame_id, header.sim_time_us, header.qpc)
}

#[test]
fn controls_configuration_is_causal_and_unknown_replaces_safe_without_affecting_old_runs() {
    let mut adapter = ReplayAdapter::default();
    let mut frames = Vec::new();
    let mut sink = |f| {
        frames.push(f);
        Ok(())
    };
    let initial = ControlConfiguration {
        header: WireHeader {
            qpc: 1,
            ..Default::default()
        },
        session_start_qpc: 1,
        effective_from_frame_id: 1,
        ..Default::default()
    };
    let safe = ControlConfiguration {
        header: WireHeader {
            qpc: 100,
            seq: 1,
            ..Default::default()
        },
        generation: 1,
        shifter_type: SHIFTER_AUTOMATIC,
        valid: 1,
        ..initial
    };
    let changed = ControlConfiguration {
        header: WireHeader {
            frame_id: 1,
            qpc: telemetry(2).header.qpc + 1,
            seq: 2,
            ..Default::default()
        },
        generation: 2,
        effective_from_frame_id: 2,
        shifter_type: SHIFTER_UNKNOWN,
        valid: 0,
        ..safe
    };
    for value in [initial, safe, changed] {
        adapter
            .push(
                "/control_configuration",
                &record(&value, value.header),
                &mut sink,
            )
            .unwrap();
    }
    for n in 1..=3 {
        adapter
            .push("/telemetry", &Record::telemetry(&telemetry(n)), &mut sink)
            .unwrap();
    }
    adapter.finish(&mut sink).unwrap();
    assert!(frames[0].control_configuration.unwrap().safe_shifter());
    assert_eq!(
        frames[1].control_configuration.unwrap().generation,
        1,
        "event after frame-start cannot leak backwards"
    );
    assert_eq!(frames[2].control_configuration.unwrap().generation, 2);
    assert!(!frames[2].control_configuration.unwrap().safe_shifter());
    let mut legacy = ReplayAdapter::default();
    legacy
        .push("/telemetry", &Record::telemetry(&telemetry(1)), &mut |_| {
            Ok(())
        })
        .unwrap();
    legacy
        .finish(&mut |frame| {
            assert!(frame.control_configuration.is_none());
            Ok(())
        })
        .unwrap();
}

#[test]
fn controls_reject_wrong_session_duplicate_generation_invalid_enum_and_boundary() {
    let base = ControlConfiguration {
        header: WireHeader {
            qpc: 100,
            seq: 1,
            ..Default::default()
        },
        session_start_qpc: 50,
        generation: 1,
        effective_from_frame_id: 1,
        shifter_type: SHIFTER_AUTOMATIC,
        valid: 1,
    };
    for case in 0..5 {
        let mut adapter = ReplayAdapter::default();
        let mut sink = |_| Ok(());
        let info = BridgeInfo {
            session_start_qpc: 50,
            process_id: 7,
            ..Default::default()
        };
        adapter
            .push("/bridge_info", &Record::wire(&info, 0, 0, 50), &mut sink)
            .unwrap();
        let mut value = base;
        match case {
            0 => value.session_start_qpc = 40,
            1 => value.header.qpc = 49,
            2 => value.shifter_type = 99,
            3 => value.effective_from_frame_id = 3,
            4 => {
                adapter
                    .push(
                        "/control_configuration",
                        &record(&value, value.header),
                        &mut sink,
                    )
                    .unwrap();
            }
            _ => unreachable!(),
        }
        assert!(
            adapter
                .push(
                    "/control_configuration",
                    &record(&value, value.header),
                    &mut sink
                )
                .is_err(),
            "case {case}"
        );
    }
    let mut adapter = ReplayAdapter::default();
    let mut sink = |_| Ok(());
    adapter
        .push(
            "/control_configuration",
            &record(&base, base.header),
            &mut sink,
        )
        .unwrap();
    let later_identity = BridgeInfo {
        session_start_qpc: 60,
        ..Default::default()
    };
    assert!(
        adapter
            .push(
                "/bridge_info",
                &Record::wire(&later_identity, 0, 0, 60),
                &mut sink
            )
            .is_err()
    );
}

#[test]
fn joins_delayed_extension_and_video_preserves_event_causality_and_config_boundary() {
    let mut adapter = ReplayAdapter::default();
    let mut frames = Vec::new();
    let mut sink = |frame| {
        frames.push(frame);
        Ok(())
    };
    let config = ConfigBlock {
        config_seq: 1,
        wheel_count: 4,
        ..Default::default()
    };
    for n in 1..=4 {
        let raw = telemetry(n);
        adapter
            .push("/telemetry", &Record::telemetry(&raw), &mut sink)
            .unwrap();
        if n == 1 {
            adapter
                .push("/config", &record(&config, raw.header), &mut sink)
                .unwrap();
        }
    }
    let changed = ConfigBlock {
        config_seq: 2,
        wheel_count: 6,
        ..config
    };
    adapter
        .push("/config", &record(&changed, telemetry(3).header), &mut sink)
        .unwrap();
    let mut extension = Box::<ExtensionFrame>::default();
    extension.header = telemetry(2).header;
    extension.available = 2;
    adapter
        .push(
            "/extension",
            &record(extension.as_ref(), extension.header),
            &mut sink,
        )
        .unwrap();
    let mut event = GameEvent {
        header: telemetry(2).header,
        ..Default::default()
    };
    event.header.qpc += 1; // Must not leak future input into physical frame 2.
    event.name[..10].copy_from_slice(b"test.event");
    adapter
        .push("/events", &record(&event, event.header), &mut sink)
        .unwrap();
    let video = VideoIndex {
        video_frame: 0,
        capture_frame: 9,
        capture_qpc: telemetry(2).header.qpc + 10,
        physics_frame: 2,
        physics_qpc: telemetry(2).header.qpc,
        sim_time_us: telemetry(2).header.sim_time_us,
        alignment_error_qpc: 10,
        ..Default::default()
    };
    let evidence = VideoAlignment {
        video_frame: 0,
        capture_frame: 9,
        capture_qpc: video.capture_qpc,
        ..Default::default()
    };
    adapter
        .push(
            "/video_index",
            &Record::wire(&video, 2, video.sim_time_us, video.capture_qpc),
            &mut sink,
        )
        .unwrap();
    adapter
        .push(
            "/video_alignment",
            &Record::wire(&evidence, 2, video.sim_time_us, video.capture_qpc),
            &mut sink,
        )
        .unwrap();
    adapter.finish(&mut sink).unwrap();
    assert_eq!(
        frames
            .iter()
            .map(|f| f.snapshot.raw.header.frame_id)
            .collect::<Vec<_>>(),
        vec![1, 2, 3, 4]
    );
    assert_eq!(frames[0].snapshot.config.unwrap().wheel_count, 4);
    assert_eq!(frames[1].snapshot.config.unwrap().wheel_count, 4);
    assert_eq!(frames[2].snapshot.config.unwrap().wheel_count, 6);
    assert!(frames[0].snapshot.extension.is_none());
    assert_eq!(
        frames[1]
            .snapshot
            .extension
            .as_ref()
            .unwrap()
            .header
            .frame_id,
        2
    );
    assert!(frames[1].snapshot.events.is_empty());
    assert_eq!(frames[2].snapshot.events.len(), 1);
    assert!(frames[3].snapshot.events.is_empty());
    assert_eq!(frames[1].video.len(), 1);
    assert!(frames[1].video[0].alignment.is_some());
    assert_eq!(
        frames[0].snapshot.local_origin,
        frames[3].snapshot.local_origin
    );
    assert_eq!(frames[3].snapshot.origin_generation, 1);
}

#[test]
fn delayed_packets_cannot_rewrite_an_emitted_frame_and_mixed_sessions_fail() {
    let mut adapter = ReplayAdapter::default();
    let mut count = 0;
    let mut sink = |_| {
        count += 1;
        Ok(())
    };
    for n in 1..=130 {
        adapter
            .push("/telemetry", &Record::telemetry(&telemetry(n)), &mut sink)
            .unwrap();
    }
    let event = GameEvent {
        header: telemetry(1).header,
        ..Default::default()
    };
    assert!(
        adapter
            .push("/events", &record(&event, event.header), &mut sink)
            .is_err()
    );
    assert!(count > 0);
    let mut adapter = ReplayAdapter::default();
    let mut sink = |_| Ok(());
    let info = BridgeInfo {
        session_start_qpc: 1,
        process_id: 10,
        ..Default::default()
    };
    adapter
        .push("/bridge_info", &Record::wire(&info, 0, 0, 1), &mut sink)
        .unwrap();
    let changed = BridgeInfo {
        session_start_qpc: 2,
        ..info
    };
    assert!(
        adapter
            .push("/bridge_info", &Record::wire(&changed, 0, 0, 2), &mut sink)
            .is_err()
    );
}

#[test]
fn malformed_video_and_extension_associations_fail_instead_of_using_nearest_frame() {
    let mut adapter = ReplayAdapter::default();
    let mut sink = |_| Ok(());
    adapter
        .push("/telemetry", &Record::telemetry(&telemetry(2)), &mut sink)
        .unwrap();
    let mut extension = Box::<ExtensionFrame>::default();
    extension.header = telemetry(2).header;
    extension.header.sim_time_us += 1;
    adapter
        .push(
            "/extension",
            &record(extension.as_ref(), extension.header),
            &mut sink,
        )
        .unwrap();
    assert!(adapter.finish(&mut sink).is_err());
    let mut adapter = ReplayAdapter::default();
    adapter
        .push("/telemetry", &Record::telemetry(&telemetry(2)), &mut sink)
        .unwrap();
    let video = VideoIndex {
        physics_frame: 2,
        capture_qpc: 40,
        physics_qpc: 123,
        sim_time_us: telemetry(2).header.sim_time_us,
        ..Default::default()
    };
    adapter
        .push(
            "/video_index",
            &Record::wire(&video, 2, video.sim_time_us, 40),
            &mut sink,
        )
        .unwrap();
    assert!(adapter.finish(&mut sink).is_err());
}

#[test]
fn extension_with_same_frame_and_sim_time_but_different_qpc_is_rejected() {
    let mut adapter = ReplayAdapter::default();
    let mut delivered = 0;
    let mut sink = |_| {
        delivered += 1;
        Ok(())
    };
    let raw = telemetry(2);
    adapter
        .push("/telemetry", &Record::telemetry(&raw), &mut sink)
        .unwrap();
    let mut extension = Box::<ExtensionFrame>::default();
    extension.header = WireHeader {
        qpc: raw.header.qpc + 1,
        ..raw.header
    };
    adapter
        .push(
            "/extension",
            &record(extension.as_ref(), extension.header),
            &mut sink,
        )
        .unwrap();
    let error = adapter.finish(&mut sink).unwrap_err();
    assert!(error.to_string().contains("physical timestamp"));
    assert_eq!(delivered, 0);
}

#[test]
fn session_identity_rejects_old_records_in_either_arrival_order() {
    let header = WireHeader {
        qpc: 99,
        ..telemetry(1).header
    };
    let raw = TelemetryFrame {
        header,
        ..telemetry(1)
    };
    let mut extension = Box::<ExtensionFrame>::default();
    extension.header = header;
    let dynamics = DynamicsFrame {
        header,
        ..Default::default()
    };
    let view = CameraViewFrame {
        header,
        session_start_qpc: 100,
        ..Default::default()
    };
    let full = ConfigurationFrame {
        header,
        effective_from_frame_id: 2,
        ..Default::default()
    };
    let event = GameEvent {
        header,
        ..Default::default()
    };
    let packets = [
        ("/telemetry", Record::telemetry(&raw)),
        ("/extension", record(extension.as_ref(), header)),
        ("/dynamics", record(&dynamics, header)),
        ("/camera_view", record(&view, header)),
        ("/config_full", record(&full, header)),
        ("/config", record(&ConfigBlock::default(), header)),
        ("/events", record(&event, header)),
    ];
    let info = BridgeInfo {
        session_start_qpc: 100,
        process_id: 5,
        ..Default::default()
    };
    let info_record = Record::wire(&info, 0, 0, 100);
    for (topic, packet) in packets {
        for info_first in [true, false] {
            let mut adapter = ReplayAdapter::default();
            let mut sink = |_| panic!("invalid session must never be emitted");
            let error = if info_first {
                adapter
                    .push("/bridge_info", &info_record, &mut sink)
                    .unwrap();
                adapter.push(topic, &packet, &mut sink).unwrap_err()
            } else {
                adapter.push(topic, &packet, &mut sink).unwrap();
                adapter
                    .push("/bridge_info", &info_record, &mut sink)
                    .unwrap_err()
            };
            assert!(
                error.to_string().contains("predates SDK session"),
                "{topic}: {error}"
            );
        }
    }
}

#[test]
fn camera_session_identity_must_match_even_with_current_physical_qpc() {
    let info = BridgeInfo {
        session_start_qpc: 100,
        process_id: 5,
        ..Default::default()
    };
    let info_record = Record::wire(&info, 0, 0, 100);
    let header = telemetry(1).header;
    let wrong_view = CameraViewFrame {
        header,
        session_start_qpc: 99,
        ..Default::default()
    };
    for info_first in [true, false] {
        let mut adapter = ReplayAdapter::default();
        let mut sink = |_| panic!("wrong-session camera must never be emitted");
        let error = if info_first {
            adapter
                .push("/bridge_info", &info_record, &mut sink)
                .unwrap();
            adapter
                .push("/camera_view", &record(&wrong_view, header), &mut sink)
                .unwrap_err()
        } else {
            adapter
                .push("/camera_view", &record(&wrong_view, header), &mut sink)
                .unwrap();
            adapter
                .push("/bridge_info", &info_record, &mut sink)
                .unwrap_err()
        };
        assert!(error.to_string().contains("another SDK session"));
    }
    let mut adapter = ReplayAdapter::default();
    let mut count = 0;
    let mut sink = |frame: etsads_replay::ReplayFrame| {
        count += 1;
        assert_eq!(frame.snapshot.session_start_qpc, 100);
        assert_eq!(frame.camera_view.unwrap().session_start_qpc, 100);
        Ok(())
    };
    adapter
        .push("/bridge_info", &info_record, &mut sink)
        .unwrap();
    adapter
        .push("/telemetry", &Record::telemetry(&telemetry(1)), &mut sink)
        .unwrap();
    let valid = CameraViewFrame {
        session_start_qpc: 100,
        ..wrong_view
    };
    adapter
        .push("/camera_view", &record(&valid, header), &mut sink)
        .unwrap();
    adapter.finish(&mut sink).unwrap();
    assert_eq!(count, 1);
}

#[test]
fn complete_recording_flows_through_live_snapshot_conversion_without_ipc() {
    let dir = std::env::temp_dir().join(format!(
        "etsads-replay-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let mut recorder = Recorder::create(&dir, RunMetadata::new("synthetic", "test")).unwrap();
    for n in 1..=300 {
        let mut raw = telemetry(n);
        raw.angular_velocity_local[1] = 0.25;
        raw.wheel_steering[0] = 0.125;
        recorder
            .record("/telemetry", &Record::telemetry(&raw))
            .unwrap();
    }
    recorder.finish().unwrap();
    let mut count = 0;
    let report = ReplayAdapter::run(&dir, |frame| {
        count += 1;
        assert_eq!(frame.snapshot.ego.yaw_rate, std::f64::consts::FRAC_PI_2);
        assert_eq!(frame.snapshot.ego.wheel_angle, std::f64::consts::FRAC_PI_4);
        assert_eq!(frame.snapshot.ego.pose.y, 10.0);
        assert_eq!(
            frame.snapshot.session_start_qpc, 0,
            "legacy identity remains unknown"
        );
        assert!(
            frame.configuration.is_none()
                && frame.dynamics.is_none()
                && frame.camera_view.is_none()
        );
        Ok(())
    })
    .unwrap();
    assert_eq!(count, 300);
    assert_eq!(report.associations.missing_extension_frames, 300);
    assert_eq!(report.associations.maximum_buffered_frames, 65);
    assert!(!report.phase_acceptance);
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn precise_configuration_and_optional_sidecars_obey_real_event_boundaries() {
    let mut adapter = ReplayAdapter::default();
    let mut frames = Vec::new();
    let mut sink = |frame| {
        frames.push(frame);
        Ok(())
    };
    for n in 1..=3 {
        adapter
            .push("/telemetry", &Record::telemetry(&telemetry(n)), &mut sink)
            .unwrap();
    }
    let legacy = ConfigBlock {
        config_seq: 1,
        wheel_count: 4,
        ..Default::default()
    };
    adapter
        .push("/config", &record(&legacy, telemetry(1).header), &mut sink)
        .unwrap();
    let full = ConfigurationFrame {
        header: WireHeader {
            qpc: telemetry(1).header.qpc + 1,
            ..telemetry(1).header
        },
        effective_from_frame_id: 2,
        config_seq: 2,
        truck: ConfigBlock {
            config_seq: 2,
            wheel_count: 6,
            ..legacy
        },
        ..Default::default()
    };
    adapter
        .push("/config_full", &record(&full, full.header), &mut sink)
        .unwrap();
    let dynamics = DynamicsFrame {
        header: telemetry(2).header,
        ..Default::default()
    };
    adapter
        .push("/dynamics", &record(&dynamics, dynamics.header), &mut sink)
        .unwrap();
    let view = CameraViewFrame {
        header: telemetry(2).header,
        config_seq: 2,
        selected_hypothesis: u32::MAX,
        ..Default::default()
    };
    adapter
        .push("/camera_view", &record(&view, view.header), &mut sink)
        .unwrap();
    adapter.finish(&mut sink).unwrap();
    assert!(
        frames[0].snapshot.config.is_none(),
        "precise stream has no established configuration before its first event"
    );
    assert!(frames[0].configuration.is_none());
    assert_eq!(frames[1].snapshot.config.unwrap().wheel_count, 6);
    assert_eq!(frames[2].configuration.as_ref().unwrap().config_seq, 2);
    assert!(frames[0].dynamics.is_none());
    assert!(frames[1].dynamics.is_some());
    assert!(frames[2].camera_view.is_none());
    assert_eq!(frames[1].camera_view.unwrap().selected_hypothesis, u32::MAX);
}

#[test]
fn newer_legacy_observation_never_backdates_an_exact_configuration_event() {
    let mut adapter = ReplayAdapter::default();
    let mut frames = Vec::new();
    let mut sink = |frame| {
        frames.push(frame);
        Ok(())
    };
    for n in 1..=3 {
        adapter
            .push("/telemetry", &Record::telemetry(&telemetry(n)), &mut sink)
            .unwrap();
    }
    let initial = ConfigurationFrame {
        header: WireHeader {
            qpc: 1,
            ..Default::default()
        },
        effective_from_frame_id: 1,
        config_seq: 1,
        truck: ConfigBlock {
            config_seq: 1,
            wheel_count: 4,
            ..Default::default()
        },
        ..Default::default()
    };
    let future = ConfigurationFrame {
        header: WireHeader {
            qpc: telemetry(1).header.qpc + 1,
            ..telemetry(1).header
        },
        effective_from_frame_id: 2,
        config_seq: 2,
        truck: ConfigBlock {
            config_seq: 2,
            wheel_count: 6,
            ..Default::default()
        },
        ..Default::default()
    };
    adapter
        .push(
            "/config",
            &record(&future.truck, telemetry(1).header),
            &mut sink,
        )
        .unwrap();
    adapter
        .push("/config_full", &record(&initial, initial.header), &mut sink)
        .unwrap();
    adapter
        .push("/config_full", &record(&future, future.header), &mut sink)
        .unwrap();
    adapter.finish(&mut sink).unwrap();
    assert_eq!(frames[0].snapshot.config.unwrap().wheel_count, 4);
    assert_eq!(frames[1].snapshot.config.unwrap().wheel_count, 6);
}
