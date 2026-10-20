#![cfg(windows)]
//! Real supervisor subprocesses with isolated named mappings. These are IPC
//! regression tests, explicitly not real-game manual-takeover acceptance.
use etsads_msgs::protocol::*;
use etsads_shm::{SharedCell, SharedRing, qpc, qpc_frequency};
use std::time::{Duration, Instant};
struct Child(std::process::Child);
impl Drop for Child {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
fn controls_fixture(
    name: impl Fn(&str) -> String,
) -> (SharedRing<ControlConfiguration, 64>, SharedCell<BridgeInfo>) {
    let controls =
        SharedRing::create(&name(CONTROL_CONFIGURATION_MAPPING), SCHEMA_VERSION).unwrap();
    let info = SharedCell::create(&name(INFO_MAPPING), SCHEMA_VERSION).unwrap();
    let session = qpc();
    info.publish(&BridgeInfo {
        session_start_qpc: session,
        telemetry_api_version: 0x10001,
        input_api_version: 0x10000,
        ..Default::default()
    })
    .unwrap();
    controls
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
    (controls, info)
}
#[test]
fn command_loss_revokes_host_authority_and_pause_stop_release_within_300ms() {
    let namespace = format!("test_{}_{}", std::process::id(), qpc());
    let name = |base: &str| format!("{base}_{namespace}");
    let (controls, _info) = controls_fixture(name);
    let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
        &name(TELEMETRY_MAPPING),
        SCHEMA_VERSION,
    )
    .unwrap();
    let commands =
        SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
    let extensions =
        SharedRing::<ExtensionFrame, 4>::create(&name(EXTENSION_MAPPING), SCHEMA_VERSION).unwrap();
    let gate = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
    let request =
        SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION).unwrap();
    let mut t = TelemetryFrame {
        qpc_frequency: qpc_frequency(),
        ..Default::default()
    };
    t.header.frame_id = 1;
    t.header.qpc = qpc();
    let mut c = ControlCmd {
        seq: 1,
        source_frame_id: 1,
        write_qpc: qpc(),
        ..Default::default()
    };
    let mut e = Box::<ExtensionFrame>::default();
    e.header = t.header;
    e.available = 2;
    telemetry.publish(&t).unwrap();
    commands.publish(&c).unwrap();
    extensions.publish(&e).unwrap();
    let mut process = std::process::Command::new(env!("CARGO_BIN_EXE_etsads-supervisor"));
    process
        .args(["--namespace", &namespace, "--no-hotkey", "--duration", "8"])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null());
    use std::os::windows::process::CommandExt;
    process.creation_flags(0x08000000);
    let _child = Child(process.spawn().unwrap());
    let deadline = Instant::now();
    while gate.latest().is_none() {
        assert!(deadline.elapsed() < Duration::from_secs(3));
        std::thread::sleep(Duration::from_millis(5));
    }
    let mut seq = 0;
    let mut publish = |paused: bool, command: bool| {
        t.header.qpc = qpc();
        t.header.frame_id += 1;
        t.header.sim_time_us += 10_000;
        t.paused = u32::from(paused);
        telemetry.publish(&t).unwrap();
        e.header = t.header;
        extensions.publish(&e).unwrap();
        if command {
            c.write_qpc = qpc();
            c.source_frame_id = t.header.frame_id;
            c.seq += 1;
            commands.publish(&c).unwrap();
        }
    };
    for condition in 0..4 {
        publish(false, true);
        seq += 1;
        request
            .publish(&SafetyRequest {
                seq,
                write_qpc: qpc(),
                arm: 1,
                reserved: 0,
            })
            .unwrap();
        let deadline = Instant::now();
        while gate.latest().is_none_or(|g| g.enabled == 0) {
            publish(false, true);
            assert!(
                deadline.elapsed() < Duration::from_secs(1),
                "could not arm isolated supervisor"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        let start = Instant::now();
        if condition == 3 {
            let mut unsafe_mode = controls.latest().unwrap();
            unsafe_mode.generation += 1;
            unsafe_mode.header.seq = unsafe_mode.generation;
            unsafe_mode.shifter_type = SHIFTER_ARCADE;
            unsafe_mode.header.qpc = qpc();
            controls.publish(&unsafe_mode).unwrap();
        }
        if condition == 2 {
            seq += 1;
            request
                .publish(&SafetyRequest {
                    seq,
                    write_qpc: qpc(),
                    arm: 0,
                    reserved: 0,
                })
                .unwrap();
        }
        loop {
            publish(condition == 1, condition != 0);
            if gate.latest().is_some_and(|g| {
                if condition == 0 {
                    g.enabled != GATE_ACTIVE
                } else {
                    g.enabled == GATE_OFF
                }
            }) {
                break;
            }
            assert!(
                start.elapsed() < Duration::from_millis(300),
                "release deadline exceeded for condition {condition}"
            );
            std::thread::sleep(Duration::from_millis(3));
        }
        assert!(start.elapsed() < Duration::from_millis(300));
        if condition == 3 {
            let denied = gate.latest().unwrap();
            assert_eq!(
                (denied.enabled, denied.aeb_brake, denied.max_throttle),
                (GATE_OFF, 0.0, 0.0)
            );
            let mut restored = controls.latest().unwrap();
            restored.generation += 1;
            restored.header.seq = restored.generation;
            restored.shifter_type = SHIFTER_AUTOMATIC;
            restored.header.qpc = qpc();
            controls.publish(&restored).unwrap();
            for _ in 0..6 {
                publish(false, true);
                std::thread::sleep(Duration::from_millis(10));
            }
            assert_eq!(
                gate.latest().unwrap().enabled,
                GATE_OFF,
                "safe mode restoration is not an arm request"
            );
        }
    }
}

fn hidden_child(command: &mut std::process::Command) -> Child {
    use std::os::windows::process::CommandExt;
    command
        .creation_flags(0x08000000)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null());
    Child(command.spawn().unwrap())
}
fn fixture_child(name: &str, namespace: &str) -> Child {
    hidden_child(
        std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", name, "--ignored"])
            .env("ETSADS_ISOLATED_LOSS_FIXTURE", namespace),
    )
}
#[test]
#[ignore = "isolated child fixture invoked by command_publisher_kill_drives_brake_only_then_latches_off"]
fn isolated_command_publisher() {
    let Ok(namespace) = std::env::var("ETSADS_ISOLATED_LOSS_FIXTURE") else {
        return;
    };
    assert!(namespace.starts_with("loss_test_"));
    let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
        &format!("{TELEMETRY_MAPPING}_{namespace}"),
        SCHEMA_VERSION,
    )
    .unwrap();
    let commands =
        SharedCell::<ControlCmd>::open(&format!("{CONTROL_MAPPING}_{namespace}"), SCHEMA_VERSION)
            .unwrap();
    let mut sequence = 1;
    loop {
        if let Some(frame) = telemetry.latest() {
            commands
                .publish(&ControlCmd {
                    seq: sequence,
                    source_frame_id: frame.header.frame_id,
                    write_qpc: qpc(),
                    steer: 0.4,
                    throttle: 0.25,
                    clutch: 0.2,
                    holds: 7,
                    ..Default::default()
                })
                .unwrap();
            sequence += 1;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}
#[test]
#[ignore = "isolated InputPolicy child; never loads or controls a game"]
fn isolated_sdk_policy_consumer() {
    let Ok(namespace) = std::env::var("ETSADS_ISOLATED_LOSS_FIXTURE") else {
        return;
    };
    assert!(namespace.starts_with("loss_test_"));
    let commands =
        SharedCell::<ControlCmd>::open(&format!("{CONTROL_MAPPING}_{namespace}"), SCHEMA_VERSION)
            .unwrap();
    let gates =
        SharedCell::<Gate>::open(&format!("{GATE_MAPPING}_{namespace}"), SCHEMA_VERSION).unwrap();
    let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
        &format!("{TELEMETRY_MAPPING}_{namespace}"),
        SCHEMA_VERSION,
    )
    .unwrap();
    let output = SharedCell::<AppliedControl>::open(
        &format!("{APPLIED_MAPPING}_{namespace}"),
        SCHEMA_VERSION,
    )
    .unwrap();
    let mut policy = etsads_game::input::InputPolicy::default();
    let mut frame = 0;
    loop {
        let command = commands.latest();
        let gate = gates.latest();
        let raw = telemetry.latest();
        let now = qpc();
        let applied = policy.apply(
            command.as_ref(),
            gate.as_ref(),
            now,
            qpc_frequency(),
            raw.is_none_or(|t| t.paused != 0),
        );
        if applied.reason == etsads_game::input::REASON_BRAKE_ONLY {
            assert_eq!(applied.holds, 0);
        }
        frame += 1;
        output
            .publish(&AppliedControl {
                applied_seq: command.map_or(0, |c| c.seq),
                applied_qpc: now,
                frame_index: frame,
                axes: applied.axes,
                reason: applied.reason,
                reserved: 0,
            })
            .unwrap();
        std::thread::sleep(Duration::from_millis(3));
    }
}
#[test]
fn command_publisher_kill_drives_brake_only_then_latches_off() {
    run_command_loss_fixture(false);
}
#[test]
fn supervisor_crash_near_retreat_end_cannot_extend_input_brake_deadline() {
    run_command_loss_fixture(true);
}
fn run_command_loss_fixture(kill_supervisor_late: bool) {
    // Three real children (supervisor, command producer, SDK policy consumer),
    // only isolated synthetic IPC. This is not ETS2 braking acceptance.
    let namespace = format!("loss_test_{}_{}", std::process::id(), qpc());
    let name = |base: &str| format!("{base}_{namespace}");
    let (_controls, _info) = controls_fixture(name);
    let telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
        &name(TELEMETRY_MAPPING),
        SCHEMA_VERSION,
    )
    .unwrap();
    let commands =
        SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
    let extensions =
        SharedRing::<ExtensionFrame, 4>::create(&name(EXTENSION_MAPPING), SCHEMA_VERSION).unwrap();
    let gates = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
    let applied =
        SharedCell::<AppliedControl>::create(&name(APPLIED_MAPPING), SCHEMA_VERSION).unwrap();
    let requests =
        SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION).unwrap();
    let mut frame = 0;
    let physics_start = Instant::now();
    let mut last_telemetry = TelemetryFrame::default();
    let mut tick = || {
        let physical_frame = physics_start.elapsed().as_micros() as u64 / 16667 + 1;
        if physical_frame == frame {
            return last_telemetry;
        }
        frame = physical_frame;
        let t = TelemetryFrame {
            header: WireHeader {
                frame_id: frame,
                sim_time_us: frame * 16667,
                qpc: qpc(),
                ..Default::default()
            },
            qpc_frequency: qpc_frequency(),
            ..Default::default()
        };
        telemetry.publish(&t).unwrap();
        let mut e = Box::<ExtensionFrame>::default();
        e.header = t.header;
        e.available = 2;
        extensions.publish(&e).unwrap();
        last_telemetry = t;
        t
    };
    tick();
    let mut supervisor = hidden_child(
        std::process::Command::new(env!("CARGO_BIN_EXE_etsads-supervisor")).args([
            "--namespace",
            &namespace,
            "--no-hotkey",
            "--duration",
            "8",
        ]),
    );
    let mut publisher = fixture_child("isolated_command_publisher", &namespace);
    let mut consumer = fixture_child("isolated_sdk_policy_consumer", &namespace);
    let ready = Instant::now();
    while gates.latest().is_none() || commands.latest().is_none() || applied.latest().is_none() {
        tick();
        assert!(ready.elapsed() < Duration::from_secs(2));
        std::thread::sleep(Duration::from_millis(5));
    }
    requests
        .publish(&SafetyRequest {
            seq: 1,
            write_qpc: qpc(),
            arm: 1,
            reserved: 0,
        })
        .unwrap();
    let started = Instant::now();
    while !applied.latest().is_some_and(|a| a.axes[1] > 0.2) {
        tick();
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "isolated supervisor failed to arm"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    publisher.0.kill().unwrap();
    publisher.0.wait().unwrap();
    let original = commands.latest().unwrap();
    let mut revoked_age = None;
    let mut max_brake = 0.0f32;
    let mut max_gate_brake = 0.0f32;
    let mut modes = std::collections::BTreeSet::new();
    let mut reasons = std::collections::BTreeSet::new();
    let mut resumed = original;
    let mut supervisor_killed = false;
    let mut observed_zero_after_deadline = false;
    let monitor = Instant::now();
    while monitor.elapsed() < Duration::from_millis(1150) {
        let t = tick();
        let now = qpc();
        let age = (now - original.write_qpc) as f64 / qpc_frequency() as f64;
        if kill_supervisor_late && !supervisor_killed && age >= 0.85 {
            supervisor.0.kill().unwrap();
            supervisor.0.wait().unwrap();
            supervisor_killed = true;
        }
        // A recovered writer must neither reset the original brake deadline nor
        // regain steering, throttle or button authority without a new request.
        if age > 0.4 {
            resumed.seq += 1;
            resumed.source_frame_id = t.header.frame_id;
            resumed.write_qpc = now;
            resumed.pulses[0] = resumed.pulses[0].wrapping_add(1);
            commands.publish(&resumed).unwrap();
        }
        // A seqlock reader may miss the writer's short publication window.
        // Skip only that observation, never reuse an older Active snapshot.
        let Some(gate) = gates.latest() else {
            std::thread::yield_now();
            continue;
        };
        modes.insert(gate.enabled);
        max_gate_brake = max_gate_brake.max(gate.aeb_brake);
        if gate.enabled != GATE_ACTIVE {
            revoked_age.get_or_insert(age);
        }
        if age > 0.3 {
            assert_ne!(
                gate.enabled, GATE_ACTIVE,
                "lost host authority returned automatically"
            );
        }
        if let Some(a) = applied.latest() {
            reasons.insert(a.reason);
            if a.reason == etsads_game::input::REASON_BRAKE_ONLY {
                assert_eq!([a.axes[0], a.axes[1], a.axes[3]], [0.0; 3]);
                max_brake = max_brake.max(a.axes[2]);
                assert!(a.axes[2] <= 0.3);
            }
            if age > 1.08 {
                if !supervisor_killed {
                    assert_eq!(gate.enabled, GATE_OFF);
                }
                assert_eq!(a.axes, [0.0; 4]);
                observed_zero_after_deadline = true;
            }
        }
        if !supervisor_killed {
            assert!(supervisor.0.try_wait().unwrap().is_none());
        }
        assert!(consumer.0.try_wait().unwrap().is_none());
        std::thread::sleep(Duration::from_millis(3));
    }
    assert!(revoked_age.unwrap() <= 0.3);
    assert!(
        observed_zero_after_deadline,
        "no coherent zero-output observation after the original deadline"
    );
    assert!(
        max_brake >= 0.28,
        "independent brake retreat was not observed: max_applied={max_brake}, max_gate={max_gate_brake}, modes={modes:?}, input_reasons={reasons:?}"
    );
    println!(
        "synthetic IPC only: host authority revoked {:.3} ms after last command; peak independent brake={max_brake}; full zero confirmed after original age 1 s; recovered command stayed disarmed; supervisor_killed_late={supervisor_killed}",
        revoked_age.unwrap() * 1000.0
    );
    if supervisor_killed {
        return;
    }
    requests
        .publish(&SafetyRequest {
            seq: 2,
            write_qpc: qpc(),
            arm: 1,
            reserved: 0,
        })
        .unwrap();
    let rearm = Instant::now();
    loop {
        let t = tick();
        resumed.seq += 1;
        resumed.write_qpc = qpc();
        resumed.source_frame_id = t.header.frame_id;
        commands.publish(&resumed).unwrap();
        if gates.latest().is_some_and(|g| g.enabled == GATE_ACTIVE) {
            break;
        }
        assert!(rearm.elapsed() < Duration::from_millis(300));
        std::thread::sleep(Duration::from_millis(3));
    }
}
