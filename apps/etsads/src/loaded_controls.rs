//! Loaded SDK state, independent of on-disk profile settings and user assertions.
use anyhow::{Context, ensure};
use etsads_msgs::protocol::*;
use etsads_shm::{SharedCell, SharedRing, qpc};

/// A single-slot seqlock can be temporarily unreadable while its producer is
/// publishing. Reacquire a coherent snapshot for at most 2 ms; never reuse a
/// cached permission and never wait past an observed Off/stale value.
pub(crate) fn coherent_read<T>(mut read: impl FnMut() -> Option<T>) -> Option<T> {
    let started = std::time::Instant::now();
    loop {
        if let Some(value) = read() {
            return Some(value);
        }
        if started.elapsed() >= std::time::Duration::from_millis(2) {
            return None;
        }
        std::thread::yield_now();
    }
}

pub(crate) struct LoadedControls {
    configurations: SharedRing<ControlConfiguration, 64>,
    info: SharedCell<BridgeInfo>,
    bound: Option<(u64, u64)>,
}
impl LoadedControls {
    pub fn connect(namespace: &str) -> anyhow::Result<Self> {
        Ok(Self {
            configurations: SharedRing::open(&format!("{CONTROL_CONFIGURATION_MAPPING}{namespace}"), SCHEMA_VERSION)
                .context("SDK controls state unavailable; deploy the current bridge before any input experiment")?,
            info: SharedCell::open(&format!("{INFO_MAPPING}{namespace}"), SCHEMA_VERSION)
                .context("SDK session identity unavailable")?,
            bound: None,
        })
    }
    pub fn validate(&self, telemetry: &TelemetryFrame) -> anyhow::Result<ControlConfiguration> {
        let current = self.configurations.latest();
        let info = self
            .info
            .latest()
            .context("SDK session identity uninitialized")?;
        ensure!(
            info.telemetry_api_version != 0 && info.input_api_version != 0,
            "SDK controls session is not active"
        );
        etsads_safety::validate_control_configuration(
            current.as_ref(),
            info.session_start_qpc,
            telemetry,
            qpc(),
        )
        .map_err(anyhow::Error::msg)?;
        let current = current.unwrap();
        ensure!(
            self.bound
                .is_none_or(|bound| bound == (current.session_start_qpc, current.generation)),
            "SDK controls mode/session changed during this run; inputs released and a new explicit run is required"
        );
        Ok(current)
    }
    pub fn bind(&mut self, telemetry: &TelemetryFrame) -> anyhow::Result<()> {
        let current = self.validate(telemetry)?;
        self.bound = Some((current.session_start_qpc, current.generation));
        Ok(())
    }
}

pub(crate) fn validate_stationary_steering(
    initial: &TelemetryFrame,
    current: &TelemetryFrame,
) -> anyhow::Result<()> {
    let displacement = initial
        .position_scs
        .iter()
        .zip(current.position_scs)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt();
    ensure!(
        current.gear == initial.gear,
        "stationary steering experiment changed gear unexpectedly; released"
    );
    ensure!(
        current.effective[1].is_finite() && current.effective[1] <= 0.01,
        "stationary zero-throttle experiment produced effective throttle; released"
    );
    ensure!(
        current.speed_mps.is_finite()
            && current.speed_mps.abs() <= 0.05
            && displacement.is_finite()
            && displacement <= 0.1,
        "stationary steering experiment moved; released"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(windows)]
    #[test]
    fn gate_publication_window_reacquires_new_off_snapshot_never_cached_active() {
        use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};
        let name = format!(
            "ETSADS_Test_GatePublication_{}_{}",
            std::process::id(),
            qpc()
        );
        let writer = SharedCell::<Gate>::create(&name, SCHEMA_VERSION).unwrap();
        let reader = SharedCell::<Gate>::open(&name, SCHEMA_VERSION).unwrap();
        writer
            .publish(&Gate {
                enabled: GATE_ACTIVE,
                heartbeat_qpc: qpc(),
                ..Default::default()
            })
            .unwrap();
        let mapping = etsads_shm::Mapping::open(&name).unwrap();
        assert!(mapping.len() >= SharedCell::<Gate>::mapping_size());
        // Controlled interleaving of the actual documented seqlock ABI: pause
        // the only writer after acquiring its slot, then finish it after the
        // reader's first attempt. No production or game namespace is touched.
        let latest = unsafe { &*mapping.as_ptr().add(32).cast::<AtomicU64>() };
        let slot = unsafe { &*mapping.as_ptr().add(64).cast::<AtomicU64>() };
        let old = slot.load(Ordering::SeqCst);
        slot.store(old + 1, Ordering::SeqCst);
        assert!(
            reader.latest().is_none(),
            "an in-flight valid publisher is not a revoked Gate"
        );
        let off = Gate {
            enabled: GATE_OFF,
            heartbeat_qpc: qpc(),
            ..Default::default()
        };
        let mut completed = false;
        let started = std::time::Instant::now();
        let observed = coherent_read(|| {
            let value = reader.latest();
            if !completed {
                for (i, byte) in bytemuck::bytes_of(&off).iter().enumerate() {
                    // Mapping remains live; payload bytes are accessed only
                    // atomically, matching SharedRing's production writer.
                    unsafe { &*mapping.as_ptr().add(72 + i).cast::<AtomicU8>() }
                        .store(*byte, Ordering::SeqCst);
                }
                slot.store(old + 2, Ordering::SeqCst);
                latest.fetch_add(1, Ordering::SeqCst);
                completed = true;
            }
            value
        })
        .unwrap();
        let acquired_elapsed = started.elapsed();
        assert_eq!(observed.enabled, GATE_OFF);
        let unavailable = std::time::Instant::now();
        assert!(coherent_read::<Gate>(|| None).is_none());
        eprintln!(
            "deterministic seqlock None observed; committed Off reacquired in {:?}; permanent missing returned None after {:?}",
            acquired_elapsed,
            unavailable.elapsed()
        );
    }
    #[test]
    fn r4_effective_throttle_gear_and_motion_are_independent_abort_reasons() {
        let before = TelemetryFrame::default();
        assert!(validate_stationary_steering(&before, &before).is_ok());
        for variant in 0..4 {
            let mut now = before;
            match variant {
                0 => now.gear = -1,
                1 => now.effective[1] = 0.25,
                2 => now.speed_mps = -0.05967,
                _ => now.position_scs[0] = 0.11,
            }
            assert!(validate_stationary_steering(&before, &now).is_err());
        }
    }
    #[cfg(windows)]
    #[test]
    fn actual_mode_reader_binds_session_generation_and_rejects_unsafe_updates() {
        let suffix = format!("_LoadedControlsTest_{}_{}", std::process::id(), qpc());
        let ring = SharedRing::<ControlConfiguration, 64>::create(
            &format!("{CONTROL_CONFIGURATION_MAPPING}{suffix}"),
            SCHEMA_VERSION,
        )
        .unwrap();
        let info =
            SharedCell::<BridgeInfo>::create(&format!("{INFO_MAPPING}{suffix}"), SCHEMA_VERSION)
                .unwrap();
        let session = qpc();
        info.publish(&BridgeInfo {
            session_start_qpc: session,
            telemetry_api_version: 0x10001,
            input_api_version: 0x10000,
            ..Default::default()
        })
        .unwrap();
        let mut c = ControlConfiguration {
            header: WireHeader {
                qpc: session,
                seq: 1,
                ..Default::default()
            },
            session_start_qpc: session,
            generation: 1,
            effective_from_frame_id: 1,
            valid: 1,
            shifter_type: SHIFTER_AUTOMATIC,
        };
        ring.publish(&c).unwrap();
        let mut t = TelemetryFrame::default();
        t.header.qpc = qpc();
        t.header.frame_id = 10;
        let mut reader = LoadedControls::connect(&suffix).unwrap();
        reader.bind(&t).unwrap();
        c.generation = 2;
        c.header.seq = 2;
        c.shifter_type = SHIFTER_ARCADE;
        ring.publish(&c).unwrap();
        assert!(reader.validate(&t).is_err());
        c.generation = 3;
        c.header.seq = 3;
        c.shifter_type = SHIFTER_AUTOMATIC;
        ring.publish(&c).unwrap();
        assert!(
            reader
                .validate(&t)
                .unwrap_err()
                .to_string()
                .contains("changed")
        );
        let mut new_run = LoadedControls::connect(&suffix).unwrap();
        new_run.bind(&t).unwrap();
        info.publish(&BridgeInfo {
            session_start_qpc: session + 1,
            telemetry_api_version: 0x10001,
            input_api_version: 0x10000,
            ..Default::default()
        })
        .unwrap();
        assert!(new_run.validate(&t).is_err());
    }
    #[cfg(windows)]
    #[test]
    fn manual_brake_probe_refuses_arcade_before_arm_or_nonzero_command() {
        for mode in [SHIFTER_ARCADE, SHIFTER_UNKNOWN] {
            let suffix = format!("_ManualModeTest_{}_{}_{}", std::process::id(), qpc(), mode);
            let name = |base: &str| format!("{base}{suffix}");
            let modes = SharedRing::<ControlConfiguration, 64>::create(
                &name(CONTROL_CONFIGURATION_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let info =
                SharedCell::<BridgeInfo>::create(&name(INFO_MAPPING), SCHEMA_VERSION).unwrap();
            let raw = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
                &name(TELEMETRY_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let commands =
                SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
            let gates = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
            let applied =
                SharedCell::<AppliedControl>::create(&name(APPLIED_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let requests =
                SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let session = qpc();
            info.publish(&BridgeInfo {
                session_start_qpc: session,
                telemetry_api_version: 0x10001,
                input_api_version: 0x10000,
                ..Default::default()
            })
            .unwrap();
            modes
                .publish(&ControlConfiguration {
                    header: WireHeader {
                        qpc: session,
                        seq: 1,
                        ..Default::default()
                    },
                    session_start_qpc: session,
                    generation: 1,
                    effective_from_frame_id: 1,
                    shifter_type: mode,
                    valid: u32::from(mode != SHIFTER_UNKNOWN),
                })
                .unwrap();
            raw.publish(&TelemetryFrame {
                header: WireHeader {
                    frame_id: 1,
                    qpc: qpc(),
                    ..Default::default()
                },
                qpc_frequency: etsads_shm::qpc_frequency(),
                ..Default::default()
            })
            .unwrap();
            gates
                .publish(&Gate {
                    heartbeat_qpc: qpc(),
                    ..Default::default()
                })
                .unwrap();
            applied
                .publish(&AppliedControl {
                    applied_qpc: qpc(),
                    ..Default::default()
                })
                .unwrap();
            let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let error = crate::run_manual_command(
                crate::ManualCommand {
                    steer: 0.3,
                    throttle: 0.0,
                    brake: 0.5,
                    seconds: 3.0,
                    pulse: None,
                    arm: true,
                },
                &stop,
                &suffix,
            )
            .unwrap_err();
            assert!(error.to_string().contains("shifter mode"), "{error:#}");
            let last = commands.latest().unwrap();
            assert_eq!(
                [last.steer, last.throttle, last.brake, last.clutch],
                [0.0; 4]
            );
            assert!(last.pulses.iter().all(|p| *p == 0));
            let request = requests.latest().unwrap();
            assert_eq!(
                (request.seq, request.arm),
                (1, 0),
                "only RAII release, never an arm request"
            );
        }
    }
    #[cfg(windows)]
    #[test]
    fn manual_pulse_success_requires_its_matching_completed_edge() {
        use std::sync::{
            Arc,
            atomic::{AtomicBool, Ordering},
        };
        use std::time::{Duration, Instant};
        for (pulse_name, matching_edge) in [
            ("menu", true),
            ("menu", false),
            ("engine", true),
            ("engine", false),
        ] {
            let expects_pause = pulse_name == "menu";
            let suffix = format!("_ManualPauseTest_{}_{}", std::process::id(), qpc());
            let name = |base: &str| format!("{base}{suffix}");
            let modes = SharedRing::<ControlConfiguration, 64>::create(
                &name(CONTROL_CONFIGURATION_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let info =
                SharedCell::<BridgeInfo>::create(&name(INFO_MAPPING), SCHEMA_VERSION).unwrap();
            let raw = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::create(
                &name(TELEMETRY_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let commands =
                SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
            let gates = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
            let applied =
                SharedCell::<AppliedControl>::create(&name(APPLIED_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let requests =
                SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let actions = SharedRing::<InputActionFrame, 256>::create(
                &name(INPUT_ACTIONS_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let session = qpc();
            info.publish(&BridgeInfo {
                session_start_qpc: session,
                telemetry_api_version: 0x10001,
                input_api_version: 0x10000,
                ..Default::default()
            })
            .unwrap();
            modes
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
            let fixture_stop = Arc::new(AtomicBool::new(false));
            let done = fixture_stop.clone();
            let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);
            let button = etsads_game::input::PULSE_NAMES
                .iter()
                .position(|n| *n == pulse_name)
                .unwrap();
            let worker = std::thread::spawn(move || {
                let start = Instant::now();
                let mut frame = 0;
                let mut emitted = false;
                let mut trace = std::collections::VecDeque::new();
                let mut request_seq = 0;
                let mut armed = false;
                while !done.load(Ordering::Acquire) && start.elapsed() < Duration::from_secs(4) {
                    frame += 1;
                    let command = commands.latest().unwrap_or_default();
                    // Model the actual supervisor's persistent arm state. A
                    // transient seqlock miss is not a new disarm request.
                    if let Some(request) = requests.latest().filter(|r| r.seq > request_seq) {
                        request_seq = request.seq;
                        armed = request.arm == 1;
                    }
                    let enabled = armed;
                    trace.push_back((frame, qpc(), enabled, command.seq, command.pulses[button]));
                    if trace.len() > 16 {
                        trace.pop_front();
                    }
                    if command.pulses[button] > 0 && !emitted {
                        let now = qpc();
                        actions
                            .publish(&InputActionFrame {
                                command_seq: command.seq,
                                prepared_qpc: now,
                                completed_qpc: now,
                                pulse_mask: 1 << (if matching_edge { button } else { 0 }),
                                ..Default::default()
                            })
                            .unwrap();
                        emitted = true;
                    }
                    let now = qpc();
                    raw.publish(&TelemetryFrame {
                        header: WireHeader {
                            frame_id: frame,
                            qpc: now,
                            ..Default::default()
                        },
                        qpc_frequency: etsads_shm::qpc_frequency(),
                        paused: u32::from(emitted && expects_pause),
                        ..Default::default()
                    })
                    .unwrap();
                    gates
                        .publish(&Gate {
                            heartbeat_qpc: now,
                            enabled: u32::from(enabled && !(emitted && expects_pause)),
                            ..Default::default()
                        })
                        .unwrap();
                    applied
                        .publish(&AppliedControl {
                            applied_qpc: now,
                            applied_seq: command.seq,
                            frame_index: frame,
                            reason: if enabled { 0 } else { 1 },
                            ..Default::default()
                        })
                        .unwrap();
                    if frame == 1 {
                        ready_tx.send(()).unwrap();
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
                (emitted, trace)
            });
            ready_rx.recv().unwrap();
            let stop = Arc::new(AtomicBool::new(false));
            let result = crate::run_manual_command(
                crate::ManualCommand {
                    steer: 0.0,
                    throttle: 0.0,
                    brake: 0.0,
                    seconds: 1.0,
                    pulse: Some(button),
                    arm: true,
                },
                &stop,
                &suffix,
            );
            fixture_stop.store(true, Ordering::Release);
            let (emitted, trace) = worker.join().unwrap();
            assert!(
                emitted,
                "fixture never reached {pulse_name}: {result:?}; producer trace={trace:?}"
            );
            if matching_edge {
                assert!(result.is_ok(), "{result:?}");
            } else {
                let expected = if expects_pause {
                    "game paused"
                } else {
                    "never acknowledged"
                };
                assert!(result.unwrap_err().to_string().contains(expected));
            }
        }
    }
}
