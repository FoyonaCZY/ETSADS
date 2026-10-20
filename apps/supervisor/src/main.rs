use clap::Parser;
use etsads_msgs::protocol::*;
use etsads_safety::{SafetyGeometryProfile, SafetyInputs, Supervisor};
use etsads_shm::{SharedCell, SharedRing, qpc, qpc_frequency};
use sha2::{Digest, Sha256};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

// A single-slot publisher can be observed midway through its seqlock write.
// Retry only an unavailable observation, for at most 2 ms; never substitute a
// cached command, ignore a coherent invalid value, or extend its freshness.
fn coherent_read<T>(mut read: impl FnMut() -> Option<T>) -> Option<T> {
    let start = std::time::Instant::now();
    loop {
        if let Some(value) = read() {
            return Some(value);
        }
        if start.elapsed() >= Duration::from_millis(2) {
            return None;
        }
        std::thread::yield_now();
    }
}

/// Left-stick deflection (of 32767) and trigger travel (of 255) that count
/// as the driver steering or pedalling: above the game's 0.16 stick dead
/// zone, so a resting or slightly worn stick never takes over.
const PAD_STICK_TAKEOVER: i32 = 8192;
const PAD_TRIGGER_TAKEOVER: u8 = 64;
/// Both stick buttons pressed together: the gamepad's F8.
const PAD_TOGGLE: u16 = 0x0040 | 0x0080;
/// Whether a gamepad state means the driver is steering or using the pedals.
fn pad_takes_over(stick_x: i16, left_trigger: u8, right_trigger: u8) -> bool {
    i32::from(stick_x).abs() >= PAD_STICK_TAKEOVER
        || left_trigger >= PAD_TRIGGER_TAKEOVER
        || right_trigger >= PAD_TRIGGER_TAKEOVER
}

/// XInput gamepads (Xbox-compatible), polled on their own thread: querying
/// an empty slot can block for milliseconds, which the safety loop must not
/// wait on. Slots without a controller are rescanned once a second.
#[cfg(windows)]
struct Gamepads {
    takeover: Arc<AtomicBool>,
    toggles: Arc<std::sync::atomic::AtomicU32>,
    seen_toggles: u32,
}
#[cfg(windows)]
impl Default for Gamepads {
    fn default() -> Self {
        let takeover = Arc::new(AtomicBool::new(false));
        let toggles = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let (t, n) = (takeover.clone(), toggles.clone());
        std::thread::spawn(move || {
            use windows_sys::Win32::UI::Input::XboxController::{XINPUT_STATE, XInputGetState};
            let mut connected = [false; 4];
            let mut scanned = None::<std::time::Instant>;
            let mut held = false;
            loop {
                let rescan = scanned.is_none_or(|s| s.elapsed() >= Duration::from_secs(1));
                if rescan {
                    scanned = Some(std::time::Instant::now());
                }
                let (mut takeover, mut chord) = (false, false);
                for (slot, connected) in connected.iter_mut().enumerate() {
                    if !*connected && !rescan {
                        continue;
                    }
                    let mut state: XINPUT_STATE = unsafe { std::mem::zeroed() };
                    *connected = unsafe { XInputGetState(slot as u32, &mut state) } == 0;
                    if *connected {
                        let pad = state.Gamepad;
                        takeover |=
                            pad_takes_over(pad.sThumbLX, pad.bLeftTrigger, pad.bRightTrigger);
                        chord |= pad.wButtons & PAD_TOGGLE == PAD_TOGGLE;
                    }
                }
                t.store(takeover, Ordering::Release);
                if chord && !held {
                    n.fetch_add(1, Ordering::AcqRel);
                }
                held = chord;
                std::thread::sleep(Duration::from_millis(8));
            }
        });
        Self {
            takeover,
            toggles,
            seen_toggles: 0,
        }
    }
}
#[cfg(windows)]
impl Gamepads {
    /// (takeover, toggle pressed since the last call)
    fn poll(&mut self, no_toggle: bool) -> (bool, bool) {
        let toggles = self.toggles.load(Ordering::Acquire);
        let pressed = toggles != self.seen_toggles && !no_toggle;
        self.seen_toggles = toggles;
        (self.takeover.load(Ordering::Acquire), pressed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_resting_stick_does_not_take_over_but_steering_or_pedals_do() {
        // Within the game's 0.16 dead zone and a little beyond.
        assert!(!pad_takes_over(0, 0, 0));
        assert!(!pad_takes_over(5000, 0, 0));
        assert!(!pad_takes_over(-8000, 30, 30));
        assert!(pad_takes_over(-12000, 0, 0));
        assert!(pad_takes_over(9000, 0, 0));
        assert!(pad_takes_over(0, 90, 0));
        assert!(pad_takes_over(0, 0, 255));
    }
    #[test]
    fn publication_retry_returns_first_coherent_value_including_invalid_or_off() {
        let mut calls = 0;
        assert_eq!(
            coherent_read(|| {
                calls += 1;
                if calls < 3 { None } else { Some(0_u32) }
            }),
            Some(0)
        );
        assert_eq!(calls, 3);
        assert_eq!(
            coherent_read(|| Some(Gate {
                enabled: GATE_OFF,
                ..Default::default()
            }))
            .unwrap()
            .enabled,
            GATE_OFF
        );
        assert!(coherent_read::<ControlCmd>(|| None).is_none());
    }
}

#[derive(Parser)]
#[command(about = "Independent ETSADS safety supervisor. Starts disarmed; F8 arms/disarms.")]
struct Args {
    #[arg(long)]
    arm: bool,
    #[arg(long)]
    duration: Option<u64>,
    /// Isolated mapping suffix for automated transport/safety tests.
    #[arg(long, default_value = "")]
    namespace: String,
    /// Headless regression only; manual keyboard checks remain enabled.
    #[arg(long)]
    no_hotkey: bool,
    /// Independently verified raw-geometry profile; absent means reverse throttle is prohibited.
    #[arg(long)]
    safety_profile: Option<std::path::PathBuf>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let mut profile_sha256 = [0_u8; 32];
    let safety_profile = if let Some(path) = &args.safety_profile {
        anyhow::ensure!(
            std::fs::metadata(path)?.len() <= 1_000_000,
            "safety profile exceeds 1 MB budget"
        );
        // Hash the same bytes we parse: a later file replacement cannot claim
        // the running supervisor loaded a different profile.
        let bytes = std::fs::read(path)?;
        let profile: SafetyGeometryProfile = serde_json::from_slice(&bytes)?;
        profile_sha256 = Sha256::digest(&bytes).into();
        anyhow::ensure!(
            profile.schema_version == 1 && profile.validated_for_reverse,
            "safety profile must have verified schema 1 reverse geometry"
        );
        let evidence = std::path::Path::new(&profile.evidence_path);
        let evidence = if evidence.is_absolute() {
            evidence.to_path_buf()
        } else {
            path.parent()
                .unwrap_or(std::path::Path::new("."))
                .join(evidence)
        };
        anyhow::ensure!(
            evidence.is_file(),
            "safety profile evidence file does not exist: {}",
            evidence.display()
        );
        Some(profile)
    } else {
        None
    };
    anyhow::ensure!(
        args.namespace
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_'),
        "namespace must be alphanumeric"
    );
    let name = |base: &str| {
        if args.namespace.is_empty() {
            base.to_owned()
        } else {
            format!("{base}_{}", args.namespace)
        }
    };
    let _writer_guard = etsads_shm::SingleWriterGuard::acquire(&name("ETSADS_Supervisor_Writer"))?;
    let mut policy = Supervisor::default();
    if args.arm {
        policy.arm();
    }
    let gate = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION)?;
    let status =
        SharedCell::<SupervisorStatus>::create(&name(SUPERVISOR_STATUS_MAPPING), SCHEMA_VERSION)?;
    status.publish(&SupervisorStatus {
        schema_version: 1,
        process_id: std::process::id(),
        heartbeat_qpc: qpc(),
        profile_sha256,
        ..Default::default()
    })?;
    let requests =
        SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)?;
    let mut request_seq = requests.latest().map_or(0, |r| r.seq);
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false, Ordering::Relaxed))?;
    #[cfg(windows)]
    unsafe {
        use windows_sys::Win32::System::Threading::*;
        SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    }
    eprintln!(
        "Safety supervisor running. F8 or both gamepad sticks pressed = arm/disarm; arrows/WASD/Space or gamepad stick/triggers = release. Initial arm={}",
        args.arm
    );
    let start = std::time::Instant::now();
    let mut last_reason = String::new();
    let mut telemetry = None;
    let mut command = None;
    let mut extension = None;
    // Geometry only: the assist's steered path, used to judge raw traffic
    // along the road ahead (the supervisor falls back to its yaw-rate arc).
    let mut planned = None;
    let mut configuration = None;
    let mut controls = None;
    let mut bridge_info = None;
    #[cfg(windows)]
    let mut hotkey_held = false;
    #[cfg(windows)]
    let mut gamepads = Gamepads::default();
    while running.load(Ordering::Relaxed)
        && args.duration.is_none_or(|s| start.elapsed().as_secs() < s)
    {
        if let Some(r) = requests.latest().filter(|r| r.seq > request_seq) {
            request_seq = r.seq;
            if etsads_safety::fresh(qpc(), r.write_qpc, qpc_frequency(), 1000) {
                if r.arm == 1 {
                    policy.arm();
                } else {
                    policy.disarm();
                }
            }
        }
        if telemetry.is_none() {
            telemetry = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
                &name(TELEMETRY_MAPPING),
                SCHEMA_VERSION,
            )
            .ok();
        }
        if command.is_none() {
            command = SharedCell::<ControlCmd>::open(&name(CONTROL_MAPPING), SCHEMA_VERSION).ok();
        }
        if extension.is_none() {
            extension =
                SharedRing::<ExtensionFrame, 4>::open(&name(EXTENSION_MAPPING), SCHEMA_VERSION)
                    .ok();
        }
        if planned.is_none() {
            planned =
                SharedRing::<PlannedPath, 2>::open(&name(PLANNED_PATH_MAPPING), SCHEMA_VERSION)
                    .ok();
        }
        if configuration.is_none() {
            configuration =
                SharedCell::<ConfigBlock>::open(&name(CONFIG_MAPPING), SCHEMA_VERSION).ok();
        }
        if controls.is_none() {
            controls = SharedRing::<ControlConfiguration, 64>::open(
                &name(CONTROL_CONFIGURATION_MAPPING),
                SCHEMA_VERSION,
            )
            .ok();
        }
        if bridge_info.is_none() {
            bridge_info = SharedCell::<BridgeInfo>::open(&name(INFO_MAPPING), SCHEMA_VERSION).ok();
        }
        #[cfg(windows)]
        let manual = unsafe {
            use windows_sys::Win32::UI::Input::KeyboardAndMouse::*;
            // The toggle is F8 alone, polled. ETS2 binds LShift/LCtrl to gear
            // up/down, so the former Ctrl+Shift+F12 chord knocked an automatic
            // into neutral (r9); its default profile binds no F8. A registered
            // hotkey is not used: games that read raw keyboard input suppress
            // WM_HOTKEY while focused, and it would steal F8 system-wide.
            let chord = !args.no_hotkey && GetAsyncKeyState(VK_F8 as i32) < 0;
            if chord && !hotkey_held {
                if policy.is_armed() || policy.is_braking_after_loss() {
                    policy.disarm();
                } else {
                    policy.arm();
                }
            }
            hotkey_held = chord;
            [
                // Esc is the game's menu key: a pause is held, not a takeover.
                VK_LEFT, VK_RIGHT, VK_UP, VK_DOWN, VK_SPACE, 0x57, 0x41, 0x53, 0x44,
            ]
            .iter()
            .any(|key| GetAsyncKeyState(*key as i32) < 0)
        };
        // A gamepad steers and drives as the keyboard does: its stick or
        // triggers take over, both stick buttons together toggle like F8.
        // r14: a player's stick fought the assist, which only knew keys.
        #[cfg(windows)]
        let manual = {
            let (takeover, toggle) = gamepads.poll(args.no_hotkey);
            if toggle {
                if policy.is_armed() || policy.is_braking_after_loss() {
                    policy.disarm();
                } else {
                    policy.arm();
                }
            }
            manual || takeover
        };
        #[cfg(not(windows))]
        let manual = false;
        let c = command.as_ref().and_then(|r| coherent_read(|| r.latest()));
        let raw_extension = extension
            .as_ref()
            .and_then(|r| coherent_read(|| r.latest_boxed()));
        let config = configuration
            .as_ref()
            .and_then(|r| coherent_read(|| r.latest()));
        let loaded_controls = controls.as_ref().and_then(|r| coherent_read(|| r.latest()));
        let session = bridge_info
            .as_ref()
            .and_then(|r| coherent_read(|| r.latest()))
            .filter(|i| i.telemetry_api_version != 0 && i.input_api_version != 0)
            .map_or(0, |i| i.session_start_qpc);
        policy.observe_control_configuration(loaded_controls, session);
        // Read commands before telemetry so a newly published command cannot
        // appear to reference a future physics frame from an earlier read.
        let mut t = telemetry.as_ref().and_then(|r| r.latest());
        if c.as_ref()
            .zip(t.as_ref())
            .is_some_and(|(c, t)| c.source_frame_id > t.header.frame_id)
        {
            t = telemetry.as_ref().and_then(|r| r.latest());
        }
        let now = qpc();
        let planned_path = planned.as_ref().and_then(|r| coherent_read(|| r.latest()));
        let obstacle = t.as_ref().zip(raw_extension.as_deref()).map_or(
            Err("traffic source missing"),
            |(t, e)| {
                if !etsads_safety::fresh(now, e.header.qpc, qpc_frequency(), 200) {
                    return Err("raw traffic capture is stale");
                }
                match (&safety_profile, &config) {
                    (Some(profile), Some(config)) => {
                        etsads_safety::reverse::swept_obstacle(t, e, config, profile)
                    }
                    (Some(_), None) => Err("independent safety profile requires raw /config"),
                    _ => etsads_safety::primary_obstacle_along(t, e, planned_path.as_ref()),
                }
            },
        );
        if obstacle.is_err() && t.as_ref().is_some_and(|t| t.speed_mps > 1.0 && t.gear >= 0) {
            policy.disarm();
        }
        let (mut g, reason) = policy.evaluate_profiled(
            now,
            qpc_frequency(),
            SafetyInputs {
                telemetry: t.as_ref(),
                command: c.as_ref(),
                configuration: config.as_ref(),
                profile: safety_profile.as_ref(),
                manual,
                obstacle: obstacle.as_ref().ok().copied().flatten(),
            },
        );
        if obstacle.is_err() {
            g.max_throttle = 0.0;
            if g.enabled == GATE_BRAKE_ONLY {
                // BrakeOnly is a bounded, explicit fallback, not permission
                // to override missing raw safety data with stronger commands.
                policy.disarm();
                g.enabled = GATE_OFF;
                g.aeb_brake = 0.0;
            } else if g.enabled == GATE_ACTIVE
                && t.as_ref()
                    .is_some_and(|t| (t.gear < 0 || t.speed_mps < 0.0) && t.speed_mps.abs() > 0.05)
            {
                g.aeb_brake = 1.0;
            }
        }
        gate.publish(&g)?;
        let reverse_ready = t
            .as_ref()
            .zip(config.as_ref())
            .zip(safety_profile.as_ref())
            .is_some_and(|((t, config), profile)| {
                etsads_safety::fresh(now, t.header.qpc, qpc_frequency(), 200)
                    && etsads_safety::validate_control_configuration(
                        loaded_controls.as_ref(),
                        session,
                        t,
                        now,
                    )
                    .is_ok()
                    && policy.reverse_profile_ready(t, config, profile)
            });
        status.publish(&SupervisorStatus {
            schema_version: 1,
            process_id: std::process::id(),
            heartbeat_qpc: now,
            profile_sha256,
            configuration_sequence: config.as_ref().map_or(0, |c| c.config_seq),
            reverse_ready: u32::from(reverse_ready),
            reserved: 0,
        })?;
        let reason = format!("{reason:?}; traffic={:?}", obstacle.err());
        if reason != last_reason {
            let unix_ms = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_millis());
            eprintln!("{unix_ms} gate={} reason={}", g.enabled, reason);
            last_reason = reason;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    gate.publish(&Gate {
        heartbeat_qpc: qpc(),
        ..Default::default()
    })?;
    status.publish(&SupervisorStatus {
        schema_version: 1,
        process_id: std::process::id(),
        heartbeat_qpc: qpc(),
        profile_sha256,
        ..Default::default()
    })?;
    #[cfg(windows)]
    unsafe {
        windows_sys::Win32::UI::Input::KeyboardAndMouse::UnregisterHotKey(
            std::ptr::null_mut(),
            0x4554,
        );
    }
    Ok(())
}
