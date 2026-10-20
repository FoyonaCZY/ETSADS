//! Official SCS telemetry and semantic-input bridge.
#![allow(non_upper_case_globals)] // Official SDK constant spelling is retained.
mod configuration;
use etsads_game::input::{
    pulse_requires_stationary, InputPolicy, AXIS_NAMES, HOLD_NAMES, INPUT_COUNT, PULSE_INPUT_START,
    PULSE_NAMES, REASON_ACTIVE,
};
use etsads_msgs::protocol::*;
use etsads_sdk_sys::*;
use etsads_shm::{qpc, qpc_frequency, Mapping, SharedCell, SharedRing};
use std::{
    collections::VecDeque,
    ffi::{c_void, CStr, CString},
    io::Write,
    panic::{catch_unwind, AssertUnwindSafe},
    ptr,
    sync::Mutex,
};

static STATE: Mutex<Option<State>> = Mutex::new(None);
fn mapping_name(name: &str) -> String {
    #[cfg(test)]
    {
        format!("{name}_BridgeTest_{}", std::process::id())
    }
    #[cfg(not(test))]
    {
        name.to_owned()
    }
}
struct Telemetry {
    ring: SharedRing<TelemetryFrame, 64>,
    extension: SharedRing<ExtensionFrame, 4>,
    config_map: SharedCell<ConfigBlock>,
    counters_map: SharedCell<VehicleCounters>,
    counters: VehicleCounters,
    events: SharedRing<GameEvent, 256>,
    frame: TelemetryFrame,
    config: ConfigBlock,
    dynamics_map: SharedRing<DynamicsFrame, 64>,
    dynamics: DynamicsFrame,
    config_full_map: SharedRing<ConfigurationFrame, 64>,
    config_full: ConfigurationFrame,
    config_history: VecDeque<ConfigurationFrame>,
    camera_view_map: SharedRing<CameraViewFrame, 64>,
    camera_tracker: etsads_game::camera::CameraViewTracker,
    session_start_qpc: u64,
    extensions: [Option<Mapping>; 5],
    extension_buffer: Box<ExtensionFrame>,
    extension_compatible: bool,
}
struct Input {
    control: SharedCell<ControlCmd>,
    gate: SharedCell<Gate>,
    applied: SharedCell<AppliedControl>,
    diagnostics_map: SharedCell<InputDiagnostics>,
    diagnostics: InputDiagnostics,
    action_map: SharedRing<InputActionFrame, 256>,
    batch_action: InputActionFrame,
    batch_active: bool,
    batch_callbacks: u32,
    policy: InputPolicy,
    controls_rearm_required: bool,
    controls_seen_off: bool,
    last_values: [f32; INPUT_COUNT],
    pulses: [u32; 64],
    pending: VecDeque<scs_input_event_t>,
    frame: u64,
}
struct State {
    telemetry: Option<Telemetry>,
    input: Option<Input>,
    info_map: SharedCell<BridgeInfo>,
    info: BridgeInfo,
    timing_map: SharedRing<FrameTiming, 256>,
    timing: FrameTiming,
    completed_physics: (u64, u64),
    controls_map: SharedRing<ControlConfiguration, 64>,
    controls: ControlConfiguration,
}
fn initialize_state<'a>(
    state: &'a mut Option<State>,
    common: &scs_sdk_init_params_v100_t,
) -> etsads_shm::Result<&'a mut State> {
    if state.is_none() {
        let info_map = SharedCell::create(&mapping_name(INFO_MAPPING), SCHEMA_VERSION)?;
        let mut info = BridgeInfo {
            schema_version: SCHEMA_VERSION,
            process_id: std::process::id(),
            session_start_qpc: qpc(),
            ..Default::default()
        };
        copy_string(common.game_id, &mut info.game_id);
        copy_string(common.game_name, &mut info.game_name);
        info_map.publish(&info)?;
        let controls_map =
            SharedRing::create(&mapping_name(CONTROL_CONFIGURATION_MAPPING), SCHEMA_VERSION)?;
        let controls = ControlConfiguration {
            header: WireHeader {
                qpc: info.session_start_qpc,
                ..Default::default()
            },
            session_start_qpc: info.session_start_qpc,
            effective_from_frame_id: 1,
            ..Default::default()
        };
        controls_map.publish(&controls)?;
        *state = Some(State {
            telemetry: None,
            input: None,
            info_map,
            info,
            timing_map: SharedRing::create(&mapping_name(FRAME_TIMING_MAPPING), SCHEMA_VERSION)?,
            timing: FrameTiming::default(),
            completed_physics: (0, 0),
            controls_map,
            controls,
        });
    }
    Ok(state.as_mut().expect("initialized state"))
}
fn publish_timing(
    timing: &mut FrameTiming,
    map: &SharedRing<FrameTiming, 256>,
    kind: u32,
    qpc: u64,
) {
    timing.event_seq = timing.event_seq.wrapping_add(1);
    timing.kind = kind;
    timing.qpc = qpc;
    let _ = map.publish(timing);
}
impl Telemetry {
    fn new(session_start_qpc: u64) -> etsads_shm::Result<Self> {
        Ok(Self {
            ring: SharedRing::create(&mapping_name(TELEMETRY_MAPPING), SCHEMA_VERSION)?,
            extension: SharedRing::create(&mapping_name(EXTENSION_MAPPING), SCHEMA_VERSION)?,
            config_map: SharedCell::create(&mapping_name(CONFIG_MAPPING), SCHEMA_VERSION)?,
            counters_map: SharedCell::create(
                &mapping_name(VEHICLE_COUNTERS_MAPPING),
                SCHEMA_VERSION,
            )?,
            counters: VehicleCounters::default(),
            events: SharedRing::create(&mapping_name(EVENTS_MAPPING), SCHEMA_VERSION)?,
            frame: TelemetryFrame {
                qpc_frequency: qpc_frequency(),
                paused: 1,
                ..Default::default()
            },
            config: ConfigBlock::default(),
            dynamics_map: SharedRing::create(&mapping_name(DYNAMICS_MAPPING), SCHEMA_VERSION)?,
            dynamics: DynamicsFrame::default(),
            config_full_map: SharedRing::create(
                &mapping_name(CONFIG_FULL_MAPPING),
                SCHEMA_VERSION,
            )?,
            config_full: ConfigurationFrame::default(),
            config_history: VecDeque::with_capacity(64),
            camera_view_map: SharedRing::create(
                &mapping_name(CAMERA_VIEW_MAPPING),
                SCHEMA_VERSION,
            )?,
            camera_tracker: Default::default(),
            session_start_qpc,
            extensions: std::array::from_fn(|_| None),
            extension_buffer: etsads_shm::zeroed_box(),
            extension_compatible: etsads_game::version::current_process_version()
                .is_some_and(|v| etsads_game::version::extension_version_supported(&v)),
        })
    }
}
impl Input {
    fn invalidate_controls(&mut self) {
        self.policy = InputPolicy::default();
        self.pending.clear();
        self.batch_active = false;
        self.controls_rearm_required = true;
        self.controls_seen_off = false;
        // Preserve last_values so the next SDK batch emits actual zero edges.
        // Discard all accumulated presses from the previous controls mode.
        if let Some(command) = self.control.latest() {
            self.pulses = command.pulses;
        }
    }
    fn controls_authorized(
        &mut self,
        controls: ControlConfiguration,
        session: u64,
        gate: Option<&Gate>,
        now: u64,
    ) -> bool {
        let current_gate = gate.filter(|g| {
            g.heartbeat_qpc > 0
                && g.heartbeat_qpc <= now
                && (now - g.heartbeat_qpc) as u128 * 1000 <= qpc_frequency() as u128 * 300
        });
        if current_gate.is_some_and(|g| g.enabled == GATE_OFF) {
            self.controls_seen_off = true;
        }
        if self.controls_rearm_required
            && self.controls_seen_off
            && current_gate.is_some_and(|g| g.enabled == GATE_ACTIVE)
        {
            self.controls_rearm_required = false;
        }
        controls.safe_shifter()
            && controls.generation > 0
            && controls.session_start_qpc == session
            && controls.header.qpc >= session
            && controls.header.qpc <= now
            && !self.controls_rearm_required
    }
    fn new() -> etsads_shm::Result<Self> {
        Ok(Self {
            control: SharedCell::create(&mapping_name(CONTROL_MAPPING), SCHEMA_VERSION)?,
            gate: SharedCell::create(&mapping_name(GATE_MAPPING), SCHEMA_VERSION)?,
            applied: SharedCell::create(&mapping_name(APPLIED_MAPPING), SCHEMA_VERSION)?,
            diagnostics_map: SharedCell::create(
                &mapping_name(INPUT_DIAGNOSTICS_MAPPING),
                SCHEMA_VERSION,
            )?,
            diagnostics: InputDiagnostics::default(),
            action_map: SharedRing::create(&mapping_name(INPUT_ACTIONS_MAPPING), SCHEMA_VERSION)?,
            batch_action: InputActionFrame::default(),
            batch_active: false,
            batch_callbacks: 0,
            policy: InputPolicy::default(),
            controls_rearm_required: true,
            controls_seen_off: false,
            last_values: [0.0; INPUT_COUNT],
            pulses: [0; 64],
            pending: VecDeque::with_capacity(INPUT_COUNT),
            frame: 0,
        })
    }
}
fn log(log: scs_log_t, kind: i32, message: &str) {
    if let (Some(log), Ok(c)) = (log, CString::new(message)) {
        unsafe {
            log(kind, c.as_ptr());
        }
    }
    if let Some(root) = std::env::var_os("LOCALAPPDATA") {
        let root = std::path::PathBuf::from(root).join("ETSADS");
        let _ = std::fs::create_dir_all(&root);
        if let Ok(mut file) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(root.join(if cfg!(test) {
                "bridge-test.log"
            } else {
                "bridge.log"
            }))
        {
            let _ = writeln!(file, "qpc={} {}", qpc(), message);
        }
    }
}
fn guarded(operation: impl FnOnce() -> i32) -> i32 {
    catch_unwind(AssertUnwindSafe(operation)).unwrap_or_else(|_| {
        log(
            None,
            2,
            "etsads_bridge: contained panic; input fails closed",
        );
        *STATE
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = None;
        STATE.clear_poison();
        SCS_RESULT_generic_error
    })
}
fn callback(operation: impl FnOnce()) {
    let _ = guarded(|| {
        operation();
        SCS_RESULT_ok
    });
}

struct TelemetryRegistration {
    unregister_event: unsafe extern "C" fn(u32) -> i32,
    unregister_channel: unsafe extern "C" fn(*const i8, u32, u32) -> i32,
    events: Vec<u32>,
    channels: Vec<(CString, u32, u32)>,
    committed: bool,
}
impl Drop for TelemetryRegistration {
    fn drop(&mut self) {
        if !self.committed {
            // No STATE lock is held while calling the SDK, including rollback.
            for (name, index, kind) in self.channels.iter().rev() {
                unsafe {
                    (self.unregister_channel)(name.as_ptr(), *index, *kind);
                }
            }
            for event in self.events.iter().rev() {
                unsafe {
                    (self.unregister_event)(*event);
                }
            }
            scs_telemetry_shutdown();
        }
    }
}
struct InputRegistration {
    committed: bool,
}
impl Drop for InputRegistration {
    fn drop(&mut self) {
        if !self.committed {
            scs_input_shutdown();
        }
    }
}

/// # Safety
/// `params` must point to the SDK initialization structure for `version`.
#[no_mangle]
pub unsafe extern "C" fn scs_telemetry_init(
    version: u32,
    params: *const scs_telemetry_init_params_t,
) -> i32 {
    guarded(|| {
        if version != 0x10001 {
            return SCS_RESULT_unsupported;
        }
        if params.is_null() {
            return SCS_RESULT_invalid_parameter;
        }
        let params = unsafe { &*params.cast::<scs_telemetry_init_params_v101_t>() };
        let (
            Some(register_event),
            Some(register_channel),
            Some(unregister_event),
            Some(unregister_channel),
        ) = (
            params.register_for_event,
            params.register_for_channel,
            params.unregister_from_event,
            params.unregister_from_channel,
        )
        else {
            return SCS_RESULT_invalid_parameter;
        };
        let prepare = (|| -> Result<bool, i32> {
            let mut lock = STATE.lock().map_err(|_| SCS_RESULT_generic_error)?;
            let state = initialize_state(&mut lock, &params.common)
                .map_err(|_| SCS_RESULT_generic_error)?;
            if state.telemetry.is_some() {
                return Err(SCS_RESULT_already_registered);
            }
            let telemetry = Telemetry::new(state.info.session_start_qpc)
                .map_err(|_| SCS_RESULT_generic_error)?;
            let compatible = telemetry.extension_compatible;
            state.telemetry = Some(telemetry);
            Ok(compatible)
        })();
        let compatible = match prepare {
            Ok(value) => value,
            Err(code) => {
                if code != SCS_RESULT_already_registered {
                    scs_telemetry_shutdown();
                }
                return code;
            }
        };
        let mut registration = TelemetryRegistration {
            unregister_event,
            unregister_channel,
            events: Vec::new(),
            channels: Vec::new(),
            committed: false,
        };
        if !compatible {
            log(
                params.common.log,
                1,
                "etsads_bridge: extension disabled; executable PE version is not verified 1.61.x",
            );
        }
        for event in 1..=6 {
            let result = unsafe { register_event(event, Some(on_event), ptr::null_mut()) };
            if result != 0 {
                return result;
            }
            registration.events.push(event);
        }
        let mut rejected = 0;
        for (name, kind, index) in subscriptions() {
            let name = CString::new(name).expect("constant channel name");
            let result = unsafe {
                register_channel(
                    name.as_ptr(),
                    index,
                    kind,
                    if supplement_bit(name.as_bytes()).is_some() {
                        SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value
                    } else {
                        SCS_TELEMETRY_CHANNEL_FLAG_none
                    },
                    Some(on_channel),
                    ptr::null_mut(),
                )
            };
            if result == 0 {
                registration.channels.push((name, index, kind));
            } else {
                rejected += 1;
                log(
                    params.common.log,
                    1,
                    &format!(
                        "etsads_bridge: optional channel {}[{index}] rejected {result}",
                        name.to_string_lossy()
                    ),
                );
            }
        }
        {
            let mut lock = match STATE.lock() {
                Ok(lock) => lock,
                Err(_) => return SCS_RESULT_generic_error,
            };
            let Some(state) = lock.as_mut() else {
                return SCS_RESULT_generic_error;
            };
            if state.telemetry.is_none() {
                return SCS_RESULT_generic_error;
            }
            state.info.game_version = params.common.game_version;
            state.info.telemetry_api_version = version;
            if state.info_map.publish(&state.info).is_err() {
                return SCS_RESULT_generic_error;
            }
        }
        registration.committed = true;
        log(
            params.common.log,
            0,
            &format!(
                "etsads_bridge loaded, telemetry 1.01; {} channels; {rejected} unsupported; input available 1.00",
                registration.channels.len()
            ),
        );
        SCS_RESULT_ok
    })
}
#[no_mangle]
pub extern "C" fn scs_telemetry_shutdown() {
    callback(|| {
        if let Ok(mut state) = STATE.lock() {
            if let Some(s) = state.as_mut() {
                s.telemetry = None;
                s.controls = ControlConfiguration {
                    header: WireHeader {
                        frame_id: s.completed_physics.0,
                        sim_time_us: s.completed_physics.1,
                        qpc: qpc(),
                        seq: s.controls.generation.saturating_add(1),
                        ..Default::default()
                    },
                    session_start_qpc: s.info.session_start_qpc,
                    generation: s.controls.generation.saturating_add(1),
                    effective_from_frame_id: s.completed_physics.0.saturating_add(1),
                    ..Default::default()
                };
                let _ = s.controls_map.publish(&s.controls);
                if let Some(input) = s.input.as_mut() {
                    input.invalidate_controls();
                }
                s.info.telemetry_api_version = 0;
                let _ = s.info_map.publish(&s.info);
                if s.input.is_none() {
                    *state = None;
                }
            }
        }
    })
}

/// # Safety
/// `params` must point to the SDK initialization structure for `version`.
#[no_mangle]
pub unsafe extern "C" fn scs_input_init(
    version: u32,
    params: *const scs_input_init_params_t,
) -> i32 {
    guarded(|| {
        if version != 0x10000 {
            return SCS_RESULT_unsupported;
        }
        if params.is_null() {
            return SCS_RESULT_invalid_parameter;
        }
        let params = unsafe { &*params.cast::<scs_input_init_params_v100_t>() };
        let Some(register) = params.register_device else {
            return SCS_RESULT_invalid_parameter;
        };
        let prepare = (|| -> Result<(), i32> {
            let mut lock = STATE.lock().map_err(|_| SCS_RESULT_generic_error)?;
            let state = initialize_state(&mut lock, &params.common)
                .map_err(|_| SCS_RESULT_generic_error)?;
            if state.input.is_some() {
                return Err(SCS_RESULT_already_registered);
            }
            state.input = Some(Input::new().map_err(|_| SCS_RESULT_generic_error)?);
            Ok(())
        })();
        if let Err(code) = prepare {
            if code != SCS_RESULT_already_registered {
                scs_input_shutdown();
            }
            return code;
        }
        let mut registration = InputRegistration { committed: false };
        let names: Vec<CString> = AXIS_NAMES
            .iter()
            .chain(HOLD_NAMES.iter())
            .chain(PULSE_NAMES.iter())
            .map(|name| CString::new(*name).unwrap())
            .collect();
        let descriptors: Vec<_> = names
            .iter()
            .enumerate()
            .map(|(index, name)| scs_input_device_input_t {
                name: name.as_ptr(),
                display_name: name.as_ptr(),
                value_type: if index < AXIS_NAMES.len() {
                    SCS_VALUE_TYPE_float
                } else {
                    SCS_VALUE_TYPE_bool
                },
                _padding: 0,
            })
            .collect();
        let device = scs_input_device_t {
            name: c"etsads".as_ptr(),
            display_name: c"ETSADS".as_ptr(),
            type_: SCS_INPUT_DEVICE_TYPE_semantical,
            input_count: descriptors.len() as u32,
            inputs: descriptors.as_ptr(),
            callback_context: ptr::null_mut(),
            input_active_callback: Some(on_active),
            input_event_callback: Some(on_input),
        };
        // The complete callback-visible Input exists before this external call.
        // The SDK is free to synchronously invoke active/input callbacks here.
        let result = unsafe { register(&device) };
        if result != 0 {
            return result;
        }
        {
            let mut lock = match STATE.lock() {
                Ok(lock) => lock,
                Err(_) => return SCS_RESULT_generic_error,
            };
            let Some(state) = lock.as_mut() else {
                return SCS_RESULT_generic_error;
            };
            if state.input.is_none() {
                return SCS_RESULT_generic_error;
            }
            state.info.input_game_version = params.common.game_version;
            state.info.input_api_version = version;
            if state.info_map.publish(&state.info).is_err() {
                return SCS_RESULT_generic_error;
            }
        }
        registration.committed = true;
        log(
            params.common.log,
            0,
            "etsads_bridge loaded, input 1.00, semantic device etsads; gate modes 0=off 1=active 2=brake-only; disabled by default",
        );
        SCS_RESULT_ok
    })
}
#[no_mangle]
pub extern "C" fn scs_input_shutdown() {
    callback(|| {
        if let Ok(mut state) = STATE.lock() {
            if let Some(s) = state.as_mut() {
                s.input = None;
                s.info.input_api_version = 0;
                let _ = s.info_map.publish(&s.info);
                if s.telemetry.is_none() {
                    *state = None;
                }
            }
        }
    })
}
unsafe extern "C" fn on_active(active: u8, _context: *mut c_void) {
    callback(|| {
        if active == 0 {
            if let Ok(mut state) = STATE.lock() {
                if let Some(input) = state.as_mut().and_then(|s| s.input.as_mut()) {
                    input.policy = InputPolicy::default();
                    input.pending.clear();
                    input.batch_active = false;
                    input.last_values.fill(0.0);
                }
            }
        }
    })
}

unsafe extern "C" fn on_input(
    out: *mut scs_input_event_t,
    flags: u32,
    _context: *mut c_void,
) -> i32 {
    guarded(|| {
        if out.is_null() {
            return SCS_RESULT_invalid_parameter;
        }
        let mut state = match STATE.lock() {
            Ok(x) => x,
            Err(_) => return SCS_RESULT_generic_error,
        };
        let Some(state) = state.as_mut() else {
            return SCS_RESULT_generic_error;
        };
        let paused = state.telemetry.as_ref().is_none_or(|t| t.frame.paused != 0);
        let stationary = state
            .telemetry
            .as_ref()
            .is_some_and(|t| t.frame.speed_mps.is_finite() && t.frame.speed_mps.abs() <= 0.05);
        let physical = state.completed_physics;
        let controls = state.controls;
        let session = state.info.session_start_qpc;
        let Some(input) = state.input.as_mut() else {
            return SCS_RESULT_generic_error;
        };
        input.diagnostics.callback_count = input.diagnostics.callback_count.wrapping_add(1);
        input.diagnostics.last_flags = flags;
        if input.batch_active && flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame != 0 {
            input.diagnostics.repeated_first_flags += 1;
        }
        if !input.batch_active && flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame != 0 {
            input.batch_active = true;
            input.batch_callbacks = 0;
            input.pending.clear();
            input.frame += 1;
            let command = input.control.latest();
            let gate = input.gate.latest();
            let now = qpc();
            input.diagnostics.batch_count += 1;
            input.diagnostics.last_begin_qpc = now;
            input.diagnostics.first_flags = flags;
            state.timing.physics_frame_id = physical.0;
            state.timing.sim_time_us = physical.1;
            state.timing.input_batch_id = input.frame;
            state.timing.input_prepare_qpc = now;
            state.timing.command_seq = command.map_or(0, |c| c.seq);
            state.timing.source_frame_id = command.map_or(0, |c| c.source_frame_id);
            publish_timing(&mut state.timing, &state.timing_map, 3, now);
            let allowed = input.controls_authorized(controls, session, gate.as_ref(), now);
            let applied = if allowed {
                input.policy.apply(
                    command.as_ref(),
                    gate.as_ref(),
                    now,
                    qpc_frequency(),
                    paused,
                )
            } else {
                input.policy = InputPolicy::default();
                etsads_game::input::Applied {
                    reason: if paused {
                        etsads_game::input::REASON_PAUSED
                    } else {
                        etsads_game::input::REASON_CONTROL_CONFIGURATION
                    },
                    ..Default::default()
                }
            };
            let force = flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_after_activation != 0;
            let mut values = [0_f32; INPUT_COUNT];
            values[..AXIS_NAMES.len()].copy_from_slice(&applied.axes);
            for i in 0..HOLD_NAMES.len() {
                values[AXIS_NAMES.len() + i] = ((applied.holds >> i) & 1) as f32;
            }
            if let Some(command) = &command {
                for i in 0..PULSE_NAMES.len() {
                    if applied.reason != REASON_ACTIVE
                        || force
                        || (pulse_requires_stationary(i) && !stationary)
                    {
                        input.pulses[i] = command.pulses[i];
                    } else if input.last_values[PULSE_INPUT_START + i] == 0.0
                        && input.pulses[i] != command.pulses[i]
                    {
                        values[PULSE_INPUT_START + i] = 1.0;
                        input.pulses[i] = input.pulses[i].wrapping_add(1);
                    }
                }
            }
            for (i, value) in values.iter().copied().enumerate() {
                if force || value != input.last_values[i] {
                    let mut event = scs_input_event_t {
                        input_index: i as u32,
                        ..Default::default()
                    };
                    if i < AXIS_NAMES.len() {
                        event.__bindgen_anon_1.value_float = scs_value_float_t { value };
                    } else {
                        event.__bindgen_anon_1.value_bool = scs_value_bool_t {
                            value: (value != 0.0) as u8,
                        };
                    }
                    input.pending.push_back(event);
                }
            }
            input.last_values = values;
            input.batch_action = InputActionFrame {
                batch_id: input.frame,
                command_seq: command.map_or(0, |c| c.seq),
                prepared_qpc: now,
                completed_qpc: 0,
                source_frame_id: command.map_or(0, |c| c.source_frame_id),
                holds: applied.holds,
                pulse_mask: values[PULSE_INPUT_START..]
                    .iter()
                    .enumerate()
                    .fold(0u64, |mask, (i, value)| {
                        mask | u64::from(*value != 0.0) << i
                    }),
                reason: applied.reason,
                reserved: 0,
                pulse_counters: input.pulses,
            };
            input.diagnostics.last_batch_events = input.pending.len() as u32;
            input.diagnostics.max_batch_events = input
                .diagnostics
                .max_batch_events
                .max(input.pending.len() as u32);
            let _ = input.applied.publish(&AppliedControl {
                applied_seq: command.map(|c| c.seq).unwrap_or(0),
                applied_qpc: now,
                frame_index: input.frame,
                axes: applied.axes,
                reason: applied.reason,
                reserved: 0,
            });
        }
        input.batch_callbacks = input.batch_callbacks.saturating_add(1);
        if let Some(event) = input.pending.pop_front() {
            unsafe { ptr::write(out, event) };
            SCS_RESULT_ok
        } else {
            if input.batch_active {
                input.diagnostics.completed_batch_count += 1;
                input.diagnostics.last_end_qpc = qpc();
                input.diagnostics.last_batch_callbacks = input.batch_callbacks;
                input.batch_action.completed_qpc = input.diagnostics.last_end_qpc;
                let _ = input.action_map.publish(&input.batch_action);
            }
            input.batch_active = false;
            let _ = input.diagnostics_map.publish(&input.diagnostics);
            SCS_RESULT_not_found
        }
    })
}

unsafe extern "C" fn on_event(event: u32, info: *const c_void, _context: *mut c_void) {
    callback(|| {
        let Ok(mut lock) = STATE.lock() else { return };
        let Some(state) = lock.as_mut() else { return };
        let Some(t) = state.telemetry.as_mut() else {
            return;
        };
        match event {
            SCS_TELEMETRY_EVENT_frame_start if !info.is_null() => {
                let start = unsafe { &*info.cast::<scs_telemetry_frame_start_t>() };
                let timestamp = qpc();
                state.timing.physics_frame_id = t.frame.header.frame_id + 1;
                state.timing.sim_time_us = start.simulation_time;
                state.timing.frame_start_qpc = timestamp;
                state.timing.frame_end_qpc = 0;
                publish_timing(&mut state.timing, &state.timing_map, 1, timestamp);
                copy_extensions(t);
                t.dynamics.valid = 0;
                t.frame.header.sim_time_us = start.simulation_time;
                t.frame.header.flags = start.flags;
                t.frame.header.qpc = timestamp;
                t.frame.render_time_us = start.render_time;
                t.frame.paused_simulation_time_us = start.paused_simulation_time;
            }
            SCS_TELEMETRY_EVENT_frame_end => {
                let timestamp = qpc();
                t.frame.header.frame_id += 1;
                t.frame.header.seq += 1;
                let _ = t.ring.publish(&t.frame);
                t.counters.header = t.frame.header;
                let _ = t.counters_map.publish(&t.counters);
                t.dynamics.header = t.frame.header;
                let _ = t.dynamics_map.publish(&t.dynamics);
                state.timing.physics_frame_id = t.frame.header.frame_id;
                state.timing.sim_time_us = t.frame.header.sim_time_us;
                state.completed_physics = (t.frame.header.frame_id, t.frame.header.sim_time_us);
                state.timing.frame_end_qpc = timestamp;
                publish_timing(&mut state.timing, &state.timing_map, 2, timestamp);
            }
            SCS_TELEMETRY_EVENT_paused => {
                t.frame.paused = 1;
                // A pause can interrupt an already prepared finite batch. Drop
                // queued presses immediately and require a new Off -> Active.
                if let Some(input) = state.input.as_mut() {
                    input.invalidate_controls();
                }
            }
            SCS_TELEMETRY_EVENT_started => t.frame.paused = 0,
            SCS_TELEMETRY_EVENT_configuration if !info.is_null() => {
                let config = unsafe { &*info.cast::<scs_telemetry_configuration_t>() };
                if config.id.is_null() {
                    return;
                }
                if unsafe { CStr::from_ptr(config.id) }.to_bytes() == b"controls" {
                    let header = WireHeader {
                        frame_id: state.completed_physics.0,
                        sim_time_us: state.completed_physics.1,
                        qpc: qpc(),
                        ..Default::default()
                    };
                    state.controls = configuration::controls(state.controls, config, header);
                    let _ = state.controls_map.publish(&state.controls);
                    if let Some(input) = state.input.as_mut() {
                        input.invalidate_controls();
                    }
                    return;
                }
                read_config(&mut t.config, config);
                t.config.config_seq += 1;
                let _ = t.config_map.publish(&t.config);
                let mut header = t.frame.header;
                header.qpc = qpc();
                configuration::update(&mut t.config_full, t.config, config, header);
                let _ = t.config_full_map.publish(&t.config_full);
                if t.config_history.len() == 64 {
                    t.config_history.pop_front();
                }
                t.config_history.push_back(t.config_full);
            }
            SCS_TELEMETRY_EVENT_gameplay if !info.is_null() => {
                let event = unsafe { &*info.cast::<scs_telemetry_gameplay_event_t>() };
                let mut output = GameEvent {
                    header: t.frame.header,
                    ..Default::default()
                };
                // Gameplay events arrive between frame_start and frame_end;
                // the latter has not committed the current frame number yet.
                output.header.frame_id += 1;
                output.header.seq += 1;
                output.header.qpc = qpc();
                copy_string(event.id, &mut output.name);
                for attr in attributes(event.attributes) {
                    if unsafe { CStr::from_ptr(attr.name) }.to_bytes() == b"fine.offence"
                        && attr.value.type_ == SCS_VALUE_TYPE_string
                    {
                        copy_string(
                            unsafe { attr.value.__bindgen_anon_1.value_string.value },
                            &mut output.detail,
                        );
                    }
                }
                let _ = t.events.publish(&output);
            }
            _ => {}
        }
    })
}
fn copy_extensions(t: &mut Telemetry) {
    if t.frame.header.frame_id == 0 {
        return;
    }
    let output = t.extension_buffer.as_mut();
    output.header = t.frame.header;
    output.available = 0;
    if !t.extension_compatible {
        let _ = t.extension.publish(output);
        publish_camera_view(t);
        return;
    }
    let names = [
        "ETS2LACameraProps",
        "ETS2LATraffic",
        "ETS2LAParkedVehicles",
        "ETS2LASemaphore",
        "ETS2LARoute",
    ];
    let destinations: [&mut [u8]; 5] = [
        &mut output.camera,
        &mut output.traffic,
        &mut output.parked,
        &mut output.signals,
        &mut output.route,
    ];
    for (i, dst) in destinations.into_iter().enumerate() {
        if t.extensions[i].is_none() {
            t.extensions[i] = Mapping::open(names[i]).ok();
        }
        if let Some(source) = &t.extensions[i] {
            if source.len() >= dst.len() {
                // SAFETY: official SDK invokes all plugins sequentially on the main thread;
                // ETS2LA writes at frame_end, and this copy is at the next frame_start.
                unsafe {
                    ptr::copy_nonoverlapping(source.as_ptr(), dst.as_mut_ptr(), dst.len());
                }
                output.available |= 1 << i;
            }
        }
    }
    let _ = t.extension.publish(output);
    publish_camera_view(t);
}
fn publish_camera_view(t: &mut Telemetry) {
    let config = t.config_history.iter().rev().find(|c| {
        c.effective_from_frame_id <= t.frame.header.frame_id
            && c.header.qpc <= t.frame.header.qpc
            && c.header.qpc >= t.session_start_qpc
    });
    let view = t.camera_tracker.update(
        &t.frame,
        Some(&t.extension_buffer),
        config,
        Some(&t.dynamics),
        t.session_start_qpc,
    );
    let _ = t.camera_view_map.publish(&view);
}
fn attributes(ptr: *const scs_named_value_t) -> impl Iterator<Item = scs_named_value_t> {
    let mut index = 0;
    std::iter::from_fn(move || {
        if ptr.is_null() || index >= 4096 {
            return None;
        }
        let item = unsafe { *ptr.add(index) };
        if item.name.is_null() {
            return None;
        }
        index += 1;
        Some(item)
    })
}
fn copy_string(source: *const i8, target: &mut [u8]) {
    target.fill(0);
    if source.is_null() {
        return;
    }
    let bytes = unsafe { CStr::from_ptr(source) }.to_bytes();
    let count = bytes.len().min(target.len().saturating_sub(1));
    target[..count].copy_from_slice(&bytes[..count]);
}
fn vector(value: &scs_value_t) -> [f32; 3] {
    unsafe {
        match value.type_ {
            SCS_VALUE_TYPE_fvector => {
                let v = value.__bindgen_anon_1.value_fvector;
                [v.x, v.y, v.z]
            }
            SCS_VALUE_TYPE_dvector => {
                let v = value.__bindgen_anon_1.value_dvector;
                [v.x as f32, v.y as f32, v.z as f32]
            }
            _ => [0.0; 3],
        }
    }
}
fn scalar(value: &scs_value_t) -> f32 {
    unsafe {
        match value.type_ {
            SCS_VALUE_TYPE_float => value.__bindgen_anon_1.value_float.value,
            SCS_VALUE_TYPE_double => value.__bindgen_anon_1.value_double.value as f32,
            SCS_VALUE_TYPE_s32 => value.__bindgen_anon_1.value_s32.value as f32,
            SCS_VALUE_TYPE_u32 => value.__bindgen_anon_1.value_u32.value as f32,
            SCS_VALUE_TYPE_bool => value.__bindgen_anon_1.value_bool.value as f32,
            _ => 0.0,
        }
    }
}
fn supplement_bit(name: &[u8]) -> Option<u32> {
    match name {
        b"truck.local.acceleration.angular" => Some(DYNAMICS_ANGULAR_ACCELERATION),
        b"truck.cabin.offset" => Some(DYNAMICS_CABIN_OFFSET),
        b"truck.light.lblinker" => Some(DYNAMICS_LIGHT_LEFT),
        b"truck.light.rblinker" => Some(DYNAMICS_LIGHT_RIGHT),
        b"truck.light.aux.front" => Some(DYNAMICS_AUX_FRONT),
        b"truck.light.aux.roof" => Some(DYNAMICS_AUX_ROOF),
        _ => None,
    }
}
fn update_dynamics(output: &mut DynamicsFrame, name: &[u8], value: Option<&scs_value_t>) {
    let Some(bit) = supplement_bit(name) else {
        return;
    };
    output.valid &= !bit;
    match bit {
        DYNAMICS_ANGULAR_ACCELERATION => output.angular_acceleration_local = [0.0; 3],
        DYNAMICS_CABIN_OFFSET => {
            output.cabin_offset_position = [0.0; 3];
            output.cabin_offset_orientation_turns = [0.0; 3];
        }
        DYNAMICS_LIGHT_LEFT => output.light_blinkers &= !1,
        DYNAMICS_LIGHT_RIGHT => output.light_blinkers &= !2,
        DYNAMICS_AUX_FRONT => output.light_aux_front = 0,
        DYNAMICS_AUX_ROOF => output.light_aux_roof = 0,
        _ => {}
    }
    let Some(value) = value else {
        return;
    };
    match bit {
        DYNAMICS_ANGULAR_ACCELERATION => {
            let vector = unsafe {
                match value.type_ {
                    SCS_VALUE_TYPE_dvector => {
                        let p = value.__bindgen_anon_1.value_dvector;
                        [p.x, p.y, p.z]
                    }
                    SCS_VALUE_TYPE_fvector => {
                        let p = value.__bindgen_anon_1.value_fvector;
                        [p.x as f64, p.y as f64, p.z as f64]
                    }
                    _ => return,
                }
            };
            if vector.iter().all(|v| v.is_finite()) {
                output.angular_acceleration_local = vector;
                output.valid |= bit;
            }
        }
        DYNAMICS_CABIN_OFFSET => {
            let (position, orientation) = unsafe {
                match value.type_ {
                    SCS_VALUE_TYPE_dplacement => {
                        let p = value.__bindgen_anon_1.value_dplacement;
                        (
                            [p.position.x, p.position.y, p.position.z],
                            [
                                p.orientation.heading,
                                p.orientation.pitch,
                                p.orientation.roll,
                            ],
                        )
                    }
                    SCS_VALUE_TYPE_fplacement => {
                        let p = value.__bindgen_anon_1.value_fplacement;
                        (
                            [
                                p.position.x as f64,
                                p.position.y as f64,
                                p.position.z as f64,
                            ],
                            [
                                p.orientation.heading,
                                p.orientation.pitch,
                                p.orientation.roll,
                            ],
                        )
                    }
                    _ => return,
                }
            };
            if position.iter().all(|v| v.is_finite()) && orientation.iter().all(|v| v.is_finite()) {
                output.cabin_offset_position = position;
                output.cabin_offset_orientation_turns = orientation;
                output.valid |= bit;
            }
        }
        DYNAMICS_LIGHT_LEFT | DYNAMICS_LIGHT_RIGHT if value.type_ == SCS_VALUE_TYPE_bool => {
            if unsafe { value.__bindgen_anon_1.value_bool.value } != 0 {
                output.light_blinkers |= if bit == DYNAMICS_LIGHT_LEFT { 1 } else { 2 };
            }
            output.valid |= bit;
        }
        DYNAMICS_AUX_FRONT | DYNAMICS_AUX_ROOF if value.type_ == SCS_VALUE_TYPE_u32 => {
            let level = unsafe { value.__bindgen_anon_1.value_u32.value };
            if bit == DYNAMICS_AUX_FRONT {
                output.light_aux_front = level;
            } else {
                output.light_aux_roof = level;
            }
            output.valid |= bit;
        }
        _ => {}
    }
}
fn read_config(output: &mut ConfigBlock, config: &scs_telemetry_configuration_t) {
    if config.id.is_null() {
        return;
    }
    let group = unsafe { CStr::from_ptr(config.id) }.to_bytes();
    if group == b"truck" {
        let previous = *output;
        *output = ConfigBlock::default();
        output.config_seq = previous.config_seq;
        output.cargo_mass_kg = previous.cargo_mass_kg;
        output.delivery_time = previous.delivery_time;
        output.destination_city = previous.destination_city;
        output.destination_company = previous.destination_company;
        output.job_market = previous.job_market;
    } else if group == b"job" {
        output.cargo_mass_kg = 0.0;
        output.delivery_time = 0;
        output.destination_city.fill(0);
        output.destination_company.fill(0);
        output.job_market.fill(0);
    }
    for attr in attributes(config.attributes) {
        let name = unsafe { CStr::from_ptr(attr.name) }.to_bytes();
        let i = attr.index as usize;
        let v = &attr.value;
        match (group, name) {
            (b"truck", b"wheel.position") if i < 16 => output.wheel_positions[i] = vector(v),
            (b"truck", b"wheel.radius") if i < 16 => output.wheel_radius[i] = scalar(v),
            (b"truck", b"wheel.steerable") if i < 16 => {
                output.wheel_steerable[i] = scalar(v) as u32
            }
            (b"truck", b"wheels.count") if v.type_ == SCS_VALUE_TYPE_u32 => {
                output.wheel_count = unsafe { v.__bindgen_anon_1.value_u32.value }
            }
            (b"truck", b"hook.position") => output.hook_position = vector(v),
            (b"truck", b"cabin.position") => output.cabin_position = vector(v),
            (b"truck", b"head.position") => output.head_position = vector(v),
            (b"truck", b"retarder.steps") if v.type_ == SCS_VALUE_TYPE_u32 => {
                output.retarder_steps = unsafe { v.__bindgen_anon_1.value_u32.value }
            }
            (b"truck", b"gears.forward") if v.type_ == SCS_VALUE_TYPE_u32 => {
                output.gears_forward = unsafe { v.__bindgen_anon_1.value_u32.value }
            }
            (b"job", b"cargo.mass") => output.cargo_mass_kg = scalar(v),
            (b"job", b"delivery.time") if v.type_ == SCS_VALUE_TYPE_u32 => {
                output.delivery_time = unsafe { v.__bindgen_anon_1.value_u32.value }
            }
            (b"job", b"destination.city.id") if v.type_ == 12 => copy_string(
                unsafe { v.__bindgen_anon_1.value_string.value },
                &mut output.destination_city,
            ),
            (b"job", b"destination.company.id") if v.type_ == 12 => copy_string(
                unsafe { v.__bindgen_anon_1.value_string.value },
                &mut output.destination_company,
            ),
            (b"job", b"job.market") if v.type_ == 12 => copy_string(
                unsafe { v.__bindgen_anon_1.value_string.value },
                &mut output.job_market,
            ),
            _ => {}
        }
    }
}

unsafe extern "C" fn on_channel(
    name: *const i8,
    index: u32,
    value: *const scs_value_t,
    _context: *mut c_void,
) {
    callback(|| {
        if name.is_null() {
            return;
        }
        let Ok(mut state) = STATE.lock() else { return };
        let Some(t) = state.as_mut().and_then(|s| s.telemetry.as_mut()) else {
            return;
        };
        let name = unsafe { CStr::from_ptr(name) }.to_bytes();
        if supplement_bit(name).is_some() {
            update_dynamics(&mut t.dynamics, name, unsafe { value.as_ref() });
            return;
        }
        if value.is_null() {
            return;
        }
        let v = unsafe { &*value };
        let scalar = scalar(v);
        let f = &mut t.frame;
        match name {
            b"truck.world.placement" if v.type_ == 11 => {
                let p = unsafe { v.__bindgen_anon_1.value_dplacement };
                f.position_scs = [p.position.x, p.position.y, p.position.z];
                f.orientation_turns = [
                    p.orientation.heading,
                    p.orientation.pitch,
                    p.orientation.roll,
                ];
            }
            b"truck.local.velocity.linear" => f.velocity_local = vector(v),
            b"truck.local.acceleration.linear" => f.acceleration_local = vector(v),
            b"truck.local.velocity.angular" => f.angular_velocity_local = vector(v),
            b"truck.speed" => f.speed_mps = scalar,
            b"truck.odometer" => {
                t.counters.odometer_km = scalar as f64;
                t.counters.valid = (scalar.is_finite() && scalar >= 0.0) as u32;
            }
            b"truck.engine.rpm" => f.rpm = scalar,
            b"truck.engine.gear" => f.gear = scalar as i32,
            b"truck.displayed.gear" => f.displayed_gear = scalar as i32,
            b"truck.input.steering" => f.input[0] = scalar,
            b"truck.input.throttle" => f.input[1] = scalar,
            b"truck.input.brake" => f.input[2] = scalar,
            b"truck.input.clutch" => f.input[3] = scalar,
            b"truck.effective.steering" => f.effective[0] = scalar,
            b"truck.effective.throttle" => f.effective[1] = scalar,
            b"truck.effective.brake" => f.effective[2] = scalar,
            b"truck.effective.clutch" => f.effective[3] = scalar,
            b"truck.navigation.speed.limit" => f.navigation_speed_limit_mps = scalar,
            b"truck.navigation.distance" => f.navigation_distance_m = scalar,
            b"truck.navigation.time" => f.navigation_time_s = scalar,
            b"truck.brake.temperature" => f.brake_temperature_c = scalar,
            b"truck.brake.air.pressure" => f.brake_air_pressure_psi = scalar,
            b"truck.brake.parking" => f.parking_brake = scalar as u32,
            b"truck.brake.motor" => f.motor_brake = scalar as u32,
            b"truck.brake.retarder" => f.retarder_level = scalar as u32,
            b"truck.fuel.amount" => f.fuel_l = scalar,
            b"truck.cruise_control" => f.cruise_control_mps = scalar,
            b"local.scale" => f.local_scale = scalar,
            b"game.time" if v.type_ == SCS_VALUE_TYPE_u32 => {
                f.game_time_min = unsafe { v.__bindgen_anon_1.value_u32.value }
            }
            b"rest.stop" => f.rest_stop_min = scalar as i32,
            b"truck.wipers" => f.wipers = scalar as u32,
            b"truck.wheel.steering" if index < 16 => f.wheel_steering[index as usize] = scalar,
            b"truck.wheel.on_ground" if index < 16 => {
                f.wheel_on_ground[index as usize] = scalar as u32
            }
            b"truck.wheel.substance" if index < 16 => {
                f.wheel_substance[index as usize] = scalar as u32
            }
            _ => {
                if let Some(rest) = name.strip_prefix(b"trailer.") {
                    if rest.len() > 2 && rest[0].is_ascii_digit() && rest[1] == b'.' {
                        let trailer = &mut f.trailers[(rest[0] - b'0') as usize];
                        match &rest[2..] {
                            b"world.placement" if v.type_ == 11 => {
                                let p = unsafe { v.__bindgen_anon_1.value_dplacement };
                                trailer.position_scs = [p.position.x, p.position.y, p.position.z];
                                trailer.orientation_turns = [
                                    p.orientation.heading,
                                    p.orientation.pitch,
                                    p.orientation.roll,
                                ];
                            }
                            b"velocity.linear" => trailer.velocity_scs = vector(v),
                            b"connected" => trailer.connected = scalar as u32,
                            _ => {}
                        }
                    }
                } else if let Some(i) = WEAR.iter().position(|n| n.as_bytes() == name) {
                    f.wear[i] = scalar
                } else if let Some(i) = LIGHTS.iter().position(|n| n.as_bytes() == name) {
                    if scalar != 0.0 {
                        f.lights |= 1 << i
                    } else {
                        f.lights &= !(1 << i)
                    }
                }
            }
        }
    })
}
const WEAR: [&str; 5] = [
    "truck.wear.engine",
    "truck.wear.transmission",
    "truck.wear.cabin",
    "truck.wear.chassis",
    "truck.wear.wheels",
];
const LIGHTS: [&str; 9] = [
    "truck.light.parking",
    "truck.light.beam.low",
    "truck.light.beam.high",
    "truck.light.beacon",
    "truck.light.brake",
    "truck.light.reverse",
    "truck.lblinker",
    "truck.rblinker",
    "truck.hazard.warning",
];
fn subscriptions() -> Vec<(String, u32, u32)> {
    let mut list = Vec::new();
    let mut add = |name: &str, kind, index| list.push((name.to_owned(), kind, index));
    add("truck.world.placement", 11, u32::MAX);
    add(
        "truck.local.acceleration.angular",
        SCS_VALUE_TYPE_dvector,
        u32::MAX,
    );
    add("truck.cabin.offset", SCS_VALUE_TYPE_dplacement, u32::MAX);
    add("truck.light.lblinker", SCS_VALUE_TYPE_bool, u32::MAX);
    add("truck.light.rblinker", SCS_VALUE_TYPE_bool, u32::MAX);
    add("truck.light.aux.front", SCS_VALUE_TYPE_u32, u32::MAX);
    add("truck.light.aux.roof", SCS_VALUE_TYPE_u32, u32::MAX);
    for n in [
        "truck.local.velocity.linear",
        "truck.local.acceleration.linear",
        "truck.local.velocity.angular",
    ] {
        add(n, 7, u32::MAX)
    }
    for n in [
        "truck.speed",
        "truck.odometer",
        "truck.engine.rpm",
        "truck.input.steering",
        "truck.input.throttle",
        "truck.input.brake",
        "truck.input.clutch",
        "truck.effective.steering",
        "truck.effective.throttle",
        "truck.effective.brake",
        "truck.effective.clutch",
        "truck.navigation.speed.limit",
        "truck.navigation.distance",
        "truck.navigation.time",
        "truck.brake.temperature",
        "truck.brake.air.pressure",
        "truck.fuel.amount",
        "truck.cruise_control",
        "local.scale",
    ]
    .into_iter()
    .chain(WEAR)
    {
        add(n, 5, u32::MAX)
    }
    for n in ["truck.engine.gear", "truck.displayed.gear", "rest.stop"] {
        add(n, 2, u32::MAX)
    }
    for n in ["game.time", "truck.brake.retarder"] {
        add(n, 3, u32::MAX)
    }
    for n in ["truck.brake.parking", "truck.brake.motor", "truck.wipers"]
        .into_iter()
        .chain(LIGHTS)
    {
        add(n, 1, u32::MAX)
    }
    for i in 0..16 {
        for (n, k) in [
            ("truck.wheel.steering", 5),
            ("truck.wheel.on_ground", 1),
            ("truck.wheel.substance", 3),
        ] {
            add(n, k, i)
        }
    }
    for i in 0..10 {
        for (suffix, kind) in [
            ("world.placement", 11),
            ("velocity.linear", 7),
            ("connected", 1),
        ] {
            add(&format!("trailer.{i}.{suffix}"), kind, u32::MAX)
        }
    }
    list
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn invalid_versions_and_null_init_fail_closed() {
        unsafe {
            assert_eq!(scs_telemetry_init(0, ptr::null()), SCS_RESULT_unsupported);
            assert_eq!(
                scs_telemetry_init(0x10001, ptr::null()),
                SCS_RESULT_invalid_parameter
            );
            assert_eq!(
                scs_input_init(0x10000, ptr::null()),
                SCS_RESULT_invalid_parameter
            );
        }
    }
    #[test]
    fn input_count_matches_wire() {
        assert_eq!(
            INPUT_COUNT,
            AXIS_NAMES.len() + HOLD_NAMES.len() + PULSE_NAMES.len()
        );
        assert!(PULSE_NAMES.len() <= 64);
        assert!(INPUT_COUNT <= SCS_INPUT_MAX_INPUT_COUNT as usize);
    }
    #[test]
    fn official_wheels_count_and_configuration_groups() {
        let mut config = ConfigBlock {
            cargo_mass_kg: 15000.0,
            ..Default::default()
        };
        config.destination_city[..3].copy_from_slice(b"abc");
        let mut value = scs_value_t {
            type_: SCS_VALUE_TYPE_u32,
            ..Default::default()
        };
        value.__bindgen_anon_1.value_u32 = scs_value_u32_t { value: 6 };
        let attributes = [
            scs_named_value_t {
                name: c"wheels.count".as_ptr(),
                index: u32::MAX,
                value,
                ..Default::default()
            },
            scs_named_value_t::default(),
        ];
        read_config(
            &mut config,
            &scs_telemetry_configuration_t {
                id: c"truck".as_ptr(),
                attributes: attributes.as_ptr(),
            },
        );
        assert_eq!(config.wheel_count, 6);
        assert_eq!(config.cargo_mass_kg, 15000.0);
        assert_eq!(&config.destination_city[..3], b"abc");
        read_config(
            &mut config,
            &scs_telemetry_configuration_t {
                id: c"job".as_ptr(),
                attributes: attributes[1..].as_ptr(),
            },
        );
        assert_eq!(config.wheel_count, 6);
        assert_eq!(config.cargo_mass_kg, 0.0);
        assert!(config.destination_city.iter().all(|x| *x == 0));
        value.__bindgen_anon_1.value_u32 = scs_value_u32_t { value: 16_777_217 };
        let deadline = [
            scs_named_value_t {
                name: c"delivery.time".as_ptr(),
                index: u32::MAX,
                value,
                ..Default::default()
            },
            scs_named_value_t::default(),
        ];
        read_config(
            &mut config,
            &scs_telemetry_configuration_t {
                id: c"job".as_ptr(),
                attributes: deadline.as_ptr(),
            },
        );
        assert_eq!(config.delivery_time, 16_777_217);
    }
    #[cfg(windows)]
    #[test]
    fn mock_sdk_ten_lifecycles_and_input_release() {
        fn controls_event(mode: &CStr) {
            let mut value = scs_value_t {
                type_: SCS_VALUE_TYPE_string,
                ..Default::default()
            };
            value.__bindgen_anon_1.value_string = scs_value_string_t {
                value: mode.as_ptr(),
            };
            let attrs = [
                scs_named_value_t {
                    name: c"shifter.type".as_ptr(),
                    index: u32::MAX,
                    value,
                    ..Default::default()
                },
                scs_named_value_t::default(),
            ];
            let configuration = scs_telemetry_configuration_t {
                id: c"controls".as_ptr(),
                attributes: attrs.as_ptr(),
            };
            unsafe {
                on_event(
                    SCS_TELEMETRY_EVENT_configuration,
                    (&configuration as *const scs_telemetry_configuration_t).cast(),
                    ptr::null_mut(),
                );
            }
        }
        fn drain_zero() {
            let mut out = scs_input_event_t::default();
            let mut count = 0;
            while unsafe { on_input(&mut out, 3, ptr::null_mut()) } == SCS_RESULT_ok {
                count += 1;
                assert!(count <= INPUT_COUNT);
                if out.input_index < AXIS_NAMES.len() as u32 {
                    assert_eq!(unsafe { out.__bindgen_anon_1.value_float.value }, 0.0);
                } else {
                    assert_eq!(unsafe { out.__bindgen_anon_1.value_bool.value }, 0);
                }
            }
        }
        unsafe extern "C" fn event(
            event: u32,
            callback: scs_telemetry_event_callback_t,
            context: *mut c_void,
        ) -> i32 {
            if event == SCS_TELEMETRY_EVENT_configuration {
                controls_event(c"automatic");
                let attrs = [scs_named_value_t::default()];
                let config = scs_telemetry_configuration_t {
                    id: c"truck".as_ptr(),
                    attributes: attrs.as_ptr(),
                };
                unsafe {
                    callback.unwrap()(
                        event,
                        (&config as *const scs_telemetry_configuration_t).cast(),
                        context,
                    );
                }
                let mut value = scs_value_t {
                    type_: SCS_VALUE_TYPE_fvector,
                    ..Default::default()
                };
                value.__bindgen_anon_1.value_fvector = scs_value_fvector_t {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                };
                let attrs = [
                    scs_named_value_t {
                        name: c"hook.position".as_ptr(),
                        index: u32::MAX,
                        value,
                        ..Default::default()
                    },
                    scs_named_value_t::default(),
                ];
                let config = scs_telemetry_configuration_t {
                    id: c"trailer.9".as_ptr(),
                    attributes: attrs.as_ptr(),
                };
                unsafe {
                    callback.unwrap()(
                        event,
                        (&config as *const scs_telemetry_configuration_t).cast(),
                        context,
                    );
                }
            }
            0
        }
        unsafe extern "C" fn channel(
            name: *const i8,
            index: u32,
            kind: u32,
            flags: u32,
            callback: scs_telemetry_channel_callback_t,
            context: *mut c_void,
        ) -> i32 {
            if supplement_bit(unsafe { CStr::from_ptr(name) }.to_bytes()).is_some() {
                assert_eq!(
                    flags,
                    SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value
                );
            }
            let mut value = scs_value_t {
                type_: kind,
                ..Default::default()
            };
            if kind == SCS_VALUE_TYPE_float {
                value.__bindgen_anon_1.value_float = scs_value_float_t { value: 1.0 };
            }
            unsafe {
                callback.unwrap()(name, index, &value, context);
            }
            0
        }
        static UNREGISTERED: std::sync::atomic::AtomicUsize =
            std::sync::atomic::AtomicUsize::new(0);
        unsafe extern "C" fn unregister_event(_: u32) -> i32 {
            UNREGISTERED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            0
        }
        unsafe extern "C" fn unregister_channel(_: *const i8, _: u32, _: u32) -> i32 {
            0
        }
        unsafe extern "C" fn device(device: *const scs_input_device_t) -> i32 {
            let device = unsafe { &*device };
            assert_eq!(device.type_, SCS_INPUT_DEVICE_TYPE_semantical);
            assert_eq!(device.input_count, INPUT_COUNT as u32);
            for (offset, name) in [(2, b"quickload".as_slice()), (1, b"engine".as_slice())] {
                let input = unsafe { &*device.inputs.add(INPUT_COUNT - offset) };
                assert_eq!(unsafe { CStr::from_ptr(input.name) }.to_bytes(), name);
                assert_eq!(input.value_type, SCS_VALUE_TYPE_bool);
            }
            assert_eq!(unsafe { CStr::from_ptr(device.name) }.to_bytes(), b"etsads");
            unsafe {
                device.input_active_callback.unwrap()(0, device.callback_context);
                let mut event = scs_input_event_t::default();
                for _ in 0..=INPUT_COUNT {
                    if device.input_event_callback.unwrap()(&mut event, 3, device.callback_context)
                        == SCS_RESULT_not_found
                    {
                        return 0;
                    }
                }
            }
            SCS_RESULT_generic_error
        }
        let telemetry = scs_telemetry_init_params_v100_t {
            common: scs_sdk_init_params_v100_t {
                game_id: c"test_ets2".as_ptr(),
                game_name: c"Mock SDK Game".as_ptr(),
                game_version: 0x10203,
                ..Default::default()
            },
            register_for_event: Some(event),
            register_for_channel: Some(channel),
            unregister_from_event: Some(unregister_event),
            unregister_from_channel: Some(unregister_channel),
        };
        let input = scs_input_init_params_v100_t {
            register_device: Some(device),
            ..Default::default()
        };
        let mut last_session = 0;
        for _ in 0..10 {
            assert_eq!(
                unsafe {
                    scs_telemetry_init(
                        0x10001,
                        (&telemetry as *const scs_telemetry_init_params_v100_t).cast(),
                    )
                },
                0
            );
            assert_eq!(
                unsafe {
                    scs_input_init(
                        0x10000,
                        (&input as *const scs_input_init_params_v100_t).cast(),
                    )
                },
                0
            );
            let info = SharedCell::<BridgeInfo>::open(&mapping_name(INFO_MAPPING), SCHEMA_VERSION)
                .unwrap();
            let metadata = info.latest().unwrap();
            assert_eq!(metadata.game_version, 0x10203);
            assert_eq!(&metadata.game_id[..9], b"test_ets2");
            assert_eq!(metadata.telemetry_api_version, 0x10001);
            assert_eq!(metadata.input_api_version, 0x10000);
            assert!(metadata.session_start_qpc > last_session);
            last_session = metadata.session_start_qpc;
            unsafe {
                on_event(SCS_TELEMETRY_EVENT_started, ptr::null(), ptr::null_mut());
            }
            let start = scs_telemetry_frame_start_t {
                simulation_time: 1000,
                paused_simulation_time: 900,
                ..Default::default()
            };
            let attributes = [scs_named_value_t::default()];
            let gameplay = scs_telemetry_gameplay_event_t {
                id: c"player.fined".as_ptr(),
                attributes: attributes.as_ptr(),
            };
            let mut angular = scs_value_t {
                type_: SCS_VALUE_TYPE_dvector,
                ..Default::default()
            };
            angular.__bindgen_anon_1.value_dvector = scs_value_dvector_t {
                x: 0.125,
                y: -0.25,
                z: 0.5,
            };
            unsafe {
                on_event(
                    SCS_TELEMETRY_EVENT_frame_start,
                    (&start as *const scs_telemetry_frame_start_t).cast(),
                    ptr::null_mut(),
                );
                on_channel(
                    c"truck.local.acceleration.angular".as_ptr(),
                    u32::MAX,
                    &angular,
                    ptr::null_mut(),
                );
                on_event(
                    SCS_TELEMETRY_EVENT_gameplay,
                    (&gameplay as *const scs_telemetry_gameplay_event_t).cast(),
                    ptr::null_mut(),
                );
                on_event(SCS_TELEMETRY_EVENT_frame_end, ptr::null(), ptr::null_mut());
            }
            {
                let telemetry = SharedRing::<TelemetryFrame, 64>::open(
                    &mapping_name(TELEMETRY_MAPPING),
                    SCHEMA_VERSION,
                )
                .unwrap();
                let events = SharedRing::<GameEvent, 256>::open(
                    &mapping_name(EVENTS_MAPPING),
                    SCHEMA_VERSION,
                )
                .unwrap();
                assert_eq!(
                    events.latest().unwrap().header.frame_id,
                    telemetry.latest().unwrap().header.frame_id
                );
                assert_eq!(events.latest().unwrap().header.sim_time_us, 1000);
                assert!(
                    events.latest().unwrap().header.qpc >= telemetry.latest().unwrap().header.qpc
                );
                let dynamics = SharedRing::<DynamicsFrame, 64>::open(
                    &mapping_name(DYNAMICS_MAPPING),
                    SCHEMA_VERSION,
                )
                .unwrap()
                .latest()
                .unwrap();
                let expected = telemetry.latest().unwrap().header;
                let observed = dynamics.header;
                assert_eq!(
                    (
                        observed.frame_id,
                        observed.seq,
                        observed.qpc,
                        observed.sim_time_us,
                        observed.flags,
                        observed.source
                    ),
                    (
                        expected.frame_id,
                        expected.seq,
                        expected.qpc,
                        expected.sim_time_us,
                        expected.flags,
                        expected.source
                    )
                );
                assert_eq!(dynamics.angular_acceleration_local, [0.125, -0.25, 0.5]);
                assert_ne!(dynamics.valid & DYNAMICS_ANGULAR_ACCELERATION, 0);
                let configuration = SharedRing::<ConfigurationFrame, 64>::open(
                    &mapping_name(CONFIG_FULL_MAPPING),
                    SCHEMA_VERSION,
                )
                .unwrap()
                .latest()
                .unwrap();
                assert_eq!(configuration.config_seq, configuration.truck.config_seq);
                assert_eq!(configuration.effective_from_frame_id, 1);
                assert!(configuration.header.qpc > 0);
                assert_eq!(configuration.present_trailer_mask, 1 << 9);
                assert_eq!(configuration.trailers[9].hook_position, [1.0, 2.0, 3.0]);
                assert_eq!(
                    configuration.trailers[9].config_seq,
                    configuration.config_seq
                );
                let before = configuration.config_seq;
                let invalid = scs_telemetry_configuration_t::default();
                unsafe {
                    on_event(
                        SCS_TELEMETRY_EVENT_configuration,
                        (&invalid as *const scs_telemetry_configuration_t).cast(),
                        ptr::null_mut(),
                    );
                }
                assert_eq!(
                    SharedRing::<ConfigurationFrame, 64>::open(
                        &mapping_name(CONFIG_FULL_MAPPING),
                        SCHEMA_VERSION,
                    )
                    .unwrap()
                    .latest()
                    .unwrap()
                    .config_seq,
                    before
                );
            }
            let control =
                SharedCell::<ControlCmd>::open(&mapping_name(CONTROL_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            let gate =
                SharedCell::<Gate>::open(&mapping_name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
            let applied =
                SharedCell::<AppliedControl>::open(&mapping_name(APPLIED_MAPPING), SCHEMA_VERSION)
                    .unwrap();
            gate.publish(&Gate {
                heartbeat_qpc: qpc(),
                ..Default::default()
            })
            .unwrap();
            drain_zero(); // New session must observe Off before a fresh arm.
            control
                .publish(&ControlCmd {
                    seq: 1,
                    write_qpc: qpc(),
                    throttle: 0.2,
                    ..Default::default()
                })
                .unwrap();
            gate.publish(&Gate {
                seq: 1,
                heartbeat_qpc: qpc(),
                enabled: 1,
                max_throttle: 1.0,
                max_steer_rate: 2.0,
                ..Default::default()
            })
            .unwrap();
            let mut out = scs_input_event_t::default();
            let mut count = 0;
            // Adversarial SDK: repeats both "first" flags for every dequeue.
            // Preparing again would endlessly replenish force/rate-limit events.
            while unsafe { on_input(&mut out, 3, ptr::null_mut()) } == 0 {
                count += 1;
                assert!(count <= INPUT_COUNT)
            }
            assert_eq!(count, INPUT_COUNT);
            let diagnostics = SharedCell::<InputDiagnostics>::open(
                &mapping_name(INPUT_DIAGNOSTICS_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap()
            .latest()
            .unwrap();
            assert_eq!(diagnostics.last_batch_events, INPUT_COUNT as u32);
            assert_eq!(diagnostics.last_batch_callbacks, INPUT_COUNT as u32 + 1);
            assert_eq!(diagnostics.max_batch_events, INPUT_COUNT as u32);
            assert!(diagnostics.repeated_first_flags >= INPUT_COUNT as u64);
            assert_eq!(applied.latest().unwrap().axes[1], 0.2);
            let timing = SharedRing::<FrameTiming, 256>::open(
                &mapping_name(FRAME_TIMING_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let records: Vec<_> = (1..=timing.latest_index())
                .filter_map(|index| timing.read(index))
                .collect();
            assert!(records.windows(2).all(|pair| {
                pair[0].event_seq < pair[1].event_seq && pair[0].qpc <= pair[1].qpc
            }));
            let start = records.iter().find(|t| t.kind == 1).unwrap();
            let end = records.iter().find(|t| t.kind == 2).unwrap();
            let input = records.last().unwrap();
            assert_eq!(start.physics_frame_id, 1);
            assert_eq!(end.physics_frame_id, 1);
            assert_eq!(start.qpc, start.frame_start_qpc);
            assert_eq!(end.frame_start_qpc, start.qpc);
            assert_eq!(end.qpc, end.frame_end_qpc);
            assert_eq!(input.kind, 3);
            assert_eq!(input.input_prepare_qpc, input.qpc);
            assert_eq!(input.input_batch_id, applied.latest().unwrap().frame_index);
            assert_eq!(input.physics_frame_id, 1);
            assert_eq!(input.command_seq, 1);
            let actions = SharedRing::<InputActionFrame, 256>::open(
                &mapping_name(INPUT_ACTIONS_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            let previous_action = actions.latest_index();
            let mut pulse = control.latest().unwrap();
            pulse.seq += 1;
            pulse.write_qpc = qpc();
            pulse.holds = 7;
            pulse.pulses[0] += 1;
            control.publish(&pulse).unwrap();
            let mut saw_button = false;
            let mut events_returned = 0;
            while unsafe { on_input(&mut out, 1, ptr::null_mut()) } == SCS_RESULT_ok {
                events_returned += 1;
                assert!(events_returned <= INPUT_COUNT);
                assert_eq!(
                    actions.latest_index(),
                    previous_action,
                    "do not certify an unfinished SDK batch"
                );
                if out.input_index == 7 {
                    saw_button |= unsafe { out.__bindgen_anon_1.value_bool.value } == 1;
                }
            }
            let consumed = actions.latest().unwrap();
            assert!(saw_button);
            assert_eq!(consumed.pulse_mask, 1);
            assert_eq!(consumed.pulse_counters[0], 1);
            assert_eq!(consumed.holds, 7);
            assert_eq!(consumed.command_seq, pulse.seq);
            assert!(consumed.completed_qpc >= consumed.prepared_qpc);
            for pulse_name in ["quickload", "engine"] {
                STATE
                    .lock()
                    .unwrap()
                    .as_mut()
                    .unwrap()
                    .telemetry
                    .as_mut()
                    .unwrap()
                    .frame
                    .speed_mps = 1.0;
                let stationary_index = PULSE_NAMES
                    .iter()
                    .position(|name| *name == pulse_name)
                    .unwrap();
                let stationary_input = (PULSE_INPUT_START + stationary_index) as u32;
                let mut stationary_command = control.latest().unwrap();
                stationary_command.seq += 1;
                stationary_command.write_qpc = qpc();
                stationary_command.pulses[stationary_index] += 1;
                control.publish(&stationary_command).unwrap();
                let mut count = 0;
                let mut emitted = false;
                while unsafe { on_input(&mut out, 1, ptr::null_mut()) } == SCS_RESULT_ok {
                    count += 1;
                    assert!(count <= INPUT_COUNT);
                    emitted |= out.input_index == stationary_input
                        && unsafe { out.__bindgen_anon_1.value_bool.value } == 1;
                }
                assert!(
                    !emitted,
                    "moving truck must discard {pulse_name}, not queue it for later"
                );
                assert_eq!(
                    actions.latest().unwrap().pulse_mask & (1u64 << stationary_index),
                    0
                );
                STATE
                    .lock()
                    .unwrap()
                    .as_mut()
                    .unwrap()
                    .telemetry
                    .as_mut()
                    .unwrap()
                    .frame
                    .speed_mps = 0.0;
                stationary_command.seq += 1;
                stationary_command.write_qpc = qpc();
                stationary_command.pulses[stationary_index] += 1;
                control.publish(&stationary_command).unwrap();
                count = 0;
                emitted = false;
                while unsafe { on_input(&mut out, 1, ptr::null_mut()) } == SCS_RESULT_ok {
                    count += 1;
                    assert!(count <= INPUT_COUNT);
                    emitted |= out.input_index == stationary_input
                        && unsafe { out.__bindgen_anon_1.value_bool.value } == 1;
                }
                assert!(
                    emitted,
                    "registered {pulse_name} input must be returned by the real SDK callback"
                );
                let completed = actions.latest().unwrap();
                assert_eq!(completed.pulse_mask, 1u64 << stationary_index);
                assert_eq!(completed.pulse_counters[stationary_index], 2);
            }
            let mut recovered = control.latest().unwrap();
            recovered.seq += 1;
            recovered.write_qpc = qpc();
            recovered.steer = 1.0;
            recovered.throttle = 1.0;
            recovered.holds = 7;
            recovered.pulses[0] += 1;
            control.publish(&recovered).unwrap();
            gate.publish(&Gate {
                seq: 2,
                heartbeat_qpc: qpc(),
                enabled: GATE_BRAKE_ONLY,
                aeb_brake: 0.2,
                max_throttle: 0.0,
                max_steer_rate: 0.8,
            })
            .unwrap();
            let mut brake_only_events = 0;
            while unsafe { on_input(&mut out, 3, ptr::null_mut()) } == SCS_RESULT_ok {
                brake_only_events += 1;
                assert!(brake_only_events <= INPUT_COUNT);
                if out.input_index >= AXIS_NAMES.len() as u32 {
                    assert_eq!(
                        unsafe { out.__bindgen_anon_1.value_bool.value },
                        0,
                        "BrakeOnly must release all held and pulse buttons"
                    );
                }
            }
            assert_eq!(brake_only_events, INPUT_COUNT);
            let retreat = applied.latest().unwrap();
            assert_eq!(retreat.axes, [0.0, 0.0, 0.2, 0.0]);
            assert_eq!(retreat.reason, etsads_game::input::REASON_BRAKE_ONLY);
            assert_eq!(actions.latest().unwrap().pulse_mask, 0);
            assert_eq!(actions.latest().unwrap().holds, 0);
            // A controls event may arrive midway through a prepared batch.
            // Its queued brake/buttons must never escape after switching mode.
            gate.publish(&Gate {
                heartbeat_qpc: qpc(),
                enabled: GATE_ACTIVE,
                max_throttle: 1.0,
                max_steer_rate: 2.0,
                ..Default::default()
            })
            .unwrap();
            let mut unsafe_command = control.latest().unwrap();
            unsafe_command.seq += 1;
            unsafe_command.write_qpc = qpc();
            unsafe_command.brake = 0.5;
            unsafe_command.pulses[1] += 1;
            control.publish(&unsafe_command).unwrap();
            assert_eq!(
                unsafe { on_input(&mut out, 3, ptr::null_mut()) },
                SCS_RESULT_ok
            );
            controls_event(c"arcade");
            let controls = SharedRing::<ControlConfiguration, 64>::open(
                &mapping_name(CONTROL_CONFIGURATION_MAPPING),
                SCHEMA_VERSION,
            )
            .unwrap();
            assert_eq!(controls.latest().unwrap().shifter_type, SHIFTER_ARCADE);
            assert!(!controls.latest().unwrap().safe_shifter());
            assert_eq!(
                unsafe { on_input(&mut out, 0, ptr::null_mut()) },
                SCS_RESULT_not_found
            );
            for mode in [GATE_ACTIVE, GATE_BRAKE_ONLY] {
                gate.publish(&Gate {
                    heartbeat_qpc: qpc(),
                    enabled: mode,
                    aeb_brake: 0.3,
                    max_throttle: 0.0,
                    max_steer_rate: 2.0,
                    ..Default::default()
                })
                .unwrap();
                drain_zero();
                assert_eq!(
                    applied.latest().unwrap().reason,
                    etsads_game::input::REASON_CONTROL_CONFIGURATION
                );
                assert_eq!(actions.latest().unwrap().pulse_mask, 0);
                assert_eq!(actions.latest().unwrap().holds, 0);
            }
            controls_event(c"future-unknown-mode");
            drain_zero();
            assert_eq!(controls.latest().unwrap().valid, 0);
            controls_event(c"automatic");
            gate.publish(&Gate {
                heartbeat_qpc: qpc(),
                enabled: GATE_ACTIVE,
                max_throttle: 1.0,
                max_steer_rate: 2.0,
                ..Default::default()
            })
            .unwrap();
            drain_zero(); // Merely returning to a safe enum never revives old arm.
            assert_eq!(
                applied.latest().unwrap().reason,
                etsads_game::input::REASON_CONTROL_CONFIGURATION
            );
            gate.publish(&Gate {
                heartbeat_qpc: qpc(),
                ..Default::default()
            })
            .unwrap();
            drain_zero();
            unsafe_command.seq += 1;
            unsafe_command.write_qpc = qpc();
            unsafe_command.steer = 0.0;
            unsafe_command.throttle = 0.2;
            unsafe_command.brake = 0.0;
            unsafe_command.holds = 0;
            control.publish(&unsafe_command).unwrap();
            gate.publish(&Gate {
                heartbeat_qpc: qpc(),
                enabled: GATE_ACTIVE,
                max_throttle: 1.0,
                max_steer_rate: 2.0,
                ..Default::default()
            })
            .unwrap();
            let mut emitted = 0;
            while unsafe { on_input(&mut out, 3, ptr::null_mut()) } == SCS_RESULT_ok {
                emitted += 1;
                assert!(emitted <= INPUT_COUNT);
                if out.input_index >= AXIS_NAMES.len() as u32 {
                    assert_eq!(
                        unsafe { out.__bindgen_anon_1.value_bool.value },
                        0,
                        "mode change must consume old queued pulses"
                    );
                }
            }
            assert_eq!(applied.latest().unwrap().reason, REASON_ACTIVE);
            assert_eq!(applied.latest().unwrap().axes[1], 0.2);
            assert_eq!(
                unsafe { on_input(&mut out, 3, ptr::null_mut()) },
                SCS_RESULT_ok
            );
            unsafe {
                on_event(SCS_TELEMETRY_EVENT_paused, ptr::null(), ptr::null_mut());
            }
            assert_eq!(
                unsafe { on_input(&mut out, 0, ptr::null_mut()) },
                SCS_RESULT_not_found
            );
            drain_zero();
            assert_eq!(
                applied.latest().unwrap().reason,
                etsads_game::input::REASON_PAUSED
            );
            unsafe {
                on_event(SCS_TELEMETRY_EVENT_started, ptr::null(), ptr::null_mut());
            }
            drain_zero();
            assert_eq!(
                applied.latest().unwrap().reason,
                etsads_game::input::REASON_CONTROL_CONFIGURATION,
                "resume must not revive the pre-pause Active gate"
            );
            gate.publish(&Gate::default()).unwrap();
            drain_zero();
            assert_eq!(applied.latest().unwrap().axes, [0.0; 4]);
            drop(control);
            drop(gate);
            drop(applied);
            scs_telemetry_shutdown();
            scs_input_shutdown();
            let metadata = info.latest().unwrap();
            assert_eq!(metadata.telemetry_api_version, 0);
            assert_eq!(metadata.input_api_version, 0);
            assert!(STATE.lock().unwrap().is_none());
        }
        unsafe extern "C" fn fail_event(
            event: u32,
            callback: scs_telemetry_event_callback_t,
            context: *mut c_void,
        ) -> i32 {
            if event == 3 {
                -7
            } else {
                unsafe { callback.unwrap()(event, ptr::null(), context) };
                0
            }
        }
        let mut failing = telemetry;
        failing.register_for_event = Some(fail_event);
        UNREGISTERED.store(0, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            unsafe {
                scs_telemetry_init(
                    0x10001,
                    (&failing as *const scs_telemetry_init_params_v100_t).cast(),
                )
            },
            -7
        );
        assert_eq!(UNREGISTERED.load(std::sync::atomic::Ordering::Relaxed), 2);
        assert!(STATE.lock().unwrap().is_none());
    }

    #[test]
    fn sdk_sidecar_types_unavailable_values_and_units_are_not_guessed() {
        let subscriptions = subscriptions();
        assert!(subscriptions.contains(&(
            "truck.local.acceleration.angular".into(),
            SCS_VALUE_TYPE_dvector,
            u32::MAX
        )));
        assert!(subscriptions.contains(&(
            "truck.cabin.offset".into(),
            SCS_VALUE_TYPE_dplacement,
            u32::MAX
        )));
        let mut dynamics = DynamicsFrame::default();
        let mut value = scs_value_t {
            type_: SCS_VALUE_TYPE_dvector,
            ..Default::default()
        };
        value.__bindgen_anon_1.value_dvector = scs_value_dvector_t {
            x: 1.25,
            y: 2.5,
            z: -3.0,
        };
        update_dynamics(
            &mut dynamics,
            b"truck.local.acceleration.angular",
            Some(&value),
        );
        assert_eq!(dynamics.angular_acceleration_local, [1.25, 2.5, -3.0]);
        update_dynamics(&mut dynamics, b"truck.local.acceleration.angular", None);
        assert_eq!(dynamics.valid & DYNAMICS_ANGULAR_ACCELERATION, 0);
        assert_eq!(dynamics.angular_acceleration_local, [0.0; 3]);
        value.__bindgen_anon_1.value_dvector = scs_value_dvector_t {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
        };
        update_dynamics(
            &mut dynamics,
            b"truck.local.acceleration.angular",
            Some(&value),
        );
        assert_eq!(dynamics.valid & DYNAMICS_ANGULAR_ACCELERATION, 0);
        let mut level = scs_value_t {
            type_: SCS_VALUE_TYPE_u32,
            ..Default::default()
        };
        level.__bindgen_anon_1.value_u32 = scs_value_u32_t { value: 2 };
        update_dynamics(&mut dynamics, b"truck.light.aux.front", Some(&level));
        assert_eq!(dynamics.light_aux_front, 2);
        assert_ne!(dynamics.valid & DYNAMICS_AUX_FRONT, 0);
        update_dynamics(&mut dynamics, b"truck.light.aux.front", None);
        assert_eq!(dynamics.valid & DYNAMICS_AUX_FRONT, 0);
    }
}
