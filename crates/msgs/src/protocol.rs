//! Versioned, pointer-free game/host wire protocol. All clocks named `qpc` are
//! Windows performance-counter ticks; use `qpc_frequency` for conversion.
use bytemuck::{Pod, Zeroable};

pub const SCHEMA_VERSION: u32 = 1;
pub const TELEMETRY_MAPPING: &str = "ETSADS_Telemetry";
pub const CONTROL_MAPPING: &str = "ETSADS_Control";
pub const GATE_MAPPING: &str = "ETSADS_Gate";
pub const APPLIED_MAPPING: &str = "ETSADS_Applied";
pub const CONFIG_MAPPING: &str = "ETSADS_Config";
pub const EVENTS_MAPPING: &str = "ETSADS_Events";
pub const EXTENSION_MAPPING: &str = "ETSADS_Extension";
pub const SAFETY_REQUEST_MAPPING: &str = "ETSADS_SafetyRequest";
pub const INFO_MAPPING: &str = "ETSADS_Info";
pub const OVERLAY_MAPPING: &str = "ETSADS_Overlay";
pub const VEHICLE_COUNTERS_MAPPING: &str = "ETSADS_VehicleCounters";
pub const INPUT_DIAGNOSTICS_MAPPING: &str = "ETSADS_InputDiagnostics";
pub const INPUT_ACTIONS_MAPPING: &str = "ETSADS_InputActions";
pub const FRAME_TIMING_MAPPING: &str = "ETSADS_FrameTiming";
pub const DYNAMICS_MAPPING: &str = "ETSADS_Dynamics";
pub const CONFIG_FULL_MAPPING: &str = "ETSADS_ConfigFull";
pub const CONTROL_CONFIGURATION_MAPPING: &str = "ETSADS_ControlConfiguration";
pub const CAMERA_VIEW_MAPPING: &str = "ETSADS_CameraView";
pub const SUPERVISOR_STATUS_MAPPING: &str = "ETSADS_SupervisorStatus";
pub const TELEMETRY_SLOTS: usize = 64;
pub const EVENT_SLOTS: usize = 256;
/// Gate is a mode, not a boolean. Unknown modes must release every input.
pub const GATE_OFF: u32 = 0;
pub const GATE_ACTIVE: u32 = 1;
/// Independent supervisor-owned, bounded brake retreat after command loss.
/// Main-stack commands, steering, throttle and buttons are ignored in this mode.
pub const GATE_BRAKE_ONLY: u32 = 2;

pub const SHIFTER_UNKNOWN: u32 = 0;
pub const SHIFTER_ARCADE: u32 = 1;
pub const SHIFTER_AUTOMATIC: u32 = 2;
pub const SHIFTER_MANUAL: u32 = 3;
pub const SHIFTER_HSHIFTER: u32 = 4;
pub const CONTROL_CONFIGURATION_SHIFTER_VALID: u32 = 1;
/// Official `controls` / `shifter.type` SDK events. Static configuration has no
/// age expiry: consumers bind session and generation and reject future events.
/// header.qpc is the real callback QPC, header.frame_id the last completed frame;
/// effective_from_frame_id is only a lower bound, also requiring event QPC <=
/// telemetry frame_start QPC. Generation 0 is explicit unknown at session init.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct ControlConfiguration {
    pub header: WireHeader,
    pub session_start_qpc: u64,
    pub generation: u64,
    pub effective_from_frame_id: u64,
    pub shifter_type: u32,
    pub valid: u32,
}
impl ControlConfiguration {
    /// Only enum semantics, not session/freshness/authorization validation.
    /// Arcade overloads the backward axis as reverse acceleration.
    pub fn safe_shifter(&self) -> bool {
        self.valid == CONTROL_CONFIGURATION_SHIFTER_VALID
            && matches!(
                self.shifter_type,
                SHIFTER_AUTOMATIC | SHIFTER_MANUAL | SHIFTER_HSHIFTER
            )
    }
}
const _: () = assert!(std::mem::size_of::<ControlConfiguration>() == 72);

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct WireHeader {
    pub sim_time_us: u64,
    pub frame_id: u64,
    pub seq: u64,
    pub qpc: u64,
    pub source: u32,
    pub flags: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct TrailerTelemetry {
    pub position_scs: [f64; 3],
    pub orientation_turns: [f32; 3],
    pub connected: u32,
    pub velocity_scs: [f32; 3],
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct TelemetryFrame {
    pub header: WireHeader,
    pub qpc_frequency: u64,
    pub render_time_us: u64,
    pub paused_simulation_time_us: u64,
    pub position_scs: [f64; 3],
    pub orientation_turns: [f32; 3],
    pub velocity_local: [f32; 3],
    pub acceleration_local: [f32; 3],
    /// SCS native unit: rotations per second (multiply by TAU for rad/s).
    pub angular_velocity_local: [f32; 3],
    pub speed_mps: f32,
    pub rpm: f32,
    pub input: [f32; 4],
    pub effective: [f32; 4],
    pub navigation_speed_limit_mps: f32,
    pub navigation_distance_m: f32,
    pub navigation_time_s: f32,
    pub brake_temperature_c: f32,
    pub brake_air_pressure_psi: f32,
    pub fuel_l: f32,
    pub cruise_control_mps: f32,
    pub local_scale: f32,
    pub gear: i32,
    pub displayed_gear: i32,
    pub paused: u32,
    pub game_time_min: u32,
    pub rest_stop_min: i32,
    pub parking_brake: u32,
    pub motor_brake: u32,
    pub retarder_level: u32,
    pub lights: u32,
    pub wipers: u32,
    /// SCS native unit: rotations, positive turns left (multiply by TAU).
    pub wheel_steering: [f32; 16],
    pub wheel_on_ground: [u32; 16],
    pub wheel_substance: [u32; 16],
    pub wear: [f32; 5],
    pub reserved: u32,
    pub trailers: [TrailerTelemetry; 10],
}
impl Default for TelemetryFrame {
    fn default() -> Self {
        Self::zeroed()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct ControlCmd {
    pub seq: u64,
    pub write_qpc: u64,
    pub source_frame_id: u64,
    pub steer: f32,
    pub throttle: f32,
    pub brake: f32,
    pub clutch: f32,
    pub holds: u64,
    pub pulses: [u32; 64],
}
impl Default for ControlCmd {
    fn default() -> Self {
        Self::zeroed()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct Gate {
    pub seq: u64,
    pub heartbeat_qpc: u64,
    pub enabled: u32,
    pub aeb_brake: f32,
    pub max_throttle: f32,
    pub max_steer_rate: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct AppliedControl {
    pub applied_seq: u64,
    pub applied_qpc: u64,
    pub frame_index: u64,
    pub axes: [f32; 4],
    pub reason: u32,
    pub reserved: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct InputDiagnostics {
    pub callback_count: u64,
    pub batch_count: u64,
    pub completed_batch_count: u64,
    pub last_begin_qpc: u64,
    pub last_end_qpc: u64,
    pub repeated_first_flags: u64,
    pub last_flags: u32,
    pub first_flags: u32,
    pub last_batch_events: u32,
    pub last_batch_callbacks: u32,
    pub max_batch_events: u32,
    pub reserved: u32,
}
/// Emitted only after the SDK consumed a finite input batch through not_found.
/// Pulse counters describe consumed edges, not merely the host's requested count.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct InputActionFrame {
    pub batch_id: u64,
    pub command_seq: u64,
    pub prepared_qpc: u64,
    pub completed_qpc: u64,
    pub source_frame_id: u64,
    pub holds: u64,
    pub pulse_mask: u64,
    pub reason: u32,
    pub reserved: u32,
    pub pulse_counters: [u32; 64],
}
impl Default for InputActionFrame {
    fn default() -> Self {
        Self::zeroed()
    }
}
/// Ordered callback evidence. kind: 1 frame_start, 2 frame_end, 3 input prepare.
/// Input records label the most recently completed physical frame; source_frame_id
/// is the frame referenced by the command. No render/physics ordering is guessed.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct FrameTiming {
    pub event_seq: u64,
    pub qpc: u64,
    pub sim_time_us: u64,
    pub physics_frame_id: u64,
    pub input_batch_id: u64,
    pub frame_start_qpc: u64,
    pub frame_end_qpc: u64,
    pub input_prepare_qpc: u64,
    pub command_seq: u64,
    pub source_frame_id: u64,
    pub kind: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct SafetyRequest {
    pub seq: u64,
    pub write_qpc: u64,
    pub arm: u32,
    pub reserved: u32,
}

/// Independent supervisor identity and the exact loaded reverse profile.
/// Only the supervisor writes this block. This is readiness evidence, never
/// an authorization to bypass Gate or a statement of driving acceptance.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct SupervisorStatus {
    pub schema_version: u32,
    pub process_id: u32,
    pub heartbeat_qpc: u64,
    pub profile_sha256: [u8; 32],
    pub configuration_sequence: u64,
    pub reverse_ready: u32,
    pub reserved: u32,
}

/// SDK metadata. `game_version` is the telemetry game API version, not the
/// displayed 1.xx game patch. `session_start_qpc` changes after a full reload.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct BridgeInfo {
    pub schema_version: u32,
    pub process_id: u32,
    pub game_version: u32,
    pub input_game_version: u32,
    pub telemetry_api_version: u32,
    pub input_api_version: u32,
    pub session_start_qpc: u64,
    pub game_id: [u8; 32],
    pub game_name: [u8; 128],
}
impl Default for BridgeInfo {
    fn default() -> Self {
        Self::zeroed()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct VideoIndex {
    pub video_frame: u64,
    pub capture_frame: u64,
    pub capture_qpc: u64,
    pub physics_frame: u64,
    pub physics_qpc: u64,
    pub sim_time_us: u64,
    pub dropped_before: u64,
    pub alignment_error_qpc: u64,
}
/// Per-index alignment evidence; VideoIndex remains the original 64-byte ABI.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct VideoAlignment {
    pub video_frame: u64,
    pub capture_frame: u64,
    pub capture_qpc: u64,
    pub recorded_qpc: u64,
    pub wait_qpc: u64,
    pub status: u32,
    pub reserved: u32,
}
pub const VIDEO_ALIGNMENT_BRACKETED: u32 = 0;
pub const VIDEO_ALIGNMENT_MISSING_PAST: u32 = 1;
pub const VIDEO_ALIGNMENT_TIMEOUT: u32 = 2;
pub const VIDEO_ALIGNMENT_TAIL: u32 = 3;
pub const VIDEO_ALIGNMENT_NO_TELEMETRY: u32 = 4;
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct VehicleCounters {
    pub header: WireHeader,
    pub odometer_km: f64,
    pub valid: u32,
    pub reserved: u32,
}

/// Pixel rectangles for the D3D11 overlay. Host projection chooses coordinates;
/// the render callback rejects nonfinite/out-of-range values and stale batches.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct OverlayRect {
    pub bounds: [f32; 4],
    pub color_rgba: [f32; 4],
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct OverlayFrame {
    pub source_frame_id: u64,
    pub write_qpc: u64,
    pub enabled: u32,
    pub count: u32,
    pub rectangles: [OverlayRect; 256],
}
impl Default for OverlayFrame {
    fn default() -> Self {
        Self::zeroed()
    }
}

/// Host-rendered heads-up display: one premultiplied RGBA8 image, top row
/// first, `width * 4` bytes per row. The renderer draws it 1:1 at `anchor` plus
/// `offset_px` and drops it once `write_qpc` is older than 500 ms, so a dead
/// host can never leave a stale "engaged" display on screen.
pub const HUD_MAPPING: &str = "ETSADS_Hud";
/// Written by the renderer: proves the HUD reached a real backbuffer and tells
/// the host the resolution to lay out for.
pub const HUD_STATUS_MAPPING: &str = "ETSADS_HudStatus";
pub const HUD_MAX_WIDTH: usize = 1024;
pub const HUD_MAX_HEIGHT: usize = 512;
pub const HUD_ANCHOR_TOP_LEFT: u32 = 0;
pub const HUD_ANCHOR_TOP_CENTER: u32 = 1;
pub const HUD_ANCHOR_TOP_RIGHT: u32 = 2;
pub const HUD_ANCHOR_BOTTOM_CENTER: u32 = 3;
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct HudImage {
    pub write_qpc: u64,
    pub sequence: u64,
    pub enabled: u32,
    pub width: u32,
    pub height: u32,
    pub anchor: u32,
    pub offset_px: [f32; 2],
    pub pixels: [u8; HUD_MAX_WIDTH * HUD_MAX_HEIGHT * 4],
}
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct HudStatus {
    pub present_qpc: u64,
    pub drawn_sequence: u64,
    pub frames_drawn: u64,
    pub backbuffer_width: u32,
    pub backbuffer_height: u32,
    pub errors: u32,
    pub reserved: u32,
}

/// The path the assist is steering along, ahead of the truck, in world ENU
/// metres (x east, y = −SCS z). The independent supervisor reads only its
/// geometry: obstacles are still taken from raw traffic, but judged along
/// the road the truck will follow instead of an arc from the current yaw
/// rate, which cut across a waiting car at a roundabout exit (r10).
pub const PLANNED_PATH_MAPPING: &str = "ETSADS_PlannedPath";
pub const PLANNED_PATH_MAX: usize = 64;
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct PlannedPath {
    pub write_qpc: u64,
    pub source_frame_id: u64,
    pub count: u32,
    pub reserved: u32,
    pub points: [[f64; 2]; PLANNED_PATH_MAX],
}
impl Default for PlannedPath {
    fn default() -> Self {
        Self::zeroed()
    }
}

/// Configuration is bounded and stores owned UTF-8, never game-owned pointers.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct ConfigBlock {
    pub config_seq: u64,
    pub wheel_positions: [[f32; 3]; 16],
    pub wheel_radius: [f32; 16],
    pub wheel_steerable: [u32; 16],
    pub hook_position: [f32; 3],
    pub cabin_position: [f32; 3],
    pub head_position: [f32; 3],
    pub retarder_steps: u32,
    pub gears_forward: u32,
    pub wheel_count: u32,
    pub cargo_mass_kg: f32,
    pub delivery_time: u32,
    pub destination_city: [u8; 64],
    pub destination_company: [u8; 64],
    pub job_market: [u8; 32],
}
impl Default for ConfigBlock {
    fn default() -> Self {
        Self::zeroed()
    }
}

pub const DYNAMICS_ANGULAR_ACCELERATION: u32 = 1;
pub const DYNAMICS_CABIN_OFFSET: u32 = 2;
pub const DYNAMICS_LIGHT_LEFT: u32 = 4;
pub const DYNAMICS_LIGHT_RIGHT: u32 = 8;
pub const DYNAMICS_AUX_FRONT: u32 = 16;
pub const DYNAMICS_AUX_ROOF: u32 = 32;
/// Same physical-frame header as TelemetryFrame. Raw SDK units are preserved.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct DynamicsFrame {
    pub header: WireHeader,
    /// Vehicle-local rotations/s², not radians/s².
    pub angular_acceleration_local: [f64; 3],
    pub cabin_offset_position: [f64; 3],
    pub cabin_offset_orientation_turns: [f32; 3],
    pub light_aux_front: u32,
    pub light_aux_roof: u32,
    /// Physical flashing lamp states; bit0 left, bit1 right.
    pub light_blinkers: u32,
    pub valid: u32,
    pub reserved: u32,
}
pub const CONFIG_HAS_HOOK: u32 = 1;
pub const CONFIG_HAS_CABIN: u32 = 2;
pub const CONFIG_HAS_HEAD: u32 = 4;
pub const CONFIG_HAS_WHEEL_COUNT: u32 = 8;
pub const CONFIG_HAS_ID: u32 = 16;
pub const CONFIG_TRUNCATED_WHEELS: u32 = 32;
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct TrailerConfiguration {
    pub update_qpc: u64,
    pub config_seq: u64,
    /// Nonempty attributes mean the slot exists; empty attributes clear it.
    pub present: u32,
    pub flags: u32,
    /// Raw reported count; >16 is retained and explicitly marked truncated.
    pub wheel_count: u32,
    pub wheel_position_mask: u32,
    pub wheel_radius_mask: u32,
    pub wheel_steerable_mask: u32,
    pub hook_position: [f32; 3],
    pub reserved: u32,
    pub wheel_positions: [[f32; 3]; 16],
    pub wheel_radius: [f32; 16],
    pub wheel_steerable: [u32; 16],
    pub id: [u8; 64],
}
impl Default for TrailerConfiguration {
    fn default() -> Self {
        Self::zeroed()
    }
}
/// Configuration event history; header.qpc is the actual SDK event QPC.
/// header.frame_id is the last completed physical frame. New values apply no
/// earlier than effective_from_frame_id; never attach them to older samples.
/// This is an association lower bound, not a measured first affected step:
/// also require header.qpc <= the telemetry frame_start QPC. An event arriving
/// within a physics step can therefore be associated one step later.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct ConfigurationFrame {
    pub header: WireHeader,
    pub effective_from_frame_id: u64,
    pub config_seq: u64,
    pub truck: ConfigBlock,
    pub truck_fields: u64,
    pub truck_wheel_position_mask: u32,
    pub truck_wheel_radius_mask: u32,
    pub truck_wheel_steerable_mask: u32,
    pub reported_trailer_mask: u32,
    pub present_trailer_mask: u32,
    pub reserved: u32,
    pub trailers: [TrailerConfiguration; 10],
}
impl Default for ConfigurationFrame {
    fn default() -> Self {
        Self::zeroed()
    }
}

pub const VIEW_UNKNOWN: u32 = 0;
/// Legacy recorded heuristic code. Its value/meaning are preserved; proximity
/// alone must not produce this label in new uncalibrated bridge observations.
pub const VIEW_INTERIOR: u32 = 1;
pub const VIEW_CHASE: u32 = 2;
pub const VIEW_OTHER: u32 = 3;
/// Within the coarse cabin-neighborhood bounds, with no calibrated camera
/// identity. Includes both driver and nearby external viewpoints; never C1 proof.
pub const VIEW_NEAR_CABIN_UNCLASSIFIED: u32 = 4;
pub const CAMERA_VIEW_POSITION_VALID: u32 = 1;
pub const CAMERA_VIEW_ORIENTATION_CANDIDATES_VALID: u32 = 2;
pub const CAMERA_VIEW_HAS_PREVIOUS: u32 = 4;
pub const CAMERA_VIEW_MISSING_CONFIG: u32 = 8;
pub const CAMERA_VIEW_MISSING_CAMERA: u32 = 16;
pub const CAMERA_VIEW_INVALID_CAMERA: u32 = 32;
pub const CAMERA_VIEW_STATIC_CABIN: u32 = 64;
pub const CAMERA_VIEW_PHYSICS_REFERENCE: u32 = 128;
pub const CAMERA_VIEW_HEURISTIC_BOUNDS: u32 = 256;
pub const CAMERA_VIEW_DISCONTINUITY: u32 = 512;
/// Derived diagnostic relative to the SDK physical cabin frame, not the
/// uncalibrated rendered bounding-box center. No cam1..8 identity is inferred.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct CameraViewFrame {
    pub header: WireHeader,
    pub config_seq: u64,
    pub relative_position_scs: [f64; 3],
    /// Candidate0: camera quaternion is local->world; candidate1: inverse.
    pub relative_quaternion_candidates_wxyz: [[f64; 4]; 2],
    pub position_delta_m: f64,
    pub rotation_delta_rad: [f64; 2],
    pub view_kind: u32,
    pub flags: u32,
    /// Bit0/bit1 are switch candidates for the two orientation hypotheses.
    pub switched_mask: u32,
    /// 0/1 if explicitly selected; u32::MAX means unresolved, never guessed.
    pub selected_hypothesis: u32,
    pub previous_frame_id: u64,
    pub session_start_qpc: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct GameEvent {
    pub header: WireHeader,
    pub name: [u8; 64],
    pub detail: [u8; 128],
}
impl Default for GameEvent {
    fn default() -> Self {
        Self::zeroed()
    }
}

/// Raw extension bytes copied on the game's main thread at frame_start. A bit
/// in `available` indicates a mapping was successfully copied for this frame.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct ExtensionFrame {
    pub header: WireHeader,
    pub available: u32,
    pub reserved: u32,
    pub camera: [u8; 128],
    pub traffic: [u8; 6960],
    pub parked: [u8; 1720],
    pub signals: [u8; 1920],
    pub route: [u8; 96000],
}
impl Default for ExtensionFrame {
    fn default() -> Self {
        Self::zeroed()
    }
}

const _: () = assert!(std::mem::size_of::<WireHeader>() == 40);
const _: () = assert!(std::mem::size_of::<ControlCmd>() == 304);
const _: () = assert!(std::mem::size_of::<Gate>() == 32);
const _: () = assert!(std::mem::size_of::<AppliedControl>() == 48);
const _: () = assert!(std::mem::size_of::<InputDiagnostics>() == 72);
const _: () = assert!(std::mem::size_of::<InputActionFrame>() == 320);
const _: () = assert!(std::mem::size_of::<FrameTiming>() == 88);
const _: () = assert!(std::mem::size_of::<TrailerTelemetry>() == 56);
const _: () = assert!(std::mem::size_of::<GameEvent>() == 232);
const _: () = assert!(std::mem::size_of::<HudImage>() == 40 + HUD_MAX_WIDTH * HUD_MAX_HEIGHT * 4);
const _: () = assert!(std::mem::size_of::<HudStatus>() == 40);
const _: () = assert!(std::mem::size_of::<TelemetryFrame>() == 1024);
const _: () = assert!(std::mem::size_of::<ConfigBlock>() == 544);
const _: () = assert!(std::mem::size_of::<DynamicsFrame>() == 120);
const _: () = assert!(std::mem::size_of::<TrailerConfiguration>() == 440);
const _: () = assert!(std::mem::size_of::<ConfigurationFrame>() == 5032);
const _: () = assert!(std::mem::size_of::<CameraViewFrame>() == 192);
const _: () = assert!(std::mem::size_of::<ExtensionFrame>() == 106776);
const _: () = assert!(std::mem::size_of::<SafetyRequest>() == 24);
const _: () = assert!(std::mem::size_of::<SupervisorStatus>() == 64);
const _: () = assert!(std::mem::size_of::<BridgeInfo>() == 192);
const _: () = assert!(std::mem::size_of::<VideoIndex>() == 64);
const _: () = assert!(std::mem::size_of::<VideoAlignment>() == 48);
const _: () = assert!(std::mem::size_of::<VehicleCounters>() == 56);
