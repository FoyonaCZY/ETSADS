//! Deterministic input policy, shared by the DLL and offline failure tests.
use etsads_msgs::protocol::{ControlCmd, Gate, GATE_ACTIVE, GATE_BRAKE_ONLY};

pub const AXIS_NAMES: [&str; 4] = ["steering", "aforward", "abackward", "clutch"];
pub const HOLD_NAMES: [&str; 3] = ["horn", "lighthorn", "motorbrake"];
pub const PULSE_NAMES: [&str; 46] = [
    "lblinker",
    "rblinker",
    "flasher4way",
    "light",
    "hblight",
    "wipers0",
    "wipers1",
    "wipers2",
    "wipers3",
    "wipers4",
    "retarder0",
    "retarder1",
    "retarder2",
    "retarder3",
    "retarder4",
    "retarder5",
    "engbraketog",
    "parkingbrake",
    "activate",
    "gearup",
    "geardown",
    "gear0",
    "geardrive",
    "gearreverse",
    "cam1",
    "cam2",
    "cam3",
    "cam4",
    "cam5",
    "cam6",
    "cam7",
    "cam8",
    "attach",
    "quicksave",
    "pause",
    "menu",
    "j_up",
    "j_down",
    "j_left",
    "j_right",
    "selectfcs",
    "back",
    "cruisctrl",
    "airhorn",
    "quickload",
    "engine",
];
pub const PULSE_INPUT_START: usize = AXIS_NAMES.len() + HOLD_NAMES.len();
pub const INPUT_COUNT: usize = PULSE_INPUT_START + PULSE_NAMES.len();

pub fn pulse_requires_stationary(index: usize) -> bool {
    matches!(PULSE_NAMES.get(index), Some(&"quickload" | &"engine"))
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Applied {
    pub axes: [f32; 4],
    pub holds: u64,
    pub reason: u32,
}
pub const REASON_ACTIVE: u32 = 0;
pub const REASON_GATE: u32 = 1;
pub const REASON_STALE: u32 = 2;
pub const REASON_INVALID: u32 = 3;
pub const REASON_PAUSED: u32 = 4;
pub const REASON_BRAKE_ONLY: u32 = 5;
pub const REASON_CONTROL_CONFIGURATION: u32 = 6;

#[derive(Default)]
pub struct InputPolicy {
    previous: Applied,
    last_qpc: u64,
    last_authorized_write_qpc: Option<u64>,
    brake_only_origin_qpc: Option<u64>,
}
impl InputPolicy {
    pub fn apply(
        &mut self,
        command: Option<&ControlCmd>,
        gate: Option<&Gate>,
        now: u64,
        frequency: u64,
        paused: bool,
    ) -> Applied {
        let frequency = frequency.max(1) as f64;
        let dt = if self.last_qpc == 0 {
            0.0
        } else {
            ((now.saturating_sub(self.last_qpc)) as f64 / frequency).clamp(0.0, 0.1) as f32
        };
        self.last_qpc = now;
        let mut output = Applied::default();
        if paused {
            output.reason = REASON_PAUSED;
        } else if let Some(gate) = gate.filter(|g| {
            matches!(g.enabled, GATE_ACTIVE | GATE_BRAKE_ONLY)
                && g.heartbeat_qpc <= now
                && (now - g.heartbeat_qpc) as f64 / frequency <= 0.3
        }) {
            let gate_valid = [gate.aeb_brake, gate.max_throttle, gate.max_steer_rate]
                .iter()
                .all(|v| v.is_finite())
                && (0.0..=1.0).contains(&gate.aeb_brake)
                && (0.0..=1.0).contains(&gate.max_throttle)
                && gate.max_steer_rate > 0.0;
            if !gate_valid {
                output.reason = REASON_INVALID;
            } else if gate.enabled == GATE_BRAKE_ONLY {
                // A recovered/future/malformed host command cannot reacquire
                // any axis or button while the supervisor loss latch is set.
                if gate.max_throttle != 0.0 || gate.aeb_brake > 0.3 {
                    output.reason = REASON_INVALID;
                } else {
                    output.reason = REASON_BRAKE_ONLY;
                    let origin = *self
                        .brake_only_origin_qpc
                        .get_or_insert(self.last_authorized_write_qpc.unwrap_or(0));
                    // A supervisor dying near the end of its retreat must not
                    // extend braking by its 300 ms heartbeat allowance. Use
                    // only a command actually accepted during ACTIVE; a newly
                    // activated SDK without that history fails closed.
                    if origin != 0 && now >= origin && (now - origin) as f64 / frequency < 1.0 {
                        output.axes[2] = gate.aeb_brake;
                    }
                }
            } else if let Some(command) = command.filter(|c| c.write_qpc <= now) {
                let age = (now - command.write_qpc) as f64 / frequency;
                if age > 0.2 {
                    output.reason = REASON_STALE;
                    if age <= 1.0 {
                        output.axes[2] = (((age - 0.2) / 0.5) as f32 * 0.3)
                            .min(0.3)
                            .max(gate.aeb_brake);
                    }
                } else if [
                    command.steer,
                    command.throttle,
                    command.brake,
                    command.clutch,
                ]
                .iter()
                .all(|v| v.is_finite())
                {
                    let max_delta = gate.max_steer_rate * dt;
                    output.axes = [
                        command.steer.clamp(-1.0, 1.0).clamp(
                            self.previous.axes[0] - max_delta,
                            self.previous.axes[0] + max_delta,
                        ),
                        command.throttle.clamp(0.0, gate.max_throttle),
                        command.brake.clamp(0.0, 1.0).max(gate.aeb_brake),
                        command.clutch.clamp(0.0, 1.0),
                    ];
                    if gate.aeb_brake > 0.0 || output.axes[2] > 0.05 {
                        output.axes[1] = 0.0;
                    }
                    output.holds = command.holds & 7;
                    self.last_authorized_write_qpc = Some(command.write_qpc);
                    self.brake_only_origin_qpc = None;
                } else {
                    output.reason = REASON_INVALID;
                }
            } else {
                output.reason = REASON_STALE;
            }
        } else {
            output.reason = REASON_GATE;
        }
        if matches!(output.reason, REASON_GATE | REASON_INVALID | REASON_PAUSED) {
            self.last_authorized_write_qpc = None;
            self.brake_only_origin_qpc = None;
        }
        self.previous = output;
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn legacy_button_indexes_are_frozen_and_new_inputs_are_appended() {
        let legacy = [
            "lblinker",
            "rblinker",
            "flasher4way",
            "light",
            "hblight",
            "wipers0",
            "wipers1",
            "wipers2",
            "wipers3",
            "wipers4",
            "retarder0",
            "retarder1",
            "retarder2",
            "retarder3",
            "retarder4",
            "retarder5",
            "engbraketog",
            "parkingbrake",
            "activate",
            "gearup",
            "geardown",
            "gear0",
            "geardrive",
            "gearreverse",
            "cam1",
            "cam2",
            "cam3",
            "cam4",
            "cam5",
            "cam6",
            "cam7",
            "cam8",
            "attach",
            "quicksave",
            "pause",
            "menu",
            "j_up",
            "j_down",
            "j_left",
            "j_right",
            "selectfcs",
            "back",
            "cruisctrl",
            "airhorn",
        ];
        assert_eq!(&PULSE_NAMES[..legacy.len()], &legacy);
        assert_eq!(PULSE_NAMES[legacy.len()], "quickload");
        assert_eq!(PULSE_NAMES[legacy.len() + 1], "engine");
        assert!(pulse_requires_stationary(legacy.len()));
        assert!(pulse_requires_stationary(legacy.len() + 1));
        assert!(!pulse_requires_stationary(0));
        assert_eq!(PULSE_INPUT_START, 7);
        assert!(PULSE_NAMES.len() <= 64);
        assert_eq!(std::mem::size_of::<ControlCmd>(), 304);
    }
    fn gate(t: u64) -> Gate {
        Gate {
            heartbeat_qpc: t,
            enabled: 1,
            max_throttle: 1.0,
            max_steer_rate: 2.0,
            ..Default::default()
        }
    }
    #[test]
    fn watchdog_boundaries() {
        let mut p = InputPolicy::default();
        let c = ControlCmd {
            write_qpc: 1000,
            throttle: 1.0,
            ..Default::default()
        };
        assert_eq!(
            p.apply(Some(&c), Some(&gate(1000)), 1200, 1000, false).axes[1],
            1.0
        );
        let a = p.apply(Some(&c), Some(&gate(1500)), 1500, 1000, false);
        assert!((a.axes[2] - 0.18).abs() < 1e-6);
        assert_eq!(a.axes[1], 0.0);
        assert_eq!(
            p.apply(Some(&c), Some(&gate(2101)), 2101, 1000, false).axes,
            [0.0; 4]
        );
        assert_eq!(
            p.apply(Some(&c), Some(&gate(1000)), 1301, 1000, false)
                .reason,
            REASON_GATE
        );
    }
    #[test]
    fn pause_invalid_and_aeb_release_throttle() {
        let mut p = InputPolicy::default();
        let mut c = ControlCmd {
            write_qpc: 1000,
            throttle: 1.0,
            ..Default::default()
        };
        let mut g = gate(1000);
        g.aeb_brake = 0.7;
        assert_eq!(
            p.apply(Some(&c), Some(&g), 1000, 1000, false).axes,
            [0.0, 0.0, 0.7, 0.0]
        );
        c.steer = f32::NAN;
        assert_eq!(
            p.apply(Some(&c), Some(&g), 1000, 1000, false).reason,
            REASON_INVALID
        );
        assert_eq!(p.apply(Some(&c), Some(&g), 1000, 1000, true).axes, [0.0; 4]);
    }
    #[test]
    fn limits_steer_rate_and_rejects_future_heartbeat() {
        let mut p = InputPolicy::default();
        let mut c = ControlCmd {
            write_qpc: 1000,
            steer: 1.0,
            ..Default::default()
        };
        p.apply(Some(&c), Some(&gate(1000)), 1000, 1000, false);
        c.write_qpc = 1050;
        assert!(
            (p.apply(Some(&c), Some(&gate(1050)), 1050, 1000, false).axes[0] - 0.1).abs() < 1e-6
        );
        assert_eq!(
            p.apply(Some(&c), Some(&gate(2000)), 1100, 1000, false)
                .reason,
            REASON_GATE
        );
    }
    #[test]
    fn brake_only_ignores_all_host_inputs_and_unknown_modes_release() {
        let command = ControlCmd {
            write_qpc: 1000,
            steer: 1.0,
            throttle: 1.0,
            clutch: 1.0,
            holds: 7,
            ..Default::default()
        };
        let mut policy = InputPolicy::default();
        let mut gate = gate(1000);
        policy.apply(Some(&command), Some(&gate), 1000, 1000, false);
        gate.enabled = GATE_BRAKE_ONLY;
        gate.max_throttle = 0.0;
        gate.aeb_brake = 0.18;
        gate.heartbeat_qpc = 1500;
        let applied = policy.apply(Some(&command), Some(&gate), 1500, 1000, false);
        assert_eq!(applied.axes, [0.0, 0.0, 0.18, 0.0]);
        assert_eq!(applied.holds, 0);
        assert_eq!(applied.reason, REASON_BRAKE_ONLY);
        let mut invalid = command;
        invalid.write_qpc = 5000;
        invalid.steer = f32::NAN;
        assert_eq!(
            policy
                .apply(Some(&invalid), Some(&gate), 1500, 1000, false)
                .axes,
            [0.0, 0.0, 0.18, 0.0]
        );
        assert_eq!(
            policy.apply(None, Some(&gate), 1500, 1000, true).axes,
            [0.0; 4]
        );
        assert_eq!(
            policy.apply(None, Some(&gate), 1801, 1000, false).axes,
            [0.0; 4]
        );
        gate.enabled = 999;
        assert_eq!(
            policy
                .apply(Some(&command), Some(&gate), 1500, 1000, false)
                .axes,
            [0.0; 4]
        );
    }
    #[test]
    fn brake_only_never_extends_past_original_command_age_even_with_fresh_gate() {
        let mut policy = InputPolicy::default();
        let mut command = ControlCmd {
            write_qpc: 1000,
            throttle: 0.2,
            ..Default::default()
        };
        policy.apply(Some(&command), Some(&gate(1000)), 1000, 1000, false);
        let mut fallback = gate(1700);
        fallback.enabled = GATE_BRAKE_ONLY;
        fallback.max_throttle = 0.0;
        fallback.aeb_brake = 0.3;
        assert_eq!(
            policy
                .apply(Some(&command), Some(&fallback), 1700, 1000, false)
                .axes[2],
            0.3
        );
        command.write_qpc = 1999;
        fallback.heartbeat_qpc = 1999;
        assert_eq!(
            policy
                .apply(Some(&command), Some(&fallback), 1999, 1000, false)
                .axes[2],
            0.3
        );
        assert_eq!(
            policy
                .apply(Some(&command), Some(&fallback), 2000, 1000, false)
                .axes,
            [0.0; 4]
        );
        assert_eq!(
            InputPolicy::default()
                .apply(Some(&command), Some(&fallback), 1999, 1000, false)
                .axes,
            [0.0; 4]
        );
    }
}
