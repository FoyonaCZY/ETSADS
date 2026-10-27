//! Independent fail-closed safety policy. Wall-clock freshness must use QPC,
//! because the simulation clock intentionally freezes in menus and on pause.
use etsads_msgs::protocol::{
    ConfigBlock, ControlCmd, ControlConfiguration, ExtensionFrame, GATE_ACTIVE, GATE_BRAKE_ONLY,
    Gate, TelemetryFrame,
};
pub mod controls;
pub use controls::validate_control_configuration;
pub mod reverse;
pub use reverse::SafetyGeometryProfile;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    Disarmed,
    MissingTelemetry,
    StaleTelemetry,
    InvalidTelemetry,
    Paused,
    MissingCommand,
    StaleCommand,
    CommandLostBraking,
    InvalidCommand,
    ManualTakeover,
    EmergencyStop,
    ReverseUnverified,
    ReverseSpeedLimit,
    ReverseConfigurationChanged,
    UnsafeControlConfiguration,
}
pub struct SafetyInputs<'a> {
    pub telemetry: Option<&'a TelemetryFrame>,
    pub command: Option<&'a ControlCmd>,
    pub configuration: Option<&'a ConfigBlock>,
    pub profile: Option<&'a SafetyGeometryProfile>,
    pub manual: bool,
    pub obstacle: Option<Obstacle>,
}

#[derive(Clone, Copy, Debug)]
pub struct Obstacle {
    pub distance_m: f32,
    pub closing_speed_mps: f32,
}

#[derive(Default)]
pub struct Supervisor {
    armed: bool,
    sequence: u64,
    last_frame: u64,
    last_sim_time: u64,
    attachment_mask: Option<u16>,
    attachment_changed: bool,
    last_authorized_command: Option<(u64, u64)>,
    command_loss_origin_qpc: Option<u64>,
    control_configuration: Option<ControlConfiguration>,
    control_session: u64,
}

impl Supervisor {
    /// Consume the newest SDK event, never an older safe as-of value. Changes
    /// revoke a previous arm/loss-retreat; a subsequent explicit arm is needed.
    pub fn observe_control_configuration(
        &mut self,
        current: Option<ControlConfiguration>,
        session: u64,
    ) {
        if self.control_configuration.is_some_and(|old| {
            current.is_none_or(|new| {
                old.session_start_qpc != new.session_start_qpc
                    || old.generation != new.generation
                    || old.shifter_type != new.shifter_type
                    || old.valid != new.valid
            }) || self.control_session != session
        }) {
            self.disarm();
        }
        self.control_configuration = current;
        self.control_session = session;
    }
    pub fn arm(&mut self) {
        self.armed = true;
        self.last_authorized_command = None;
        self.command_loss_origin_qpc = None;
    }
    pub fn disarm(&mut self) {
        self.armed = false;
        self.last_frame = 0;
        self.last_sim_time = 0;
        self.last_authorized_command = None;
        self.command_loss_origin_qpc = None;
    }
    pub fn is_armed(&self) -> bool {
        self.armed
    }
    pub fn is_braking_after_loss(&self) -> bool {
        self.command_loss_origin_qpc.is_some()
    }
    /// Geometry readiness is distinct from arming and obstacle clearance.
    /// Hosts must still obey Gate and the raw-obstacle AEB result.
    pub fn reverse_profile_ready(
        &self,
        telemetry: &TelemetryFrame,
        configuration: &ConfigBlock,
        profile: &SafetyGeometryProfile,
    ) -> bool {
        !self.attachment_changed
            && profile.validate(configuration).is_ok()
            && usize::from(profile.trailer.is_some())
                == telemetry
                    .trailers
                    .iter()
                    .filter(|t| t.connected != 0)
                    .count()
    }
    pub fn evaluate(
        &mut self,
        now: u64,
        frequency: u64,
        telemetry: Option<&TelemetryFrame>,
        command: Option<&ControlCmd>,
        manual: bool,
        obstacle: Option<Obstacle>,
    ) -> (Gate, Option<StopReason>) {
        self.evaluate_profiled(
            now,
            frequency,
            SafetyInputs {
                telemetry,
                command,
                configuration: None,
                profile: None,
                manual,
                obstacle,
            },
        )
    }
    pub fn evaluate_profiled(
        &mut self,
        now: u64,
        frequency: u64,
        inputs: SafetyInputs<'_>,
    ) -> (Gate, Option<StopReason>) {
        let SafetyInputs {
            telemetry,
            command,
            configuration,
            profile,
            manual,
            obstacle,
        } = inputs;
        self.sequence = self.sequence.wrapping_add(1);
        let mut gate = Gate {
            seq: self.sequence,
            heartbeat_qpc: now,
            enabled: 0,
            aeb_brake: 0.0,
            max_throttle: 0.0,
            max_steer_rate: 0.8,
        };
        if telemetry.is_some_and(|t| {
            validate_control_configuration(
                self.control_configuration.as_ref(),
                self.control_session,
                t,
                now,
            )
            .is_err()
        }) {
            self.disarm();
            return (gate, Some(StopReason::UnsafeControlConfiguration));
        }
        if let Some(origin) = self.command_loss_origin_qpc {
            if let Some(reason) = self.telemetry_reason(now, frequency, telemetry, manual) {
                self.disarm();
                return (gate, Some(reason));
            }
            return self.brake_after_loss(gate, now, frequency, origin, telemetry.unwrap());
        }
        let reason = self.reason(now, frequency, telemetry, command, manual);
        if let Some(reason) = reason {
            if reason == StopReason::StaleCommand
                && let Some(c) = command
                && self.last_authorized_command == Some((c.seq, c.write_qpc))
                && c.write_qpc <= now
            {
                self.armed = false;
                self.command_loss_origin_qpc = Some(c.write_qpc);
                return self.brake_after_loss(
                    gate,
                    now,
                    frequency,
                    c.write_qpc,
                    telemetry.unwrap(),
                );
            }
            self.disarm();
            return (gate, Some(reason));
        }
        let telemetry = telemetry.expect("validated telemetry");
        let mask = telemetry
            .trailers
            .iter()
            .enumerate()
            .fold(0_u16, |mask, (i, t)| {
                if t.connected != 0 {
                    mask | (1 << i)
                } else {
                    mask
                }
            });
        if profile.is_some() {
            if self
                .attachment_mask
                .is_some_and(|previous| previous != mask)
            {
                self.attachment_changed = true;
            }
            self.attachment_mask.get_or_insert(mask);
        }
        self.last_frame = telemetry.header.frame_id;
        self.last_sim_time = telemetry.header.sim_time_us;
        let command = command.expect("validated command");
        self.last_authorized_command = Some((command.seq, command.write_qpc));
        gate.enabled = GATE_ACTIVE;
        gate.max_throttle = if telemetry.navigation_speed_limit_mps > 0.0
            && telemetry.speed_mps > telemetry.navigation_speed_limit_mps
        {
            0.0
        } else {
            1.0
        };
        if telemetry.paused != 0 {
            gate.max_throttle = 0.0;
        }
        let reversing = telemetry.gear < 0 || telemetry.speed_mps < -0.02;
        let mut restriction = None;
        if reversing {
            let allowed = profile
                .zip(configuration)
                .is_some_and(|(p, c)| self.reverse_profile_ready(telemetry, c, p));
            if !allowed {
                gate.max_throttle = 0.0;
                if telemetry.speed_mps.abs() > 0.05 {
                    gate.aeb_brake = 0.5;
                }
                restriction = Some(if self.attachment_changed {
                    StopReason::ReverseConfigurationChanged
                } else {
                    StopReason::ReverseUnverified
                });
            }
            if telemetry.speed_mps.abs() > 3.0 / 3.6 {
                gate.max_throttle = 0.0;
                gate.aeb_brake = gate.aeb_brake.max(0.4);
                restriction = Some(StopReason::ReverseSpeedLimit);
            }
        }
        if let Some(obstacle) = obstacle {
            if !obstacle.distance_m.is_finite()
                || !obstacle.closing_speed_mps.is_finite()
                || obstacle.distance_m <= 0.0
            {
                gate.aeb_brake = 1.0;
            } else if obstacle.closing_speed_mps > 0.0 {
                let ttc = obstacle.distance_m / obstacle.closing_speed_mps;
                let margin = if reversing { 0.5 } else { 3.0 };
                let required = obstacle.closing_speed_mps.powi(2)
                    / (2.0 * (obstacle.distance_m - margin).max(0.1));
                if ttc < if reversing { 1.0 } else { 2.0 }
                    || required > if reversing { 1.0 } else { 3.0 }
                {
                    gate.aeb_brake = gate.aeb_brake.max((required / 4.0).clamp(0.3, 1.0));
                }
            }
            if gate.aeb_brake > 0.0 {
                gate.max_throttle = 0.0;
            }
        }
        (gate, restriction)
    }
    fn brake_after_loss(
        &mut self,
        mut gate: Gate,
        now: u64,
        frequency: u64,
        origin: u64,
        telemetry: &TelemetryFrame,
    ) -> (Gate, Option<StopReason>) {
        if frequency == 0
            || now < origin
            || (now - origin) as u128 * 1000 >= frequency as u128 * 1000
        {
            self.disarm();
            return (gate, Some(StopReason::StaleCommand));
        }
        self.last_frame = telemetry.header.frame_id;
        self.last_sim_time = telemetry.header.sim_time_us;
        gate.enabled = GATE_BRAKE_ONLY;
        gate.max_throttle = 0.0;
        let age = (now - origin) as f64 / frequency as f64;
        gate.aeb_brake = (((age - 0.2) / 0.5) * 0.3).clamp(0.0, 0.3) as f32;
        (gate, Some(StopReason::CommandLostBraking))
    }
    fn reason(
        &self,
        now: u64,
        frequency: u64,
        telemetry: Option<&TelemetryFrame>,
        command: Option<&ControlCmd>,
        manual: bool,
    ) -> Option<StopReason> {
        if manual {
            return Some(StopReason::ManualTakeover);
        }
        if !self.armed {
            return Some(StopReason::Disarmed);
        }
        if let Some(reason) = self.telemetry_reason(now, frequency, telemetry, manual) {
            return Some(reason);
        }
        let t = telemetry.expect("validated telemetry");
        let Some(c) = command else {
            return Some(StopReason::MissingCommand);
        };
        if c.write_qpc == 0 || c.write_qpc > now {
            return Some(StopReason::InvalidCommand);
        }
        if c.seq == 0
            || ![c.steer, c.throttle, c.brake, c.clutch]
                .iter()
                .all(|v| v.is_finite())
            || c.steer.abs() > 1.0
            || !(0.0..=1.0).contains(&c.throttle)
            || !(0.0..=1.0).contains(&c.brake)
            || !(0.0..=1.0).contains(&c.clutch)
            || (c.throttle > 0.0 && c.brake > 0.0)
            || c.source_frame_id > t.header.frame_id
        {
            return Some(StopReason::InvalidCommand);
        }
        if !fresh(now, c.write_qpc, frequency, 200)
            || t.header.frame_id.saturating_sub(c.source_frame_id) > 12
        {
            return Some(StopReason::StaleCommand);
        }
        None
    }
    fn telemetry_reason(
        &self,
        now: u64,
        frequency: u64,
        telemetry: Option<&TelemetryFrame>,
        manual: bool,
    ) -> Option<StopReason> {
        if manual {
            return Some(StopReason::ManualTakeover);
        }
        let Some(t) = telemetry else {
            return Some(StopReason::MissingTelemetry);
        };
        if !fresh(now, t.header.qpc, frequency, 200) {
            return Some(StopReason::StaleTelemetry);
        }
        if t.header.frame_id == 0
            || t.header.frame_id < self.last_frame
            || t.header.sim_time_us < self.last_sim_time
            || !t.speed_mps.is_finite()
            || t.speed_mps.abs() > 70.0
            || !t.navigation_speed_limit_mps.is_finite()
            || !(0.0..=70.0).contains(&t.navigation_speed_limit_mps)
            || !t
                .position_scs
                .iter()
                .all(|x| x.is_finite() && x.abs() < 1e7)
            || !t.input.iter().all(|x| x.is_finite())
            || !t.effective.iter().all(|x| x.is_finite())
            || !t.orientation_turns.iter().all(|x| x.is_finite())
            || !t.angular_velocity_local.iter().all(|x| x.is_finite())
            || !t.acceleration_local.iter().all(|x| x.is_finite())
            || !t.velocity_local.iter().all(|x| x.is_finite())
        {
            return Some(StopReason::InvalidTelemetry);
        }
        // A pause refuses engagement, but an engaged drive holds through it:
        // the game ignores inputs while paused, and releasing let the truck
        // coast on without a driver when the game resumed (L7, L8).
        if t.paused != 0
            && (self.last_authorized_command.is_none() || self.command_loss_origin_qpc.is_some())
        {
            return Some(StopReason::Paused);
        }
        None
    }
}

pub fn fresh(now: u64, timestamp: u64, frequency: u64, limit_ms: u64) -> bool {
    frequency > 0
        && timestamp > 0
        && timestamp <= now
        && (now - timestamp) as u128 * 1000 <= frequency as u128 * limit_ms as u128
}

/// Independent raw-traffic AEB path: no fused Scene, controller or planner is
/// linked. Packed ETS2LA layouts are decoded bytewise (never unaligned refs).
/// Returns an error on corrupt objects so the caller can release control.
pub fn primary_obstacle(
    ego: &TelemetryFrame,
    extension: &ExtensionFrame,
) -> Result<Option<Obstacle>, &'static str> {
    primary_obstacle_along(ego, extension, None)
}
/// As `primary_obstacle`, judged along the assist's planned path when it is
/// fresh and the truck is on it; otherwise along the yaw-rate arc.
pub fn primary_obstacle_along(
    ego: &TelemetryFrame,
    extension: &ExtensionFrame,
    planned: Option<&etsads_msgs::protocol::PlannedPath>,
) -> Result<Option<Obstacle>, &'static str> {
    if extension.available & 2 == 0 {
        return Err("privileged traffic unavailable");
    }
    if !fresh(ego.header.qpc, extension.header.qpc, ego.qpc_frequency, 200) {
        return Err("traffic timestamp is stale or invalid");
    }
    if extension.header.frame_id > ego.header.frame_id
        || ego
            .header
            .frame_id
            .saturating_sub(extension.header.frame_id)
            > 3
    {
        return Err("traffic/telemetry frame mismatch");
    }
    let yaw = ego.orientation_turns[0] as f64 * std::f64::consts::TAU + std::f64::consts::FRAC_PI_2;
    if !yaw.is_finite() || !ego.speed_mps.is_finite() {
        return Err("invalid ego pose");
    }
    let kappa = ego_path_curvature(ego);
    let corridor = planned
        .and_then(|p| Corridor::planned(ego, yaw, p))
        .unwrap_or(Corridor::Arc(kappa));
    let mut candidates = Vec::new();
    for record in extension.traffic.as_chunks::<174>().0 {
        let id = i16::from_le_bytes([record[50], record[51]]);
        if id < 0 || record.iter().all(|b| *b == 0) {
            continue;
        }
        let speed = read_f32(record, 40);
        if !speed.is_finite() || speed.abs() > 70.0 {
            return Err("invalid traffic speed");
        }
        if let Some(o) = obstacle_box(ego, yaw, &corridor, record, speed)? {
            candidates.push(o);
        }
        let count = i16::from_le_bytes([record[48], record[49]]);
        if !(0..=3).contains(&count) {
            return Err("invalid trailer count");
        }
        for trailer in record[54..].as_chunks::<40>().0.iter().take(count as usize) {
            if let Some(o) = obstacle_box(ego, yaw, &corridor, trailer, speed)? {
                candidates.push(o);
            }
        }
    }
    if extension.available & 4 != 0 {
        for record in extension.parked.as_chunks::<43>().0 {
            let id = i16::from_le_bytes([record[40], record[41]]);
            if id < 0 || record.iter().all(|b| *b == 0) {
                continue;
            }
            if let Some(o) = obstacle_box(ego, yaw, &corridor, record, 0.0)? {
                candidates.push(o);
            }
        }
    }
    Ok(candidates.into_iter().min_by(|a, b| {
        let risk = |o: &Obstacle| o.distance_m.max(0.01) / o.closing_speed_mps.max(0.1);
        risk(a).total_cmp(&risk(b))
    }))
}
fn read_f32(data: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(
        data[offset..offset + 4]
            .try_into()
            .expect("fixed validated layout"),
    )
}
/// Signed curvature (1/m, positive = left) of the path the ego is on now,
/// from the ego's own telemetry only. Moving: yaw rate over speed. Slow or
/// stopped: mean steered-wheel angle over a nominal tractor wheelbase. A
/// straight corridor on a bend sweeps the oncoming lane and brakes for every
/// approaching car (r9 field log: TTC 1.8 s false alarms at standstill).
pub fn ego_path_curvature(ego: &TelemetryFrame) -> f64 {
    const NOMINAL_WHEELBASE_M: f64 = 3.8;
    let speed = ego.speed_mps as f64;
    let kappa = if speed.abs() > 1.5 {
        ego.angular_velocity_local[1] as f64 * std::f64::consts::TAU / speed
    } else {
        let (sum, count) = ego
            .wheel_steering
            .iter()
            .map(|v| *v as f64)
            .filter(|v| v.is_finite() && v.abs() > 1e-6 && v.abs() < 0.25)
            .fold((0.0, 0), |(s, n), v| (s + v, n + 1));
        if count == 0 {
            0.0
        } else {
            (sum / count as f64 * std::f64::consts::TAU).tan() / NOMINAL_WHEELBASE_M
        }
    };
    if kappa.is_finite() {
        kappa.clamp(-1.0 / 12.0, 1.0 / 12.0)
    } else {
        0.0
    }
}
/// Station, signed offset (left positive) and heading change of a point given
/// in the ego frame, relative to an arc of curvature `kappa` from the ego.
fn along_path(longitudinal: f64, lateral: f64, kappa: f64) -> Option<(f64, f64, f64)> {
    if kappa.abs() < 1e-5 {
        return (longitudinal > 0.0).then_some((longitudinal, lateral, 0.0));
    }
    // Solve the left-turn case; a right turn is its mirror image.
    let sign = kappa.signum();
    let radius = 1.0 / kappa.abs();
    let (cx, cy) = (longitudinal, lateral * sign - radius);
    let swept = cx.atan2(-cy);
    if !(swept > 0.0 && swept < std::f64::consts::PI) {
        return None;
    }
    let station = radius * swept;
    Some((station, sign * (radius - cx.hypot(cy)), kappa * station))
}
/// The ego's future path used to judge obstacles.
enum Corridor {
    /// Constant curvature (1/m, left positive) from the ego.
    Arc(f64),
    /// The assist's planned path relative to the ego (ENU metres), with the
    /// ego's own station on it and a far straight extension past its end.
    Path {
        points: Vec<[f64; 2]>,
        stations: Vec<f64>,
        ego_s: f64,
    },
}
impl Corridor {
    fn planned(
        ego: &TelemetryFrame,
        yaw: f64,
        planned: &etsads_msgs::protocol::PlannedPath,
    ) -> Option<Self> {
        let count = planned.count as usize;
        // The path is written just after the frame it was planned on, so it
        // may be a little newer than this telemetry; at most 250 ms apart.
        let apart_ms = ego.header.qpc.abs_diff(planned.write_qpc) as u128 * 1000
            / (ego.qpc_frequency.max(1) as u128);
        if !(2..=etsads_msgs::protocol::PLANNED_PATH_MAX).contains(&count)
            || planned.write_qpc == 0
            || ego.qpc_frequency == 0
            || apart_ms > 250
            || planned.source_frame_id > ego.header.frame_id
        {
            return None;
        }
        let (ex, ey) = (ego.position_scs[0], -ego.position_scs[2]);
        let mut points: Vec<[f64; 2]> = planned.points[..count]
            .iter()
            .map(|p| [p[0] - ex, p[1] - ey])
            .collect();
        if points
            .iter()
            .flatten()
            .any(|v| !v.is_finite() || v.abs() > 1e4)
        {
            return None;
        }
        let [a, b] = [points[count - 2], points[count - 1]];
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        if length < 1e-3 {
            return None;
        }
        points.push([
            b[0] + (b[0] - a[0]) / length * 250.0,
            b[1] + (b[1] - a[1]) / length * 250.0,
        ]);
        let mut stations = vec![0.0];
        for w in points.windows(2) {
            let d = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
            if d < 1e-6 {
                return None;
            }
            stations.push(stations.last().unwrap() + d);
        }
        let mut corridor = Self::Path {
            points,
            stations,
            ego_s: 0.0,
        };
        // Only a path the truck is actually on stands in for its motion.
        let (s, d, heading) = corridor.project(0.0, 0.0)?;
        let turn = (heading - yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        if d.abs() > 1.5 || turn.abs() > 0.35 {
            return None;
        }
        if let Self::Path { ego_s, .. } = &mut corridor {
            *ego_s = s;
        }
        Some(corridor)
    }
    /// Station, signed offset (left positive) and heading of the nearest
    /// path point to (x, y), ego-relative ENU.
    fn project(&self, x: f64, y: f64) -> Option<(f64, f64, f64)> {
        let Self::Path {
            points, stations, ..
        } = self
        else {
            return None;
        };
        let mut best = None::<(f64, (f64, f64, f64))>;
        for (i, w) in points.windows(2).enumerate() {
            let (dx, dy) = (w[1][0] - w[0][0], w[1][1] - w[0][1]);
            let length = stations[i + 1] - stations[i];
            let t = (((x - w[0][0]) * dx + (y - w[0][1]) * dy) / (length * length)).clamp(0.0, 1.0);
            let (px, py) = (w[0][0] + t * dx, w[0][1] + t * dy);
            let distance = (x - px).hypot(y - py);
            if best.is_none_or(|(b, _)| distance < b) {
                let d = (dx * (y - w[0][1]) - dy * (x - w[0][0])) / length;
                best = Some((distance, (stations[i] + t * length, d, dy.atan2(dx))));
            }
        }
        best.map(|b| b.1)
    }
    /// Station ahead of the ego, signed offset and heading change of a point
    /// given ego-relative in ENU; `None` when it is not ahead.
    fn locate(&self, dx: f64, dy: f64, yaw: f64) -> Option<(f64, f64, f64)> {
        match self {
            Self::Arc(kappa) => along_path(
                dx * yaw.cos() + dy * yaw.sin(),
                -dx * yaw.sin() + dy * yaw.cos(),
                *kappa,
            ),
            Self::Path { ego_s, .. } => {
                let (s, d, heading) = self.project(dx, dy)?;
                (s > *ego_s).then_some((s - ego_s, d, heading - yaw))
            }
        }
    }
}
fn obstacle_box(
    ego: &TelemetryFrame,
    yaw: f64,
    corridor: &Corridor,
    data: &[u8],
    speed: f32,
) -> Result<Option<Obstacle>, &'static str> {
    let values: [f64; 10] = std::array::from_fn(|i| read_f32(data, 4 * i) as f64);
    if values.iter().any(|v| !v.is_finite()) || values[..3].iter().any(|v| v.abs() > 1e7) {
        return Err("invalid obstacle coordinates");
    }
    let [x, y, z, qw, qx, qy, qz, width, height, length] = values;
    let norm = qw * qw + qx * qx + qy * qy + qz * qz;
    if !(0.8..=1.2).contains(&norm)
        || !(0.1..20.0).contains(&width)
        || !(0.1..30.0).contains(&height)
        || !(0.1..100.0).contains(&length)
    {
        return Err("invalid obstacle shape/quaternion");
    }
    // Ego height is not known without an independently verified geometry
    // profile. A fixed centre-height threshold can hide a tall overlapping body.
    // The profiled path performs vertical interval separation instead.
    // Only the unambiguous case is excluded: a body entirely more than
    // STACKED_ROAD_M above or below the ego reference is on another level of
    // a bridge or interchange (r9: full braking under an overpass, 19 m).
    let rise = y - ego.position_scs[1];
    if rise - height * 0.5 > STACKED_ROAD_M || rise + height * 0.5 < -STACKED_ROAD_M {
        return Ok(None);
    }
    let dx = x - ego.position_scs[0];
    let dy = -(z - ego.position_scs[2]);
    let Some((station, offset, turn)) = corridor.locate(dx, dy, yaw) else {
        return Ok(None);
    };
    // A body whose top is well below the road ahead, as the ego's own pitch
    // projects it, is on a road passing underneath: a box van crossing 9 m
    // under a bridge reached within 6 m and braked the truck on the bridge
    // at 40 km/h, four times on one evening (live). The margin covers a
    // crest or sag and pitch on the suspension, and grows so that far ahead
    // only STACKED_ROAD_M applies; so does an implausible pitch.
    let pitch = ego.orientation_turns[1] as f64 * std::f64::consts::TAU;
    let ahead = station.max(0.0);
    if pitch.abs() < 0.2
        && rise + height * 0.5
            < ahead * pitch.tan() - (UNDERPASS_MARGIN_M + UNDERPASS_MARGIN_PER_M * ahead)
    {
        return Ok(None);
    }
    // Extents and closing speed are taken along the path's local heading.
    let yaw = yaw + turn;
    let fx = -2.0 * (qx * qz + qw * qy) / norm;
    let fy = (1.0 - 2.0 * (qx * qx + qy * qy) / norm).clamp(-1.0, 1.0);
    let forward = fx * yaw.cos() + fy * yaw.sin();
    let side = -fx * yaw.sin() + fy * yaw.cos();
    let right = [
        1.0 - 2.0 * (qy * qy + qz * qz) / norm,
        -2.0 * (qx * qz - qw * qy) / norm,
    ];
    let up = [
        2.0 * (qx * qy - qw * qz) / norm,
        -2.0 * (qy * qz + qw * qx) / norm,
    ];
    let lateral_extent = length * 0.5 * side.abs()
        + width * 0.5 * (-right[0] * yaw.sin() + right[1] * yaw.cos()).abs()
        + height * 0.5 * (-up[0] * yaw.sin() + up[1] * yaw.cos()).abs();
    if station > 250.0 || offset.abs() > 1.4 + lateral_extent {
        return Ok(None);
    }
    let front_extent = length * 0.5 * forward.abs()
        + width * 0.5 * (right[0] * yaw.cos() + right[1] * yaw.sin()).abs()
        + height * 0.5 * (up[0] * yaw.cos() + up[1] * yaw.sin()).abs();
    let distance = station - front_extent - 3.2;
    let closing = ego.speed_mps as f64 - speed as f64 * forward;
    // Oncoming bodies: braking only mitigates while the ego still moves, and
    // without lanes a single-curvature path cannot tell an S-bend's oncoming
    // lane from our own beyond a short horizon (r9: repeated full braking at
    // standstill and at 19 km/h on a bend). The map-aware planner handles an
    // oncoming vehicle in our lane earlier; this keeps the last-resort stop.
    if forward < -0.5
        && distance > 0.0
        && (ego.speed_mps < ONCOMING_MIN_EGO_SPEED
            || closing <= 0.0
            || distance / closing > ONCOMING_TTC_S)
    {
        return Ok(None);
    }
    Ok(Some(Obstacle {
        distance_m: distance as f32,
        closing_speed_mps: closing as f32,
    }))
}
/// No truck is this tall: anything beyond it is on another road level.
const STACKED_ROAD_M: f64 = 6.0;
/// Clearance below the projected road ahead past which a body is on a lower
/// road: 1.5 m at the truck, plus 0.06 m per metre ahead.
const UNDERPASS_MARGIN_M: f64 = 1.5;
const UNDERPASS_MARGIN_PER_M: f64 = 0.06;
const ONCOMING_MIN_EGO_SPEED: f32 = 2.0;
const ONCOMING_TTC_S: f64 = 1.2;

#[cfg(test)]
mod tests {
    use super::*;
    fn loaded_automatic() -> ControlConfiguration {
        ControlConfiguration {
            header: etsads_msgs::protocol::WireHeader {
                qpc: 900,
                seq: 1,
                ..Default::default()
            },
            session_start_qpc: 1,
            generation: 1,
            effective_from_frame_id: 1,
            shifter_type: etsads_msgs::protocol::SHIFTER_AUTOMATIC,
            valid: 1,
        }
    }
    fn fixture() -> (Supervisor, TelemetryFrame, ControlCmd) {
        let mut s = Supervisor::default();
        s.observe_control_configuration(Some(loaded_automatic()), 1);
        s.arm();
        let mut t = TelemetryFrame::default();
        t.header.qpc = 1000;
        t.header.frame_id = 1;
        let c = ControlCmd {
            seq: 1,
            write_qpc: 1000,
            source_frame_id: 1,
            ..Default::default()
        };
        (s, t, c)
    }
    #[test]
    fn expires_and_cannot_rearm_implicitly() {
        let (mut s, t, c) = fixture();
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), false, None)
                .0
                .enabled,
            1
        );
        assert_eq!(
            s.evaluate(1201, 1000, Some(&t), Some(&c), false, None).1,
            Some(StopReason::StaleTelemetry)
        );
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), false, None).1,
            Some(StopReason::Disarmed)
        );
    }
    #[test]
    fn unsafe_controls_revoke_active_aeb_and_latched_brake_only() {
        for loss_retreat in [false, true] {
            let (mut s, mut t, c) = fixture();
            assert_eq!(
                s.evaluate(1000, 1000, Some(&t), Some(&c), false, None)
                    .0
                    .enabled,
                GATE_ACTIVE
            );
            if loss_retreat {
                t.header.qpc = 1300;
                assert_eq!(
                    s.evaluate(1300, 1000, Some(&t), Some(&c), false, None)
                        .0
                        .enabled,
                    GATE_BRAKE_ONLY
                );
            }
            let mut mode = loaded_automatic();
            mode.generation = 2;
            mode.header.seq = 2;
            mode.shifter_type = etsads_msgs::protocol::SHIFTER_ARCADE;
            s.observe_control_configuration(Some(mode), 1);
            t.header.qpc = 1300;
            t.gear = -1;
            t.speed_mps = -1.0;
            let (g, reason) = s.evaluate(
                1300,
                1000,
                Some(&t),
                Some(&c),
                false,
                Some(Obstacle {
                    distance_m: 0.0,
                    closing_speed_mps: 2.0,
                }),
            );
            assert_eq!(reason, Some(StopReason::UnsafeControlConfiguration));
            assert_eq!((g.enabled, g.aeb_brake, g.max_throttle), (0, 0.0, 0.0));
            mode.generation = 3;
            mode.header.seq = 3;
            mode.shifter_type = etsads_msgs::protocol::SHIFTER_AUTOMATIC;
            s.observe_control_configuration(Some(mode), 1);
            let current = ControlCmd {
                seq: 2,
                write_qpc: 1300,
                ..c
            };
            assert_eq!(
                s.evaluate(1300, 1000, Some(&t), Some(&current), false, None)
                    .0
                    .enabled,
                0
            );
            s.arm();
            assert_eq!(
                s.evaluate(1300, 1000, Some(&t), Some(&current), false, None)
                    .0
                    .enabled,
                GATE_ACTIVE
            );
        }
    }
    #[test]
    fn missing_or_future_controls_never_grant_any_brake_authority() {
        let (mut s, t, c) = fixture();
        s.observe_control_configuration(None, 1);
        s.arm();
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), false, None)
                .0
                .enabled,
            0
        );
        let mut future = loaded_automatic();
        future.header.qpc = 1001;
        s.observe_control_configuration(Some(future), 1);
        s.arm();
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), false, None)
                .0
                .enabled,
            0
        );
    }
    #[test]
    fn command_loss_brake_only_is_bounded_and_ignores_recovered_commands() {
        let (mut supervisor, mut telemetry, original) = fixture();
        assert_eq!(
            supervisor
                .evaluate(1000, 1000, Some(&telemetry), Some(&original), false, None)
                .0
                .enabled,
            GATE_ACTIVE
        );
        telemetry.header.qpc = 1201;
        let (gate, reason) =
            supervisor.evaluate(1201, 1000, Some(&telemetry), Some(&original), false, None);
        assert_eq!(gate.enabled, GATE_BRAKE_ONLY);
        assert_eq!(reason, Some(StopReason::CommandLostBraking));
        assert_eq!(gate.max_throttle, 0.0);
        assert!(gate.aeb_brake > 0.0 && gate.aeb_brake < 0.01);
        assert!(!supervisor.is_armed());
        assert!(supervisor.is_braking_after_loss());
        let mut recovered = original;
        recovered.seq = 2;
        recovered.throttle = 1.0;
        recovered.steer = 1.0;
        recovered.holds = 7;
        for (now, expected_brake) in [(1450, 0.15), (1700, 0.3), (1999, 0.3)] {
            telemetry.header.qpc = now;
            recovered.write_qpc = now;
            let (gate, _) =
                supervisor.evaluate(now, 1000, Some(&telemetry), Some(&recovered), false, None);
            assert_eq!(gate.enabled, GATE_BRAKE_ONLY);
            assert_eq!(gate.max_throttle, 0.0);
            assert!((gate.aeb_brake - expected_brake).abs() < 1e-6);
        }
        telemetry.header.qpc = 2000;
        recovered.write_qpc = 2000;
        assert_eq!(
            supervisor
                .evaluate(2000, 1000, Some(&telemetry), Some(&recovered), false, None)
                .0
                .enabled,
            0
        );
        telemetry.header.qpc = 2010;
        recovered.write_qpc = 2010;
        assert_eq!(
            supervisor
                .evaluate(2010, 1000, Some(&telemetry), Some(&recovered), false, None)
                .1,
            Some(StopReason::Disarmed)
        );
        supervisor.arm();
        assert_eq!(
            supervisor
                .evaluate(2010, 1000, Some(&telemetry), Some(&recovered), false, None)
                .0
                .enabled,
            GATE_ACTIVE
        );
    }
    #[test]
    fn explicit_release_pause_manual_and_bad_telemetry_override_loss_braking() {
        for condition in 0..4 {
            let (mut supervisor, mut telemetry, command) = fixture();
            supervisor.evaluate(1000, 1000, Some(&telemetry), Some(&command), false, None);
            telemetry.header.qpc = 1300;
            assert_eq!(
                supervisor
                    .evaluate(1300, 1000, Some(&telemetry), Some(&command), false, None)
                    .0
                    .enabled,
                GATE_BRAKE_ONLY
            );
            match condition {
                0 => supervisor.disarm(),
                1 => telemetry.paused = 1,
                3 => telemetry.speed_mps = f32::NAN,
                _ => {}
            }
            let (gate, _) = supervisor.evaluate(
                1300,
                1000,
                Some(&telemetry),
                Some(&command),
                condition == 2,
                None,
            );
            assert_eq!(gate.enabled, 0);
            assert_eq!(gate.aeb_brake, 0.0);
            assert!(!supervisor.is_braking_after_loss());
        }
    }
    #[test]
    fn unobserved_old_command_and_future_command_cannot_start_fallback_braking() {
        let (mut supervisor, mut telemetry, mut command) = fixture();
        telemetry.header.qpc = 1500;
        assert_eq!(
            supervisor
                .evaluate(1500, 1000, Some(&telemetry), Some(&command), false, None)
                .0
                .enabled,
            0
        );
        supervisor.arm();
        telemetry.header.qpc = 1600;
        command.write_qpc = 1600;
        supervisor.evaluate(1600, 1000, Some(&telemetry), Some(&command), false, None);
        command.write_qpc = 1700;
        let (gate, reason) =
            supervisor.evaluate(1600, 1000, Some(&telemetry), Some(&command), false, None);
        assert_eq!(gate.enabled, 0);
        assert_eq!(reason, Some(StopReason::InvalidCommand));
    }
    #[test]
    fn physical_frame_expiry_revokes_authority_without_early_braking_or_invalid_fallback() {
        let (mut supervisor, mut telemetry, command) = fixture();
        supervisor.evaluate(1000, 1000, Some(&telemetry), Some(&command), false, None);
        telemetry.header.frame_id = 14;
        telemetry.header.qpc = 1190;
        let (gate, reason) =
            supervisor.evaluate(1190, 1000, Some(&telemetry), Some(&command), false, None);
        assert_eq!(reason, Some(StopReason::CommandLostBraking));
        assert_eq!(gate.enabled, GATE_BRAKE_ONLY);
        assert_eq!(gate.aeb_brake, 0.0);
        let (mut supervisor, mut telemetry, mut command) = fixture();
        supervisor.evaluate(1000, 1000, Some(&telemetry), Some(&command), false, None);
        telemetry.header.qpc = 1300;
        command.throttle = f32::NAN;
        let (gate, reason) =
            supervisor.evaluate(1300, 1000, Some(&telemetry), Some(&command), false, None);
        assert_eq!(gate.enabled, 0);
        assert_eq!(reason, Some(StopReason::InvalidCommand));
    }
    #[test]
    fn pause_refuses_engagement_holds_an_engaged_drive_and_manual_overrides() {
        let (mut s, mut t, c) = fixture();
        t.paused = 1;
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), false, None).1,
            Some(StopReason::Paused)
        );
        // Engaged first, then paused: control is held with no throttle.
        s.arm();
        t.paused = 0;
        let (gate, reason) = s.evaluate(1000, 1000, Some(&t), Some(&c), false, None);
        assert_eq!((gate.enabled, reason), (GATE_ACTIVE, None));
        t.paused = 1;
        let (gate, reason) = s.evaluate(1000, 1000, Some(&t), Some(&c), false, None);
        assert_eq!((gate.enabled, reason), (GATE_ACTIVE, None));
        assert_eq!(gate.max_throttle, 0.0);
        assert_eq!(
            s.evaluate(1000, 1000, Some(&t), Some(&c), true, None).1,
            Some(StopReason::ManualTakeover)
        );
    }
    #[test]
    fn invalid_commands_fail_closed() {
        for value in [f32::NAN, f32::INFINITY, -1.0, 1.01] {
            let (mut s, t, mut c) = fixture();
            c.throttle = value;
            assert_eq!(
                s.evaluate(1000, 1000, Some(&t), Some(&c), false, None)
                    .0
                    .enabled,
                0
            );
        }
    }
    #[test]
    fn aeb_cuts_throttle() {
        let (mut s, t, c) = fixture();
        let (g, r) = s.evaluate(
            1000,
            1000,
            Some(&t),
            Some(&c),
            false,
            Some(Obstacle {
                distance_m: 8.0,
                closing_speed_mps: 10.0,
            }),
        );
        assert!(r.is_none());
        assert_eq!(g.max_throttle, 0.0);
        assert_eq!(g.aeb_brake, 1.0);
    }
    #[test]
    fn future_timestamp_and_clock_overflow_fail_closed() {
        assert!(!fresh(10, 11, 1000, 200));
        assert!(!fresh(u64::MAX, 1, u64::MAX, 200));
        assert!(!fresh(10, 1, 0, 200));
    }
    #[test]
    fn raw_aeb_sees_front_vehicle_and_ignores_adjacent_lane() {
        let mut e = TelemetryFrame::default();
        e.header.frame_id = 5;
        e.header.qpc = 1000;
        e.qpc_frequency = 1000;
        e.speed_mps = 20.0;
        let mut x = Box::<ExtensionFrame>::default();
        x.header.frame_id = 4;
        x.header.qpc = 990;
        x.available = 2;
        let values = [
            0.0_f32, 0.0, -20.0, 1.0, 0.0, 0.0, 0.0, 2.5, 3.0, 6.0, 5.0, 0.0,
        ];
        for (i, value) in values.iter().enumerate() {
            x.traffic[4 * i..4 * i + 4].copy_from_slice(&value.to_le_bytes());
        }
        x.traffic[50] = 1;
        let o = primary_obstacle(&e, &x).unwrap().unwrap();
        assert!((o.distance_m - 13.8).abs() < 0.01);
        assert_eq!(o.closing_speed_mps, 15.0);
        // A car on an overpass 19 m above is another road level; one 3 m up a
        // slope is not.
        x.traffic[4..8].copy_from_slice(&19.0_f32.to_le_bytes());
        assert!(primary_obstacle(&e, &x).unwrap().is_none());
        x.traffic[4..8].copy_from_slice(&3.0_f32.to_le_bytes());
        assert!(primary_obstacle(&e, &x).unwrap().is_some());
        x.traffic[4..8].copy_from_slice(&0.0_f32.to_le_bytes());
        x.traffic[0..4].copy_from_slice(&8.0_f32.to_le_bytes());
        assert!(primary_obstacle(&e, &x).unwrap().is_none());
        x.traffic[0..4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(primary_obstacle(&e, &x).is_err());
    }
    #[test]
    fn raw_aeb_ignores_a_tall_van_on_the_road_under_a_bridge() {
        let mut e = TelemetryFrame::default();
        e.header.frame_id = 5;
        e.header.qpc = 1000;
        e.qpc_frequency = 1000;
        e.speed_mps = 11.0;
        let mut x = Box::<ExtensionFrame>::default();
        x.header.frame_id = 4;
        x.header.qpc = 990;
        x.available = 2;
        // A 3.5 m van 15 m ahead with its centre `y` from the ego reference.
        let place = |x: &mut ExtensionFrame, y: f32, ahead: f32| {
            let values = [
                0.0_f32, y, -ahead, 1.0, 0.0, 0.0, 0.0, 2.5, 3.5, 6.0, 0.0, 0.0,
            ];
            for (i, value) in values.iter().enumerate() {
                x.traffic[4 * i..4 * i + 4].copy_from_slice(&value.to_le_bytes());
            }
            x.traffic[50] = 1;
        };
        let pitch = |e: &mut TelemetryFrame, grade: f64| {
            e.orientation_turns[1] = (grade.atan() / std::f64::consts::TAU) as f32;
        };
        // Road 9.2 m below: its top is 5.45 m down, within STACKED_ROAD_M.
        place(&mut x, -9.2 + 1.75, 15.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_none());
        // Beyond a crest, 3 m lower 20 m ahead: our road.
        place(&mut x, -3.0 + 1.75, 20.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_some());
        // Down a 10% grade, 2 m lower 20 m ahead: our road, not one below.
        pitch(&mut e, -0.1);
        place(&mut x, -2.0 + 1.75, 20.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_some());
        // Up a 10% grade a lower road passes 9 m below the truck.
        pitch(&mut e, 0.1);
        place(&mut x, -9.2 + 1.75, 15.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_none());
        // An implausible pitch keeps only the 6 m test.
        e.orientation_turns[1] = f32::NAN;
        assert!(primary_obstacle(&e, &x).unwrap().is_some());
    }
    #[test]
    fn planned_path_replaces_the_yaw_rate_arc_when_the_truck_is_on_it() {
        // r10 roundabout exit: the yaw rate still says R 28 m while the exit
        // lane tightens to R 12 m; a car waits beside the exit.
        let mut e = TelemetryFrame::default();
        e.header.frame_id = 5;
        e.header.qpc = 1000;
        e.qpc_frequency = 1000;
        e.speed_mps = 4.6;
        e.angular_velocity_local[1] = (-4.6 / 28.0 / std::f64::consts::TAU) as f32;
        // ENU position and heading change from the ego's (north) heading.
        let place = |x: &mut ExtensionFrame, east: f64, north: f64, a: f64| {
            let values = [
                east as f32,
                0.0,
                -north as f32,
                (a / 2.0).cos() as f32,
                0.0,
                (a / 2.0).sin() as f32,
                0.0,
                1.9,
                1.4,
                4.6,
                0.0,
                0.0,
            ];
            x.traffic.fill(0);
            for (i, value) in values.iter().enumerate() {
                x.traffic[4 * i..4 * i + 4].copy_from_slice(&value.to_le_bytes());
            }
            x.traffic[50] = 1;
        };
        let mut x = Box::<ExtensionFrame>::default();
        x.header.frame_id = 4;
        x.header.qpc = 990;
        x.available = 2;
        // On the R 28 arc 15.2 m ahead, 4.5 m outside the R 12 exit lane.
        place(&mut x, 4.0, 14.45, 0.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_some(), "arc sweeps it");
        let mut path = etsads_msgs::protocol::PlannedPath {
            write_qpc: 1003,
            source_frame_id: 5,
            ..Default::default()
        };
        for i in 0..40 {
            let t = (i as f64 * 0.05).min(std::f64::consts::FRAC_PI_2);
            path.points[i] = if i as f64 * 0.05 <= std::f64::consts::FRAC_PI_2 {
                [12.0 - 12.0 * t.cos(), 12.0 * t.sin()]
            } else {
                [12.0 + (i as f64 * 0.05 - t) * 12.0, 12.0]
            };
            path.count += 1;
        }
        assert!(
            primary_obstacle_along(&e, &x, Some(&path))
                .unwrap()
                .is_none()
        );
        // A car standing in the exit lane itself is still found.
        let t: f64 = 0.8;
        place(&mut x, 12.0 - 12.0 * t.cos(), 12.0 * t.sin(), -t);
        let o = primary_obstacle_along(&e, &x, Some(&path))
            .unwrap()
            .expect("car on the planned path");
        assert!(
            (o.distance_m - (9.6 - 2.3 - 3.2) as f32).abs() < 0.4,
            "{o:?}"
        );
        // A stale path, or one the truck is not on, falls back to the arc.
        place(&mut x, 4.0, 14.45, 0.0);
        path.write_qpc = 1300;
        assert!(
            primary_obstacle_along(&e, &x, Some(&path))
                .unwrap()
                .is_some()
        );
        path.write_qpc = 1003;
        for p in &mut path.points[..path.count as usize] {
            p[0] += 3.0;
        }
        assert!(
            primary_obstacle_along(&e, &x, Some(&path))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn bend_corridor_follows_steering_and_ignores_oncoming_lane() {
        // r9 field case: stopped on a right-hand bend (R 90 m) while oncoming
        // cars pass in their own lane 4 m to the left of the path.
        const RADIUS: f64 = 90.0;
        let mut e = TelemetryFrame::default();
        e.header.frame_id = 5;
        e.header.qpc = 1000;
        e.qpc_frequency = 1000;
        let wheel = -((3.8 / RADIUS).atan() / std::f64::consts::TAU) as f32;
        e.wheel_steering[0] = wheel;
        e.wheel_steering[1] = wheel;
        assert!((ego_path_curvature(&e) + 1.0 / RADIUS).abs() < 1e-4);
        // A point `offset` metres left of the arc at station `s`, in SCS, with
        // the arc's local heading plus `turn` radians.
        let place = |x: &mut ExtensionFrame, s: f64, offset: f64, turn: f64, speed: f32| {
            let h = -s / RADIUS;
            let forward = RADIUS * (-h).sin() + offset * -h.sin();
            let left = -(RADIUS - RADIUS * h.cos()) + offset * h.cos();
            let a = h + turn;
            let values = [
                -left as f32,
                0.0,
                -forward as f32,
                (a / 2.0).cos() as f32,
                0.0,
                (a / 2.0).sin() as f32,
                0.0,
                1.9,
                1.4,
                4.6,
                speed,
                0.0,
            ];
            x.traffic.fill(0);
            for (i, value) in values.iter().enumerate() {
                x.traffic[4 * i..4 * i + 4].copy_from_slice(&value.to_le_bytes());
            }
            x.traffic[50] = 1;
        };
        let mut x = Box::<ExtensionFrame>::default();
        x.header.frame_id = 4;
        x.header.qpc = 990;
        x.available = 2;
        for s in [15.0, 30.0, 45.0] {
            place(&mut x, s, 4.0, std::f64::consts::PI, 14.0);
            assert!(
                primary_obstacle(&e, &x).unwrap().is_none(),
                "oncoming car in its own lane at s={s} must not brake"
            );
        }
        place(&mut x, 30.0, 0.3, 0.0, 0.0);
        let o = primary_obstacle(&e, &x)
            .unwrap()
            .expect("stopped car in our lane");
        assert!(
            (o.distance_m - (30.0 - 2.3 - 3.2) as f32).abs() < 0.3,
            "{o:?}"
        );
        // Straight steering keeps the original straight corridor.
        e.wheel_steering = [0.0; 16];
        assert_eq!(ego_path_curvature(&e), 0.0);
        // An oncoming car even in our lane: braking a stopped truck cannot
        // help; a moving truck brakes for it only at a short TTC.
        place(&mut x, 20.0, 0.0, std::f64::consts::PI, 14.0);
        e.wheel_steering[0] = wheel;
        e.wheel_steering[1] = wheel;
        assert!(primary_obstacle(&e, &x).unwrap().is_none());
        e.speed_mps = 8.0;
        e.angular_velocity_local[1] = (-8.0 / RADIUS / std::f64::consts::TAU) as f32;
        let o = primary_obstacle(&e, &x).unwrap().expect("TTC below 1.2 s");
        assert!((o.closing_speed_mps - 22.0).abs() < 0.2, "{o:?}");
        place(&mut x, 60.0, 0.0, std::f64::consts::PI, 14.0);
        assert!(primary_obstacle(&e, &x).unwrap().is_none(), "TTC 2.5 s");
    }
    #[test]
    fn reverse_throttle_is_disabled_without_independent_geometry() {
        let (mut supervisor, mut t, mut c) = fixture();
        t.gear = -1;
        c.throttle = 0.5;
        let (g, reason) = supervisor.evaluate(1000, 1000, Some(&t), Some(&c), false, None);
        assert_eq!(g.enabled, 1);
        assert_eq!(g.max_throttle, 0.0);
        assert_eq!(reason, Some(StopReason::ReverseUnverified));
        t.speed_mps = -0.5;
        let (g, _) = supervisor.evaluate(1000, 1000, Some(&t), Some(&c), false, None);
        assert!(g.aeb_brake >= 0.5);
    }
    #[test]
    fn validated_profile_caps_reverse_speed_and_latches_attachment_changes() {
        let (t, config, profile, _) = reverse::tests::fixture(false);
        let mut supervisor = Supervisor::default();
        supervisor.observe_control_configuration(Some(loaded_automatic()), 1);
        supervisor.arm();
        let c = ControlCmd {
            seq: 1,
            write_qpc: 1000,
            source_frame_id: t.header.frame_id,
            throttle: 0.5,
            ..Default::default()
        };
        let (g, reason) = supervisor.evaluate_profiled(
            1000,
            1000,
            SafetyInputs {
                telemetry: Some(&t),
                command: Some(&c),
                configuration: Some(&config),
                profile: Some(&profile),
                manual: false,
                obstacle: None,
            },
        );
        assert!(reason.is_none());
        assert_eq!(g.max_throttle, 1.0);
        assert!(supervisor.reverse_profile_ready(&t, &config, &profile));
        let mut overspeed = t;
        overspeed.speed_mps = -1.0;
        let (g, reason) = supervisor.evaluate_profiled(
            1000,
            1000,
            SafetyInputs {
                telemetry: Some(&overspeed),
                command: Some(&c),
                configuration: Some(&config),
                profile: Some(&profile),
                manual: false,
                obstacle: None,
            },
        );
        assert_eq!(reason, Some(StopReason::ReverseSpeedLimit));
        assert_eq!(g.max_throttle, 0.0);
        assert!(g.aeb_brake >= 0.4);
        let mut changed = t;
        changed.trailers[0].connected = 1;
        supervisor.evaluate_profiled(
            1000,
            1000,
            SafetyInputs {
                telemetry: Some(&changed),
                command: Some(&c),
                configuration: Some(&config),
                profile: Some(&profile),
                manual: false,
                obstacle: None,
            },
        );
        let (g, reason) = supervisor.evaluate_profiled(
            1000,
            1000,
            SafetyInputs {
                telemetry: Some(&t),
                command: Some(&c),
                configuration: Some(&config),
                profile: Some(&profile),
                manual: false,
                obstacle: None,
            },
        );
        assert_eq!(g.max_throttle, 0.0);
        assert_eq!(reason, Some(StopReason::ReverseConfigurationChanged));
        assert!(!supervisor.reverse_profile_ready(&t, &config, &profile));
    }
}
