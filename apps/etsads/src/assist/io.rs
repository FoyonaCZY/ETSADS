use anyhow::ensure;
use etsads_msgs::protocol::*;
use etsads_shm::{SharedCell, SharedRing, SingleWriterGuard, qpc, qpc_frequency};
use std::{
    fs::{File, OpenOptions},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

pub struct SupervisorProcess(Option<Child>);
impl SupervisorProcess {
    pub fn ensure() -> anyhow::Result<Self> {
        let fresh = || {
            SharedCell::<SupervisorStatus>::open(SUPERVISOR_STATUS_MAPPING, SCHEMA_VERSION)
                .ok()
                .and_then(|m| m.latest())
                .is_some_and(|s| {
                    s.schema_version == 1
                        && etsads_safety::fresh(qpc(), s.heartbeat_qpc, qpc_frequency(), 200)
                })
        };
        if fresh() {
            return Ok(Self(None));
        }
        let exe = std::env::current_exe()?.with_file_name(if cfg!(windows) {
            "etsads-supervisor.exe"
        } else {
            "etsads-supervisor"
        });
        ensure!(exe.is_file(), "缺少同目录的安全监督程序：{}", exe.display());
        let mut command = Command::new(exe);
        // Its gate decisions go to a log: r13 lost the gate at 78 km/h and
        // nothing recorded why.
        let log = crate::setup::data_dir().join("logs");
        std::fs::create_dir_all(&log)?;
        let stderr = OpenOptions::new()
            .create(true)
            .append(true)
            .open(log.join("supervisor.log"))
            .map_or_else(|_| Stdio::null(), Stdio::from);
        command
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(stderr);
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let mut owned = Self(Some(command.spawn()?));
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(4) {
            if fresh() {
                return Ok(owned);
            }
            ensure!(
                owned.0.as_mut().unwrap().try_wait()?.is_none(),
                "安全监督启动失败，可能热键被占用"
            );
            std::thread::sleep(Duration::from_millis(20));
        }
        anyhow::bail!("安全监督没有产生心跳")
    }
}
impl Drop for SupervisorProcess {
    fn drop(&mut self) {
        if let Some(child) = &mut self.0 {
            std::thread::sleep(Duration::from_millis(50));
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

pub struct Sink {
    command: SharedCell<ControlCmd>,
    requests: SharedCell<SafetyRequest>,
    pub gate: SharedCell<Gate>,
    pub applied: SharedCell<AppliedControl>,
    last: ControlCmd,
    /// Current speed (m/s) and engine speed (rpm), for the clutch below.
    speed: f64,
    rpm: f64,
    pub steering: SteeringServo,
}
/// Rate (1/s) at which a steering position error is closed in relative mode.
const SERVO_GAIN: f64 = 5.0;
/// The bridge slews the steering input at the gate's 0.8/s; the servo plans
/// with a little less so its own ramps are the ones applied.
const SERVO_INPUT_SLEW: f64 = 0.76;
/// At speed a relatively steered wheel returns toward the centre at this
/// rate per m/s of speed (1/s per m/s): r15 fitted 0.095–0.11 over three
/// drives from 3 to 9 m/s; at a standstill it stays put. Holding a turn
/// takes a steady input, and the reachable turning rate shrinks with the
/// position.
pub const RELATIVE_CENTERING: f64 = 0.10;
/// The game turns a relatively steered wheel more slowly at speed: over r17
/// (2,500 samples from 0 to 15 m/s) the rate per unit input fell linearly by
/// 1.95 % per m/s, from 0.39/s at a crawl to 0.30/s at 50 km/h, and the
/// return toward the centre fell with it.
const RELATIVE_SPEED_FALLOFF: f64 = 0.0195;
/// The share of the standstill turning rate left at `speed` (m/s).
pub fn relative_speed_factor(speed: f64) -> f64 {
    (1.0 - RELATIVE_SPEED_FALLOFF * speed.abs()).max(0.5)
}
const RELATIVE_ACCEL_SHARE: f64 = 0.9;
/// A steering measurement older than this is not used.
const SERVO_MEASUREMENT_AGE: Duration = Duration::from_millis(150);
/// Braking toward the goal is planned at this share of the acceleration
/// limit: a goal that stops after moving was overrun by about 0.1 (r16).
const SHAPE_BRAKING_SHARE: f64 = 0.5;
/// Steering positions a relatively steered wheel can follow: the rate is at
/// most `max_rate`, and it changes no faster than the input can ramp
/// (`max_accel`), braking in time to stop on the goal. `goal_rate` is how
/// fast the goal itself moves, so a steadily turning goal is followed
/// without a lag. Returns the next position and rate.
pub fn shape_steering(
    position: f64,
    rate: f64,
    goal: f64,
    goal_rate: f64,
    limits: (f64, f64),
    dt: f64,
) -> (f64, f64) {
    let (max_rate, max_accel) = limits;
    let error = goal - position;
    let wanted = (goal_rate
        + error.signum() * (2.0 * SHAPE_BRAKING_SHARE * max_accel * error.abs()).sqrt())
    .clamp(-max_rate, max_rate);
    let rate = wanted.clamp(rate - max_accel * dt, rate + max_accel * dt);
    let next = position + rate * dt;
    // Never step past a goal that is not moving on.
    if (goal - next) * error < 0.0 && goal_rate.abs() < 1e-6 {
        (goal, 0.0)
    } else {
        (next, rate)
    }
}
/// Rate and acceleration limits for a relatively steered position: the
/// wheel turns at `rate_per_s` per unit input and the input ramps at the
/// servo's slew; the target keeps a margin for corrections. Away from the
/// centre at `position` and `speed` (m/s), part of the input holds the
/// wheel against its return.
pub fn relative_limits(rate_per_s: f64, speed: f64, position: f64) -> (f64, f64) {
    let factor = relative_speed_factor(speed);
    let rate_per_s = factor * rate_per_s;
    let holding = factor * RELATIVE_CENTERING * speed.abs() * position.abs();
    (
        (0.85 * rate_per_s - holding).max(0.2 * rate_per_s),
        RELATIVE_ACCEL_SHARE * SERVO_INPUT_SLEW * rate_per_s,
    )
}
/// Turns the controller's steering position into the game's input. The
/// position is the input an absolute-steering profile takes (the game's
/// effective steering with the opposite sign). With relative steering the
/// input is a turning rate, so it closes the position loop on the measured
/// effective steering.
pub struct SteeringServo {
    mode: crate::setup::SteeringMode,
    measured: Option<(f64, Instant)>,
    last_target: Option<(f64, Instant)>,
    last_input: f64,
    /// Vehicle speed (m/s), for the wheel's return toward the centre.
    speed: f64,
}
impl SteeringServo {
    pub fn new(mode: crate::setup::SteeringMode) -> Self {
        Self {
            mode,
            measured: None,
            last_target: None,
            last_input: 0.0,
            speed: 0.0,
        }
    }
    pub fn mode(&self) -> crate::setup::SteeringMode {
        self.mode
    }
    pub fn set_mode(&mut self, mode: crate::setup::SteeringMode) {
        if mode != self.mode {
            *self = Self {
                measured: self.measured,
                speed: self.speed,
                ..Self::new(mode)
            };
        }
    }
    /// The game's effective steering and the speed (m/s), from telemetry.
    pub fn observe(&mut self, effective: f64, speed: f64, now: Instant) {
        self.measured = effective.is_finite().then_some((-effective, now));
        self.speed = if speed.is_finite() { speed.abs() } else { 0.0 };
    }
    /// The measured steering position, if fresh.
    pub fn position(&self, now: Instant) -> Option<f64> {
        self.measured
            .filter(|(_, at)| now.saturating_duration_since(*at) <= SERVO_MEASUREMENT_AGE)
            .map(|(position, _)| position)
    }
    /// Fastest steering position change the mode allows (1/s), if limited.
    pub fn max_rate(&self) -> Option<f64> {
        match self.mode {
            crate::setup::SteeringMode::Absolute => None,
            crate::setup::SteeringMode::Relative { rate_per_s } => Some(rate_per_s),
        }
    }
    pub fn input(&mut self, target: f64, now: Instant) -> f64 {
        let target = target.clamp(-1.0, 1.0);
        let previous = self.last_target.replace((target, now));
        let crate::setup::SteeringMode::Relative { rate_per_s } = self.mode else {
            return target;
        };
        let dt = previous.map_or(0.0, |(_, at)| {
            now.saturating_duration_since(at).as_secs_f64()
        });
        // Without a fresh measurement the wheel is left where it is.
        let Some(position) = self.position(now) else {
            self.last_input = 0.0;
            return 0.0;
        };
        // Feed-forward of the target's own motion and of the input that
        // holds it against the wheel's return, then a proportional
        // correction capped so the input can ramp back (at the slew the
        // bridge allows) before the error is used up. r14 left the return
        // to the correction alone: the wheel lagged a bend by about a third.
        let motion = previous
            .filter(|_| (0.002..=0.1).contains(&dt))
            .map_or(0.0, |(last, _)| (target - last) / dt);
        // The game turns the wheel more slowly at speed (r17); the
        // correction keeps its standstill gain, which scaled up overshot.
        let factor = relative_speed_factor(self.speed);
        let feed = (motion / factor + RELATIVE_CENTERING * self.speed * target) / rate_per_s;
        let error = target - position;
        let cap = (2.0 * SERVO_INPUT_SLEW * error.abs() / rate_per_s).sqrt();
        let correction = (SERVO_GAIN * error / rate_per_s).clamp(-cap, cap);
        let step = SERVO_INPUT_SLEW * dt.clamp(0.0, 0.1);
        let input = (feed + correction)
            .clamp(-1.0, 1.0)
            .clamp(self.last_input - step, self.last_input + step);
        self.last_input = input;
        input
    }
}
pub struct ParkingReceipt {
    sequence: u64,
    counter: u32,
    qpc: u64,
    frame: u64,
    cursor: u64,
    acknowledged: bool,
    desired: bool,
    completed: Option<InputActionFrame>,
}
impl ParkingReceipt {
    fn matches(&self, event: &InputActionFrame) -> bool {
        event.command_seq >= self.sequence
            && event.completed_qpc >= self.qpc
            && event.completed_qpc >= event.prepared_qpc
            && event.completed_qpc != 0
            && event.source_frame_id >= self.frame
            && event.reason == etsads_game::input::REASON_ACTIVE
            && event.pulse_mask & (1 << 17) != 0
            && event.pulse_counters[17] == self.counter
    }
    pub fn confirmed(
        &mut self,
        actions: &SharedRing<InputActionFrame, 256>,
        raw: &TelemetryFrame,
    ) -> anyhow::Result<bool> {
        let latest = actions.latest_index();
        ensure!(
            latest.saturating_sub(self.cursor) <= 256,
            "驻车SDK完成证据发生覆盖"
        );
        for index in self.cursor + 1..=latest {
            let Some(event) = actions.read(index) else {
                break;
            };
            if event.completed_qpc <= qpc() && self.matches(&event) {
                self.acknowledged = true;
                self.completed = Some(event);
            }
            self.cursor = index;
        }
        Ok(self.acknowledged
            && raw.header.frame_id > self.frame
            && raw.header.qpc > self.qpc
            && (raw.parking_brake != 0) == self.desired)
    }
    pub fn evidence(&self) -> Option<serde_json::Value> {
        self.completed.as_ref().map(|e|serde_json::json!({"request":{"command_seq":self.sequence,"counter":self.counter,"qpc":self.qpc,"frame":self.frame,"desired_parking":self.desired},
            "completion":{"command_seq":e.command_seq,"source_frame":e.source_frame_id,"prepared_qpc":e.prepared_qpc,"completed_qpc":e.completed_qpc,"pulse_mask":e.pulse_mask,"parking_counter":e.pulse_counters[17],"reason":e.reason}}))
    }
}
impl Sink {
    pub fn connect() -> anyhow::Result<Self> {
        Self::connect_named("")
    }
    fn connect_named(suffix: &str) -> anyhow::Result<Self> {
        let name = |base: &str| format!("{base}{suffix}");
        let command = SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION)?;
        // The caller holds ETSADS_Control_Writer; a killed predecessor may have
        // left a half-written command that would block every publish.
        command.recover_after_writer_crash();
        let old = command.latest().unwrap_or_default();
        let sink = Self {
            command,
            requests: SharedCell::open(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)?,
            gate: SharedCell::open(&name(GATE_MAPPING), SCHEMA_VERSION)?,
            applied: SharedCell::open(&name(APPLIED_MAPPING), SCHEMA_VERSION)?,
            last: ControlCmd {
                seq: old.seq,
                pulses: old.pulses,
                ..Default::default()
            },
            speed: 0.0,
            rpm: f64::INFINITY,
            steering: SteeringServo::new(crate::setup::SteeringMode::Absolute),
        };
        sink.request(false)?;
        Ok(sink)
    }
    pub fn request(&self, arm: bool) -> anyhow::Result<()> {
        let old = crate::loaded_controls::coherent_read(|| self.requests.latest());
        ensure!(!arm || old.is_some(), "安全请求暂不可读");
        self.requests.publish(&SafetyRequest {
            seq: old.map_or(1, |r| r.seq.wrapping_add(1)),
            write_qpc: qpc(),
            arm: arm as u32,
            reserved: 0,
        })?;
        Ok(())
    }
    /// The latest measured speed (m/s), engine speed (rpm) and the game's
    /// effective steering.
    pub fn observe(&mut self, speed: f64, rpm: f64, effective_steering: f64) {
        self.speed = speed;
        self.rpm = if rpm.is_finite() { rpm } else { f64::INFINITY };
        self.steering
            .observe(effective_steering, speed, Instant::now());
    }
    /// `output.steer` is a steering position; the servo turns it into the
    /// input of the game's steering mode.
    pub fn publish(&mut self, frame: u64, output: super::core::Output) -> anyhow::Result<()> {
        self.last.seq = self.last.seq.wrapping_add(1);
        self.last.write_qpc = qpc();
        self.last.source_frame_id = frame;
        self.last.steer = self.steering.input(output.steer, Instant::now()) as f32;
        self.last.throttle = output.throttle as f32;
        self.last.brake = output.brake as f32;
        self.last.clutch = clutch(output.throttle, output.brake, self.speed, self.rpm);
        self.command.publish(&self.last)?;
        Ok(())
    }
    /// Press a toggle button (e.g. a turn signal) once, with the next publish.
    pub fn press(&mut self, pulse: usize) {
        self.last.pulses[pulse] = self.last.pulses[pulse].wrapping_add(1);
    }
    pub fn park(
        &mut self,
        raw: &TelemetryFrame,
        actions: &SharedRing<InputActionFrame, 256>,
        steer: f64,
    ) -> anyhow::Result<ParkingReceipt> {
        self.request_parking(raw, actions, steer, true)
    }
    pub fn request_parking(
        &mut self,
        raw: &TelemetryFrame,
        actions: &SharedRing<InputActionFrame, 256>,
        steer: f64,
        desired: bool,
    ) -> anyhow::Result<ParkingReceipt> {
        ensure!(
            raw.paused == 0
                && (raw.parking_brake != 0) != desired
                && raw.speed_mps.is_finite()
                && raw.speed_mps.abs() < 0.05,
            "只能在确认停稳后请求驻车"
        );
        ensure!(
            steer.is_finite() && steer.abs() <= 1.0,
            "驻车收尾转向超出范围"
        );
        let cursor = actions.latest_index();
        self.last.pulses[17] = self.last.pulses[17].wrapping_add(1);
        self.publish(
            raw.header.frame_id,
            super::core::Output {
                steer,
                throttle: 0.0,
                brake: 0.55,
            },
        )?;
        Ok(ParkingReceipt {
            sequence: self.last.seq,
            counter: self.last.pulses[17],
            qpc: self.last.write_qpc,
            frame: raw.header.frame_id,
            cursor,
            acknowledged: false,
            desired,
            completed: None,
        })
    }
}
impl Drop for Sink {
    fn drop(&mut self) {
        let _ = self.publish(self.last.source_frame_id, super::neutral());
        let _ = self.request(false);
    }
}

/// One file; a longer drive continues in a new file of the same session.
/// Below this speed (m/s) a rolling truck without throttle has its clutch down.
const LOW_SPEED_CLUTCH: f64 = 12.0 / 3.6;
/// Brake that holds a stopped truck; with it the clutch may stay down.
const HOLD_CLUTCH_BRAKE: f64 = 0.2;
/// Engine speed near idle (550 rpm) below which a coasting truck declutches.
const LUGGING_RPM: f64 = 800.0;
/// Clutch pedal for the automatic gearbox, which stalls in gear. Rolling
/// slowly without throttle it is held down as a driver would: coasting below
/// ~12 km/h in the starting gear stalled the engine (L11, L16). Near idle
/// it is down at any speed: r13 stalled three times braking at 14–50 km/h
/// as the engine fell below idle. Under throttle the game's own automatic
/// clutch takes over. At a standstill it stays down only while the brake
/// holds the truck: released there without braking it let the truck roll
/// back on a slope (L19); released while braking to a stop it stalled the
/// engine (r13 yard exit).
fn clutch(throttle: f64, brake: f64, speed: f64, rpm: f64) -> f32 {
    let held = brake >= HOLD_CLUTCH_BRAKE;
    let lugging = rpm < LUGGING_RPM;
    if throttle < 0.01 && (speed < LOW_SPEED_CLUTCH || lugging) && (speed >= 0.5 || held) {
        1.0
    } else {
        0.0
    }
}
pub const LOG_LIMIT: u64 = 16 * 1024 * 1024;
/// The whole directory: about 8 h of town driving at 10 Hz, longer on the
/// open road where steady cruise is logged at 2 Hz.
pub const TOTAL_LIMIT: u64 = 256 * 1024 * 1024;
pub struct Trace {
    /// `None` when the directory budget was full at start: driving continues
    /// without diagnostics instead of deleting older evidence.
    writer: Option<BufWriter<File>>,
    pub path: PathBuf,
    root: PathBuf,
    bytes: u64,
    last_flush: Instant,
    last_write: Instant,
    pub exhausted: bool,
}
impl Trace {
    pub fn open(root: &Path) -> anyhow::Result<Self> {
        std::fs::create_dir_all(root)?;
        let (path, file) = Self::create(root)?;
        Ok(Self {
            writer: Some(BufWriter::with_capacity(64 * 1024, file)),
            path,
            root: root.to_owned(),
            bytes: 0,
            last_flush: Instant::now(),
            last_write: Instant::now() - Duration::from_secs(60),
            exhausted: false,
        })
    }
    /// Make room within the budget and create the next log file.
    fn create(root: &Path) -> anyhow::Result<(PathBuf, File)> {
        // Rotate: the oldest of our own `assist-*.jsonl` logs make room for
        // the new one, so diagnostics never stop and the directory stays
        // within budget. Files of any other name are never touched.
        let mut total = 0_u64;
        let mut ours = Vec::new();
        for entry in std::fs::read_dir(root)? {
            let entry = entry?;
            if entry.file_type()?.is_file() {
                let meta = entry.metadata()?;
                total = total.saturating_add(meta.len());
                let name = entry.file_name().to_string_lossy().into_owned();
                if name.starts_with("assist-") && name.ends_with(".jsonl") {
                    ours.push((meta.modified()?, meta.len(), entry.path()));
                }
            }
        }
        ours.sort_by_key(|o| o.0);
        for (_, len, path) in ours {
            if total + LOG_LIMIT <= TOTAL_LIMIT {
                break;
            }
            if std::fs::remove_file(&path).is_ok() {
                total = total.saturating_sub(len);
            }
        }
        ensure!(
            total + LOG_LIMIT <= TOTAL_LIMIT,
            "辅助日志目录被其他文件占满64MiB预算，本次不记录诊断"
        );
        let path = root.join(format!("assist-{}-{}.jsonl", std::process::id(), qpc()));
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)?;
        Ok((path, file))
    }
    pub fn open_or_disabled(root: &Path) -> Self {
        Self::open(root).unwrap_or_else(|_| Self {
            writer: None,
            path: root.join("（诊断日志预算已满，本次未记录）"),
            root: root.to_owned(),
            bytes: 0,
            last_flush: Instant::now(),
            last_write: Instant::now(),
            exhausted: true,
        })
    }
    pub fn since_last(&self) -> Duration {
        self.last_write.elapsed()
    }
    pub fn write(&mut self, value: &impl serde::Serialize) -> anyhow::Result<()> {
        let Some(writer) = self.writer.as_mut() else {
            return Ok(());
        };
        let mut bytes = serde_json::to_vec(value)?;
        bytes.push(b'\n');
        // A full file continues in the next one, the oldest logs making
        // room; driving is never ended by diagnostics.
        if self.bytes + bytes.len() as u64 > LOG_LIMIT {
            writer.flush()?;
            match Self::create(&self.root) {
                Ok((path, file)) => {
                    *writer = BufWriter::with_capacity(64 * 1024, file);
                    self.path = path;
                    self.bytes = 0;
                }
                Err(_) => {
                    self.writer = None;
                    self.exhausted = true;
                    return Ok(());
                }
            }
        }
        let writer = self.writer.as_mut().expect("rotated writer");
        writer.write_all(&bytes)?;
        self.last_write = Instant::now();
        self.bytes += bytes.len() as u64;
        if self.last_flush.elapsed() >= Duration::from_secs(1) {
            self.flush()?;
            self.last_flush = Instant::now();
        }
        Ok(())
    }
    pub fn flush(&mut self) -> anyhow::Result<()> {
        if let Some(writer) = self.writer.as_mut() {
            writer.flush()?;
        }
        Ok(())
    }
}

pub struct Overlay {
    _lease: SingleWriterGuard,
    cell: SharedCell<OverlayFrame>,
}
impl Overlay {
    pub fn connect() -> anyhow::Result<Self> {
        let lease = SingleWriterGuard::acquire("ETSADS_Overlay_Writer")?;
        let cell = SharedCell::create(OVERLAY_MAPPING, SCHEMA_VERSION)?;
        cell.recover_after_writer_crash();
        Ok(Self {
            _lease: lease,
            cell,
        })
    }
    pub fn publish(&self, frame: u64, label: &str, active: bool) -> anyhow::Result<()> {
        let mut f = OverlayFrame {
            source_frame_id: frame,
            write_qpc: qpc(),
            enabled: 1,
            ..Default::default()
        };
        for (i, c) in label.chars().take(12).enumerate() {
            let rows = glyph(c);
            for (y, row) in rows.into_iter().enumerate() {
                let mut x = 0;
                while x < 5 {
                    if row & (1 << (4 - x)) == 0 {
                        x += 1;
                        continue;
                    }
                    let start = x;
                    x += 1;
                    while x < 5 && row & (1 << (4 - x)) != 0 {
                        x += 1;
                    }
                    if f.count as usize >= f.rectangles.len() {
                        break;
                    }
                    f.rectangles[f.count as usize] = OverlayRect {
                        bounds: [
                            16.0 + (i * 18 + start * 3) as f32,
                            16.0 + (y * 3) as f32,
                            16.0 + (i * 18 + x * 3) as f32,
                            19.0 + (y * 3) as f32,
                        ],
                        color_rgba: if active {
                            [0.1, 1.0, 0.5, 1.0]
                        } else {
                            [1.0, 0.7, 0.1, 1.0]
                        },
                    };
                    f.count += 1;
                }
            }
        }
        self.cell.publish(&f)?;
        Ok(())
    }
}
impl Drop for Overlay {
    fn drop(&mut self) {
        let _ = self.cell.publish(&OverlayFrame {
            write_qpc: qpc(),
            ..Default::default()
        });
    }
}
fn glyph(c: char) -> [u8; 7] {
    match c {
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'I' => [31, 4, 4, 4, 4, 4, 31],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 25, 25, 21, 19, 19, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'W' => [17, 17, 17, 21, 21, 27, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 1, 14],
        _ => [0; 7],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::setup::SteeringMode;
    /// The game with relative steering as measured: the position moves at
    /// 0.4/s per unit input, linear, no self-centring; the bridge slews the
    /// input at 0.8/s.
    fn drive_relative(
        servo: &mut SteeringServo,
        position: &mut f64,
        target: impl Fn(f64) -> f64,
        seconds: f64,
    ) -> Vec<(f64, f64, f64)> {
        let dt = 1.0 / 60.0;
        let start = Instant::now();
        let mut applied = 0.0_f64;
        let mut trace = Vec::new();
        for i in 0..(seconds / dt) as usize {
            let t = i as f64 * dt;
            let now = start + Duration::from_secs_f64(t);
            servo.observe(-*position, 0.0, now);
            let goal = target(t);
            let input = servo.input(goal, now);
            applied = input.clamp(applied - 0.8 * dt, applied + 0.8 * dt);
            *position = (*position + 0.4 * applied * dt).clamp(-1.0, 1.0);
            trace.push((t, goal, *position));
        }
        trace
    }
    #[test]
    fn relative_servo_reaches_and_holds_a_position_without_overshoot() {
        let mut servo = SteeringServo::new(SteeringMode::Relative { rate_per_s: 0.4 });
        // The wheel left where the driver had it (as in the field test).
        let mut position = -0.37;
        let trace = drive_relative(&mut servo, &mut position, |_| 0.3, 6.0);
        let reached = trace
            .iter()
            .find(|(_, _, p)| (p - 0.3).abs() < 0.002)
            .unwrap()
            .0;
        assert!(reached < 3.0, "0.67 of travel at 0.4/s took {reached:.2} s");
        let overshoot = trace
            .iter()
            .map(|(_, _, p)| p - 0.3)
            .fold(f64::MIN, f64::max);
        assert!(overshoot < 0.01, "overshoot {overshoot}");
        assert!((position - 0.3).abs() < 0.002);
        // Holding: the input settles at zero, the wheel stays.
        let held = drive_relative(&mut servo, &mut position, |_| 0.3, 2.0);
        assert!(held.iter().all(|(_, _, p)| (p - 0.3).abs() < 0.002));
    }
    #[test]
    fn relative_servo_tracks_a_smooth_steering_change_closely() {
        let mut servo = SteeringServo::new(SteeringMode::Relative { rate_per_s: 0.4 });
        let mut position = 0.0;
        // Into a bend over 4 s: up to 0.225/s and 0.225/s².
        let smooth = |t: f64| {
            let x = ((t - 0.5) / 4.0).clamp(0.0, 1.0);
            0.6 * x * x * (3.0 - 2.0 * x)
        };
        let trace = drive_relative(&mut servo, &mut position, smooth, 6.0);
        let worst = trace
            .iter()
            .map(|(_, goal, p)| (goal - p).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.03, "worst tracking error {worst}");
        assert!((position - 0.6).abs() < 0.003);
    }
    #[test]
    fn a_shaped_step_is_followed_without_overshoot() {
        let mut servo = SteeringServo::new(SteeringMode::Relative { rate_per_s: 0.4 });
        let mut position = 0.0;
        // The controller asks for a sudden full turn and back: shaped, the
        // servo follows within a few hundredths and stops on the goal.
        let shaped = std::cell::RefCell::new((0.0, 0.0));
        let goal = |t: f64| if t < 4.0 { 0.8 } else { -0.3 };
        let trace = drive_relative(
            &mut servo,
            &mut position,
            |t| {
                let mut s = shaped.borrow_mut();
                *s = shape_steering(
                    s.0,
                    s.1,
                    goal(t),
                    0.0,
                    relative_limits(0.4, 0.0, 0.0),
                    1.0 / 60.0,
                );
                s.0
            },
            12.0,
        );
        let worst = trace
            .iter()
            .map(|(_, target, p)| (target - p).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.04, "worst tracking error {worst}");
        let overshoot = trace
            .iter()
            .map(|(_, _, p)| -0.3 - p)
            .fold(f64::MIN, f64::max);
        assert!(overshoot < 0.01, "overshoot {overshoot}");
        assert!((position + 0.3).abs() < 0.003);
    }
    #[test]
    fn shaping_follows_a_moving_goal_without_lag() {
        let (mut position, mut rate) = (0.0, 0.0);
        let dt = 1.0 / 60.0;
        for i in 0..300 {
            let goal = 0.2 * i as f64 * dt;
            (position, rate) = shape_steering(
                position,
                rate,
                goal,
                0.2,
                relative_limits(0.4, 0.0, 0.0),
                dt,
            );
        }
        assert!((position - 0.2 * 299.0 * dt).abs() < 0.01, "{position}");
        assert!((rate - 0.2).abs() < 0.01);
    }
    /// At speed the game returns the wheel toward the centre (r15: 0.10/s
    /// per m/s); the servo holds a bend's position with a steady input
    /// instead of trailing it by about a third.
    #[test]
    fn relative_servo_holds_a_turn_against_the_return_at_speed() {
        let mut servo = SteeringServo::new(SteeringMode::Relative { rate_per_s: 0.4 });
        let (dt, speed) = (1.0 / 60.0, 8.6);
        let start = Instant::now();
        let (mut position, mut applied) = (0.0_f64, 0.0_f64);
        let goal = |t: f64| 0.1 * ((t - 0.5) / 1.5).clamp(0.0, 1.0);
        let mut worst = 0.0_f64;
        for i in 0..(8.0 / dt) as usize {
            let t = i as f64 * dt;
            let now = start + Duration::from_secs_f64(t);
            servo.observe(-position, speed, now);
            let input = servo.input(goal(t), now);
            applied = input.clamp(applied - 0.8 * dt, applied + 0.8 * dt);
            position += (0.4 * applied - RELATIVE_CENTERING * speed * position) * dt;
            if t > 3.0 {
                worst = worst.max((goal(t) - position).abs());
            }
        }
        assert!(worst < 0.003, "held within {worst}");
        // Holding takes the input that balances the return.
        assert!((applied - RELATIVE_CENTERING * speed * 0.1 / 0.4).abs() < 0.01);
    }
    #[test]
    fn relative_servo_leaves_the_wheel_without_a_measurement() {
        let mut servo = SteeringServo::new(SteeringMode::Relative { rate_per_s: 0.4 });
        let now = Instant::now();
        servo.observe(0.5, 0.0, now - Duration::from_secs(1));
        assert_eq!(servo.input(0.2, now), 0.0);
        let mut absolute = SteeringServo::new(SteeringMode::Absolute);
        assert_eq!(absolute.input(0.2, now), 0.2);
        assert_eq!(absolute.max_rate(), None);
    }
    #[test]
    fn clutch_is_down_near_idle_and_only_held_at_a_standstill() {
        // Cruising or under throttle: the gearbox's own clutch.
        assert_eq!(clutch(0.0, 0.0, 20.0, 1400.0), 0.0);
        assert_eq!(clutch(0.3, 0.0, 2.0, 600.0), 0.0);
        // Coasting slowly, or braking at speed with the engine near idle.
        assert_eq!(clutch(0.0, 0.0, 2.0, 1200.0), 1.0);
        assert_eq!(clutch(0.0, 0.1, 14.0, 700.0), 1.0);
        // Stopped: down while the brake holds, released otherwise.
        assert_eq!(clutch(0.0, 0.3, 0.0, 550.0), 1.0);
        assert_eq!(clutch(0.0, 0.0, 0.0, 550.0), 0.0);
    }
    #[test]
    fn parking_completion_requires_the_requested_edge_not_an_axis_batch() {
        let receipt = ParkingReceipt {
            sequence: 20,
            counter: 7,
            qpc: 100,
            frame: 9,
            cursor: 0,
            acknowledged: false,
            desired: true,
            completed: None,
        };
        let mut event = InputActionFrame {
            command_seq: 20,
            prepared_qpc: 101,
            completed_qpc: 110,
            source_frame_id: 9,
            pulse_mask: 1 << 17,
            ..Default::default()
        };
        event.pulse_counters[17] = 7;
        assert!(receipt.matches(&event));
        event.pulse_mask = 0;
        assert!(!receipt.matches(&event));
        event.pulse_mask = 1 << 17;
        event.pulse_counters[17] = 6;
        assert!(!receipt.matches(&event));
        event.pulse_counters[17] = 7;
        event.completed_qpc = 0;
        assert!(!receipt.matches(&event));
        event.completed_qpc = 110;
        event.command_seq = 19;
        assert!(!receipt.matches(&event));
    }
    #[cfg(windows)]
    #[test]
    fn first_request_and_unwind_release_axes_preserving_pulses_without_writing_gate() {
        let suffix = format!("_AssistTest_{}_{}", std::process::id(), qpc());
        let name = |base: &str| format!("{base}{suffix}");
        let requests =
            SharedCell::<SafetyRequest>::create(&name(SAFETY_REQUEST_MAPPING), SCHEMA_VERSION)
                .unwrap();
        let gate = SharedCell::<Gate>::create(&name(GATE_MAPPING), SCHEMA_VERSION).unwrap();
        let _applied =
            SharedCell::<AppliedControl>::create(&name(APPLIED_MAPPING), SCHEMA_VERSION).unwrap();
        let command =
            SharedCell::<ControlCmd>::create(&name(CONTROL_MAPPING), SCHEMA_VERSION).unwrap();
        let mut old = ControlCmd {
            seq: 400,
            ..Default::default()
        };
        old.pulses[45] = 17;
        command.publish(&old).unwrap();
        gate.publish(&Gate {
            enabled: GATE_ACTIVE,
            seq: 92,
            ..Default::default()
        })
        .unwrap();
        let result = std::panic::catch_unwind(|| {
            let mut sink = Sink::connect_named(&suffix).unwrap();
            assert_eq!(requests.latest().unwrap().arm, 0);
            sink.request(true).unwrap();
            sink.publish(
                20,
                super::super::core::Output {
                    steer: 0.1,
                    throttle: 0.2,
                    brake: 0.0,
                },
            )
            .unwrap();
            panic!("test owner failure");
        });
        assert!(result.is_err());
        let last = command.latest().unwrap();
        assert_eq!(
            [last.steer, last.throttle, last.brake, last.clutch],
            [0.0; 4]
        );
        assert_eq!(last.pulses[45], 17);
        assert!(last.seq > 400);
        assert_eq!(requests.latest().unwrap().arm, 0);
        assert_eq!(gate.latest().unwrap().seq, 92);
    }
    #[test]
    fn log_budget_rotates_only_our_oldest_logs() {
        let root = std::env::temp_dir().join(format!(
            "etsads-assist-budget-{}-{}",
            std::process::id(),
            qpc()
        ));
        std::fs::create_dir(&root).unwrap();
        // A foreign file filling the budget is never deleted.
        let retained = root.join("retained.jsonl");
        File::create(&retained)
            .unwrap()
            .set_len(TOTAL_LIMIT)
            .unwrap();
        assert!(Trace::open(&root).is_err());
        assert_eq!(std::fs::metadata(&retained).unwrap().len(), TOTAL_LIMIT);
        std::fs::remove_file(retained).unwrap();
        // Our own logs rotate oldest first.
        let old = root.join("assist-1-1.jsonl");
        File::create(&old)
            .unwrap()
            .set_len(TOTAL_LIMIT / 2)
            .unwrap();
        std::thread::sleep(Duration::from_millis(20));
        let recent = root.join("assist-2-2.jsonl");
        File::create(&recent)
            .unwrap()
            .set_len(TOTAL_LIMIT / 2 - 1)
            .unwrap();
        let trace = Trace::open(&root).unwrap();
        assert!(!old.exists() && recent.exists());
        let path = trace.path.clone();
        drop(trace);
        std::fs::remove_file(path).unwrap();
        std::fs::remove_file(recent).unwrap();
        std::fs::remove_dir(root).unwrap();
    }
}
