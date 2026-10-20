//! Explicit experimental road assistance, separate from certified Runtime
//! modes. This consumes measured static steering, not a dynamic certificate.
pub mod calibrate;
mod core;
mod finish_check;
mod io;
mod navigation;
mod safety_diagnostics;
mod stopping;
pub mod trailer;
pub mod trip_stats;

use anyhow::{Context, ensure};
use core::{DRIFT_WARN_M, Feedback, MAX_SPEED, Output, SteeringLibrary};
pub use core::{Longitudinal, MAX_SECONDS, STEERING_NOT_CALIBRATED};
use etsads_game::adapter::{GameAdapter, GameSnapshot};
use etsads_msgs::{Agent, Pose2, protocol::*};
use etsads_shm::{SharedRing, SingleWriterGuard, qpc, qpc_frequency};
pub use navigation::{LeadEvidence, Motion};
use serde::Serialize;
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::SyncSender,
    },
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub struct Options {
    pub map: PathBuf,
    /// An extra steering report (developer override); per-truck reports made
    /// by the one-click calibration live in `steering_dir`.
    pub steering_report: PathBuf,
    pub steering_dir: PathBuf,
    pub log_dir: PathBuf,
    pub seconds: u64,
    pub maximum_kmh: f64,
    pub arm: bool,
    pub prepare_seconds: u64,
    pub stop_at_end: bool,
    /// Return to standby after each engagement instead of exiting. Every
    /// engagement still needs a fresh, explicit hotkey press.
    pub continuous: bool,
    /// Below this fuel level (litres) the truck is stopped on the road, so a
    /// test drive can refuel and continue; `None` only warns.
    pub fuel_stop_l: Option<f32>,
}
impl Default for Options {
    fn default() -> Self {
        let mut roots = Vec::new();
        if let Ok(cwd) = std::env::current_dir() {
            roots.push(cwd);
        }
        if let Ok(exe) = std::env::current_exe() {
            for p in exe.ancestors().skip(1).take(5) {
                roots.push(p.to_path_buf());
            }
        }
        let root = roots
            .iter()
            .find(|p| {
                p.join("tools/map_export/output/sector-europe-profile-v3-final/sector_index.json")
                    .exists()
            })
            .cloned()
            .or_else(|| {
                roots
                    .iter()
                    .find(|p| p.join("Cargo.toml").is_file())
                    .cloned()
            })
            .unwrap_or_else(|| PathBuf::from("."));
        Self {
            map: crate::setup::default_map_dir(),
            // Prefer the full-range r9 scan; the r7 report covers only ±0.25.
            steering_report: [
                "artifacts/r9-static-steering-wide.json",
                "artifacts/live-r7-absolute-steering-01-static-identification.json",
            ]
            .iter()
            .map(|p| root.join(p))
            .find(|p| p.is_file())
            .unwrap_or_else(|| root.join("artifacts/r9-static-steering-wide.json")),
            steering_dir: crate::setup::data_dir().join("steering"),
            log_dir: root.join("runs/assist"),
            seconds: 60,
            maximum_kmh: 30.0,
            arm: false,
            prepare_seconds: 300,
            stop_at_end: false,
            continuous: false,
            fuel_stop_l: None,
        }
    }
}
/// What the driver needs to know, independent of the wording of `phase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Mode {
    Starting,
    Unavailable,
    Ready,
    Engaged,
    TakeoverSoon,
    SafetyBrake,
    Stopping,
    Released,
}
#[derive(Clone, Debug, Serialize)]
pub struct Progress {
    #[serde(skip)]
    pub produced_at: Instant,
    pub mode: Mode,
    pub phase: String,
    pub reason: String,
    pub frame_id: u64,
    pub source_qpc: u64,
    pub position_enu: [f64; 3],
    pub heading_rad: f64,
    pub requested: Option<Output>,
    pub applied_axes: Option<[f32; 4]>,
    pub applied_seq: Option<u64>,
    /// Why the bridge applied or withheld the command (0 = applied).
    pub applied_reason: Option<u32>,
    /// The game's effective steer, throttle, brake, clutch.
    pub effective: [f32; 4],
    pub gate: Option<u32>,
    pub gate_aeb_brake: Option<f32>,
    pub primary_obstacle: Option<[f32; 2]>,
    pub speed_kmh: f64,
    pub target_kmh: f64,
    /// The constraint that set the speed target.
    pub binding: Option<navigation::Binding>,
    pub limit_kmh: f64,
    pub lateral_m: f64,
    /// Mean steered-wheel angle from telemetry (rad, left positive).
    pub wheel_angle_rad: f64,
    /// Yaw rate (rad/s, left positive).
    pub yaw_rate: f64,
    pub lane: String,
    pub lead_gap_m: Option<f64>,
    pub lead: Option<LeadEvidence>,
    pub longitudinal: Option<Longitudinal>,
    /// Distance to the end of the usable route prefix (stop point).
    pub remaining_route_m: Option<f64>,
    pub terminal: String,
    pub engagements: u64,
    pub hud_visible: bool,
    pub maximum_kmh: f64,
    pub unlimited: bool,
    pub remaining_s: f64,
    pub parking_brake: bool,
    pub fuel_l: f32,
    pub rpm: f32,
    /// GPS distance to the destination, in the game's scaled kilometres.
    pub navigation_km: f64,
}
impl Default for Progress {
    fn default() -> Self {
        Self {
            produced_at: Instant::now(),
            mode: Mode::Starting,
            phase: "准备中".into(),
            reason: "读取本地地图和实测转向表".into(),
            frame_id: 0,
            source_qpc: 0,
            position_enu: [0.0; 3],
            heading_rad: 0.0,
            requested: None,
            applied_axes: None,
            applied_seq: None,
            applied_reason: None,
            effective: [0.0; 4],
            gate: None,
            gate_aeb_brake: None,
            primary_obstacle: None,
            speed_kmh: 0.0,
            target_kmh: 0.0,
            binding: None,
            limit_kmh: 0.0,
            lateral_m: 0.0,
            wheel_angle_rad: 0.0,
            yaw_rate: 0.0,
            lane: String::new(),
            lead_gap_m: None,
            lead: None,
            longitudinal: None,
            remaining_route_m: None,
            terminal: String::new(),
            engagements: 0,
            hud_visible: false,
            maximum_kmh: 0.0,
            unlimited: false,
            remaining_s: 0.0,
            parking_brake: false,
            fuel_l: 0.0,
            rpm: 0.0,
            navigation_km: 0.0,
        }
    }
}
#[derive(Debug, Serialize)]
pub struct Report {
    pub source: &'static str,
    pub stop_reason: String,
    pub active_frames: u64,
    pub log: PathBuf,
    pub full_acceptance: bool,
    pub parking_confirmed: bool,
}
pub(super) fn bounded_read(path: &Path, limit: usize) -> anyhow::Result<Vec<u8>> {
    use std::io::Read;
    let f = std::fs::File::open(path)?;
    ensure!(f.metadata()?.len() <= limit as u64, "文件超过读取预算");
    let mut bytes = Vec::new();
    f.take(limit as u64 + 1).read_to_end(&mut bytes)?;
    ensure!(bytes.len() <= limit, "文件超过读取预算");
    Ok(bytes)
}
pub(super) fn neutral() -> Output {
    Output {
        steer: 0.0,
        throttle: 0.0,
        brake: 0.0,
    }
}
impl Options {
    pub fn steering(&self) -> SteeringLibrary {
        SteeringLibrary::load(
            &self.steering_dir,
            Some(&self.steering_report)
                .filter(|p| p.is_file())
                .map(PathBuf::as_path),
        )
    }
}
/// Number of trucks with a valid steering table in `dir`.
pub fn steering_reports(dir: &Path) -> usize {
    SteeringLibrary::load(dir, None).len()
}
/// Whether this truck (by its current SDK configuration) has a table.
pub fn steering_calibrated(options: &Options, config: &ConfigBlock) -> bool {
    options.steering().vehicle(config).is_ok()
}
/// Read-only: whether the truck currently driven in the game has a steering
/// table. `None` while the game is not in the driving view.
pub fn current_truck_calibrated(options: &Options) -> Option<bool> {
    let mut game = GameAdapter::connect().ok()?;
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(400) {
        if let Ok(Some(s)) = game.poll() {
            if !etsads_safety::fresh(qpc(), s.raw.header.qpc, qpc_frequency(), 500) {
                return None;
            }
            return s.config.as_ref().map(|c| steering_calibrated(options, c));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    None
}
pub fn check_finish(arm: bool, stop: Arc<AtomicBool>) -> anyhow::Result<Report> {
    finish_check::run(arm, stop)
}

/// Read-only field diagnosis. Does not acquire a writer, start the supervisor,
/// publish control/overlay or create a recording.
pub fn inspect(options: Options) -> anyhow::Result<serde_json::Value> {
    let steering = options.steering();
    let navigator = navigation::Navigator::open(&options.map)?;
    let mut game = GameAdapter::connect()?;
    let start = Instant::now();
    loop {
        if let Some(s) = game.poll()? {
            ensure!(
                etsads_safety::fresh(qpc(), s.raw.header.qpc, qpc_frequency(), 200),
                "遥测过期"
            );
            let (axles, _) = steering.vehicle(s.config.as_ref().context("车辆配置缺失")?)?;
            let rear = axles.rear_pose(s.ego.pose);
            let route = navigator
                .prepare_from(rear, s.ego.z, s.ego.speed, &s.scene.route, route_position(&s), None)
                .map(|g| serde_json::json!({"usable":true,"terminal":g.terminal,"controls":g.describe_controls()}))
                .unwrap_or_else(|e| serde_json::json!({"usable":false,"reason":format!("{e:#}")}));
            // Raw ETS2LA route records: node, distance and time to the end.
            let route_records: Vec<_> = s
                .extension
                .as_deref()
                .map(|e| {
                    etsads_game::ets2la::records::<etsads_game::ets2la::RouteNode>(&e.route)
                        .filter(|r| r.node_uid != 0)
                        .map(|r| {
                            let (node, distance, time) =
                                (r.node_uid, r.distance_to_end, r.time_to_end);
                            serde_json::json!([node, distance, time])
                        })
                        .collect()
                })
                .unwrap_or_default();
            return Ok(
                serde_json::json!({"frame":s.raw.header.frame_id,"qpc":s.raw.header.qpc,"navigation_distance_m":s.raw.navigation_distance_m,"navigation_time_s":s.raw.navigation_time_s,"route_records":route_records,"sdk_pose":s.ego.pose,"rear_pose":rear,"z":s.ego.z,"speed_kmh":s.ego.speed*3.6,"rpm":s.raw.rpm,"rest_stop_min":s.raw.rest_stop_min,"game_time_min":s.raw.game_time_min,"gear":s.raw.gear,"displayed_gear":s.raw.displayed_gear,"parking_brake":s.raw.parking_brake,"brake_air_pressure_psi":s.raw.brake_air_pressure_psi,"motor_brake":s.raw.motor_brake,"wear":s.raw.wear,"wheel_on_ground":s.raw.wheel_on_ground,"limit_kmh":s.scene.speed_limit*3.6,"gps":s.scene.route,"available":s.extension.as_ref().map(|e|e.available),"signals_seen":signal_observations(&s),"route":route,"route_probe":navigator.route_probe(rear,s.ego.z,s.ego.speed,&s.scene.route,route_position(&s)).unwrap_or_else(|e| serde_json::json!({"error":format!("{e:#}")})),"map":navigator.inspect(rear,s.ego.z)?,"agents":traffic(&s)?,"safety":s.extension.as_deref().map(|e|safety_diagnostics::inspect(&s.raw,e))}),
            );
        }
        ensure!(start.elapsed() < Duration::from_secs(2), "未收到新遥测帧");
        std::thread::sleep(Duration::from_millis(10));
    }
}

pub fn run(
    options: Options,
    stop: Arc<AtomicBool>,
    progress: Option<SyncSender<Progress>>,
) -> anyhow::Result<Report> {
    ensure!(
        options.seconds == 0 || (5..=MAX_SECONDS).contains(&options.seconds),
        "单次时长须为不限时或5～{MAX_SECONDS}秒"
    );
    ensure!(
        options.maximum_kmh.is_finite() && (10.0..=MAX_SPEED * 3.6).contains(&options.maximum_kmh),
        "速度上限须为10～90km/h"
    );
    ensure!(
        (1..=600).contains(&options.prepare_seconds),
        "准备等待上限须为1～600秒"
    );
    // Serialize both control ownership and log budget reservation. No second
    // writer may produce inputs or grow diagnostics concurrently.
    let _writer = SingleWriterGuard::acquire("ETSADS_Control_Writer")?;
    let mut steering = options.steering();
    let mut steering_loaded = Instant::now();
    // The game profile's steering mode, re-read while waiting so a changed
    // setting or profile is picked up before the next engagement.
    let mut profile_steering = crate::setup::active_steering();
    let mut profile_checked = Instant::now();
    let navigator = navigation::Navigator::open(&options.map)?;
    let mut trace = io::Trace::open_or_disabled(&options.log_dir);
    trace.write(&serde_json::json!({"kind":"experimental_assist_mvp","source":"game","maximum_kmh":options.maximum_kmh,"seconds":options.seconds,"stop_at_end":options.stop_at_end,"map":options.map,"steering_report":options.steering_report,"full_acceptance":false,"static_steering_only":true}))?;
    let _supervisor = io::SupervisorProcess::ensure()?;
    let mut sink = io::Sink::connect()?;
    let actions = SharedRing::<InputActionFrame, 256>::open(INPUT_ACTIONS_MAPPING, SCHEMA_VERSION)
        .context("需要带SDK输入完成记录的插件以确认停车动作")?;
    let overlay = io::Overlay::connect()?;
    // The HUD is a convenience: without it the rectangle label still shows.
    let hud = crate::hud::HudPublisher::start().ok();
    let mut hud_checked = Instant::now() - Duration::from_secs(2);
    // The planned path lets the independent supervisor judge obstacles along
    // the road ahead; without it the supervisor keeps its yaw-rate arc.
    let planned_path = SharedRing::<PlannedPath, 2>::create(PLANNED_PATH_MAPPING, SCHEMA_VERSION)
        .ok()
        .inspect(|ring| {
            ring.recover_after_writer_crash();
        });
    let mut game = GameAdapter::connect()?;
    game.require_extension = true;
    let mut controls = crate::loaded_controls::LoadedControls::connect("")?;
    let mut status = Progress {
        maximum_kmh: options.maximum_kmh,
        unlimited: options.seconds == 0,
        ..Default::default()
    };
    let mut started = Instant::now();
    let mut last_frame_at = Instant::now();
    let mut last_prepare = Instant::now() - Duration::from_secs(2);
    let mut last_report = Instant::now() - Duration::from_secs(1);
    let mut guidance = None;
    let mut active = None::<Instant>;
    let mut stable_since = None::<Instant>;
    let mut feedback = None;
    let mut last_sim = 0;
    let mut frames = 0;
    let mut auto_arm_sent = false;
    let mut last_safety_diagnostic = Instant::now() - Duration::from_secs(2);
    let mut safety_diagnostics_count = 0;
    let mut ending = None::<stopping::StopSequence>;
    let mut parking_receipt = None::<io::ParkingReceipt>;
    let mut parking_confirmed = false;
    let mut ending_reason = "本次接管时长结束";
    // A Gate that is still Active after a release (or left over from before
    // this process) must be seen off once; only a new hotkey press engages.
    let mut gate_seen_off = false;
    let mut released = None::<(Instant, String)>;
    // A tracking or planning fault while engaged does not let go of a moving
    // truck (r10 rolled into a guardrail at 14 km/h): hold the wheel, brake to
    // a stop and park. Manual takeover still releases at once.
    let mut degraded = None::<String>;
    // Turn signal the assist switched on (true = left) and when it last
    // pressed one: the switch state shows up in telemetry a frame later.
    let mut blinker_ours = None::<bool>;
    let mut blinker_pressed = Instant::now() - Duration::from_secs(5);
    let mut lights_pressed = Instant::now() - Duration::from_secs(5);
    let mut held_behind = None::<Instant>;
    let mut bad_frames = 0_u64;
    // Held through a game pause; after it, the gate Off -> Active handshake.
    let mut held_pause = false;
    let mut resume = None::<Rearm>;
    // First frame of a low engine speed while engaged.
    let mut low_rpm_since = None::<Instant>;
    let mut restart = None::<Restart>;
    // Reference (time, axis, wheel) for detecting inputs the game ignores.
    let mut input_check = None::<(Instant, f64, f64)>;
    let mut input_frozen = false;
    // Since when (and from which position) the steering has disagreed with an
    // absolute-mode input.
    let mut steering_mismatch = None::<(Instant, f64)>;
    // Speed held while relative steering catches up.
    let mut relative_hold = core::RelativeHold::default();
    let mut trailers = trailer::TrailerReader::default();
    let mut combination = trailer::Combination::default();
    let mut fatigue_seen = false;
    let mut pushing_since = None::<Instant>;
    // Minimal-risk stop that could not confirm parking: brake until takeover.
    let mut brake_held = false;
    let mut preparer = navigation::Preparer::spawn(options.map.clone());
    let mut last_extension_error = String::new();
    let result = loop {
        let outcome = (|| -> anyhow::Result<String> {
            macro_rules! minimal_risk_stop {
                ($error:expr) => {{
                    let reason = format!("{:#}", $error);
                    trace.write(&serde_json::json!({"event":"minimal_risk_stop","frame":status.frame_id,"qpc":status.source_qpc,"reason":reason}))?;
                    degraded = Some(reason);
                    ending = Some(stopping::StopSequence::urgent(
                        started.elapsed().as_secs_f64(),
                    ));
                    parking_receipt = None;
                    continue;
                }};
            }
            loop {
                if stop.load(Ordering::Acquire) {
                    return Ok("已手动停止，控制已释放".into());
                }
                if !options.continuous
                    && active.is_none()
                    && started.elapsed().as_secs() >= options.prepare_seconds
                {
                    return Ok("等待超时，请上路后重新准备".into());
                }
                if options.seconds != 0
                    && active.is_some_and(|t| t.elapsed().as_secs_f64() >= options.seconds as f64)
                {
                    if options.stop_at_end {
                        ending.get_or_insert_with(|| {
                            stopping::StopSequence::new(started.elapsed().as_secs_f64())
                        });
                    } else {
                        return Ok("本次接管时长结束，请继续手动驾驶".into());
                    }
                }
                // One implausible frame is skipped like a missing one; a
                // lasting fault trips the 150 ms data check below. L2 exited
                // the whole assist over one out-of-order frame.
                let polled = match game.poll() {
                    Ok(polled) => polled,
                    Err(error) => {
                        if bad_frames == 0 {
                            trace.write(&serde_json::json!({"event":"telemetry_frame_skipped","frame":status.frame_id,"reason":error.to_string()}))?;
                        }
                        bad_frames += 1;
                        None
                    }
                };
                let Some(snapshot) = polled else {
                    // A frozen game (an autosave) produces no frames and moves
                    // nothing; the bridge brakes on its own once commands go
                    // stale. r13 released at 30 km/h over such a gap and the
                    // truck coasted on when the game resumed.
                    if active.is_some() {
                        ensure!(
                            last_frame_at.elapsed() < TELEMETRY_GAP,
                            "遥测或交通数据中断，控制已释放"
                        );
                    }
                    std::thread::sleep(Duration::from_millis(3));
                    continue;
                };
                last_frame_at = Instant::now();
                bad_frames = 0;
                status.frame_id = snapshot.raw.header.frame_id;
                status.source_qpc = snapshot.raw.header.qpc;
                status.position_enu = [snapshot.ego.pose.x, snapshot.ego.pose.y, snapshot.ego.z];
                status.heading_rad = snapshot.ego.pose.yaw;
                status.requested = Some(neutral());
                status.speed_kmh = snapshot.ego.speed * 3.6;
                sink.observe(
                    snapshot.ego.speed,
                    snapshot.raw.rpm as f64,
                    snapshot.raw.effective[0] as f64,
                );
                status.yaw_rate =
                    snapshot.raw.angular_velocity_local[1] as f64 * std::f64::consts::TAU;
                status.wheel_angle_rad = snapshot.config.as_ref().map_or(0.0, |c| {
                    let steered: Vec<f64> = (0..(c.wheel_count as usize).min(16))
                        .filter(|i| c.wheel_steerable[*i] != 0)
                        .map(|i| snapshot.raw.wheel_steering[i] as f64 * std::f64::consts::TAU)
                        .collect();
                    steered.iter().sum::<f64>() / steered.len().max(1) as f64
                });
                status.limit_kmh = snapshot.scene.speed_limit * 3.6;
                status.parking_brake = snapshot.raw.parking_brake != 0;
                status.fuel_l = snapshot.raw.fuel_l;
                status.rpm = snapshot.raw.rpm;
                status.navigation_km = snapshot.raw.navigation_distance_m as f64 / 1000.0;
                let observed_gate = crate::loaded_controls::coherent_read(|| sink.gate.latest());
                let applied = crate::loaded_controls::coherent_read(|| sink.applied.latest());
                status.applied_axes = applied.map(|a| a.axes);
                status.applied_seq = applied.map(|a| a.applied_seq);
                status.applied_reason = applied.map(|a| a.reason);
                status.effective = snapshot.raw.effective;
                status.gate = observed_gate.map(|g| g.enabled);
                status.gate_aeb_brake = observed_gate.map(|g| g.aeb_brake);
                status.primary_obstacle = snapshot
                    .extension
                    .as_deref()
                    .and_then(|e| {
                        etsads_safety::primary_obstacle(&snapshot.raw, e)
                            .ok()
                            .flatten()
                    })
                    .map(|o| [o.distance_m, o.closing_speed_mps]);
                // An engaged drive is held through a game pause (Esc, the
                // map): the game ignores inputs meanwhile, and the supervisor
                // keeps authority with no throttle. L7 and L8 released here
                // and coasted into the barrier on resume.
                if active.is_some() && ending.is_none() {
                    if snapshot.ego.paused {
                        ensure!(
                            resume.is_some()
                                || observed_gate.is_some_and(|g| g.enabled == GATE_ACTIVE
                                    && etsads_safety::fresh(
                                        qpc(),
                                        g.heartbeat_qpc,
                                        qpc_frequency(),
                                        150
                                    )),
                            "已交还人工驾驶（人工接管、热键退出或安全监督释放）"
                        );
                        held_pause = true;
                    } else if held_pause {
                        // The bridge drops its input authority on every pause
                        // and only accepts commands again after seeing the gate
                        // go Off then Active (L9 steered nothing after resume).
                        held_pause = false;
                        sink.request(false)?;
                        resume = Some(Rearm::OffRequested(Instant::now()));
                        trace.write(&serde_json::json!({"event":"pause_resume_rearm","frame":status.frame_id}))?;
                    }
                    if let Some(state) = resume
                        && !snapshot.ego.paused
                    {
                        let gate_is = |state| observed_gate.is_some_and(|g| g.enabled == state);
                        resume = match state {
                            Rearm::OffRequested(since) if gate_is(GATE_OFF) => {
                                Some(Rearm::OffSeen(since, Instant::now()))
                            }
                            // The bridge must see Off in an input frame or two.
                            Rearm::OffSeen(since, off)
                                if off.elapsed() >= Duration::from_millis(60)
                                    && gate_is(GATE_OFF) =>
                            {
                                sink.request(true)?;
                                Some(Rearm::OnRequested(since))
                            }
                            Rearm::OnRequested(since)
                                if gate_is(GATE_ACTIVE)
                                    && applied.is_some_and(|a| a.reason == 0) =>
                            {
                                trace.write(&serde_json::json!({"event":"pause_resume_active","frame":status.frame_id,"ms":since.elapsed().as_millis() as u64}))?;
                                None
                            }
                            other => Some(other),
                        };
                        ensure!(
                            resume
                                .is_none_or(|r| r.since().elapsed() < Duration::from_millis(1500)),
                            "暂停恢复后未能重新取得控制，请立即接管"
                        );
                    }
                    if snapshot.ego.paused || resume.is_some() {
                        // The controller steps from here on: no simulated time
                        // passes for it while held (a stale clock tripped the
                        // 0.1 s step check right after the handshake).
                        last_sim = snapshot.raw.paused_simulation_time_us;
                        let hold = Output {
                            steer: feedback.as_ref().map_or(0.0, |f: &Feedback| f.held_axis()),
                            throttle: 0.0,
                            brake: 0.0,
                        };
                        sink.publish(status.frame_id, hold)?;
                        status.requested = Some(hold);
                        status.phase = if snapshot.ego.paused {
                            "游戏已暂停"
                        } else {
                            "恢复控制中"
                        }
                        .into();
                        status.reason =
                            "恢复游戏后自动继续驾驶；方向键 / WASD / 空格 / 手柄 随时接管".into();
                        report(
                            &mut status,
                            &progress,
                            &mut trace,
                            &overlay,
                            hud.as_ref(),
                            &released,
                            &mut hud_checked,
                            &mut last_report,
                            true,
                        )?;
                        continue;
                    }
                }
                // A damaged engine can stall mid-drive (L11 at 18 km/h): brake to
                // a stop on the road instead of releasing a rolling truck.
                // Confirmed over STALL_CONFIRM, so one stray reading does not
                // stop the truck (r13 stopped at 50 km/h during an automatic
                // downshift; rpm is now in the trace to tell the two apart).
                let running = snapshot.raw.rpm.is_finite() && snapshot.raw.rpm > 300.0;
                if (running && restart.is_none()) || active.is_none() {
                    low_rpm_since = None;
                    restart = None;
                } else if ending.is_none() && !running {
                    low_rpm_since.get_or_insert_with(Instant::now);
                }
                // Stopped in traffic the game sometimes lets the engine die
                // (r13: 8 s after stopping, clutch requested but not applied).
                // At a standstill it is restarted in place (see `Restart`).
                let mut stalled = low_rpm_since.is_some_and(|t| t.elapsed() >= STALL_CONFIRM);
                if active.is_some()
                    && ending.is_none()
                    && (stalled || restart.is_some())
                    && snapshot.ego.speed.abs() < 0.3
                {
                    let state = restart.get_or_insert_with(|| {
                        let _ = trace.write(&serde_json::json!({"event":"engine_restart_begin","frame":status.frame_id}));
                        Restart::new()
                    });
                    let request = state.step(
                        running,
                        snapshot.raw.parking_brake != 0,
                        snapshot.raw.displayed_gear > 0,
                    );
                    if let Some(pulse) = request.pulse {
                        sink.press(pulse);
                    }
                    if state.done {
                        trace.write(&serde_json::json!({"event":"engine_restarted","frame":status.frame_id,"ms":state.since.elapsed().as_millis() as u64}))?;
                        restart = None;
                        low_rpm_since = None;
                        stalled = false;
                    } else {
                        ensure!(
                            state.since.elapsed() < RESTART_LIMIT,
                            "发动机无法重新启动，请手动处理"
                        );
                        last_sim = snapshot.raw.paused_simulation_time_us;
                        let hold = Output {
                            steer: feedback.as_ref().map_or(0.0, |f: &Feedback| f.held_axis()),
                            throttle: request.throttle,
                            brake: request.brake,
                        };
                        sink.publish(status.frame_id, hold)?;
                        status.requested = Some(hold);
                        status.phase = "重新启动发动机".into();
                        status.reason = "发动机熄火，停车状态下自动重新启动".into();
                        report(
                            &mut status,
                            &progress,
                            &mut trace,
                            &overlay,
                            hud.as_ref(),
                            &released,
                            &mut hud_checked,
                            &mut last_report,
                            true,
                        )?;
                        continue;
                    }
                }
                // Moving, the truck keeps its lane and brakes to a stop (air
                // brakes and steering still work), then restarts as above. The
                // minimal-risk stop only holds the wheel, unsafe in a bend.
                let stall_rolling = active.is_some() && ending.is_none() && stalled;
                let now = qpc();
                let valid = (|| -> anyhow::Result<_> {
                    validate_snapshot(&snapshot, now, ending.is_some(), low_rpm_since.is_some())?;
                    controls.validate(&snapshot.raw)?;
                    ensure!(
                        applied.is_some_and(|a| etsads_safety::fresh(
                            now,
                            a.applied_qpc,
                            qpc_frequency(),
                            150
                        )),
                        "输入回调未就绪"
                    );
                    ensure!(
                        observed_gate.is_some_and(|g| etsads_safety::fresh(
                            now,
                            g.heartbeat_qpc,
                            qpc_frequency(),
                            150
                        )),
                        "安全监督心跳失效"
                    );
                    let (axles, _) =
                        steering.vehicle(snapshot.config.as_ref().context("车辆配置未知")?)?;
                    let pose = axles.rear_pose(snapshot.ego.pose);
                    let agents = traffic(&snapshot)?;
                    Ok((axles, pose, agents))
                })();
                let (axles, pose, agents) = match valid {
                    Ok(value) => value,
                    Err(error) => {
                        if active.is_some() {
                            return Err(error);
                        }
                        sink.publish(status.frame_id, neutral())?;
                        if observed_gate.is_some_and(|g| g.enabled != GATE_OFF) {
                            sink.request(false)?;
                        }
                        stable_since = None;
                        guidance = None;
                        // A calibration made from the panel meanwhile is
                        // picked up without restarting the assist.
                        if format!("{error:#}").contains("还没有标定转向")
                            && steering_loaded.elapsed() > Duration::from_secs(2)
                        {
                            steering = options.steering();
                            steering_loaded = Instant::now();
                        }
                        status.phase = "等待上路".into();
                        status.mode = Mode::Unavailable;
                        status.reason = format!("{error:#}");
                        report(
                            &mut status,
                            &progress,
                            &mut trace,
                            &overlay,
                            hud.as_ref(),
                            &released,
                            &mut hud_checked,
                            &mut last_report,
                            false,
                        )?;
                        continue;
                    }
                };
                if observed_gate.is_some_and(|g| g.aeb_brake > 0.0)
                    && last_safety_diagnostic.elapsed() >= Duration::from_secs(1)
                    && safety_diagnostics_count < 20
                    && let Some(ext) = snapshot.extension.as_deref()
                {
                    trace.write(&serde_json::json!({"event":"independent_aeb_observation","gate_aeb_brake":status.gate_aeb_brake,"diagnostic":safety_diagnostics::inspect(&snapshot.raw,ext)}))?;
                    last_safety_diagnostic = Instant::now();
                    safety_diagnostics_count += 1;
                }
                if active.is_some()
                    && let Some(reason) = degraded.clone()
                {
                    ensure!(
                        observed_gate.is_some_and(|g| g.enabled == GATE_ACTIVE),
                        "已交还人工驾驶（人工接管、热键退出或安全监督释放）"
                    );
                    let acknowledged = match &mut parking_receipt {
                        Some(receipt) => {
                            receipt.confirmed(&actions, &snapshot.raw).unwrap_or(false)
                        }
                        None => false,
                    };
                    // Parking unconfirmed or too slow: never let go of the
                    // truck; keep the service brake on until the driver takes
                    // over (r11 released here and the truck rolled back).
                    let action = match ending.as_mut().context("安全停车状态缺失")?.update(
                        started.elapsed().as_secs_f64(),
                        snapshot.ego.speed,
                        status.parking_brake,
                        acknowledged,
                    ) {
                        Ok(action) if !brake_held => action,
                        Ok(_) => stopping::Action::Brake(0.6),
                        Err(error) => {
                            if !brake_held {
                                trace.write(&serde_json::json!({"event":"minimal_risk_brake_hold","frame":status.frame_id,"reason":format!("{error:#}")}))?;
                                brake_held = true;
                            }
                            stopping::Action::Brake(0.6)
                        }
                    };
                    if action == stopping::Action::Complete {
                        parking_confirmed = true;
                        trace.write(&serde_json::json!({"event":"parking_confirmed","frame":status.frame_id,"qpc":status.source_qpc,"sdk_evidence":parking_receipt.as_ref().and_then(|p|p.evidence())}))?;
                        return Ok(format!("{reason}；已安全停车并拉起驻车，请手动驾驶"));
                    }
                    let output = respect_safety(
                        Output {
                            steer: feedback.as_ref().map_or(0.0, Feedback::held_axis),
                            throttle: 0.0,
                            brake: match action {
                                stopping::Action::Brake(brake) => brake,
                                _ => 0.55,
                            },
                        },
                        observed_gate.map_or(0.0, |g| g.aeb_brake),
                        snapshot.ego.speed,
                    );
                    ensure!(
                        etsads_safety::fresh(qpc(), snapshot.raw.header.qpc, qpc_frequency(), 150),
                        "规划耗时超过遥测新鲜度，已释放"
                    );
                    ensure!(
                        crate::loaded_controls::coherent_read(|| sink.gate.latest()).is_some_and(
                            |g| g.enabled == GATE_ACTIVE
                                && etsads_safety::fresh(
                                    qpc(),
                                    g.heartbeat_qpc,
                                    qpc_frequency(),
                                    150
                                )
                        ),
                        "控制授权已撤销"
                    );
                    if action == stopping::Action::RequestParking {
                        match sink.park(&snapshot.raw, &actions, output.steer) {
                            Ok(receipt) => {
                                parking_receipt = Some(receipt);
                                trace.write(&serde_json::json!({"event":"parking_requested_once","frame":status.frame_id,"qpc":status.source_qpc}))?;
                            }
                            Err(error) => {
                                trace.write(&serde_json::json!({"event":"minimal_risk_brake_hold","frame":status.frame_id,"reason":format!("{error:#}")}))?;
                                brake_held = true;
                                sink.publish(status.frame_id, output)?;
                            }
                        }
                    } else {
                        sink.publish(status.frame_id, output)?;
                    }
                    status.requested = Some(output);
                    status.phase = "安全停车".into();
                    status.mode = Mode::Stopping;
                    status.reason = if brake_held {
                        format!("{reason}；已停稳并保持制动，请接管")
                    } else {
                        format!("{reason}；正在减速停车，可随时接管")
                    };
                    status.target_kmh = 0.0;
                    report(
                        &mut status,
                        &progress,
                        &mut trace,
                        &overlay,
                        hud.as_ref(),
                        &released,
                        &mut hud_checked,
                        &mut last_report,
                        true,
                    )?;
                    continue;
                }
                if active.is_none()
                    && guidance.is_none()
                    && last_prepare.elapsed() > Duration::from_secs(1)
                {
                    last_prepare = Instant::now();
                    // File I/O is strictly outside active control. Every activation
                    // re-observes a fresh frame after preparation completes.
                    sink.publish(status.frame_id, neutral())?;
                    sink.request(false)?;
                    match navigator
                        .prepare_from(
                            pose,
                            snapshot.ego.z,
                            snapshot.ego.speed,
                            &snapshot.scene.route,
                            route_position(&snapshot),
                            None,
                        )
                        .and_then(|g| route_matches_game(&snapshot, g.checkpoint()).map(|()| g))
                    {
                        Ok(value) => {
                            guidance = Some(value);
                            stable_since = None;
                        }
                        Err(error) => {
                            guidance = None;
                            status.phase = "等待导航道路".into();
                            status.mode = Mode::Unavailable;
                            status.reason = format!("{error:#}");
                        }
                    }
                    report(
                        &mut status,
                        &progress,
                        &mut trace,
                        &overlay,
                        hud.as_ref(),
                        &released,
                        &mut hud_checked,
                        &mut last_report,
                        false,
                    )?;
                    std::thread::sleep(Duration::from_millis(50));
                    continue;
                }
                let Some(route) = guidance.as_mut() else {
                    sink.publish(status.frame_id, neutral())?;
                    std::thread::sleep(Duration::from_millis(20));
                    continue;
                };
                let road = match route.observe(
                    pose,
                    snapshot.ego.z,
                    snapshot.ego.speed,
                    &snapshot.scene.route,
                ) {
                    Ok(r) => r,
                    Err(error) => {
                        if active.is_some() {
                            minimal_risk_stop!(error);
                        }
                        guidance = None;
                        stable_since = None;
                        sink.publish(status.frame_id, neutral())?;
                        sink.request(false)?;
                        status.phase = "等待导航道路".into();
                        status.mode = Mode::Unavailable;
                        status.reason = format!("{error:#}");
                        report(
                            &mut status,
                            &progress,
                            &mut trace,
                            &overlay,
                            hud.as_ref(),
                            &released,
                            &mut hud_checked,
                            &mut last_report,
                            false,
                        )?;
                        continue;
                    }
                };
                // Rolling horizon: while engaged, keep a longer route prepared in
                // the background and swap it in where it coincides with this one.
                if active.is_some() {
                    if let Some(result) = preparer.poll() {
                        match result.and_then(|next| {
                            route.adopt(next, pose, snapshot.ego.z, snapshot.ego.speed)
                        }) {
                            Ok(Some(extension)) => {
                                trace.write(&serde_json::json!({"event":"route_extended","frame":status.frame_id,"extension_m":extension,"terminal":route.terminal}))?;
                            }
                            Ok(None) => {}
                            Err(error) => {
                                let message = format!("{error:#}");
                                if message != last_extension_error {
                                    trace.write(&serde_json::json!({"event":"route_extension_unavailable","frame":status.frame_id,"reason":message}))?;
                                    last_extension_error = message;
                                }
                            }
                        }
                    }
                    if road.remaining_m - 18.0 < 800.0 {
                        preparer.request(navigation::PrepareRequest {
                            pose,
                            z: snapshot.ego.z,
                            speed: snapshot.ego.speed,
                            gps: snapshot.scene.route.clone(),
                            from: route.passed_checkpoint(road.station_m),
                            lane: Some(route.lane(road.station_m).to_owned()),
                        });
                    }
                }
                status.lane = route.lane(road.station_m).into();
                status.lateral_m = road.lateral_m;
                route.observe_signals(signal_observations(&snapshot));
                let attached = trailers.read(&snapshot.raw);
                if attached != combination {
                    combination = attached;
                    trace.write(&serde_json::json!({"event":"combination","frame":status.frame_id,"trailers":attached.trailers,"behind_hitch_m":attached.behind_hitch_m,"first_wheelbase_m":attached.first_wheelbase_m}))?;
                }
                route.set_combination(combination);
                // Relative steering turns the wheel more slowly: curvature
                // changes are taken correspondingly slower.
                route.set_relative_steering(if active.is_some() {
                    sink.steering.max_rate()
                } else {
                    match profile_steering.as_ref().map(|p| p.mode) {
                        Ok(crate::setup::SteeringMode::Relative { rate_per_s }) => Some(rate_per_s),
                        _ => None,
                    }
                });
                let speed = match route.speed(
                    &road,
                    pose,
                    snapshot.ego.speed,
                    snapshot.scene.speed_limit,
                    options.maximum_kmh / 3.6,
                    &agents,
                    snapshot.raw.paused_simulation_time_us as f64 / 1e6,
                ) {
                    Ok(speed) => speed,
                    Err(error) if active.is_none() => {
                        stable_since = None;
                        sink.publish(status.frame_id, neutral())?;
                        status.phase = "等待上路".into();
                        status.mode = Mode::Unavailable;
                        status.reason = format!("{error:#}");
                        report(
                            &mut status,
                            &progress,
                            &mut trace,
                            &overlay,
                            hud.as_ref(),
                            &released,
                            &mut hud_checked,
                            &mut last_report,
                            false,
                        )?;
                        continue;
                    }
                    Err(error) => minimal_risk_stop!(error),
                };
                status.target_kmh = speed.target * 3.6;
                status.binding = Some(speed.binding);
                status.lead_gap_m = speed.lead_gap;
                status.lead = speed.lead;
                status.reason = speed.reason;
                status.terminal = route.terminal.clone();
                status.remaining_route_m = Some((road.remaining_m - 18.0).max(0.0));
                if active.is_none() {
                    sink.publish(status.frame_id, neutral())?;
                    // The first unmet condition, phrased as what the driver can do.
                    if profile_checked.elapsed() > Duration::from_secs(3) {
                        profile_steering = crate::setup::active_steering();
                        profile_checked = Instant::now();
                    }
                    let blocker = if snapshot.ego.speed < -0.02 {
                        Some("车辆正在后溜，请先停稳".to_owned())
                    } else if let Err(error) = &profile_steering {
                        Some(format!("{error:#}"))
                    } else if fatigue_seen && snapshot.raw.rest_stop_min <= 0 {
                        // While the fatigued driver dozes the game ignores every
                        // input, steering and brakes included (r11 drifts).
                        Some(
                            "司机需要休息：疲劳时游戏会让司机打盹，期间方向和刹车都不起作用。请先到停车区睡觉"
                                .to_owned(),
                        )
                    } else if snapshot.ego.speed > options.maximum_kmh / 3.6 + 1.0 {
                        Some(format!(
                            "车速高于所选上限 {:.0} km/h，请先减速",
                            options.maximum_kmh
                        ))
                    } else if road.lateral_m.abs() >= 0.55 {
                        Some(format!(
                            "请回到车道中央（偏{} {:.1} m）",
                            if road.lateral_m > 0.0 { "左" } else { "右" },
                            road.lateral_m.abs()
                        ))
                    // At a standstill a larger angle is corrected while pulling
                    // away: r13 stopped in a junction curve 0.16 rad off.
                    } else if road.heading_error_rad.abs()
                        >= if snapshot.ego.speed.abs() < 2.0 {
                            0.35
                        } else {
                            0.12
                        }
                    {
                        Some("请摆正车头，与车道方向一致".to_owned())
                    } else if speed.acceleration <= -1.0 {
                        Some("前方车辆或路段终点过近，请先手动通过".to_owned())
                    } else {
                        None
                    };
                    if blocker.is_none() {
                        stable_since.get_or_insert(Instant::now());
                    } else {
                        stable_since = None;
                    }
                    let ready =
                        stable_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500));
                    if observed_gate.is_some_and(|g| g.enabled != GATE_ACTIVE) {
                        gate_seen_off = true;
                    }
                    status.phase = if ready {
                        "可以接管"
                    } else {
                        "等待上路"
                    }
                    .into();
                    status.mode = if ready {
                        Mode::Ready
                    } else {
                        Mode::Unavailable
                    };
                    status.reason = match &blocker {
                        Some(blocker) => blocker.clone(),
                        None if !ready => "保持稳定，即将可以启用".into(),
                        None => "按 F8（或同时按下手柄两个摇杆）启用；方向键 / WASD / 空格 / 手柄 随时接管".into(),
                    };
                    if (!ready || !gate_seen_off)
                        && observed_gate.is_some_and(|g| g.enabled != GATE_OFF)
                    {
                        sink.request(false)?;
                    }
                    if ready && options.arm && !auto_arm_sent && gate_seen_off {
                        sink.request(true)?;
                        auto_arm_sent = true;
                    }
                    if ready
                        && gate_seen_off
                        && observed_gate.is_some_and(|g| g.enabled == GATE_ACTIVE)
                    {
                        controls.bind(&snapshot.raw)?;
                        let map = steering
                            .vehicle(snapshot.config.as_ref().context("车辆配置未知")?)?
                            .1
                            .clone();
                        let profile = profile_steering
                            .as_ref()
                            .map_err(|e| anyhow::anyhow!("{e:#}"))?;
                        sink.steering.set_mode(profile.mode);
                        feedback = Some(Feedback::new(axles, map).with_steering(
                            sink.steering.position(Instant::now()),
                            sink.steering.max_rate(),
                        ));
                        active = Some(Instant::now());
                        released = None;
                        last_sim = snapshot.raw.paused_simulation_time_us;
                        trace.write(&serde_json::json!({"event":"active","frame":status.frame_id,"qpc":snapshot.raw.header.qpc,"steering":profile.to_string()}))?;
                    }
                } else if let Some(active_start) = active {
                    ensure!(
                        observed_gate.is_some_and(|g| g.enabled == GATE_ACTIVE),
                        "已交还人工驾驶（人工接管、热键退出或安全监督释放）"
                    );
                    let dt = snapshot
                        .raw
                        .paused_simulation_time_us
                        .checked_sub(last_sim)
                        .context("游戏时钟倒退")? as f64
                        / 1e6;
                    last_sim = snapshot.raw.paused_simulation_time_us;
                    // A gap the telemetry check tolerates (a frozen game, a slow
                    // planning step) is stepped as the longest valid step: r13
                    // stopped at 28 km/h on the controller's 0.1 s limit.
                    if dt > 0.1 && dt <= TELEMETRY_GAP.as_secs_f64() + 0.1 {
                        trace.write(&serde_json::json!({"event":"control_step_clamped","frame":status.frame_id,"dt":dt}))?;
                    }
                    let dt = if dt <= TELEMETRY_GAP.as_secs_f64() + 0.1 {
                        dt.min(0.1)
                    } else {
                        dt
                    };
                    if road.remaining_m < 21.0
                        && snapshot.ego.speed.abs() < 0.15
                        && ending.is_none()
                    {
                        ending = Some(stopping::StopSequence::new(started.elapsed().as_secs_f64()));
                        ending_reason = if route.terminal.starts_with("即将到达目的地") {
                            "已到达目的地"
                        } else {
                            "已到可用导航路段末端"
                        };
                    }
                    let ending_action = if let Some(sequence) = &mut ending {
                        let acknowledged = if let Some(receipt) = &mut parking_receipt {
                            receipt.confirmed(&actions, &snapshot.raw)?
                        } else {
                            false
                        };
                        Some(sequence.update(
                            started.elapsed().as_secs_f64(),
                            snapshot.ego.speed,
                            status.parking_brake,
                            acknowledged,
                        )?)
                    } else {
                        None
                    };
                    if ending_action == Some(stopping::Action::Complete) {
                        parking_confirmed = true;
                        trace.write(&serde_json::json!({"event":"parking_confirmed","frame":status.frame_id,"qpc":status.source_qpc,"sdk_evidence":parking_receipt.as_ref().and_then(|p|p.evidence())}))?;
                        return Ok(format!("{ending_reason}；已停稳并确认驻车"));
                    }
                    if let Some(f) = feedback.as_mut() {
                        f.coast = speed.approach;
                    }
                    // A drift is corrected at reduced speed rather than handed
                    // back; only beyond the locked tolerance does the truck stop.
                    let drifting = road.lateral_m.abs() >= DRIFT_WARN_M;
                    let (target, planned) = if drifting {
                        (
                            speed.target.min(DRIFT_SPEED),
                            if snapshot.ego.speed > DRIFT_SPEED {
                                speed.acceleration.min(-1.0)
                            } else {
                                speed.acceleration
                            },
                        )
                    } else {
                        (speed.target, speed.acceleration)
                    };
                    let hold = match (&feedback, sink.steering.max_rate()) {
                        (Some(f), Some(_)) => {
                            relative_hold.cap(f.steering_lag(), road.lateral_m, snapshot.ego.speed)
                        }
                        _ => None,
                    };
                    let (target, planned) = match hold {
                        Some(held) => (target.min(held), planned.min(0.0)),
                        None => (target, planned),
                    };
                    let (target, planned) = if stall_rolling {
                        status.reason = "发动机熄火，靠车道减速停车后自动重新启动".into();
                        (
                            (snapshot.ego.speed - 1.0).max(0.0),
                            planned.min(-STALL_DECELERATION),
                        )
                    } else {
                        (target, planned)
                    };
                    // With a trailer the tractor keeps to the outside of a
                    // tight bend, within its lane, so the trailer cuts in less.
                    let steered = match combination.trailers {
                        0 => road.clone(),
                        _ => {
                            let radius = 1.0 / road.curvature.abs().max(1e-6);
                            let bias = (TRAILER_OFFTRACK_SHARE
                                * trailer::offtracking(combination.first_wheelbase_m, radius))
                            .min(TRAILER_OUTSIDE_MAX_M);
                            road.offset(pose, -road.curvature.signum() * bias)
                        }
                    };
                    let step = feedback.as_mut().context("控制器尚未绑定车辆")?.step(
                        &steered,
                        pose,
                        snapshot.ego.speed,
                        if ending.is_some() { 0.0 } else { target },
                        planned,
                        dt,
                    );
                    let mut output = match step {
                        Ok(output) => output,
                        Err(error) => minimal_risk_stop!(error),
                    };
                    // Throttle without motion: wedged against an obstacle the
                    // traffic data does not show (r11: a tree beside a yard).
                    if output.throttle > 0.2 && snapshot.ego.speed.abs() < 0.3 {
                        if pushing_since.get_or_insert_with(Instant::now).elapsed()
                            > Duration::from_secs(8)
                        {
                            minimal_risk_stop!(anyhow::anyhow!("车辆被卡住无法前进，请手动处理"));
                        }
                    } else {
                        pushing_since = None;
                    }
                    status.longitudinal = feedback.as_ref().map(|f| f.longitudinal);
                    if let Some(action) = ending_action {
                        output.throttle = 0.0;
                        output.brake = match action {
                            stopping::Action::Brake(brake) => brake,
                            _ => 0.55,
                        };
                    }
                    let aeb = observed_gate.map_or(0.0, |g| g.aeb_brake);
                    output = respect_safety(output, aeb, snapshot.ego.speed);
                    ensure!(!stop.load(Ordering::Acquire), "已请求停止");
                    ensure!(
                        etsads_safety::fresh(qpc(), snapshot.raw.header.qpc, qpc_frequency(), 150),
                        "规划耗时超过遥测新鲜度，已释放"
                    );
                    // Recheck after planning so a concurrently revoked Gate cannot
                    // authorize another nonzero publication from a cached snapshot.
                    ensure!(
                        crate::loaded_controls::coherent_read(|| sink.gate.latest()).is_some_and(
                            |g| g.enabled == GATE_ACTIVE
                                && etsads_safety::fresh(
                                    qpc(),
                                    g.heartbeat_qpc,
                                    qpc_frequency(),
                                    150
                                )
                        ),
                        "控制授权已撤销"
                    );
                    if ending_action == Some(stopping::Action::RequestParking) {
                        parking_receipt = Some(sink.park(&snapshot.raw, &actions, output.steer)?);
                        output.throttle = 0.0;
                        output.brake = 0.55;
                        trace.write(&serde_json::json!({"event":"parking_requested_once","frame":status.frame_id,"qpc":status.source_qpc}))?;
                    } else {
                        if let Some(ring) = &planned_path {
                            let points = route.path_ahead(road.station_m);
                            let mut message = PlannedPath {
                                write_qpc: qpc(),
                                source_frame_id: status.frame_id,
                                count: points.len() as u32,
                                ..Default::default()
                            };
                            message.points[..points.len()].copy_from_slice(&points);
                            let _ = ring.publish(&message);
                        }
                        // Signal lane changes; switch off only what we switched on.
                        let want = route.blinker(road.station_m, snapshot.ego.speed);
                        let left_on = snapshot.raw.lights & (1 << 6) != 0;
                        let right_on = snapshot.raw.lights & (1 << 7) != 0;
                        if blinker_pressed.elapsed() > Duration::from_millis(1200) {
                            let press = match want {
                                Some(true) if right_on && blinker_ours == Some(false) => Some(1),
                                Some(true) if !left_on => Some(0),
                                Some(false) if left_on && blinker_ours == Some(true) => Some(0),
                                Some(false) if !right_on => Some(1),
                                None if left_on && blinker_ours == Some(true) => Some(0),
                                None if right_on && blinker_ours == Some(false) => Some(1),
                                _ => None,
                            };
                            if let Some(pulse) = press {
                                sink.press(pulse);
                                blinker_pressed = Instant::now();
                            }
                            // Ownership lasts until telemetry shows the switch off.
                            blinker_ours = match want {
                                Some(left) => Some(left),
                                None if press.is_some() => blinker_ours,
                                None => None,
                            };
                        }
                        // Low beam on while engaged: some countries require
                        // it at all times, all at night (L7–L11 drew seven
                        // no_lights fines). The light switch cycles off ->
                        // parking -> low beam; one step per 1.2 s so telemetry
                        // catches up before the next press.
                        let low_beam = snapshot.raw.lights & (1 << 1) != 0;
                        if !low_beam
                            && blinker_pressed.elapsed() > Duration::from_millis(1200)
                            && lights_pressed.elapsed() > Duration::from_millis(1200)
                        {
                            sink.press(LIGHT_PULSE);
                            lights_pressed = Instant::now();
                        }
                        sink.publish(status.frame_id, output)?;
                    }
                    status.requested = Some(output);
                    frames += 1;
                    status.remaining_s = if options.seconds == 0 {
                        f64::INFINITY
                    } else {
                        (options.seconds as f64 - active_start.elapsed().as_secs_f64()).max(0.0)
                    };
                    // Warn early enough to take over before the planned stop: at
                    // least 8 s of travel or 150 m before the end of the prefix.
                    let to_stop = status.remaining_route_m.unwrap_or(f64::INFINITY);
                    let route_ending = to_stop < (snapshot.ego.speed * 8.0).max(150.0);
                    (status.phase, status.mode) = if status.remaining_s <= 5.0 {
                        ("即将结束，请接管".into(), Mode::TakeoverSoon)
                    } else if route_ending {
                        status.reason = format!("{} · {:.0} m", route.terminal, to_stop);
                        ("前方需人工接管".into(), Mode::TakeoverSoon)
                    } else if drifting {
                        status.target_kmh = target * 3.6;
                        status.reason = format!(
                            "偏{} {:.1} m，正在修正并减速",
                            if road.lateral_m > 0.0 { "左" } else { "右" },
                            road.lateral_m.abs()
                        );
                        ("正在修正偏离".into(), Mode::TakeoverSoon)
                    } else {
                        ("辅助驾驶中".into(), Mode::Engaged)
                    };
                    // Held at a standstill behind a vehicle that does not
                    // move (no junction rule involved): it may be parked.
                    // Keep holding, but ask the driver after 45 s (r11: a
                    // parked van blocked a yard exit for ten minutes).
                    let stuck = snapshot.ego.speed.abs() < 0.3
                        && speed
                            .lead
                            .is_some_and(|l| l.speed_mps < 0.5 && l.gap_m < 30.0)
                        && status.reason.starts_with("前车");
                    if !stuck {
                        held_behind = None;
                    } else if held_behind.get_or_insert_with(Instant::now).elapsed()
                        > Duration::from_secs(45)
                    {
                        let waited = held_behind.map_or(0, |t| t.elapsed().as_secs());
                        status.reason =
                            format!("前方车辆已停 {waited} 秒未动；若是停放车辆，请手动绕行");
                        (status.phase, status.mode) = ("前方受阻".into(), Mode::TakeoverSoon);
                    }
                    // An absolute-steering profile follows the input within
                    // about 0.15 s. A wheel that keeps turning away from the
                    // input is the game applying it as a turning rate (the
                    // profile was switched to relative steering): from then on
                    // the position is held by the relative servo while stopping.
                    if sink.steering.mode() == crate::setup::SteeringMode::Absolute
                        && let (Some(axes), Some(position)) =
                            (status.applied_axes, sink.steering.position(Instant::now()))
                    {
                        if (position - axes[0] as f64).abs() > 0.15 {
                            let (since, from) =
                                *steering_mismatch.get_or_insert((Instant::now(), position));
                            if since.elapsed() > Duration::from_millis(600)
                                && (position - from).abs() > 0.05
                            {
                                steering_mismatch = None;
                                sink.steering
                                    .set_mode(crate::setup::SteeringMode::Relative {
                                        rate_per_s: 0.4,
                                    });
                                minimal_risk_stop!(anyhow::anyhow!(
                                    "游戏转向方式与档案设置不符（档案可能改成了相对转向），已停车；请重新开启自动驾驶"
                                ));
                            }
                        } else {
                            steering_mismatch = None;
                        }
                    }
                    // The game ignores inputs while a fatigued driver dozes:
                    // the measured wheel stops following a changing command
                    // (the steering position; with relative steering the input
                    // itself is a turning rate).
                    let axis = match sink.steering.mode() {
                        crate::setup::SteeringMode::Absolute => {
                            status.applied_axes.map_or(0.0, |a| a[0] as f64)
                        }
                        crate::setup::SteeringMode::Relative { .. } => {
                            status.requested.map_or(0.0, |r| r.steer)
                        }
                    };
                    let wheel = status.wheel_angle_rad;
                    match input_check {
                        Some((since, axis0, wheel0))
                            if (wheel - wheel0).abs() < 0.005
                                && since.elapsed() < Duration::from_secs(4) =>
                        {
                            input_frozen = snapshot.ego.speed > 1.0
                                && (axis - axis0).abs() > 0.1
                                && since.elapsed() > Duration::from_millis(800);
                        }
                        _ => {
                            input_check = Some((Instant::now(), axis, wheel));
                            input_frozen = false;
                        }
                    }
                    let rest = snapshot.raw.rest_stop_min;
                    // The SDK does not say whether fatigue simulation is on;
                    // an observed freeze proves it (and that the driver dozes).
                    fatigue_seen |= input_frozen;
                    if input_frozen {
                        status.reason =
                            "游戏没有响应控制（司机疲劳打盹中），恢复后自动修正；请尽快停车休息"
                                .into();
                        (status.phase, status.mode) = ("控制无响应".into(), Mode::TakeoverSoon);
                    } else if fatigue_seen && rest <= 0 && status.mode == Mode::Engaged {
                        status.reason =
                            "司机已疲劳，随时可能打盹（打盹时车辆不受控制），请尽快停车休息".into();
                        (status.phase, status.mode) = ("需要休息".into(), Mode::TakeoverSoon);
                    } else if fatigue_seen && rest < 120 && status.mode == Mode::Engaged {
                        status.reason =
                            format!("{} · 约 {rest} 分钟（游戏时间）后需要休息", status.reason);
                    }
                    // A long haul can outlast the tank; the route does not
                    // include fuel stops, so the driver is told in good time.
                    let fuel = snapshot.raw.fuel_l;
                    if let Some(limit) = options.fuel_stop_l
                        && fuel < limit
                        && ending.is_none()
                        && active.is_some()
                    {
                        minimal_risk_stop!(anyhow::anyhow!("燃油不足（剩 {fuel:.0} L），停车加油"));
                    }
                    if fuel < LOW_FUEL_TAKEOVER_L && status.mode == Mode::Engaged {
                        status.reason = format!("燃油即将耗尽（剩 {fuel:.0} L），请接管去加油");
                        (status.phase, status.mode) = ("燃油不足".into(), Mode::TakeoverSoon);
                    } else if fuel < LOW_FUEL_WARN_L && status.mode == Mode::Engaged {
                        status.reason =
                            format!("{} · 燃油剩 {fuel:.0} L，请安排加油", status.reason);
                    }
                    if aeb > 0.0 {
                        status.phase = "安全制动中".into();
                        status.mode = Mode::SafetyBrake;
                        status.reason = "独立监督检测到接近风险，已停止加速".into();
                        status.target_kmh = 0.0;
                    }
                    if ending.is_some() {
                        status.phase = "停车收尾".into();
                        status.mode = Mode::Stopping;
                        status.reason = "保持制动，停稳后确认驻车；人工接管仍立即退出".into();
                        status.target_kmh = 0.0;
                    }
                }
                report(
                    &mut status,
                    &progress,
                    &mut trace,
                    &overlay,
                    hud.as_ref(),
                    &released,
                    &mut hud_checked,
                    &mut last_report,
                    active.is_some(),
                )?;
            }
        })();
        // Release first, before any bookkeeping or diagnostic write.
        let _ = sink.publish(status.frame_id, neutral());
        let _ = sink.request(false);
        let reason = match &outcome {
            Ok(reason) => reason.clone(),
            Err(error) => format!("{error:#}"),
        };
        let was_active = active.is_some();
        if was_active {
            status.engagements += 1;
            let _ = trace.write(&serde_json::json!({"event":"released","frame":status.frame_id,"qpc":status.source_qpc,"reason":reason,"parking_confirmed":parking_confirmed,"engagement":status.engagements}));
            released = Some((Instant::now(), reason.clone()));
        }
        if !(options.continuous && was_active && !stop.load(Ordering::Acquire)) {
            break outcome;
        }
        // Back to standby. Nothing carries authority over: the Gate must be seen
        // off and the driver must press the hotkey again.
        guidance = None;
        active = None;
        stable_since = None;
        feedback = None;
        ending = None;
        degraded = None;
        brake_held = false;
        pushing_since = None;
        parking_receipt = None;
        parking_confirmed = false;
        ending_reason = "本次接管时长结束";
        gate_seen_off = false;
        started = Instant::now();
        status.lead = None;
        status.longitudinal = None;
        status.remaining_route_m = None;
    };
    // Release before any potentially slow final diagnostic write.
    drop(sink);
    let reason = match result {
        Ok(reason) => reason,
        Err(error) => format!("{error:#}"),
    };
    status.phase = "已释放控制".into();
    status.requested = Some(neutral());
    status.applied_axes = None;
    status.applied_seq = None;
    status.gate = None;
    status.reason = reason.clone();
    status.produced_at = Instant::now();
    if let Some(tx) = progress {
        let _ = tx.try_send(status.clone());
    }
    trace.write(&status)?;
    trace.flush()?;
    Ok(Report {
        source: "game_experimental_assist",
        stop_reason: reason,
        active_frames: frames,
        log: trace.path.clone(),
        full_acceptance: false,
        parking_confirmed,
    })
}
#[allow(clippy::too_many_arguments)]
fn report(
    status: &mut Progress,
    tx: &Option<SyncSender<Progress>>,
    trace: &mut io::Trace,
    overlay: &io::Overlay,
    hud: Option<&crate::hud::HudPublisher>,
    released: &Option<(Instant, String)>,
    hud_checked: &mut Instant,
    last: &mut Instant,
    active: bool,
) -> anyhow::Result<()> {
    if !active
        && let Some((at, reason)) = released
            .as_ref()
            .filter(|(at, _)| at.elapsed() < RELEASED_HOLD)
    {
        status.mode = Mode::Released;
        status.phase = "已交还人工驾驶".into();
        status.reason = format!(
            "{reason} · {:.0} 秒后回到待命",
            (RELEASED_HOLD - at.elapsed()).as_secs_f64().ceil()
        );
    }
    if hud_checked.elapsed() >= Duration::from_secs(1) {
        status.hud_visible = hud.is_some() && crate::hud::HudPublisher::visible();
        *hud_checked = Instant::now();
    }
    if let Some(hud) = hud {
        hud.show(hud_model(status));
    }
    // The rectangle label is only a fallback for a renderer without the HUD.
    let label = if status.hud_visible {
        String::new()
    } else {
        match status.mode {
            Mode::Stopping => "STOP".into(),
            Mode::SafetyBrake => "BRAKE".into(),
            _ if active && status.remaining_s.is_finite() => {
                format!("ASSIST {}", status.remaining_s.ceil())
            }
            _ if active => "ASSIST".into(),
            Mode::Ready => "READY".into(),
            _ => "WAIT".into(),
        }
    };
    overlay.publish(
        status.frame_id,
        &label,
        !status.hud_visible && active && matches!(status.mode, Mode::Engaged | Mode::TakeoverSoon),
    )?;
    if last.elapsed() >= Duration::from_millis(100) {
        status.produced_at = Instant::now();
        // Standby is logged at 1 Hz; engaged driving at 10 Hz, steady
        // cruise near the limit at 2 Hz so a long haul fits the log budget.
        let steady = status.mode == Mode::Engaged
            && status.binding.is_some_and(|b| {
                matches!(b, navigation::Binding::Limit | navigation::Binding::Ceiling)
            })
            && (status.speed_kmh - status.target_kmh).abs() < 3.0
            && status.lateral_m.abs() < 0.3
            && status.requested.is_some_and(|r| r.brake == 0.0);
        let period = if !active {
            Duration::from_secs(1)
        } else if steady {
            Duration::from_millis(500)
        } else {
            Duration::ZERO
        };
        if trace.since_last() >= period {
            trace.write(status)?;
        }
        if let Some(tx) = tx {
            let _ = tx.try_send(status.clone());
        }
        *last = Instant::now();
    }
    Ok(())
}
const RELEASED_HOLD: Duration = Duration::from_secs(5);
/// The SDK `light` input: the headlight switch, one step per press.
const LIGHT_PULSE: usize = 3;
/// Longest telemetry gap held through while engaged.
const TELEMETRY_GAP: Duration = Duration::from_millis(1000);
/// A low engine speed lasting this long while engaged is a stall.
const STALL_CONFIRM: Duration = Duration::from_millis(500);
/// Braking after a stall while moving (m/s²).
const STALL_DECELERATION: f64 = 1.5;
/// SDK pulses for restarting a stalled engine (indices in `PULSE_NAMES`).
const GEAR_NEUTRAL_PULSE: usize = 21;
const GEAR_DRIVE_PULSE: usize = 22;
const PARKING_PULSE: usize = 17;
/// Brake holding the truck while its engine is restarted.
const RESTART_BRAKE: f64 = 0.4;
/// Throttle that cranks the engine through the game's automatic start.
const CRANK_THROTTLE: f64 = 0.35;
/// A restart not finished within this time hands over.
const RESTART_LIMIT: Duration = Duration::from_secs(45);
/// Crank attempts per restart.
const RESTART_CRANKS: u8 = 6;
/// Restarting a stalled engine at a standstill. The game's engine switch is a
/// button that must be held about 1.5 s; the SDK `engine` pulse lasts one
/// frame and the engine caught and died again (r13: eight in a row). With
/// automatic engine start the throttle cranks it for as long as it is held,
/// but the bridge drops throttle under any service brake, so the truck is
/// held by its parking brake: neutral, parking brake, throttle until the
/// engine keeps running, parking brake off, drive.
struct Restart {
    since: Instant,
    stage: u8,
    at: Instant,
    /// When the engine was last seen to start running.
    running_since: Option<Instant>,
    cranks: u8,
    done: bool,
}
/// What the restart asks for this frame.
#[derive(Debug, PartialEq)]
struct RestartStep {
    pulse: Option<usize>,
    throttle: f64,
    brake: f64,
}
impl Restart {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            since: now,
            stage: 0,
            at: now,
            running_since: None,
            cranks: 0,
            done: false,
        }
    }
    fn step(&mut self, running: bool, parked: bool, in_gear: bool) -> RestartStep {
        let waited = self.at.elapsed();
        if !running {
            self.running_since = None;
        } else if self.running_since.is_none() {
            self.running_since = Some(Instant::now());
        }
        let steady = self
            .running_since
            .is_some_and(|t| t.elapsed() >= Duration::from_millis(1500));
        let hold = |pulse| RestartStep {
            pulse,
            throttle: 0.0,
            brake: RESTART_BRAKE,
        };
        let (stage, out) = match self.stage {
            // Neutral; then the parking brake, or straight on if it is set.
            0 => (Some(1), hold(Some(GEAR_NEUTRAL_PULSE))),
            1 if waited >= Duration::from_millis(400) => {
                (Some(2), hold((!parked).then_some(PARKING_PULSE)))
            }
            // Crank while parked (no service brake, or the bridge drops the
            // throttle); pause between attempts.
            2 if parked && waited >= Duration::from_millis(300) => {
                if steady {
                    (Some(4), hold(None))
                } else if !running && waited >= Duration::from_millis(4000) {
                    self.cranks += 1;
                    (Some(3), hold(None))
                } else {
                    let crank = RestartStep {
                        pulse: None,
                        throttle: CRANK_THROTTLE,
                        brake: 0.0,
                    };
                    (None, crank)
                }
            }
            3 if waited >= Duration::from_millis(1500) && self.cranks < RESTART_CRANKS => {
                (Some(2), hold(None))
            }
            // Running: parking brake off, then drive.
            4 if steady => (Some(5), hold(parked.then_some(PARKING_PULSE))),
            5 if !parked && waited >= Duration::from_millis(300) => {
                (Some(6), hold(Some(GEAR_DRIVE_PULSE)))
            }
            6 if running && in_gear && waited >= Duration::from_millis(500) => {
                self.done = true;
                (None, hold(None))
            }
            _ => (None, hold(None)),
        };
        if let Some(stage) = stage {
            self.stage = stage;
            self.at = Instant::now();
        }
        out
    }
}
/// Re-establishing input authority after a game pause, each stage with the
/// instant the handshake began.
#[derive(Clone, Copy)]
enum Rearm {
    OffRequested(Instant),
    /// Began, and when the gate was first seen Off.
    OffSeen(Instant, Instant),
    OnRequested(Instant),
}
impl Rearm {
    fn since(self) -> Instant {
        match self {
            Self::OffRequested(t) | Self::OffSeen(t, _) | Self::OnRequested(t) => t,
        }
    }
}
/// Fuel left (L) at which the driver is asked to plan, then to take over.
const LOW_FUEL_WARN_L: f32 = 80.0;
const LOW_FUEL_TAKEOVER_L: f32 = 25.0;
/// Speed cap (m/s) while correcting a drift from the lane centre.
const DRIFT_SPEED: f64 = 15.0 / 3.6;
/// With a trailer: the share of its off-tracking the tractor moves outward
/// in a bend, at most staying within a 3.5 m lane.
const TRAILER_OFFTRACK_SHARE: f64 = 0.4;
const TRAILER_OUTSIDE_MAX_M: f64 = 0.5;

/// The in-game panel for the current state. Wording is for the driver: what
/// is happening and what to press, never internal diagnostics.
pub fn hud_model(status: &Progress) -> crate::hud::HudModel {
    use crate::hud::{HudModel, Tone};
    let fresh = status.frame_id != 0;
    let limit = (status.limit_kmh > 0.5).then_some(status.limit_kmh);
    let scale = limit
        .unwrap_or(status.maximum_kmh)
        .max(status.maximum_kmh.min(limit.unwrap_or(f64::INFINITY)))
        .max(status.speed_kmh)
        .max(30.0)
        * 1.1;
    let mut model = HudModel {
        speed_kmh: fresh.then_some(status.speed_kmh.max(0.0)),
        limit_kmh: limit,
        scale_kmh: scale,
        ..Default::default()
    };
    let lead = status.lead.map(|l| {
        format!(
            "前车 {:.0} m · {}",
            l.gap_m,
            match l.motion {
                Motion::SameDirection => format!("同向 {:.0} km/h", l.speed_mps * 3.6),
                Motion::Opposing => "对向来车".into(),
                Motion::Crossing => "横穿车辆".into(),
            }
        )
    });
    let route = status
        .remaining_route_m
        .filter(|m| m.is_finite() && *m < 5000.0)
        .map(|m| {
            if m >= 1000.0 {
                format!("导航路段剩余 {:.1} km", m / 1000.0)
            } else {
                format!("导航路段剩余 {m:.0} m")
            }
        });
    match status.mode {
        Mode::Starting => {
            model.title = "自动驾驶 · 启动中".into();
            model.detail.push(status.reason.clone());
        }
        Mode::Unavailable => {
            model.title = "待命 · 暂不可用".into();
            model.detail.push(status.reason.clone());
            model.hint = "条件满足后按 F8 启用".into();
        }
        Mode::Ready => {
            model.tone = Tone::Ready;
            model.title = "可以启用自动驾驶".into();
            model.target_kmh = Some(status.target_kmh);
            model.detail.extend(route);
            model.hint = "按 F8 启用 · 先松开方向和油门键".into();
        }
        Mode::Engaged | Mode::TakeoverSoon => {
            model.tone = if status.mode == Mode::Engaged {
                Tone::Active
            } else {
                Tone::Warning
            };
            model.blink = status.mode == Mode::TakeoverSoon;
            model.title = if status.mode == Mode::Engaged {
                "自动驾驶中".into()
            } else {
                "请准备接管".into()
            };
            model.badge = if status.remaining_s.is_finite() {
                format!("{:.0} s", status.remaining_s.ceil())
            } else {
                "不限时".into()
            };
            model.target_kmh = Some(status.target_kmh);
            // Junction and route reasons are shown; routine cruising text is not.
            if status.mode == Mode::TakeoverSoon
                || !(status.reason.starts_with("按 GPS") || status.reason.starts_with("前车"))
            {
                model.detail.push(status.reason.clone());
            }
            model.detail.extend(lead);
            if status.mode == Mode::Engaged {
                model.detail.extend(route);
            }
            model.hint =
                "F8 或同时按下两个摇杆退出 · 方向键 / WASD / 空格 / 手柄摇杆或扳机 立即接管".into();
        }
        Mode::SafetyBrake => {
            model.tone = Tone::Alert;
            model.title = "安全制动".into();
            model.target_kmh = Some(0.0);
            model
                .detail
                .push("独立监督检测到接近风险，已停止加速".into());
            model.detail.extend(lead);
            model.hint = "方向键 / WASD / 空格 / 手柄 随时接管".into();
        }
        Mode::Stopping => {
            model.tone = Tone::Warning;
            model.title = "正在停车".into();
            model.target_kmh = Some(0.0);
            model.detail.push("停稳后拉起驻车制动，然后交还人工".into());
            model.hint = "方向键 / WASD / 空格 / 手柄 随时接管".into();
        }
        Mode::Released => {
            model.title = "已交还人工驾驶".into();
            model.speed_kmh = None;
            model.detail.push(status.reason.clone());
        }
    }
    model
}
fn validate_snapshot(
    s: &GameSnapshot,
    now: u64,
    ending: bool,
    stall_check: bool,
) -> anyhow::Result<()> {
    ensure!(
        !s.ego.paused && etsads_safety::fresh(now, s.raw.header.qpc, qpc_frequency(), 150),
        "请进入驾驶画面，保持游戏运行"
    );
    // A stall while engaged is handled by the minimal-risk stop (the air
    // brakes still work), which must be allowed to finish.
    ensure!(
        ending || stall_check || (s.raw.rpm.is_finite() && s.raw.rpm > 300.0),
        "请启动发动机"
    );
    ensure!(
        (s.raw.parking_brake == 0 || ending) && s.raw.gear >= 0 && s.raw.displayed_gear > 0,
        "请选择前进挡并松开驻车制动"
    );
    ensure!(s.ego.speed >= -0.2, "首版不支持倒车");
    let ext = s.extension.as_deref().context("交通和导航插件未连接")?;
    ensure!(
        ext.available & 22 == 22 && ext.header.frame_id == s.raw.header.frame_id,
        "交通、停车车辆或GPS数据不可用"
    );
    ensure!(!s.scene.route.is_empty(), "请在游戏地图设置GPS目的地");
    ensure!(
        s.scene.speed_limit.is_finite() && (0.0..=45.0).contains(&s.scene.speed_limit),
        "当前路段限速数据无效"
    );
    Ok(())
}
// Small negative wheel-speed rebounds after a stop are not a reverse mission.
// They may only hold braking: the independent supervisor keeps its stricter
// reverse-throttle interlock unchanged.
/// ETS2LA's route must be the game's current one: its remaining length from
/// our checkpoint has to agree with the game's own navigation distance. r10
/// drove a stale 8 km route while the game was navigating 78 km south.
fn route_records(snapshot: &GameSnapshot) -> Vec<etsads_game::ets2la::RouteNode> {
    let Some(extension) = snapshot.extension.as_deref() else {
        return Vec::new();
    };
    let mut records: Vec<etsads_game::ets2la::RouteNode> =
        etsads_game::ets2la::records(&extension.route)
            .filter(|r: &etsads_game::ets2la::RouteNode| r.node_uid != 0)
            .collect();
    // Indexed like the deduplicated node list the route is resolved from.
    records.dedup_by_key(|r| r.node_uid);
    records
}
/// Index of the last checkpoint already passed, from the game's own
/// remaining navigation distance: ETS2LA keeps every checkpoint since the
/// route was planned (r11: 1 km on, inside a junction, nothing matched).
fn route_position(snapshot: &GameSnapshot) -> usize {
    let game = snapshot.raw.navigation_distance_m as f64;
    if !(game.is_finite() && game > 1.0) {
        return 0;
    }
    route_records(snapshot)
        .iter()
        .position(|r| (r.distance_to_end as f64) < game)
        .map_or(0, |i| i.saturating_sub(1))
}
fn route_matches_game(snapshot: &GameSnapshot, checkpoint: usize) -> anyhow::Result<()> {
    let game = snapshot.raw.navigation_distance_m as f64;
    ensure!(
        game.is_finite() && game > 1.0,
        "游戏里没有导航路线，请先设置目的地"
    );
    let records = route_records(snapshot);
    ensure!(!records.is_empty(), "路线数据缺失");
    // The truck is between the checkpoint the route resumes at and the one
    // before it; on highways these can be kilometres apart.
    let next = records
        .get(checkpoint)
        .map(|r| r.distance_to_end as f64)
        .context("路线数据缺失")?;
    let previous = records
        .get(checkpoint.saturating_sub(1))
        .map_or(next, |r| r.distance_to_end as f64);
    let tolerance = 500.0_f64.max(0.05 * game);
    ensure!(
        game >= next.min(previous) - tolerance && game <= next.max(previous) + tolerance,
        "导航数据未同步（游戏剩余 {:.1} km，路线数据 {:.1} km），请在游戏里重新设置目的地",
        game / 1000.0,
        next / 1000.0
    );
    Ok(())
}
fn respect_safety(mut output: Output, aeb: f32, speed: f64) -> Output {
    if aeb > 0.0 || speed < -0.02 {
        output.throttle = 0.0;
        output.brake = output
            .brake
            .max(aeb as f64)
            .max(if speed < -0.02 { 0.5 } else { 0.0 });
    }
    output
}
/// ETS2LA traffic-light heads (kind 1) of this physics frame, in map ENU.
fn signal_observations(snapshot: &GameSnapshot) -> Vec<etsads_map::traffic::SignalObservation> {
    use etsads_game::ets2la;
    let Some(ext) = snapshot
        .extension
        .as_deref()
        .filter(|e| e.available & 8 != 0 && e.header.frame_id == snapshot.raw.header.frame_id)
    else {
        return Vec::new();
    };
    ets2la::records::<ets2la::Signal>(&ext.signals)
        .filter(|s| s.kind == 1)
        .filter_map(|s| {
            let p = ets2la::sector_world(s.position, s.sector);
            let position = etsads_msgs::Point3 {
                x: p[0],
                y: -p[2],
                z: p[1],
            };
            (position.valid() && s.time_remaining.is_finite() && s.time_remaining >= 0.0).then_some(
                etsads_map::traffic::SignalObservation {
                    position,
                    raw_state: s.state,
                    age_us: 0,
                },
            )
        })
        .collect()
}
fn traffic(snapshot: &GameSnapshot) -> anyhow::Result<Vec<Agent>> {
    use etsads_game::ets2la::{self, Parked, Traffic};
    let ext = snapshot.extension.as_deref().context("交通数据缺失")?;
    // A single implausible record (an AI vehicle in a collision) is handled
    // by the adapter, which clamps its acceleration or leaves out a body it
    // cannot place; only a mostly corrupt table means the source is broken.
    // L2 released control at 47 km/h over one −77 m/s² record.
    let mut total = 0_usize;
    let mut invalid = Vec::new();
    for r in ets2la::records::<Traffic>(&ext.traffic).filter(|r| r.id != 0) {
        total += 1;
        let count = r.trailer_count;
        let position = r.position;
        let size = r.size;
        let trailers_valid =
            r.trailers
                .into_iter()
                .take(count.clamp(0, 3) as usize)
                .all(|trailer| {
                    let position = trailer.position;
                    let size = trailer.size;
                    ets2la::valid_quaternion(trailer.quaternion_wxyz)
                        && position.iter().chain(size.iter()).all(|v| v.is_finite())
                        && size.iter().all(|v| *v > 0.0)
                        && size[2] < 100.0
                        && size[0] < 100.0
                });
        let valid = trailers_valid
            && ets2la::valid_quaternion(r.quaternion_wxyz)
            && (0..=3).contains(&count)
            && position.iter().chain(size.iter()).all(|v| v.is_finite())
            && size.iter().all(|v| *v > 0.0)
            && r.speed.is_finite()
            && r.speed.abs() <= 100.0
            && size[2] < 50.0
            && size[0] < 20.0;
        if !valid {
            invalid.push(format!(
                "id={} q={:?} trailers={} size={:?} speed={}",
                { r.id },
                { r.quaternion_wxyz },
                count,
                size,
                { r.speed }
            ));
        }
    }
    ensure!(
        invalid.len() * 2 <= total.max(2),
        "交通对象原始数据大面积无效（{}/{}）：{}",
        invalid.len(),
        total,
        invalid.first().map_or("", |s| s.as_str())
    );
    let mut agents = snapshot.scene.agents.clone();
    for r in ets2la::records::<Parked>(&ext.parked).filter(|r| r.id != 0) {
        let yaw = ets2la::yaw_scs_quaternion(r.quaternion_wxyz).context("停车车辆姿态无效")?;
        let agent = Agent {
            id: r.id as u64,
            pose: Pose2 {
                x: r.position[0] as f64,
                y: -r.position[2] as f64,
                yaw,
            },
            z: r.position[1] as f64,
            speed: 0.0,
            acceleration: 0.0,
            length: r.size[2] as f64,
            width: r.size[0] as f64,
            lane_id: None,
            trailers: vec![],
        };
        ensure!(agent.valid(), "停车车辆几何无效");
        agents.push(agent);
    }
    Ok(agents)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn button_indices_name_the_sdk_inputs_they_press() {
        let name = |i: usize| etsads_game::input::PULSE_NAMES[i];
        assert_eq!(name(LIGHT_PULSE), "light");
        assert_eq!(name(GEAR_NEUTRAL_PULSE), "gear0");
        assert_eq!(name(GEAR_DRIVE_PULSE), "geardrive");
        assert_eq!(name(PARKING_PULSE), "parkingbrake");
    }
    #[test]
    fn restart_parks_cranks_with_the_throttle_then_selects_drive() {
        let mut r = Restart::new();
        let age = |r: &mut Restart, ms| {
            r.at -= Duration::from_millis(ms);
            if let Some(t) = r.running_since.as_mut() {
                *t -= Duration::from_millis(ms);
            }
        };
        assert_eq!(r.step(false, false, true).pulse, Some(GEAR_NEUTRAL_PULSE));
        age(&mut r, 400);
        assert_eq!(r.step(false, false, false).pulse, Some(PARKING_PULSE));
        age(&mut r, 300);
        // Parked: the throttle cranks, with no service brake.
        let crank = r.step(false, true, false);
        assert_eq!((crank.throttle, crank.brake), (CRANK_THROTTLE, 0.0));
        // No start within 4 s: pause, then crank again.
        age(&mut r, 4000);
        assert_eq!(r.step(false, true, false).throttle, 0.0);
        age(&mut r, 1500);
        r.step(false, true, false);
        age(&mut r, 300);
        assert_eq!(r.step(true, true, false).throttle, CRANK_THROTTLE);
        // Running steadily: parking brake off, then drive.
        age(&mut r, 1500);
        assert_eq!(r.step(true, true, false).pulse, None);
        assert_eq!(r.step(true, true, false).pulse, Some(PARKING_PULSE));
        age(&mut r, 300);
        assert_eq!(r.step(true, false, false).pulse, Some(GEAR_DRIVE_PULSE));
        age(&mut r, 500);
        r.step(true, false, true);
        assert!(r.done);
    }
    #[test]
    fn braking_rebound_and_independent_aeb_never_request_forward_throttle() {
        let accelerate = Output {
            steer: 0.1,
            throttle: 0.3,
            brake: 0.0,
        };
        let bounce = respect_safety(accelerate, 0.0, -0.06);
        assert_eq!(bounce.throttle, 0.0);
        assert_eq!(bounce.brake, 0.5);
        let aeb = respect_safety(accelerate, 1.0, 3.0);
        assert_eq!(aeb.throttle, 0.0);
        assert_eq!(aeb.brake, 1.0);
        assert_eq!(respect_safety(accelerate, 0.0, 3.0).throttle, 0.3);
    }
    #[test]
    fn hud_tells_the_driver_what_to_press_in_each_mode() {
        use crate::hud::Tone;
        let mut s = Progress {
            frame_id: 1,
            maximum_kmh: 50.0,
            limit_kmh: 60.0,
            speed_kmh: 40.0,
            remaining_s: f64::INFINITY,
            remaining_route_m: Some(1800.0),
            mode: Mode::Ready,
            ..Default::default()
        };
        let ready = hud_model(&s);
        assert_eq!(ready.tone, Tone::Ready);
        assert!(ready.hint.contains("F8"));
        assert!(ready.detail.iter().any(|d| d.contains("1.8 km")));
        s.mode = Mode::Engaged;
        let engaged = hud_model(&s);
        assert_eq!(
            (engaged.tone, engaged.badge.as_str()),
            (Tone::Active, "不限时")
        );
        assert!(engaged.hint.contains("接管") && engaged.scale_kmh >= 60.0);
        s.mode = Mode::TakeoverSoon;
        s.reason = "前方路口需要手动处理 · 120 m".into();
        let soon = hud_model(&s);
        assert!(soon.blink && soon.tone == Tone::Warning && soon.detail[0].contains("120 m"));
        s.mode = Mode::Released;
        let released = hud_model(&s);
        assert!(released.tone == Tone::Idle && released.speed_kmh.is_none());
        s.frame_id = 0;
        s.mode = Mode::Unavailable;
        assert!(hud_model(&s).speed_kmh.is_none());
    }
}
