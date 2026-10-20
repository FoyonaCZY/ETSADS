//! One-click static steering calibration for the current truck: parked, zero
//! throttle, the wheel is held at a series of axis levels while the front
//! wheel angles are measured. Produces the same report the assist loads, keyed
//! by the truck's geometry fingerprint. It never moves the truck: any motion,
//! throttle, gear change, pause or manual takeover aborts and releases.
//! With relative steering (the default for new profiles) the levels are
//! steering positions reached through the servo and measured from telemetry.
use super::{
    core::{Output, SteeringMap, vehicle_geometry},
    io,
};
use anyhow::{Context, ensure};
use etsads_control::static_identification::{StaticSteeringSample, analyze};
use etsads_game::adapter::GameAdapter;
use etsads_msgs::protocol::*;
use etsads_shm::{SingleWriterGuard, qpc, qpc_frequency};
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

/// Axis levels in order; neutral is revisited so its group straddles both
/// directions. Full lock is included: the controller limits it by speed.
const LEVELS: [f64; 13] = [
    0.0, 0.15, 0.3, 0.5, 0.75, 1.0, 0.0, -0.15, -0.3, -0.5, -0.75, -1.0, 0.0,
];
/// Axis slew while moving between levels; the game wheel follows smoothly.
const SLEW_PER_S: f64 = 1.0;
const HOLD_S: f64 = 1.8;
const SETTLE_S: f64 = 0.75;
/// With relative steering a level is held once the measured steering
/// position is this close (half the analysis' 0.005 grouping).
const HELD_TOLERANCE: f64 = 0.002;
const MODE_MISMATCH: &str =
    "转向没有稳定在目标位置：游戏的转向方式与档案设置不符，请确认当前档案后重试";

#[derive(Clone, Debug)]
pub struct CalibrationProgress {
    /// 0.0..=1.0
    pub fraction: f64,
    pub message: String,
}

/// The report file for a truck, inside `dir`.
pub fn report_path(dir: &Path, fingerprint: &str) -> PathBuf {
    let short: String = fingerprint
        .chars()
        .filter(|c| c.is_ascii_hexdigit())
        .take(16)
        .collect();
    dir.join(format!("steering-{short}.json"))
}

pub fn run(
    dir: &Path,
    stop: Arc<AtomicBool>,
    mut progress: impl FnMut(CalibrationProgress),
) -> anyhow::Result<PathBuf> {
    let _writer = SingleWriterGuard::acquire("ETSADS_Control_Writer")
        .context("自动驾驶正在运行，请先关闭自动驾驶再标定")?;
    // The game applies SDK inputs only while it has focus.
    focus_game();
    let hud = crate::hud::HudPublisher::start().ok();
    let mut say = |fraction: f64, message: &str| {
        if let Some(hud) = &hud {
            hud.show(crate::hud::HudModel {
                tone: crate::hud::Tone::Warning,
                title: "正在标定转向".into(),
                badge: format!("{:.0}%", fraction * 100.0),
                detail: vec![message.into()],
                hint: "请勿操作；方向键/WASD/空格/Esc 取消".into(),
                ..Default::default()
            });
        }
        progress(CalibrationProgress {
            fraction,
            message: message.into(),
        })
    };
    say(0.0, "连接游戏和安全监督");
    let _supervisor = io::SupervisorProcess::ensure()?;
    let mut sink = io::Sink::connect()?;
    let mut game = GameAdapter::connect()?;
    let mut controls = crate::loaded_controls::LoadedControls::connect("")?;
    let started = Instant::now();
    let first = loop {
        ensure!(!stop.load(Ordering::Acquire), "已取消");
        if let Some(s) = game.poll()? {
            break s;
        }
        ensure!(
            started.elapsed() < Duration::from_secs(3),
            "没有收到游戏遥测，请进入驾驶界面"
        );
        std::thread::sleep(Duration::from_millis(5));
    };
    let raw = first.raw;
    ensure!(raw.paused == 0, "游戏处于暂停状态，请回到驾驶画面");
    ensure!(raw.speed_mps.abs() < 0.05, "请先停稳车辆");
    ensure!(raw.parking_brake != 0, "请先拉起驻车制动（默认空格键）");
    let config = first.config.context("车辆配置未知，请稍等后重试")?;
    let fingerprint =
        etsads_msgs::vehicle::geometry_fingerprint(&config).map_err(anyhow::Error::msg)?;
    let (_, front) = vehicle_geometry(&config)?;
    let profile = crate::setup::active_steering()?;
    controls.bind(&raw)?;
    // The supervisor checks command freshness the moment it arms: publish a
    // stream of neutral commands first, then keep it going while waiting.
    let warmup = Instant::now();
    while warmup.elapsed() < Duration::from_millis(300) {
        if let Some(s) = game.poll()? {
            sink.publish(s.raw.header.frame_id, super::neutral())?;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    say(0.02, "请求安全监督授权（随时按方向键/WASD/空格/Esc 取消）");
    sink.request(true)?;
    let gate_active = |sink: &io::Sink| {
        crate::loaded_controls::coherent_read(|| sink.gate.latest()).is_some_and(|g| {
            g.enabled == GATE_ACTIVE
                && etsads_safety::fresh(qpc(), g.heartbeat_qpc, qpc_frequency(), 150)
        })
    };
    let wait = Instant::now();
    while !gate_active(&sink) {
        ensure!(!stop.load(Ordering::Acquire), "已取消");
        ensure!(
            wait.elapsed() < Duration::from_secs(3),
            "安全监督没有授权：请确认游戏在前台、车辆停稳且未暂停"
        );
        if let Some(s) = game.poll()? {
            sink.publish(s.raw.header.frame_id, super::neutral())?;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    // Until here the input was zero, which holds a relatively steered wheel.
    sink.steering.set_mode(profile.mode);
    sink.observe(0.0, raw.rpm as f64, raw.effective[0] as f64);
    let mut samples = Vec::<StaticSteeringSample>::new();
    let mut gaps = 0_u64;
    let mut frames = 0_u64;
    // The commanded steering position, ramped between levels no faster than
    // the steering mode can follow.
    let slew = sink
        .steering
        .max_rate()
        .map_or(SLEW_PER_S, |rate| SLEW_PER_S.min(0.85 * rate));
    let mut axis = sink
        .steering
        .position(Instant::now())
        .unwrap_or(0.0)
        .clamp(-1.0, 1.0);
    let mut axis_rate = 0.0_f64;
    let mut level = 0;
    let mut level_started = None::<f64>;
    let mut held_since = None::<f64>;
    let mut hold_position = None::<f64>;
    let mut last_time = None::<f64>;
    let mut neutral = None::<f64>;
    let total = LEVELS.len() as f64;
    let relative = sink.steering.max_rate().is_some();
    let result = (|| -> anyhow::Result<()> {
        loop {
            ensure!(!stop.load(Ordering::Acquire), "已取消");
            let Some(s) = game.poll()? else {
                std::thread::sleep(Duration::from_millis(2));
                continue;
            };
            let raw = s.raw;
            ensure!(raw.paused == 0, "游戏暂停，标定已取消");
            ensure!(
                gate_active(&sink),
                "已人工接管或安全监督撤销授权，标定已取消"
            );
            controls.validate(&raw)?;
            crate::loaded_controls::validate_stationary_steering(&first.raw, &raw)
                .map_err(|_| anyhow::anyhow!("车辆移动、换挡或有油门输入，标定已取消"))?;
            sink.observe(0.0, raw.rpm as f64, raw.effective[0] as f64);
            let wheel = front
                .iter()
                .map(|i| raw.wheel_steering[*i] as f64 * std::f64::consts::TAU)
                .sum::<f64>()
                / front.len() as f64;
            let time = raw.header.qpc as f64 / raw.qpc_frequency.max(1) as f64;
            let dt = last_time.map_or(0.0, |t| (time - t).clamp(0.0, 0.1));
            if last_time.is_some_and(|t| time - t > 0.1) {
                gaps += 1;
            }
            last_time = Some(time);
            frames += 1;
            // The game's own steering position (opposite sign of its
            // effective steering), which the table is built on.
            let position = -(raw.effective[0] as f64);
            let target = LEVELS[level];
            let started = *level_started.get_or_insert(time);
            if (axis - target).abs() > 1e-9 {
                if let Some(rate) = sink.steering.max_rate() {
                    (axis, axis_rate) = io::shape_steering(
                        axis,
                        axis_rate,
                        target,
                        0.0,
                        io::relative_limits(rate, 0.0, 0.0),
                        dt,
                    );
                } else {
                    let step = slew * dt;
                    axis = if (target - axis).abs() <= step {
                        target
                    } else {
                        axis + step * (target - axis).signum()
                    };
                }
                held_since = None;
            } else if relative && (position - target).abs() > HELD_TOLERANCE {
                // With relative steering the servo brings the wheel to the
                // level; a level being held must stay held.
                ensure!(held_since.is_none(), MODE_MISMATCH);
                if time - started > (1.0 / slew) + 5.0 {
                    anyhow::bail!(
                        if (position - LEVELS[level.saturating_sub(1)]).abs() < 0.03 {
                            "游戏没有响应方向输入：请让游戏窗口保持在前台、不要操作方向，然后重试"
                        } else {
                            MODE_MISMATCH
                        }
                    );
                }
            } else {
                let since = *held_since.get_or_insert(time);
                // With absolute steering the wheel settles on the input and
                // stays; one that keeps turning is applying it as a rate.
                if !relative && time - since >= SETTLE_S {
                    let from = *hold_position.get_or_insert(position);
                    ensure!((position - from).abs() < 0.02, MODE_MISMATCH);
                }
                if level == 0 && time - since >= HOLD_S * 0.8 {
                    neutral.get_or_insert(wheel);
                }
                // The first steered level must visibly turn the wheels; if
                // not, the game is ignoring the input (not focused).
                if level == 1
                    && time - since >= 1.0
                    && neutral.is_some_and(|n| (wheel - n).abs() < 0.03)
                {
                    anyhow::bail!(
                        "车轮没有随方向转动：请让游戏窗口保持在前台、无转向辅助，然后重试"
                    );
                }
                if time - since >= HOLD_S {
                    level += 1;
                    level_started = None;
                    held_since = None;
                    hold_position = None;
                    if level == LEVELS.len() {
                        return Ok(());
                    }
                }
            }
            say(
                0.05 + 0.9
                    * (level as f64 + held_since.map_or(0.0, |t| ((time - t) / HOLD_S).min(1.0)))
                    / total,
                &format!(
                    "测量第 {}/{} 档（方向 {:+.2}{}）",
                    level + 1,
                    LEVELS.len(),
                    LEVELS[level],
                    if relative { "，相对转向" } else { "" }
                ),
            );
            sink.publish(
                raw.header.frame_id,
                Output {
                    steer: axis,
                    throttle: 0.0,
                    brake: 0.0,
                },
            )?;
            // The table's axis is the steering position the controller will
            // command: the input the game applied with absolute steering, the
            // measured position with relative steering.
            if let Some(applied) = crate::loaded_controls::coherent_read(|| sink.applied.latest())
                .filter(|a| etsads_safety::fresh(qpc(), a.applied_qpc, qpc_frequency(), 150))
                && held_since.is_some()
            {
                samples.push(StaticSteeringSample {
                    time_s: time,
                    axis: if relative {
                        position.clamp(-1.0, 1.0)
                    } else {
                        applied.axes[0] as f64
                    },
                    applied_throttle: applied.axes[1] as f64,
                    effective_throttle: raw.effective[1] as f64,
                    speed_mps: raw.speed_mps as f64,
                    paused: raw.paused != 0,
                    wheel_angles_rad: front
                        .iter()
                        .map(|i| raw.wheel_steering[*i] as f64 * std::f64::consts::TAU)
                        .collect(),
                });
            }
        }
    })();
    // Centre the wheel and release before anything else.
    let _ = sink.publish(0, super::neutral());
    let _ = sink.request(false);
    result?;
    say(0.96, "分析测量结果");
    let analysis = analyze(&samples, SETTLE_S).map_err(anyhow::Error::msg)?;
    ensure!(
        analysis.issues.is_empty() && analysis.monotone_response,
        "测量结果不可用（{}）：请保持游戏在前台、车辆静止并不要操作方向盘后重试",
        if analysis.measured_axis_polarity.is_none() {
            "车轮没有随方向输入转动"
        } else if !analysis.monotone_response {
            "车轮角度与输入不成单调关系"
        } else if analysis.moving_samples > 0 {
            "测量中车辆有移动"
        } else if analysis.nonzero_throttle_samples > 0 {
            "测量中有油门输入"
        } else {
            "有效测量点不足"
        }
    );
    std::fs::create_dir_all(dir)?;
    let path = report_path(dir, &fingerprint);
    let report = serde_json::json!({
        "schema_version": 1,
        "report_kind": "static_zero_throttle_wheel_angle_scan_not_driving_calibration",
        "vehicle_id": format!("live-{}", &fingerprint.chars().take(16).collect::<String>()),
        "source_run": "live one-click calibration",
        "recording_source": "game",
        "complete_recording": true,
        "frame_gaps": gaps,
        "geometry_fingerprint": fingerprint,
        "steered_wheel_indices": front,
        "synchronized_applied_samples": samples.len(),
        "recording_telemetry_frames": frames,
        "unaligned_or_inactive_frames": frames.saturating_sub(samples.len() as u64),
        "analysis": analysis,
    });
    let temporary = path.with_extension("json.tmp");
    std::fs::write(&temporary, serde_json::to_vec_pretty(&report)?)?;
    // Only a report the assist itself accepts replaces an existing one.
    if let Err(error) = SteeringMap::read(&temporary) {
        let _ = std::fs::remove_file(&temporary);
        anyhow::bail!("测量结果未通过校验：{error:#}");
    }
    std::fs::rename(&temporary, &path)?;
    say(1.0, "标定完成");
    Ok(path)
}

/// Bring the game window to the foreground (we are the foreground process
/// when the driver clicks, so Windows allows the switch).
fn focus_game() {
    #[cfg(windows)]
    unsafe {
        use windows_sys::Win32::UI::WindowsAndMessaging::{
            FindWindowW, SW_RESTORE, SetForegroundWindow, ShowWindow,
        };
        let title: Vec<u16> = "Euro Truck Simulator 2\0".encode_utf16().collect();
        let window = FindWindowW(std::ptr::null(), title.as_ptr());
        if !window.is_null() {
            ShowWindow(window, SW_RESTORE);
            SetForegroundWindow(window);
            std::thread::sleep(Duration::from_millis(400));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn levels_cover_both_directions_and_return_to_centre() {
        assert_eq!(LEVELS.first(), Some(&0.0));
        assert_eq!(LEVELS.last(), Some(&0.0));
        assert!(LEVELS.iter().any(|l| *l >= 1.0) && LEVELS.iter().any(|l| *l <= -1.0));
        let duration: f64 = LEVELS
            .windows(2)
            .map(|w| (w[1] - w[0]).abs() / SLEW_PER_S + HOLD_S)
            .sum();
        assert!(duration < 45.0, "about half a minute, was {duration}");
    }
    #[test]
    fn report_name_is_stable_and_safe() {
        let p = report_path(Path::new("d"), "ab12/../CD34ef567890aa11bb22");
        assert_eq!(p, Path::new("d").join("steering-ab12CD34ef567890.json"));
    }
}
