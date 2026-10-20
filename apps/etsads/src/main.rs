use clap::{Parser, Subcommand};
mod offline_replay;
mod offline_scenarios;
use etsads_msgs::protocol::*;
use etsads_shm::SharedRing;
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};
fn emit_line(
    output: &mut impl std::io::Write,
    args: std::fmt::Arguments<'_>,
) -> std::io::Result<bool> {
    let result = (|| {
        output.write_fmt(args)?;
        output.write_all(b"\n")
    })();
    match result {
        Ok(()) => Ok(true),
        Err(error) if error.kind() == std::io::ErrorKind::BrokenPipe => Ok(false),
        Err(error) => Err(error),
    }
}
macro_rules! cli_println {($($args:tt)*)=>{{let _=emit_line(&mut std::io::stdout().lock(),format_args!($($args)*))?;}}}
#[derive(Parser)]
#[command(
    name = "etsads-cli",
    version,
    about = "ETSADS • 欧卡2驾驶开发、遥测与验收工具"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// 把游戏内 HUD 的各状态渲染成 PNG 预览，不连接游戏。
    HudPreview { output: PathBuf },
    /// 读取游戏内 HUD 渲染端回报的状态（只读）。
    HudStatus,
    /// 只读检查道路辅助的车辆、GPS与附近车道；不发送控制。
    AssistInspect,
    /// 一键标定当前车辆的转向：停稳拉驻车后约半分钟，结果按车辆自动选用。
    CalibrateSteering,
    /// 检查首次设置：游戏、插件、地图和已标定车辆（只读）。
    SetupCheck,
    /// 列出最近的游戏事件（罚单、任务等，只读）。
    Events,
    /// 汇总辅助驾驶日志：里程、各类道路车速、限速原因、释放事件（只读）。
    TripStats {
        /// 日志文件或目录，默认 runs/assist。
        logs: Vec<PathBuf>,
    },
    /// 安装/更新游戏插件（需先退出游戏）；替换的文件备份到 data/plugin-backup。
    InstallPlugins,
    /// 从本机游戏文件生成自动驾驶地图（约 5 分钟）。
    GenerateMap,
    /// 熄火驻车下验证结束停车握手；始终零油门，并保持行车制动。
    AssistStopCheck {
        #[arg(long)]
        arm: bool,
    },
    /// 道路辅助MVP：准备后在游戏按F8；默认30km/h试验上限。
    Assist {
        #[arg(long)]
        map: Option<PathBuf>,
        #[arg(long)]
        steering_report: Option<PathBuf>,
        #[arg(long)]
        log_dir: Option<PathBuf>,
        #[arg(long, default_value_t = 60)]
        seconds: u64,
        #[arg(long, default_value_t = 30.0)]
        maximum_kmh: f64,
        #[arg(long, default_value_t = 300)]
        prepare_seconds: u64,
        /// 满足上路条件后请求一次启用；省略时需游戏内热键。
        #[arg(long)]
        arm: bool,
        /// 时长结束后先减速停稳并确认驻车；人工接管仍立即释放。
        #[arg(long)]
        stop_at_end: bool,
        /// 每次结束后回到待命，等待下一次热键启用（seconds=0 表示不限时）。
        #[arg(long)]
        continuous: bool,
        /// 燃油低于该升数时就地安全停车（长途测试中便于加油后继续）。
        #[arg(long)]
        fuel_stop_l: Option<f32>,
    },
    /// 检查游戏安装、插件和遥测连接。
    Doctor,
    /// 每秒十次显示实时遥测摘要（只读）。
    Telemetry {
        #[arg(long, default_value_t = 10)]
        seconds: u64,
    },
    /// 录制真实游戏数据到新的 MCAP 运行目录。
    Record {
        output: PathBuf,
        #[arg(long, default_value_t = 60)]
        seconds: u64,
        #[arg(long)]
        video: bool,
    },
    /// 校验并回放运行目录，显示帧缺失/时间单调性。
    Replay {
        run: PathBuf,
        /// 同时重建供Runtime使用的逐帧快照；要求录制有有效QPC频率。
        #[arg(long)]
        snapshots: bool,
    },
    /// 把真实或合成录制逐帧送入Runtime；只写离线结果，绝不控制游戏。
    ReplayDrive {
        run: PathBuf,
        #[arg(long)]
        config: PathBuf,
        #[arg(long)]
        calibration: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long)]
        map: Option<PathBuf>,
        #[arg(long)]
        policy: Option<PathBuf>,
        #[arg(long, default_value_t = 2000.0)]
        region_radius_m: f64,
    },
    /// 从录制自动切出接管、急刹、红灯接近和地图指定环岛片段。
    ScenarioExtract {
        run: PathBuf,
        #[arg(long)]
        map: PathBuf,
        #[arg(long)]
        policy: PathBuf,
        #[arg(long)]
        settings: Option<PathBuf>,
        #[arg(long)]
        output: PathBuf,
    },
    /// 使用辨识模型和IDM交通运行完整录制场景库；不连接游戏输入。
    ScenarioRun {
        run: PathBuf,
        #[arg(long)]
        library: PathBuf,
        #[arg(long)]
        map: PathBuf,
        #[arg(long)]
        policy: PathBuf,
        #[arg(long)]
        config: PathBuf,
        #[arg(long)]
        calibration: PathBuf,
        #[arg(long)]
        baseline: Option<PathBuf>,
        #[arg(long)]
        output: PathBuf,
    },
    /// 可重复的录制/回放基准；结果始终标记 synthetic。
    RecordBench {
        output: PathBuf,
        #[arg(long, default_value_t = 216_000)]
        frames: u64,
    },
    /// 请求独立安全监督开关。暂停后必须重新显式启用。
    Gate {
        #[arg(value_parser=["on","off"])]
        state: String,
    },
    /// 受安全监督约束的短时间输入实验；默认零油门。
    Steer {
        #[arg(allow_negative_numbers = true)]
        value: f32,
        #[arg(long, default_value_t = 3.0)]
        seconds: f64,
        #[arg(long, default_value_t = 0.0)]
        throttle: f32,
        #[arg(long, default_value_t = 0.0)]
        brake: f32,
        #[arg(long)]
        arm: bool,
    },
    /// 列出 SDK 语义输入名称。
    Inputs,
    /// 运行可重复的离线闭环回归，保存完整轨迹与指标。
    Sim {
        #[arg(long, default_value = "artifacts/simulation.json")]
        output: PathBuf,
    },
    /// 从游戏捕获插件保存一帧 PNG 与帧号/QPC 元数据。
    FrameDump { output: PathBuf },
    /// 分析真实录制的物理步频率、暂停时钟及输入生效信息。
    Analyze { run: PathBuf },
    /// 依据真实录制计算距离、罚单、控制延迟与证据缺口。
    Metrics { run: PathBuf },
    /// 执行一次语义按键脉冲（需要已启用的安全监督）。
    Pulse {
        name: String,
        #[arg(long)]
        arm: bool,
    },
    /// 真实游戏驾驶栈；默认仅观察，高阶开发试验需要独立证据许可。
    Drive(Box<DriveArgs>),
}
#[derive(clap::Args)]
struct DriveArgs {
    #[arg(long)]
    config: PathBuf,
    #[arg(long)]
    calibration: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long)]
    map: Option<PathBuf>,
    #[arg(long)]
    policy: Option<PathBuf>,
    #[arg(long, default_value_t = 2000.0)]
    region_radius_m: f64,
    #[arg(long, default_value_t = 60)]
    seconds: u64,
    #[arg(long)]
    arm: bool,
    #[arg(long)]
    video: bool,
    #[arg(long, value_enum)]
    experiment: Option<etsads::driving::experiment::ExperimentKind>,
    #[arg(long)]
    experiment_evidence: Option<PathBuf>,
    /// 道路运行的地图、标定与真实记录证明；不替代阶段实车验收。
    #[arg(long)]
    road_evidence: Option<PathBuf>,
    #[arg(long)]
    safety_profile: Option<PathBuf>,
}
fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::AssistStopCheck { arm } => {
            cli_println!(
                "{}",
                serde_json::to_string_pretty(&etsads::assist::check_finish(
                    arm,
                    manual_stop_signal()?
                )?)?
            );
        }
        Command::HudStatus => {
            use etsads_msgs::protocol::{
                HUD_MAPPING, HUD_STATUS_MAPPING, HudImage, HudStatus, SCHEMA_VERSION,
            };
            let status =
                etsads_shm::SharedCell::<HudStatus>::open(HUD_STATUS_MAPPING, SCHEMA_VERSION)
                    .ok()
                    .and_then(|s| s.latest());
            let images = etsads_shm::SharedRing::<HudImage, 2>::open(HUD_MAPPING, SCHEMA_VERSION);
            let latest = images.as_ref().map(|r| r.latest_index()).ok();
            let now = etsads_shm::qpc();
            let f = etsads_shm::qpc_frequency() as f64;
            cli_println!(
                "renderer={:?} present_age_ms={:?} image_ring={:?} latest_index={:?}",
                status,
                status.map(|s| (now.saturating_sub(s.present_qpc)) as f64 / f * 1000.0),
                images.as_ref().map(|_| "ok").map_err(|e| e.to_string()),
                latest
            );
        }
        Command::HudPreview { output } => {
            for path in etsads::hud::preview(&output)? {
                cli_println!("{}", path.display());
            }
        }
        Command::AssistInspect => {
            cli_println!(
                "{}",
                serde_json::to_string_pretty(&etsads::assist::inspect(Default::default())?)?
            );
        }
        Command::CalibrateSteering => {
            let options = etsads::assist::Options::default();
            let mut last = String::new();
            let path = etsads::assist::calibrate::run(
                &options.steering_dir,
                manual_stop_signal()?,
                |p| {
                    if p.message != last {
                        let _ = emit_line(
                            &mut std::io::stdout().lock(),
                            format_args!("{:3.0}% {}", p.fraction * 100.0, p.message),
                        );
                        last = p.message;
                    }
                },
            )?;
            cli_println!("已保存：{}", path.display());
        }
        Command::TripStats { logs } => {
            let logs = if logs.is_empty() {
                vec![etsads::assist::Options::default().log_dir]
            } else {
                logs
            };
            cli_println!("{}", etsads::assist::trip_stats::summarize(&logs)?);
        }
        Command::Events => {
            use etsads_msgs::protocol::{EVENT_SLOTS, EVENTS_MAPPING, GameEvent, SCHEMA_VERSION};
            let ring = etsads_shm::SharedRing::<GameEvent, EVENT_SLOTS>::open(
                EVENTS_MAPPING,
                SCHEMA_VERSION,
            )?;
            let latest = ring.latest_index();
            let text = |b: &[u8]| {
                String::from_utf8_lossy(&b[..b.iter().position(|c| *c == 0).unwrap_or(b.len())])
                    .into_owned()
            };
            for index in latest.saturating_sub(EVENT_SLOTS as u64 - 1)..=latest {
                if let Some(e) = ring.read(index) {
                    cli_println!(
                        "#{index} frame={} {} {}",
                        e.header.frame_id,
                        text(&e.name),
                        text(&e.detail)
                    );
                }
            }
        }
        Command::SetupCheck => {
            let options = etsads::assist::Options::default();
            let r = etsads::setup::check(&options.map, &options.steering_dir);
            cli_println!(
                "game_dir={:?}\ngame_version={:?} supported={} running={}\nbridge={:?} graphics={:?} ets2la={:?}\nmap={:?} at {}\ncalibrated_trucks={}\nsteering={}",
                r.game_dir,
                r.game_version,
                r.version_supported,
                r.game_running,
                r.bridge,
                r.graphics,
                r.ets2la,
                r.map,
                r.map_dir.display(),
                r.calibrated_trucks,
                r.steering
                    .as_ref()
                    .map_or("未知".to_owned(), ToString::to_string)
            );
        }
        Command::InstallPlugins => {
            etsads::setup::install_plugins(|line| {
                let _ = emit_line(&mut std::io::stdout().lock(), format_args!("{line}"));
            })?;
        }
        Command::GenerateMap => {
            let output = etsads::setup::data_dir().join("map");
            let stop = manual_stop_signal()?;
            let mut last = usize::MAX;
            etsads::setup::generate_map(&output, &stop, |p| {
                if p.step != last {
                    last = p.step;
                    let _ = emit_line(
                        &mut std::io::stdout().lock(),
                        format_args!(
                            "[{}/{}] {} （已用 {:.0} s）",
                            p.step + 1,
                            p.steps,
                            p.message,
                            p.elapsed_s
                        ),
                    );
                }
            })?;
            cli_println!("地图已生成：{}", output.display());
        }
        Command::Assist {
            map,
            steering_report,
            log_dir,
            seconds,
            maximum_kmh,
            prepare_seconds,
            arm,
            stop_at_end,
            continuous,
            fuel_stop_l,
        } => {
            let mut options = etsads::assist::Options {
                seconds,
                maximum_kmh,
                prepare_seconds,
                arm,
                stop_at_end,
                continuous,
                fuel_stop_l,
                ..Default::default()
            };
            if let Some(path) = map {
                options.map = path;
            }
            if let Some(path) = steering_report {
                options.steering_report = path;
            }
            if let Some(path) = log_dir {
                options.log_dir = path;
            }
            let (tx, rx) = std::sync::mpsc::sync_channel(16);
            let output = std::thread::spawn(move || {
                for status in rx {
                    if !emit_line(
                        &mut std::io::stdout().lock(),
                        format_args!("{}", serde_json::to_string(&status).unwrap_or_default()),
                    )
                    .unwrap_or(false)
                    {
                        break;
                    }
                }
            });
            let result = etsads::assist::run(options, manual_stop_signal()?, Some(tx));
            let _ = output.join();
            cli_println!("{}", serde_json::to_string_pretty(&result?)?);
        }
        Command::Doctor => {
            let game = etsads::find_game();
            let t = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
                TELEMETRY_MAPPING,
                SCHEMA_VERSION,
            )
            .ok()
            .and_then(|r| r.latest());
            cli_println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"environment":etsads::environment::describe(),"control_configuration":etsads::environment::control_status(),"game_dir":game,"bridge_installed":game.as_ref().is_some_and(|g|g.join("bin/win_x64/plugins/etsads_bridge.dll").exists()),"telemetry_connected":t.is_some_and(|v|etsads_safety::fresh(etsads_shm::qpc(),v.header.qpc,etsads_shm::qpc_frequency(),500)),"frame_id":t.map(|v|v.header.frame_id),"live_acceptance":"pending evidence; see docs/ACCEPTANCE.md"})
                )?
            );
        }
        Command::Telemetry { seconds } => {
            let ring = SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(
                TELEMETRY_MAPPING,
                SCHEMA_VERSION,
            )?;
            let start = std::time::Instant::now();
            while start.elapsed().as_secs() < seconds {
                if let Some(t) = ring.latest()
                    && !emit_line(
                        &mut std::io::stdout().lock(),
                        format_args!(
                            "frame={} sim={:.3}s speed={:.2}km/h rpm={:.0} fuel={:.0}L parking={} gear={} paused={} pos={:?} input={:?} effective={:?}",
                            t.header.frame_id,
                            t.header.sim_time_us as f64 / 1e6,
                            t.speed_mps * 3.6,
                            t.rpm,
                            t.fuel_l,
                            t.parking_brake,
                            t.gear,
                            t.paused,
                            t.position_scs,
                            t.input,
                            t.effective
                        ),
                    )?
                {
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
        Command::Record {
            output,
            seconds,
            video,
        } => {
            let stop = Arc::new(AtomicBool::new(false));
            let s = stop.clone();
            ctrlc::set_handler(move || s.store(true, Ordering::Relaxed))?;
            cli_println!(
                "{}",
                serde_json::to_string_pretty(&etsads::record_game_with_video(
                    &output, seconds, &stop, video
                )?)?
            );
        }
        Command::Replay { run, snapshots } => {
            let integrity = if snapshots {
                let report = etsads_replay::ReplayAdapter::run(&run, |_| Ok(()))?;
                cli_println!("{}", serde_json::to_string_pretty(&report)?);
                report.integrity
            } else {
                let report = etsads_recorder::replay(&run, |_, _| Ok(()))?;
                cli_println!("{}", serde_json::to_string_pretty(&report)?);
                report
            };
            anyhow::ensure!(
                integrity.completed && integrity.frame_gaps == 0 && integrity.nonmonotonic == 0,
                "recording integrity check failed"
            );
        }
        Command::ReplayDrive {
            run,
            config,
            calibration,
            output,
            map,
            policy,
            region_radius_m,
        } => {
            let report = offline_replay::run(offline_replay::Options {
                run,
                config,
                calibration,
                output,
                map,
                policy,
                region_radius_m,
            })?;
            cli_println!("{}", serde_json::to_string_pretty(&report)?);
        }
        Command::ScenarioExtract {
            run,
            map,
            policy,
            settings,
            output,
        } => {
            let report =
                offline_scenarios::extract(&run, &map, &policy, settings.as_deref(), &output)?;
            cli_println!("{}", serde_json::to_string_pretty(&report)?);
        }
        Command::ScenarioRun {
            run,
            library,
            map,
            policy,
            config,
            calibration,
            baseline,
            output,
        } => {
            let report = offline_scenarios::run(offline_scenarios::RunOptions {
                run: &run,
                library: &library,
                map: &map,
                policy: &policy,
                config: &config,
                calibration: &calibration,
                baseline: baseline.as_deref(),
                output: &output,
            })?;
            cli_println!("{}", serde_json::to_string_pretty(&report)?);
        }
        Command::RecordBench { output, frames } => {
            anyhow::ensure!(
                frames > 0 && frames <= 10_000_000,
                "frames must be 1..10000000"
            );
            let started = std::time::Instant::now();
            let mut r = etsads_recorder::Recorder::create(
                &output,
                etsads_recorder::RunMetadata::new("synthetic", "none"),
            )?;
            for i in 1..=frames {
                let mut t = TelemetryFrame::default();
                t.header.frame_id = i;
                t.header.sim_time_us = i * 1_000_000 / 60;
                t.header.qpc = i;
                t.qpc_frequency = 60;
                t.paused_simulation_time_us = t.header.sim_time_us;
                t.speed_mps = 20.0;
                t.position_scs = [i as f64 / 3.0, 0.0, 0.0];
                r.record("/telemetry", &etsads_recorder::Record::telemetry(&t))?;
            }
            r.finish()?;
            let report = etsads_recorder::replay(&output, |_, _| Ok(()))?;
            cli_println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"elapsed_seconds":started.elapsed().as_secs_f64(),"report":report})
                )?
            );
        }
        Command::Gate { state } => {
            etsads::set_gate(state == "on")?;
            cli_println!("Safety request sent: {state}");
        }
        Command::Steer {
            value,
            seconds,
            throttle,
            brake,
            arm,
        } => {
            let stop = manual_stop_signal()?;
            etsads::manual_command_with_stop(value, throttle, brake, seconds, None, arm, &stop)?;
        }
        Command::Inputs => cli_println!(
            "Axes: {:?}\nHolds: {:?}\nPulses: {:?}",
            etsads_game::input::AXIS_NAMES,
            etsads_game::input::HOLD_NAMES,
            etsads_game::input::PULSE_NAMES
        ),
        Command::Sim { output } => {
            let report = etsads_sim::run_suite()?;
            if let Some(parent) = output.parent() {
                std::fs::create_dir_all(parent)?;
            }
            serde_json::to_writer_pretty(std::fs::File::create(&output)?, &report)?;
            cli_println!("{}", serde_json::to_string_pretty(&report.checks)?);
            cli_println!("Offline report: {}", output.display());
            anyhow::ensure!(report.all_passed, "offline scenario regression failed");
        }
        Command::FrameDump { output } => {
            let mut reader = etsads_game::capture::CaptureReader::connect()?;
            let start = std::time::Instant::now();
            loop {
                if let Some(frame) = reader.poll()? {
                    let mut rgba =
                        Vec::with_capacity(frame.width as usize * frame.height as usize * 4);
                    for row in frame
                        .data
                        .chunks_exact(frame.stride as usize)
                        .take(frame.height as usize)
                    {
                        for pixel in row[..frame.width as usize * 4].as_chunks::<4>().0 {
                            if matches!(frame.format, 87 | 90 | 91 | 88 | 92 | 93) {
                                rgba.extend_from_slice(&[pixel[2], pixel[1], pixel[0], 255]);
                            } else {
                                rgba.extend_from_slice(&[pixel[0], pixel[1], pixel[2], 255]);
                            }
                        }
                    }
                    if let Some(parent) = output.parent() {
                        std::fs::create_dir_all(parent)?;
                    }
                    image::save_buffer(
                        &output,
                        &rgba,
                        frame.width,
                        frame.height,
                        image::ColorType::Rgba8,
                    )?;
                    let info = serde_json::json!({"source":"game","capture_frame_index":frame.frame_index,"timestamp_qpc":frame.timestamp_qpc,"publish_qpc":frame.publish_qpc,"qpc_frequency":reader.qpc_frequency,"width":frame.width,"height":frame.height});
                    serde_json::to_writer_pretty(
                        std::fs::File::create(output.with_extension("json"))?,
                        &info,
                    )?;
                    cli_println!("{info}");
                    break;
                }
                anyhow::ensure!(
                    start.elapsed() < Duration::from_secs(5),
                    "no capture frame within 5 seconds"
                );
                std::thread::sleep(Duration::from_millis(5));
            }
        }
        Command::Analyze { run } => {
            let report = etsads::analyze(&run)?;
            cli_println!("{}", serde_json::to_string_pretty(&report)?);
        }
        Command::Metrics { run } => cli_println!(
            "{}",
            serde_json::to_string_pretty(&etsads_metrics::analyze(&run)?)?
        ),
        Command::Pulse { name, arm } => {
            let index = etsads_game::input::PULSE_NAMES
                .iter()
                .position(|n| *n == name)
                .ok_or_else(|| anyhow::anyhow!("unknown pulse; use inputs to list"))?;
            let stop = manual_stop_signal()?;
            etsads::manual_command_with_stop(0.0, 0.0, 0.0, 0.3, Some(index), arm, &stop)?;
            if name == "quickload" {
                cli_println!("quickload 请求已由 SDK 完整批次确认；载入完成仍须等待新鲜驾驶遥测。");
            }
        }
        Command::Drive(args) => {
            let DriveArgs {
                config,
                calibration,
                output,
                map,
                policy,
                region_radius_m,
                seconds,
                arm,
                video,
                experiment,
                experiment_evidence,
                road_evidence,
                safety_profile,
            } = *args;
            let stop = Arc::new(AtomicBool::new(false));
            let signal = stop.clone();
            ctrlc::set_handler(move || signal.store(true, Ordering::Release))?;
            let report = etsads::driving::run(
                etsads::driving::DriveOptions {
                    config,
                    calibration,
                    output,
                    map,
                    policy,
                    region_radius_m,
                    seconds,
                    arm,
                    video,
                    experiment,
                    experiment_evidence,
                    road_evidence,
                    safety_profile,
                },
                stop,
            )?;
            cli_println!("{}", serde_json::to_string_pretty(&report)?);
        }
    }
    Ok(())
}

fn manual_stop_signal() -> anyhow::Result<Arc<AtomicBool>> {
    let stop = Arc::new(AtomicBool::new(false));
    let signal = stop.clone();
    ctrlc::set_handler(move || signal.store(true, Ordering::Release))?;
    Ok(stop)
}

#[cfg(test)]
mod tests {
    use super::*;
    struct ClosedPipe;
    impl std::io::Write for ClosedPipe {
        fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
            Err(std::io::Error::new(
                std::io::ErrorKind::BrokenPipe,
                "reader closed",
            ))
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    #[test]
    fn closed_stdout_is_a_normal_stream_end() {
        assert!(!emit_line(&mut ClosedPipe, format_args!("frame={}", 1)).unwrap());
        let mut bytes = Vec::new();
        assert!(emit_line(&mut bytes, format_args!("frame={}", 42)).unwrap());
        assert_eq!(bytes, b"frame=42\n");
    }
}
