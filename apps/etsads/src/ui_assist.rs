use super::{ACCENT, AMBER, MUTED, PANEL};
use eframe::egui::{self, Color32, RichText, Stroke};
use etsads::assist::{Longitudinal, Mode, Motion, Options, Progress, Report};
use etsads::setup::{MapState, PluginState, SetupReport};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc,
};
use std::time::{Duration, Instant};

const READY: Color32 = Color32::from_rgb(86, 190, 255);
const ALERT: Color32 = Color32::from_rgb(255, 96, 96);
const IDLE: Color32 = Color32::from_rgb(150, 163, 178);

#[derive(Clone, Copy, PartialEq, Eq)]
enum TaskKind {
    Plugins,
    Map,
    Calibrate,
}
/// A setup action running on its own thread.
struct Task {
    kind: TaskKind,
    stop: Arc<AtomicBool>,
    progress: mpsc::Receiver<(f64, String)>,
    done: mpsc::Receiver<Result<String, String>>,
    fraction: f64,
    message: String,
}
struct Setup {
    report: Option<SetupReport>,
    /// Some(calibrated) while the game is in the driving view.
    truck: Option<bool>,
    pending: Option<mpsc::Receiver<(SetupReport, Option<bool>)>>,
    checked: Instant,
    task: Option<Task>,
    outcome: Option<(TaskKind, Result<String, String>)>,
    expanded: bool,
}

pub struct AssistPanel {
    options: Options,
    unlimited: bool,
    map: String,
    steering: String,
    stop: Option<Arc<AtomicBool>>,
    progress: Option<mpsc::Receiver<Progress>>,
    done: Option<mpsc::Receiver<Result<Report, String>>>,
    status: Progress,
    result: String,
    setup: Setup,
}
impl Default for AssistPanel {
    fn default() -> Self {
        let options = Options {
            continuous: true,
            seconds: 0,
            ..Options::default()
        };
        Self {
            map: options.map.display().to_string(),
            steering: options.steering_report.display().to_string(),
            options,
            unlimited: true,
            stop: None,
            progress: None,
            done: None,
            status: Progress::default(),
            result: String::new(),
            setup: Setup {
                report: None,
                truck: None,
                pending: None,
                checked: Instant::now() - Duration::from_secs(60),
                task: None,
                outcome: None,
                expanded: false,
            },
        }
    }
}
impl AssistPanel {
    /// The assist itself or a setup action holds the game.
    pub fn busy(&self) -> bool {
        self.running() || self.setup.task.is_some()
    }
    fn running(&self) -> bool {
        self.done.is_some()
    }
    pub fn request_stop(&self) {
        if let Some(stop) = &self.stop {
            stop.store(true, Ordering::Release);
        }
        if let Some(task) = &self.setup.task {
            task.stop.store(true, Ordering::Release);
        }
    }
    fn poll_setup(&mut self) {
        let setup = &mut self.setup;
        if let Some(rx) = &setup.pending
            && let Ok((report, truck)) = rx.try_recv()
        {
            setup.report = Some(report);
            setup.truck = truck;
            setup.pending = None;
        }
        // Refresh every few seconds off the UI thread (process list, plugin
        // hashes and one telemetry read).
        if setup.pending.is_none()
            && setup.task.is_none()
            && setup.checked.elapsed() > Duration::from_secs(4)
        {
            setup.checked = Instant::now();
            let (tx, rx) = mpsc::sync_channel(1);
            let mut options = self.options.clone();
            options.map = self.map.trim().into();
            let assist_running = self.done.is_some();
            std::thread::spawn(move || {
                let report = etsads::setup::check(&options.map, &options.steering_dir);
                // While the assist runs, its own status reports calibration.
                let truck = if assist_running {
                    None
                } else {
                    etsads::assist::current_truck_calibrated(&options)
                };
                let _ = tx.send((report, truck));
            });
            setup.pending = Some(rx);
        }
        if let Some(task) = &mut setup.task {
            for (fraction, message) in task.progress.try_iter() {
                task.fraction = fraction;
                task.message = message;
            }
            let finished = match task.done.try_recv() {
                Ok(r) => Some(r),
                Err(mpsc::TryRecvError::Disconnected) => Some(Err("任务意外退出".into())),
                Err(mpsc::TryRecvError::Empty) => None,
            };
            if let Some(result) = finished {
                if task.kind == TaskKind::Map && result.is_ok() {
                    self.map = etsads::setup::data_dir().join("map").display().to_string();
                }
                setup.outcome = Some((task.kind, result));
                setup.task = None;
                setup.checked = Instant::now() - Duration::from_secs(60);
            }
        }
    }
    fn start_task(&mut self, kind: TaskKind) {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let (progress_tx, progress) = mpsc::sync_channel::<(f64, String)>(64);
        let (done_tx, done) = mpsc::sync_channel(1);
        let options = self.options.clone();
        std::thread::spawn(move || {
            let send = |f: f64, m: String| {
                let _ = progress_tx.try_send((f, m));
            };
            let run = || -> anyhow::Result<String> {
                match kind {
                    TaskKind::Plugins => {
                        let mut lines = Vec::new();
                        etsads::setup::install_plugins(|line| {
                            send(0.5, line.clone());
                            lines.push(line);
                        })?;
                        Ok(format!("{}。现在可以启动游戏。", lines.join("；")))
                    }
                    TaskKind::Map => {
                        let output = etsads::setup::data_dir().join("map");
                        etsads::setup::generate_map(&output, &worker_stop, |p| {
                            let fraction = match p.step {
                                0 => (p.elapsed_s / 130.0).min(1.0) * 0.45,
                                1 => 0.45 + ((p.elapsed_s - 130.0) / 90.0).clamp(0.0, 1.0) * 0.35,
                                _ => 0.8 + ((p.elapsed_s - 220.0) / 60.0).clamp(0.0, 1.0) * 0.19,
                            };
                            send(
                                fraction,
                                format!(
                                    "第 {}/{} 步：{} · 已用 {:.0} 秒",
                                    p.step + 1,
                                    p.steps,
                                    p.message,
                                    p.elapsed_s
                                ),
                            );
                        })?;
                        Ok("地图已生成".into())
                    }
                    TaskKind::Calibrate => {
                        let path = etsads::assist::calibrate::run(
                            &options.steering_dir,
                            worker_stop.clone(),
                            |p| send(p.fraction, p.message),
                        )?;
                        Ok(format!(
                            "这辆车的转向已标定（{}）",
                            path.file_name().unwrap_or_default().to_string_lossy()
                        ))
                    }
                }
            };
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(run))
                .map_err(|_| "任务异常结束".to_owned())
                .and_then(|r| r.map_err(|e| format!("{e:#}")));
            let _ = done_tx.send(result);
        });
        self.setup.outcome = None;
        self.setup.task = Some(Task {
            kind,
            stop,
            progress,
            done,
            fraction: 0.0,
            message: "开始…".into(),
        });
    }
    pub fn poll(&mut self) -> Option<String> {
        self.poll_setup();
        if let Some(rx) = &self.progress {
            for p in rx.try_iter() {
                self.status = p;
            }
        }
        let result = self.done.as_ref().and_then(|rx| match rx.try_recv() {
            Ok(r) => Some(r),
            Err(mpsc::TryRecvError::Disconnected) => {
                Some(Err("自动驾驶任务意外退出，控制已释放".into()))
            }
            Err(mpsc::TryRecvError::Empty) => None,
        });
        result.map(|result| {
            self.done = None;
            self.progress = None;
            self.stop = None;
            self.result = match result {
                Ok(r) => format!(
                    "自动驾驶已关闭：{} · 诊断记录 {}",
                    r.stop_reason,
                    r.log.display()
                ),
                Err(e) => format!("自动驾驶已关闭：{e}"),
            };
            self.result.clone()
        })
    }
    fn start(&mut self) {
        let mut options = self.options.clone();
        options.map = self.map.trim().into();
        options.steering_report = self.steering.trim().into();
        options.seconds = if self.unlimited {
            0
        } else {
            options.seconds.max(5)
        };
        options.continuous = true;
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let (tx, rx) = mpsc::sync_channel(16);
        let (done_tx, done_rx) = mpsc::sync_channel(1);
        self.stop = Some(stop);
        self.progress = Some(rx);
        self.done = Some(done_rx);
        self.result.clear();
        self.status = Progress::default();
        std::thread::spawn(move || {
            let result =
                std::panic::catch_unwind(|| etsads::assist::run(options, worker_stop, Some(tx)))
                    .map_err(|_| "自动驾驶任务异常，控制已释放".to_owned())
                    .and_then(|r| r.map_err(|e| format!("{e:#}")));
            let _ = done_tx.send(result);
        });
    }
    fn stale(&self) -> bool {
        self.running() && self.status.produced_at.elapsed().as_secs_f64() > 0.7
    }
    fn mode_style(&self) -> (Color32, &'static str) {
        if !self.running() {
            return (IDLE, "自动驾驶已关闭");
        }
        if self.stale() {
            return (AMBER, "等待最新状态…");
        }
        match self.status.mode {
            Mode::Starting => (IDLE, "正在启动…"),
            Mode::Unavailable => (IDLE, "待命 · 暂不可用"),
            Mode::Ready => (READY, "可以启用"),
            Mode::Engaged => (ACCENT, "自动驾驶中"),
            Mode::TakeoverSoon => (AMBER, "请准备接管"),
            Mode::SafetyBrake => (ALERT, "安全制动"),
            Mode::Stopping => (AMBER, "正在停车"),
            Mode::Released => (IDLE, "已交还人工驾驶"),
        }
    }
    pub fn show(&mut self, ui: &mut egui::Ui, other_busy: bool, closing: bool, connected: bool) {
        ui.horizontal(|ui| {
            ui.heading(RichText::new("自动驾驶").size(30.0).strong());
            ui.add_space(8.0);
            ui.label(RichText::new("跟随游戏导航 · 按限速巡航 · 前车跟随").color(MUTED));
        });
        ui.add_space(12.0);
        self.setup_card(ui, other_busy, connected);
        ui.add_space(12.0);
        self.status_card(ui);
        ui.add_space(12.0);
        self.primary_action(ui, other_busy, closing, connected);
        if !self.result.is_empty() && !self.running() {
            ui.add_space(6.0);
            ui.label(RichText::new(&self.result).color(MUTED));
        }
        ui.add_space(14.0);
        ui.columns(2, |columns| {
            self.checklist(&mut columns[0], connected);
            keys_card(&mut columns[1]);
        });
        ui.add_space(14.0);
        self.settings(ui);
        ui.add_space(12.0);
        ui.colored_label(
            AMBER,
            "试验版：跟随导航通过已标注规则的路口（红灯停、绿灯行、让行与停车让行），需要时自动换道；不掉头，不处理收费站和挂车，遇到这些情况之前会提醒你接管并减速停车。请始终注意路况，随时准备接管。",
        );
    }
    /// First-run setup: what is missing and the one button that fixes it.
    fn setup_card(&mut self, ui: &mut egui::Ui, other_busy: bool, connected: bool) {
        let Some(report) = self.setup.report.clone() else {
            ui.label(RichText::new("正在检查游戏、插件和地图…").color(MUTED));
            return;
        };
        let truck_ok = self.setup.truck;
        let game_ok = report.game_dir.is_some();
        let plugins_ok = report.plugins_ready();
        let map_ok = report.map_ready();
        let all_ok = game_ok && plugins_ok && map_ok && truck_ok != Some(false);
        let task_kind = self.setup.task.as_ref().map(|t| t.kind);
        let locked = self.running() || other_busy || self.setup.task.is_some();
        let mut start = None;
        egui::Frame::new()
            .fill(PANEL)
            .stroke(Stroke::new(
                1.5_f32,
                if all_ok { Color32::from_rgb(48, 64, 80) } else { AMBER },
            ))
            .inner_margin(16)
            .corner_radius(12)
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(if all_ok { "✔ 设置完成" } else { "首次设置" })
                            .strong()
                            .size(18.0)
                            .color(if all_ok { ACCENT } else { AMBER }),
                    );
                    if all_ok {
                        ui.label(
                            RichText::new(format!(
                                "欧卡2 {} · 地图就绪 · 已标定 {} 辆车",
                                report.game_version.as_deref().unwrap_or("?"),
                                report.calibrated_trucks
                            ))
                            .color(MUTED),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let label = if self.setup.expanded { "收起" } else { "详情" };
                            if ui.small_button(label).clicked() {
                                self.setup.expanded = !self.setup.expanded;
                            }
                        });
                    } else {
                        ui.label(RichText::new("按顺序完成下面几步，之后就不用再管了。").color(MUTED));
                    }
                });
                if all_ok && !self.setup.expanded && task_kind.is_none() && self.setup.outcome.is_none() {
                    return;
                }
                ui.add_space(8.0);
                // 1. Game
                step_row(
                    ui,
                    game_ok && report.version_supported,
                    "1  欧卡2",
                    &match (&report.game_dir, &report.game_version) {
                        (None, _) => "没有找到游戏。请确认通过 Steam 安装，或设置环境变量 ETSADS_GAME_DIR 指向游戏目录。".into(),
                        (Some(_), Some(v)) if !report.version_supported => {
                            format!("游戏版本 {v} 尚未验证，交通数据插件可能不兼容。")
                        }
                        (Some(_), v) => format!(
                            "已找到，版本 {} · {}",
                            v.as_deref().unwrap_or("?"),
                            if report.game_running { "正在运行" } else { "未运行" }
                        ),
                    },
                    |_| {},
                );
                // 2. Plugins
                let plugin_text = if plugins_ok
                    && report.bridge == PluginState::Current
                    && report.graphics == PluginState::Current
                {
                    "已安装，且是最新版本".to_owned()
                } else if plugins_ok {
                    "已安装（与本程序附带的版本不同，建议更新）".to_owned()
                } else {
                    let missing: Vec<&str> = [
                        ("数据接口", report.bridge),
                        ("画面/HUD", report.graphics),
                        ("交通数据", report.ets2la),
                    ]
                    .iter()
                    .filter(|(_, s)| *s != PluginState::Current && *s != PluginState::Unknown)
                    .map(|(n, _)| *n)
                    .collect();
                    format!("需要安装：{}（一键完成，约 10 秒）", missing.join("、"))
                };
                let plugins_current = plugins_ok
                    && report.bridge != PluginState::Different
                    && report.graphics != PluginState::Different;
                step_row(ui, plugins_ok, "2  游戏插件", &plugin_text, |ui| {
                    if !plugins_current {
                        let can = !locked && !report.game_running && game_ok;
                        if ui
                            .add_enabled(can, egui::Button::new(if plugins_ok { "更新插件" } else { "安装插件" }))
                            .on_disabled_hover_text(if report.game_running {
                                "请先退出游戏（插件文件被占用）"
                            } else {
                                "请稍候"
                            })
                            .clicked()
                        {
                            start = Some(TaskKind::Plugins);
                        }
                        if report.game_running {
                            ui.label(RichText::new("需先退出游戏").color(MUTED));
                        }
                    }
                });
                // 3. Map
                let map_text = match &report.map {
                    MapState::Missing => "还没有地图。从你本机的游戏文件生成，约 5 分钟、占用约 1.6 GB。".into(),
                    MapState::Outdated { map_version } => format!(
                        "地图来自游戏 {map_version}，游戏已更新，请重新生成（约 5 分钟）。"
                    ),
                    MapState::Ready { game_version } => format!("已就绪（游戏 {game_version}）"),
                };
                step_row(ui, map_ok, "3  自动驾驶地图", &map_text, |ui| {
                    if (!map_ok || self.setup.expanded)
                        && ui
                            .add_enabled(
                                !locked && game_ok,
                                egui::Button::new(if map_ok { "重新生成" } else { "生成地图" }),
                            )
                            .clicked()
                    {
                        start = Some(TaskKind::Map);
                    }
                });
                // 4. Steering calibration for the truck being driven
                let truck_text = match truck_ok {
                    Some(true) => "当前驾驶的车已标定".to_owned(),
                    Some(false) => "当前驾驶的车还没有标定。停稳、拉起驻车制动（空格），点右边按钮，约 30 秒，期间不要碰方向盘。".to_owned(),
                    None if self.running() && self.status.reason.contains("还没有标定转向") => {
                        "当前驾驶的车还没有标定：先关闭自动驾驶，再点这里的“标定转向”。".to_owned()
                    }
                    None if self.running() => "由自动驾驶状态显示".to_owned(),
                    None => format!(
                        "进入游戏驾驶画面后检查（已标定 {} 辆车；每辆车只需一次）",
                        report.calibrated_trucks
                    ),
                };
                step_row(ui, truck_ok == Some(true), "4  车辆转向标定", &truck_text, |ui| {
                    if (truck_ok == Some(false) || (self.setup.expanded && truck_ok.is_some()))
                        && ui
                            .add_enabled(
                                !locked && connected && map_ok,
                                egui::Button::new(if truck_ok == Some(true) { "重新标定" } else { "标定转向" }),
                            )
                            .on_disabled_hover_text(if self.running() {
                                "请先关闭自动驾驶"
                            } else {
                                "请进入游戏驾驶画面"
                            })
                            .clicked()
                    {
                        start = Some(TaskKind::Calibrate);
                    }
                });
                if let Some(task) = &self.setup.task {
                    ui.add_space(8.0);
                    ui.add(
                        egui::ProgressBar::new(task.fraction as f32)
                            .text(&task.message)
                            .animate(true),
                    );
                    if ui.button("取消").clicked() {
                        task.stop.store(true, Ordering::Release);
                    }
                }
                if let Some((_, outcome)) = &self.setup.outcome {
                    ui.add_space(6.0);
                    match outcome {
                        Ok(text) => ui.colored_label(ACCENT, format!("✔ {text}")),
                        Err(text) => ui.colored_label(ALERT, format!("✖ {text}")),
                    };
                }
            });
        if let Some(kind) = start {
            self.start_task(kind);
        }
    }
    fn status_card(&self, ui: &mut egui::Ui) {
        let (color, title) = self.mode_style();
        let s = &self.status;
        let live = self.running() && !self.stale() && s.frame_id != 0;
        egui::Frame::new()
            .fill(PANEL)
            .stroke(Stroke::new(2.0_f32, color))
            .inner_margin(20)
            .corner_radius(14)
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(RichText::new(title).size(28.0).strong().color(color));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if self.running() {
                            ui.label(
                                RichText::new(if s.hud_visible {
                                    "● 游戏内 HUD 已显示"
                                } else {
                                    "○ 游戏内 HUD 未显示"
                                })
                                .color(if s.hud_visible {
                                    ACCENT
                                } else {
                                    MUTED
                                }),
                            );
                            if s.engagements > 0 {
                                ui.label(
                                    RichText::new(format!("本次已启用 {} 次", s.engagements))
                                        .color(MUTED),
                                );
                            }
                        }
                    });
                });
                let reason = if !self.running() {
                    "点击下方「开启自动驾驶」，然后切回游戏。"
                } else {
                    s.reason.as_str()
                };
                ui.label(RichText::new(reason).size(17.0));
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    let dash = |v: f64, live: bool| {
                        if live {
                            format!("{v:.0}")
                        } else {
                            "—".into()
                        }
                    };
                    big_value(ui, "当前车速", &dash(s.speed_kmh.abs(), live), "km/h");
                    let engaged = matches!(
                        s.mode,
                        Mode::Engaged | Mode::TakeoverSoon | Mode::SafetyBrake | Mode::Stopping
                    );
                    big_value(ui, "目标车速", &dash(s.target_kmh, live && engaged), "km/h");
                    big_value(
                        ui,
                        "道路限速",
                        &if !live {
                            "—".into()
                        } else if s.limit_kmh < 0.5 {
                            "未显示".into()
                        } else {
                            format!("{:.0}", s.limit_kmh)
                        },
                        if live && s.limit_kmh < 0.5 {
                            ""
                        } else {
                            "km/h"
                        },
                    );
                    big_value(
                        ui,
                        "导航可用",
                        &match s.remaining_route_m.filter(|_| live) {
                            Some(m) if m >= 1000.0 => format!("{:.1}", m / 1000.0),
                            Some(m) => format!("{m:.0}"),
                            None => "—".into(),
                        },
                        if s.remaining_route_m.is_some_and(|m| m >= 1000.0) {
                            "km"
                        } else {
                            "m"
                        },
                    );
                });
                if live {
                    let mut line = match s.lead {
                        Some(l) => format!(
                            "前车 {:.0} m（{}）",
                            l.gap_m,
                            match l.motion {
                                Motion::SameDirection => "同向",
                                Motion::Opposing => "对向",
                                Motion::Crossing => "横穿",
                            }
                        ),
                        None => "前方无车辆约束".into(),
                    };
                    line.push_str(&format!(" · 车道偏差 {:+.2} m", s.lateral_m));
                    if matches!(s.longitudinal, Some(Longitudinal::Holding { .. })) {
                        line.push_str(" · 停稳保持制动");
                    }
                    if s.remaining_s.is_finite() && s.remaining_s > 0.0 {
                        line.push_str(&format!(" · 本次剩余 {:.0} s", s.remaining_s));
                    }
                    ui.add_space(6.0);
                    ui.label(RichText::new(line).color(MUTED));
                }
            });
    }
    fn primary_action(
        &mut self,
        ui: &mut egui::Ui,
        other_busy: bool,
        closing: bool,
        connected: bool,
    ) {
        ui.horizontal(|ui| {
            if self.running() {
                if ui
                    .add(
                        egui::Button::new(RichText::new("关闭自动驾驶").size(18.0).strong())
                            .fill(Color32::from_rgb(120, 46, 50))
                            .min_size(egui::vec2(220.0, 52.0)),
                    )
                    .clicked()
                {
                    self.request_stop();
                    let _ = etsads::set_gate(false);
                }
                ui.label(
                    RichText::new("关闭后不再接收热键；游戏中按 F8 只退出本次驾驶。").color(MUTED),
                );
            } else {
                let map_missing = self.setup.report.as_ref().is_some_and(|r| !r.map_ready());
                let task = self.setup.task.is_some();
                let enabled = !other_busy && !closing && !map_missing && !task;
                if ui
                    .add_enabled(
                        enabled,
                        egui::Button::new(
                            RichText::new("开启自动驾驶")
                                .size(18.0)
                                .strong()
                                .color(super::BG),
                        )
                        .fill(ACCENT)
                        .min_size(egui::vec2(220.0, 52.0)),
                    )
                    .clicked()
                {
                    self.start();
                }
                ui.label(
                    RichText::new(if other_busy {
                        "请先停止录制或其它驾驶运行。"
                    } else if task {
                        "请等待设置步骤完成。"
                    } else if map_missing {
                        "请先在上方完成首次设置（生成地图）。"
                    } else if !connected {
                        "可以先开启；游戏进入驾驶后会自动连接。"
                    } else {
                        "开启后切回游戏，HUD 显示「可以启用自动驾驶」时按 F8。"
                    })
                    .color(MUTED),
                );
            }
        });
    }
    fn checklist(&self, ui: &mut egui::Ui, connected: bool) {
        egui::Frame::new()
            .fill(PANEL)
            .inner_margin(18)
            .corner_radius(12)
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new("启用前检查").strong().size(18.0));
                ui.add_space(6.0);
                let s = &self.status;
                let live = self.running() && !self.stale() && s.frame_id != 0;
                let usable =
                    live && matches!(s.mode, Mode::Ready | Mode::Engaged | Mode::TakeoverSoon);
                check(ui, connected, "游戏已连接（单机驾驶画面）");
                check(ui, self.running(), "自动驾驶已开启（待命中）");
                check(ui, self.running() && s.hud_visible, "游戏内 HUD 正常显示");
                check(
                    ui,
                    usable || (live && s.remaining_route_m.is_some()),
                    "已设置 GPS 目的地，当前路段可用",
                );
                check(ui, usable, "车辆在车道中央，挡位和车速合适");
                ui.add_space(6.0);
                ui.label(
                    RichText::new(
                        "首次使用：发动机运行、前进挡、松开驻车，在简单道路中央以低速稳定行驶。",
                    )
                    .color(MUTED)
                    .size(13.0),
                );
            });
    }
    fn settings(&mut self, ui: &mut egui::Ui) {
        egui::Frame::new()
            .fill(PANEL)
            .inner_margin(18)
            .corner_radius(12)
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(RichText::new("驾驶设置").strong().size(18.0));
                    if self.running() {
                        ui.label(RichText::new("（关闭自动驾驶后可修改）").color(MUTED));
                    }
                });
                ui.add_space(6.0);
                ui.add_enabled_ui(!self.running(), |ui| {
                    ui.horizontal(|ui| {
                        ui.label("最高车速");
                        ui.add(
                            egui::Slider::new(&mut self.options.maximum_kmh, 10.0..=90.0)
                                .step_by(5.0)
                                .suffix(" km/h"),
                        );
                    });
                    ui.label(
                        RichText::new("实际车速取道路限速、弯道、前车和这里的最低值；没有限速牌时按这里巡航。")
                            .color(MUTED)
                            .size(13.0),
                    );
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("单次时长");
                        ui.radio_value(&mut self.unlimited, true, "不限时");
                        ui.radio_value(&mut self.unlimited, false, "限时");
                        if !self.unlimited {
                            ui.add(
                                egui::Slider::new(
                                    &mut self.options.seconds,
                                    5..=etsads::assist::MAX_SECONDS,
                                )
                                .suffix(" 秒"),
                            );
                        }
                    });
                    ui.checkbox(
                        &mut self.options.stop_at_end,
                        "限时结束时先减速停车并拉驻车，再交还人工",
                    );
                    ui.collapsing("高级设置", |ui| {
                        ui.label("分区地图目录");
                        ui.text_edit_singleline(&mut self.map);
                        ui.label("额外转向报告（开发者用；一般由“标定转向”按车辆自动管理）");
                        ui.text_edit_singleline(&mut self.steering);
                        ui.label(
                            RichText::new("诊断记录写入 runs/assist：单次最多 8 MiB、合计 64 MiB，超出时自动删除最旧的记录。")
                                .color(MUTED)
                                .size(13.0),
                        );
                    });
                });
            });
    }
}
fn step_row(
    ui: &mut egui::Ui,
    ok: bool,
    title: &str,
    text: &str,
    action: impl FnOnce(&mut egui::Ui),
) {
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(if ok { "✔" } else { "●" })
                .color(if ok { ACCENT } else { AMBER })
                .strong(),
        );
        ui.add_sized(
            [130.0, 22.0],
            egui::Label::new(RichText::new(title).strong()),
        );
        ui.label(RichText::new(text).color(if ok {
            MUTED
        } else {
            Color32::from_rgb(226, 232, 240)
        }));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), action);
    });
}
fn big_value(ui: &mut egui::Ui, label: &str, value: &str, unit: &str) {
    ui.vertical(|ui| {
        ui.set_min_width(150.0);
        ui.label(RichText::new(label).color(MUTED));
        ui.horizontal(|ui| {
            ui.label(RichText::new(value).size(34.0).strong());
            ui.label(RichText::new(unit).color(MUTED));
        });
    });
}
fn check(ui: &mut egui::Ui, ok: bool, text: &str) {
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(if ok { "✔" } else { "○" })
                .color(if ok { ACCENT } else { MUTED })
                .strong(),
        );
        ui.label(RichText::new(text).color(if ok {
            Color32::from_rgb(226, 232, 240)
        } else {
            MUTED
        }));
    });
}
fn key(ui: &mut egui::Ui, text: &str) {
    egui::Frame::new()
        .fill(Color32::from_rgb(40, 54, 70))
        .stroke(Stroke::new(1.0_f32, Color32::from_rgb(78, 96, 116)))
        .inner_margin(egui::Margin::symmetric(8, 3))
        .corner_radius(5)
        .show(ui, |ui| {
            ui.label(RichText::new(text).monospace().strong());
        });
}
fn keys_card(ui: &mut egui::Ui) {
    egui::Frame::new()
        .fill(PANEL)
        .inner_margin(18)
        .corner_radius(12)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new("在游戏中操作").strong().size(18.0));
            ui.add_space(6.0);
            for (label, keys) in [
                ("启用", &["F8", "手柄 L3+R3"][..]),
                ("退出", &["F8", "手柄 L3+R3"][..]),
                (
                    "立即接管",
                    &["方向键", "W A S D", "空格", "手柄摇杆/扳机"][..],
                ),
            ] {
                ui.horizontal(|ui| {
                    ui.add_sized(
                        [72.0, 24.0],
                        egui::Label::new(RichText::new(label).color(MUTED)),
                    );
                    for (i, k) in keys.iter().enumerate() {
                        if i > 0 {
                            ui.label(RichText::new("/").color(MUTED));
                        }
                        key(ui, k);
                    }
                });
            }
            ui.add_space(6.0);
            ui.label(
                RichText::new(
                    "接管后会回到待命，再按热键即可重新启用；不会自行重新接管。游戏须保持在前台。",
                )
                .color(MUTED)
                .size(13.0),
            );
        });
}
