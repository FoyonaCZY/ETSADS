#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
mod ui_assist;
mod ui_drive;
mod ui_runs;
mod ui_sim;
use eframe::egui::{self, Color32, RichText, Stroke, Vec2};
use etsads_msgs::protocol::*;
use etsads_shm::{SharedCell, SharedRing};
use std::{
    collections::VecDeque,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

const BG: Color32 = Color32::from_rgb(13, 20, 29);
const PANEL: Color32 = Color32::from_rgb(22, 32, 44);
const ACCENT: Color32 = Color32::from_rgb(64, 221, 184);
const MUTED: Color32 = Color32::from_rgb(147, 165, 185);
const AMBER: Color32 = Color32::from_rgb(245, 184, 87);
struct Dashboard {
    lab: ui_sim::Laboratory,
    recordings: ui_runs::Recordings,
    drive: ui_drive::DrivePanel,
    assist: ui_assist::AssistPanel,
    page: usize,
    telemetry: Option<SharedRing<TelemetryFrame, TELEMETRY_SLOTS>>,
    latest: Option<TelemetryFrame>,
    next_connect: Instant,
    history: VecDeque<[f64; 2]>,
    last_frame: u64,
    logs: VecDeque<String>,
    game_dir: String,
    portable_install: bool,
    recording: Option<Arc<AtomicBool>>,
    record_result: Option<mpsc::Receiver<String>>,
    record_path: String,
    supervisor: Option<std::process::Child>,
    supervisor_messages: Option<mpsc::Receiver<String>>,
    gate: Option<SharedCell<Gate>>,
    controls_status: etsads::environment::ControlStatus,
    closing: bool,
    record_video: bool,
}
impl Dashboard {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.set_theme(egui::Theme::Dark);
        let mut visuals = egui::Visuals::dark();
        visuals.panel_fill = BG;
        visuals.window_fill = PANEL;
        visuals.widgets.noninteractive.bg_fill = PANEL;
        visuals.widgets.inactive.bg_fill = Color32::from_rgb(31, 45, 60);
        visuals.selection.bg_fill = Color32::from_rgb(29, 104, 93);
        cc.egui_ctx.set_visuals(visuals);
        let mut style = (*cc.egui_ctx.style()).clone();
        style.spacing.item_spacing = Vec2::new(12.0, 12.0);
        style.spacing.button_padding = Vec2::new(16.0, 10.0);
        style
            .text_styles
            .insert(egui::TextStyle::Body, egui::FontId::proportional(16.0));
        style
            .text_styles
            .insert(egui::TextStyle::Button, egui::FontId::proportional(16.0));
        cc.egui_ctx.set_style(style);
        // The redistributable font keeps Chinese labels readable even when the
        // OS has no Chinese language pack. Never bundle the system's fonts.
        let font = std::env::var_os("ETSADS_FONT")
            .and_then(|path| std::fs::read(path).ok())
            .map(egui::FontData::from_owned)
            .unwrap_or_else(|| {
                egui::FontData::from_static(include_bytes!(
                    "../../../vendor/fonts/NotoSansSC-Regular.otf"
                ))
            });
        let mut fonts = egui::FontDefinitions::default();
        fonts.font_data.insert("cjk".into(), font.into());
        for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
            fonts.families.entry(family).or_default().push("cjk".into());
        }
        cc.egui_ctx.set_fonts(fonts);
        Self {
            lab: ui_sim::Laboratory::default(),
            recordings: ui_runs::Recordings::default(),
            drive: ui_drive::DrivePanel::default(),
            assist: ui_assist::AssistPanel::default(),
            page: 7,
            telemetry: None,
            latest: None,
            next_connect: Instant::now(),
            history: VecDeque::new(),
            last_frame: 0,
            logs: VecDeque::from(["欢迎使用 ETSADS。安全监督默认关闭控制。".into()]),
            game_dir: etsads::find_game()
                .map_or("未找到，请设置 ETSADS_GAME_DIR".into(), |p| {
                    p.display().to_string()
                }),
            portable_install: std::env::current_exe()
                .ok()
                .is_some_and(|path| path.with_file_name("install-plugin.ps1").is_file()),
            recording: None,
            record_result: None,
            record_path: String::new(),
            supervisor: None,
            supervisor_messages: None,
            gate: None,
            controls_status: etsads::environment::control_status(),
            closing: false,
            record_video: true,
        }
    }
    fn log(&mut self, message: impl Into<String>) {
        self.logs.push_front(message.into());
        self.logs.truncate(100);
    }
    fn poll(&mut self) {
        if let Some(message) = self.assist.poll() {
            self.log(message);
        }
        if let Some(message) = self.drive.poll() {
            self.log(message);
        }
        let messages = self
            .supervisor_messages
            .as_ref()
            .map(|rx| rx.try_iter().take(16).collect::<Vec<_>>())
            .unwrap_or_default();
        for message in messages {
            self.log(format!("安全监督：{message}"));
        }
        if let Some(status) = self
            .supervisor
            .as_mut()
            .and_then(|child| child.try_wait().ok().flatten())
        {
            self.supervisor = None;
            self.log(format!("安全监督已退出（{status}）"));
        }
        if self.next_connect <= Instant::now() {
            self.next_connect = Instant::now() + Duration::from_secs(2);
            self.controls_status = etsads::environment::control_status();
            if self.telemetry.is_none() {
                self.telemetry = SharedRing::open(TELEMETRY_MAPPING, SCHEMA_VERSION).ok();
            }
            if self.gate.is_none() {
                self.gate = SharedCell::open(GATE_MAPPING, SCHEMA_VERSION).ok();
            }
        }
        if let Some(t) = self.telemetry.as_ref().and_then(|r| r.latest()) {
            if self.latest.is_some_and(|last| {
                t.header.frame_id < last.header.frame_id
                    || t.header.sim_time_us < last.header.sim_time_us
            }) {
                self.history.clear();
                self.last_frame = 0;
            }
            self.latest = Some(t);
            if t.header.frame_id != self.last_frame {
                self.last_frame = t.header.frame_id;
                self.history
                    .push_back([t.header.sim_time_us as f64 / 1e6, t.speed_mps as f64 * 3.6]);
                if self.history.len() > 600 {
                    self.history.pop_front();
                }
            }
        }
        let result = self
            .record_result
            .as_ref()
            .and_then(|r| match r.try_recv() {
                Ok(s) => Some(s),
                Err(mpsc::TryRecvError::Disconnected) => {
                    Some("录制进程意外结束；本次记录不完整".into())
                }
                Err(mpsc::TryRecvError::Empty) => None,
            });
        if let Some(result) = result {
            self.record_result = None;
            self.recording = None;
            self.log(result);
        }
    }
    fn connected(&self) -> bool {
        self.latest.is_some_and(|t| {
            etsads_safety::fresh(
                etsads_shm::qpc(),
                t.header.qpc,
                etsads_shm::qpc_frequency(),
                500,
            )
        })
    }
    fn start_record(&mut self) {
        if self.drive.busy() || self.assist.busy() || self.closing {
            self.log("驾驶运行正在录制，不能同时启动独立录制。");
            return;
        }
        let dir = std::path::PathBuf::from(format!(
            "runs/game-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        ));
        self.record_path = dir.display().to_string();
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let (tx, rx) = mpsc::channel();
        let video = self.record_video;
        std::thread::spawn(move || {
            let result = etsads::record_game_with_video(&dir, 86400, &worker_stop, video);
            let message = match result {
                Ok(m) => format!(
                    "录制已保存：{}（{} 条消息，丢帧 {}）",
                    dir.display(),
                    m.messages,
                    m.dropped_frames
                ),
                Err(e) => format!("录制失败：{e:#}"),
            };
            let _ = tx.send(message);
        });
        self.recording = Some(stop);
        self.record_result = Some(rx);
        self.log("录制开始：真实游戏数据 → MCAP");
    }
    fn overview(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("驾驶工作台").size(30.0).strong());
        ui.label(RichText::new("实时状态、独立安全监督与可追溯的运行记录").color(MUTED));
        if ui.button("配置运行与目标速度").clicked() {
            self.page = 6;
        }
        ui.add_space(14.0);
        let t = self.latest.unwrap_or_default();
        let current = self.connected();
        ui.columns(4, |columns| {
            metric(
                &mut columns[0],
                "当前速度",
                &if current && t.speed_mps.is_finite() {
                    format!("{:.1}", t.speed_mps * 3.6)
                } else {
                    "—".into()
                },
                "km/h",
            );
            metric(
                &mut columns[1],
                "游戏限速",
                &if current
                    && t.navigation_speed_limit_mps.is_finite()
                    && t.navigation_speed_limit_mps > 0.0
                {
                    format!("{:.0}", t.navigation_speed_limit_mps * 3.6)
                } else {
                    "—".into()
                },
                "km/h",
            );
            metric(
                &mut columns[2],
                "物理帧",
                &if current {
                    t.header.frame_id.to_string()
                } else {
                    "—".into()
                },
                "frame",
            );
            metric(
                &mut columns[3],
                "挡位",
                &if current {
                    t.gear.to_string()
                } else {
                    "—".into()
                },
                if !self.connected() {
                    "未连接"
                } else if t.paused != 0 {
                    "已暂停"
                } else {
                    "运行中"
                },
            );
        });
        ui.add_space(14.0);
        egui::Frame::new()
            .fill(PANEL)
            .inner_margin(20)
            .corner_radius(12)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("速度时间线").strong());
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new("模拟时间 · 最近 600 次采样").color(MUTED));
                    });
                });
                let (rect, _) = ui.allocate_exact_size(
                    Vec2::new(ui.available_width(), 160.0),
                    egui::Sense::hover(),
                );
                let p = ui.painter();
                for n in 0..5 {
                    let y = rect.bottom() - rect.height() * n as f32 / 4.0;
                    p.line_segment(
                        [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                        Stroke::new(1.0_f32, Color32::from_rgb(37, 51, 67)),
                    );
                }
                if self.history.len() > 1 {
                    let first = self.history.front().unwrap()[0];
                    let range = (self.history.back().unwrap()[0] - first).max(1.0);
                    let pts = self
                        .history
                        .iter()
                        .map(|v| {
                            egui::pos2(
                                rect.left() + ((v[0] - first) / range) as f32 * rect.width(),
                                rect.bottom()
                                    - (v[1] / 120.0).clamp(0.0, 1.0) as f32 * rect.height(),
                            )
                        })
                        .collect();
                    p.add(egui::Shape::line(pts, Stroke::new(2.5_f32, ACCENT)));
                } else {
                    p.text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        "等待游戏遥测",
                        egui::FontId::proportional(19.0),
                        MUTED,
                    );
                }
            });
        ui.add_space(14.0);
        ui.columns(2, |columns| {
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(20)
                .corner_radius(12)
                .show(&mut columns[0], |ui| {
                    ui.heading("安全监督");
                    ui.colored_label(
                        if self.controls_status.mode_compatible {
                            ACCENT
                        } else {
                            AMBER
                        },
                        self.controls_status.label,
                    );
                    ui.label(self.controls_status.message);
                    let gate_sample = self.gate.as_ref().and_then(|g| g.latest()).filter(|g| {
                        etsads_safety::fresh(
                            etsads_shm::qpc(),
                            g.heartbeat_qpc,
                            etsads_shm::qpc_frequency(),
                            200,
                        )
                    });
                    let gate_mode = gate_sample.map_or(0, |g| g.enabled);
                    ui.colored_label(
                        if gate_mode == 1 { ACCENT } else { AMBER },
                        match gate_mode {
                            1 => "控制闸门已开启",
                            2 => "失联缓刹中 · 主控制已撤销",
                            _ => "控制已释放",
                        },
                    );
                    let supervisor_running = self
                        .supervisor
                        .as_mut()
                        .is_some_and(|child| !matches!(child.try_wait(), Ok(Some(_))))
                        || gate_sample.is_some();
                    if ui
                        .add_enabled(
                            !supervisor_running && !self.closing,
                            egui::Button::new(if supervisor_running {
                                "安全监督正在运行"
                            } else {
                                "启动独立安全监督"
                            }),
                        )
                        .clicked()
                    {
                        let exe = std::env::current_exe()
                            .unwrap_or_default()
                            .with_file_name("etsads-supervisor.exe");
                        let mut command = std::process::Command::new(exe);
                        command
                            .stdout(std::process::Stdio::null())
                            .stderr(std::process::Stdio::piped());
                        match self.drive.supervisor_profile() {
                            Ok(Some(profile)) => {
                                command.arg("--safety-profile").arg(profile);
                            }
                            Ok(None) => {}
                            Err(error) => {
                                self.log(format!("安全几何配置无效：{error:#}"));
                                return;
                            }
                        }
                        #[cfg(windows)]
                        {
                            use std::os::windows::process::CommandExt;
                            command.creation_flags(0x08000000);
                        }
                        match command.spawn() {
                            Ok(mut child) => {
                                if let Some(stderr) = child.stderr.take() {
                                    let (tx, rx) = mpsc::sync_channel(64);
                                    self.supervisor_messages = Some(rx);
                                    std::thread::spawn(move || {
                                        use std::io::BufRead;
                                        for line in std::io::BufReader::new(stderr).lines() {
                                            match line {
                                                Ok(line) => {
                                                    let _ = tx.try_send(line);
                                                }
                                                Err(_) => break,
                                            }
                                        }
                                    });
                                }
                                self.supervisor = Some(child);
                                self.log("安全监督已启动。F8 切换；Esc 释放。");
                            }
                            Err(e) => self.log(format!("无法启动安全监督：{e}")),
                        }
                    }
                    if ui
                        .add(egui::Button::new("立即释放控制").fill(Color32::from_rgb(109, 45, 48)))
                        .clicked()
                    {
                        self.drive.request_stop();
                        let r = etsads::set_gate(false);
                        self.log(match r {
                            Ok(()) => "已请求释放控制".into(),
                            Err(e) => format!("{e:#}"),
                        });
                    }
                    ui.label(
                        RichText::new("暂停、数据过期或非法命令会锁定闸门，需显式重新启用。")
                            .size(13.0)
                            .color(MUTED),
                    );
                });
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(20)
                .corner_radius(12)
                .show(&mut columns[1], |ui| {
                    ui.heading("运行记录");
                    ui.add_enabled(
                        self.recording.is_none() && !self.drive.busy() && !self.closing,
                        egui::Checkbox::new(&mut self.record_video, "同时录制游戏画面"),
                    );
                    ui.label(if self.recording.is_some() {
                        "● 正在录制真实游戏"
                    } else {
                        "尚未录制"
                    });
                    if self.recording.is_none() {
                        if ui
                            .add_enabled(
                                self.connected() && !self.drive.busy() && !self.closing,
                                egui::Button::new("开始录制").fill(Color32::from_rgb(28, 99, 86)),
                            )
                            .clicked()
                        {
                            self.start_record();
                        }
                    } else if ui.button("停止并保存").clicked()
                        && let Some(s) = &self.recording
                    {
                        s.store(true, Ordering::Relaxed);
                    }
                    ui.label(
                        RichText::new("MCAP / Protobuf · 帧缺失统计 · 原始数据可回放")
                            .size(13.0)
                            .color(MUTED),
                    );
                    ui.label(&self.record_path);
                    if self.drive.busy() {
                        ui.colored_label(AMBER, "驾驶运行正在录制；独立录制已禁用。");
                    }
                });
        });
    }
    fn acceptance(&mut self, ui: &mut egui::Ui) {
        ui.heading("验收中心");
        ui.colored_label(AMBER, "阶段通过仅依据真实录制证据；离线仿真独立记账。");
        ui.add_space(16.0);
        egui::Grid::new("acceptance")
            .spacing([20.0, 20.0])
            .striped(true)
            .show(ui, |ui| {
                for (stage, goal) in [
                    (
                        "P0 · 基础设施",
                        "E1–E9；1 小时录制回放；双视角叠加；跟圆误差 < 0.5 m",
                    ),
                    ("P1 · 高速 L1", "连续 100 km 无接管；横向 RMS < 0.3 m"),
                    ("P2 · 高速 L2", "跨城高速含匝道/收费站；无接管、零罚单"),
                    ("P3 · 全路网", "城市至城市；每 100 km 罚单 < 1"),
                    ("P4 · 困难泊车", "至少 50 次实测；成功率 ≥ 90%"),
                ] {
                    ui.label(RichText::new(stage).strong());
                    ui.label(goal);
                    ui.colored_label(AMBER, "待验收");
                    ui.end_row();
                }
            });
        ui.add_space(20.0);
        ui.label("详细证据：docs/ACCEPTANCE.md 与每次 runs/ 目录。当前软件仍处于开发验证阶段。");
    }
}
fn metric(ui: &mut egui::Ui, title: &str, value: &str, unit: &str) {
    egui::Frame::new()
        .fill(PANEL)
        .inner_margin(20)
        .corner_radius(12)
        .show(ui, |ui| {
            ui.set_min_width((ui.available_width() - 8.0).max(20.0));
            ui.label(RichText::new(title).color(MUTED));
            ui.label(RichText::new(value).size(34.0).strong().color(ACCENT));
            ui.label(RichText::new(unit).size(13.0).color(MUTED));
        });
}
impl eframe::App for Dashboard {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        self.poll();
        if ctx.input(|i| i.viewport().close_requested())
            && (self.recording.is_some() || self.drive.busy() || self.assist.busy())
        {
            self.closing = true;
            if let Some(s) = &self.recording {
                s.store(true, Ordering::Relaxed);
            }
            self.drive.request_stop();
            self.assist.request_stop();
            let _ = etsads::set_gate(false);
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.log("正在保存录制，完成后关闭窗口…");
        }
        if self.closing && self.recording.is_none() && !self.drive.busy() && !self.assist.busy() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        ctx.request_repaint_after(Duration::from_millis(100));
        egui::TopBottomPanel::top("header")
            .exact_height(65.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.add_space(16.0);
                    ui.label(RichText::new("ETSADS").strong().size(24.0).color(ACCENT));
                    ui.label(RichText::new("驾驶开发平台").color(MUTED));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.add_space(16.0);
                        if ui
                            .add(egui::Button::new("释放控制").fill(Color32::from_rgb(109, 45, 48)))
                            .clicked()
                        {
                            self.drive.request_stop();
                            self.assist.request_stop();
                            let result = etsads::set_gate(false);
                            self.log(match result {
                                Ok(()) => "已请求释放控制".into(),
                                Err(e) => format!("{e:#}"),
                            });
                        }
                        ui.colored_label(
                            if self.connected() { ACCENT } else { AMBER },
                            if self.connected() {
                                "● 游戏已连接"
                            } else {
                                "○ 等待游戏"
                            },
                        );
                        ui.label(RichText::new("v0.1 · DEVELOPMENT").size(12.0).color(MUTED));
                    });
                });
            });
        egui::SidePanel::left("navigation")
            .exact_width(190.0)
            .resizable(false)
            .show(ctx, |ui| {
                ui.add_space(24.0);
                let entry =
                    |ui: &mut egui::Ui, page: &mut usize, i: usize, name: &str, size: f32| {
                        if ui
                            .selectable_label(*page == i, RichText::new(name).size(size))
                            .clicked()
                        {
                            *page = i;
                        }
                        ui.add_space(10.0);
                    };
                entry(ui, &mut self.page, 7, "自动驾驶", 19.0);
                ui.add_space(14.0);
                ui.label(RichText::new("开发者工具").color(MUTED).size(12.0));
                ui.add_space(6.0);
                for (i, name) in [
                    (0, "驾驶数据"),
                    (1, "录制与回放"),
                    (6, "运行与目标速度"),
                    (2, "验收中心"),
                    (4, "离线实验室"),
                    (3, "环境配置"),
                    (5, "运行日志"),
                ] {
                    entry(ui, &mut self.page, i, name, 15.0);
                }
                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    ui.label(RichText::new("单机模式 · 本地数据").color(MUTED).size(12.0));
                });
            });
        egui::TopBottomPanel::bottom("footer")
            .exact_height(32.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(
                        RichText::new(self.logs.front().map_or("就绪", String::as_str))
                            .size(13.0)
                            .color(MUTED),
                    );
                });
            });
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().id_salt(("page",self.page)).show(ui, |ui| {
                ui.add_space(20.0);
                match self.page {
                    0 => self.overview(ui),
                    1 => self.recordings.show(ui),
                    2 => self.acceptance(ui),
                    4 => self.lab.show(ui),
                    5 => {
                        ui.heading("运行日志");
                        ui.label("最近100条本地状态，最新消息在上方。");
                        ui.separator();
                        for message in &self.logs { ui.label(message); }
                    },
                    6 => self.drive.show(ui,self.recording.is_some() || self.assist.busy(),self.closing),
                    7 => {
                        let connected = self.connected();
                        self.assist.show(ui, self.recording.is_some() || self.drive.busy(), self.closing, connected)
                    },
                    _ => {
                        ui.heading("环境配置");
                        ui.label("游戏安装目录");
                        ui.code(&self.game_dir);
                        ui.add_space(16.0);
                        if self.portable_install {
                            ui.label("1. 先退出游戏。包内“开始使用.txt”包含完整安装说明。");
                            ui.label("2. 在包目录运行 install-plugin.ps1，指定游戏目录；已有同名插件会先备份。");
                            ui.code(".\\install-plugin.ps1 -GameDir '<游戏目录>' -NativeGraphics");
                        } else {
                            ui.label("1. 运行 tools/prepare-game.ps1 备份存档并设置游戏。");
                            ui.label("2. 运行 tools/deploy.ps1 安装桥接 DLL。");
                        }
                        ui.label("3. 从 Steam 启动 DX11，选择存档进入单机驾驶。");
                        ui.label("4. 启动安全监督并开始录制。");
                        ui.add_space(16.0);
                        ui.colored_label(AMBER,"“运行与目标速度”页提供受证据门槛约束的开发试验；启用不代表P0～P4实车验收完成。");
                    }
                }
            });
        });
    }
}
impl Drop for Dashboard {
    fn drop(&mut self) {
        let _ = etsads::set_gate(false);
        self.drive.request_stop();
        self.assist.request_stop();
        if let Some(s) = &self.recording {
            s.store(true, Ordering::Relaxed);
        }
        if let Some(child) = &mut self.supervisor {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}
fn main() -> eframe::Result {
    eframe::run_native(
        "ETSADS — 驾驶开发平台",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1220.0, 840.0])
                .with_min_inner_size([1000.0, 720.0]),
            ..Default::default()
        },
        Box::new(|cc| Ok(Box::new(Dashboard::new(cc)))),
    )
}
