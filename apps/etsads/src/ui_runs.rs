//! Local recording browser. File reading and MCAP analysis stay off the UI thread.
use eframe::egui::{self, RichText};
use etsads_metrics::RunMetrics;
use etsads_recorder::RunMetadata;
use std::{path::PathBuf, sync::mpsc};

struct Entry {
    path: PathBuf,
    metadata: RunMetadata,
}

pub struct Recordings {
    root: String,
    entries: Vec<Entry>,
    selected: Option<PathBuf>,
    scan: Option<mpsc::Receiver<Result<Vec<Entry>, String>>>,
    analysis: Option<mpsc::Receiver<Result<RunMetrics, String>>>,
    report: Option<RunMetrics>,
    error: Option<String>,
    initialized: bool,
}

impl Default for Recordings {
    fn default() -> Self {
        Self {
            root: "runs".into(),
            entries: vec![],
            selected: None,
            scan: None,
            analysis: None,
            report: None,
            error: None,
            initialized: false,
        }
    }
}

impl Recordings {
    fn refresh(&mut self) {
        if self.scan.is_some() {
            return;
        }
        self.error = None;
        let root = PathBuf::from(&self.root);
        let (tx, rx) = mpsc::channel();
        self.scan = Some(rx);
        std::thread::spawn(move || {
            let result = (|| -> anyhow::Result<Vec<Entry>> {
                if !root.exists() {
                    return Ok(vec![]);
                }
                let mut entries = vec![];
                for directory in std::fs::read_dir(root)? {
                    let path = directory?.path();
                    let metadata_file = path.join("meta.json");
                    // Ignore unrelated folders and large/malformed metadata.
                    if !metadata_file.is_file()
                        || std::fs::metadata(&metadata_file)?.len() > 1_048_576
                    {
                        continue;
                    }
                    if let Ok(metadata) =
                        serde_json::from_slice::<RunMetadata>(&std::fs::read(metadata_file)?)
                    {
                        entries.push(Entry { path, metadata });
                    }
                }
                entries.sort_by_key(|entry| std::cmp::Reverse(entry.metadata.started_unix_s));
                entries.truncate(200);
                Ok(entries)
            })();
            let _ = tx.send(result.map_err(|e| format!("无法读取运行目录：{e:#}")));
        });
    }

    fn poll(&mut self) {
        if let Some(result) = self.scan.as_ref().and_then(|rx| match rx.try_recv() {
            Ok(value) => Some(value),
            Err(mpsc::TryRecvError::Disconnected) => Some(Err("目录读取线程已结束".into())),
            Err(mpsc::TryRecvError::Empty) => None,
        }) {
            self.scan = None;
            match result {
                Ok(entries) => self.entries = entries,
                Err(error) => self.error = Some(error),
            }
        }
        if let Some(result) = self.analysis.as_ref().and_then(|rx| match rx.try_recv() {
            Ok(value) => Some(value),
            Err(mpsc::TryRecvError::Disconnected) => Some(Err("分析线程已结束".into())),
            Err(mpsc::TryRecvError::Empty) => None,
        }) {
            self.analysis = None;
            match result {
                Ok(report) => self.report = Some(report),
                Err(error) => self.error = Some(error),
            }
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        if !self.initialized {
            self.initialized = true;
            self.refresh();
        }
        self.poll();
        ui.heading("录制与回放");
        ui.label("选择本地记录，一键检查帧连续性、视频对齐和运行指标。");
        ui.add_space(12.0);
        ui.horizontal(|ui| {
            ui.label("运行目录");
            ui.add_enabled(
                self.scan.is_none() && self.analysis.is_none(),
                egui::TextEdit::singleline(&mut self.root).desired_width(400.0),
            );
            if ui
                .add_enabled(
                    self.scan.is_none() && self.analysis.is_none(),
                    egui::Button::new("刷新列表"),
                )
                .clicked()
            {
                self.refresh();
            }
        });
        if self.scan.is_some() {
            ui.spinner();
            ui.label("正在读取运行目录…");
        }
        if let Some(error) = &self.error {
            ui.colored_label(super::AMBER, error);
        }
        if self.entries.is_empty() && self.scan.is_none() {
            ui.add_space(20.0);
            ui.label("这里还没有运行记录。连接游戏后，在驾驶面板点击“开始录制”。");
        }
        let mut analyze = None;
        egui::ScrollArea::vertical()
            .id_salt("recording_list")
            .max_height(260.0)
            .show(ui, |ui| {
                egui::Grid::new("recording_grid")
                    .striped(true)
                    .min_col_width(105.0)
                    .show(ui, |ui| {
                        for title in ["运行名称", "数据来源", "保存状态", "消息数", "操作"]
                        {
                            ui.label(RichText::new(title).strong());
                        }
                        ui.end_row();
                        for entry in &self.entries {
                            let name = entry.path.file_name().unwrap_or_default().to_string_lossy();
                            ui.label(name.as_ref())
                                .on_hover_text(entry.path.display().to_string());
                            ui.label(if entry.metadata.source == "game" {
                                "真实游戏"
                            } else {
                                "合成 / 离线"
                            });
                            ui.colored_label(
                                if entry.metadata.completed {
                                    super::ACCENT
                                } else {
                                    super::AMBER
                                },
                                if entry.metadata.completed {
                                    "已保存"
                                } else {
                                    "未完整保存"
                                },
                            );
                            ui.label(entry.metadata.messages.to_string());
                            if ui
                                .add_enabled(
                                    self.analysis.is_none(),
                                    egui::Button::new("检查并分析"),
                                )
                                .clicked()
                            {
                                analyze = Some(entry.path.clone());
                            }
                            ui.end_row();
                        }
                    });
            });
        if let Some(path) = analyze {
            self.selected = Some(path.clone());
            self.report = None;
            self.error = None;
            let (tx, rx) = mpsc::channel();
            self.analysis = Some(rx);
            std::thread::spawn(move || {
                let _ = tx.send(
                    etsads_metrics::analyze(&path)
                        .map_err(|e| format!("无法分析（正在写入的记录请先停止保存）：{e:#}")),
                );
            });
        }
        ui.add_space(16.0);
        if let Some(path) = &self.selected {
            ui.label(RichText::new(path.display().to_string()).strong());
        }
        if self.analysis.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("正在逐帧检查 MCAP，大文件可能需要一些时间…");
            });
        }
        if let Some(report) = &self.report {
            let integrity = &report.integrity;
            let good = integrity.completed
                && integrity.frame_gaps == 0
                && integrity.nonmonotonic == 0
                && integrity.telemetry_frames > 0;
            ui.colored_label(
                if good { super::ACCENT } else { super::AMBER },
                if good {
                    "数据完整性检查通过"
                } else {
                    "记录存在不完整或异常数据"
                },
            );
            ui.label("数据完整性通过仅说明记录可读，不代表驾驶阶段验收通过。");
            egui::Grid::new("recording_metrics")
                .spacing([30.0, 10.0])
                .show(ui, |ui| {
                    for (name, value) in [
                        ("物理帧数", integrity.telemetry_frames.to_string()),
                        (
                            "缺失帧 / 逆序",
                            format!("{} / {}", integrity.frame_gaps, integrity.nonmonotonic),
                        ),
                        (
                            "有效模拟时间",
                            format!("{:.1} 秒", report.active_simulation_seconds),
                        ),
                        (
                            "物理世界行驶距离",
                            format!("{:.3} km", report.physical_world_km),
                        ),
                        (
                            "视频帧 / 捕获丢帧",
                            format!("{} / {}", report.video_frames, report.dropped_video_frames),
                        ),
                        (
                            "视频对齐误差 P95",
                            report
                                .video_alignment_error_ms
                                .p95
                                .map_or("无数据".into(), |v| format!("{v:.2} ms")),
                        ),
                        (
                            "视频逐帧双向对齐",
                            match report.every_video_frame_bracketed {
                                Some(true) => "全部通过",
                                Some(false) => "存在未对齐帧",
                                None => "缺少证据",
                            }
                            .into(),
                        ),
                        (
                            "行驶中横向偏差 RMS",
                            report
                                .moving_lateral_error_rms_m
                                .map_or("缺少车道观测".into(), |v| format!("{v:.3} m")),
                        ),
                        (
                            "实测跟车时距 P05 / P95",
                            match (
                                report.verified_following_headway_seconds.p05,
                                report.verified_following_headway_seconds.p95,
                            ) {
                                (Some(low), Some(high)) => format!(
                                    "{low:.2} / {high:.2} s（{} 个样本）",
                                    report.verified_following_headway_seconds.samples
                                ),
                                _ => "缺少有效的前车/车身尺寸证据".into(),
                            },
                        ),
                        (
                            "观测到的罚单",
                            format!("{}（需另行核实游戏罚单开关）", report.observed_fines),
                        ),
                    ] {
                        ui.label(name);
                        ui.label(value);
                        ui.end_row();
                    }
                });
            ui.collapsing("完整分析结果（可复制）", |ui| {
                let mut json = serde_json::to_string_pretty(report).unwrap_or_default();
                ui.add(
                    egui::TextEdit::multiline(&mut json)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(f32::INFINITY)
                        .desired_rows(15),
                );
            });
        }
    }
}
