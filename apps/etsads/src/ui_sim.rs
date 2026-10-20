use super::{ACCENT, AMBER, MUTED, PANEL, metric};
use eframe::egui::{self, RichText, Stroke, Vec2};
use std::sync::mpsc;

#[derive(Default)]
pub struct Laboratory {
    report: Option<etsads_sim::SuiteReport>,
    pending: Option<mpsc::Receiver<Result<etsads_sim::SuiteReport, String>>>,
    selected: usize,
    time: f64,
    playing: bool,
    error: Option<String>,
}
impl Laboratory {
    pub fn show(&mut self, ui: &mut egui::Ui) {
        if let Some(result) = self.pending.as_ref().and_then(|r| r.try_recv().ok()) {
            self.pending = None;
            match result {
                Ok(r) => {
                    self.report = Some(r);
                    self.time = 0.0;
                    self.playing = true;
                }
                Err(e) => self.error = Some(e),
            }
        }
        ui.label(RichText::new("离线实验室").size(30.0).strong());
        ui.colored_label(AMBER, "合成场景 · 与真实游戏验收分开记录");
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.pending.is_none(),
                    egui::Button::new("运行闭环回归").fill(egui::Color32::from_rgb(28, 99, 86)),
                )
                .clicked()
            {
                let (tx, rx) = mpsc::channel();
                self.pending = Some(rx);
                self.error = None;
                std::thread::spawn(move || {
                    let _ = tx.send(etsads_sim::run_suite().map_err(|e| e.to_string()));
                });
            }
            if self.pending.is_some() {
                ui.spinner();
                ui.label("正在运行车辆、规划与控制…");
            }
        });
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
        let Some(report) = &self.report else {
            ui.add_space(32.0);
            ui.label("运行一次回归，即可查看弯道保持、前车停车与起步的完整轨迹和指标。");
            return;
        };
        ui.horizontal(|ui| {
            for (i, s) in report.scenarios.iter().enumerate() {
                if ui
                    .selectable_label(
                        self.selected == i,
                        if s.scenario.contains("acc") {
                            "跟车与停车起步"
                        } else {
                            "弯道车道保持"
                        },
                    )
                    .clicked()
                {
                    self.selected = i;
                    self.time = 0.0;
                }
            }
        });
        let Some(scenario) = report.scenarios.get(self.selected) else {
            return;
        };
        if self.playing {
            self.time += ui.ctx().input(|i| i.stable_dt.min(0.1)) as f64 * 4.0;
            if self.time >= scenario.duration_s {
                self.time = 0.0;
            }
        }
        ui.horizontal(|ui| {
            if ui
                .button(if self.playing {
                    "暂停回放"
                } else {
                    "播放回放"
                })
                .clicked()
            {
                self.playing = !self.playing;
            }
            ui.add(egui::Slider::new(&mut self.time, 0.0..=scenario.duration_s).text("模拟秒"));
            ui.label("4× 回放");
        });
        let index = scenario
            .samples
            .partition_point(|s| s.time_s <= self.time)
            .saturating_sub(1);
        if let Some(sample) = scenario.samples.get(index) {
            ui.columns(4, |columns| {
                metric(
                    &mut columns[0],
                    "当前车速",
                    &format!("{:.1}", sample.speed_mps * 3.6),
                    "km/h",
                );
                metric(
                    &mut columns[1],
                    "横向偏差",
                    &format!("{:.3}", sample.lateral_error_m),
                    "m",
                );
                metric(
                    &mut columns[2],
                    "全程 RMS",
                    &format!("{:.3}", scenario.rms_lateral_error_m),
                    "m",
                );
                metric(
                    &mut columns[3],
                    "碰撞",
                    &scenario.collision_count.to_string(),
                    "次",
                );
            });
            let (rect, _) = ui
                .allocate_exact_size(Vec2::new(ui.available_width(), 330.0), egui::Sense::hover());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 12.0, PANEL);
            let mut min = [f64::INFINITY; 2];
            let mut max = [f64::NEG_INFINITY; 2];
            for s in &scenario.samples {
                min[0] = min[0].min(s.ego.x);
                min[1] = min[1].min(s.ego.y);
                max[0] = max[0].max(s.ego.x);
                max[1] = max[1].max(s.ego.y);
            }
            let scale = ((rect.width() - 70.0) as f64 / (max[0] - min[0]).max(1.0))
                .min((rect.height() - 70.0) as f64 / (max[1] - min[1]).max(1.0));
            let point = |p: etsads_msgs::Pose2| {
                egui::pos2(
                    rect.center().x + ((p.x - (min[0] + max[0]) / 2.0) * scale) as f32,
                    rect.center().y - ((p.y - (min[1] + max[1]) / 2.0) * scale) as f32,
                )
            };
            painter.add(egui::Shape::line(
                scenario
                    .samples
                    .iter()
                    .map(|s| point(s.reference))
                    .collect(),
                Stroke::new(10.0_f32, egui::Color32::from_rgb(40, 56, 71)),
            ));
            painter.add(egui::Shape::line(
                scenario.samples[..=index]
                    .iter()
                    .map(|s| point(s.ego))
                    .collect(),
                Stroke::new(2.0_f32, ACCENT),
            ));
            let ego = point(sample.ego);
            let trailer = point(sample.trailer);
            painter.line_segment([ego, trailer], Stroke::new(3.0_f32, AMBER));
            painter.circle_filled(trailer, 4.0, AMBER);
            painter.circle_filled(ego, 6.0, ACCENT);
            painter.line_segment(
                [
                    ego,
                    ego + Vec2::new(sample.ego.yaw.cos() as f32, -sample.ego.yaw.sin() as f32)
                        * 18.0,
                ],
                Stroke::new(2.0_f32, ACCENT),
            );
            if let Some(lead) = sample.lead {
                painter.circle_filled(point(lead), 6.0, egui::Color32::from_rgb(112, 162, 255));
            }
            painter.text(
                rect.left_top() + Vec2::new(16.0, 16.0),
                egui::Align2::LEFT_TOP,
                "青色：牵引车    黄色：挂车    蓝色：前车",
                egui::FontId::proportional(14.0),
                MUTED,
            );
        }
        ui.add_space(12.0);
        for check in &report.checks {
            ui.horizontal(|ui| {
                ui.colored_label(
                    if check.passed {
                        ACCENT
                    } else {
                        egui::Color32::LIGHT_RED
                    },
                    if check.passed { "通过" } else { "失败" },
                );
                ui.label(format!(
                    "{}  {}：{:.4} {}",
                    check.stage, check.name, check.measured, check.unit
                ));
            });
        }
    }
}
