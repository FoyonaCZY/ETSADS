//! Frontend for the existing evidence-checked host. No alternative permission
//! or input transport is implemented here.
use super::{ACCENT, AMBER, MUTED, PANEL};
use anyhow::{Context, ensure};
use eframe::egui::{self, RichText};
use etsads::driving::{DriveOptions, DriveProgress, DriveReport, experiment::ExperimentKind};
use etsads_runtime::{DrivingMode, RuntimeConfig, RuntimeStatus};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Component, Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const CONFIG_LIMIT: usize = 8 * 1024 * 1024;

#[derive(Default)]
struct DriveForm {
    config: String,
    calibration: String,
    map: String,
    policy: String,
    experiment: Option<ExperimentKind>,
    experiment_evidence: String,
    safety_profile: String,
    road_evidence: String,
    output: String,
    seconds: u64,
    video: bool,
}
impl DriveForm {
    fn options(&self, arm: bool, independent_recording: bool) -> anyhow::Result<DriveOptions> {
        ensure!(
            !independent_recording,
            "请先停止独立录制，驾驶运行会自行录制完整证据。"
        );
        ensure!(
            (1..=86400).contains(&self.seconds),
            "运行时长须为1～86400秒。"
        );
        let config = required_file(&self.config, "运行配置")?;
        let preview = ConfigPreview::read(&config)?;
        ensure!(
            preview.required_experiment == self.experiment,
            "试验类型与运行配置不一致；高级模式必须选择对应试验。"
        );
        let map = optional_path(&self.map, false, "地图")?;
        let policy = optional_path(&self.policy, true, "区域策略")?;
        ensure!(
            map.is_some() == policy.is_some(),
            "地图与区域策略必须同时指定。"
        );
        let evidence = optional_path(&self.experiment_evidence, true, "实验许可证据")?;
        let safety_profile = optional_path(&self.safety_profile, true, "独立安全几何配置")?;
        ensure!(
            self.experiment.is_none() || evidence.is_some(),
            "高级试验需要已审查的实验许可证据。"
        );
        ensure!(
            self.experiment.is_some() || (evidence.is_none() && safety_profile.is_none()),
            "实验许可与安全几何配置需要对应的高级试验类型。"
        );
        ensure!(!self.output.trim().is_empty(), "请填写新的运行记录目录。");
        let output = std::path::absolute(self.output.trim())?;
        ensure!(!output.exists(), "运行目录已存在，请使用新目录保留旧证据。");
        Ok(DriveOptions {
            config,
            calibration: required_file(&self.calibration, "车辆标定")?,
            output,
            map,
            policy,
            region_radius_m: 2000.0,
            seconds: self.seconds,
            arm,
            video: self.video,
            experiment: self.experiment,
            experiment_evidence: evidence,
            safety_profile,
            road_evidence: optional_path(&self.road_evidence, true, "道路运行证据")?,
        })
    }
    fn select_draft(&mut self, path: &Path) {
        self.config = path.display().to_string();
        self.experiment_evidence.clear();
        self.road_evidence.clear();
    }
}
fn required_file(text: &str, label: &str) -> anyhow::Result<PathBuf> {
    optional_path(text, true, label)?.with_context(|| format!("请填写{label}路径。"))
}
fn optional_path(text: &str, file: bool, label: &str) -> anyhow::Result<Option<PathBuf>> {
    if text.trim().is_empty() {
        return Ok(None);
    }
    let path = fs::canonicalize(text.trim()).with_context(|| format!("无法访问{label}：{text}"))?;
    ensure!(!file || path.is_file(), "{label}必须是文件。");
    Ok(Some(path))
}
fn bounded_read(path: &Path) -> anyhow::Result<Vec<u8>> {
    let file = File::open(path)?;
    ensure!(
        file.metadata()?.len() <= CONFIG_LIMIT as u64,
        "运行配置超过8MiB。"
    );
    let mut bytes = Vec::new();
    file.take(CONFIG_LIMIT as u64 + 1).read_to_end(&mut bytes)?;
    ensure!(bytes.len() <= CONFIG_LIMIT, "运行配置读取超过8MiB。");
    Ok(bytes)
}
struct ConfigPreview {
    path: PathBuf,
    bytes_hash: [u8; 32],
    value: Value,
    mode: &'static str,
    speed_pointer: Option<&'static str>,
    speed_kmh: f64,
    maximum_kmh: f64,
    required_experiment: Option<ExperimentKind>,
}
impl ConfigPreview {
    fn read(path: &Path) -> anyhow::Result<Self> {
        let path = fs::canonicalize(path)?;
        let bytes = bounded_read(&path)?;
        let value: Value = serde_json::from_slice(&bytes).context("运行配置必须是有效JSON。")?;
        let config: RuntimeConfig =
            serde_json::from_value(value.clone()).context("运行配置字段与当前Runtime不兼容。")?;
        let (mode, speed_pointer, speed, maximum_kmh, required_experiment) = match config.mode {
            DrivingMode::CommissioningCircle { speed_mps, .. } => (
                "标定绕圆",
                Some("/mode/CommissioningCircle/speed_mps"),
                speed_mps,
                30.0,
                None,
            ),
            DrivingMode::LaneFollow {
                maximum_speed_mps, ..
            } => (
                "车道保持 / ACC",
                Some("/mode/LaneFollow/maximum_speed_mps"),
                maximum_speed_mps,
                216.0,
                None,
            ),
            DrivingMode::ControlledRoute { mission } => (
                "受控路线",
                Some("/mode/ControlledRoute/mission/maximum_speed_mps"),
                mission.maximum_speed_mps,
                90.0,
                Some(ExperimentKind::ControlledRoute),
            ),
            DrivingMode::Parking { .. } => {
                ("泊车", None, 3.0 / 3.6, 3.0, Some(ExperimentKind::Parking))
            }
        };
        ensure!(speed.is_finite() && speed > 0.0, "配置车速无效。");
        Ok(Self {
            path,
            bytes_hash: Sha256::digest(&bytes).into(),
            value,
            mode,
            speed_pointer,
            speed_kmh: speed * 3.6,
            maximum_kmh,
            required_experiment,
        })
    }
    fn save_speed_draft(&self, name: &str, speed_kmh: f64) -> anyhow::Result<PathBuf> {
        let pointer = self
            .speed_pointer
            .context("泊车使用固定3km/h上限，不提供速度覆盖。")?;
        ensure!(
            speed_kmh.is_finite() && speed_kmh > 0.0 && speed_kmh <= self.maximum_kmh,
            "草稿速度超出此模式的结构范围。"
        );
        let mut components = Path::new(name).components();
        ensure!(
            matches!(components.next(), Some(Component::Normal(_)))
                && components.next().is_none()
                && !name.contains(['/', '\\', ':'])
                && name.ends_with(".json"),
            "草稿名称只能是同目录下的.json文件名。"
        );
        let current: [u8; 32] = Sha256::digest(bounded_read(&self.path)?).into();
        ensure!(
            current == self.bytes_hash,
            "原配置已被修改，请重新读取后再保存草稿。"
        );
        let mut value = self.value.clone();
        *value
            .pointer_mut(pointer)
            .context("配置中未找到速度字段。")? = serde_json::json!(speed_kmh / 3.6);
        let bytes = serde_json::to_vec_pretty(&value)?;
        ensure!(bytes.len() <= CONFIG_LIMIT, "保存后的配置超过8MiB。");
        let destination = self.path.parent().context("配置缺少父目录。")?.join(name);
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&destination)
            .context("不能创建草稿：同名文件不可覆盖。")?;
        file.write_all(&bytes)?;
        file.sync_all()?;
        Ok(destination)
    }
}

struct RunSession {
    stop: Arc<AtomicBool>,
    result: mpsc::Receiver<Result<DriveReport, String>>,
    progress: mpsc::Receiver<DriveProgress>,
    last_progress: Option<DriveProgress>,
    arm: bool,
    stopping: bool,
    output: PathBuf,
}
impl RunSession {
    fn request_stop(&mut self) {
        self.stopping = true;
        self.stop.store(true, Ordering::Release);
    }
    fn poll(&mut self) -> Option<Result<DriveReport, String>> {
        for progress in self.progress.try_iter().take(64) {
            self.last_progress = Some(progress);
        }
        match self.result.try_recv() {
            Ok(result) => Some(result),
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => Some(Err(
                "驾驶工作线程已断开；运行已结束，请检查记录是否完整。".into(),
            )),
        }
    }
}
pub struct DrivePanel {
    form: DriveForm,
    preview: Option<ConfigPreview>,
    draft_kmh: f64,
    draft_name: String,
    session: Option<RunSession>,
    notice: Option<String>,
    error: Option<String>,
}
impl Default for DrivePanel {
    fn default() -> Self {
        Self {
            form: DriveForm {
                seconds: 60,
                output: next_output(),
                ..Default::default()
            },
            preview: None,
            draft_kmh: 0.0,
            draft_name: String::new(),
            session: None,
            notice: None,
            error: None,
        }
    }
}
fn next_output() -> String {
    format!(
        "runs/drive-ui-{}",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
    )
}
fn state_label(arm: bool, progress: &DriveProgress) -> (&'static str, egui::Color32) {
    if progress.status == RuntimeStatus::WaitingForParkingPlan {
        return (
            if !arm {
                "观察中 · 等待泊车规划"
            } else if progress.arm_requested {
                "等待泊车规划 · 已请求安全启用"
            } else {
                "等待泊车规划 · 尚未请求启用"
            },
            AMBER,
        );
    }
    if !arm {
        return ("观察中 · 未启用控制", AMBER);
    }
    match progress.status {
        RuntimeStatus::WaitingForParkingPlan => unreachable!(),
        RuntimeStatus::Running if progress.control_sent => ("运行中 · 已发送控制", ACCENT),
        RuntimeStatus::Running => ("运行中 · 等待安全许可", AMBER),
        RuntimeStatus::WaitingForSecondFrame | RuntimeStatus::WaitingForTrajectory => {
            ("等待新鲜帧 / 可用轨迹", AMBER)
        }
        RuntimeStatus::Paused => ("游戏暂停 · 已释放", AMBER),
        RuntimeStatus::Fault => ("运行故障 · 已释放", egui::Color32::LIGHT_RED),
        RuntimeStatus::Completed => ("任务结束 · 已释放", MUTED),
    }
}
fn progress_is_fresh(progress: &DriveProgress) -> bool {
    Instant::now()
        .checked_duration_since(progress.produced_at)
        .is_some_and(|age| age <= Duration::from_millis(500))
}
impl DrivePanel {
    pub fn busy(&self) -> bool {
        self.session.is_some()
    }
    pub fn supervisor_profile(&self) -> anyhow::Result<Option<PathBuf>> {
        optional_path(&self.form.safety_profile, true, "安全几何配置")
    }
    pub fn request_stop(&mut self) {
        if let Some(session) = &mut self.session {
            session.request_stop();
        }
    }
    pub fn poll(&mut self) -> Option<String> {
        let result = self.session.as_mut().and_then(RunSession::poll)?;
        self.session = None;
        let message = match result {
            Ok(report) => format!(
                "驾驶运行结束：{}；{}帧，实际控制{}帧；记录：{}",
                report.stop_reason,
                report.frames,
                report.active_frames,
                report.output.display()
            ),
            Err(error) => {
                self.error = Some(error.clone());
                format!("驾驶运行失败：{error}")
            }
        };
        self.notice = Some(message.clone());
        Some(message)
    }
    fn load(&mut self) -> anyhow::Result<()> {
        let preview = ConfigPreview::read(Path::new(self.form.config.trim()))?;
        self.draft_kmh = preview.speed_kmh;
        self.draft_name = format!(
            "{}-speed-draft.json",
            preview
                .path
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
        );
        self.form.config = preview.path.display().to_string();
        self.preview = Some(preview);
        Ok(())
    }
    fn save(&mut self) -> anyhow::Result<()> {
        let preview = self.preview.as_ref().context("请先读取配置。")?;
        ensure!(
            fs::canonicalize(self.form.config.trim())? == preview.path,
            "路径已更改，请先读取当前配置。"
        );
        let path = preview.save_speed_draft(self.draft_name.trim(), self.draft_kmh)?;
        self.form.select_draft(&path);
        self.load()?;
        self.notice=Some("已切换到新草稿，原配置未覆盖。实验及道路许可已清空；配置指纹已变化，须选择匹配证据并重新通过Host校验。".into());
        Ok(())
    }
    fn speed_draft_dirty(&self) -> bool {
        self.preview.as_ref().is_some_and(|p| {
            p.speed_pointer.is_some()
                && (!self.draft_kmh.is_finite() || (self.draft_kmh - p.speed_kmh).abs() > 1e-9)
        })
    }
    fn restore_speed_draft(&mut self) {
        if let Some(preview) = &self.preview {
            self.draft_kmh = preview.speed_kmh;
        }
    }
    fn start(&mut self, arm: bool, independent_recording: bool) -> anyhow::Result<()> {
        ensure!(!self.busy(), "当前已有驾驶运行。");
        ensure!(
            !self.speed_draft_dirty(),
            "速度草稿尚未保存；请保存新配置或还原速度后再启动。"
        );
        let options = self.form.options(arm, independent_recording)?;
        let output = options.output.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let (result_tx, result) = mpsc::sync_channel(1);
        let (progress_tx, progress) = mpsc::sync_channel(32);
        std::thread::Builder::new()
            .name("ui-driving-host".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    etsads::driving::run_with_progress(options, worker_stop, progress_tx)
                }))
                .map_err(|_| "驾驶工作线程发生异常；已请求释放控制，请检查记录完整性。".to_string())
                .and_then(|result| result.map_err(|e| format!("{e:#}")));
                if result.is_err() {
                    let _ = etsads::set_gate(false);
                }
                let _ = result_tx.send(result);
            })?;
        self.session = Some(RunSession {
            stop,
            result,
            progress,
            last_progress: None,
            arm,
            stopping: false,
            output,
        });
        self.error = None;
        self.notice = None;
        Ok(())
    }
    pub fn show(&mut self, ui: &mut egui::Ui, independent_recording: bool, closing: bool) {
        ui.label(RichText::new("运行与目标速度").size(30.0).strong());
        ui.colored_label(
            AMBER,
            "开发验证入口 · 默认观察 · 全部标定、来源及实测证据仍由原Host校验",
        );
        ui.label("观察运行也会记录数据并要求有效配置，不代表实车阶段已验收。路径可直接粘贴；空的可选字段不传入Host。");
        let busy = self.busy();
        if let Some(session) = &self.session {
            egui::Frame::new()
                .fill(PANEL)
                .inner_margin(16)
                .corner_radius(10)
                .show(ui, |ui| {
                    if session.stopping {
                        ui.colored_label(AMBER, "正在停止并保存运行记录…");
                    } else if let Some(progress) = &session.last_progress {
                        if !progress_is_fresh(progress) {
                            ui.colored_label(AMBER, "等待最新Host状态 · 不推断控制仍有效");
                        } else {
                            let (label, color) = state_label(session.arm, progress);
                            ui.colored_label(color, label);
                        }
                        ui.label(format!(
                            "物理帧 {} · 当前 {:.1} km/h",
                            progress.frame_id,
                            progress.speed_mps * 3.6
                        ));
                        ui.label(&progress.reason);
                    } else {
                        ui.spinner();
                        ui.label(if session.arm {
                            "正在校验配置和证据 · 尚未确认已启用"
                        } else {
                            "观察运行预检中 · 未启用控制"
                        });
                    }
                    ui.label(format!("记录目录：{}", session.output.display()));
                });
            if ui
                .add_enabled(
                    !closing,
                    egui::Button::new("停止运行并释放控制")
                        .fill(egui::Color32::from_rgb(109, 45, 48)),
                )
                .clicked()
            {
                self.request_stop();
                if let Err(error) = etsads::set_gate(false) {
                    self.error = Some(format!("已请求停止；释放请求返回：{error:#}"));
                }
            }
        }
        if independent_recording {
            ui.colored_label(
                AMBER,
                "独立录制正在进行；请先在驾驶面板停止并保存，避免双重录制。",
            );
        }
        ui.add_enabled_ui(!busy && !closing,|ui| {
            egui::Grid::new("drive-paths").num_columns(2).spacing([14.0,9.0]).show(ui,|ui| {
                let previous_config=self.form.config.clone();
                path_row(ui,"运行配置 JSON",&mut self.form.config);
                if self.form.config!=previous_config { self.preview=None; }
                path_row(ui,"车辆标定 JSON / TOML",&mut self.form.calibration);
                path_row(ui,"地图（绕圆可留空）",&mut self.form.map);
                path_row(ui,"区域策略",&mut self.form.policy);
                path_row(ui,"实验许可证据",&mut self.form.experiment_evidence);
                path_row(ui,"安全几何配置",&mut self.form.safety_profile);
                path_row(ui,"道路运行证据",&mut self.form.road_evidence);
                path_row(ui,"新运行记录目录",&mut self.form.output);
            });
            ui.label(RichText::new("安全几何配置也用于面板启动独立监督；已运行的监督不会自动换配置。原生导航使用2km地图窗口。").size(13.0).color(MUTED));
            ui.horizontal_wrapped(|ui| {
                ui.label("试验类型");
                egui::ComboBox::from_id_salt("drive-experiment").selected_text(match self.form.experiment {None=>"无 · P0/P1",Some(ExperimentKind::ControlledRoute)=>"受控路线",Some(ExperimentKind::Parking)=>"泊车"}).show_ui(ui,|ui| {
                    ui.selectable_value(&mut self.form.experiment,None,"无 · P0/P1");
                    ui.selectable_value(&mut self.form.experiment,Some(ExperimentKind::ControlledRoute),"受控路线");
                    ui.selectable_value(&mut self.form.experiment,Some(ExperimentKind::Parking),"泊车");
                });
                ui.label("时长"); ui.add(egui::DragValue::new(&mut self.form.seconds).range(1..=86400).suffix(" 秒"));
                ui.checkbox(&mut self.form.video,"同时录制视频");
                if ui.button("生成新记录路径").clicked() { self.form.output=next_output(); }
            });
            ui.separator();
            if ui.button("读取配置与速度").clicked() {
                self.error=None;
                if let Err(error)=self.load() { self.preview=None; self.error=Some(format!("{error:#}")); }
            }
            if let Some(preview)=&self.preview {
                ui.label(format!("已读取：{} · 配置目标/上限 {:.1} km/h",preview.mode,preview.speed_kmh));
                let hash:String=preview.bytes_hash.iter().map(|b|format!("{b:02x}")).collect();
                ui.label(RichText::new(format!("读取时配置SHA-256：{hash}")).size(12.0).color(MUTED));
                if preview.speed_pointer.is_some() {
                    ui.horizontal_wrapped(|ui| {
                        ui.label("新目标/上限");
                        ui.add(egui::DragValue::new(&mut self.draft_kmh).speed(0.5).range(0.1..=preview.maximum_kmh).suffix(" km/h"));
                        ui.label("同目录草稿文件名"); ui.text_edit_singleline(&mut self.draft_name);
                    });
                    ui.label("仅保存新草稿，不修改运行中的速度；地图、标定及证据仍决定实际可用速度。保存不会授予原证据权限。");
                    if ui.button("保存新速度草稿并选用").clicked() { self.error=None; if let Err(error)=self.save() {self.error=Some(format!("{error:#}"));} }
                } else { ui.label("泊车固定3 km/h上限；这里不能覆盖其速度。"); }
            }
            ui.separator();
            let unsaved_speed=self.speed_draft_dirty();
            if unsaved_speed {
                ui.colored_label(AMBER,"速度草稿尚未保存，启动已禁用。请保存新草稿，或还原为当前配置速度。");
                if ui.button("还原为当前配置速度").clicked() { self.restore_speed_draft(); }
            }
            ui.add_enabled_ui(!independent_recording && !unsaved_speed,|ui| {
                if ui.button("开始观察（不启用控制）").clicked() && let Err(error)=self.start(false,independent_recording) {self.error=Some(format!("{error:#}"));}
                if ui.add(egui::Button::new("启用并运行").fill(egui::Color32::from_rgb(100,70,27))).clicked() && let Err(error)=self.start(true,independent_recording) {self.error=Some(format!("{error:#}"));}
            });
        });
        if let Some(message) = &self.notice {
            ui.label(message);
        }
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
    }
}
impl Drop for DrivePanel {
    fn drop(&mut self) {
        self.request_stop();
    }
}
fn path_row(ui: &mut egui::Ui, label: &str, value: &mut String) {
    ui.label(label);
    ui.add(
        egui::TextEdit::singleline(value).desired_width((ui.available_width() - 12.0).max(220.0)),
    );
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;
    fn directory(name: &str) -> PathBuf {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-runs")
            .join(format!(
                "ui-drive-{name}-{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ));
        fs::create_dir_all(&path).unwrap();
        path
    }
    fn circle() -> Value {
        serde_json::json!({"execution":"Replay","vehicle_id":"synthetic UI fixture","vehicle_configuration_verified":false,
            "mode":{"CommissioningCircle":{"radius_m":50.0,"speed_mps":2.0,"clockwise":false,"verified_clear_area":[]}},
            "unknown_future_field":{"relative":"evidence/relative.json","native_uid":u64::MAX}})
    }
    fn write_config(dir: &Path, value: &Value) -> PathBuf {
        let path = dir.join("原始 配置.json");
        fs::write(&path, serde_json::to_vec(value).unwrap()).unwrap();
        path
    }
    #[test]
    fn speed_draft_is_new_bounded_and_preserves_unknown_fields_and_relative_references() {
        let dir = directory("draft");
        let original = circle();
        let path = write_config(&dir, &original);
        let before = fs::read(&path).unwrap();
        let preview = ConfigPreview::read(&path).unwrap();
        let draft = preview.save_speed_draft("新 速度.json", 10.0).unwrap();
        assert_eq!(draft.parent(), Some(preview.path.parent().unwrap()));
        assert_eq!(fs::read(&path).unwrap(), before);
        let saved: Value = serde_json::from_slice(&fs::read(&draft).unwrap()).unwrap();
        assert_eq!(
            saved["unknown_future_field"],
            original["unknown_future_field"]
        );
        assert_eq!(
            saved
                .pointer(preview.speed_pointer.unwrap())
                .unwrap()
                .as_f64(),
            Some(10.0 / 3.6)
        );
        assert!(preview.save_speed_draft("新 速度.json", 12.0).is_err());
        assert!(preview.save_speed_draft("原始 配置.json", 12.0).is_err());
        for name in ["../escape.json", "folder/new.json", "file:stream.json"] {
            assert!(preview.save_speed_draft(name, 8.0).is_err());
        }
        fs::write(&path, b"changed by another writer").unwrap();
        assert!(preview.save_speed_draft("changed.json", 8.0).is_err());
        assert!(!dir.join("changed.json").exists());
    }
    #[test]
    fn oversize_read_or_draft_never_creates_a_usable_output() {
        let dir = directory("limits");
        let path = write_config(&dir, &circle());
        let mut preview = ConfigPreview::read(&path).unwrap();
        preview.value["oversize"] = Value::String("x".repeat(CONFIG_LIMIT));
        assert!(preview.save_speed_draft("oversize.json", 8.0).is_err());
        assert!(!dir.join("oversize.json").exists());
        OpenOptions::new()
            .write(true)
            .open(&path)
            .unwrap()
            .set_len(CONFIG_LIMIT as u64 + 1)
            .unwrap();
        assert!(ConfigPreview::read(&path).is_err());
    }
    #[test]
    fn parking_speed_is_fixed_and_draft_selection_invalidates_only_associated_permissions() {
        let dir = directory("parking");
        let mut value = circle();
        value["mode"] = serde_json::json!({"Parking":{"mission":{
            "target":{"source_id":"synthetic","pose":{"x":0.0,"y":0.0,"yaw":0.0},"reference":"Unknown"},
            "boundary":{"polygon":[],"physical_boundaries_verified":false,"evidence_path":"relative/boundary.json"},
            "static_obstacles":[],"max_expansions":1000,"maximum_duration_s":30.0}}});
        let path = write_config(&dir, &value);
        let preview = ConfigPreview::read(&path).unwrap();
        assert_eq!(preview.speed_kmh, 3.0);
        assert!(preview.speed_pointer.is_none());
        assert!(preview.save_speed_draft("forbidden.json", 4.0).is_err());
        let mut form = DriveForm {
            experiment_evidence: "review-old.json".into(),
            road_evidence: "road-old.json".into(),
            calibration: "vehicle.json".into(),
            safety_profile: "safety.json".into(),
            ..Default::default()
        };
        form.select_draft(&path);
        assert!(form.experiment_evidence.is_empty() && form.road_evidence.is_empty());
        assert_eq!(form.calibration, "vehicle.json");
        assert_eq!(form.safety_profile, "safety.json");
    }
    #[test]
    fn parameters_remain_observation_by_default_and_prevent_double_recording() {
        let dir = directory("options");
        let path = write_config(&dir, &circle());
        let calibration = dir.join("synthetic calibration.json");
        fs::write(&calibration, b"{}").unwrap();
        let mut form = DriveForm {
            config: path.display().to_string(),
            calibration: calibration.display().to_string(),
            output: dir.join("new run").display().to_string(),
            seconds: 60,
            ..Default::default()
        };
        let options = form.options(false, false).unwrap();
        assert!(!options.arm && !options.video);
        assert_eq!(options.region_radius_m, 2000.0);
        assert_eq!(options.seconds, 60);
        assert!(form.options(true, false).unwrap().arm);
        assert!(form.options(false, true).is_err());
        form.map = dir.display().to_string();
        assert!(form.options(false, false).is_err());
        form.map.clear();
        form.seconds = 0;
        assert!(form.options(false, false).is_err());
        form.seconds = 60;
        fs::create_dir(dir.join("new run")).unwrap();
        assert!(form.options(false, false).is_err());
    }
    #[test]
    fn stop_and_worker_disconnect_release_busy_without_waiting_forever() {
        let (result_tx, result) = mpsc::sync_channel(1);
        let (_progress_tx, progress) = mpsc::sync_channel(1);
        let stop = Arc::new(AtomicBool::new(false));
        let mut panel = DrivePanel::default();
        panel.session.replace(RunSession {
            stop: stop.clone(),
            result,
            progress,
            last_progress: None,
            arm: false,
            stopping: false,
            output: PathBuf::from("synthetic"),
        });
        panel.request_stop();
        assert!(stop.load(Ordering::Acquire));
        assert!(panel.busy());
        drop(result_tx);
        let message = panel.poll().unwrap();
        assert!(message.contains("断开"));
        assert!(!panel.busy());
        assert!(panel.poll().is_none());
    }
    #[test]
    fn observation_and_parking_wait_never_claim_active_driving() {
        let mut progress = DriveProgress {
            produced_at: Instant::now(),
            frame_id: 1,
            status: RuntimeStatus::Running,
            reason: "synthetic".into(),
            arm_requested: false,
            control_sent: true,
            speed_mps: 0.0,
        };
        assert_eq!(state_label(false, &progress).0, "观察中 · 未启用控制");
        progress.status = RuntimeStatus::WaitingForParkingPlan;
        assert!(state_label(true, &progress).0.contains("等待泊车规划"));
        progress.control_sent = false;
        assert!(state_label(true, &progress).0.contains("尚未请求启用"));
        assert_eq!(state_label(false, &progress).0, "观察中 · 等待泊车规划");
        progress.status = RuntimeStatus::Running;
        assert_eq!(state_label(true, &progress).0, "运行中 · 等待安全许可");
    }
    #[test]
    fn queued_old_progress_cannot_become_fresh_when_the_ui_receives_it() {
        let (_result_tx, result) = mpsc::sync_channel(1);
        let (progress_tx, progress) = mpsc::sync_channel(1);
        let stale = DriveProgress {
            produced_at: Instant::now() - Duration::from_secs(3),
            frame_id: 1,
            status: RuntimeStatus::Running,
            reason: "synthetic queued state".into(),
            arm_requested: true,
            control_sent: true,
            speed_mps: 0.0,
        };
        progress_tx.send(stale).unwrap();
        let mut session = RunSession {
            stop: Arc::new(AtomicBool::new(false)),
            result,
            progress,
            last_progress: None,
            arm: true,
            stopping: false,
            output: PathBuf::from("synthetic"),
        };
        assert!(session.poll().is_none());
        assert!(!progress_is_fresh(session.last_progress.as_ref().unwrap()));
    }
    #[test]
    fn unsaved_speed_cannot_silently_start_the_old_configuration() {
        let dir = directory("unsaved");
        let path = write_config(&dir, &circle());
        let mut panel = DrivePanel::default();
        panel.form.config = path.display().to_string();
        panel.load().unwrap();
        panel.draft_kmh += 1.0;
        assert!(panel.speed_draft_dirty());
        assert!(
            panel
                .start(false, false)
                .unwrap_err()
                .to_string()
                .contains("草稿尚未保存")
        );
        assert!(!panel.busy());
        panel.restore_speed_draft();
        assert!(!panel.speed_draft_dirty());
        panel.draft_kmh += 2.0;
        panel.save().unwrap();
        assert!(!panel.speed_draft_dirty());
        assert!((panel.preview.as_ref().unwrap().speed_kmh - 9.2).abs() < 1e-8);
    }
}
