//! First-run setup: find the game, install the plugins, generate the local
//! map from the player's own game files and keep per-truck steering tables.
//! Every action here is explicit (a button or command); nothing is changed in
//! the game directory while the game runs, and replaced plugins are backed up.
use anyhow::{Context, ensure};
use std::{
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};

/// The ETS2LA SDK plugin build verified with ETS2 1.61 (MIT, downloaded from
/// its pinned commit, never redistributed with this program).
pub const ETS2LA_URL: &str = "https://raw.githubusercontent.com/ETS2LA/ETS2LA/26f32720b5c1dd7e8d7d14efaa483835f044e2a5/Assets/SDKs/1.61/Windows/ets2la_plugin.dll";
pub const ETS2LA_SHA256: &str = "0E1893719F84F28079857451B82A22A263401D98B8FC299CD561EB7695BFA88F";
const PLUGINS: &str = "bin/win_x64/plugins";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PluginState {
    Missing,
    Current,
    /// Installed but not the file this program ships (or the pinned build).
    Different,
    /// Nothing to compare against (no bundled copy found).
    Unknown,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MapState {
    Missing,
    Ready {
        game_version: String,
    },
    /// Generated from another game version: positions or junctions may differ.
    Outdated {
        map_version: String,
    },
}
#[derive(Clone, Debug)]
pub struct SetupReport {
    pub game_dir: Option<PathBuf>,
    pub game_version: Option<String>,
    pub game_running: bool,
    pub bridge: PluginState,
    pub graphics: PluginState,
    pub ets2la: PluginState,
    pub version_supported: bool,
    pub map: MapState,
    pub map_dir: PathBuf,
    pub calibrated_trucks: usize,
    /// The loaded profile's steering mode, when it can be read.
    pub steering: Option<ProfileSteering>,
}
impl SetupReport {
    pub fn plugins_ready(&self) -> bool {
        self.bridge != PluginState::Missing
            && self.graphics != PluginState::Missing
            && self.ets2la == PluginState::Current
    }
    pub fn map_ready(&self) -> bool {
        matches!(self.map, MapState::Ready { .. })
    }
}

/// Program root: the portable package directory, or the source workspace.
pub fn root() -> PathBuf {
    let mut roots = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        roots.extend(exe.ancestors().skip(1).take(5).map(Path::to_path_buf));
    }
    if let Ok(cwd) = std::env::current_dir() {
        roots.push(cwd);
    }
    roots
        .iter()
        .find(|p| p.join("ETSADS.cmd").is_file() || p.join("Cargo.toml").is_file())
        .or(roots.first())
        .cloned()
        .unwrap_or_else(|| PathBuf::from("."))
}
pub fn data_dir() -> PathBuf {
    root().join("data")
}
/// Where the generated map lives; a developer export is used if present.
pub fn default_map_dir() -> PathBuf {
    let generated = data_dir().join("map");
    if generated.join("traffic_index.json").is_file() {
        return generated;
    }
    let developer = root().join("tools/map_export/output/sector-europe-profile-v3-final");
    if developer.join("traffic_index.json").is_file() {
        developer
    } else {
        generated
    }
}

fn bundled(name: &str) -> Option<PathBuf> {
    let exe_dir = std::env::current_exe().ok()?.parent()?.to_path_buf();
    [
        exe_dir.join(name),
        exe_dir.join("plugins").join(name),
        root().join("target/release").join(name),
    ]
    .into_iter()
    .find(|p| p.is_file())
}
pub fn sha256(path: &Path) -> anyhow::Result<String> {
    use sha2::{Digest, Sha256};
    use std::io::Read;
    let mut file =
        std::fs::File::open(path).with_context(|| format!("无法读取 {}", path.display()))?;
    let mut hash = Sha256::new();
    let mut buffer = vec![0u8; 1 << 16];
    loop {
        let count = file.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        hash.update(&buffer[..count]);
    }
    Ok(hash.finalize().iter().map(|b| format!("{b:02X}")).collect())
}
fn plugin_state(installed: &Path, expected: Option<String>) -> PluginState {
    if !installed.is_file() {
        return PluginState::Missing;
    }
    match (expected, sha256(installed)) {
        (Some(want), Ok(have)) if want.eq_ignore_ascii_case(&have) => PluginState::Current,
        (Some(_), _) => PluginState::Different,
        (None, _) => PluginState::Unknown,
    }
}
pub fn game_running() -> bool {
    Command::new("tasklist")
        .args(["/FI", "IMAGENAME eq eurotrucks2.exe", "/NH"])
        .creation_flags_hidden()
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains("eurotrucks2.exe"))
        .unwrap_or(false)
}

pub fn check(map_dir: &Path, steering_dir: &Path) -> SetupReport {
    let game_dir = crate::find_game();
    let game_version = game_dir
        .as_ref()
        .and_then(|g| etsads_game::version::file_version(&g.join("bin/win_x64/eurotrucks2.exe")));
    let plugins = game_dir.as_ref().map(|g| g.join(PLUGINS));
    let state = |name: &str, expected: Option<String>| {
        plugins.as_ref().map_or(PluginState::Missing, |p| {
            plugin_state(&p.join(name), expected)
        })
    };
    let bridge = state(
        "etsads_bridge.dll",
        bundled("etsads_bridge.dll").and_then(|p| sha256(&p).ok()),
    );
    let graphics = state(
        "etsads_gfx.dll",
        bundled("etsads_gfx.dll").and_then(|p| sha256(&p).ok()),
    );
    let ets2la = state("ets2la_plugin.dll", Some(ETS2LA_SHA256.into()));
    let map = map_state(map_dir, game_version.as_deref());
    SetupReport {
        version_supported: game_version
            .as_deref()
            .is_some_and(etsads_game::version::extension_version_supported),
        game_dir,
        game_version,
        game_running: game_running(),
        bridge,
        graphics,
        ets2la,
        map,
        map_dir: map_dir.to_path_buf(),
        calibrated_trucks: crate::assist::steering_reports(steering_dir),
        steering: active_steering().ok(),
    }
}

/// How the active game profile applies the steering input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SteeringMode {
    /// The input sets the steering position (`c_relatsteer 0`).
    Absolute,
    /// The input turns the wheel (`c_relatsteer 1`, the default for new
    /// profiles): the steering position changes by `rate_per_s` x input per
    /// second, and stays where it is at zero input. The rate is the profile's
    /// steering sensitivity `c_rsteersens` (measured 0.4/s at 0.4, linear in
    /// the input from 0.1 to 1.0, no self-centring).
    Relative { rate_per_s: f64 },
}
#[derive(Clone, Debug, PartialEq)]
pub struct ProfileSteering {
    pub profile: String,
    pub mode: SteeringMode,
}
impl std::fmt::Display for ProfileSteering {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.mode {
            SteeringMode::Absolute => write!(f, "档案“{}”：绝对转向", self.profile),
            SteeringMode::Relative { rate_per_s } => write!(
                f,
                "档案“{}”：相对转向（灵敏度 {rate_per_s:.2}）",
                self.profile
            ),
        }
    }
}
/// The steering mode of the profile the game has loaded: the last profile
/// named in the game log, else the mode every profile agrees on.
pub fn active_steering() -> anyhow::Result<ProfileSteering> {
    let documents =
        game_documents().context("没有找到欧卡2的设置目录（文档\\Euro Truck Simulator 2）")?;
    if let Some(name) = std::fs::read(documents.join("game.log.txt"))
        .ok()
        .and_then(|log| current_profile(&String::from_utf8_lossy(&log)))
    {
        let folder: String = name.bytes().map(|b| format!("{b:02X}")).collect();
        for kind in ["steam_profiles", "profiles"] {
            let controls = documents.join(kind).join(&folder).join("controls.sii");
            if let Ok(text) = std::fs::read_to_string(&controls) {
                let mode = steering_mode(&text)
                    .with_context(|| format!("无法识别档案“{name}”的转向设置"))?;
                return Ok(ProfileSteering {
                    profile: name,
                    mode,
                });
            }
        }
    }
    let mut modes = Vec::new();
    for kind in ["steam_profiles", "profiles"] {
        for entry in std::fs::read_dir(documents.join(kind))
            .into_iter()
            .flatten()
            .flatten()
        {
            if let Some(mode) = std::fs::read_to_string(entry.path().join("controls.sii"))
                .ok()
                .and_then(|text| steering_mode(&text))
            {
                modes.push((profile_name(&entry.file_name().to_string_lossy()), mode));
            }
        }
    }
    let (profile, mode) = modes
        .first()
        .cloned()
        .context("没有找到游戏档案的控制设置")?;
    ensure!(
        modes.iter().all(|(_, m)| *m == mode),
        "无法确定游戏当前使用哪个档案（各档案的转向设置不同）：请进入游戏驾驶后重试"
    );
    Ok(ProfileSteering { profile, mode })
}
/// The profile named last in the game log.
fn current_profile(log: &str) -> Option<String> {
    log.lines()
        .rev()
        .find_map(|line| {
            ["New profile selected: '", "Set profile finished: '"]
                .iter()
                .find_map(|key| line.split(key).nth(1))
        })
        .and_then(|rest| rest.rsplit_once('\''))
        .map(|(name, _)| name.to_owned())
        .filter(|name| !name.is_empty())
}
fn steering_mode(controls: &str) -> Option<SteeringMode> {
    let constant = |key: &str| {
        controls.lines().find_map(|line| {
            line.split(&format!("constant {key} "))
                .nth(1)?
                .trim_end_matches(|c: char| c == '"' || c.is_whitespace())
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
        })
    };
    if constant("c_relatsteer")? > 0.5 {
        Some(SteeringMode::Relative {
            rate_per_s: constant("c_rsteersens")?,
        })
        .filter(|m| matches!(m, SteeringMode::Relative { rate_per_s } if (0.05..=5.0).contains(rate_per_s)))
    } else {
        Some(SteeringMode::Absolute)
    }
}
/// Profile folders are named by the hex-encoded profile name.
fn profile_name(folder: &str) -> String {
    let bytes: Option<Vec<u8>> = folder
        .len()
        .is_multiple_of(2)
        .then(|| {
            (0..folder.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(folder.get(i..i + 2)?, 16).ok())
                .collect()
        })
        .flatten();
    bytes
        .and_then(|b| String::from_utf8(b).ok())
        .filter(|name| !name.is_empty() && !name.chars().any(char::is_control))
        .unwrap_or_else(|| folder.to_owned())
}
/// The game's settings folder under the player's Documents, which may be
/// redirected (OneDrive); `ETSADS_ETS2_DOCUMENTS` overrides it.
fn game_documents() -> Option<PathBuf> {
    if let Some(dir) = std::env::var_os("ETSADS_ETS2_DOCUMENTS") {
        return Some(dir.into());
    }
    documents_folder()
        .map(|d| d.join("Euro Truck Simulator 2"))
        .filter(|d| d.is_dir())
}
#[cfg(windows)]
fn documents_folder() -> Option<PathBuf> {
    use std::os::windows::ffi::OsStringExt;
    use windows_sys::Win32::{
        System::Com::CoTaskMemFree,
        UI::Shell::{FOLDERID_Documents, SHGetKnownFolderPath},
    };
    unsafe {
        let mut path = std::ptr::null_mut();
        let result = SHGetKnownFolderPath(&FOLDERID_Documents, 0, std::ptr::null_mut(), &mut path);
        let folder = (result == 0 && !path.is_null()).then(|| {
            let length = (0..).take_while(|&i| *path.add(i) != 0).count();
            PathBuf::from(std::ffi::OsString::from_wide(std::slice::from_raw_parts(
                path, length,
            )))
        });
        CoTaskMemFree(path as _);
        folder
    }
}
#[cfg(not(windows))]
fn documents_folder() -> Option<PathBuf> {
    std::env::var_os("HOME").map(|home| PathBuf::from(home).join("Documents"))
}
fn map_state(dir: &Path, game_version: Option<&str>) -> MapState {
    if !dir.join("traffic_index.json").is_file() || !dir.join("sector_index.json").is_file() {
        return MapState::Missing;
    }
    let version = std::fs::read(dir.join("manifest.json"))
        .ok()
        .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
        .and_then(|v| v["game_version"].as_str().map(str::to_owned))
        .unwrap_or_default();
    match game_version {
        Some(game) if game != version => MapState::Outdated {
            map_version: version,
        },
        _ => MapState::Ready {
            game_version: version,
        },
    }
}

/// Copy our two plugins (and the pinned ETS2LA plugin, downloaded if absent)
/// into the game. Replaced files are moved to `data/plugin-backup`, outside
/// the plugins folder, so the game never loads a stale copy.
pub fn install_plugins(mut log: impl FnMut(String)) -> anyhow::Result<()> {
    ensure!(!game_running(), "请先退出欧卡2（游戏运行时插件文件被占用）");
    let game =
        crate::find_game().context("没有找到欧卡2安装目录；可设置环境变量 ETSADS_GAME_DIR")?;
    let plugins = game.join(PLUGINS);
    std::fs::create_dir_all(&plugins)?;
    let backup = data_dir().join("plugin-backup").join(chrono_stamp());
    let place = |source: &Path, name: &str| -> anyhow::Result<String> {
        let target = plugins.join(name);
        if target.is_file() {
            if sha256(&target)? == sha256(source)? {
                return Ok(format!("{name} 已是最新"));
            }
            std::fs::create_dir_all(&backup)?;
            std::fs::copy(&target, backup.join(name))?;
        }
        std::fs::copy(source, &target).with_context(|| format!("无法写入 {}", target.display()))?;
        Ok(format!("已安装 {name}"))
    };
    for name in ["etsads_bridge.dll", "etsads_gfx.dll"] {
        let source =
            bundled(name).with_context(|| format!("程序目录缺少 {name}，请重新下载完整程序包"))?;
        log(place(&source, name)?);
    }
    // A different screen-capture plugin hooking the same Present call would
    // conflict with ours; disable it by renaming (reversible).
    let capture = plugins.join("ets2la_capture.dll");
    if capture.is_file() {
        std::fs::create_dir_all(&backup)?;
        std::fs::rename(&capture, backup.join("ets2la_capture.dll"))?;
        log("已停用冲突的 ets2la_capture.dll（备份在 data/plugin-backup）".into());
    }
    let ets2la = plugins.join("ets2la_plugin.dll");
    if plugin_state(&ets2la, Some(ETS2LA_SHA256.into())) != PluginState::Current {
        log("下载 ETS2LA 交通数据插件（MIT 许可，约 370 KB）".into());
        let temporary = data_dir().join("download").join("ets2la_plugin.dll");
        std::fs::create_dir_all(temporary.parent().unwrap())?;
        let status = Command::new("curl.exe")
            .args(["-fsSL", "--retry", "2", "-o"])
            .arg(&temporary)
            .arg(ETS2LA_URL)
            .creation_flags_hidden()
            .status()
            .context("无法运行 curl.exe 下载插件")?;
        ensure!(status.success(), "下载失败，请检查网络后重试");
        let hash = sha256(&temporary)?;
        if !hash.eq_ignore_ascii_case(ETS2LA_SHA256) {
            let _ = std::fs::remove_file(&temporary);
            anyhow::bail!("下载的插件校验值不符，已丢弃");
        }
        log(place(&temporary, "ets2la_plugin.dll")?);
        let _ = std::fs::remove_file(&temporary);
    } else {
        log("ets2la_plugin.dll 已是最新".into());
    }
    Ok(())
}
fn chrono_stamp() -> String {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    format!("{seconds}")
}

/// The map exporter: a self-contained build shipped beside the program, or
/// the source build run with the repository's .NET.
fn exporter() -> anyhow::Result<Command> {
    let exe_dir = std::env::current_exe()?
        .parent()
        .context("程序目录未知")?
        .to_path_buf();
    for candidate in [
        exe_dir.join("map-export/etsads-map-export.exe"),
        root().join("target/map-export/etsads-map-export.exe"),
    ] {
        if candidate.is_file() {
            return Ok(Command::new(candidate));
        }
    }
    let dll = root().join("tools/map_export/bin/Release/net10.0/etsads-map-export.dll");
    let dotnet = root().join(".tools/dotnet/dotnet.exe");
    ensure!(
        dll.is_file() && dotnet.is_file(),
        "程序目录缺少地图导出工具（map-export），请重新下载完整程序包"
    );
    let mut command = Command::new(dotnet);
    command.arg(dll);
    Ok(command)
}
#[derive(Clone, Debug)]
pub struct MapProgress {
    pub step: usize,
    pub steps: usize,
    pub message: String,
    pub elapsed_s: f64,
}
/// Generate the runtime map from the installed game's own archives, about
/// five minutes and 2 GB (the intermediate export is removed afterwards).
pub fn generate_map(
    output: &Path,
    stop: &std::sync::atomic::AtomicBool,
    mut progress: impl FnMut(MapProgress),
) -> anyhow::Result<()> {
    let game =
        crate::find_game().context("没有找到欧卡2安装目录；可设置环境变量 ETSADS_GAME_DIR")?;
    let work = output.with_file_name("map-build");
    if work.exists() {
        std::fs::remove_dir_all(&work).context("无法清理上次未完成的地图生成目录")?;
    }
    let raw = work.join("raw");
    let sectors = work.join("sectors");
    std::fs::create_dir_all(&raw)?;
    let started = Instant::now();
    let steps: [(&str, Vec<std::ffi::OsString>); 3] = [
        ("读取游戏地图（约 2 分钟）", vec![]),
        ("切分地图分片（约 2 分钟）", vec![]),
        ("整理路口与信号灯（约 1 分钟）", vec![]),
    ];
    let run = |step: usize,
               args: &[&std::ffi::OsStr],
               progress: &mut dyn FnMut(MapProgress)|
     -> anyhow::Result<()> {
        progress(MapProgress {
            step,
            steps: steps.len(),
            message: steps[step].0.into(),
            elapsed_s: started.elapsed().as_secs_f64(),
        });
        let log = std::fs::File::create(work.join(format!("step-{step}.log")))?;
        let mut child = exporter()?
            .args(args)
            .stdout(log.try_clone()?)
            .stderr(log)
            .creation_flags_hidden()
            .spawn()
            .context("无法启动地图导出工具")?;
        loop {
            if let Some(status) = child.try_wait()? {
                ensure!(
                    status.success(),
                    "地图生成第 {} 步失败，详情见 {}",
                    step + 1,
                    work.join(format!("step-{step}.log")).display()
                );
                return Ok(());
            }
            if stop.load(std::sync::atomic::Ordering::Acquire) {
                let _ = child.kill();
                anyhow::bail!("已取消地图生成");
            }
            progress(MapProgress {
                step,
                steps: steps.len(),
                message: steps[step].0.into(),
                elapsed_s: started.elapsed().as_secs_f64(),
            });
            std::thread::sleep(std::time::Duration::from_millis(250));
        }
    };
    run(
        0,
        &[
            "export".as_ref(),
            "--game-dir".as_ref(),
            game.as_os_str(),
            "--output".as_ref(),
            raw.as_os_str(),
        ],
        &mut progress,
    )?;
    // The exporter writes one directory named after its cache key.
    let export = std::fs::read_dir(&raw)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .find(|p| p.join("manifest.json").is_file())
        .context("地图导出没有产生结果")?;
    run(
        1,
        &[
            "partition".as_ref(),
            "--input".as_ref(),
            export.as_os_str(),
            "--output".as_ref(),
            sectors.as_os_str(),
        ],
        &mut progress,
    )?;
    run(
        2,
        &[
            "partition-metadata".as_ref(),
            "--input".as_ref(),
            export.as_os_str(),
            "--output".as_ref(),
            sectors.as_os_str(),
            "--game-dir".as_ref(),
            game.as_os_str(),
        ],
        &mut progress,
    )?;
    // The runtime reads the export manifest for the game version.
    if !sectors.join("manifest.json").is_file() {
        std::fs::copy(export.join("manifest.json"), sectors.join("manifest.json"))?;
    }
    ensure!(
        sectors.join("traffic_index.json").is_file(),
        "地图生成不完整，请重试"
    );
    if output.exists() {
        std::fs::remove_dir_all(output)
            .context("无法替换旧地图（是否正在使用？请先关闭自动驾驶）")?;
    }
    std::fs::rename(&sectors, output)?;
    let _ = std::fs::remove_dir_all(&work);
    Ok(())
}

trait HiddenWindow {
    fn creation_flags_hidden(&mut self) -> &mut Self;
}
impl HiddenWindow for Command {
    fn creation_flags_hidden(&mut self) -> &mut Self {
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            self.creation_flags(0x0800_0000);
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const CONTROLS: &str = r#"input_config : _nameless.6a5.d5f8 {
 config_lines[19]: "constant c_steer_func 0.000000"
 config_lines[35]: "constant c_rsteersens 0.400000"
 config_lines[36]: "constant c_asteersens 2.400000"
 config_lines[40]: "constant c_relatsteer 1.000000"
}"#;
    #[test]
    fn steering_mode_and_sensitivity_are_read_from_the_profile_controls() {
        assert_eq!(
            steering_mode(CONTROLS),
            Some(SteeringMode::Relative { rate_per_s: 0.4 })
        );
        let absolute = CONTROLS.replace("c_relatsteer 1.000000", "c_relatsteer 0.000000");
        assert_eq!(steering_mode(&absolute), Some(SteeringMode::Absolute));
        // Unknown when the setting is missing or out of range.
        assert_eq!(steering_mode("input_config : x {\n}"), None);
        let broken = CONTROLS.replace("c_rsteersens 0.400000", "c_rsteersens 0.000000");
        assert_eq!(steering_mode(&broken), None);
    }
    #[test]
    fn the_current_profile_is_the_last_one_the_game_log_selected() {
        let log = "00:00:22.970 : Set profile finished: 'FoyonaCZY1'\n\
                   00:00:23.004 : New profile selected: 'FoyonaCZY1'\n\
                   00:01:16.230 : Set profile finished: 'CZY'\n\
                   00:01:20.220 : Requested profile is already current profile.\n\
                   00:01:20.692 : New profile selected: 'CZY'\n\
                   00:02:08.814 : Current profile saved.\n";
        assert_eq!(current_profile(log).as_deref(), Some("CZY"));
        assert_eq!(current_profile("no profile here"), None);
        assert_eq!(profile_name("435A59"), "CZY");
        assert_eq!(profile_name("466F796F6E61435A5931"), "FoyonaCZY1");
        assert_eq!(profile_name("not-hex"), "not-hex");
    }
}
