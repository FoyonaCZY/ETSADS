use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{io::Read, path::Path};

/// Read-only diagnosis of the mode loaded by the SDK, never a driving permit.
#[derive(Clone, Debug, serde::Serialize)]
pub struct ControlStatus {
    pub sdk_mode: &'static str,
    pub label: &'static str,
    pub mode_compatible: bool,
    pub message: &'static str,
    pub generation: Option<u64>,
}
pub fn control_status() -> ControlStatus {
    use etsads_msgs::protocol::*;
    use etsads_shm::{SharedCell, SharedRing};
    let controls =
        SharedRing::<ControlConfiguration, 64>::open(CONTROL_CONFIGURATION_MAPPING, SCHEMA_VERSION)
            .ok()
            .and_then(|r| r.latest());
    let info = SharedCell::<BridgeInfo>::open(INFO_MAPPING, SCHEMA_VERSION)
        .ok()
        .and_then(|r| r.latest());
    let telemetry =
        SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(TELEMETRY_MAPPING, SCHEMA_VERSION)
            .ok()
            .and_then(|r| r.latest());
    describe_controls(
        controls.as_ref(),
        info.map_or(0, |i| i.session_start_qpc),
        telemetry.as_ref(),
        etsads_shm::qpc(),
        etsads_shm::qpc_frequency(),
    )
}
fn describe_controls(
    controls: Option<&etsads_msgs::protocol::ControlConfiguration>,
    session: u64,
    telemetry: Option<&etsads_msgs::protocol::TelemetryFrame>,
    now: u64,
    frequency: u64,
) -> ControlStatus {
    use etsads_msgs::protocol::*;
    let mut result = ControlStatus {
        sdk_mode: "unknown",
        label: "变速箱模式未知",
        mode_compatible: false,
        message: "等待当前插件报告实际控制模式；模式未知时禁止车辆控制。",
        generation: None,
    };
    let Some(t) = telemetry.filter(|t| etsads_safety::fresh(now, t.header.qpc, frequency, 500))
    else {
        result.message = "未连接新鲜游戏遥测；请进入驾驶并检查插件。";
        return result;
    };
    let Some(c) = controls.filter(|c| {
        session != 0
            && c.session_start_qpc == session
            && c.generation > 0
            && c.header.seq == c.generation
            && c.header.frame_id.checked_add(1) == Some(c.effective_from_frame_id)
            && c.header.qpc >= session
            && c.header.qpc <= t.header.qpc
            && c.header.frame_id <= t.header.frame_id
            && c.effective_from_frame_id <= t.header.frame_id
            && c.valid == CONTROL_CONFIGURATION_SHIFTER_VALID
    }) else {
        return result;
    };
    result.generation = Some(c.generation);
    (result.sdk_mode, result.label) = match c.shifter_type {
        SHIFTER_ARCADE => ("arcade", "简易自动挡 · 禁止控制"),
        SHIFTER_AUTOMATIC => ("automatic", "真实自动挡"),
        SHIFTER_MANUAL => ("manual", "序列式变速箱"),
        SHIFTER_HSHIFTER => ("hshifter", "H 挡变速箱"),
        _ => return result,
    };
    result.mode_compatible =
        etsads_safety::validate_control_configuration(Some(c), session, t, now).is_ok();
    result.message = if !result.mode_compatible {
        "简易自动挡会将刹车用于倒车。请在游戏“选项 → 控制”中选择真实自动挡。"
    } else if t.paused != 0 {
        "游戏已暂停；返回驾驶后需重新启用控制。"
    } else {
        "SDK 已确认独立油门与制动模式；运行仍需通过其余安全与标定检查。"
    };
    result
}

pub fn describe() -> Value {
    let game = crate::find_game();
    let executable = std::env::current_exe().ok();
    let runtime = json!({
        "version": env!("CARGO_PKG_VERSION"),
        "file": executable.as_ref().and_then(|path| path.file_name()).map(|name| name.to_string_lossy()),
        "sha256": executable.as_ref().and_then(|path| hash(path).ok()),
    });
    let mut plugins = Vec::new();
    let mut archives = Vec::new();
    if let Some(game) = &game {
        if let Ok(entries) = std::fs::read_dir(game.join("bin/win_x64/plugins")) {
            for file in entries
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|e| e == "dll"))
            {
                plugins.push(json!({"name":file.file_name().map(|n|n.to_string_lossy()),"sha256":hash(&file).ok()}));
            }
        }
        if let Ok(entries) = std::fs::read_dir(game) {
            for file in entries
                .filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|e| e == "scs"))
            {
                if let Ok(meta) = file.metadata() {
                    archives.push(json!({"name":file.file_name().map(|n|n.to_string_lossy()),"bytes":meta.len(),"modified_unix_s":meta.modified().ok().and_then(|t|t.duration_since(std::time::UNIX_EPOCH).ok()).map(|d|d.as_secs())}));
                }
            }
        }
    }
    let mut git_command = std::process::Command::new("git");
    git_command.args(["rev-parse", "HEAD"]);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        git_command.creation_flags(0x08000000);
    }
    let git = git_command
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned());
    json!({"os":std::env::consts::OS,"arch":std::env::consts::ARCH,"runtime":runtime,"game_version":game.as_ref().and_then(|p|file_version(&p.join("bin/win_x64/eurotrucks2.exe"))),"git_commit":git,"plugin_files":plugins,"installed_archives":archives,"active_dlc_and_mods":"not inferred from installed files; record profile activation separately"})
}
fn hash(path: &Path) -> std::io::Result<String> {
    let mut file = std::fs::File::open(path)?;
    let mut h = Sha256::new();
    let mut buffer = [0u8; 65536];
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        h.update(&buffer[..n]);
    }
    Ok(format!("{:x}", h.finalize()))
}
#[cfg(windows)]
pub fn file_version(path: &Path) -> Option<String> {
    use std::os::windows::ffi::OsStrExt;
    use windows_sys::Win32::Storage::FileSystem::*;
    let path: Vec<_> = path.as_os_str().encode_wide().chain(Some(0)).collect();
    let mut ignored = 0;
    // SAFETY: all API buffers are owned, correctly sized, and retained until
    // the version value is copied. VerQueryValue points within this buffer.
    unsafe {
        let size = GetFileVersionInfoSizeW(path.as_ptr(), &mut ignored);
        if size == 0 {
            return None;
        }
        let mut data = vec![0u8; size as usize];
        if GetFileVersionInfoW(path.as_ptr(), 0, size, data.as_mut_ptr().cast()) == 0 {
            return None;
        }
        let mut value = std::ptr::null_mut();
        let mut length = 0;
        let sub = [b'\\' as u16, 0];
        if VerQueryValueW(data.as_ptr().cast(), sub.as_ptr(), &mut value, &mut length) == 0
            || value.is_null()
            || length < std::mem::size_of::<VS_FIXEDFILEINFO>() as u32
        {
            return None;
        }
        let info = value.cast::<VS_FIXEDFILEINFO>().read_unaligned();
        if info.dwSignature != 0xFEEF04BD {
            return None;
        }
        Some(format!(
            "{}.{}.{}.{}",
            info.dwProductVersionMS >> 16,
            info.dwProductVersionMS & 65535,
            info.dwProductVersionLS >> 16,
            info.dwProductVersionLS & 65535
        ))
    }
}
#[cfg(not(windows))]
pub fn file_version(_: &Path) -> Option<String> {
    None
}

#[cfg(test)]
mod controls_status_tests {
    use super::*;
    use etsads_msgs::protocol::*;
    #[test]
    fn stale_or_previous_session_cannot_display_a_compatible_mode() {
        let c = ControlConfiguration {
            header: WireHeader {
                qpc: 110,
                seq: 1,
                ..Default::default()
            },
            session_start_qpc: 100,
            generation: 1,
            effective_from_frame_id: 1,
            shifter_type: SHIFTER_AUTOMATIC,
            valid: CONTROL_CONFIGURATION_SHIFTER_VALID,
        };
        let t = TelemetryFrame {
            header: WireHeader {
                qpc: 200,
                frame_id: 2,
                ..Default::default()
            },
            ..Default::default()
        };
        assert!(describe_controls(Some(&c), 100, Some(&t), 200, 1000).mode_compatible);
        for (session, now) in [(101, 200), (100, 701)] {
            let status = describe_controls(Some(&c), session, Some(&t), now, 1000);
            assert!(!status.mode_compatible);
            assert_eq!(status.sdk_mode, "unknown");
        }
    }
}
