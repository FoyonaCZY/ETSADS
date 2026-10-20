//! Read PE product versions; SDK game_version is deliberately not used here.
use std::path::Path;
#[cfg(windows)]
pub fn file_version(path: &Path) -> Option<String> {
    use std::os::windows::ffi::OsStrExt;
    use windows_sys::Win32::Storage::FileSystem::*;
    let path: Vec<_> = path.as_os_str().encode_wide().chain(Some(0)).collect();
    let mut ignored = 0;
    unsafe {
        let size = GetFileVersionInfoSizeW(path.as_ptr(), &mut ignored);
        if size == 0 || size > 16 * 1024 * 1024 {
            return None;
        }
        let mut bytes = vec![0u8; size as usize];
        if GetFileVersionInfoW(path.as_ptr(), 0, size, bytes.as_mut_ptr().cast()) == 0 {
            return None;
        }
        let mut value = std::ptr::null_mut();
        let mut length = 0;
        let sub = [b'\\' as u16, 0];
        if VerQueryValueW(bytes.as_ptr().cast(), sub.as_ptr(), &mut value, &mut length) == 0
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
#[cfg(windows)]
pub fn current_process_version() -> Option<String> {
    use std::os::windows::ffi::OsStringExt;
    let mut path = vec![0u16; 32768];
    let count = unsafe {
        windows_sys::Win32::System::LibraryLoader::GetModuleFileNameW(
            std::ptr::null_mut(),
            path.as_mut_ptr(),
            path.len() as u32,
        )
    } as usize;
    if count == 0 || count >= path.len() {
        return None;
    }
    file_version(Path::new(&std::ffi::OsString::from_wide(&path[..count])))
}
#[cfg(not(windows))]
pub fn current_process_version() -> Option<String> {
    None
}
pub fn extension_version_supported(version: &str) -> bool {
    let parts: Vec<_> = version.split('.').collect();
    parts.len() == 4
        && parts[0] == "1"
        && parts[1] == "61"
        && parts[2..]
            .iter()
            .all(|s| !s.is_empty() && s.parse::<u32>().is_ok())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn extension_whitelist_never_guesses_unknown_patch() {
        assert!(extension_version_supported("1.61.1.2"));
        for version in ["1.62.0.0", "1.60.1.0", "1.61", "65537", "1.61.x.0"] {
            assert!(!extension_version_supported(version));
        }
    }
}
