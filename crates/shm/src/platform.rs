use super::{Error, Result};
use std::ptr::{null, NonNull};
use std::{
    collections::HashSet,
    marker::PhantomData,
    rc::Rc,
    sync::{Mutex, OnceLock},
};
use windows_sys::Win32::{
    Foundation::{CloseHandle, GetLastError, ERROR_ALREADY_EXISTS, HANDLE, INVALID_HANDLE_VALUE},
    System::{
        Memory::{
            CreateFileMappingW, MapViewOfFile, OpenFileMappingW, UnmapViewOfFile, VirtualQuery,
            FILE_MAP_ALL_ACCESS, MEMORY_BASIC_INFORMATION, MEMORY_MAPPED_VIEW_ADDRESS,
            PAGE_READWRITE,
        },
        Performance::{QueryPerformanceCounter, QueryPerformanceFrequency},
    },
};

pub struct Mapping {
    handle: HANDLE,
    pointer: NonNull<u8>,
    length: usize,
    created: bool,
}

fn owned_mutexes() -> &'static Mutex<HashSet<String>> {
    static OWNED: OnceLock<Mutex<HashSet<String>>> = OnceLock::new();
    OWNED.get_or_init(|| Mutex::new(HashSet::new()))
}
/// Exclusive named writer lease. Windows mutexes are owned by a thread, so the
/// guard is deliberately !Send and !Sync. Abandoned ownership is recoverable.
pub struct SingleWriterGuard {
    handle: HANDLE,
    name: String,
    _thread_owned: PhantomData<Rc<()>>,
}
impl SingleWriterGuard {
    pub fn acquire(name: &str) -> Result<Self> {
        use windows_sys::Win32::{
            Foundation::{WAIT_ABANDONED, WAIT_OBJECT_0, WAIT_TIMEOUT},
            System::Threading::{CreateMutexW, WaitForSingleObject},
        };
        let wide = Mapping::name(name)?;
        let key = String::from_utf16_lossy(&wide[..wide.len() - 1]);
        let mut owned = owned_mutexes().lock().unwrap_or_else(|p| p.into_inner());
        // Win32 mutexes are recursive for the same thread; a second guard must
        // still fail, otherwise two writers could be started in one process.
        if owned.contains(&key) {
            return Err(Error::Busy);
        }
        let handle = unsafe { CreateMutexW(null(), 0, wide.as_ptr()) };
        if handle.is_null() {
            return Err(std::io::Error::last_os_error().into());
        }
        let result = unsafe { WaitForSingleObject(handle, 0) };
        if result != WAIT_OBJECT_0 && result != WAIT_ABANDONED {
            let error = std::io::Error::last_os_error();
            unsafe {
                CloseHandle(handle);
            }
            return Err(if result == WAIT_TIMEOUT {
                Error::Busy
            } else {
                Error::Io(error)
            });
        }
        owned.insert(key.clone());
        Ok(Self {
            handle,
            name: key,
            _thread_owned: PhantomData,
        })
    }
}
impl Drop for SingleWriterGuard {
    fn drop(&mut self) {
        let mut owned = owned_mutexes().lock().unwrap_or_else(|p| p.into_inner());
        unsafe {
            windows_sys::Win32::System::Threading::ReleaseMutex(self.handle);
            CloseHandle(self.handle);
        }
        owned.remove(&self.name);
    }
}
// SAFETY: mapping ownership is independent of the thread; protocol users use
// atomics. Raw pointer access is explicitly unsafe to dereference.
unsafe impl Send for Mapping {}
unsafe impl Sync for Mapping {}
impl Mapping {
    fn name(name: &str) -> Result<Vec<u16>> {
        if name.is_empty() || name.contains('\0') || name.starts_with("Global\\") {
            return Err(Error::Invalid);
        }
        let name = if name.starts_with("Local\\") {
            name.to_owned()
        } else {
            format!("Local\\{name}")
        };
        Ok(name.encode_utf16().chain(Some(0)).collect())
    }
    pub fn create(name: &str, size: usize) -> Result<Self> {
        if size == 0 {
            return Err(Error::Invalid);
        }
        let name = Self::name(name)?;
        // SAFETY: name is NUL-terminated and size is nonzero; pagefile-backed.
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                null(),
                PAGE_READWRITE,
                ((size as u64) >> 32) as u32,
                size as u32,
                name.as_ptr(),
            )
        };
        if handle.is_null() {
            return Err(std::io::Error::last_os_error().into());
        }
        let created = unsafe { GetLastError() } != ERROR_ALREADY_EXISTS;
        Self::map(handle, created)
    }
    pub fn open(name: &str) -> Result<Self> {
        let name = Self::name(name)?;
        let handle = unsafe { OpenFileMappingW(FILE_MAP_ALL_ACCESS, 0, name.as_ptr()) };
        if handle.is_null() {
            return Err(std::io::Error::last_os_error().into());
        }
        Self::map(handle, false)
    }
    fn map(handle: HANDLE, created: bool) -> Result<Self> {
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, 0) };
        let Some(pointer) = NonNull::new(view.Value.cast::<u8>()) else {
            unsafe {
                CloseHandle(handle);
            }
            return Err(std::io::Error::last_os_error().into());
        };
        let mut info: MEMORY_BASIC_INFORMATION = unsafe { std::mem::zeroed() };
        if unsafe {
            VirtualQuery(
                pointer.as_ptr().cast(),
                &mut info,
                std::mem::size_of_val(&info),
            )
        } == 0
        {
            unsafe {
                UnmapViewOfFile(view);
                CloseHandle(handle);
            }
            return Err(std::io::Error::last_os_error().into());
        }
        Ok(Self {
            handle,
            pointer,
            length: info.RegionSize,
            created,
        })
    }
    pub fn len(&self) -> usize {
        self.length
    }
    pub fn is_empty(&self) -> bool {
        self.length == 0
    }
    pub fn was_created(&self) -> bool {
        self.created
    }
    pub fn as_ptr(&self) -> *mut u8 {
        self.pointer.as_ptr()
    }
    /// Copy foreign-writer bytes through the OS, without constructing a Rust
    /// reference to concurrently modified non-atomic memory. Caller must still
    /// validate the producer's generation/seqlock around this copy.
    pub fn copy_bytes(&self, offset: usize, dst: &mut [u8]) -> Result<()> {
        if offset
            .checked_add(dst.len())
            .is_none_or(|end| end > self.length)
        {
            return Err(Error::Invalid);
        }
        let mut read = 0;
        let success = unsafe {
            windows_sys::Win32::System::Diagnostics::Debug::ReadProcessMemory(
                windows_sys::Win32::System::Threading::GetCurrentProcess(),
                self.pointer.as_ptr().add(offset).cast(),
                dst.as_mut_ptr().cast(),
                dst.len(),
                &mut read,
            )
        };
        if success == 0 || read != dst.len() {
            return Err(std::io::Error::last_os_error().into());
        }
        Ok(())
    }
}
impl Drop for Mapping {
    fn drop(&mut self) {
        unsafe {
            UnmapViewOfFile(MEMORY_MAPPED_VIEW_ADDRESS {
                Value: self.pointer.as_ptr().cast(),
            });
            CloseHandle(self.handle);
        }
    }
}
pub fn qpc() -> u64 {
    let mut value = 0;
    unsafe {
        QueryPerformanceCounter(&mut value);
    }
    value as u64
}
pub fn qpc_frequency() -> u64 {
    let mut value = 0;
    unsafe {
        QueryPerformanceFrequency(&mut value);
    }
    value as u64
}
pub fn tick_count_ms() -> u64 {
    unsafe { windows_sys::Win32::System::SystemInformation::GetTickCount64() }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn abandoned_owner_can_be_recovered() {
        let name = format!("ETSADS_Abandoned_{}_{}", std::process::id(), qpc());
        let wide = Mapping::name(&name).unwrap();
        // Keep the kernel object alive after its owning thread exits without
        // releasing it. The handle itself is transferred to this test thread.
        let keeper = std::thread::spawn(move || {
            let handle = unsafe {
                windows_sys::Win32::System::Threading::CreateMutexW(null(), 1, wide.as_ptr())
            };
            assert!(!handle.is_null());
            handle as usize
        })
        .join()
        .unwrap() as HANDLE;
        let guard = SingleWriterGuard::acquire(&name).unwrap();
        unsafe {
            CloseHandle(keeper);
        }
        assert!(SingleWriterGuard::acquire(&name).is_err());
        drop(guard);
        assert!(SingleWriterGuard::acquire(&name).is_ok());
    }
}
