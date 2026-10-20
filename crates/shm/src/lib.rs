//! Bounded seqlock transport with an atomic payload, avoiding the data race in
//! traditional Rust `UnsafeCell<T>` seqlocks. The wire ABI is little-endian x64.
pub use bytemuck::allocation::zeroed_box;
use bytemuck::Pod;
use std::{
    marker::PhantomData,
    mem::size_of,
    sync::atomic::{AtomicU64, AtomicU8, Ordering},
};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("shared memory: {0}")]
    Io(#[from] std::io::Error),
    #[error("shared memory schema or size mismatch")]
    Schema,
    #[error("mapping name or length is invalid")]
    Invalid,
    #[error("a concurrent writer owns this slot")]
    Busy,
    #[error("Windows named mappings are unavailable on this platform")]
    Unsupported,
}
pub type Result<T> = std::result::Result<T, Error>;

/// Process-local reference implementation used by the ring and stress tests.
pub struct SeqLock<T: Pod> {
    seq: AtomicU64,
    payload: Box<[AtomicU8]>,
    marker: PhantomData<T>,
}
impl<T: Pod> SeqLock<T> {
    pub fn new(value: T) -> Self {
        Self {
            seq: AtomicU64::new(0),
            payload: bytemuck::bytes_of(&value)
                .iter()
                .map(|b| AtomicU8::new(*b))
                .collect(),
            marker: PhantomData,
        }
    }
    pub fn write(&self, value: &T) -> Result<()> {
        write_slot(&self.seq, &self.payload, value)
    }
    pub fn read(&self) -> Option<T> {
        read_slot(&self.seq, &self.payload)
    }
}
fn write_slot<T: Pod>(seq: &AtomicU64, data: &[AtomicU8], value: &T) -> Result<()> {
    let old = seq.load(Ordering::SeqCst);
    if old & 1 != 0
        || seq
            .compare_exchange(old, old.wrapping_add(1), Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
    {
        return Err(Error::Busy);
    }
    for (dst, src) in data.iter().zip(bytemuck::bytes_of(value)) {
        dst.store(*src, Ordering::SeqCst);
    }
    seq.store(old.wrapping_add(2), Ordering::SeqCst);
    Ok(())
}
fn read_slot<T: Pod>(seq: &AtomicU64, data: &[AtomicU8]) -> Option<T> {
    for _ in 0..8 {
        let before = seq.load(Ordering::SeqCst);
        if before & 1 != 0 {
            std::hint::spin_loop();
            continue;
        }
        let mut value = T::zeroed();
        for (dst, src) in bytemuck::bytes_of_mut(&mut value).iter_mut().zip(data) {
            *dst = src.load(Ordering::SeqCst);
        }
        if before == seq.load(Ordering::SeqCst) {
            return Some(value);
        }
    }
    None
}

pub struct Ring<T: Pod, const N: usize> {
    slots: [SeqLock<T>; N],
    latest: AtomicU64,
}
impl<T: Pod, const N: usize> Default for Ring<T, N> {
    fn default() -> Self {
        assert!(N > 0);
        Self {
            slots: std::array::from_fn(|_| SeqLock::new(T::zeroed())),
            latest: AtomicU64::new(0),
        }
    }
}
impl<T: Pod, const N: usize> Ring<T, N> {
    pub fn publish(&self, value: &T) -> Result<u64> {
        let next = self.latest.load(Ordering::SeqCst) + 1;
        self.slots[next as usize % N].write(value)?;
        self.latest.store(next, Ordering::SeqCst);
        Ok(next)
    }
    pub fn latest(&self) -> Option<T> {
        let n = self.latest.load(Ordering::SeqCst);
        if n == 0 {
            None
        } else {
            self.slots[n as usize % N].read()
        }
    }
}

#[cfg(windows)]
mod platform;
#[cfg(windows)]
pub use platform::{qpc, qpc_frequency, tick_count_ms, Mapping, SingleWriterGuard};
#[cfg(not(windows))]
pub struct SingleWriterGuard;
#[cfg(not(windows))]
impl SingleWriterGuard {
    pub fn acquire(_: &str) -> Result<Self> {
        Err(Error::Unsupported)
    }
}
#[cfg(not(windows))]
pub struct Mapping;
#[cfg(not(windows))]
impl Mapping {
    pub fn create(_: &str, _: usize) -> Result<Self> {
        Err(Error::Unsupported)
    }
    pub fn open(_: &str) -> Result<Self> {
        Err(Error::Unsupported)
    }
    pub fn len(&self) -> usize {
        0
    }
    pub fn is_empty(&self) -> bool {
        true
    }
    pub fn was_created(&self) -> bool {
        false
    }
    pub fn as_ptr(&self) -> *mut u8 {
        std::ptr::null_mut()
    }
    pub fn copy_bytes(&self, _: usize, _: &mut [u8]) -> Result<()> {
        Err(Error::Unsupported)
    }
}
#[cfg(not(windows))]
pub fn qpc() -> u64 {
    use std::sync::OnceLock;
    static START: OnceLock<std::time::Instant> = OnceLock::new();
    START
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_nanos() as u64
}
#[cfg(not(windows))]
pub fn qpc_frequency() -> u64 {
    1_000_000_000
}
#[cfg(not(windows))]
pub fn tick_count_ms() -> u64 {
    qpc() / 1_000_000
}

const MAGIC: u64 = 0x3153484d53414445;
const HEADER: usize = 64;
/// Mappings are zero-initialized by Windows. One writer per ring is required;
/// readers never reset memory. A schema mismatch fails closed.
pub struct SharedRing<T: Pod, const N: usize> {
    mapping: Mapping,
    marker: PhantomData<T>,
}
impl<T: Pod, const N: usize> SharedRing<T, N> {
    fn stride() -> usize {
        (8 + size_of::<T>() + 7) & !7
    }
    pub fn mapping_size() -> usize {
        HEADER + N * Self::stride()
    }
    pub fn create(name: &str, schema: u32) -> Result<Self> {
        if N == 0 || size_of::<T>() == 0 {
            return Err(Error::Invalid);
        }
        let mapping = Mapping::create(name, Self::mapping_size())?;
        let result = Self {
            mapping,
            marker: PhantomData,
        };
        if result.mapping.was_created() {
            result.word(8).store(schema as u64, Ordering::SeqCst);
            result
                .word(16)
                .store(size_of::<T>() as u64, Ordering::SeqCst);
            result.word(24).store(N as u64, Ordering::SeqCst);
            result.word(0).store(MAGIC, Ordering::SeqCst);
        }
        result.validate(schema)?;
        Ok(result)
    }
    pub fn open(name: &str, schema: u32) -> Result<Self> {
        let result = Self {
            mapping: Mapping::open(name)?,
            marker: PhantomData,
        };
        result.validate(schema)?;
        Ok(result)
    }
    fn validate(&self, schema: u32) -> Result<()> {
        if self.mapping.len() < Self::mapping_size()
            || self.word(0).load(Ordering::SeqCst) != MAGIC
            || self.word(8).load(Ordering::SeqCst) != schema as u64
            || self.word(16).load(Ordering::SeqCst) != size_of::<T>() as u64
            || self.word(24).load(Ordering::SeqCst) != N as u64
        {
            Err(Error::Schema)
        } else {
            Ok(())
        }
    }
    fn word(&self, offset: usize) -> &AtomicU64 {
        assert!(offset.is_multiple_of(8) && offset + 8 <= self.mapping.len());
        // SAFETY: Windows mapping is page-aligned; offset is aligned and in bounds.
        // Every access to each protocol word uses an AtomicU64, including init.
        unsafe { &*self.mapping.as_ptr().add(offset).cast::<AtomicU64>() }
    }
    fn slot(&self, index: u64) -> (&AtomicU64, &[AtomicU8]) {
        let offset = HEADER + index as usize % N * Self::stride();
        // SAFETY: slot range is inside validated mapping; all payload accesses
        // use AtomicU8, and no references to a non-atomic T are formed.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                self.mapping.as_ptr().add(offset + 8).cast::<AtomicU8>(),
                size_of::<T>(),
            )
        };
        (self.word(offset), bytes)
    }
    pub fn publish(&self, value: &T) -> Result<u64> {
        let latest = self.word(32);
        let next = latest.load(Ordering::SeqCst).wrapping_add(1);
        let (seq, data) = self.slot(next);
        write_slot(seq, data, value)?;
        latest.store(next, Ordering::SeqCst);
        Ok(next)
    }
    pub fn latest_index(&self) -> u64 {
        self.word(32).load(Ordering::SeqCst)
    }
    /// Undo a write that a previous, killed writer left half done. A slot whose
    /// sequence is odd would reject every later write (`Busy`) and freeze the
    /// ring forever. Rolling it back to the preceding even value restores the
    /// accounting; the torn payload belongs to an index outside the readable
    /// window, so no reader can accept it. Call only while holding the ring's
    /// single-writer guard, before the first publish.
    pub fn recover_after_writer_crash(&self) -> usize {
        let mut recovered = 0;
        for slot in 0..N as u64 {
            let (seq, _) = self.slot(slot);
            let value = seq.load(Ordering::SeqCst);
            if value & 1 == 1 {
                seq.store(value - 1, Ordering::SeqCst);
                recovered += 1;
            }
        }
        recovered
    }
    pub fn latest(&self) -> Option<T> {
        for _ in 0..8 {
            let n = self.latest_index();
            if n == 0 {
                return None;
            }
            if let Some(value) = self.read(n) {
                return Some(value);
            }
            std::hint::spin_loop();
        }
        None
    }
    pub fn read(&self, index: u64) -> Option<T> {
        let mut value = T::zeroed();
        self.read_into(index, &mut value).then_some(value)
    }
    /// Heap-backed read for large messages, avoiding repeated stack copies.
    pub fn latest_boxed(&self) -> Option<Box<T>> {
        let mut value = zeroed_box::<T>();
        for _ in 0..8 {
            let n = self.latest_index();
            if n == 0 {
                return None;
            }
            if self.read_into(n, &mut value) {
                return Some(value);
            }
            std::hint::spin_loop();
        }
        None
    }
    pub fn read_into(&self, index: u64, value: &mut T) -> bool {
        let current = self.latest_index();
        if index == 0 || index > current || current - index >= N as u64 {
            return false;
        }
        let (seq, data) = self.slot(index);
        let expected_seq = ((index - 1) / N as u64 + 1).wrapping_mul(2);
        if seq.load(Ordering::SeqCst) != expected_seq {
            return false;
        }
        for (dst, src) in bytemuck::bytes_of_mut(value).iter_mut().zip(data) {
            *dst = src.load(Ordering::SeqCst);
        }
        seq.load(Ordering::SeqCst) == expected_seq
            && self.latest_index().wrapping_sub(index) < N as u64
    }
}
pub type SharedCell<T> = SharedRing<T, 1>;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stress_atomic_snapshot_never_tears() {
        let cell = std::sync::Arc::new(SeqLock::new([0_u64; 16]));
        let writer = cell.clone();
        let handle = std::thread::spawn(move || {
            for i in 1..20_000 {
                writer.write(&[i; 16]).unwrap();
            }
        });
        while !handle.is_finished() {
            if let Some(value) = cell.read() {
                assert!(value.iter().all(|x| *x == value[0]));
            }
        }
        handle.join().unwrap();
        assert_eq!(cell.read().unwrap(), [19_999; 16]);
    }
    #[test]
    fn ring_latest() {
        let ring = Ring::<u64, 4>::default();
        assert_eq!(ring.latest(), None);
        for i in 1..100 {
            ring.publish(&i).unwrap();
            assert_eq!(ring.latest(), Some(i));
        }
    }
    #[cfg(windows)]
    #[test]
    fn writer_killed_mid_publish_is_recovered_by_the_next_writer() {
        let name = format!("ETSADS_Crash_{}_{}", std::process::id(), qpc());
        let ring = SharedRing::<[u64; 4], 2>::create(&name, 1).unwrap();
        for i in 1..=3 {
            ring.publish(&[i; 4]).unwrap();
        }
        // A killed writer leaves the next slot odd ("being written").
        let (seq, _) = ring.slot(ring.latest_index() + 1);
        seq.fetch_add(1, Ordering::SeqCst);
        assert!(matches!(ring.publish(&[9; 4]), Err(Error::Busy)));
        assert_eq!(ring.latest(), Some([3; 4]));
        let next = SharedRing::<[u64; 4], 2>::create(&name, 1).unwrap();
        assert_eq!(next.recover_after_writer_crash(), 1);
        assert_eq!(next.recover_after_writer_crash(), 0);
        for i in 4..=7 {
            next.publish(&[i; 4]).unwrap();
            assert_eq!(ring.latest(), Some([i; 4]));
        }
    }
    #[cfg(windows)]
    #[test]
    fn real_mapping_roundtrip_and_schema_rejection() {
        let name = format!("ETSADS_Test_{}_{}", std::process::id(), qpc());
        let writer = SharedRing::<[u64; 4], 8>::create(&name, 1).unwrap();
        let reader = SharedRing::<[u64; 4], 8>::open(&name, 1).unwrap();
        assert!(SharedRing::<u64, 8>::open(&name, 1).is_err());
        for i in 1..20 {
            writer.publish(&[i; 4]).unwrap();
            assert_eq!(reader.latest(), Some([i; 4]));
        }
        assert!(reader.read(1).is_none());
    }
    #[cfg(windows)]
    #[test]
    fn separate_process_reads_coherent_mapping() {
        let name = format!("ETSADS_IPC_{}_{}", std::process::id(), qpc());
        let writer = SharedRing::<[u64; 4], 8>::create(&name, 1).unwrap();
        let ack = SharedCell::<u64>::create(&format!("{name}_Ack"), 1).unwrap();
        let _writer_guard = SingleWriterGuard::acquire(&format!("{name}_Guard")).unwrap();
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "tests::ipc_reader_process", "--ignored"])
            .env("ETSADS_SHM_IPC_TEST", &name)
            .spawn()
            .unwrap();
        for i in 1..=10_000 {
            writer.publish(&[i, !i, i.wrapping_mul(7), i]).unwrap();
        }
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while ack.latest() != Some(10_000) && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        let observed = ack.latest();
        if observed != Some(10_000) {
            let _ = child.kill();
        }
        let status = child.wait().unwrap();
        assert_eq!(observed, Some(10_000));
        assert!(status.success());
    }
    #[cfg(windows)]
    #[test]
    #[ignore = "helper invoked by separate_process_reads_coherent_mapping"]
    fn ipc_reader_process() {
        let name = std::env::var("ETSADS_SHM_IPC_TEST").expect("child-only invocation");
        assert!(SingleWriterGuard::acquire(&format!("{name}_Guard")).is_err());
        let reader = SharedRing::<[u64; 4], 8>::open(&name, 1).unwrap();
        let ack = SharedCell::<u64>::open(&format!("{name}_Ack"), 1).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while std::time::Instant::now() < deadline {
            if let Some(v) = reader.latest() {
                assert_eq!(v[1], !v[0]);
                assert_eq!(v[2], v[0] * 7);
                assert_eq!(v[3], v[0]);
                ack.publish(&v[0]).unwrap();
                if v[0] == 10_000 {
                    return;
                }
            }
            std::thread::yield_now();
        }
        panic!("IPC producer did not publish terminal frame");
    }
    #[cfg(windows)]
    #[test]
    fn writer_guard_rejects_recursive_ownership_and_releases_on_drop() {
        let name = format!("ETSADS_Guard_{}_{}", std::process::id(), qpc());
        let guard = SingleWriterGuard::acquire(&name).unwrap();
        assert!(matches!(
            SingleWriterGuard::acquire(&name),
            Err(Error::Busy)
        ));
        drop(guard);
        assert!(SingleWriterGuard::acquire(&name).is_ok());
    }
}
