//! Original Rust writer for the publicly described capture v1 wire format.
//! This module shares a protocol with readers; it contains no ETS2LA capture
//! implementation. Immutable generation headers are published before selection.
use etsads_shm::{qpc, qpc_frequency, tick_count_ms, Mapping};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Transport(#[from] etsads_shm::Error),
    #[error("invalid capture dimensions or pixel buffer")]
    Invalid,
}
pub struct CapturePublisher {
    control: Mapping,
    data: Option<Mapping>,
    generation: u32,
    width: u32,
    height: u32,
    format: u32,
    slot_stride: usize,
    frame: u64,
    data_prefix: String,
    writer_name: String,
}
impl CapturePublisher {
    pub fn create() -> Result<Self, Error> {
        Self::create_named(
            "ETSADS_Capture_Control",
            "ETSADS_Capture_Data_",
            "ETSADS_Capture_Writer",
        )
    }
    /// Create a publisher in caller-selected mapping names. Integration tests
    /// must use a unique namespace so they never claim the production capture
    /// mappings or interfere with a running game's readers.
    pub fn create_named(
        control_name: &str,
        data_prefix: &str,
        writer_name: &str,
    ) -> Result<Self, Error> {
        let _lease = etsads_shm::SingleWriterGuard::acquire(writer_name)?;
        let control = Mapping::create(control_name, 4096)?;
        let mut header = [0u8; 64];
        for (o, v) in [
            (0, 0x32435445u32),
            (4, 1),
            (8, 6),
            (12, 32),
            (24, std::process::id()),
        ] {
            header[o..o + 4].copy_from_slice(&v.to_le_bytes())
        }
        header[16..24].copy_from_slice(&qpc_frequency().to_le_bytes());
        // The immutable header is initialized once. An old live mapping must
        // retain its registered readers across SDK reloads.
        if control.was_created() {
            write_bytes(&control, 0, &header)?
        }
        let generation = word32(&control, 64).load(Ordering::Acquire);
        let frame = word(&control, 72).load(Ordering::Acquire);
        Ok(Self {
            control,
            data: None,
            generation,
            width: 0,
            height: 0,
            format: 0,
            slot_stride: 0,
            frame,
            data_prefix: data_prefix.to_owned(),
            writer_name: writer_name.to_owned(),
        })
    }
    pub fn has_readers(&self) -> bool {
        let now = tick_count_ms();
        (0..32).any(|i| {
            let offset = 128 + 64 * i;
            word(&self.control, offset).load(Ordering::Acquire) != 0
                && now.saturating_sub(word(&self.control, offset + 8).load(Ordering::Acquire))
                    < 10_000
        })
    }
    pub fn publish(
        &mut self,
        width: u32,
        height: u32,
        format: u32,
        source_stride: usize,
        pixels: &[u8],
        present_qpc: u64,
    ) -> Result<u64, Error> {
        let _lease = etsads_shm::SingleWriterGuard::acquire(&self.writer_name)?;
        let row = width as usize * 4;
        if width == 0
            || height == 0
            || width > 16384
            || height > 16384
            || !matches!(format, 27 | 28 | 29 | 87 | 88 | 90 | 91 | 92 | 93)
            || source_stride < row
            || source_stride
                .checked_mul(height as usize)
                .is_none_or(|n| n > pixels.len())
        {
            return Err(Error::Invalid);
        }
        let payload = row * height as usize;
        if payload > 256 * 1024 * 1024 {
            return Err(Error::Invalid);
        }
        if self.data.is_none() || (self.width, self.height, self.format) != (width, height, format)
        {
            self.generation = self.generation.wrapping_add(1).max(1);
            self.slot_stride = (256 + payload).div_ceil(4096) * 4096;
            let total = 4096 + 6 * self.slot_stride;
            let data = Mapping::create(&format!("{}{}", self.data_prefix, self.generation), total)?;
            if !data.was_created() {
                return Err(Error::Invalid);
            }
            let mut h = [0u8; 64];
            for (o, v) in [
                (0, 0x32445445u32),
                (4, 1),
                (8, self.generation),
                (12, 6),
                (32, width),
                (36, height),
                (40, row as u32),
                (44, format),
                (48, 4),
            ] {
                h[o..o + 4].copy_from_slice(&v.to_le_bytes())
            }
            for (o, v) in [
                (16, self.slot_stride as u64),
                (24, 4096),
                (56, total as u64),
            ] {
                h[o..o + 8].copy_from_slice(&v.to_le_bytes())
            }
            write_bytes(&data, 0, &h)?;
            self.data = Some(data);
            self.width = width;
            self.height = height;
            self.format = format;
            word32(&self.control, 64).store(self.generation, Ordering::Release);
        }
        let data = self.data.as_ref().unwrap();
        let next = self.frame.wrapping_add(1);
        let offset = 4096 + (next % 6) as usize * self.slot_stride;
        let seq = word(data, offset);
        let old = seq.load(Ordering::Acquire);
        seq.store(old.wrapping_add(1), Ordering::Release);
        for (o, v) in [(8, next), (16, present_qpc), (24, qpc())] {
            write_bytes(data, offset + o, &v.to_le_bytes())?
        }
        for y in 0..height as usize {
            write_bytes(
                data,
                offset + 256 + y * row,
                &pixels[y * source_stride..y * source_stride + row],
            )?
        }
        seq.store(old.wrapping_add(2), Ordering::Release);
        self.frame = next;
        word(&self.control, 72).store(next, Ordering::Release);
        word(&self.control, 80).fetch_add(1, Ordering::Relaxed);
        word(&self.control, 96).store(qpc(), Ordering::Release);
        word32(&self.control, 68).store(1, Ordering::Release);
        Ok(next)
    }
    pub fn dropped(&self) {
        word(&self.control, 88).fetch_add(1, Ordering::Relaxed);
    }
}
impl Drop for CapturePublisher {
    fn drop(&mut self) {
        word32(&self.control, 68).store(0, Ordering::Release);
    }
}
fn write_bytes(mapping: &Mapping, offset: usize, bytes: &[u8]) -> Result<(), Error> {
    if offset
        .checked_add(bytes.len())
        .is_none_or(|n| n > mapping.len())
    {
        return Err(Error::Invalid);
    }
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), mapping.as_ptr().add(offset), bytes.len());
    }
    Ok(())
}
fn word(mapping: &Mapping, offset: usize) -> &AtomicU64 {
    assert!(offset.is_multiple_of(8) && offset + 8 <= mapping.len());
    unsafe { &*mapping.as_ptr().add(offset).cast::<AtomicU64>() }
}
fn word32(mapping: &Mapping, offset: usize) -> &AtomicU32 {
    assert!(offset.is_multiple_of(4) && offset + 4 <= mapping.len());
    unsafe { &*mapping.as_ptr().add(offset).cast::<AtomicU32>() }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    #[test]
    fn publishes_padded_rows_and_resize_generation() {
        let prefix = format!("ETSADS_PublisherTest_{}_{}", std::process::id(), qpc());
        let control = format!("{prefix}_Control");
        let data = format!("{prefix}_Data_");
        let mut publisher =
            CapturePublisher::create_named(&control, &data, &format!("{prefix}_Writer")).unwrap();
        assert!(!publisher.has_readers());
        let mut reader = etsads_game::capture::CaptureReader::connect_to(&control, &data).unwrap();
        assert!(publisher.has_readers());
        let pixels = [
            1u8, 2, 3, 4, 5, 6, 7, 8, 99, 99, 99, 99, 9, 10, 11, 12, 13, 14, 15, 16, 99, 99, 99, 99,
        ];
        let timestamp = qpc();
        publisher.publish(2, 2, 28, 12, &pixels, timestamp).unwrap();
        let frame = reader.poll().unwrap().unwrap();
        assert_eq!(frame.data, (1u8..=16).collect::<Vec<_>>());
        assert_eq!(frame.timestamp_qpc, timestamp);
        publisher.publish(3, 2, 28, 12, &pixels, qpc()).unwrap();
        let frame = reader.poll().unwrap().unwrap();
        assert_eq!((frame.width, frame.height, frame.frame_index), (3, 2, 2));
        assert_eq!(frame.data, pixels);
        assert!(publisher.publish(0, 2, 28, 12, &pixels, 300).is_err());
        drop(reader);
        assert!(!publisher.has_readers());
    }
}
