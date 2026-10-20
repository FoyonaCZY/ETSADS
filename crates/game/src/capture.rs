//! Owned-frame reader for ETS2LA/capture protocol v1. Offsets are verified from
//! upstream include/ets2la_capture/protocol.h; no zero-copy borrowed pixel view
//! is exposed because the producer may recycle a slot during processing.
use etsads_shm::{tick_count_ms, Mapping};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Transport(#[from] etsads_shm::Error),
    #[error("capture protocol mismatch: {0}")]
    Protocol(&'static str),
    #[error("all 32 capture reader slots are occupied")]
    ReadersFull,
}
#[derive(Clone, Debug)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub format: u32,
    pub frame_index: u64,
    pub timestamp_qpc: u64,
    pub publish_qpc: u64,
    pub data: Vec<u8>,
}
/// Validated capture dimensions and timing without copying the pixel payload.
#[derive(Clone, Copy, Debug)]
pub struct FrameMetadata {
    pub width: u32,
    pub height: u32,
    pub format: u32,
    pub generation: u32,
    pub frame_index: u64,
    pub timestamp_qpc: u64,
    pub publish_qpc: u64,
}
pub struct CaptureReader {
    control: Mapping,
    data: Option<Mapping>,
    generation: u32,
    reader_offset: usize,
    reader_id: u64,
    last_frame: u64,
    data_prefix: String,
    pub qpc_frequency: u64,
}
impl CaptureReader {
    pub fn connect() -> Result<Self, Error> {
        Self::connect_to("ETSADS_Capture_Control", "ETSADS_Capture_Data_")
            .or_else(|_| Self::connect_to("ETS2LA_Capture_Control", "ETS2LA_Capture_Data_"))
    }
    pub fn connect_to(control_name: &str, data_prefix: &str) -> Result<Self, Error> {
        let control = Mapping::open(control_name)?;
        if control.len() < 4096 {
            return Err(Error::Protocol("short control mapping"));
        }
        let mut header = [0; 32];
        control.copy_bytes(0, &mut header)?;
        if u32_at(&header, 0) != 0x32435445
            || u32_at(&header, 4) != 1
            || u32_at(&header, 8) != 6
            || u32_at(&header, 12) != 32
        {
            return Err(Error::Protocol("control header"));
        }
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        let reader_id = ((std::process::id() as u64) << 32)
            | COUNTER.fetch_add(1, Ordering::Relaxed).wrapping_add(1) as u64;
        let now = tick_count_ms();
        let mut reader_offset = None;
        for i in 0..32 {
            let offset = 128 + 64 * ((i + std::process::id() as usize) % 32);
            let id = word(&control, offset);
            let old = id.load(Ordering::Acquire);
            let heartbeat = word(&control, offset + 8).load(Ordering::Acquire);
            if (old == 0 || now.saturating_sub(heartbeat) > 20_000)
                && id
                    .compare_exchange(old, reader_id, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            {
                word(&control, offset + 8).store(now, Ordering::Release);
                reader_offset = Some(offset);
                break;
            }
        }
        Ok(Self {
            qpc_frequency: u64_at(&header, 16),
            control,
            data: None,
            generation: 0,
            reader_offset: reader_offset.ok_or(Error::ReadersFull)?,
            reader_id,
            last_frame: 0,
            data_prefix: data_prefix.to_owned(),
        })
    }
    /// Call at least every 250 ms while recording; a call also renews the reader lease.
    pub fn poll(&mut self) -> Result<Option<Frame>, Error> {
        self.poll_impl(true)
    }
    /// Consumes this reader's latest frame index. Use a separate reader when
    /// another consumer also needs pixels from the same frame.
    pub fn poll_metadata(&mut self) -> Result<Option<FrameMetadata>, Error> {
        Ok(self.poll_impl(false)?.map(|frame| FrameMetadata {
            width: frame.width,
            height: frame.height,
            format: frame.format,
            generation: self.generation,
            frame_index: frame.frame_index,
            timestamp_qpc: frame.timestamp_qpc,
            publish_qpc: frame.publish_qpc,
        }))
    }
    fn poll_impl(&mut self, include_pixels: bool) -> Result<Option<Frame>, Error> {
        if word(&self.control, self.reader_offset).load(Ordering::Acquire) != self.reader_id {
            return Err(Error::Protocol("reader lease was replaced"));
        }
        word(&self.control, self.reader_offset + 8).store(tick_count_ms(), Ordering::Release);
        let generation = word32(&self.control, 64).load(Ordering::Acquire);
        let latest = word(&self.control, 72).load(Ordering::Acquire);
        if latest == 0 || latest == self.last_frame {
            return Ok(None);
        }
        if self.data.is_none() || generation != self.generation {
            self.data = Some(Mapping::open(&format!("{}{generation}", self.data_prefix))?);
            self.generation = generation;
        }
        let data = self.data.as_ref().unwrap();
        let mut header = [0; 64];
        data.copy_bytes(0, &mut header)?;
        let layout = Layout::parse(&header, data.len(), generation)?;
        let offset = layout.slots_offset + (latest % 6) as usize * layout.slot_stride;
        for _ in 0..3 {
            let before = word(data, offset).load(Ordering::Acquire);
            if before & 1 != 0 {
                continue;
            }
            let mut slot = [0; 24];
            data.copy_bytes(offset + 8, &mut slot)?;
            if u64_at(&slot, 0) != latest {
                continue;
            }
            let timestamp = u64_at(&slot, 8);
            let now = etsads_shm::qpc();
            // A mapping can retain a menu image for minutes while no reader is
            // registered. Never label such a cache entry as a new live image.
            if timestamp == 0
                || self.qpc_frequency == 0
                || timestamp > now
                || now - timestamp > self.qpc_frequency / 2
            {
                return Ok(None);
            }
            let mut pixels = Vec::new();
            if include_pixels {
                pixels.resize(layout.payload, 0);
                data.copy_bytes(offset + 256, &mut pixels)?;
            }
            std::sync::atomic::fence(Ordering::Acquire);
            if word(data, offset).load(Ordering::Acquire) != before
                || word32(&self.control, 64).load(Ordering::Acquire) != generation
            {
                continue;
            }
            self.last_frame = latest;
            return Ok(Some(Frame {
                width: layout.width,
                height: layout.height,
                stride: layout.stride,
                format: layout.format,
                frame_index: latest,
                timestamp_qpc: u64_at(&slot, 8),
                publish_qpc: u64_at(&slot, 16),
                data: pixels,
            }));
        }
        Ok(None)
    }
}
impl Drop for CaptureReader {
    fn drop(&mut self) {
        let id = word(&self.control, self.reader_offset);
        let _ = id.compare_exchange(self.reader_id, 0, Ordering::AcqRel, Ordering::Acquire);
    }
}
struct Layout {
    width: u32,
    height: u32,
    stride: u32,
    format: u32,
    slot_stride: usize,
    slots_offset: usize,
    payload: usize,
}
impl Layout {
    fn parse(h: &[u8; 64], mapped: usize, generation: u32) -> Result<Self, Error> {
        if u32_at(h, 0) != 0x32445445
            || u32_at(h, 4) != 1
            || u32_at(h, 8) != generation
            || u32_at(h, 12) != 6
        {
            return Err(Error::Protocol("data header"));
        }
        let width = u32_at(h, 32);
        let height = u32_at(h, 36);
        let stride = u32_at(h, 40);
        let format = u32_at(h, 44);
        let bpp = u32_at(h, 48);
        let slot_stride = u64_at(h, 16) as usize;
        let slots_offset = u64_at(h, 24) as usize;
        let payload = (stride as usize)
            .checked_mul(height as usize)
            .ok_or(Error::Protocol("dimensions overflow"))?;
        let total = slots_offset
            .checked_add(
                slot_stride
                    .checked_mul(6)
                    .ok_or(Error::Protocol("slot size overflow"))?,
            )
            .ok_or(Error::Protocol("mapping size overflow"))?;
        if width == 0
            || height == 0
            || width > 16384
            || height > 16384
            || bpp != 4
            || !(matches!(format, 87 | 90 | 91 | 88 | 92 | 93 | 27 | 28 | 29))
            || stride < width * 4
            || stride > width * 4 + 65536
            || payload > 256 * 1024 * 1024
            || slots_offset != 4096
            || slot_stride < 256 + payload
            || !slot_stride.is_multiple_of(8)
            || total > mapped
            || total > u64_at(h, 56) as usize
        {
            return Err(Error::Protocol("invalid dimensions/format/slot range"));
        }
        Ok(Self {
            width,
            height,
            stride,
            format,
            slot_stride,
            slots_offset,
            payload,
        })
    }
}
fn word(mapping: &Mapping, offset: usize) -> &AtomicU64 {
    assert!(offset.is_multiple_of(8) && offset + 8 <= mapping.len());
    unsafe { &*mapping.as_ptr().add(offset).cast::<AtomicU64>() }
}
fn word32(mapping: &Mapping, offset: usize) -> &AtomicU32 {
    assert!(offset.is_multiple_of(4) && offset + 4 <= mapping.len());
    unsafe { &*mapping.as_ptr().add(offset).cast::<AtomicU32>() }
}
fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}
fn u64_at(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_wrong_layout_before_pixel_access() {
        let mut h = [0_u8; 64];
        for (o, v) in [
            (0, 0x32445445_u32),
            (4, 1),
            (8, 2),
            (12, 6),
            (32, 1920),
            (36, 1080),
            (40, 7680),
            (44, 87),
            (48, 4),
        ] {
            h[o..o + 4].copy_from_slice(&v.to_le_bytes())
        }
        let stride = (256_usize + 1920 * 1080 * 4).div_ceil(4096) * 4096;
        for (o, v) in [
            (16, stride as u64),
            (24, 4096),
            (56, (4096 + 6 * stride) as u64),
        ] {
            h[o..o + 8].copy_from_slice(&v.to_le_bytes())
        }
        assert!(Layout::parse(&h, 4096 + 6 * stride, 2).is_ok());
        assert!(Layout::parse(&h, 4096, 2).is_err());
        h[44..48].copy_from_slice(&0_u32.to_le_bytes());
        assert!(Layout::parse(&h, 4096 + 6 * stride, 2).is_err());
    }
    #[cfg(windows)]
    #[test]
    fn real_capture_mapping_registers_reads_and_releases_reader() {
        let prefix = format!(
            "ETSADS_CaptureTest_{}_{}",
            std::process::id(),
            etsads_shm::qpc()
        );
        let control = Mapping::create(&format!("{prefix}_Control"), 4096).unwrap();
        let data = Mapping::create(&format!("{prefix}_Data_1"), 4096 * 7).unwrap();
        let mut h = [0u8; 64];
        for (o, v) in [(0, 0x32435445u32), (4, 1), (8, 6), (12, 32)] {
            h[o..o + 4].copy_from_slice(&v.to_le_bytes());
        }
        h[16..24].copy_from_slice(&etsads_shm::qpc_frequency().to_le_bytes());
        unsafe {
            std::ptr::copy_nonoverlapping(h.as_ptr(), control.as_ptr(), h.len());
        }
        h.fill(0);
        for (o, v) in [
            (0, 0x32445445u32),
            (4, 1),
            (8, 1),
            (12, 6),
            (32, 2),
            (36, 2),
            (40, 8),
            (44, 87),
            (48, 4),
        ] {
            h[o..o + 4].copy_from_slice(&v.to_le_bytes());
        }
        for (o, v) in [(16, 4096_u64), (24, 4096), (56, 4096 * 7)] {
            h[o..o + 8].copy_from_slice(&v.to_le_bytes());
        }
        unsafe {
            std::ptr::copy_nonoverlapping(h.as_ptr(), data.as_ptr(), h.len());
        }
        let offset = 8192;
        let present_qpc = etsads_shm::qpc();
        for (o, v) in [(8, 1_u64), (16, present_qpc), (24, present_qpc)] {
            unsafe {
                std::ptr::copy_nonoverlapping(
                    v.to_le_bytes().as_ptr(),
                    data.as_ptr().add(offset + o),
                    8,
                );
            }
        }
        let pixels: [u8; 16] = std::array::from_fn(|i| i as u8);
        unsafe {
            std::ptr::copy_nonoverlapping(pixels.as_ptr(), data.as_ptr().add(offset + 256), 16);
        }
        word(&data, offset).store(2, Ordering::Release);
        word32(&control, 64).store(1, Ordering::Release);
        word(&control, 72).store(1, Ordering::Release);
        let mut reader =
            CaptureReader::connect_to(&format!("{prefix}_Control"), &format!("{prefix}_Data_"))
                .unwrap();
        let reader_offset = reader.reader_offset;
        assert_ne!(word(&control, reader_offset).load(Ordering::Acquire), 0);
        let frame = reader.poll().unwrap().unwrap();
        assert_eq!(frame.data, pixels);
        assert_eq!(
            (frame.width, frame.height, frame.timestamp_qpc),
            (2, 2, present_qpc)
        );
        assert!(reader.poll().unwrap().is_none());
        reader.last_frame = 0;
        let metadata = reader.poll_metadata().unwrap().unwrap();
        assert_eq!(
            (metadata.width, metadata.height, metadata.generation),
            (2, 2, 1)
        );
        assert_eq!(metadata.timestamp_qpc, present_qpc);
        assert!(reader.poll_metadata().unwrap().is_none());
        reader.last_frame = 0;
        for timestamp in [
            etsads_shm::qpc().saturating_sub(etsads_shm::qpc_frequency()),
            etsads_shm::qpc().saturating_add(etsads_shm::qpc_frequency()),
        ] {
            let seq = word(&data, offset).load(Ordering::Acquire);
            word(&data, offset).store(seq + 1, Ordering::Release);
            unsafe {
                std::ptr::copy_nonoverlapping(
                    timestamp.to_le_bytes().as_ptr(),
                    data.as_ptr().add(offset + 16),
                    8,
                );
            }
            word(&data, offset).store(seq + 2, Ordering::Release);
            assert!(
                reader.poll().unwrap().is_none(),
                "stale or future capture must be rejected"
            );
            assert!(reader.poll_metadata().unwrap().is_none());
        }
        // A published index with an odd sequence must never leak a partial frame.
        word(&control, 72).store(2, Ordering::Release);
        word(&data, 12288).store(1, Ordering::Release);
        assert!(reader.poll().unwrap().is_none());
        drop(reader);
        assert_eq!(word(&control, reader_offset).load(Ordering::Acquire), 0);
    }
}
