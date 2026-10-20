//! Capture and NVENC run on a separate worker so a slow encoder cannot block
//! telemetry recording or the control loop. Every accepted frame has an index.
use anyhow::{Context, ensure};
use etsads_game::capture::CaptureReader;
use etsads_msgs::protocol::*;
use etsads_shm::SharedRing;
use std::{
    io::Write,
    path::Path,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    thread,
    time::Duration,
};

pub use etsads_msgs::protocol::VideoIndex;
type EncoderProcess = Arc<Mutex<Option<std::process::Child>>>;
pub struct VideoSession {
    stop: Arc<AtomicBool>,
    pub indexes: mpsc::Receiver<VideoIndex>,
    worker: Option<thread::JoinHandle<anyhow::Result<u64>>>,
    encoder: EncoderProcess,
    progress: Arc<AtomicU64>,
    done: Arc<AtomicBool>,
    watchdog: Mutex<Option<thread::JoinHandle<()>>>,
}
impl VideoSession {
    pub fn start(directory: &Path) -> anyhow::Result<Self> {
        let capture = CaptureReader::connect().context("video requires the capture plugin")?;
        let telemetry =
            SharedRing::<TelemetryFrame, TELEMETRY_SLOTS>::open(TELEMETRY_MAPPING, SCHEMA_VERSION)?;
        let directory = directory.to_owned();
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let (tx, indexes) = mpsc::sync_channel(512);
        let encoder = Arc::new(Mutex::new(None));
        let worker_encoder = encoder.clone();
        let progress = Arc::new(AtomicU64::new(0));
        let worker_progress = progress.clone();
        let done = Arc::new(AtomicBool::new(false));
        let worker_done = done.clone();
        let worker = thread::spawn(move || {
            struct Done(Arc<AtomicBool>);
            impl Drop for Done {
                fn drop(&mut self) {
                    self.0.store(true, Ordering::Release);
                }
            }
            let _done = Done(worker_done);
            encode(
                capture,
                telemetry,
                &directory,
                &worker_stop,
                tx,
                worker_encoder,
                &worker_progress,
            )
        });
        Ok(Self {
            stop,
            indexes,
            worker: Some(worker),
            encoder,
            progress,
            done,
            watchdog: Mutex::new(None),
        })
    }
    pub fn request_stop(&self) {
        if !self.stop.swap(true, Ordering::AcqRel) {
            // Killing our own child closes its pipe and unblocks a stalled
            // write_all. No mutex is held during video writes or process waits.
            let encoder = self.encoder.clone();
            let done = self.done.clone();
            let watchdog = thread::spawn(move || {
                let deadline = std::time::Instant::now() + Duration::from_secs(2);
                while !done.load(Ordering::Acquire) && std::time::Instant::now() < deadline {
                    thread::sleep(Duration::from_millis(10));
                }
                if let Ok(mut slot) = encoder.lock()
                    && let Some(child) = slot.as_mut()
                    && child.try_wait().ok().flatten().is_none()
                {
                    let _ = child.kill();
                }
            });
            *self.watchdog.lock().unwrap_or_else(|p| p.into_inner()) = Some(watchdog);
        }
    }
    pub fn progress_qpc(&self) -> u64 {
        self.progress.load(Ordering::Acquire)
    }
    pub fn is_finished(&self) -> bool {
        self.worker.as_ref().is_none_or(|w| w.is_finished())
    }
    pub fn finish(mut self) -> anyhow::Result<u64> {
        self.request_stop();
        let joined = self
            .worker
            .take()
            .unwrap()
            .join()
            .map_err(|_| anyhow::anyhow!("video worker panicked"));
        let cleanup = self.cleanup_encoder();
        if let Some(watchdog) = self
            .watchdog
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            let _ = watchdog.join();
        }
        let result = joined?;
        cleanup?;
        result
    }
    fn cleanup_encoder(&self) -> anyhow::Result<()> {
        let child = self
            .encoder
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some(mut child) = child
            && !matches!(child.try_wait(), Ok(Some(_)))
        {
            let _ = child.kill();
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while child.try_wait()?.is_none() {
                ensure!(
                    std::time::Instant::now() < deadline,
                    "ffmpeg did not exit after termination request"
                );
                thread::sleep(Duration::from_millis(10));
            }
        }
        Ok(())
    }
}

impl Drop for VideoSession {
    fn drop(&mut self) {
        self.request_stop();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
        let _ = self.cleanup_encoder();
        if let Some(watchdog) = self
            .watchdog
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            let _ = watchdog.join();
        }
    }
}
fn encode(
    mut capture: CaptureReader,
    telemetry: SharedRing<TelemetryFrame, TELEMETRY_SLOTS>,
    directory: &Path,
    stop: &AtomicBool,
    tx: mpsc::SyncSender<VideoIndex>,
    encoder: EncoderProcess,
    progress: &AtomicU64,
) -> anyhow::Result<u64> {
    let mut encoder_input: Option<std::process::ChildStdin> = None;
    let mut size = (0, 0);
    let mut number = 0;
    let mut last_capture = 0;
    let mut last_frame = std::time::Instant::now();
    let result = (|| -> anyhow::Result<()> {
        while !stop.load(Ordering::Relaxed) {
            let Some(frame) = capture.poll()? else {
                ensure!(
                    last_frame.elapsed() < Duration::from_secs(10),
                    "capture stalled for 10 seconds"
                );
                thread::sleep(Duration::from_millis(3));
                continue;
            };
            last_frame = std::time::Instant::now();
            if encoder_input.is_none() {
                size = (frame.width, frame.height);
                let logfile = std::fs::File::create(directory.join("video-ffmpeg.log"))?;
                let pixel_format = if matches!(frame.format, 87 | 90 | 91 | 88 | 92 | 93) {
                    "bgra"
                } else {
                    "rgba"
                };
                let mut command = std::process::Command::new(
                    std::env::var_os("ETSADS_FFMPEG").unwrap_or_else(|| "ffmpeg".into()),
                );
                command
                    .args([
                        "-hide_banner",
                        "-loglevel",
                        "warning",
                        "-nostdin",
                        "-n",
                        "-f",
                        "rawvideo",
                        "-pix_fmt",
                        pixel_format,
                        "-s",
                        &format!("{}x{}", frame.width, frame.height),
                        "-r",
                        "60",
                        "-i",
                        "pipe:0",
                        "-an",
                        "-c:v",
                        "hevc_nvenc",
                        "-preset",
                        "p4",
                        "-cq",
                        "23",
                        "-f",
                        "segment",
                        "-segment_time",
                        "1800",
                        "-reset_timestamps",
                        "1",
                    ])
                    .arg(directory.join("video-%04d.mp4"))
                    .stdin(std::process::Stdio::piped())
                    .stdout(std::process::Stdio::null())
                    .stderr(logfile);
                #[cfg(windows)]
                {
                    use std::os::windows::process::CommandExt;
                    command.creation_flags(0x08000000);
                }
                let mut child = command
                    .spawn()
                    .context("cannot launch ffmpeg (set ETSADS_FFMPEG)")?;
                encoder_input = child.stdin.take();
                *encoder
                    .lock()
                    .map_err(|_| anyhow::anyhow!("encoder process lock poisoned"))? = Some(child);
            }
            ensure!(
                size == (frame.width, frame.height),
                "resolution changed during recording; start a new run"
            );
            let latest = telemetry.latest_index();
            let closest = (latest.saturating_sub(TELEMETRY_SLOTS as u64 - 1).max(1)..=latest)
                .filter_map(|i| telemetry.read(i))
                .min_by_key(|t| t.header.qpc.abs_diff(frame.timestamp_qpc));
            let t = closest.context("no physics frame available for video alignment")?;
            if stop.load(Ordering::Acquire) {
                break;
            }
            let input = encoder_input.as_mut().context("ffmpeg input closed")?;
            let row_len = frame.width as usize * 4;
            if frame.stride as usize == row_len {
                input.write_all(&frame.data)?;
            } else {
                for row in frame
                    .data
                    .chunks_exact(frame.stride as usize)
                    .take(frame.height as usize)
                {
                    input.write_all(&row[..row_len])?;
                }
            }
            let index = VideoIndex {
                video_frame: number,
                capture_frame: frame.frame_index,
                capture_qpc: frame.timestamp_qpc,
                physics_frame: t.header.frame_id,
                physics_qpc: t.header.qpc,
                sim_time_us: t.header.sim_time_us,
                dropped_before: if last_capture == 0 {
                    0
                } else {
                    frame.frame_index.saturating_sub(last_capture + 1)
                },
                alignment_error_qpc: t.header.qpc.abs_diff(frame.timestamp_qpc),
            };
            tx.try_send(index)
                .context("video-index queue full or recorder disconnected")?;
            progress.store(etsads_shm::qpc(), Ordering::Release);
            number += 1;
            last_capture = frame.frame_index;
        }
        Ok(())
    })();
    drop(encoder_input.take());
    let wait = std::time::Instant::now();
    loop {
        let finished = {
            let mut slot = encoder
                .lock()
                .map_err(|_| anyhow::anyhow!("encoder process lock poisoned"))?;
            if let Some(child) = slot.as_mut() {
                if wait.elapsed() > Duration::from_secs(3) {
                    let _ = child.kill();
                }
                child.try_wait()?
            } else {
                break;
            }
        };
        if let Some(exit) = finished {
            ensure!(
                exit.success(),
                "NVENC failed or timed out; see video-ffmpeg.log"
            );
            break;
        }
        ensure!(
            wait.elapsed() < Duration::from_secs(4),
            "ffmpeg did not exit after EOF and termination request"
        );
        thread::sleep(Duration::from_millis(10));
    }
    result?;
    ensure!(number > 0, "no video frames recorded");
    Ok(number)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dropping_session_joins_worker_and_watchdog() {
        let stop = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let worker_done = done.clone();
        let worker = thread::spawn(move || {
            while !worker_stop.load(Ordering::Acquire) {
                thread::sleep(Duration::from_millis(1));
            }
            thread::sleep(Duration::from_millis(50));
            worker_done.store(true, Ordering::Release);
            Ok(0)
        });
        let (_tx, indexes) = mpsc::sync_channel(1);
        let encoder = Arc::new(Mutex::new(None));
        let session = VideoSession {
            stop,
            indexes,
            worker: Some(worker),
            encoder: encoder.clone(),
            progress: Arc::new(AtomicU64::new(0)),
            done: done.clone(),
            watchdog: Mutex::new(None),
        };
        drop(session);
        assert!(done.load(Ordering::Acquire));
        assert!(encoder.lock().unwrap().is_none());
    }

    #[test]
    #[ignore = "subprocess fixture; invoked by blocked_encoder_pipe_is_terminated_and_reaped"]
    fn blocked_encoder_child() {
        if std::env::var_os("ETSADS_TEST_BLOCKED_ENCODER").is_some() {
            thread::sleep(Duration::from_secs(30));
        }
    }

    #[test]
    fn blocked_encoder_pipe_is_terminated_and_reaped() {
        let mut child = std::process::Command::new(std::env::current_exe().unwrap());
        child
            .args([
                "--exact",
                "video::tests::blocked_encoder_child",
                "--ignored",
            ])
            .env("ETSADS_TEST_BLOCKED_ENCODER", "1")
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            child.creation_flags(0x08000000);
        }
        let mut child = child.spawn().unwrap();
        let mut pipe = child.stdin.take().unwrap();
        let encoder = Arc::new(Mutex::new(Some(child)));
        let done = Arc::new(AtomicBool::new(false));
        let worker_done = done.clone();
        let worker = thread::spawn(move || {
            let result = pipe.write_all(&vec![0_u8; 8 * 1024 * 1024]);
            worker_done.store(true, Ordering::Release);
            result?;
            Ok(1)
        });
        let (_tx, indexes) = mpsc::sync_channel(1);
        let session = VideoSession {
            stop: Arc::new(AtomicBool::new(false)),
            indexes,
            worker: Some(worker),
            encoder: encoder.clone(),
            progress: Arc::new(AtomicU64::new(0)),
            done: done.clone(),
            watchdog: Mutex::new(None),
        };
        let started = std::time::Instant::now();
        assert!(session.finish().is_err());
        assert!(started.elapsed() < Duration::from_secs(5));
        assert!(done.load(Ordering::Acquire));
        assert!(encoder.lock().unwrap().is_none());
    }
}
