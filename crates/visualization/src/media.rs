//! Bounded media work for diagnostics. Source images and video files are read
//! only; JPEG is a lossy display representation, never measurement evidence.
use crate::{live_diagnostics::rgba_pixels, rerun_export::VideoProbe};
use anyhow::{Context, ensure};
use etsads_game::capture::Frame;
use std::{
    io::{self, Read, Write},
    path::Path,
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};

const PROBE_STDOUT_LIMIT: usize = 64 * 1024 * 1024;
const PROBE_STDERR_LIMIT: usize = 64 * 1024;
const IMAGE_LIMIT: usize = 64 * 1024 * 1024;
const MAX_PROBE_FRAMES: usize = 500_000;

#[derive(Clone, Copy)]
struct Limits {
    stdout: usize,
    stderr: usize,
    timeout: Duration,
}

struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        // Also covers pipe/setup failures and panic unwinding. Waiting reaps
        // the direct ffprobe child on both Windows and Unix.
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn read_bounded(reader: &mut impl Read, limit: usize) -> io::Result<Vec<u8>> {
    let mut result = Vec::with_capacity(limit.min(8192));
    let mut buffer = [0; 8192];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            return Ok(result);
        }
        if count > limit.saturating_sub(result.len()) {
            return Err(io::Error::other("pipe output exceeded byte limit"));
        }
        result.extend_from_slice(&buffer[..count]);
    }
}

fn run_bounded(command: &mut Command, limits: Limits) -> anyhow::Result<Vec<u8>> {
    command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
    }
    let started = Instant::now();
    let mut child = ChildGuard(command.spawn().context("start ffprobe")?);
    let mut stdout = child
        .0
        .stdout
        .take()
        .context("ffprobe stdout pipe missing")?;
    let mut stderr = child
        .0
        .stderr
        .take()
        .context("ffprobe stderr pipe missing")?;
    let (send, receive) = mpsc::channel();
    let stderr_send = send.clone();
    let out = thread::spawn(move || {
        let result = read_bounded(&mut stdout, limits.stdout);
        let _ = send.send(result.as_ref().map(|_| ()).map_err(ToString::to_string));
        result
    });
    let err = thread::spawn(move || {
        let result = read_bounded(&mut stderr, limits.stderr);
        let _ = stderr_send.send(result.as_ref().map(|_| ()).map_err(ToString::to_string));
        result
    });
    let outcome = loop {
        if let Some(error) = receive.try_iter().find_map(Result::err) {
            break Err(anyhow::anyhow!("ffprobe {error}"));
        }
        match child.0.try_wait() {
            Ok(Some(status)) if out.is_finished() && err.is_finished() => break Ok(status),
            Ok(_) => {}
            Err(error) => break Err(error.into()),
        }
        if started.elapsed() >= limits.timeout {
            break Err(anyhow::anyhow!("ffprobe exceeded total timeout"));
        }
        thread::sleep(Duration::from_millis(10));
    };
    // Kill before joining pipe readers, so a blocked writer cannot deadlock
    // error cleanup. Drop additionally guarantees wait on every return path.
    if outcome.is_err() {
        let _ = child.0.kill();
    }
    let wait = child.0.wait();
    let stdout = out
        .join()
        .map_err(|_| anyhow::anyhow!("stdout reader panicked"))?;
    let stderr = err
        .join()
        .map_err(|_| anyhow::anyhow!("stderr reader panicked"))?;
    let status = outcome?;
    wait.context("reap ffprobe")?;
    let stdout = stdout.context("read bounded ffprobe stdout")?;
    let stderr = stderr.context("read bounded ffprobe stderr")?;
    ensure!(
        status.success(),
        "ffprobe failed ({status}): {}",
        String::from_utf8_lossy(&stderr)
    );
    Ok(stdout)
}

fn parse_probe(bytes: &[u8], max_frames: usize) -> anyhow::Result<VideoProbe> {
    ensure!(
        (1..=MAX_PROBE_FRAMES).contains(&max_frames),
        "video frame budget must be 1..500000"
    );
    ensure!(
        bytes.len() <= PROBE_STDOUT_LIMIT,
        "ffprobe JSON exceeds byte limit"
    );
    let probe: VideoProbe =
        serde_json::from_slice(bytes).context("parse ffprobe frame metadata")?;
    ensure!(
        probe.streams.len() == 1,
        "expected exactly one selected video stream"
    );
    ensure!(
        !probe.frames.is_empty() && probe.frames.len() <= max_frames,
        "decoded video frame count exceeds budget or is empty"
    );
    let stream = &probe.streams[0];
    ensure!(
        stream.width > 0
            && stream.height > 0
            && (stream.width as u64)
                .checked_mul(stream.height as u64)
                .and_then(|pixels| pixels.checked_mul(4))
                .is_some_and(|bytes| bytes <= IMAGE_LIMIT as u64)
            && !stream.codec_name.is_empty(),
        "invalid video dimensions/codec"
    );
    // Actual per-frame PTS, including variable cadence and strict monotonicity.
    // Do not replace this with frame_number / FPS.
    probe.timestamps_ns()?;
    Ok(probe)
}

/// Probe actual decoded frame PTS, bounded by 64 MiB stdout, 64 KiB stderr and
/// 300 seconds total wall time. The direct child is killed/reaped on failure.
pub fn probe_video(path: &Path, ffprobe: &str, max_frames: usize) -> anyhow::Result<VideoProbe> {
    ensure!(
        (1..=MAX_PROBE_FRAMES).contains(&max_frames),
        "video frame budget must be 1..500000"
    );
    ensure!(path.is_file(), "video source is not a file");
    let mut command = Command::new(ffprobe);
    command
        .args([
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_frames",
            "-show_streams",
            "-show_entries",
            "frame=best_effort_timestamp:stream=width,height,time_base,codec_name",
            "-of",
            "json",
        ])
        .arg(path);
    let bytes = run_bounded(
        &mut command,
        Limits {
            stdout: PROBE_STDOUT_LIMIT,
            stderr: PROBE_STDERR_LIMIT,
            timeout: Duration::from_secs(300),
        },
    )?;
    parse_probe(&bytes, max_frames)
}

struct ImageBytes(Vec<u8>);
impl Write for ImageBytes {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if bytes.len() > IMAGE_LIMIT.saturating_sub(self.0.len()) {
            return Err(io::Error::other("encoded JPEG exceeds byte limit"));
        }
        self.0.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn encode_jpeg(frame: &Frame) -> anyhow::Result<Vec<u8>> {
    ensure!(
        frame.data.len() <= IMAGE_LIMIT,
        "capture bytes exceed 64 MiB"
    );
    let rgba = rgba_pixels(frame).map_err(anyhow::Error::msg)?;
    let image = image::RgbaImage::from_raw(frame.width, frame.height, rgba)
        .context("validated capture dimensions disagree with pixels")?;
    let mut bytes = ImageBytes(Vec::new());
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, 80).encode_image(&image)?;
    Ok(bytes.0)
}

/// JPEG quality 80 at the original dimensions. RGBA/BGRA/BGRX row padding and
/// channel order are validated before encoding; alpha is omitted for display.
pub fn encode_live_image(frame: &Frame) -> anyhow::Result<rerun::EncodedImage> {
    Ok(rerun::EncodedImage::new(encode_jpeg(frame)?).with_media_type("image/jpeg"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn jpeg_preserves_dimensions_and_channel_order_and_rejects_invalid_layout() {
        let mut frame = Frame {
            width: 64,
            height: 32,
            stride: 260,
            format: 87,
            frame_index: 1,
            timestamp_qpc: 1,
            publish_qpc: 2,
            data: vec![0; 260 * 32],
        };
        for row in frame.data.as_chunks_mut::<260>().0 {
            for pixel in row[..256].as_chunks_mut::<4>().0 {
                *pixel = [20, 70, 220, 255];
            }
        }
        let bytes = encode_jpeg(&frame).unwrap();
        assert!(bytes.starts_with(&[0xff, 0xd8]));
        assert!(bytes.len() < frame.data.len() / 4);
        let decoded = image::load_from_memory(&bytes).unwrap().to_rgb8();
        assert_eq!(decoded.dimensions(), (64, 32));
        for (actual, expected) in decoded.get_pixel(32, 16).0.into_iter().zip([220, 70, 20]) {
            assert!((actual as i32 - expected).abs() <= 5);
        }
        let encoded = encode_live_image(&frame).unwrap();
        assert!(encoded.blob.is_some() && encoded.media_type.is_some());
        frame.stride = 255;
        assert!(encode_live_image(&frame).is_err());
        frame.width = u32::MAX;
        assert!(encode_live_image(&frame).is_err());
        assert!(ImageBytes(Vec::new()).write(&[]).is_ok());
    }

    #[test]
    fn probe_requires_actual_finite_monotonic_pts_and_frame_budget() {
        let bytes = br#"{"streams":[{"width":2,"height":2,"time_base":"1/90000","codec_name":"h264"}],"frames":[{"best_effort_timestamp":4500},{"best_effort_timestamp":6000},{"best_effort_timestamp":9000}]}"#;
        assert_eq!(
            parse_probe(bytes, 3).unwrap().timestamps_ns().unwrap(),
            vec![0, 16_666_667, 50_000_000]
        );
        assert!(parse_probe(bytes, 2).is_err());
        assert!(parse_probe(bytes, 0).is_err());
        assert!(parse_probe(br#"{"streams":[],"frames":[]}"#, 3).is_err());
        let bad = String::from_utf8(bytes.to_vec())
            .unwrap()
            .replace("9000}", "5000}");
        assert!(parse_probe(bad.as_bytes(), 3).is_err());
    }

    #[test]
    #[ignore = "subprocess fixture; parent tests invoke with isolated environment"]
    fn child_process_fixture() {
        match std::env::var("ETSADS_MEDIA_FIXTURE").unwrap().as_str() {
            "stdout" => loop {
                let _ = io::stdout().write_all(&[b'x'; 8192]);
            },
            "stderr" => loop {
                let _ = io::stderr().write_all(&[b'x'; 8192]);
            },
            "timeout" => thread::sleep(Duration::from_secs(60)),
            "fail" => std::process::exit(7),
            "success" => {}
            _ => panic!("unknown fixture"),
        }
    }

    #[test]
    fn bounded_child_output_failure_and_timeout_are_killed_and_reaped() {
        for mode in ["stdout", "stderr", "timeout", "fail", "success"] {
            let mut command = Command::new(std::env::current_exe().unwrap());
            command
                .args([
                    "--exact",
                    "media::tests::child_process_fixture",
                    "--ignored",
                    "--nocapture",
                ])
                .env("ETSADS_MEDIA_FIXTURE", mode);
            let started = Instant::now();
            let result = run_bounded(
                &mut command,
                Limits {
                    stdout: 4096,
                    stderr: 4096,
                    timeout: Duration::from_millis(400),
                },
            );
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "{mode} cleanup stalled"
            );
            if mode == "success" {
                assert!(result.is_ok());
            } else {
                assert!(result.is_err(), "{mode} must fail");
            }
        }
    }

    #[test]
    #[ignore = "read-only existing video validation; set ETSADS_MEDIA_VIDEO and ETSADS_MEDIA_FFPROBE"]
    fn existing_video_pts_match_viewer_sample_table() {
        let path = std::env::var_os("ETSADS_MEDIA_VIDEO").unwrap();
        let ffprobe = std::env::var("ETSADS_MEDIA_FFPROBE").unwrap();
        let path = Path::new(&path);
        let probe = probe_video(path, &ffprobe, MAX_PROBE_FRAMES).unwrap();
        let decoded = probe.timestamps_ns().unwrap();
        let viewer = crate::rerun_export::viewer_video_timestamps(path).unwrap();
        assert_eq!(decoded.len(), viewer.len());
        assert!(
            decoded
                .iter()
                .zip(&viewer)
                .all(|(a, b)| a.abs_diff(*b) <= 1)
        );
        println!(
            "actual decoded frames={} last_pts_ns={} viewer_pts_match=true",
            decoded.len(),
            decoded.last().unwrap()
        );
    }
}
