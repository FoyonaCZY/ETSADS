use crate::applied_alignment::AppliedTimeline;
use anyhow::ensure;
use etsads_control::static_identification::{
    StaticSteeringAnalysis, StaticSteeringSample, analyze,
};
use etsads_msgs::{
    protocol::{AppliedControl, ConfigBlock, TelemetryFrame},
    vehicle::geometry_fingerprint,
};
use serde::Serialize;
use std::path::Path;
#[derive(Serialize)]
pub struct StaticRunReport {
    pub schema_version: u32,
    pub report_kind: String,
    pub vehicle_id: String,
    pub source_run: String,
    pub recording_source: String,
    pub complete_recording: bool,
    pub frame_gaps: u64,
    pub geometry_fingerprint: Option<String>,
    pub steered_wheel_indices: Vec<usize>,
    pub synchronized_applied_samples: usize,
    pub recording_telemetry_frames: u64,
    pub unaligned_or_inactive_frames: usize,
    pub analysis: StaticSteeringAnalysis,
}
pub fn analyze_run(
    run: &Path,
    vehicle_id: &str,
    settle_seconds: f64,
) -> anyhow::Result<StaticRunReport> {
    ensure!(!vehicle_id.is_empty(), "vehicle id is required");
    let mut configuration: Option<ConfigBlock> = None;
    let mut fingerprint = None;
    let mut indices = Vec::new();
    let mut samples = Vec::new();
    let mut clocks = Vec::new();
    let mut timeline = AppliedTimeline::default();
    let replay = etsads_recorder::replay(run, |topic, record| {
        match topic {
            "/config" => {
                let config: ConfigBlock = record.decode_wire()?;
                let observed = geometry_fingerprint(&config).map_err(anyhow::Error::msg)?;
                ensure!(
                    fingerprint.as_ref().is_none_or(|old| old == &observed),
                    "truck geometry changed during static steering scan"
                );
                fingerprint = Some(observed);
                indices = (0..config.wheel_count as usize)
                    .filter(|index| config.wheel_steerable[*index] != 0)
                    .collect();
                configuration = Some(config);
            }
            "/telemetry" => {
                let raw: TelemetryFrame = record.decode_wire()?;
                if configuration.is_some() && !indices.is_empty() && raw.qpc_frequency > 0 {
                    samples.push(StaticSteeringSample {
                        time_s: raw.header.qpc as f64 / raw.qpc_frequency as f64,
                        axis: 0.0,
                        applied_throttle: 0.0,
                        effective_throttle: raw.effective[1] as f64,
                        speed_mps: raw.speed_mps as f64,
                        paused: raw.paused != 0,
                        wheel_angles_rad: indices
                            .iter()
                            .map(|index| raw.wheel_steering[*index] as f64 * std::f64::consts::TAU)
                            .collect(),
                    });
                    clocks.push((raw.header.qpc, raw.qpc_frequency));
                }
            }
            "/applied" => {
                let applied: AppliedControl = record.decode_wire()?;
                timeline.push(applied)?;
            }
            _ => {}
        }
        Ok(())
    })?;
    ensure!(
        replay.nonmonotonic == 0,
        "static scan recording has time/frame regressions"
    );
    timeline.finish();
    let observed = samples.len();
    let samples: Vec<_> = samples
        .into_iter()
        .zip(clocks)
        .filter_map(|(mut sample, (qpc, frequency))| {
            timeline.at(qpc, frequency).map(|applied| {
                sample.axis = applied.axes[0] as f64;
                sample.applied_throttle = applied.axes[1] as f64;
                sample
            })
        })
        .collect();
    let mut analysis = analyze(&samples, settle_seconds).map_err(anyhow::Error::msg)?;
    if !replay.completed {
        analysis
            .issues
            .push("recording did not finish cleanly".into());
    }
    if replay.frame_gaps > 0 {
        analysis.issues.push(format!(
            "recording has {} missing frames",
            replay.frame_gaps
        ));
    }
    Ok(StaticRunReport {
        schema_version: 1,
        report_kind: "static_zero_throttle_wheel_angle_scan_not_driving_calibration".into(),
        vehicle_id: vehicle_id.into(),
        source_run: run.to_string_lossy().into(),
        recording_source: replay.source,
        complete_recording: replay.completed,
        frame_gaps: replay.frame_gaps,
        geometry_fingerprint: fingerprint,
        steered_wheel_indices: indices,
        synchronized_applied_samples: samples.len(),
        recording_telemetry_frames: replay.telemetry_frames,
        unaligned_or_inactive_frames: observed - samples.len(),
        analysis,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mcap_static_scan_uses_causal_applied_samples_and_cannot_certify_driving() {
        use etsads_recorder::{Record, Recorder, RunMetadata};
        let root = std::env::temp_dir().canonicalize().unwrap();
        let dir = root.join(format!(
            "etsads-static-sysid-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = Recorder::create(&dir, RunMetadata::new("synthetic", "none")).unwrap();
        let mut config = ConfigBlock {
            config_seq: 1,
            wheel_count: 4,
            ..Default::default()
        };
        config.wheel_radius[..4].fill(0.5);
        config.wheel_steerable[..2].fill(1);
        writer
            .record("/config", &Record::wire(&config, 0, 0, 1))
            .unwrap();
        for (index, axis) in [-0.2_f32, -0.1, 0.0, 0.1, 0.2].iter().enumerate() {
            for step in 0..150 {
                let frame = (index * 150 + step + 1) as u64;
                let mut raw = TelemetryFrame {
                    qpc_frequency: 1000,
                    ..Default::default()
                };
                raw.header.frame_id = frame;
                raw.header.sim_time_us = frame * 20000;
                raw.paused_simulation_time_us = raw.header.sim_time_us;
                raw.header.qpc = 1000 + frame * 20;
                raw.wheel_steering[0] = -axis * 0.6 / std::f32::consts::TAU;
                raw.wheel_steering[1] = -axis * 0.55 / std::f32::consts::TAU;
                writer
                    .record("/telemetry", &Record::telemetry(&raw))
                    .unwrap();
                let applied = AppliedControl {
                    applied_seq: frame,
                    applied_qpc: raw.header.qpc - 1,
                    frame_index: frame,
                    axes: [*axis, 0.0, 0.0, 0.0],
                    ..Default::default()
                };
                writer
                    .record(
                        "/applied",
                        &Record::wire(&applied, frame, raw.header.sim_time_us, applied.applied_qpc),
                    )
                    .unwrap();
            }
        }
        writer.finish().unwrap();
        let report = analyze_run(&dir, "synthetic-static-truck", 0.75).unwrap();
        assert_eq!(report.recording_source, "synthetic");
        assert_eq!(report.synchronized_applied_samples, 750);
        assert_eq!(report.analysis.points.len(), 5);
        assert_eq!(report.analysis.measured_axis_polarity, Some(-1));
        assert!(report.analysis.zero_throttle_verified && report.analysis.stationary_verified);
        assert!(!report.analysis.driving_authorized);
        let resolved = dir.canonicalize().unwrap();
        assert_eq!(resolved.parent(), Some(root.as_path()));
        assert!(
            resolved
                .file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("etsads-static-sysid-test-")
        );
        std::fs::remove_dir_all(resolved).unwrap();
    }
}
