//! Offline evidence probe; never opens a game mapping or writes controls.
use etsads_msgs::protocol::{TelemetryFrame, VideoIndex};
fn distribution(mut values: Vec<f64>) -> serde_json::Value {
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
        return serde_json::Value::Null;
    }
    let percentile = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    serde_json::json!({"n":values.len(),"min_ms":values[0],"p50_ms":percentile(0.5),"p95_ms":percentile(0.95),"max_ms":values[values.len()-1]})
}
fn main() -> anyhow::Result<()> {
    let run = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: alignment_probe RUN"))?;
    let mut telemetry = Vec::new();
    let mut video = Vec::new();
    etsads_recorder::replay(std::path::Path::new(&run), |topic, r| {
        match topic {
            "/telemetry" => telemetry.push(r.decode_wire::<TelemetryFrame>()?),
            "/video_index" => video.push(r.decode_wire::<VideoIndex>()?),
            _ => {}
        }
        Ok(())
    })?;
    telemetry.sort_by_key(|t| t.header.qpc);
    let frequency = telemetry.first().context("no telemetry")?.qpc_frequency as f64;
    anyhow::ensure!(frequency > 0.0, "invalid frequency");
    let ms = |ticks: u64| ticks as f64 * 1000.0 / frequency;
    let mut original = Vec::new();
    let mut nearest = Vec::new();
    let mut previous = Vec::new();
    let mut next = Vec::new();
    let mut future_better = 0;
    let mut missing_next = 0;
    let mut examples = Vec::new();
    for index in &video {
        original.push(ms(index.alignment_error_qpc));
        let split = telemetry.partition_point(|t| t.header.qpc < index.capture_qpc);
        let before = split.checked_sub(1).and_then(|i| telemetry.get(i));
        let after = telemetry.get(split);
        if let Some(t) = before {
            previous.push(ms(index.capture_qpc - t.header.qpc));
        }
        if let Some(t) = after {
            next.push(ms(t.header.qpc - index.capture_qpc));
        } else {
            missing_next += 1;
        }
        let chosen = [before, after]
            .into_iter()
            .flatten()
            .min_by_key(|t| t.header.qpc.abs_diff(index.capture_qpc));
        if let Some(t) = chosen {
            let error = ms(t.header.qpc.abs_diff(index.capture_qpc));
            nearest.push(error);
            if t.header.qpc > index.capture_qpc {
                future_better += 1;
            }
            if examples.len() < 8 && index.alignment_error_qpc > frequency as u64 / 60 {
                examples.push(serde_json::json!({"video_frame":index.video_frame,"original_physics_frame":index.physics_frame,"original_error_ms":ms(index.alignment_error_qpc),"nearest_frame":t.header.frame_id,"nearest_error_ms":error,"nearest_is_future":t.header.qpc>index.capture_qpc}));
            }
        }
    }
    let physical_wall_steps = telemetry
        .windows(2)
        .map(|p| ms(p[1].header.qpc.saturating_sub(p[0].header.qpc)))
        .collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"run":run,"telemetry":telemetry.len(),"video":video.len(),"qpc_frequency":frequency,"original":distribution(original),"offline_nearest":distribution(nearest),"previous":distribution(previous),"next":distribution(next),"future_better":future_better,"missing_next":missing_next,"physics_wall_steps":distribution(physical_wall_steps),"examples":examples})
        )?
    );
    Ok(())
}
use anyhow::Context;
