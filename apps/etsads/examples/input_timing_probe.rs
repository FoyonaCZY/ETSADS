//! Offline callback-rate evidence; never accesses live mappings or controls.
use anyhow::Context;
use etsads_msgs::protocol::{AppliedControl, ControlCmd, TelemetryFrame};
fn main() -> anyhow::Result<()> {
    let run = std::env::args()
        .nth(1)
        .context("usage: input_timing_probe RUN")?;
    let (mut telemetry, mut applied, mut commands) = (Vec::new(), Vec::new(), Vec::new());
    etsads_recorder::replay(std::path::Path::new(&run), |topic, r| {
        match topic {
            "/telemetry" => telemetry.push(r.decode_wire::<TelemetryFrame>()?),
            "/applied" => applied.push(r.decode_wire::<AppliedControl>()?),
            "/control_cmd" => commands.push(r.decode_wire::<ControlCmd>()?),
            _ => {}
        }
        Ok(())
    })?;
    let frequency = telemetry.first().context("no telemetry")?.qpc_frequency as f64;
    let ms = |ticks: u64| ticks as f64 * 1000.0 / frequency;
    applied.sort_by_key(|a| a.applied_qpc);
    applied.dedup_by_key(|a| a.applied_qpc);
    commands.sort_by_key(|c| c.write_qpc);
    commands.dedup_by_key(|c| c.seq);
    let active: Vec<_> = commands
        .iter()
        .filter(|c| c.steer.abs() > 0.001 || c.throttle > 0.0 || c.brake > 0.0)
        .collect();
    let start = active.first().map(|c| c.write_qpc);
    let end = active.last().map(|c| c.write_qpc);
    let near = |q: u64| {
        start.zip(end).is_some_and(|(s, e)| {
            q.saturating_add(frequency as u64 / 2) >= s
                && q <= e.saturating_add(frequency as u64 / 2)
        })
    };
    let mut rates = Vec::new();
    for pair in applied.windows(2) {
        let dt = pair[1].applied_qpc.saturating_sub(pair[0].applied_qpc);
        let count = pair[1].frame_index.saturating_sub(pair[0].frame_index);
        if dt > 0 {
            rates.push((count as f64*frequency/dt as f64,serde_json::json!({"hz":count as f64*frequency/dt as f64,"callback_count":count,"elapsed_ms":ms(dt),"previous_index":pair[0].frame_index,"next_index":pair[1].frame_index,"near_command":near(pair[1].applied_qpc),"reason":pair[1].reason})));
        }
    }
    rates.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut gaps:Vec<_>=telemetry.windows(2).map(|p|{let gap=p[1].header.qpc.saturating_sub(p[0].header.qpc);(gap,serde_json::json!({"wall_ms":ms(gap),"sim_us":p[1].header.sim_time_us.saturating_sub(p[0].header.sim_time_us),"before_frame":p[0].header.frame_id,"after_frame":p[1].header.frame_id,"near_command":near(p[1].header.qpc)}))}).collect();
    gaps.sort_by_key(|p| std::cmp::Reverse(p.0));
    let mut reasons = std::collections::BTreeMap::new();
    for a in &applied {
        *reasons.entry(a.reason).or_insert(0usize) += 1;
    }
    let overall=applied.first().zip(applied.last()).map(|(a,b)|serde_json::json!({"counter_delta":b.frame_index.saturating_sub(a.frame_index),"wall_seconds":b.applied_qpc.saturating_sub(a.applied_qpc)as f64/frequency,"hz":b.frame_index.saturating_sub(a.frame_index)as f64*frequency/(b.applied_qpc.saturating_sub(a.applied_qpc).max(1))as f64}));
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"run":run,"telemetry_count":telemetry.len(),"applied_unique":applied.len(),"command_unique":commands.len(),"active_command_records":active.len(),"active_window_ms":start.zip(end).map(|(a,b)|ms(b-a)),"overall_callbacks":overall,"max_callback_rates":rates.into_iter().take(12).map(|(_,v)|v).collect::<Vec<_>>(),"max_physics_gaps":gaps.into_iter().take(8).map(|(_,v)|v).collect::<Vec<_>>(),"applied_reasons":reasons,"max_input_steer":telemetry.iter().map(|t|t.input[0].abs()).fold(0.0f32,f32::max),"max_effective_steer":telemetry.iter().map(|t|t.effective[0].abs()).fold(0.0f32,f32::max),"max_applied_steer":applied.iter().map(|a|a.axes[0].abs()).fold(0.0f32,f32::max)})
        )?
    );
    Ok(())
}
