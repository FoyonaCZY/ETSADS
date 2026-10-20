//! Offline E9 evidence. Never connects to a game mapping or emits control.
use anyhow::{Context, ensure};
use etsads_msgs::protocol::{ControlCmd, FrameTiming, InputDiagnostics, TelemetryFrame};
use std::collections::BTreeMap;

fn distribution(mut values: Vec<f64>) -> serde_json::Value {
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
        return serde_json::Value::Null;
    }
    let percentile = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    serde_json::json!({"n": values.len(), "min_ms": values[0], "p50_ms": percentile(0.5),
        "p95_ms": percentile(0.95), "max_ms": values[values.len()-1]})
}
fn main() -> anyhow::Result<()> {
    let run = std::env::args()
        .nth(1)
        .context("usage: frame_timing_probe RUN")?;
    let mut frequency = 0;
    let mut timing = Vec::new();
    let mut diagnostics = Vec::new();
    let mut commands = BTreeMap::new();
    etsads_recorder::replay(std::path::Path::new(&run), |topic, record| {
        match topic {
            "/telemetry" => frequency = record.decode_wire::<TelemetryFrame>()?.qpc_frequency,
            "/frame_timing" => timing.push(record.decode_wire::<FrameTiming>()?),
            "/input_diagnostics" => diagnostics.push(record.decode_wire::<InputDiagnostics>()?),
            "/control_cmd" => {
                let command = record.decode_wire::<ControlCmd>()?;
                commands.insert(command.seq, command);
            }
            _ => {}
        }
        Ok(())
    })?;
    ensure!(frequency > 0, "no valid telemetry frequency");
    ensure!(
        !timing.is_empty(),
        "run has no frame timing; requires current bridge and recorder"
    );
    let ms = |ticks: u64| ticks as f64 * 1000.0 / frequency as f64;
    let mut missing = 0;
    let mut order_errors = 0;
    for pair in timing.windows(2) {
        if pair[1].event_seq <= pair[0].event_seq || pair[1].qpc < pair[0].qpc {
            order_errors += 1;
        } else {
            missing += pair[1].event_seq - pair[0].event_seq - 1;
        }
    }
    let mut kinds = BTreeMap::new();
    let mut transitions = BTreeMap::new();
    let mut physics = BTreeMap::new();
    let mut complete_duration = Vec::new();
    let mut prepare_after_end = Vec::new();
    let mut command_to_prepare = Vec::new();
    let mut input_physical_reference_mismatch = 0;
    let mut invalid_timestamps = 0;
    for (index, event) in timing.iter().enumerate() {
        *kinds.entry(event.kind).or_insert(0usize) += 1;
        if let Some(previous) = index.checked_sub(1).and_then(|i| timing.get(i)) {
            *transitions
                .entry(format!("{}->{}", previous.kind, event.kind))
                .or_insert(0usize) += 1;
        }
        match event.kind {
            1 => {
                if event.qpc != event.frame_start_qpc {
                    invalid_timestamps += 1;
                }
                physics.insert(event.physics_frame_id, (event.qpc, None));
            }
            2 => {
                if event.qpc != event.frame_end_qpc || event.frame_start_qpc > event.qpc {
                    invalid_timestamps += 1;
                }
                if let Some((start, end)) = physics.get_mut(&event.physics_frame_id) {
                    if *start != event.frame_start_qpc {
                        invalid_timestamps += 1;
                    }
                    *end = Some(event.qpc);
                    if event.qpc >= *start {
                        complete_duration.push(ms(event.qpc - *start));
                    }
                }
            }
            3 => {
                if event.qpc != event.input_prepare_qpc {
                    invalid_timestamps += 1;
                }
                if let Some((_, Some(end))) = physics.get(&event.physics_frame_id) {
                    if event.qpc >= *end {
                        prepare_after_end.push(ms(event.qpc - *end));
                    } else {
                        input_physical_reference_mismatch += 1;
                    }
                }
                if let Some(command) = commands.get(&event.command_seq)
                    && event.qpc >= command.write_qpc
                {
                    command_to_prepare.push(ms(event.qpc - command.write_qpc));
                }
            }
            _ => order_errors += 1,
        }
    }
    let counters = diagnostics.first().zip(diagnostics.last()).map(|(first, last)| {
        serde_json::json!({
            "first_flags": first.first_flags, "last_flags": last.last_flags,
            "callback_delta": last.callback_count.saturating_sub(first.callback_count),
            "batch_delta": last.batch_count.saturating_sub(first.batch_count),
            "completed_batch_delta": last.completed_batch_count.saturating_sub(first.completed_batch_count),
            "repeated_first_flags_delta": last.repeated_first_flags.saturating_sub(first.repeated_first_flags),
            "max_batch_events": diagnostics.iter().map(|d| d.max_batch_events).max(),
            "max_sampled_batch_callbacks": diagnostics.iter().map(|d| d.last_batch_callbacks).max(),
            "registered_input_count": etsads_game::input::INPUT_COUNT,
            "sampled_batches_over_registered_count": diagnostics.iter().filter(|d| d.last_batch_events as usize > etsads_game::input::INPUT_COUNT).count()
        })
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "run": run, "source": "recorded_game_callbacks", "timing_records": timing.len(),
            "event_kinds": kinds, "event_transitions": transitions, "missing_event_sequences": missing,
            "event_order_errors": order_errors, "invalid_timestamps": invalid_timestamps,
            "input_physical_reference_mismatch": input_physical_reference_mismatch,
            "frame_start_to_end": distribution(complete_duration),
            "completed_frame_end_to_input_prepare": distribution(prepare_after_end),
            "sampled_command_write_to_input_prepare": distribution(command_to_prepare),
            "input_diagnostics": counters,
            "limitations": "Prepare timing proves callback order, not effective vehicle actuation; latest command/diagnostic cells may be undersampled. Boundary events outside recording are excluded."
        }))?
    );
    Ok(())
}
