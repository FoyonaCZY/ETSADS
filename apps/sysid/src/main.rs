//! Offline calibration and explicit excitation plans. This executable never
//! writes to the game's input mapping.
mod applied_alignment;
mod static_steering;
use anyhow::{Context, ensure};
use clap::{Parser, Subcommand, ValueEnum};
use etsads_control::identification::{
    CalibrationConditions, IdentificationSample, geometry_fingerprint, identify,
    reference_from_config, validate_reference_geometry,
};
use etsads_msgs::{
    VehicleGeometry,
    frames::{heading_to_yaw, wrap_angle},
    protocol::TelemetryFrame,
};
use serde::Serialize;
use std::{
    fs::{File, OpenOptions},
    path::PathBuf,
};
#[derive(Parser)]
#[command(about = "Identify vehicle controls from an immutable MCAP run; never commands the game")]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Analyze a stationary, zero-throttle semantic steering sweep. Never produces a driving certificate.
    StaticSteering {
        #[arg(long)]
        run: PathBuf,
        #[arg(long)]
        vehicle_id: String,
        #[arg(long, default_value_t = 0.75)]
        settle_seconds: f64,
        #[arg(long)]
        output: PathBuf,
    },
    /// Produce a review-only stationary sweep with throttle fixed at zero.
    StaticPlan {
        #[arg(long)]
        vehicle_id: String,
        #[arg(long,value_delimiter=',',num_args=1..,allow_hyphen_values=true)]
        steering_axes: Vec<f64>,
        #[arg(long, default_value_t = 3.0)]
        hold_seconds: f64,
        #[arg(long)]
        output: PathBuf,
    },
    /// Fit only recorded observations. Geometry must come from measured vehicle configuration.
    Fit {
        #[arg(long)]
        run: PathBuf,
        #[arg(long)]
        vehicle_id: String,
        #[arg(long)]
        geometry: PathBuf,
        #[arg(long, value_enum)]
        steering_source: AxisSource,
        #[arg(long,value_delimiter=',',num_args=1..)]
        rear_wheel_indices: Vec<u32>,
        #[arg(long)]
        reference_evidence: PathBuf,
        /// Optional existing exterior measurement; never inferred from wheel telemetry.
        #[arg(long)]
        front_bumper_measurement: Option<PathBuf>,
        /// Existing measured body offsets and their source evidence; never inferred from wheel positions.
        #[arg(long)]
        envelope_measurement: Option<PathBuf>,
        #[arg(long)]
        output: PathBuf,
    },
    /// Write a reviewable experimental sequence; no input is applied.
    Plan {
        #[arg(long)]
        vehicle_id: String,
        #[arg(long,value_delimiter=',',num_args=1..)]
        speeds_kmh: Vec<f64>,
        #[arg(long,value_delimiter=',',num_args=1..,allow_hyphen_values=true)]
        steering_axes: Vec<f64>,
        #[arg(long)]
        hold_seconds: f64,
        #[arg(long)]
        output: PathBuf,
    },
}
#[derive(Clone, Copy, Debug, ValueEnum)]
enum AxisSource {
    Applied,
    Input,
    Effective,
}
#[derive(Serialize)]
struct ExcitationStep {
    speed_kmh: f64,
    steering_axis: f64,
    hold_seconds: f64,
    settle_seconds: f64,
}
fn create_output(path: &PathBuf) -> anyhow::Result<File> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .context("output already exists; select a new filename to preserve previous calibration")
}
fn main() -> anyhow::Result<()> {
    match Args::parse().command {
        Command::StaticSteering {
            run,
            vehicle_id,
            settle_seconds,
            output,
        } => {
            let report = static_steering::analyze_run(&run, &vehicle_id, settle_seconds)?;
            serde_json::to_writer_pretty(create_output(&output)?, &report)?;
            println!(
                "Static steering report: {} levels={} polarity={:?} zero_throttle={} stationary={} driving_authorized=false",
                output.display(),
                report.analysis.points.len(),
                report.analysis.measured_axis_polarity,
                report.analysis.zero_throttle_verified,
                report.analysis.stationary_verified
            );
            for issue in &report.analysis.issues {
                println!("- {issue}");
            }
        }
        Command::StaticPlan {
            vehicle_id,
            steering_axes,
            hold_seconds,
            output,
        } => {
            ensure!(
                !vehicle_id.is_empty() && !steering_axes.is_empty(),
                "vehicle id and explicit axis values are required"
            );
            ensure!(
                hold_seconds.is_finite() && (1.0..=10.0).contains(&hold_seconds),
                "hold must be 1..10 seconds"
            );
            ensure!(
                steering_axes
                    .iter()
                    .all(|axis| axis.is_finite() && axis.abs() <= 0.5),
                "static steering axes must be within ±0.5"
            );
            let steps:Vec<_>=steering_axes.iter().map(|axis|serde_json::json!({"steering_axis":axis,"throttle":0.0,"hold_seconds":hold_seconds,"require_stationary":true})).collect();
            serde_json::to_writer_pretty(
                create_output(&output)?,
                &serde_json::json!({"schema_version":1,"vehicle_id":vehicle_id,"status":"review_only_no_game_inputs_written","driving_authorized":false,"steps":steps}),
            )?;
            println!(
                "Zero-throttle stationary plan saved: {}. No inputs sent.",
                output.display()
            );
        }
        Command::Plan {
            vehicle_id,
            speeds_kmh,
            steering_axes,
            hold_seconds,
            output,
        } => {
            ensure!(!vehicle_id.is_empty(), "vehicle id cannot be empty");
            ensure!(
                !speeds_kmh.is_empty() && !steering_axes.is_empty(),
                "provide explicit speed and axis lists"
            );
            ensure!(
                hold_seconds.is_finite() && (2.0..=10.0).contains(&hold_seconds),
                "hold must be 2..10 seconds"
            );
            ensure!(
                speeds_kmh
                    .iter()
                    .all(|v| v.is_finite() && (1.0..=80.0).contains(v)),
                "speeds must be 1..80 km/h"
            );
            ensure!(
                steering_axes
                    .iter()
                    .all(|v| v.is_finite() && v.abs() <= 0.4),
                "experimental axis must be within ±0.4"
            );
            let steps: Vec<_> = speeds_kmh
                .iter()
                .flat_map(|speed| {
                    steering_axes.iter().map(move |axis| ExcitationStep {
                        speed_kmh: *speed,
                        steering_axis: *axis,
                        hold_seconds,
                        settle_seconds: 2.0,
                    })
                })
                .collect();
            let document = serde_json::json!({"schema_version":1,"vehicle_id":vehicle_id,"status":"review_only_no_game_inputs_written","requirements":["isolated test area and continuous MCAP recording","supervisor active and player takeover verified","confirm axis polarity at low speed before higher-speed sweeps","record independent throttle, brake, coast and grade sequences for longitudinal identification"],"steps":steps});
            serde_json::to_writer_pretty(create_output(&output)?, &document)?;
            println!(
                "Experimental plan saved: {}. No controls were sent.",
                output.display()
            );
        }
        Command::Fit {
            run,
            vehicle_id,
            geometry,
            steering_source,
            rear_wheel_indices,
            reference_evidence,
            front_bumper_measurement,
            envelope_measurement,
            output,
        } => {
            let geometry: VehicleGeometry = serde_json::from_reader(File::open(geometry)?)?;
            ensure!(geometry.valid(), "invalid measured geometry");
            ensure!(
                reference_evidence.is_file(),
                "reference evidence file must exist"
            );
            let mut previous: Option<TelemetryFrame> = None;
            let mut fingerprint: Option<String> = None;
            let mut physical_configuration = None;
            let mut trailer_count: Option<usize> = None;
            let mut attachment_changed = false;
            let mut cargo_min = f64::INFINITY;
            let mut cargo_max = f64::NEG_INFINITY;
            let mut samples = Vec::new();
            let mut sample_clocks = Vec::new();
            let mut applied_timeline = applied_alignment::AppliedTimeline::default();
            let mut skipped = 0;
            let replay = etsads_recorder::replay(&run, |topic, record| {
                if topic == "/applied" && matches!(steering_source, AxisSource::Applied) {
                    let applied: etsads_msgs::protocol::AppliedControl = record.decode_wire()?;
                    applied_timeline.push(applied)?;
                    return Ok(());
                }
                if topic == "/config" {
                    let config: etsads_msgs::protocol::ConfigBlock = record.decode_wire()?;
                    let observed = geometry_fingerprint(&config)?;
                    ensure!(
                        fingerprint.as_ref().is_none_or(|known| known == &observed),
                        "truck geometry changed during calibration recording"
                    );
                    fingerprint = Some(observed);
                    physical_configuration = Some(config);
                    let cargo = config.cargo_mass_kg as f64;
                    ensure!(
                        cargo.is_finite() && (0.0..=200_000.0).contains(&cargo),
                        "invalid cargo mass in /config"
                    );
                    cargo_min = cargo_min.min(cargo);
                    cargo_max = cargo_max.max(cargo);
                    return Ok(());
                }
                if topic != "/telemetry" {
                    return Ok(());
                }
                let raw: TelemetryFrame = record.decode_wire()?;
                if raw.paused == 0 && raw.header.frame_id > 0 && raw.qpc_frequency > 0 {
                    let count = raw.trailers.iter().filter(|t| t.connected != 0).count();
                    if trailer_count.is_some_and(|old| old != count) {
                        attachment_changed = true;
                    }
                    trailer_count.get_or_insert(count);
                }
                if raw.paused != 0 {
                    previous = None;
                    skipped += 1;
                    return Ok(());
                }
                if let Some(old) = previous {
                    let current_time = raw.paused_simulation_time_us;
                    let old_time = old.paused_simulation_time_us;
                    if current_time <= old_time {
                        skipped += 1;
                        previous = Some(raw);
                        return Ok(());
                    }
                    let dt = (current_time - old_time) as f64 / 1e6;
                    let speed = raw.speed_mps as f64;
                    if dt > 0.1 || speed < 1.0 {
                        skipped += 1;
                        previous = Some(raw);
                        return Ok(());
                    }
                    let yaw = heading_to_yaw(raw.orientation_turns[0] as f64);
                    let prior_yaw = heading_to_yaw(old.orientation_turns[0] as f64);
                    let controls = match steering_source {
                        AxisSource::Applied => [0.0; 4],
                        AxisSource::Input => raw.input,
                        AxisSource::Effective => raw.effective,
                    };
                    let sample = IdentificationSample {
                        time: current_time as f64 / 1e6,
                        speed,
                        steering_axis: controls[0] as f64,
                        throttle: controls[1] as f64,
                        brake: controls[2] as f64,
                        curvature: wrap_angle(yaw - prior_yaw) / dt / speed,
                        acceleration: (speed - old.speed_mps as f64) / dt,
                        grade: raw.orientation_turns[1] as f64 * std::f64::consts::TAU,
                    };
                    samples.push(sample);
                    sample_clocks.push((raw.header.qpc, raw.qpc_frequency));
                }
                previous = Some(raw);
                Ok(())
            })?;
            ensure!(replay.completed, "run has no clean recorder finish marker");
            ensure!(
                replay.nonmonotonic == 0,
                "recording contains time/frame regressions"
            );
            let real = matches!(
                replay.source.as_str(),
                "game" | "ets2_live" | "live_game" | "ets2"
            );
            if matches!(steering_source, AxisSource::Applied) {
                let total = samples.len();
                applied_timeline.finish();
                samples = samples
                    .into_iter()
                    .zip(sample_clocks)
                    .filter_map(|(mut sample, (qpc, frequency))| {
                        applied_timeline.at(qpc, frequency).map(|applied| {
                            sample.steering_axis = applied.axes[0] as f64;
                            sample.throttle = applied.axes[1] as f64;
                            sample.brake = applied.axes[2] as f64;
                            sample
                        })
                    })
                    .collect();
                skipped += total - samples.len();
            }
            let mut calibration = identify(
                &samples,
                &vehicle_id,
                &run.to_string_lossy(),
                geometry,
                real,
            )?;
            calibration.geometry_fingerprint = fingerprint;
            if let Some(count) = trailer_count.filter(|count| *count <= 1)
                && cargo_min.is_finite()
                && cargo_max.is_finite()
            {
                let conditions = CalibrationConditions {
                    trailer_count: count as u8,
                    cargo_mass_range_kg: [(cargo_min - 50.0).max(0.0), cargo_max + 50.0],
                    evidence_path: run.to_string_lossy().into(),
                };
                if attachment_changed
                    || cargo_max - cargo_min > 100.0
                    || !conditions.valid(calibration.geometry)
                {
                    calibration.issues.push("trailer attachment or load changed, or does not match the explicit vehicle model; fit separate configurations".into());
                    calibration.accepted_for_control = false;
                }
                calibration.conditions = Some(conditions);
            } else {
                calibration.issues.push(
                    "trailer count and cargo applicability are not established by the recording"
                        .into(),
                );
                calibration.accepted_for_control = false;
            }
            if let Some(config) = physical_configuration {
                let reference = reference_from_config(
                    &config,
                    &rear_wheel_indices,
                    &reference_evidence.to_string_lossy(),
                )?;
                if let Err(e) =
                    validate_reference_geometry(&config, &reference, calibration.geometry)
                {
                    calibration.issues.push(e.to_string());
                    calibration.accepted_for_control = false;
                }
                calibration.reference = Some(reference);
            }
            calibration.steering_axis_source = match steering_source {
                AxisSource::Applied => "semantic_applied",
                AxisSource::Input => "telemetry_input",
                AxisSource::Effective => "telemetry_effective",
            }
            .into();
            if calibration.geometry_fingerprint.is_none() {
                calibration.issues.push("recording lacks physical /config fingerprint; live control cannot be authorized".into());
                calibration.accepted_for_control = false;
            }
            if let Some(path) = envelope_measurement {
                let mut measurement: etsads_control::identification::EnvelopeMeasurement =
                    serde_json::from_reader(File::open(&path)?)?;
                let evidence = std::path::Path::new(&measurement.evidence_path);
                let evidence = if evidence.is_absolute() {
                    evidence.to_path_buf()
                } else {
                    path.parent()
                        .unwrap_or(std::path::Path::new("."))
                        .join(evidence)
                };
                ensure!(
                    evidence.is_file(),
                    "body-envelope measurement evidence file is missing"
                );
                measurement.evidence_path = evidence.canonicalize()?.to_string_lossy().into_owned();
                calibration.envelope_measurement = Some(measurement);
            }
            if let Some(path) = front_bumper_measurement {
                let mut measurement: etsads_control::identification::FrontBumperMeasurement =
                    serde_json::from_reader(File::open(&path)?)?;
                let evidence = std::path::Path::new(&measurement.evidence_path);
                let evidence = if evidence.is_absolute() {
                    evidence.to_path_buf()
                } else {
                    path.parent()
                        .unwrap_or(std::path::Path::new("."))
                        .join(evidence)
                };
                ensure!(
                    evidence.is_file(),
                    "front bumper exterior-measurement evidence file is missing"
                );
                measurement.evidence_path = evidence.canonicalize()?.to_string_lossy().into_owned();
                calibration.front_bumper = Some(measurement);
                ensure!(
                    calibration.measured_front_bumper().is_some(),
                    "front bumper measurement is outside measured body bounds or belongs to another truck geometry"
                );
            }
            if let Err(error) = calibration.validate_measured_envelope() {
                calibration.issues.push(error.to_string());
                calibration.accepted_for_control = false;
            }
            if replay.frame_gaps > 0 {
                calibration
                    .issues
                    .push(format!("{} missing telemetry frames", replay.frame_gaps));
                calibration.accepted_for_control = false;
            }
            if !matches!(steering_source, AxisSource::Applied) {
                calibration.issues.push(
                    "telemetry-axis fit does not establish semantic plugin input mapping; use --steering-source applied with active SDK inputs".into(),
                );
                calibration.accepted_for_control = false;
            }
            let extension = output
                .extension()
                .and_then(|e| e.to_str())
                .unwrap_or("json");
            let mut file = create_output(&output)?;
            if extension == "toml" {
                use std::io::Write;
                file.write_all(toml::to_string_pretty(&calibration)?.as_bytes())?;
            } else {
                serde_json::to_writer_pretty(file, &calibration)?;
            }
            println!(
                "Calibration saved: {}. accepted={} samples={} skipped={} issues={}",
                output.display(),
                calibration.accepted_for_control,
                samples.len(),
                skipped,
                calibration.issues.len()
            );
            for issue in calibration.issues {
                println!("- {issue}");
            }
        }
    }
    Ok(())
}
