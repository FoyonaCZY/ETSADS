//! Open-loop replay of immutable observations through the production Runtime.
//! There is deliberately no Live mode, gate, shared-memory writer or rearming.
use anyhow::{Context, ensure};
use etsads_control::identification::VehicleCalibration;
use etsads_map::importer::{ExportedMap, RegionPolicy, load_parquet_region};
use etsads_msgs::Point3;
use etsads_replay::ReplayAdapter;
use etsads_runtime::{ExecutionMode, Runtime, RuntimeConfig};
use serde_json::json;
use std::{
    collections::BTreeMap,
    fs::OpenOptions,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

pub struct Options {
    pub run: PathBuf,
    pub config: PathBuf,
    pub calibration: PathBuf,
    pub output: PathBuf,
    pub map: Option<PathBuf>,
    pub policy: Option<PathBuf>,
    pub region_radius_m: f64,
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> anyhow::Result<T> {
    use std::io::Read;
    let mut bytes = Vec::new();
    std::fs::File::open(path)
        .with_context(|| format!("read {}", path.display()))?
        .take(8 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= 8 * 1024 * 1024,
        "configuration exceeds 8 MiB"
    );
    serde_json::from_slice(&bytes).with_context(|| format!("parse {}", path.display()))
}

pub fn run(options: Options) -> anyhow::Result<serde_json::Value> {
    let config: RuntimeConfig = read_json(&options.config)?;
    ensure!(
        config.execution == ExecutionMode::Replay,
        "replay-drive requires execution=Replay; Live is never accepted"
    );
    let calibration: VehicleCalibration = read_json(&options.calibration)?;
    ensure!(
        options.map.is_some() == options.policy.is_some(),
        "map and policy must be supplied together"
    );
    ensure!(
        options.region_radius_m.is_finite() && (50.0..=20_000.0).contains(&options.region_radius_m),
        "region radius must be 50..20000 m"
    );
    ensure!(
        !options.output.exists(),
        "output already exists; choose a new directory"
    );
    std::fs::create_dir_all(options.output.parent().unwrap_or(Path::new(".")))?;
    std::fs::create_dir(&options.output)?;
    let output_file = OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(options.output.join("runtime.jsonl"))?;
    let mut output = BufWriter::new(output_file);
    let mut runtime = None;
    let mut map_session = None;
    let mut statuses = BTreeMap::<String, u64>::new();
    let mut first_fault = None;
    let mut frames = 0;
    let execution = ReplayAdapter::run(&options.run, |frame| {
        if runtime.is_none() {
            let mut native_region = None;
            let region = match (&options.map, &options.policy) {
                (Some(map), Some(policy)) => {
                    let policy: RegionPolicy = read_json(policy)?;
                    if matches!(
                        config.route_source,
                        etsads_runtime::RouteSource::NativeGps(_)
                    ) {
                        let (session, navigation) =
                            etsads::driving::map_session::MapSession::start(
                                map,
                                policy,
                                &config,
                                options.config.parent().unwrap_or(Path::new(".")),
                                &frame.snapshot,
                            )?;
                        let region = navigation.region.clone();
                        map_session = Some(session);
                        native_region = Some(navigation);
                        Some(region)
                    } else {
                        let exported = if map.is_dir() {
                            load_parquet_region(
                                map,
                                Point3 {
                                    x: frame.snapshot.ego.pose.x,
                                    y: frame.snapshot.ego.pose.y,
                                    z: frame.snapshot.ego.z,
                                },
                                options.region_radius_m,
                            )?
                        } else {
                            ExportedMap::from_path(map)?
                        };
                        Some(exported.into_commissioning_region(
                            policy,
                            frame.snapshot.raw.navigation_speed_limit_mps as f64,
                        )?)
                    }
                }
                _ => None,
            };
            let mut initialized = Runtime::new(config.clone(), calibration.clone(), region)?;
            if let Some(navigation) = native_region {
                initialized.install_native_region(navigation)?;
            }
            runtime = Some(initialized);
        }
        if let Some(session) = &mut map_session
            && let Err(error) = session.poll_replay(&frame.snapshot, runtime.as_mut().unwrap())
        {
            runtime
                .as_mut()
                .unwrap()
                .report_navigation_failure(format!("{error:#}"));
        }
        let result = runtime.as_mut().unwrap().step(&frame.snapshot);
        let status = format!("{:?}", result.status);
        *statuses.entry(status).or_default() += 1;
        if result.status == etsads_runtime::RuntimeStatus::Fault && first_fault.is_none() {
            first_fault =
                Some(json!({"frame_id": result.header.frame_id, "reason": result.reason}));
        }
        // Inputs remain the recorded human/game values. The Runtime's
        // proposed actuation is logged, never fed back into these frames.
        let row = json!({"execution":"open_loop_replay", "frame_id":frame.snapshot.raw.header.frame_id,
                "qpc":frame.snapshot.raw.header.qpc, "recorded_input":frame.snapshot.raw.input,
                "recorded_effective":frame.snapshot.raw.effective, "extension_available":frame.snapshot.extension.is_some(),
                "events":frame.snapshot.events.len(), "video_indices":frame.video.len(), "runtime":result});
        serde_json::to_writer(&mut output, &row)?;
        output.write_all(b"\n")?;
        frames += 1;
        Ok(())
    });
    output.flush()?;
    output.get_ref().sync_all()?;
    let report = json!({"schema_version":1, "execution":"open_loop_replay", "source_run":options.run,
        "runtime_config":config, "calibration":calibration,
        "completed":execution.is_ok(), "frames":frames, "status_counts":statuses,
        "first_fault":first_fault, "phase_acceptance":false, "control_written":false,
        "replay":execution.as_ref().ok(), "error":execution.as_ref().err().map(|error| format!("{error:#}")),
        "limitations":["Recorded observations do not respond to proposed control; this is not a closed-loop vehicle test.",
            "Replay calibration and source-game observations cannot certify live driving."]});
    let report_file = OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(options.output.join("report.json"))?;
    serde_json::to_writer_pretty(&report_file, &report)?;
    report_file.sync_all()?;
    execution?;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use etsads_control::identification::{LongitudinalModel, SteeringRow};
    use etsads_msgs::{VehicleGeometry, protocol::*};
    use etsads_recorder::{Record, Recorder, RunMetadata};

    #[test]
    fn recorded_native_gps_enters_runtime_and_advances_the_background_map_window() {
        let temp_parent = std::env::temp_dir().canonicalize().unwrap();
        let root = temp_parent.join(format!(
            "etsads-native-replay-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&root).unwrap();
        let source = root.join("recording");
        let mut recorder =
            Recorder::create(&source, RunMetadata::new("synthetic", "native-gps-fixture")).unwrap();
        for n in 1..=1050 {
            let mut raw = TelemetryFrame {
                header: WireHeader {
                    frame_id: n,
                    sim_time_us: n * 100000,
                    qpc: n * 1000000,
                    ..Default::default()
                },
                qpc_frequency: 10000000,
                paused_simulation_time_us: n * 100000,
                position_scs: [1800.0 + n as f64 * 0.6, 0.0, 0.0],
                orientation_turns: [0.75, 0.0, 0.0],
                speed_mps: 6.0,
                navigation_speed_limit_mps: 8.0,
                ..Default::default()
            };
            raw.trailers[0].connected = 1;
            raw.trailers[0].orientation_turns[0] = 0.75;
            recorder
                .record("/telemetry", &Record::telemetry(&raw))
                .unwrap();
            let mut extension = Box::<ExtensionFrame>::default();
            extension.header = raw.header;
            extension.available = 18;
            for (index, uid) in [2, 3, 4].into_iter().enumerate() {
                let node = etsads_game::ets2la::RouteNode {
                    node_uid: uid,
                    distance_to_end: 6000.0 - raw.position_scs[0] as f32,
                    time_to_end: 500.0,
                };
                extension.route[index * 16..(index + 1) * 16]
                    .copy_from_slice(bytemuck::bytes_of(&node));
            }
            recorder
                .record(
                    "/extension",
                    &Record::wire(extension.as_ref(), n, n * 100000, n * 1000000),
                )
                .unwrap();
        }
        recorder.finish().unwrap();
        let (config, mut calibration, map) =
            etsads_runtime::navigation_regression::synthetic_disk_navigation_fixture(1800.6, 1);
        calibration.conditions = None;
        let config_path = root.join("runtime.json");
        let calibration_path = root.join("calibration.json");
        let policy_path = root.join("policy.json");
        std::fs::write(&config_path, serde_json::to_vec(&config).unwrap()).unwrap();
        std::fs::write(&calibration_path, serde_json::to_vec(&calibration).unwrap()).unwrap();
        std::fs::write(
            &policy_path,
            serde_json::to_vec(&map.authorization).unwrap(),
        )
        .unwrap();
        let report = run(Options {
            run: source,
            config: config_path,
            calibration: calibration_path,
            output: root.join("result"),
            map: Some(
                Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../crates/map/testdata/navigation-sectors"),
            ),
            policy: Some(policy_path),
            region_radius_m: 2000.0,
        })
        .unwrap();
        assert!(report["first_fault"].is_null(), "{report}");
        assert_eq!(report["control_written"], false);
        assert_eq!(report["phase_acceptance"], false);
        let rows = std::fs::read_to_string(root.join("result/runtime.jsonl")).unwrap();
        let last: serde_json::Value = serde_json::from_str(rows.lines().last().unwrap()).unwrap();
        assert_eq!(last["runtime"]["navigation"]["map"]["generation"], 2);
        assert_eq!(
            last["runtime"]["navigation"]["gps_node_uids"],
            serde_json::json!([2, 3, 4])
        );
        assert_eq!(
            last["runtime"]["navigation"]["corridor_source_ids"],
            serde_json::json!(["m", "z"])
        );
        let resolved = root.canonicalize().unwrap();
        assert!(resolved.starts_with(&temp_parent));
        std::fs::remove_dir_all(resolved).unwrap();
    }

    #[test]
    fn recorded_frames_execute_selected_runtime_controller_and_never_accept_live_mode() {
        let root = std::env::temp_dir().join(format!(
            "etsads-open-loop-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&root).unwrap();
        let source = root.join("source");
        let mut recorder =
            Recorder::create(&source, RunMetadata::new("synthetic", "fixture")).unwrap();
        for n in 1..=24 {
            let raw = TelemetryFrame {
                header: WireHeader {
                    frame_id: n,
                    sim_time_us: n * 16_667,
                    qpc: n * 166_670,
                    ..Default::default()
                },
                qpc_frequency: 10_000_000,
                paused_simulation_time_us: n * 16_667,
                position_scs: [n as f64 * 0.1, 0.0, 0.0],
                orientation_turns: [0.75, 0.0, 0.0],
                speed_mps: 6.0,
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&raw))
                .unwrap();
        }
        recorder.finish().unwrap();
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_length: 0.0,
            trailer_wheelbase: 0.0,
            ..Default::default()
        };
        let calibration = VehicleCalibration {
            dynamic_speed_range_mps: None,
            schema_version: 1,
            vehicle_id: "offline-fixture".into(),
            source_run: "synthetic fixture".into(),
            evidence: "synthetic calibration; never live".into(),
            geometry,
            geometry_fingerprint: None,
            steering_axis_source: "synthetic".into(),
            reference: None,
            conditions: None,
            front_bumper: None,
            envelope_measurement: None,
            sample_count: 1000,
            steering_rows: [0.0, 10.0]
                .into_iter()
                .map(|speed| SteeringRow {
                    speed,
                    samples: vec![(-1.0, -0.15), (0.0, 0.0), (1.0, 0.15)],
                    counts: vec![100; 3],
                    rms_curvature: 0.0,
                })
                .collect(),
            delay_s: Some(0.1),
            steering_tau_s: Some(0.1),
            delay_correlation: Some(1.0),
            longitudinal: Some(LongitudinalModel {
                coefficients: [-0.1, 1.2, -4.0, 0.0, 0.0],
                training_rmse: 0.0,
                validation_rmse: 0.0,
                sample_count: 1000,
            }),
            accepted_for_control: true,
            issues: vec![],
        };
        let calibration_path = root.join("synthetic-calibration.json");
        std::fs::write(&calibration_path, serde_json::to_vec(&calibration).unwrap()).unwrap();
        let config_path = root.join("runtime.json");
        let mut config = json!({"execution":"Replay","vehicle_id":"offline-fixture","vehicle_configuration_verified":true,
            "forward_lateral":{"kind":"ltv_mpc","lateral_acceleration_limit_mps2":1.2,"maximum_solve_time_ms":50.0},
            "mode":{"CommissioningCircle":{"radius_m":50.0,"speed_mps":6.0,"clockwise":false,
                "verified_clear_area":[[-200.0,-200.0],[200.0,-200.0],[200.0,200.0],[-200.0,200.0]]}}});
        std::fs::write(&config_path, serde_json::to_vec(&config).unwrap()).unwrap();
        let options = |output| Options {
            run: source.clone(),
            config: config_path.clone(),
            calibration: calibration_path.clone(),
            output: root.join(output),
            map: None,
            policy: None,
            region_radius_m: 2000.0,
        };
        let report = run(options("result")).unwrap();
        assert_eq!(report["frames"], 24);
        assert_eq!(report["control_written"], false);
        assert_eq!(report["phase_acceptance"], false);
        assert!(report["first_fault"].is_null(), "{report}");
        let rows = std::fs::read_to_string(root.join("result/runtime.jsonl")).unwrap();
        let values: Vec<serde_json::Value> = rows
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert!(
            values.iter().any(
                |value| value["runtime"]["lateral_control"]["qp_solved"] == true
                    && value["runtime"]["lateral_control"]["executed"] == "ltv_mpc"
            )
        );
        config["execution"] = json!("Live");
        std::fs::write(&config_path, serde_json::to_vec(&config).unwrap()).unwrap();
        assert!(
            run(options("rejected"))
                .unwrap_err()
                .to_string()
                .contains("execution=Replay")
        );
        assert!(!root.join("rejected").exists());
        std::fs::remove_dir_all(root).unwrap();
    }
}
