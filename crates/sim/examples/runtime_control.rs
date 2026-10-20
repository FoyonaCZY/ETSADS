fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = std::path::PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "artifacts/runtime-mpc-evidence".into()),
    );
    std::fs::create_dir_all(&directory)?;
    let config_path = directory.join("runtime-mpc-config.json");
    let config = etsads_runtime::control_regression::mpc_circle_config();
    std::fs::write(&config_path, serde_json::to_vec_pretty(&config)?)?;
    let report = etsads_runtime::control_regression::run_mpc_circle_json(
        &std::fs::read_to_string(&config_path)?,
        20.0,
    )?;
    std::fs::write(
        directory.join("trajectory-and-state.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!(
        "offline_runtime_mpc success={} frames={} warm_linearizations={} rms_m={:.6} solve_p50_ms={:.3} solve_p95_ms={:.3} solve_max_ms={:.3} deadline_failures={} actual_acceleration_violations={} failure={:?}",
        report.success,
        report.executed_mpc_frames,
        report.linearization_warm_frames,
        report.rms_error_m,
        report.solve_p50_ms,
        report.solve_p95_ms,
        report.solve_max_ms,
        report.solve_deadline_failures,
        report.lateral_acceleration_violations,
        report.failure
    );
    if !report.success {
        return Err("Runtime MPC closed-loop regression failed".into());
    }
    Ok(())
}
