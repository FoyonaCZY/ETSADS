fn main() -> Result<(), Box<dyn std::error::Error>> {
    use etsads_runtime::recovery_regression::{RecoveryScenario, run_recovery};
    let directory = std::path::PathBuf::from(
        std::env::args()
            .nth(1)
            .unwrap_or_else(|| "artifacts/runtime-recovery-evidence".into()),
    );
    std::fs::create_dir_all(&directory)?;
    let mut all = true;
    for (name, scenario) in [
        ("successful-pull-up", RecoveryScenario::SuccessfulPullUp),
        ("no-space", RecoveryScenario::NoSpace),
        (
            "missing-gear-ack",
            RecoveryScenario::MissingForwardAcknowledgement,
        ),
        ("attempt-limit", RecoveryScenario::AttemptLimit),
    ] {
        let report = run_recovery(scenario)?;
        std::fs::write(
            directory.join(format!("{name}.json")),
            serde_json::to_vec_pretty(&report)?,
        )?;
        println!(
            "offline_parking_recovery {name}: passed={} completed={} attempts={} disturbances={} constraint_violations={} target_error_m={:.3} final_speed_mps={:.4} failure={:?}",
            report.passed,
            report.completed,
            report.recovery_attempts,
            report.disturbances,
            report.constraint_violations,
            report.final_position_error_m,
            report.final_speed_mps,
            report.failure
        );
        all &= report.passed;
    }
    if !all {
        return Err("parking recovery regression failed".into());
    }
    Ok(())
}
