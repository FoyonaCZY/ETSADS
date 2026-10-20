fn main() -> Result<(), Box<dyn std::error::Error>> {
    let start = std::time::Instant::now();
    let suite = etsads_sim::run_suite()?;
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "target/core-suite.json".into());
    std::fs::write(&path, serde_json::to_vec_pretty(&suite)?)?;
    println!(
        "offline_checks={}/{} parking={}/{} runtime_replays={}/{} elapsed={:.2}s report={}",
        suite.checks.iter().filter(|c| c.passed).count(),
        suite.checks.len(),
        suite.parking_successes,
        suite.parking_attempts,
        suite
            .runtime_replay
            .cases
            .iter()
            .filter(|c| c.passed)
            .count(),
        suite.runtime_replay.cases.len(),
        start.elapsed().as_secs_f64(),
        path
    );
    if !suite.all_passed {
        for check in suite.checks.iter().filter(|c| !c.passed) {
            eprintln!(
                "FAILED {} {} measured={}",
                check.stage, check.name, check.measured
            );
        }
        return Err("offline regression suite failed".into());
    }
    Ok(())
}
