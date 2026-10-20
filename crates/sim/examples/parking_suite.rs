fn main() -> Result<(), Box<dyn std::error::Error>> {
    let start = std::time::Instant::now();
    let suite = etsads_sim::parking::run_parking_suite();
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "target/parking-suite.json".into());
    std::fs::write(&path, serde_json::to_vec_pretty(&suite)?)?;
    println!(
        "successes={}/{} rate={:.1}% elapsed={:.2}s report={}",
        suite.successes,
        suite.attempts,
        suite.success_rate * 100.0,
        start.elapsed().as_secs_f64(),
        path
    );
    for trial in &suite.trials {
        if let Some(error) = &trial.failure {
            println!("trial {}: {}", trial.index, error);
        } else if let Some(r) = &trial.report
            && !r.success
        {
            println!(
                "trial {}: position={:.3}m heading={:.2}deg interventions={} collisions={}",
                trial.index,
                r.position_error_m,
                r.heading_error_rad.to_degrees(),
                r.interventions,
                r.collision_count
            );
        }
    }
    Ok(())
}
