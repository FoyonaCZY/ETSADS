fn main() -> anyhow::Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: run_metrics <run directory>"))?;
    let report = etsads_metrics::analyze(std::path::Path::new(&path))?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
