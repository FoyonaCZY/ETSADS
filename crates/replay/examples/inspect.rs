fn main() -> anyhow::Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: inspect <closed-run>"))?;
    let report = etsads_replay::ReplayAdapter::run(std::path::Path::new(&path), |_| Ok(()))?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
