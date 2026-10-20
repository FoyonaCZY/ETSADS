fn main() -> Result<(), Box<dyn std::error::Error>> {
    let report = etsads_runtime::navigation_regression::run_native_navigation_regression()
        .map_err(std::io::Error::other)?;
    let path = std::env::args()
        .nth(1)
        .ok_or("provide a new output JSON path")?;
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)?;
    serde_json::to_writer_pretty(&file, &report)?;
    file.sync_all()?;
    Ok(())
}
