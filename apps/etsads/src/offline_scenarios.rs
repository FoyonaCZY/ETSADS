use anyhow::{Context, ensure};
use etsads_map::importer::{ExportedMap, RegionGraph, RegionPolicy};
use etsads_sim::recording::{
    BaselineDocument, BaselineIdentity, ExtractionSettings, ScenarioLibrary,
};
use std::{
    collections::BTreeMap,
    fs::OpenOptions,
    io::{BufWriter, Read, Write},
    path::Path,
};
fn json<T: serde::de::DeserializeOwned>(path: &Path) -> anyhow::Result<T> {
    let mut bytes = vec![];
    std::fs::File::open(path)?
        .take(8 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= 8 * 1024 * 1024,
        "scenario configuration exceeds 8 MiB"
    );
    serde_json::from_slice(&bytes).with_context(|| format!("parse {}", path.display()))
}
fn region(map: &Path, policy: &Path) -> anyhow::Result<RegionGraph> {
    let policy: RegionPolicy = json(policy)?;
    let limit = policy.maximum_speed_mps;
    Ok(ExportedMap::from_path(map)?.into_commissioning_region(policy, limit)?)
}
fn write_new(path: &Path, value: &impl serde::Serialize) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let file = OpenOptions::new().write(true).create_new(true).open(path)?;
    serde_json::to_writer_pretty(&file, value)?;
    file.sync_all()?;
    Ok(())
}
pub fn extract(
    run: &Path,
    map: &Path,
    policy: &Path,
    settings: Option<&Path>,
    output: &Path,
) -> anyhow::Result<ScenarioLibrary> {
    ensure!(!output.exists(), "output already exists");
    let settings: ExtractionSettings = settings.map(json).transpose()?.unwrap_or_default();
    let region = region(map, policy)?;
    let library = etsads_sim::recording::extract(run, &region.imported.graph, settings)?;
    write_new(output, &library)?;
    Ok(library)
}
pub struct RunOptions<'a> {
    pub run: &'a Path,
    pub library: &'a Path,
    pub map: &'a Path,
    pub policy: &'a Path,
    pub config: &'a Path,
    pub calibration: &'a Path,
    pub baseline: Option<&'a Path>,
    pub output: &'a Path,
}
pub fn run(options: RunOptions<'_>) -> anyhow::Result<serde_json::Value> {
    let library: ScenarioLibrary = json(options.library)?;
    let config: etsads_runtime::RuntimeConfig = json(options.config)?;
    let calibration: etsads_control::identification::VehicleCalibration =
        json(options.calibration)?;
    let region = region(options.map, options.policy)?;
    ensure!(
        config.execution == etsads_runtime::ExecutionMode::Replay,
        "scenario-run requires execution Replay"
    );
    ensure!(
        !library.cases.is_empty() && library.cases.len() <= 128,
        "library must contain 1..128 scenarios; empty extraction is not an acceptance pass"
    );
    ensure!(
        library
            .cases
            .iter()
            .map(|case| &case.id)
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            == library.cases.len(),
        "duplicate case IDs"
    );
    let identity = BaselineIdentity::new(&library, &config, &calibration, &region.policy)?;
    let baseline: Option<BaselineDocument> = options.baseline.map(json).transpose()?;
    if let Some(baseline) = &baseline {
        baseline.validate_for(&identity)?;
    }
    ensure!(!options.output.exists(), "output already exists");
    std::fs::create_dir_all(options.output.parent().unwrap_or(Path::new(".")))?;
    std::fs::create_dir(options.output)?;
    let mut metrics = BTreeMap::new();
    let mut failures = vec![];
    for (index, case) in library.cases.iter().enumerate() {
        let directory = options.output.join(format!("case-{index:03}"));
        std::fs::create_dir(&directory)?;
        let mut stream = BufWriter::new(
            OpenOptions::new()
                .create_new(true)
                .write(true)
                .open(directory.join("steps.jsonl"))?,
        );
        let result = etsads_sim::recording::run_case(
            options.run,
            &library,
            case,
            config.clone(),
            calibration.clone(),
            region.clone(),
            |step| {
                serde_json::to_writer(&mut stream, step)?;
                stream.write_all(b"\n")?;
                Ok(())
            },
        );
        stream.flush()?;
        stream.get_ref().sync_all()?;
        match result {
            Ok(report) => {
                if !report.completed || report.metrics.collision_frames > 0 {
                    failures.push(format!("{}: fault or collision", case.id));
                }
                if let Some(previous) = baseline.as_ref().and_then(|b| b.metrics.get(&case.id))
                    && let Err(error) =
                        etsads_sim::recording::check_regression(previous, &report.metrics)
                {
                    failures.push(format!("{}: {error}", case.id));
                }
                write_new(&directory.join("report.json"), &report)?;
                metrics.insert(case.id.clone(), report.metrics);
            }
            Err(error) => {
                let error = format!("{error:#}");
                write_new(
                    &directory.join("error.json"),
                    &serde_json::json!({"case":case,"error":error,"phase_acceptance":false}),
                )?;
                failures.push(format!("{}: {error}", case.id));
            }
        }
    }
    let processed = metrics.len();
    let document = BaselineDocument {
        schema_version: 1,
        identity,
        completed: failures.is_empty(),
        metrics,
    };
    write_new(&options.output.join("metrics.json"), &document)?;
    let summary = serde_json::json!({"execution":"recording_seeded_closed_loop","total":library.cases.len(),"processed":processed,"failures":failures,"baseline_compared":baseline.is_some(),"phase_acceptance":false,"control_written":false});
    write_new(&options.output.join("summary.json"), &summary)?;
    ensure!(
        failures.is_empty(),
        "scenario library contains failures; see summary.json"
    );
    Ok(summary)
}
