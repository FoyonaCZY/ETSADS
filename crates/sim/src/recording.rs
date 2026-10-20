//! Recording-seeded scenarios. Sources are immutable; all rolled-out states
//! remain simulated, even when the seed was recorded in a real game.
mod world;
pub use world::{CaseMetrics, CaseReport, SimStep, run_case};
#[cfg(test)]
mod tests;
use anyhow::{Context, ensure};
use etsads_game::{adapter::GameSnapshot, ets2la};
use etsads_map::LaneGraph;
use etsads_msgs::{Point3, frames::wrap_angle};
use etsads_replay::ReplayAdapter;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    io::Read,
    path::Path,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Trigger {
    Takeover,
    HardBraking,
    RedLightApproach,
    Roundabout,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExtractionSettings {
    pub pre_seconds: f64,
    pub post_seconds: f64,
    /// Explicit map-classified roundabout IDs, never guessed from road names.
    pub roundabout_lane_ids: Vec<u64>,
}
impl Default for ExtractionSettings {
    fn default() -> Self {
        Self {
            pre_seconds: 10.0,
            post_seconds: 5.0,
            roundabout_lane_ids: vec![],
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordedCase {
    pub id: String,
    pub trigger: Trigger,
    pub trigger_frame: u64,
    pub trigger_sim_time_us: u64,
    pub start_frame: u64,
    pub start_sim_time_us: u64,
    pub end_frame: u64,
    pub end_sim_time_us: u64,
    pub complete_pre_window: bool,
    pub complete_post_window: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioLibrary {
    pub schema_version: u32,
    pub source: String,
    pub source_hashes: BTreeMap<String, String>,
    pub map_sha256: String,
    pub settings: ExtractionSettings,
    pub cases: Vec<RecordedCase>,
    pub phase_acceptance: bool,
    pub notes: Vec<String>,
}
/// Immutable inputs that must remain identical across a metric comparison.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BaselineIdentity {
    pub source_hashes: BTreeMap<String, String>,
    pub map_sha256: String,
    pub cases: Vec<RecordedCase>,
    pub runtime_configuration_sha256: String,
    pub calibration_sha256: String,
    pub region_policy_sha256: String,
}
impl BaselineIdentity {
    pub fn new(
        library: &ScenarioLibrary,
        config: &etsads_runtime::RuntimeConfig,
        calibration: &etsads_control::identification::VehicleCalibration,
        policy: &etsads_map::importer::RegionPolicy,
    ) -> anyhow::Result<Self> {
        let hash = |value: serde_json::Value| -> anyhow::Result<String> {
            Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(&value)?)))
        };
        Ok(Self {
            source_hashes: library.source_hashes.clone(),
            map_sha256: library.map_sha256.clone(),
            cases: library.cases.clone(),
            runtime_configuration_sha256: hash(serde_json::to_value(config)?)?,
            calibration_sha256: hash(serde_json::to_value(calibration)?)?,
            region_policy_sha256: hash(serde_json::to_value(policy)?)?,
        })
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BaselineDocument {
    pub schema_version: u32,
    pub identity: BaselineIdentity,
    pub completed: bool,
    pub metrics: BTreeMap<String, CaseMetrics>,
}
impl BaselineDocument {
    pub fn validate_for(&self, identity: &BaselineIdentity) -> anyhow::Result<()> {
        ensure!(
            self.schema_version == 1 && self.completed && self.identity == *identity,
            "baseline source, map, case windows, configuration or calibration changed, or baseline incomplete"
        );
        ensure!(
            self.metrics.len() == identity.cases.len(),
            "baseline must include every case"
        );
        for case in &identity.cases {
            ensure!(
                case.end_sim_time_us > case.start_sim_time_us,
                "invalid baseline window"
            );
            let metrics = self
                .metrics
                .get(&case.id)
                .context("baseline case missing")?;
            ensure!(
                metrics.fault.is_none()
                    && metrics.collision_frames == 0
                    && metrics.simulated_seconds + 0.020001
                        >= (case.end_sim_time_us - case.start_sim_time_us) as f64 / 1e6,
                "baseline contains failed or truncated scenario"
            );
            check_regression(metrics, metrics)?;
        }
        Ok(())
    }
}
/// Every compared metric has an explicit direction. Missing metrics cannot
/// improve a baseline; shorter/faulted simulation cannot pass by doing less.
pub fn check_regression(previous: &CaseMetrics, current: &CaseMetrics) -> anyhow::Result<()> {
    ensure!(
        previous.fault.is_none(),
        "failed baseline cannot certify a regression check"
    );
    for metrics in [previous, current] {
        ensure!(
            [
                metrics.simulated_seconds,
                metrics.distance_m,
                metrics.maximum_deceleration_mps2,
                metrics.maximum_lateral_error_m
            ]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0)
                && metrics.minimum_agent_gap_m.is_none_or(f64::is_finite),
            "invalid baseline or result metric"
        );
    }
    ensure!(current.fault.is_none(), "runtime fault");
    ensure!(
        current.simulated_seconds + 1e-6 >= previous.simulated_seconds,
        "duration regressed"
    );
    ensure!(
        current.distance_m + 1e-6 >= previous.distance_m,
        "progress regressed"
    );
    ensure!(
        current.collision_frames <= previous.collision_frames,
        "collisions regressed"
    );
    ensure!(
        current.maximum_deceleration_mps2 <= previous.maximum_deceleration_mps2 + 1e-6,
        "deceleration regressed"
    );
    ensure!(
        current.maximum_lateral_error_m <= previous.maximum_lateral_error_m + 1e-6,
        "lateral error regressed"
    );
    ensure!(
        current.unmapped_agents <= previous.unmapped_agents,
        "map coverage regressed"
    );
    if let Some(old) = previous.minimum_agent_gap_m {
        ensure!(
            current
                .minimum_agent_gap_m
                .is_some_and(|new| new + 1e-6 >= old),
            "minimum lead gap regressed or unavailable"
        );
    }
    Ok(())
}
fn digest(path: &Path) -> anyhow::Result<String> {
    let mut file = std::fs::File::open(path)?;
    let mut hash = Sha256::new();
    let mut buffer = [0; 65536];
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        hash.update(&buffer[..n]);
    }
    Ok(format!("{:x}", hash.finalize()))
}
pub fn source_hashes(run: &Path) -> anyhow::Result<BTreeMap<String, String>> {
    let mut result = BTreeMap::new();
    for entry in std::fs::read_dir(run)? {
        let entry = entry?;
        let path = entry.path();
        if path.extension().is_some_and(|e| e == "mcap") || entry.file_name() == "meta.json" {
            ensure!(
                entry.file_type()?.is_file(),
                "recording source must be a regular file"
            );
            result.insert(
                entry.file_name().to_string_lossy().into_owned(),
                digest(&path)?,
            );
        }
    }
    ensure!(
        result.len() >= 2 && result.len() <= 1024,
        "recording source file count outside bounds"
    );
    Ok(result)
}
pub fn graph_hash(graph: &LaneGraph) -> anyhow::Result<String> {
    Ok(format!("{:x}", Sha256::digest(graph.to_json()?.as_bytes())))
}
pub fn extract(
    run: &Path,
    graph: &LaneGraph,
    settings: ExtractionSettings,
) -> anyhow::Result<ScenarioLibrary> {
    ensure!(
        (1.0..=30.0).contains(&settings.pre_seconds)
            && (1.0..=30.0).contains(&settings.post_seconds),
        "scenario windows must be 1..30 seconds"
    );
    ensure!(
        settings.roundabout_lane_ids.len() <= 4096
            && settings
                .roundabout_lane_ids
                .iter()
                .all(|id| graph.lanes.contains_key(id)),
        "roundabout scope contains missing lanes or exceeds limit"
    );
    let hashes = source_hashes(run)?;
    // /stack describes actual control-request edges, not guessed physical input.
    let mut takeovers = BTreeSet::new();
    let mut requested = false;
    let integrity = etsads_recorder::replay(run, |topic, record| {
        if topic == "/stack" {
            ensure!(
                record.raw_payload.len() <= 8 * 1024 * 1024,
                "stack record exceeds budget"
            );
            let row: serde_json::Value = serde_json::from_slice(&record.raw_payload)?;
            let row = row.get("runtime").unwrap_or(&row);
            if let Some(next) = row.get("request_control").and_then(|v| v.as_bool()) {
                if requested && !next {
                    takeovers.insert(record.frame_id);
                }
                requested = next;
            }
            ensure!(
                takeovers.len() <= 4096,
                "too many takeover edges; split recording"
            );
        }
        Ok(())
    })?;
    let pre = (settings.pre_seconds * 1e6) as u64;
    let post = (settings.post_seconds * 1e6) as u64;
    let mut history = VecDeque::new();
    let mut cases: Vec<RecordedCase> = vec![];
    let mut active = BTreeSet::new();
    let mut last_trigger = BTreeMap::<Trigger, u64>::new();
    let mut segment_start = 0;
    let mut previous_time = None;
    let mut closed = BTreeSet::new();
    ReplayAdapter::run(run, |frame| {
        let snapshot = frame.snapshot;
        let ego = snapshot.ego;
        let time = ego.header.sim_time_us;
        let id = ego.header.frame_id;
        if ego.paused {
            previous_time = None;
            for case in &cases {
                closed.insert(case.id.clone());
            }
            return Ok(());
        }
        if previous_time.is_none_or(|t| time <= t || time - t > 200_000) {
            for case in &cases {
                closed.insert(case.id.clone());
            }
            history.clear();
            active.clear();
            segment_start = time;
        }
        previous_time = Some(time);
        history.push_back((id, time));
        while history.len() > 1
            && history
                .get(1)
                .is_some_and(|(_, t)| *t <= time.saturating_sub(pre))
        {
            history.pop_front();
        }
        ensure!(
            history.len() <= 10000,
            "physical frame rate exceeds extraction budget"
        );
        let mut triggers = BTreeSet::new();
        if takeovers.contains(&id) {
            triggers.insert(Trigger::Takeover);
        }
        if ego.acceleration < -3.0 {
            triggers.insert(Trigger::HardBraking);
        }
        let matched = nearest(graph, ego.pose, ego.z);
        if matched.is_some_and(|p| settings.roundabout_lane_ids.contains(&p.lane_id)) {
            triggers.insert(Trigger::Roundabout);
        }
        if ego.speed > 2.0
            && snapshot.extension.as_ref().is_some_and(|extension| {
                extension.available & 8 != 0
                    && ets2la::records::<ets2la::Signal>(&extension.signals).any(|signal| {
                        if signal.kind != 1 || !matches!(signal.state, 2 | 4) {
                            return false;
                        }
                        let p = ets2la::sector_world(signal.position, signal.sector);
                        let dx = p[0] - ego.pose.x;
                        let dy = -p[2] - ego.pose.y;
                        let ahead = dx * ego.pose.yaw.cos() + dy * ego.pose.yaw.sin();
                        let lateral = -dx * ego.pose.yaw.sin() + dy * ego.pose.yaw.cos();
                        ahead > 0.0
                            && ahead < 25.0
                            && lateral.abs() < 6.0
                            && (p[1] - ego.z).abs() < 5.0
                    })
            })
        {
            triggers.insert(Trigger::RedLightApproach);
        }
        for trigger in triggers.difference(&active) {
            if last_trigger
                .get(trigger)
                .is_some_and(|t| time.saturating_sub(*t) < pre + post)
            {
                continue;
            }
            ensure!(
                cases.len() < 128,
                "scenario count exceeds 128; split recording"
            );
            let (start_frame, start_time) = *history.front().unwrap();
            cases.push(RecordedCase {
                id: format!("{:?}-{id}", trigger),
                trigger: *trigger,
                trigger_frame: id,
                trigger_sim_time_us: time,
                start_frame,
                start_sim_time_us: start_time,
                end_frame: id,
                end_sim_time_us: time,
                complete_pre_window: time.saturating_sub(segment_start) >= pre,
                complete_post_window: false,
            });
            last_trigger.insert(*trigger, time);
        }
        active = triggers;
        for case in &mut cases {
            if !case.complete_post_window
                && !closed.contains(&case.id)
                && time >= case.trigger_sim_time_us
            {
                case.end_frame = id;
                case.end_sim_time_us = time;
                if time >= case.trigger_sim_time_us + post {
                    case.complete_post_window = true;
                }
            }
        }
        Ok(())
    })?;
    ensure!(
        source_hashes(run)? == hashes,
        "source recording changed during extraction"
    );
    Ok(ScenarioLibrary {schema_version:1,source:integrity.source,source_hashes:hashes,map_sha256:graph_hash(graph)?,settings,cases,phase_acceptance:false,notes:vec!["Red-light approach is an automatically selected review candidate; nearby red signals do not prove a violation.".into(),"Takeover is a /stack request_control true→false edge; absent /stack means unknown, not zero human interventions.".into(),"Roundabout classification comes from the supplied map scope. Empty scope means that category was not evaluated.".into()]})
}
pub(super) fn nearest(
    graph: &LaneGraph,
    pose: etsads_msgs::Pose2,
    z: f64,
) -> Option<etsads_map::Projection> {
    graph
        .nearest_lanes(
            Point3 {
                x: pose.x,
                y: pose.y,
                z,
            },
            8.0,
        )
        .into_iter()
        .filter(|p| (p.z - z).abs() < 3.0 && wrap_angle(p.heading - pose.yaw).abs() < 0.8)
        .min_by(|a, b| {
            a.distance
                .total_cmp(&b.distance)
                .then(a.lane_id.cmp(&b.lane_id))
        })
}
pub(super) struct Seed {
    pub snapshot: GameSnapshot,
    pub signals: Vec<(u64, Vec<u8>)>,
    pub source: String,
}
pub(super) fn seed(run: &Path, case: &RecordedCase) -> anyhow::Result<Seed> {
    ensure!(
        case.end_sim_time_us > case.start_sim_time_us
            && case.end_sim_time_us - case.start_sim_time_us <= 61_000_000,
        "invalid or oversized case window"
    );
    let mut snapshot = None;
    let mut signals = vec![];
    let mut last_signal = None;
    let mut found_end = false;
    let summary = ReplayAdapter::run(run, |frame| {
        let s = frame.snapshot;
        let id = s.ego.header.frame_id;
        let time = s.ego.header.sim_time_us;
        if id == case.start_frame {
            ensure!(time == case.start_sim_time_us, "case start time mismatch");
            snapshot = Some(s.clone());
        }
        if id >= case.start_frame && id <= case.end_frame {
            if s.ego.paused {
                anyhow::bail!("case crosses pause; extract separate continuous segment")
            }
            if last_signal.is_none_or(|t| time >= t + 100_000) {
                // Only exogenous light observations are replayed. Paid/fine/job
                // events and recorded traffic positions must not follow ego.
                let signal = s
                    .extension
                    .as_ref()
                    .filter(|e| e.available & 8 != 0)
                    .map(|e| e.signals.to_vec())
                    .unwrap_or_default();
                signals.push((time, signal));
                last_signal = Some(time);
            }
        }
        if id == case.end_frame {
            ensure!(time == case.end_sim_time_us, "case end time mismatch");
            found_end = true;
        }
        ensure!(signals.len() <= 611, "signal sample budget exceeded");
        Ok(())
    })?;
    ensure!(found_end, "recording does not contain case end");
    Ok(Seed {
        snapshot: snapshot.context("recording does not contain case start")?,
        signals,
        source: summary.integrity.source,
    })
}
