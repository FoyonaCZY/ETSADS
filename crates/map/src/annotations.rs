//! Portable, read-only regional traffic metadata. Cross-window conflict
//! partners remain references, never silently becoming an empty intersection.
use crate::{
    importer::ExportedMap,
    traffic::{SemaphoreKind, TrafficContext, TrafficError},
};
use etsads_msgs::Point3;
use flate2::read::MultiGzDecoder;
use parquet::{
    file::reader::{FileReader, SerializedFileReader},
    record::{Field, Row},
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, HashSet},
    fs::File,
    io::Read,
    path::{Component, Path, PathBuf},
    sync::{Arc, Mutex},
};
fn bad(s: impl Into<String>) -> TrafficError {
    TrafficError::Invalid(s.into())
}
const TABLES: [&str; 5] = [
    "conflict_pairs",
    "lane_controls",
    "signals",
    "signal_lanes",
    "stop_lines",
];
#[derive(Clone, Debug)]
pub struct AnnotationLimits {
    pub max_cache_bytes: usize,
    pub max_shards: usize,
    pub max_uncompressed_shard_bytes: usize,
    pub max_region_records: usize,
}
impl Default for AnnotationLimits {
    fn default() -> Self {
        Self {
            max_cache_bytes: 64 * 1024 * 1024,
            max_shards: 16,
            max_uncompressed_shard_bytes: 32 * 1024 * 1024,
            max_region_records: 250_000,
        }
    }
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AnnotationStats {
    pub candidate_shards: usize,
    pub shards_opened: usize,
    pub cache_hits: usize,
    pub compressed_bytes_read: u64,
    pub cache_bytes: usize,
    pub cache_evictions: u64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GateCandidate {
    #[serde(default)]
    pub owner_uid: u64,
    #[serde(default)]
    pub semaphore_id: u32,
    pub native_signal_group_id: String,
    pub source_lane_id: String,
    pub kind: SemaphoreKind,
    pub position: Point3,
    pub stop_target: Option<Point3>,
    pub stop_geometry_status: Option<String>,
    /// A barrier is not proof of a toll charge, activation protocol, or a
    /// verified stop location. Runtime keeps its existing verified toll config.
    pub toll_classification_verified: bool,
    pub activation_verified: bool,
    #[serde(default)]
    pub semantics_resolved: bool,
    #[serde(default)]
    pub profile: String,
    #[serde(default)]
    pub profile_resolution: Option<String>,
    #[serde(default)]
    pub profile_evidence_id: Option<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionAnnotations {
    pub traffic: TrafficContext,
    pub gate_candidates: Vec<GateCandidate>,
    pub source_id: String,
    pub missing_sources: Vec<String>,
    pub stats: AnnotationStats,
}
#[derive(Clone, Debug, Deserialize)]
pub(crate) struct CompressedFile {
    pub path: String,
    pub bytes: u64,
    pub uncompressed_bytes: u64,
    pub sha256: String,
}
#[derive(Clone, Deserialize)]
struct Shard {
    #[serde(flatten)]
    file: CompressedFile,
    bounds: [f64; 4],
}
#[derive(Deserialize)]
struct ProfileEvidence {
    evidence_path: String,
    evidence_sha256: String,
    evidence_bytes: u64,
    resolved_signals_path: String,
    resolved_signals_sha256: String,
    resolved_signals_bytes: u64,
}
#[derive(Deserialize)]
pub(crate) struct AnnotationIndex {
    schema_version: u32,
    format: String,
    coordinate_system: String,
    units: String,
    pub source_manifest_sha256: String,
    pub lane_index_sha256: String,
    pub source_cache_key: String,
    present_tables: Vec<String>,
    missing_tables: Vec<String>,
    shards: Vec<Shard>,
    pub native_route_index: CompressedFile,
    #[serde(default)]
    signal_profile_resolution: Option<ProfileEvidence>,
}
pub(crate) fn read_bounded(path: &Path, limit: u64) -> Result<Vec<u8>, TrafficError> {
    let mut bytes = Vec::new();
    File::open(path)?
        .take(limit.saturating_add(1))
        .read_to_end(&mut bytes)?;
    if bytes.len() as u64 > limit {
        return Err(bad("metadata byte budget exceeded"));
    }
    Ok(bytes)
}
pub(crate) fn read_index(root: &Path) -> Result<AnnotationIndex, TrafficError> {
    let bytes = read_bounded(&root.join("traffic_index.json"), 16 * 1024 * 1024)?;
    let index: AnnotationIndex = serde_json::from_slice(&bytes)?;
    if index.schema_version != 1
        || index.format != "etsads_sector_traffic_json_gzip_v1"
        || index.coordinate_system != "ENU"
        || index.units != "metres"
        || index.source_cache_key.is_empty()
        || index.shards.len() > 100_000
    {
        return Err(bad("unsupported annotation index"));
    }
    let lane_bytes = read_bounded(&root.join("sector_index.json"), 16 * 1024 * 1024)?;
    if format!("{:x}", Sha256::digest(&lane_bytes)) != index.lane_index_sha256 {
        return Err(bad("annotation/lane index identity mismatch"));
    }
    let lane_index: Value = serde_json::from_slice(&lane_bytes)?;
    if lane_index["source_manifest_sha256"].as_str() != Some(&index.source_manifest_sha256)
        || lane_index["source_cache_key"].as_str() != Some(&index.source_cache_key)
    {
        return Err(bad("annotation source manifest mismatch"));
    }
    let mut paths = HashSet::new();
    for shard in &index.shards {
        if !valid_bounds(shard.bounds) || !paths.insert(&shard.file.path) {
            return Err(bad("invalid/duplicate annotation shard"));
        }
        valid_descriptor(&shard.file)?;
    }
    valid_descriptor(&index.native_route_index)?;
    if let Some(proof) = &index.signal_profile_resolution {
        for (path, expected, bytes) in [
            (
                &proof.evidence_path,
                &proof.evidence_sha256,
                proof.evidence_bytes,
            ),
            (
                &proof.resolved_signals_path,
                &proof.resolved_signals_sha256,
                proof.resolved_signals_bytes,
            ),
        ] {
            valid_descriptor(&CompressedFile {
                path: path.clone(),
                sha256: expected.clone(),
                bytes,
                uncompressed_bytes: bytes,
            })?;
            if bytes > 128 * 1024 * 1024 {
                return Err(bad("profile evidence byte budget exceeded"));
            }
            let full = root.join(path).canonicalize()?;
            if !full.starts_with(root) {
                return Err(bad("profile evidence escapes bundle root"));
            }
            let content = read_bounded(&full, bytes)?;
            if content.len() as u64 != bytes
                || format!("{:x}", Sha256::digest(&content)) != expected.to_ascii_lowercase()
            {
                return Err(bad("profile evidence checksum or length mismatch"));
            }
        }
    }
    Ok(index)
}
fn valid_descriptor(meta: &CompressedFile) -> Result<(), TrafficError> {
    if meta.path.is_empty()
        || Path::new(&meta.path)
            .components()
            .any(|p| !matches!(p, Component::Normal(_)))
        || meta.bytes == 0
        || meta.uncompressed_bytes == 0
        || meta.sha256.len() != 64
        || !meta.sha256.bytes().all(|c| c.is_ascii_hexdigit())
    {
        return Err(bad("invalid compressed metadata descriptor"));
    }
    Ok(())
}
pub(crate) fn checked_compressed(
    root: &Path,
    meta: &CompressedFile,
    limit: u64,
) -> Result<Vec<u8>, TrafficError> {
    valid_descriptor(meta)?;
    if meta.uncompressed_bytes > limit || meta.bytes > limit.saturating_add(65536) {
        return Err(bad("metadata shard exceeds byte budget"));
    }
    let path = root.join(&meta.path).canonicalize()?;
    if !path.starts_with(root) {
        return Err(bad("metadata shard escapes map root"));
    }
    let compressed = read_bounded(&path, meta.bytes)?;
    if compressed.len() as u64 != meta.bytes
        || format!("{:x}", Sha256::digest(&compressed)) != meta.sha256.to_ascii_lowercase()
    {
        return Err(bad("metadata checksum/length mismatch"));
    }
    let mut bytes = Vec::new();
    MultiGzDecoder::new(compressed.as_slice())
        .take(meta.uncompressed_bytes + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() as u64 != meta.uncompressed_bytes {
        return Err(bad("metadata decompressed length mismatch"));
    }
    Ok(bytes)
}
enum Source {
    Partition(Box<AnnotationIndex>),
    Slice,
    Parquet,
    Missing,
}
#[derive(Default)]
struct Cache {
    entries: BTreeMap<usize, (Arc<Vec<u8>>, u64)>,
    bytes: usize,
    tick: u64,
    evictions: u64,
}
pub struct AnnotationStore {
    root: PathBuf,
    source: Source,
    source_id: String,
    limits: AnnotationLimits,
    cache: Mutex<Cache>,
}
impl AnnotationStore {
    /// Prefer this API for moving windows: metadata must be tied to the same
    /// loaded lane source before any annotation can be used by the runtime.
    pub fn load_for_region(
        &self,
        center: Point3,
        radius: f64,
        region: &crate::sector::RegionLoad,
    ) -> Result<RegionAnnotations, TrafficError> {
        let expected = match &self.source {
            Source::Partition(index) => Some(index.lane_index_sha256.clone()),
            Source::Slice => Some(format!("json:{}", self.root.display())),
            Source::Parquet => Some(format!("legacy:{}", self.root.display())),
            Source::Missing => None,
        };
        if expected.is_some_and(|id| id != region.source_id) {
            return Err(bad("traffic and lane region source identity mismatch"));
        }
        self.load_region(center, radius, &region.map)
    }
    pub fn open(path: impl AsRef<Path>, limits: AnnotationLimits) -> Result<Self, TrafficError> {
        if limits.max_cache_bytes == 0
            || limits.max_shards == 0
            || limits.max_uncompressed_shard_bytes == 0
            || limits.max_uncompressed_shard_bytes > limits.max_cache_bytes
            || limits.max_region_records == 0
        {
            return Err(bad("invalid annotation cache limits"));
        }
        let mut root = path.as_ref().canonicalize()?;
        let (source, source_id) = if root.is_dir() && root.join("traffic_index.json").is_file() {
            let index = read_index(&root)?;
            let id = format!(
                "{:x}",
                Sha256::digest(read_bounded(
                    &root.join("traffic_index.json"),
                    16 * 1024 * 1024
                )?)
            );
            (Source::Partition(Box::new(index)), id)
        } else if root.is_file() || root.join("runtime-map.json").is_file() {
            if root.is_dir() {
                root = root.join("runtime-map.json").canonicalize()?;
            }
            let id = format!("slice:{}", root.display());
            (Source::Slice, id)
        } else if root.join("lanes.parquet").is_file() {
            let manifest = read_bounded(&root.join("manifest.json"), 16 * 1024 * 1024)?;
            let value: Value = serde_json::from_slice(&manifest)?;
            if value["schema_version"] != 1
                || value["coordinate_system"] != "ENU"
                || value["units"] != "metres"
            {
                return Err(bad("invalid legacy metadata manifest"));
            }
            (Source::Parquet, format!("{:x}", Sha256::digest(&manifest)))
        } else {
            (Source::Missing, format!("missing:{}", root.display()))
        };
        Ok(Self {
            root,
            source,
            source_id,
            limits,
            cache: Mutex::new(Cache::default()),
        })
    }
    pub fn load_region(
        &self,
        center: Point3,
        radius: f64,
        map: &ExportedMap,
    ) -> Result<RegionAnnotations, TrafficError> {
        if !center.valid() || !radius.is_finite() || radius <= 0.0 || radius > 5000.0 {
            return Err(bad("invalid annotation region"));
        }
        let ids: HashSet<_> = map.lanes.iter().map(|l| l.id.clone()).collect();
        if ids.len() != map.lanes.len() {
            return Err(bad("duplicate regional lane identity"));
        }
        let bounds = [
            center.x - radius,
            center.y - radius,
            center.x + radius,
            center.y + radius,
        ];
        let mut stats = AnnotationStats::default();
        let mut missing = Vec::new();
        let mut contexts = Vec::new();
        match &self.source {
            Source::Partition(index) => {
                missing = index.missing_tables.clone();
                for name in TABLES {
                    if !index.present_tables.iter().any(|s| s == name)
                        && !missing.iter().any(|s| s == name)
                    {
                        missing.push(name.into());
                    }
                }
                for (i, shard) in index
                    .shards
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| intersects(s.bounds, bounds))
                {
                    stats.candidate_shards += 1;
                    let bytes = self.shard(i, &shard.file, &mut stats)?;
                    contexts.push(TrafficContext::from_json(
                        std::str::from_utf8(&bytes).map_err(|e| bad(e.to_string()))?,
                    )?);
                    if contexts.iter().map(context_size).sum::<usize>()
                        > self.limits.max_region_records
                    {
                        return Err(bad("loaded annotation shard record budget exceeded"));
                    }
                }
                // A map with no matching metadata shards is not proof that
                // every requested lane lacks controls.
                if contexts.is_empty() && !ids.is_empty() {
                    missing.push("no_metadata_coverage".into());
                }
            }
            Source::Slice => {
                let bytes = read_bounded(&self.root, 256 * 1024 * 1024)?;
                stats.compressed_bytes_read = bytes.len() as u64;
                contexts.push(TrafficContext::from_json(
                    std::str::from_utf8(&bytes).map_err(|e| bad(e.to_string()))?,
                )?);
            }
            Source::Parquet => {
                let (context, absent) =
                    load_parquet(&self.root, &ids, self.limits.max_region_records)?;
                contexts.push(context);
                missing = absent;
            }
            Source::Missing => missing.push("traffic_index_or_source_tables".into()),
        }
        let mut traffic = merge(contexts, &ids, self.limits.max_region_records)?;
        traffic.annotations_present &= missing.is_empty();
        let gates = gate_candidates(&traffic);
        let cache = self.cache.lock().unwrap_or_else(|p| p.into_inner());
        stats.cache_bytes = cache.bytes;
        stats.cache_evictions = cache.evictions;
        Ok(RegionAnnotations {
            traffic,
            gate_candidates: gates,
            source_id: self.source_id.clone(),
            missing_sources: missing,
            stats,
        })
    }
    fn shard(
        &self,
        id: usize,
        meta: &CompressedFile,
        stats: &mut AnnotationStats,
    ) -> Result<Arc<Vec<u8>>, TrafficError> {
        // Serialize miss/decode/insert. Cached bytes are exact Vec capacities;
        // parsed regional objects are caller-owned and have a record budget.
        let mut cache = self.cache.lock().unwrap_or_else(|p| p.into_inner());
        cache.tick += 1;
        let tick = cache.tick;
        if let Some((bytes, used)) = cache.entries.get_mut(&id) {
            *used = tick;
            stats.cache_hits += 1;
            return Ok(bytes.clone());
        }
        while !cache.entries.is_empty()
            && (cache.entries.len() >= self.limits.max_shards
                || cache.bytes + self.limits.max_uncompressed_shard_bytes
                    > self.limits.max_cache_bytes)
        {
            evict(&mut cache);
        }
        let bytes = checked_compressed(
            &self.root,
            meta,
            self.limits.max_uncompressed_shard_bytes as u64,
        )?;
        let bytes = Arc::new(bytes.into_boxed_slice().into_vec());
        if bytes.capacity() > self.limits.max_uncompressed_shard_bytes {
            return Err(bad("metadata cache allocation budget"));
        }
        cache.bytes += bytes.capacity();
        cache.entries.insert(id, (bytes.clone(), tick));
        stats.shards_opened += 1;
        stats.compressed_bytes_read += meta.bytes;
        Ok(bytes)
    }
}
fn context_size(context: &TrafficContext) -> usize {
    context.source_lane_ids.len()
        + context.reference_only_lane_ids.len()
        + context.conflicts.len()
        + context.lane_controls.len()
        + context.signals.len()
        + context.signal_lanes.len()
        + context.stop_lines.len()
}
fn evict(cache: &mut Cache) {
    if let Some(id) = cache
        .entries
        .iter()
        .min_by_key(|(_, (_, t))| *t)
        .map(|(i, _)| *i)
    {
        cache.bytes -= cache.entries.remove(&id).unwrap().0.capacity();
        cache.evictions += 1;
    }
}
fn valid_bounds(b: [f64; 4]) -> bool {
    b.iter().all(|x| x.is_finite() && x.abs() < 1e8) && b[0] <= b[2] && b[1] <= b[3]
}
fn intersects(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1]
}
fn merge(
    contexts: Vec<TrafficContext>,
    ids: &HashSet<String>,
    limit: usize,
) -> Result<TrafficContext, TrafficError> {
    let mut complete = !contexts.is_empty();
    let mut covered = HashSet::new();
    let mut arrays: [BTreeMap<String, Value>; 5] = std::array::from_fn(|_| BTreeMap::new());
    for context in contexts {
        complete &= context.annotations_present;
        covered.extend(
            context
                .source_lane_ids
                .iter()
                .filter(|s| !context.reference_only_lane_ids.contains(s))
                .cloned(),
        );
        let value = serde_json::to_value(context)?;
        for (index, name) in [
            "conflicts",
            "lane_controls",
            "signals",
            "signal_lanes",
            "stop_lines",
        ]
        .into_iter()
        .enumerate()
        {
            for row in value[name].as_array().unwrap() {
                let keep = match index {
                    0 => {
                        ids.contains(row["lane_a"].as_str().unwrap())
                            || ids.contains(row["lane_b"].as_str().unwrap())
                    }
                    2 => true,
                    _ => ids.contains(row["lane_id"].as_str().unwrap()),
                };
                if !keep {
                    continue;
                }
                let key = match index {
                    0 | 4 => row["id"].as_str().unwrap().to_string(),
                    1 => row["lane_id"].as_str().unwrap().to_string(),
                    _ => serde_json::to_string(row)?,
                };
                if let Some(old) = arrays[index].insert(key, row.clone())
                    && old != *row
                {
                    return Err(bad("conflicting duplicate metadata row"));
                }
                if arrays.iter().map(BTreeMap::len).sum::<usize>() > limit {
                    return Err(bad("regional annotation record budget exceeded"));
                }
            }
        }
    }
    complete &= ids.iter().all(|id| covered.contains(id));
    let signal_ids: HashSet<_> = arrays[3]
        .values()
        .filter_map(|r| r["signal_id"].as_str())
        .chain(
            arrays[1]
                .values()
                .filter_map(|r| r["signal_group_id"].as_str()),
        )
        .chain(arrays[4].values().filter_map(|r| r["signal_id"].as_str()))
        .map(str::to_string)
        .collect();
    arrays[2].retain(|_, v| signal_ids.contains(v["id"].as_str().unwrap()));
    let references: HashSet<_> = arrays[0]
        .values()
        .flat_map(|p| {
            [
                p["lane_a"].as_str().unwrap().to_string(),
                p["lane_b"].as_str().unwrap().to_string(),
            ]
        })
        .filter(|id| !ids.contains(id))
        .collect();
    let mut all_ids: Vec<_> = ids.iter().chain(&references).cloned().collect();
    all_ids.sort();
    let mut references: Vec<_> = references.into_iter().collect();
    references.sort();
    let document = serde_json::json!({"schema_version":1,"source_lane_ids":all_ids,"reference_only_lane_ids":references,"annotations_present":complete,
        "conflicts":arrays[0].values().collect::<Vec<_>>(),"lane_controls":arrays[1].values().collect::<Vec<_>>(),"signals":arrays[2].values().collect::<Vec<_>>(),
        "signal_lanes":arrays[3].values().collect::<Vec<_>>(),"stop_lines":arrays[4].values().collect::<Vec<_>>()});
    TrafficContext::from_json(&document.to_string())
}
fn gate_candidates(traffic: &TrafficContext) -> Vec<GateCandidate> {
    let mut output = Vec::new();
    for link in &traffic.signal_lanes {
        for signal in traffic.signals.iter().filter(|s| {
            s.id == link.signal_id
                && matches!(
                    s.kind,
                    SemaphoreKind::BarrierManualTimed
                        | SemaphoreKind::BarrierDistanceActivated
                        | SemaphoreKind::BarrierAutomatic
                        | SemaphoreKind::BarrierGas
                )
        }) {
            let stop = traffic
                .stop_lines
                .iter()
                .find(|s| s.lane_id == link.lane_id && s.signal_id.as_ref() == Some(&signal.id));
            output.push(GateCandidate {
                owner_uid: signal.owner_uid,
                semaphore_id: signal.semaphore_id,
                native_signal_group_id: signal.id.clone(),
                source_lane_id: link.lane_id.clone(),
                kind: signal.kind,
                position: signal.position(),
                stop_target: stop.map(|s| s.position()),
                stop_geometry_status: stop.map(|s| s.geometry_status.clone()),
                toll_classification_verified: false,
                activation_verified: false,
                semantics_resolved: true,
                profile: signal
                    .effective_profile
                    .as_ref()
                    .unwrap_or(&signal.profile)
                    .clone(),
                profile_resolution: signal.profile_resolution.clone(),
                profile_evidence_id: signal.profile_evidence_id.clone(),
            });
        }
    }
    output
}
fn row_json(row: Row) -> Result<Value, TrafficError> {
    let mut out = serde_json::Map::new();
    for (name, field) in row.into_columns() {
        let value = match field {
            Field::Null => Value::Null,
            Field::Bool(v) => v.into(),
            Field::Int(v) => v.into(),
            Field::Long(v) => v.into(),
            Field::UInt(v) => v.into(),
            Field::ULong(v) => v.into(),
            Field::Str(v) => v.into(),
            // Match the exporter's shortest round-trip decimal serialization of
            // original f32 metadata, rather than silently widening binary noise.
            Field::Float(v) => serde_json::from_str(&v.to_string())?,
            Field::Double(v) => serde_json::Number::from_f64(v)
                .map(Value::Number)
                .ok_or_else(|| bad("nonfinite metadata"))?,
            _ => return Err(bad("unsupported metadata Parquet field")),
        };
        out.insert(name, value);
    }
    Ok(Value::Object(out))
}
fn load_parquet(
    root: &Path,
    ids: &HashSet<String>,
    limit: usize,
) -> Result<(TrafficContext, Vec<String>), TrafficError> {
    let mut arrays = serde_json::Map::new();
    let mut missing = Vec::new();
    let mut signal_ids = HashSet::new();
    let mut references = HashSet::new();
    let mut count = 0;
    for name in [
        "conflict_pairs",
        "lane_controls",
        "signal_lanes",
        "stop_lines",
        "signals",
    ] {
        let path = root.join(format!("{name}.parquet"));
        let key = if name == "conflict_pairs" {
            "conflicts"
        } else {
            name
        };
        let mut values = Vec::new();
        if path.is_file() {
            let reader =
                SerializedFileReader::new(File::open(path)?).map_err(|e| bad(e.to_string()))?;
            for row in reader.get_row_iter(None).map_err(|e| bad(e.to_string()))? {
                let row = row_json(row.map_err(|e| bad(e.to_string()))?)?;
                let keep = match name {
                    "conflict_pairs" => {
                        ids.contains(
                            row["lane_a"]
                                .as_str()
                                .ok_or_else(|| bad("missing conflict lane"))?,
                        ) || ids.contains(
                            row["lane_b"]
                                .as_str()
                                .ok_or_else(|| bad("missing conflict lane"))?,
                        )
                    }
                    "signals" => signal_ids
                        .contains(row["id"].as_str().ok_or_else(|| bad("missing signal id"))?),
                    _ => ids.contains(
                        row["lane_id"]
                            .as_str()
                            .ok_or_else(|| bad("missing control lane"))?,
                    ),
                };
                if keep {
                    if name == "conflict_pairs" {
                        for k in ["lane_a", "lane_b"] {
                            let id = row[k].as_str().unwrap().to_string();
                            if !ids.contains(&id) {
                                references.insert(id);
                            }
                        }
                    }
                    for key in ["signal_id", "signal_group_id"] {
                        if let Some(id) = row[key].as_str() {
                            signal_ids.insert(id.to_string());
                        }
                    }
                    values.push(row);
                    count += 1;
                    if count > limit {
                        return Err(bad("regional metadata record budget exceeded"));
                    }
                }
            }
        } else {
            missing.push(name.into());
        }
        arrays.insert(key.into(), Value::Array(values));
    }
    arrays.insert("schema_version".into(), 1.into());
    arrays.insert("annotations_present".into(), missing.is_empty().into());
    arrays.insert(
        "source_lane_ids".into(),
        serde_json::to_value(ids.iter().chain(&references).collect::<Vec<_>>())?,
    );
    arrays.insert(
        "reference_only_lane_ids".into(),
        serde_json::to_value(references)?,
    );
    Ok((
        TrafficContext::from_json(&Value::Object(arrays).to_string())?,
        missing,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn context() -> TrafficContext {
        TrafficContext::from_json(&serde_json::json!({"schema_version":1,"source_lane_ids":["a","b"],"reference_only_lane_ids":["b"],"annotations_present":true,
            "conflicts":[{"id":"cross","owner_uid":u64::MAX,"lane_a":"a","lane_b":"b","kind":"cross","source":"derived_geometry","x":20.0,"y":0.0,"z":0.0,"radius_m":4.0,"geometry_gap_m":0.0,"priority_a":null,"priority_b":null,"relation":"unknown","conservative_wait":true}],
            "lane_controls":[],"signals":[{"id":"gate","owner_uid":u64::MAX,"semaphore_id":0,"type":"BarrierManualTimed","x":10.0,"y":0.0,"z":0.0,"yaw":0.0,"profile":"unknown","cycle_delay_s":0.0,"interval_0":0.0,"interval_1":0.0,"interval_2":0.0,"interval_3":0.0}],
            "signal_lanes":[{"signal_id":"gate","lane_id":"a"}],"stop_lines":[{"id":"stop","lane_id":"a","signal_id":"gate","rule":"unknown","x":8.0,"y":0.0,"z":0.0,"geometry_status":"curve_start_proxy","control_kind":"unknown"}]}).to_string()).unwrap()
    }
    #[test]
    fn cross_window_conflicts_survive_without_granting_external_annotation_coverage() {
        let context = context();
        let merged = merge(
            vec![context.clone(), context],
            &HashSet::from(["a".into()]),
            100,
        )
        .unwrap();
        assert_eq!(merged.conflicts.len(), 1);
        assert_eq!(merged.reference_only_lane_ids, vec!["b"]);
        assert!(merged.has_annotations_for_route(&["a".into()]));
        assert!(!merged.has_annotations_for_route(&["b".into()]));
        assert!(
            merged
                .upcoming_stop_controls(&["a".into()])
                .iter()
                .any(|c| c.conflict_lane_ids == ["b"] && c.requires_conservative_wait)
        );
        let gates = gate_candidates(&merged);
        assert_eq!(gates.len(), 1);
        assert!(!gates[0].toll_classification_verified);
        assert!(!gates[0].activation_verified);
        assert_eq!(
            gates[0].stop_geometry_status.as_deref(),
            Some("curve_start_proxy")
        );
    }
    #[test]
    fn missing_coverage_and_conflicting_duplicate_rows_fail_conservatively() {
        let c = context();
        let missing = merge(
            vec![c.clone()],
            &HashSet::from(["a".into(), "b".into()]),
            100,
        )
        .unwrap();
        assert!(!missing.annotations_present);
        let mut altered = c.clone();
        altered.conflicts[0].radius_m = 8.0;
        assert!(merge(vec![c, altered], &HashSet::from(["a".into()]), 100).is_err());
    }
    #[test]
    fn metadata_checksums_and_paths_are_enforced() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("testdata/navigation-sectors")
            .canonicalize()
            .unwrap();
        let index = read_index(&root).unwrap();
        let mut file = index.shards[0].file.clone();
        file.sha256 = "0".repeat(64);
        assert!(checked_compressed(&root, &file, 1024 * 1024).is_err());
        file.path = "../outside.gz".into();
        assert!(checked_compressed(&root, &file, 1024 * 1024).is_err());
        assert!(checked_compressed(&root, &index.shards[0].file, 8).is_err());
    }
}
