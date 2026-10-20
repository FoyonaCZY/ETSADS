//! Authenticate portable partition contents once before a live experiment.
//! Runtime loaders still validate each shard when it enters a map window.
use super::hash_bytes;
use anyhow::{Context, ensure};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs::File,
    io::Read,
    path::{Component, Path, PathBuf},
};

const MAX_INDEX: u64 = 16 * 1024 * 1024;
const MAX_TOTAL: u64 = 64 * 1024 * 1024 * 1024;

#[derive(Debug)]
pub(super) struct PartitionIdentity {
    pub data_hash: String,
    pub source_id: String,
    pub annotations_source_id: Option<String>,
}
#[derive(Deserialize)]
struct Shard {
    path: String,
    bytes: u64,
    sha256: String,
}
#[derive(Deserialize)]
struct ProfileResolution {
    evidence_path: String,
    evidence_sha256: String,
    evidence_bytes: u64,
    resolved_signals_path: String,
    resolved_signals_sha256: String,
    resolved_signals_bytes: u64,
}
impl ProfileResolution {
    fn references(&self) -> [Shard; 2] {
        [
            Shard {
                path: self.evidence_path.clone(),
                bytes: self.evidence_bytes,
                sha256: self.evidence_sha256.clone(),
            },
            Shard {
                path: self.resolved_signals_path.clone(),
                bytes: self.resolved_signals_bytes,
                sha256: self.resolved_signals_sha256.clone(),
            },
        ]
    }
}
#[derive(Deserialize)]
struct Index {
    schema_version: u32,
    format: String,
    coordinate_system: String,
    units: String,
    source_cache_key: String,
    source_manifest_sha256: String,
    compressed_bytes: u64,
    shards: Vec<Shard>,
    #[serde(default)]
    lane_index_sha256: Option<String>,
    #[serde(default)]
    native_route_index: Option<Shard>,
    #[serde(default)]
    signal_profile_resolution: Option<ProfileResolution>,
}
fn read_json(path: &Path) -> anyhow::Result<(Vec<u8>, serde_json::Value)> {
    let mut file =
        File::open(path).with_context(|| format!("open map metadata {}", path.display()))?;
    let mut bytes = Vec::new();
    (&mut file).take(MAX_INDEX + 1).read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() as u64 <= MAX_INDEX,
        "map metadata exceeds 16 MiB"
    );
    let value = serde_json::from_slice(&bytes)?;
    Ok((bytes, value))
}
fn contained(root: &Path, relative: &str) -> anyhow::Result<PathBuf> {
    ensure!(
        !relative.is_empty()
            && Path::new(relative)
                .components()
                .all(|c| matches!(c, Component::Normal(_))),
        "map reference must be a relative path without traversal: {relative}"
    );
    let path = root
        .join(relative)
        .canonicalize()
        .with_context(|| format!("referenced map file missing: {relative}"))?;
    ensure!(
        path.starts_with(root) && path.is_file(),
        "map reference escapes its root: {relative}"
    );
    Ok(path)
}
fn actual_file_hash(path: &Path, expected_bytes: u64) -> anyhow::Result<[u8; 32]> {
    let mut options = std::fs::OpenOptions::new();
    options.read(true);
    #[cfg(windows)]
    {
        use std::os::windows::fs::OpenOptionsExt;
        options.share_mode(1);
    }
    let mut file = options.open(path)?;
    let meta = file.metadata()?;
    ensure!(
        meta.is_file() && meta.len() == expected_bytes,
        "map file size changed: {}",
        path.display()
    );
    let mut hash = Sha256::new();
    let mut buffer = [0u8; 64 * 1024];
    let mut total = 0u64;
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        total = total
            .checked_add(n as u64)
            .context("map byte count overflow")?;
        ensure!(total <= expected_bytes, "map file grew while reading");
        hash.update(&buffer[..n]);
    }
    ensure!(total == expected_bytes, "map file truncated while reading");
    Ok(hash.finalize().into())
}
fn validate(index: &Index, expected_format: &str) -> anyhow::Result<()> {
    ensure!(
        index.schema_version == 1
            && index.format == expected_format
            && index.coordinate_system == "ENU"
            && index.units == "metres"
            && !index.source_cache_key.is_empty()
            && !index.shards.is_empty()
            && index.shards.len() <= 100_000,
        "unsupported/incomplete partition index"
    );
    hash_bytes(&index.source_manifest_sha256)?;
    let sum = index.shards.iter().try_fold(0u64, |sum, s| {
        sum.checked_add(s.bytes).context("partition size overflow")
    })?;
    ensure!(
        sum == index.compressed_bytes && sum <= MAX_TOTAL,
        "partition totals or byte budget invalid"
    );
    Ok(())
}

pub(super) fn verify_partition(root: &Path) -> anyhow::Result<PartitionIdentity> {
    let root = root.canonicalize()?;
    ensure!(
        root.is_dir(),
        "native navigation requires a partition directory"
    );
    let mut files = BTreeMap::<String, [u8; 32]>::new();
    let mut paths = std::collections::BTreeSet::new();
    let mut total = 0u64;
    let mut metadata = |name: &str| -> anyhow::Result<(Vec<u8>, serde_json::Value)> {
        let path = contained(&root, name)?;
        ensure!(paths.insert(path.clone()), "duplicate map file reference");
        let (bytes, value) = read_json(&path)?;
        files.insert(name.to_owned(), Sha256::digest(&bytes).into());
        Ok((bytes, value))
    };
    let (_, manifest) = metadata("manifest.json")?;
    let (lane_bytes, lane_value) = metadata("sector_index.json")?;
    let lanes: Index = serde_json::from_value(lane_value)?;
    validate(&lanes, "etsads_sector_lanes_jsonl_gzip_v1")?;
    let manifest_key = manifest["source_cache_key"]
        .as_str()
        .or_else(|| manifest["cache_key"].as_str());
    ensure!(
        manifest_key == Some(lanes.source_cache_key.as_str())
            && manifest["source_manifest_sha256"].as_str()
                == Some(lanes.source_manifest_sha256.as_str()),
        "partition manifest and lane index provenance disagree"
    );
    let source_id = format!("{:x}", Sha256::digest(&lane_bytes));
    let traffic = if root.join("traffic_index.json").exists() {
        let (bytes, value) = metadata("traffic_index.json")?;
        let index: Index = serde_json::from_value(value)?;
        validate(&index, "etsads_sector_traffic_json_gzip_v1")?;
        ensure!(
            index.source_cache_key == lanes.source_cache_key
                && hash_bytes(&index.source_manifest_sha256)?
                    == hash_bytes(&lanes.source_manifest_sha256)?
                && index
                    .lane_index_sha256
                    .as_deref()
                    .map(hash_bytes)
                    .transpose()?
                    == Some(hash_bytes(&source_id)?),
            "traffic index is not bound to this lane partition source"
        );
        ensure!(
            index.native_route_index.is_some(),
            "traffic partition lacks global native route index"
        );
        Some((index, format!("{:x}", Sha256::digest(&bytes))))
    } else {
        None
    };
    let profiles = traffic
        .as_ref()
        .and_then(|(index, _)| index.signal_profile_resolution.as_ref())
        .map(ProfileResolution::references);
    let references = lanes
        .shards
        .iter()
        .chain(
            traffic
                .as_ref()
                .into_iter()
                .flat_map(|(index, _)| index.shards.iter().chain(index.native_route_index.iter())),
        )
        .chain(profiles.iter().flatten());
    for shard in references {
        ensure!(shard.bytes > 0, "zero-length map shard");
        total = total
            .checked_add(shard.bytes)
            .context("partition byte count overflow")?;
        ensure!(
            total <= MAX_TOTAL,
            "partition exceeds 64 GiB validation budget"
        );
        let path = contained(&root, &shard.path)?;
        ensure!(
            paths.insert(path.clone()),
            "duplicate/aliased map file reference: {}",
            shard.path
        );
        let expected = hash_bytes(&shard.sha256)?;
        let actual = actual_file_hash(&path, shard.bytes)?;
        ensure!(
            actual == expected,
            "map shard content changed: {}",
            shard.path
        );
        files.insert(shard.path.clone(), actual);
    }
    let mut hash = Sha256::new();
    hash.update(b"ETSADS_PARTITION_CONTENT_V1\0");
    for (name, digest) in files {
        hash.update((name.len() as u64).to_le_bytes());
        hash.update(name.as_bytes());
        hash.update(digest);
    }
    Ok(PartitionIdentity {
        data_hash: format!("{:x}", hash.finalize()),
        source_id,
        annotations_source_id: traffic.map(|(_, digest)| digest),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> PathBuf {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let root = std::env::temp_dir().join(format!(
            "etsads-partition-evidence-{}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        ));
        std::fs::create_dir_all(root.join("sectors")).unwrap();
        std::fs::create_dir_all(root.join("traffic")).unwrap();
        // Byte-authentication fixtures only; not a validated geometry dataset
        // and never passed to a live-experiment constructor.
        let descriptor = |name: &str, bytes: &[u8]| {
            std::fs::write(root.join(name), bytes).unwrap();
            serde_json::json!({"path":name,"bytes":bytes.len(),"sha256":format!("{:x}",Sha256::digest(bytes))})
        };
        let lane = descriptor("sectors/a.bin", b"synthetic lane bytes");
        let traffic = descriptor("traffic/a.bin", b"synthetic annotation bytes");
        let native = descriptor("native.bin", b"synthetic ordered route bytes");
        let provenance = "01".repeat(32);
        let index = serde_json::json!({"schema_version":1,"format":"etsads_sector_lanes_jsonl_gzip_v1",
            "coordinate_system":"ENU","units":"metres","source_cache_key":"fixture","source_manifest_sha256":provenance,
            "compressed_bytes":lane["bytes"],"shards":[lane]});
        let lane_bytes = serde_json::to_vec(&index).unwrap();
        std::fs::write(root.join("sector_index.json"), &lane_bytes).unwrap();
        std::fs::write(
            root.join("manifest.json"),
            serde_json::to_vec(&serde_json::json!({"game_version":"synthetic",
            "source_cache_key":"fixture","source_manifest_sha256":provenance}))
            .unwrap(),
        )
        .unwrap();
        let index = serde_json::json!({"schema_version":1,"format":"etsads_sector_traffic_json_gzip_v1",
            "coordinate_system":"ENU","units":"metres","source_cache_key":"fixture","source_manifest_sha256":provenance,
            "lane_index_sha256":format!("{:x}",Sha256::digest(&lane_bytes)),
            "compressed_bytes":traffic["bytes"],"shards":[traffic],"native_route_index":native});
        std::fs::write(
            root.join("traffic_index.json"),
            serde_json::to_vec(&index).unwrap(),
        )
        .unwrap();
        root
    }
    #[test]
    fn partition_hash_reads_all_referenced_bytes_and_metadata_without_original_parquet() {
        let root = fixture();
        let initial = verify_partition(&root).unwrap();
        assert!(initial.annotations_source_id.is_some());
        assert!(!root.join("lanes.parquet").exists());
        for file in ["sectors/a.bin", "traffic/a.bin", "native.bin"] {
            let path = root.join(file);
            let original = std::fs::read(&path).unwrap();
            let mut changed = original.clone();
            changed[0] ^= 1;
            std::fs::write(&path, &changed).unwrap();
            assert!(
                verify_partition(&root)
                    .unwrap_err()
                    .to_string()
                    .contains("content changed")
            );
            std::fs::write(path, original).unwrap();
        }
        let path = root.join("manifest.json");
        let mut metadata: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        metadata["game_version"] = "different synthetic".into();
        std::fs::write(path, serde_json::to_vec(&metadata).unwrap()).unwrap();
        assert_ne!(
            initial.data_hash,
            verify_partition(&root).unwrap().data_hash
        );
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn partition_references_cannot_escape_alias_or_change_index_identity() {
        let root = fixture();
        let path = root.join("traffic_index.json");
        let original = std::fs::read(&path).unwrap();
        let mut index: serde_json::Value = serde_json::from_slice(&original).unwrap();
        index["lane_index_sha256"] = "02".repeat(32).into();
        std::fs::write(&path, serde_json::to_vec(&index).unwrap()).unwrap();
        assert!(
            verify_partition(&root)
                .unwrap_err()
                .to_string()
                .contains("not bound")
        );
        index = serde_json::from_slice(&original).unwrap();
        index["shards"][0]["path"] = "../escape.bin".into();
        std::fs::write(&path, serde_json::to_vec(&index).unwrap()).unwrap();
        assert!(
            verify_partition(&root)
                .unwrap_err()
                .to_string()
                .contains("traversal")
        );
        index = serde_json::from_slice(&original).unwrap();
        index["native_route_index"] = index["shards"][0].clone();
        std::fs::write(&path, serde_json::to_vec(&index).unwrap()).unwrap();
        assert!(
            verify_partition(&root)
                .unwrap_err()
                .to_string()
                .contains("duplicate")
        );
        std::fs::write(&path, original).unwrap();
        std::fs::remove_file(root.join("native.bin")).unwrap();
        assert!(verify_partition(&root).is_err());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn profile_proof_and_resolved_signals_are_actual_content_dependencies() {
        let root = fixture();
        let initial = verify_partition(&root).unwrap().data_hash;
        let path = root.join("traffic_index.json");
        let original = std::fs::read(&path).unwrap();
        let proof = br#"{"source":"synthetic profile proof"}"#;
        let signals = b"synthetic resolved table bytes; hash-only fixture";
        std::fs::write(root.join("proof.json"), proof).unwrap();
        std::fs::write(root.join("resolved.parquet"), signals).unwrap();
        let mut index: serde_json::Value = serde_json::from_slice(&original).unwrap();
        index["signal_profile_resolution"] = serde_json::json!({
            "evidence_path":"proof.json","evidence_bytes":proof.len(),"evidence_sha256":format!("{:x}",Sha256::digest(proof)),
            "resolved_signals_path":"resolved.parquet","resolved_signals_bytes":signals.len(),"resolved_signals_sha256":format!("{:x}",Sha256::digest(signals)),
            "activation_verified":false,"geometry_permissions_changed":false});
        std::fs::write(&path, serde_json::to_vec(&index).unwrap()).unwrap();
        assert_ne!(initial, verify_partition(&root).unwrap().data_hash);
        for name in ["proof.json", "resolved.parquet"] {
            let target = root.join(name);
            let content = std::fs::read(&target).unwrap();
            let mut changed = content.clone();
            changed[0] ^= 1;
            std::fs::write(&target, changed).unwrap();
            assert!(
                verify_partition(&root)
                    .unwrap_err()
                    .to_string()
                    .contains("content changed")
            );
            std::fs::write(target, content).unwrap();
        }
        index["signal_profile_resolution"]
            .as_object_mut()
            .unwrap()
            .remove("resolved_signals_sha256");
        std::fs::write(&path, serde_json::to_vec(&index).unwrap()).unwrap();
        assert!(
            verify_partition(&root).is_err(),
            "declaring references without complete digests is not compatible legacy data"
        );
        std::fs::write(&path, original).unwrap();
        assert_eq!(
            initial,
            verify_partition(&root).unwrap().data_hash,
            "old absent-field bundle digest is unchanged"
        );
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    #[ignore = "read-only complete partition authentication; set ETSADS_PARTITION_SOURCE"]
    fn existing_partition_complete_content_hash() {
        let path = std::env::var_os("ETSADS_PARTITION_SOURCE").unwrap();
        let start = std::time::Instant::now();
        let identity = verify_partition(Path::new(&path)).unwrap();
        println!(
            "{}",
            serde_json::json!({"execution":"read_only_partition_authentication","data_sha256":identity.data_hash,
            "source_id":identity.source_id,"annotations_source_id":identity.annotations_source_id,
            "elapsed_seconds":start.elapsed().as_secs_f64(),"phase_acceptance":false})
        );
    }
}
