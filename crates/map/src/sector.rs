//! Read-only sector storage. Each source lane belongs to exactly one shard;
//! shard coverage uses its geometry AABB, not just its nominal 4 km cell.
//! The cache owns bounded decoded data. Returned regional maps are caller-owned.
use crate::{
    MapError,
    importer::{ExportLane, ExportedMap, load_parquet_region_with_stats},
};
use etsads_msgs::Point3;
use flate2::read::MultiGzDecoder;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, HashSet},
    fs::{self, File},
    io::{self, BufRead, BufReader, Read},
    path::{Component, Path, PathBuf},
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, SyncSender},
    },
    thread,
    time::{Duration, Instant},
};

fn invalid(message: impl Into<String>) -> MapError {
    MapError::Invalid(message.into())
}
fn io_error(error: impl std::fmt::Display) -> MapError {
    invalid(error.to_string())
}
#[derive(Clone, Debug)]
pub struct CacheLimits {
    pub max_cache_bytes: usize,
    pub max_shards: usize,
    pub max_shard_decoded_bytes: usize,
    pub max_shard_uncompressed_bytes: u64,
    pub max_line_bytes: usize,
    pub max_region_points: usize,
    pub prefetch_queue: usize,
}
impl Default for CacheLimits {
    fn default() -> Self {
        Self {
            max_cache_bytes: 256 * 1024 * 1024,
            max_shards: 16,
            max_shard_decoded_bytes: 128 * 1024 * 1024,
            max_shard_uncompressed_bytes: 256 * 1024 * 1024,
            max_line_bytes: 16 * 1024 * 1024,
            max_region_points: 2_000_000,
            prefetch_queue: 4,
        }
    }
}
impl CacheLimits {
    fn validate(&self) -> Result<(), MapError> {
        if self.max_cache_bytes == 0
            || self.max_shards == 0
            || self.max_shard_decoded_bytes == 0
            || self.max_shard_decoded_bytes > self.max_cache_bytes
            || self.max_shard_uncompressed_bytes == 0
            || self.max_line_bytes == 0
            || self.max_region_points == 0
            || !(1..=32).contains(&self.prefetch_queue)
        {
            return Err(invalid("invalid sector cache limits"));
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ReadStats {
    pub source_kind: String,
    pub candidate_shards: usize,
    pub shards_opened: usize,
    pub cache_hits: usize,
    pub compressed_bytes_read: u64,
    pub uncompressed_bytes_read: u64,
    pub point_rows_decoded: u64,
    pub returned_lanes: usize,
    pub returned_points: usize,
    pub cache_resident_shards: usize,
    pub cache_resident_bytes: usize,
    pub cache_evictions: u64,
    pub source_storage_bytes: u64,
    pub elapsed_ms: f64,
}
#[derive(Clone, Debug)]
pub struct RegionLoad {
    pub map: ExportedMap,
    pub stats: ReadStats,
    pub source_id: String,
}
#[derive(Clone, Debug, Deserialize)]
struct ShardMeta {
    sector_x: i32,
    sector_z: i32,
    path: String,
    bounds: [f64; 4],
    lane_count: u64,
    point_count: u64,
    bytes: u64,
    uncompressed_bytes: u64,
    sha256: String,
}
#[derive(Deserialize)]
struct Index {
    schema_version: u32,
    format: String,
    coordinate_system: String,
    units: String,
    source: String,
    accepted_for_autonomous_driving: bool,
    source_cache_key: String,
    lane_count: u64,
    point_count: u64,
    compressed_bytes: u64,
    shards: Vec<ShardMeta>,
}
#[derive(Clone, Debug, Deserialize)]
struct StoredLane {
    bounds: [f64; 4],
    lane: ExportLane,
}
struct Shard {
    lanes: Vec<StoredLane>,
    heap_bytes: usize,
}
struct Entry {
    data: Arc<Shard>,
    tick: u64,
}
#[derive(Default)]
struct Cache {
    entries: BTreeMap<usize, Entry>,
    bytes: usize,
    tick: u64,
    evictions: u64,
}
enum Source {
    Partition(Index),
    LegacyParquet,
    JsonSlice,
}
struct Inner {
    root: PathBuf,
    source: Source,
    source_id: String,
    limits: CacheLimits,
    cache: Mutex<Cache>,
    io_lock: Mutex<()>,
    stop: AtomicBool,
    completion: Mutex<BTreeMap<u64, Option<Result<ReadStats, String>>>>,
    changed: Condvar,
}
struct Request {
    ticket: u64,
    center: Point3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PrefetchTicket(pub u64);
pub struct SectorStore {
    inner: Arc<Inner>,
    sender: Option<SyncSender<Request>>,
    worker: Option<thread::JoinHandle<()>>,
    next_ticket: AtomicU64,
}
impl SectorStore {
    pub fn open(path: impl AsRef<Path>, limits: CacheLimits) -> Result<Self, MapError> {
        limits.validate()?;
        let path = path.as_ref();
        let root = path.canonicalize().map_err(io_error)?;
        let root = if root.is_dir()
            && !root.join("sector_index.json").is_file()
            && !root.join("lanes.parquet").is_file()
            && root.join("runtime-map.json").is_file()
        {
            root.join("runtime-map.json")
                .canonicalize()
                .map_err(io_error)?
        } else {
            root
        };
        let (source, source_id) = if root.is_file() {
            (Source::JsonSlice, format!("json:{}", root.display()))
        } else if root.join("sector_index.json").is_file() {
            let file = root.join("sector_index.json");
            if file.metadata().map_err(io_error)?.len() > 16 * 1024 * 1024 {
                return Err(invalid("sector index exceeds 16 MB"));
            }
            let bytes = fs::read(file).map_err(io_error)?;
            let index: Index = serde_json::from_slice(&bytes)?;
            validate_index(&index)?;
            let digest = format!("{:x}", Sha256::digest(&bytes));
            (Source::Partition(index), digest)
        } else if root.join("lanes.parquet").is_file() {
            (Source::LegacyParquet, format!("legacy:{}", root.display()))
        } else {
            return Err(invalid(
                "map directory has neither a completed sector index nor legacy Parquet tables",
            ));
        };
        let queue = limits.prefetch_queue;
        let inner = Arc::new(Inner {
            root,
            source,
            source_id,
            limits,
            cache: Mutex::new(Cache::default()),
            io_lock: Mutex::new(()),
            stop: AtomicBool::new(false),
            completion: Mutex::new(BTreeMap::new()),
            changed: Condvar::new(),
        });
        let (sender, receiver) = mpsc::sync_channel::<Request>(queue);
        let weak = Arc::downgrade(&inner);
        let worker = thread::Builder::new()
            .name("map-prefetch".into())
            .spawn(move || {
                while let Ok(request) = receiver.recv() {
                    let Some(inner) = weak.upgrade() else {
                        break;
                    };
                    if inner.stop.load(Ordering::Acquire) {
                        break;
                    }
                    let result = inner
                        .load_region(request.center, 2000.0)
                        .map(|r| r.stats)
                        .map_err(|e| e.to_string());
                    let mut complete = inner.completion.lock().unwrap_or_else(|p| p.into_inner());
                    complete.insert(request.ticket, Some(result));
                    while complete.len() > 64 {
                        let Some(key) =
                            complete.iter().find_map(|(k, v)| v.is_some().then_some(*k))
                        else {
                            break;
                        };
                        complete.remove(&key);
                    }
                    inner.changed.notify_all();
                }
            })
            .map_err(io_error)?;
        Ok(Self {
            inner,
            sender: Some(sender),
            worker: Some(worker),
            next_ticket: AtomicU64::new(1),
        })
    }
    pub fn load_region(&self, center: Point3, radius: f64) -> Result<RegionLoad, MapError> {
        self.inner.load_region(center, radius)
    }
    pub fn load_2km(&self, center: Point3) -> Result<RegionLoad, MapError> {
        self.load_region(center, 2000.0)
    }
    /// Bounded nonblocking submission. Legacy inputs remain readable but need
    /// partitioning before background prefetch can avoid whole-table scans.
    pub fn prefetch_2km(&self, center: Point3) -> Result<PrefetchTicket, MapError> {
        if !center.valid() {
            return Err(invalid("invalid prefetch center"));
        }
        if !matches!(self.inner.source, Source::Partition(_)) {
            return Err(invalid(
                "background prefetch requires a sector index; legacy input remains synchronously readable",
            ));
        }
        let ticket = self.next_ticket.fetch_add(1, Ordering::Relaxed);
        self.inner
            .completion
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(ticket, None);
        if let Err(error) = self
            .sender
            .as_ref()
            .ok_or_else(|| invalid("prefetch worker closed"))?
            .try_send(Request { ticket, center })
        {
            self.inner
                .completion
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .remove(&ticket);
            return Err(invalid(format!("prefetch queue unavailable: {error}")));
        }
        Ok(PrefetchTicket(ticket))
    }
    pub fn wait_prefetch(
        &self,
        ticket: PrefetchTicket,
        timeout: Duration,
    ) -> Result<Option<ReadStats>, MapError> {
        let guard = self
            .inner
            .completion
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let (mut guard, _) = self
            .inner
            .changed
            .wait_timeout_while(guard, timeout, |map| {
                matches!(map.get(&ticket.0), Some(None))
            })
            .unwrap_or_else(|p| p.into_inner());
        match guard.get(&ticket.0) {
            Some(None) => Ok(None),
            Some(Some(_)) => match guard.remove(&ticket.0).unwrap().unwrap() {
                Ok(stats) => Ok(Some(stats)),
                Err(error) => Err(invalid(error)),
            },
            None => Err(invalid("unknown or expired prefetch ticket")),
        }
    }
}
impl Drop for SectorStore {
    fn drop(&mut self) {
        self.inner.stop.store(true, Ordering::Release);
        self.sender.take();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
impl Inner {
    fn load_region(&self, center: Point3, radius: f64) -> Result<RegionLoad, MapError> {
        if !center.valid() || !radius.is_finite() || radius <= 0.0 || radius > 5000.0 {
            return Err(invalid("invalid regional query"));
        }
        let _serial = self.io_lock.lock().unwrap_or_else(|p| p.into_inner());
        let started = Instant::now();
        let bounds = [
            center.x - radius,
            center.y - radius,
            center.x + radius,
            center.y + radius,
        ];
        let mut stats = ReadStats::default();
        let map = match &self.source {
            Source::Partition(index) => {
                stats.source_kind = "sector_jsonl_gzip_v1".into();
                stats.source_storage_bytes = index.compressed_bytes;
                let candidates = index
                    .shards
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| intersects(s.bounds, bounds))
                    .collect::<Vec<_>>();
                stats.candidate_shards = candidates.len();
                let mut selected = BTreeMap::new();
                let mut total_points = 0;
                for (id, meta) in candidates {
                    if self.stop.load(Ordering::Acquire) {
                        return Err(invalid("map store closing"));
                    }
                    let shard = self.shard(id, meta, &mut stats)?;
                    for entry in &shard.lanes {
                        if intersects(entry.bounds, bounds) {
                            total_points += entry.lane.center.len();
                            if total_points > self.limits.max_region_points {
                                return Err(invalid("regional point budget exceeded"));
                            }
                            if selected
                                .insert(entry.lane.id.clone(), entry.lane.clone())
                                .is_some()
                            {
                                return Err(invalid("duplicate lane across sector shards"));
                            }
                        }
                    }
                }
                let mut map = ExportedMap {
                    schema_version: 1,
                    accepted_for_autonomous_driving: index.accepted_for_autonomous_driving,
                    source: index.source.clone(),
                    lanes: selected.into_values().collect(),
                };
                clip(&mut map);
                map
            }
            Source::LegacyParquet => {
                stats.source_kind = "legacy_parquet".into();
                stats.source_storage_bytes = [
                    "lanes.parquet",
                    "lane_points.parquet",
                    "lane_links.parquet",
                    "lane_neighbors.parquet",
                ]
                .iter()
                .filter_map(|f| self.root.join(f).metadata().ok().map(|m| m.len()))
                .sum();
                let (map, io) = load_parquet_region_with_stats(&self.root, center, radius)?;
                stats.compressed_bytes_read = io.logical_bytes_read;
                stats.point_rows_decoded = io.point_rows_decoded;
                map
            }
            Source::JsonSlice => {
                stats.source_kind = "legacy_json_slice".into();
                stats.source_storage_bytes = self.root.metadata().map_err(io_error)?.len();
                let mut map = ExportedMap::from_path(&self.root)?;
                stats.compressed_bytes_read = stats.source_storage_bytes;
                stats.uncompressed_bytes_read = stats.source_storage_bytes;
                stats.point_rows_decoded = map.lanes.iter().map(|l| l.center.len() as u64).sum();
                map.lanes.retain(|l| {
                    l.center
                        .iter()
                        .fold(None, |b: Option<[f64; 4]>, p| {
                            Some(match b {
                                None => [p.x, p.y, p.x, p.y],
                                Some(v) => {
                                    [v[0].min(p.x), v[1].min(p.y), v[2].max(p.x), v[3].max(p.y)]
                                }
                            })
                        })
                        .is_some_and(|b| intersects(b, bounds))
                });
                clip(&mut map);
                map
            }
        };
        stats.returned_lanes = map.lanes.len();
        stats.returned_points = map.lanes.iter().map(|l| l.center.len()).sum();
        if stats.returned_points > self.limits.max_region_points {
            return Err(invalid("regional point budget exceeded"));
        }
        let cache = self.cache.lock().unwrap_or_else(|p| p.into_inner());
        stats.cache_resident_shards = cache.entries.len();
        stats.cache_resident_bytes = cache.bytes;
        stats.cache_evictions = cache.evictions;
        stats.elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
        Ok(RegionLoad {
            map,
            stats,
            source_id: self.source_id.clone(),
        })
    }
    fn shard(
        &self,
        id: usize,
        meta: &ShardMeta,
        stats: &mut ReadStats,
    ) -> Result<Arc<Shard>, MapError> {
        {
            let mut cache = self.cache.lock().unwrap_or_else(|p| p.into_inner());
            cache.tick += 1;
            let tick = cache.tick;
            if let Some(entry) = cache.entries.get_mut(&id) {
                entry.tick = tick;
                stats.cache_hits += 1;
                return Ok(entry.data.clone());
            }
            // Reserve decoder headroom before reading another shard. Cached
            // data plus the new decoded shard never exceed max_cache_bytes.
            while !cache.entries.is_empty()
                && (cache.entries.len() >= self.limits.max_shards
                    || cache.bytes + self.limits.max_shard_decoded_bytes
                        > self.limits.max_cache_bytes)
            {
                evict(&mut cache);
            }
        }
        let shard = Arc::new(decode(&self.root, meta, &self.limits, stats)?);
        let mut cache = self.cache.lock().unwrap_or_else(|p| p.into_inner());
        while !cache.entries.is_empty()
            && (cache.entries.len() >= self.limits.max_shards
                || cache.bytes + shard.heap_bytes > self.limits.max_cache_bytes)
        {
            evict(&mut cache);
        }
        cache.bytes += shard.heap_bytes;
        cache.tick += 1;
        let tick = cache.tick;
        cache.entries.insert(
            id,
            Entry {
                data: shard.clone(),
                tick,
            },
        );
        Ok(shard)
    }
}
fn evict(cache: &mut Cache) {
    if let Some(id) = cache
        .entries
        .iter()
        .min_by_key(|(_, e)| e.tick)
        .map(|(id, _)| *id)
    {
        cache.bytes -= cache.entries.remove(&id).unwrap().data.heap_bytes;
        cache.evictions += 1;
    }
}
fn intersects(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1]
}
fn valid_bounds(b: [f64; 4]) -> bool {
    b.iter().all(|v| v.is_finite() && v.abs() < 1e8) && b[0] <= b[2] && b[1] <= b[3]
}
fn clip(map: &mut ExportedMap) {
    let ids: HashSet<_> = map.lanes.iter().map(|l| l.id.clone()).collect();
    for lane in &mut map.lanes {
        lane.successors.retain(|id| ids.contains(id));
        lane.predecessors.retain(|id| ids.contains(id));
        lane.successors.sort();
        lane.successors.dedup();
        lane.predecessors.sort();
        lane.predecessors.dedup();
        lane.retain_neighbors(&ids);
    }
}
fn validate_index(index: &Index) -> Result<(), MapError> {
    if index.schema_version != 1
        || index.format != "etsads_sector_lanes_jsonl_gzip_v1"
        || index.coordinate_system != "ENU"
        || index.units != "metres"
        || index.source_cache_key.is_empty()
        || index.shards.is_empty()
        || index.shards.len() > 100_000
    {
        return Err(invalid("unsupported or incomplete sector index"));
    }
    let mut keys = HashSet::new();
    let mut paths = HashSet::new();
    for shard in &index.shards {
        if !valid_bounds(shard.bounds)
            || !keys.insert((shard.sector_x, shard.sector_z))
            || !paths.insert(&shard.path)
            || shard.bytes == 0
            || shard.uncompressed_bytes == 0
            || shard.sha256.len() != 64
            || !shard.sha256.bytes().all(|v| v.is_ascii_hexdigit())
        {
            return Err(invalid("invalid sector descriptor"));
        }
        if Path::new(&shard.path)
            .components()
            .any(|c| !matches!(c, Component::Normal(_)))
        {
            return Err(invalid(
                "sector path must stay relative to its map directory",
            ));
        }
    }
    if index.shards.iter().map(|s| s.lane_count).sum::<u64>() != index.lane_count
        || index.shards.iter().map(|s| s.point_count).sum::<u64>() != index.point_count
        || index.shards.iter().map(|s| s.bytes).sum::<u64>() != index.compressed_bytes
    {
        return Err(invalid("sector index totals disagree"));
    }
    Ok(())
}
struct CountedRead {
    file: File,
    hash: Sha256,
    bytes: u64,
}
impl Read for CountedRead {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let n = self.file.read(out)?;
        self.hash.update(&out[..n]);
        self.bytes += n as u64;
        Ok(n)
    }
}
fn bounded_line(reader: &mut impl BufRead, limit: usize) -> Result<Option<Vec<u8>>, MapError> {
    let mut line = Vec::new();
    loop {
        let available = reader.fill_buf().map_err(io_error)?;
        if available.is_empty() {
            return if line.is_empty() {
                Ok(None)
            } else {
                Err(invalid("unterminated sector JSON line"))
            };
        }
        let end = available.iter().position(|b| *b == b'\n');
        let count = end.map_or(available.len(), |i| i + 1);
        if line.len().saturating_add(count) > limit {
            return Err(invalid("sector lane line exceeds byte budget"));
        }
        line.extend_from_slice(&available[..count]);
        reader.consume(count);
        if end.is_some() {
            return Ok(Some(line));
        }
    }
}
fn heap(lane: &ExportLane) -> usize {
    lane.id.capacity()
        + lane.geometry_status.capacity()
        + lane.center.capacity() * std::mem::size_of::<Point3>()
        + lane.successors.capacity() * std::mem::size_of::<String>()
        + lane.successors.iter().map(String::capacity).sum::<usize>()
        + lane.predecessors.capacity() * std::mem::size_of::<String>()
        + lane
            .predecessors
            .iter()
            .map(String::capacity)
            .sum::<usize>()
        + lane.left.as_ref().map_or(0, String::capacity)
        + lane.right.as_ref().map_or(0, String::capacity)
}
fn decode(
    root: &Path,
    meta: &ShardMeta,
    limits: &CacheLimits,
    stats: &mut ReadStats,
) -> Result<Shard, MapError> {
    if meta.uncompressed_bytes > limits.max_shard_uncompressed_bytes {
        return Err(invalid("sector decompression byte budget exceeded"));
    }
    let path = root.join(&meta.path).canonicalize().map_err(io_error)?;
    if !path.starts_with(root) {
        return Err(invalid("sector path escapes map directory"));
    }
    let mut options = fs::OpenOptions::new();
    options.read(true);
    #[cfg(windows)]
    {
        use std::os::windows::fs::OpenOptionsExt;
        options.share_mode(1);
    }
    let file = options.open(path).map_err(io_error)?;
    if file.metadata().map_err(io_error)?.len() != meta.bytes {
        return Err(invalid("sector file size changed"));
    }
    let counted = CountedRead {
        file,
        hash: Sha256::new(),
        bytes: 0,
    };
    let decoder = MultiGzDecoder::new(counted);
    let mut reader = BufReader::with_capacity(
        65536,
        decoder.take(meta.uncompressed_bytes.saturating_add(1)),
    );
    let mut lanes = Vec::<StoredLane>::new();
    let mut uncompressed = 0_u64;
    let mut points = 0_u64;
    let mut dynamic_heap = 0_usize;
    let mut ids = HashSet::new();
    while let Some(line) = bounded_line(&mut reader, limits.max_line_bytes)? {
        uncompressed += line.len() as u64;
        if uncompressed > meta.uncompressed_bytes {
            return Err(invalid("sector exceeds declared decompressed size"));
        }
        let stored: StoredLane = serde_json::from_slice(&line)?;
        let lane = &stored.lane;
        if !valid_bounds(stored.bounds)
            || lane.id.is_empty()
            || lane.id.len() > 256
            || !ids.insert(lane.id.clone())
            || lane.center.len() < 2
            || lane.center.len() > 1_000_000
            || !lane.center.iter().all(|p| {
                p.valid()
                    && p.x >= stored.bounds[0]
                    && p.x <= stored.bounds[2]
                    && p.y >= stored.bounds[1]
                    && p.y <= stored.bounds[3]
            })
        {
            return Err(invalid("invalid sector lane geometry or identity"));
        }
        if !intersects(meta.bounds, stored.bounds)
            || stored.bounds[0] < meta.bounds[0]
            || stored.bounds[1] < meta.bounds[1]
            || stored.bounds[2] > meta.bounds[2]
            || stored.bounds[3] > meta.bounds[3]
        {
            return Err(invalid("lane falls outside indexed shard coverage"));
        }
        points += lane.center.len() as u64;
        dynamic_heap = dynamic_heap.saturating_add(heap(lane));
        lanes.push(stored);
        let bytes =
            dynamic_heap.saturating_add(lanes.capacity() * std::mem::size_of::<StoredLane>());
        if bytes > limits.max_shard_decoded_bytes
            || lanes.len() as u64 > meta.lane_count
            || points > meta.point_count
        {
            return Err(invalid("sector decoded memory/count budget exceeded"));
        }
    }
    let counted = reader.into_inner().into_inner().into_inner();
    if uncompressed != meta.uncompressed_bytes
        || lanes.len() as u64 != meta.lane_count
        || points != meta.point_count
        || counted.bytes != meta.bytes
        || format!("{:x}", counted.hash.finalize()) != meta.sha256.to_ascii_lowercase()
    {
        return Err(invalid("sector content hash or count mismatch"));
    }
    stats.shards_opened += 1;
    stats.compressed_bytes_read += counted.bytes;
    stats.uncompressed_bytes_read += uncompressed;
    stats.point_rows_decoded += points;
    Ok(Shard {
        heap_bytes: dynamic_heap + lanes.capacity() * std::mem::size_of::<StoredLane>(),
        lanes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use flate2::{Compression, write::GzEncoder};
    use std::io::Write;
    struct Fixture {
        root: PathBuf,
    }
    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.root);
        }
    }
    impl Fixture {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let root = std::env::temp_dir().join(format!(
                "etsads-sector-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&root).unwrap();
            let mut metas = Vec::new();
            for (i, (x, id, next)) in [(0.0, "a", Some("b")), (5000.0, "b", None)]
                .into_iter()
                .enumerate()
            {
                let lane = ExportLane {
                    id: id.into(),
                    owner_uid: i as u64 + 1,
                    node_start: i as u64 + 1,
                    node_end: i as u64 + 2,
                    center: vec![
                        Point3 { x, y: 0.0, z: 0.0 },
                        Point3 {
                            x: x + 100.0,
                            y: 0.0,
                            z: 0.0,
                        },
                    ],
                    successors: next.into_iter().map(str::to_string).collect(),
                    ..Default::default()
                };
                let line = serde_json::to_vec(
                    &serde_json::json!({"bounds":[x,0.0,x+100.0,0.0],"lane":lane}),
                )
                .unwrap();
                let mut encoder = GzEncoder::new(Vec::new(), Compression::fast());
                encoder.write_all(&line).unwrap();
                encoder.write_all(b"\n").unwrap();
                let gzip = encoder.finish().unwrap();
                let path = format!("{i}.jsonl.gz");
                fs::write(root.join(&path), &gzip).unwrap();
                metas.push(serde_json::json!({"sector_x":i,"sector_z":0,"path":path,"bounds":[x,0.0,x+100.0,0.0],
                    "lane_count":1,"point_count":2,"bytes":gzip.len(),"uncompressed_bytes":line.len()+1,"sha256":format!("{:x}",Sha256::digest(&gzip))}));
            }
            let total: u64 = metas.iter().map(|m| m["bytes"].as_u64().unwrap()).sum();
            fs::write(root.join("sector_index.json"),serde_json::to_vec(&serde_json::json!({
                "schema_version":1,"format":"etsads_sector_lanes_jsonl_gzip_v1","coordinate_system":"ENU","units":"metres",
                "source":"synthetic_test_only","accepted_for_autonomous_driving":false,"source_cache_key":"fixture",
                "lane_count":2,"point_count":4,"compressed_bytes":total,"shards":metas})).unwrap()).unwrap();
            Self { root }
        }
    }
    #[test]
    fn relevant_only_cache_eviction_and_background_prefetch() {
        let fixture = Fixture::new();
        let limits = CacheLimits {
            max_shards: 1,
            ..Default::default()
        };
        let store = SectorStore::open(&fixture.root, limits).unwrap();
        let a = store.load_2km(Point3::default()).unwrap();
        assert_eq!(a.stats.shards_opened, 1);
        assert_eq!(a.map.lanes.len(), 1);
        assert!(a.map.lanes[0].successors.is_empty());
        assert!(!a.map.audit().driveable);
        assert_eq!(
            store
                .load_2km(Point3::default())
                .unwrap()
                .stats
                .compressed_bytes_read,
            0
        );
        let center = Point3 {
            x: 5000.0,
            y: 0.0,
            z: 0.0,
        };
        let ticket = store.prefetch_2km(center).unwrap();
        let p = store
            .wait_prefetch(ticket, Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(p.shards_opened, 1);
        assert_eq!(p.cache_evictions, 1);
        let b = store.load_2km(center).unwrap();
        assert_eq!(b.stats.cache_hits, 1);
        assert_eq!(b.stats.cache_resident_shards, 1);
        assert!(b.stats.cache_resident_bytes < CacheLimits::default().max_cache_bytes);
        assert_eq!(
            store
                .load_2km(Point3::default())
                .unwrap()
                .stats
                .shards_opened,
            1
        );
    }
    #[test]
    fn corrupt_shards_and_budgets_are_rejected_before_cache_insertion() {
        let fixture = Fixture::new();
        let mut index: serde_json::Value =
            serde_json::from_slice(&fs::read(fixture.root.join("sector_index.json")).unwrap())
                .unwrap();
        index["shards"][0]["sha256"] = serde_json::Value::String("0".repeat(64));
        fs::write(
            fixture.root.join("sector_index.json"),
            serde_json::to_vec(&index).unwrap(),
        )
        .unwrap();
        let store = SectorStore::open(&fixture.root, CacheLimits::default()).unwrap();
        assert!(
            store
                .load_2km(Point3::default())
                .unwrap_err()
                .to_string()
                .contains("hash")
        );
        assert!(store.inner.cache.lock().unwrap().entries.is_empty());
        let limited = SectorStore::open(
            &fixture.root,
            CacheLimits {
                max_line_bytes: 16,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(
            limited
                .load_2km(Point3::default())
                .unwrap_err()
                .to_string()
                .contains("line")
        );
    }
    #[test]
    fn geometry_coverage_not_nominal_sector_and_paths_cannot_escape() {
        let fixture = Fixture::new();
        let mut index: serde_json::Value =
            serde_json::from_slice(&fs::read(fixture.root.join("sector_index.json")).unwrap())
                .unwrap();
        index["shards"][0]["sector_x"] = 999.into();
        fs::write(
            fixture.root.join("sector_index.json"),
            serde_json::to_vec(&index).unwrap(),
        )
        .unwrap();
        let store = SectorStore::open(&fixture.root, CacheLimits::default()).unwrap();
        assert_eq!(
            store
                .load_region(Point3::default(), 0.5)
                .unwrap()
                .map
                .lanes
                .len(),
            1
        );
        index["shards"][0]["path"] = "../outside.gz".into();
        fs::write(
            fixture.root.join("sector_index.json"),
            serde_json::to_vec(&index).unwrap(),
        )
        .unwrap();
        assert!(SectorStore::open(&fixture.root, CacheLimits::default()).is_err());
    }
}
