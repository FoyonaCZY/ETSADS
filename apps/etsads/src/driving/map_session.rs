//! Background regional navigation loader used by the driving host. There is
//! one bounded request/result in flight; control never waits for map disk I/O.
use anyhow::{Context, ensure};
use etsads_game::adapter::GameSnapshot;
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    corridor::{GlobalNativeRouteIndex, NativeRouteIndex},
    importer::RegionPolicy,
    sector::{CacheLimits, SectorStore},
};
use etsads_msgs::Point3;
use etsads_runtime::{
    ExecutionMode, NavigationRegion, RouteSource, Runtime, RuntimeConfig, navigation::MapProvenance,
};
use std::{
    path::Path,
    sync::{
        Arc,
        mpsc::{self, Receiver, SyncSender},
    },
    thread,
};

struct Request {
    center: Point3,
    speed_limit: f64,
    generation: u64,
    heading: f64,
}

struct Loader {
    map: SectorStore,
    annotations: AnnotationStore,
    global: Arc<GlobalNativeRouteIndex>,
    policy: RegionPolicy,
    live: bool,
    road_evidence: Option<etsads_map::importer::RoadEvidence>,
    road_boundary: Option<etsads_runtime::navigation::RoadBoundary>,
}
impl Loader {
    fn load(&self, request: &Request) -> anyhow::Result<NavigationRegion> {
        let loaded = self.map.load_2km(request.center)?;
        ensure!(
            !self.live || loaded.map.source == "installed_game_archives",
            "live native map must come from installed game archives"
        );
        let native = NativeRouteIndex::from_region(&loaded)?;
        let annotation = self
            .annotations
            .load_for_region(request.center, 2000.0, &loaded)?;
        let policy = self.policy.for_loaded_map(&loaded.map)?;
        let region = if let Some(evidence) = &self.road_evidence {
            loaded
                .map
                .into_road_region(
                    etsads_map::importer::RoadRegionPolicy {
                        authorization: policy,
                        evidence: evidence.clone(),
                    },
                    request.speed_limit,
                )?
                .region
        } else {
            loaded
                .map
                .into_commissioning_region(policy, request.speed_limit)?
        };
        let native = native.for_region_graph(&region)?;
        Ok(NavigationRegion {
            region,
            native,
            global: self.global.clone(),
            traffic: annotation.traffic,
            gate_candidates: annotation.gate_candidates,
            authorization: self.policy.clone(),
            road_evidence: self.road_evidence.clone(),
            road_boundary: self.road_boundary.clone(),
            provenance: MapProvenance {
                source_id: loaded.source_id,
                annotations_source_id: annotation.source_id,
                center: request.center,
                radius_m: 2000.0,
                generation: request.generation,
                reason: if request.generation == 1 {
                    "initial native GPS window"
                } else {
                    "vehicle moved 600 m; background 2 km replacement"
                }
                .into(),
                reads: loaded.stats,
            },
        })
    }
    fn prefetch(&self, request: &Request) {
        let center = Point3 {
            x: request.center.x + 1000.0 * request.heading.cos(),
            y: request.center.y + 1000.0 * request.heading.sin(),
            z: request.center.z,
        };
        // The authoritative replacement still reports load errors. A failed
        // speculative read cannot enlarge the executable corridor.
        let _ = self.map.prefetch_2km(center);
    }
}
pub struct MapSession {
    requests: SyncSender<Request>,
    results: Receiver<Result<NavigationRegion, String>>,
    center: Point3,
    generation: u64,
    loading: bool,
    pending: Option<NavigationRegion>,
}
impl MapSession {
    /// Replay observations advance independently of disk wall time. Wait for a
    /// requested map read here, outside Runtime, without ever touching IPC.
    pub fn poll_replay(
        &mut self,
        snapshot: &GameSnapshot,
        runtime: &mut Runtime,
    ) -> anyhow::Result<()> {
        self.poll(snapshot, runtime)?;
        if self.loading {
            self.pending = Some(
                self.results
                    .recv_timeout(std::time::Duration::from_secs(30))
                    .context("offline region read timeout/disconnect")?
                    .map_err(anyhow::Error::msg)?,
            );
            self.loading = false;
        }
        if self.pending.is_some() && runtime.can_replace_native_region() {
            let map = self.pending.take().unwrap();
            self.center = map.provenance.center;
            runtime.install_native_region(map)?;
        }
        Ok(())
    }
    pub fn start(
        source: &Path,
        policy: RegionPolicy,
        config: &RuntimeConfig,
        config_directory: &Path,
        first: &GameSnapshot,
    ) -> anyhow::Result<(Self, NavigationRegion)> {
        ensure!(
            matches!(config.route_source, RouteSource::NativeGps(_)),
            "moving map session requires native GPS route source"
        );
        ensure!(
            source.is_dir() && source.join("sector_index.json").is_file(),
            "native GPS moving maps require a portable sector partition directory"
        );
        let loader = Loader {
            map: SectorStore::open(source, CacheLimits::default())?,
            annotations: AnnotationStore::open(source, AnnotationLimits::default())?,
            global: Arc::new(GlobalNativeRouteIndex::load_source(source)?),
            policy,
            live: config.execution == ExecutionMode::Live,
            road_evidence: match &config.route_source {
                RouteSource::NativeGps(settings) => settings.road_evidence.clone(),
                _ => None,
            },
            road_boundary: match &config.route_source {
                RouteSource::NativeGps(settings) if settings.road_evidence.is_some() => {
                    let path = Path::new(
                        settings
                            .road_boundary_path
                            .as_deref()
                            .context("road configuration requires road_boundary_path")?,
                    );
                    let path = if path.is_absolute() {
                        path.to_owned()
                    } else {
                        config_directory.join(path)
                    };
                    Some(
                        super::read_config(&path, false)
                            .context("read measured ENU road boundary")?,
                    )
                }
                _ => None,
            },
        };
        let center = Point3 {
            x: first.ego.pose.x,
            y: first.ego.pose.y,
            z: first.ego.z,
        };
        let initial = Request {
            center,
            speed_limit: first.raw.navigation_speed_limit_mps as f64,
            generation: 1,
            heading: first.ego.pose.yaw,
        };
        let map = loader.load(&initial)?;
        if let RouteSource::NativeGps(settings) = &config.route_source {
            map.validate(settings).map_err(anyhow::Error::msg)?;
        }
        let (requests, rx) = mpsc::sync_channel::<Request>(1);
        let (tx, results) = mpsc::sync_channel(1);
        thread::Builder::new()
            .name("etsads-map-regions".into())
            .spawn(move || {
                loader.prefetch(&initial);
                while let Ok(request) = rx.recv() {
                    let result = loader
                        .load(&request)
                        .map_err(|e| format!("background native map update failed: {e:#}"));
                    if tx.send(result).is_err() {
                        break;
                    }
                    loader.prefetch(&request);
                }
            })
            .context("spawn native map worker")?;
        Ok((
            Self {
                requests,
                results,
                center,
                generation: 1,
                loading: false,
                pending: None,
            },
            map,
        ))
    }
    pub fn poll(&mut self, snapshot: &GameSnapshot, runtime: &mut Runtime) -> anyhow::Result<()> {
        if self.loading {
            match self.results.try_recv() {
                Ok(result) => {
                    self.pending = Some(result.map_err(anyhow::Error::msg)?);
                    self.loading = false;
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => {
                    anyhow::bail!("native map worker disconnected")
                }
            }
        }
        if self.pending.is_some() && runtime.can_replace_native_region() {
            let map = self.pending.take().unwrap();
            self.center = map.provenance.center;
            runtime.install_native_region(map)?;
        }
        let center = Point3 {
            x: snapshot.ego.pose.x,
            y: snapshot.ego.pose.y,
            z: snapshot.ego.z,
        };
        let distance = center.distance_xy(self.center);
        ensure!(
            distance < 2000.0,
            "vehicle left the loaded 2 km region before a verified replacement was ready"
        );
        if distance >= 600.0 && !self.loading && self.pending.is_none() {
            self.generation = self
                .generation
                .checked_add(1)
                .context("map generation overflow")?;
            self.requests
                .try_send(Request {
                    center,
                    speed_limit: snapshot.raw.navigation_speed_limit_mps as f64,
                    generation: self.generation,
                    heading: snapshot.ego.pose.yaw,
                })
                .context("native map request queue unavailable")?;
            self.loading = true;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use etsads_msgs::{
        EgoState, Header, Pose2, Scene,
        protocol::{ConfigBlock, ExtensionFrame, TelemetryFrame},
    };
    fn snapshot(x: f64, index: u64) -> GameSnapshot {
        let ego = EgoState {
            header: Header {
                frame_id: index,
                sim_time_us: index * 20000,
                seq: index,
            },
            pose: Pose2 {
                x,
                y: 0.0,
                yaw: 0.0,
            },
            speed: 6.0,
            ..Default::default()
        };
        let mut raw = TelemetryFrame {
            navigation_speed_limit_mps: 8.0,
            speed_mps: 6.0,
            ..Default::default()
        };
        raw.header.frame_id = index;
        raw.trailers[0].connected = 1;
        let mut extension = Box::<ExtensionFrame>::default();
        extension.header = raw.header;
        extension.available = 18;
        GameSnapshot {
            ego,
            raw,
            extension: Some(extension),
            config: Some(ConfigBlock::default()),
            events: vec![],
            scene: Scene {
                ego,
                route: vec![2, 3, 4],
                ..Default::default()
            },
            local_origin: [0.0; 3],
            origin_generation: 1,
            session_start_qpc: 1,
        }
    }
    #[test]
    fn production_background_loader_replaces_window_and_rebinds_runtime() {
        let (config, calibration, map) =
            etsads_runtime::navigation_regression::synthetic_disk_navigation_fixture(1200.0, 1);
        let source = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../crates/map/testdata/navigation-sectors");
        let (mut session, initial) = MapSession::start(
            &source,
            map.authorization.clone(),
            &config,
            Path::new("."),
            &snapshot(2500.0, 0),
        )
        .unwrap();
        let mut runtime = Runtime::new(config, calibration, Some(initial.region.clone())).unwrap();
        runtime.install_native_region(initial).unwrap();
        runtime.step(&snapshot(2500.0, 0));
        let before = runtime.step(&snapshot(2500.12, 1));
        assert_eq!(
            before.status,
            etsads_runtime::RuntimeStatus::Running,
            "{}",
            before.reason
        );
        // Separate open-loop observations test loader/ID semantics, not vehicle
        // continuity. Closed-loop tracking has its own Runtime regression.
        session
            .poll_replay(&snapshot(3200.0, 2), &mut runtime)
            .unwrap();
        let after = runtime.step(&snapshot(3200.0, 2));
        assert_eq!(
            after.status,
            etsads_runtime::RuntimeStatus::Running,
            "{}",
            after.reason
        );
        assert_eq!(after.navigation.as_ref().unwrap().map.generation, 2);
        assert_eq!(
            after.navigation.unwrap().corridor_source_ids,
            vec!["m", "z"]
        );
        assert_eq!(after.control_ego.pose.x, 3200.0);
    }
}
