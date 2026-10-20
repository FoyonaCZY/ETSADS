//! Optional Rerun sink shared by immutable MCAP replay and read-only live polling.
//! GameAdapter owns scene conversion. Native traffic/parked boxes retain their
//! measured height and rotation; missing ego dimensions are never invented.
use anyhow::{Context, ensure};
use etsads_game::{adapter::GameSnapshot, ets2la};
use etsads_msgs::{Point3, Pose2, frames::scs_to_enu};
use rerun as rr;
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::Path};

#[derive(Default)]
pub struct MapContext {
    pub nodes: Vec<NativeNode>,
    pub lanes: Vec<etsads_map::importer::ExportLane>,
    pub runtime_audit: Option<etsads_map::importer::ImportAudit>,
}
#[derive(Deserialize)]
pub struct NativeNode {
    pub uid: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl MapContext {
    pub fn from_path(path: &Path) -> anyhow::Result<Self> {
        ensure!(
            std::fs::metadata(path)?.len() <= 128 * 1024 * 1024,
            "viewer map JSON exceeds 128 MiB; provide a regional slice"
        );
        // Reuse the exact runtime schema loader; rendering must not maintain an
        // independent interpretation of exported lane IDs or coordinates.
        let model = etsads_map::importer::ExportedMap::from_path(path)
            .with_context(|| format!("runtime map schema rejected {}", path.display()))?;
        let audit = model.audit();
        #[derive(Deserialize)]
        struct NodeDocument {
            #[serde(default)]
            nodes: Vec<NativeNode>,
        }
        let nodes: NodeDocument = serde_json::from_reader(std::fs::File::open(path)?)?;
        let this = Self {
            nodes: nodes.nodes,
            lanes: model.lanes,
            runtime_audit: Some(audit),
        };
        ensure!(
            this.nodes.iter().all(|n| Point3 {
                x: n.x,
                y: n.y,
                z: n.z
            }
            .valid()),
            "invalid native map node"
        );
        ensure!(
            this.lanes
                .iter()
                .all(|l| l.center.iter().all(|p| p.valid())),
            "invalid lane point"
        );
        Ok(this)
    }
    pub fn node_lookup(&self) -> BTreeMap<u64, [f32; 3]> {
        self.nodes
            .iter()
            .map(|n| (n.uid, [n.x as f32, n.y as f32, n.z as f32]))
            .collect()
    }
    pub fn log(&self, rec: &rr::RecordingStream) -> anyhow::Result<()> {
        if let Some(audit) = &self.runtime_audit {
            rec.log_static(
                "status/map",
                &rr::TextDocument::new(format!(
                    "Runtime schema parsed. Visualization does not authorize driving.\n{}",
                    serde_json::to_string_pretty(audit)?
                )),
            )?;
        }
        if !self.lanes.is_empty() {
            rec.log_static(
                "world/map/centrelines",
                &rr::LineStrips3D::new(self.lanes.iter().map(|lane| {
                    lane.center
                        .iter()
                        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                        .collect::<Vec<_>>()
                }))
                .with_colors([[65, 125, 160]])
                .with_radii([rr::Radius::new_ui_points(1.0)]),
            )?;
        }
        Ok(())
    }
}

pub struct Sink {
    pub recording: rr::RecordingStream,
    pub first_qpc: u64,
    pub qpc_frequency: u64,
    pub native_nodes: BTreeMap<u64, [f32; 3]>,
}
impl Sink {
    pub fn new(
        recording: rr::RecordingStream,
        first_qpc: u64,
        qpc_frequency: u64,
        map: &MapContext,
    ) -> anyhow::Result<Self> {
        ensure!(qpc_frequency > 0, "QPC frequency is unknown");
        recording.set_log_time_enabled(false);
        recording.set_log_tick_enabled(false);
        recording.log_static("world", &rr::ViewCoordinates::RIGHT_HAND_Z_UP())?;
        map.log(&recording)?;
        Ok(Self {
            recording,
            first_qpc,
            qpc_frequency,
            native_nodes: map.node_lookup(),
        })
    }
    pub fn time(&self, qpc: u64, physics_frame: u64, sim_time_us: u64) {
        let elapsed = (qpc as i128 - self.first_qpc as i128) as f64 / self.qpc_frequency as f64;
        self.recording.set_duration_secs("elapsed", elapsed);
        self.recording
            .set_duration_secs("simulation", sim_time_us as f64 / 1e6);
        self.recording
            .set_time_sequence("physics_frame", physics_frame as i64);
    }
    pub fn telemetry(&self, snapshot: &GameSnapshot) -> anyhow::Result<()> {
        self.telemetry_at(snapshot, snapshot.raw.header.qpc)?;
        self.telemetry_series(snapshot)
    }
    /// Geometry presentation can follow capture exposure while the original
    /// telemetry remains on its separately logged physical-sample timeline.
    pub fn telemetry_at(
        &self,
        snapshot: &GameSnapshot,
        presentation_qpc: u64,
    ) -> anyhow::Result<()> {
        let raw = &snapshot.raw;
        let ego = &snapshot.ego;
        self.time(
            presentation_qpc,
            raw.header.frame_id,
            raw.paused_simulation_time_us,
        );
        let center = [ego.pose.x as f32, ego.pose.y as f32, ego.z as f32];
        self.recording.log(
            "world/ego/reference",
            &rr::Points3D::new([center])
                .with_colors([[255, 190, 40]])
                .with_radii([0.35])
                .with_labels(["SDK reference (not bbox)"]),
        )?;
        self.recording.log(
            "world/ego/heading",
            &rr::Arrows3D::from_vectors([[
                2.0 * ego.pose.yaw.cos() as f32,
                2.0 * ego.pose.yaw.sin() as f32,
                0.0,
            ]])
            .with_origins([center])
            .with_colors([[255, 190, 40]]),
        )?;
        Ok(())
    }
    pub fn telemetry_series(&self, snapshot: &GameSnapshot) -> anyhow::Result<()> {
        let raw = &snapshot.raw;
        let ego = &snapshot.ego;
        self.time(
            raw.header.qpc,
            raw.header.frame_id,
            raw.paused_simulation_time_us,
        );
        for (name, value) in [
            ("speed_mps", ego.speed),
            ("steer_wheel_rad", ego.wheel_angle),
            ("gear", raw.gear as f64),
            ("speed_limit_mps", raw.navigation_speed_limit_mps as f64),
            ("input/steer", raw.input[0] as f64),
            ("input/throttle", raw.input[1] as f64),
            ("input/brake", raw.input[2] as f64),
            ("input/clutch", raw.input[3] as f64),
            ("effective/steer", raw.effective[0] as f64),
            ("effective/throttle", raw.effective[1] as f64),
            ("effective/brake", raw.effective[2] as f64),
            ("effective/clutch", raw.effective[3] as f64),
        ] {
            if value.is_finite() {
                self.recording
                    .log(format!("telemetry/{name}"), &rr::Scalars::new([value]))?;
            }
        }
        Ok(())
    }
    pub fn command_delay(
        &self,
        observation: &crate::live_diagnostics::CommandDelay,
        frame_id: u64,
        paused_sim_time_us: Option<u64>,
    ) -> anyhow::Result<()> {
        self.time(
            observation.applied_qpc,
            frame_id,
            paused_sim_time_us.unwrap_or(0),
        );
        if paused_sim_time_us.is_none() {
            self.recording.disable_timeline("simulation");
        }
        self.recording.log(
            "control/delay_status",
            &rr::TextLog::new(serde_json::to_string(observation)?),
        )?;
        if let Some(delay) = observation.delay_ms {
            self.recording.log(
                "control/sdk_application_delay_ms",
                &rr::Scalars::new([delay]),
            )?;
        } else {
            self.recording
                .log("control/sdk_application_delay_ms", &rr::Clear::recursive())?;
        }
        Ok(())
    }
    pub fn scene(&self, snapshot: &GameSnapshot) -> anyhow::Result<()> {
        self.scene_at(snapshot, snapshot.raw.header.qpc)
    }
    pub fn scene_at(&self, snapshot: &GameSnapshot, presentation_qpc: u64) -> anyhow::Result<()> {
        let raw = &snapshot.raw;
        self.time(
            presentation_qpc,
            raw.header.frame_id,
            raw.paused_simulation_time_us,
        );
        let Some(extension) = &snapshot.extension else {
            self.recording
                .log("world/agents", &rr::Clear::recursive())?;
            self.recording
                .log("world/signals", &rr::Clear::recursive())?;
            self.recording.log("world/route", &rr::Clear::recursive())?;
            self.recording.log(
                "status/extension",
                &rr::TextLog::new("missing; scene cleared"),
            )?;
            return Ok(());
        };
        self.recording.log(
            "status/extension",
            &rr::TextLog::new(format!(
                "availability mask={}; physical_frame={}",
                extension.available, extension.header.frame_id
            )),
        )?;
        // Aggregate arrays replace every frame, so vanished IDs cannot persist.
        let agents = &snapshot.scene.agents;
        let strips = agents
            .iter()
            .map(|a| footprint(a.pose, a.z, a.length, a.width))
            .collect::<Vec<_>>();
        self.recording.log(
            "world/agents/footprints",
            &rr::LineStrips3D::new(strips)
                .with_colors([[255, 110, 65]])
                .with_radii([rr::Radius::new_ui_points(1.0)]),
        )?;
        self.recording.log(
            "world/agents/centers",
            &rr::Points3D::new(
                agents
                    .iter()
                    .map(|a| [a.pose.x as f32, a.pose.y as f32, a.z as f32]),
            )
            .with_labels(agents.iter().map(|a| a.id.to_string()))
            .with_show_labels(false)
            .with_colors([[255, 110, 65]])
            .with_radii([0.2]),
        )?;
        let bodies = native_boxes(extension);
        self.recording.log(
            "world/agents/native_boxes",
            &rr::LineStrips3D::new(bodies.iter().flat_map(|body| body.edges()))
                .with_colors([[255, 165, 80]])
                .with_radii([rr::Radius::new_ui_points(1.0)]),
        )?;
        self.recording.log(
            "world/agents/native_box_labels",
            &rr::Points3D::new(bodies.iter().map(|body| body.center))
                .with_labels(bodies.iter().map(|body| body.label.clone()))
                .with_show_labels(false)
                .with_radii([0.1]),
        )?;
        let mut signal_positions = Vec::new();
        let mut signal_labels = Vec::new();
        let mut signal_colors = Vec::new();
        if extension.available & 8 != 0 {
            for signal in ets2la::records::<ets2la::Signal>(&extension.signals) {
                let id = signal.id;
                let state = signal.state;
                let kind = signal.kind;
                if id < 0 || bytemuck::bytes_of(&signal).iter().all(|b| *b == 0) {
                    continue;
                }
                let world = ets2la::sector_world(signal.position, signal.sector);
                let enu = scs_to_enu(Point3 {
                    x: world[0],
                    y: world[1],
                    z: world[2],
                });
                if !enu.valid() {
                    continue;
                }
                signal_positions.push([enu.x as f32, enu.y as f32, enu.z as f32]);
                signal_labels.push(format!("id={id} kind={kind} raw={state}"));
                signal_colors.push(match (kind, state) {
                    (1, 8) => [50, 225, 90],
                    (1, 1 | 4) => [255, 200, 30],
                    (1, 2) => [255, 70, 70],
                    _ => [160, 160, 160],
                });
            }
        }
        self.recording.log(
            "world/signals",
            &rr::Points3D::new(signal_positions)
                .with_colors(signal_colors)
                .with_labels(signal_labels)
                .with_show_labels(false)
                .with_radii([0.4]),
        )?;
        let (segments, resolved) = route_segments(&snapshot.scene.route, &self.native_nodes);
        self.recording.log(
            "world/route",
            &rr::LineStrips3D::new(segments)
                .with_colors([[240, 80, 230]])
                .with_radii([0.25]),
        )?;
        self.recording.log("status/route", &rr::TextLog::new(format!("{} native UIDs, {resolved} known positions; unresolved nodes break geometry; route is navigation-node graph, not planned lane trajectory. UIDs={:?}",snapshot.scene.route.len(),snapshot.scene.route)))?;
        Ok(())
    }
}

struct NativeBox {
    label: String,
    center: [f32; 3],
    corners: [[f32; 3]; 8],
}
impl NativeBox {
    fn from_scs(
        label: String,
        position: [f32; 3],
        size: [f32; 3],
        quaternion: [f32; 4],
    ) -> Option<Self> {
        let corners =
            etsads_game::camera::bbox_corners_scs(position.map(f64::from), size, quaternion)?;
        let enu = |point: [f64; 3]| [point[0] as f32, -point[2] as f32, point[1] as f32];
        Some(Self {
            label,
            center: enu(position.map(f64::from)),
            corners: corners.map(enu),
        })
    }
    fn edges(&self) -> impl Iterator<Item = [[f32; 3]; 2]> + '_ {
        (0..8).flat_map(move |corner| {
            (0..3).filter_map(move |axis| {
                (corner & (1 << axis) == 0)
                    .then_some([self.corners[corner], self.corners[corner | (1 << axis)]])
            })
        })
    }
}
fn native_boxes(extension: &etsads_msgs::protocol::ExtensionFrame) -> Vec<NativeBox> {
    let mut boxes = Vec::new();
    if extension.available & 2 != 0 {
        for actor in ets2la::records::<ets2la::Traffic>(&extension.traffic) {
            let id = actor.id;
            let trailer_count = actor.trailer_count;
            if id < 0
                || !(0..=3).contains(&trailer_count)
                || bytemuck::bytes_of(&actor).iter().all(|byte| *byte == 0)
            {
                continue;
            }
            if let Some(body) = NativeBox::from_scs(
                format!("traffic/{id}"),
                actor.position,
                actor.size,
                actor.quaternion_wxyz,
            ) {
                boxes.push(body);
            }
            for (index, trailer) in actor
                .trailers
                .into_iter()
                .take(trailer_count as usize)
                .enumerate()
            {
                if let Some(body) = NativeBox::from_scs(
                    format!("traffic/{id}/trailer/{index}"),
                    trailer.position,
                    trailer.size,
                    trailer.quaternion_wxyz,
                ) {
                    boxes.push(body);
                }
            }
        }
    }
    if extension.available & 4 != 0 {
        for parked in ets2la::records::<ets2la::Parked>(&extension.parked) {
            let id = parked.id;
            if id < 0 || bytemuck::bytes_of(&parked).iter().all(|byte| *byte == 0) {
                continue;
            }
            if let Some(body) = NativeBox::from_scs(
                format!("parked/{id}"),
                parked.position,
                parked.size,
                parked.quaternion_wxyz,
            ) {
                boxes.push(body);
            }
        }
    }
    boxes
}

fn footprint(p: Pose2, z: f64, length: f64, width: f64) -> Vec<[f32; 3]> {
    [(-1., -1.), (-1., 1.), (1., 1.), (1., -1.), (-1., -1.)]
        .map(|(x, y)| {
            [
                (p.x + x * length / 2. * p.yaw.cos() - y * width / 2. * p.yaw.sin()) as f32,
                (p.y + x * length / 2. * p.yaw.sin() + y * width / 2. * p.yaw.cos()) as f32,
                z as f32,
            ]
        })
        .to_vec()
}
/// Unknown nodes break polylines; never join unrelated known points across a gap.
pub fn route_segments(ids: &[u64], nodes: &BTreeMap<u64, [f32; 3]>) -> (Vec<Vec<[f32; 3]>>, usize) {
    let mut segments = Vec::new();
    let mut current = Vec::new();
    let mut resolved = 0;
    for id in ids {
        if let Some(point) = nodes.get(id) {
            current.push(*point);
            resolved += 1;
        } else {
            if current.len() > 1 {
                segments.push(std::mem::take(&mut current));
            }
            current.clear();
        }
    }
    if current.len() > 1 {
        segments.push(current);
    }
    (segments, resolved)
}

#[derive(Clone, Serialize)]
pub struct CameraSample {
    pub physics_frame: u64,
    pub qpc: u64,
    pub sim_time_us: u64,
    pub position_scs: [f64; 3],
    pub quaternion_wxyz: [f32; 4],
    pub projection: [f32; 16],
    pub fov: f32,
}
impl CameraSample {
    pub fn new(header: etsads_msgs::protocol::WireHeader, camera: ets2la::Camera) -> Self {
        Self {
            physics_frame: header.frame_id,
            qpc: header.qpc,
            sim_time_us: header.sim_time_us,
            position_scs: ets2la::sector_world(camera.position, camera.sector),
            quaternion_wxyz: camera.quaternion_wxyz,
            projection: camera.projection,
            fov: camera.fov,
        }
    }
    /// C1 is a *requested* display convention, supported only by stationary
    /// interior direction evidence. It is not automatically a certified model.
    pub fn log(
        &self,
        rec: &rr::RecordingStream,
        resolution: Option<[u32; 2]>,
        enable_c1: bool,
    ) -> anyhow::Result<()> {
        rec.log(
            "status/camera",
            &rr::TextLog::new(serde_json::to_string(self)?),
        )?;
        if !enable_c1 {
            return Ok(());
        }
        let Some([width, height]) = resolution else {
            return Ok(());
        };
        ensure!(
            ets2la::valid_quaternion(self.quaternion_wxyz),
            "invalid camera quaternion"
        );
        let q = self.quaternion_wxyz;
        let rotation = glam::Quat::from_xyzw(q[1], q[2], q[3], q[0]).normalize();
        // Transform each original camera basis vector through the canonical
        // SCS->ENU function. M*R (not M*R*M^-1) retains RUB camera-local axes.
        let axes = [glam::Vec3::X, glam::Vec3::Y, glam::Vec3::Z].map(|v| {
            let v = rotation * v;
            let v = scs_to_enu(Point3 {
                x: v.x as f64,
                y: v.y as f64,
                z: v.z as f64,
            });
            glam::Vec3::new(v.x as f32, v.y as f32, v.z as f32)
        });
        let orientation = glam::Quat::from_mat3(&glam::Mat3::from_cols(axes[0], axes[1], axes[2]));
        let p = self.position_scs;
        let p = scs_to_enu(Point3 {
            x: p[0],
            y: p[1],
            z: p[2],
        });
        let projection = glam::Mat4::from_cols_array(&self.projection).transpose();
        let clip = projection * glam::Vec4::new(0., 0., -1., 0.);
        ensure!(
            clip.w.is_finite() && clip.w > 0.,
            "C1 projection has invalid homogeneous depth"
        );
        let fx = projection.x_axis.x / clip.w * width as f32 / 2.;
        let fy = projection.y_axis.y / clip.w * height as f32 / 2.;
        let cx = (clip.x / clip.w + 1.) * width as f32 / 2.;
        let cy = (1. - clip.y / clip.w) * height as f32 / 2.;
        ensure!(
            fx.is_finite() && fy.is_finite() && fx > 0. && fy > 0.,
            "invalid focal scale"
        );
        rec.log(
            "world/camera",
            &rr::Transform3D::from_translation_rotation(
                [p.x as f32, p.y as f32, p.z as f32],
                rr::Quaternion::from_xyzw(orientation.to_array()),
            ),
        )?;
        rec.log(
            "world/camera",
            &rr::Pinhole::new([[fx, 0., 0.], [0., fy, 0.], [cx, cy, 1.]])
                .with_resolution([width as f32, height as f32])
                .with_camera_xyz(rr::components::ViewCoordinates::RUB),
        )?;
        Ok(())
    }
}

/// Decode-order and edit-list handling follows the exact MP4 parser version
/// used by Rerun 0.38.1. Presentation order is sorted, as in re_video.
pub fn viewer_video_timestamps(path: &Path) -> anyhow::Result<Vec<i64>> {
    let mut file = std::fs::File::open(path)?;
    let size = file.metadata()?.len();
    let mp4 = re_mp4::Mp4::read(&mut file, size)?;
    let tracks = mp4
        .tracks()
        .values()
        .filter(|t| t.kind == Some(re_mp4::TrackKind::Video))
        .collect::<Vec<_>>();
    ensure!(
        tracks.len() == 1,
        "exactly one supported video track is required"
    );
    let track = tracks[0];
    ensure!(
        track.samples.len() <= 500_000,
        "MP4 sample table exceeds the diagnostic frame budget"
    );
    ensure!(track.timescale > 0, "MP4 timescale is zero");
    let mut pts = track
        .samples
        .iter()
        .map(|s| s.composition_timestamp)
        .collect::<Vec<_>>();
    pts.sort_unstable();
    ensure!(
        pts.first() == Some(&0) && pts.windows(2).all(|p| p[0] < p[1]),
        "MP4 presentation timestamps are ambiguous"
    );
    Ok(pts
        .into_iter()
        .map(|p| (p as f64 / track.timescale as f64 * 1e9).round() as i64)
        .collect())
}

/// Independent decoded-frame cross-check. Normalize the first presented frame
/// to zero, then compare *every* timestamp against the viewer's own parser.
#[derive(Deserialize)]
pub struct VideoProbe {
    pub streams: Vec<VideoStream>,
    pub frames: Vec<VideoPts>,
}
#[derive(Deserialize)]
pub struct VideoStream {
    pub width: u32,
    pub height: u32,
    pub time_base: String,
    pub codec_name: String,
}
#[derive(Clone, Deserialize)]
pub struct VideoPts {
    pub best_effort_timestamp: i64,
}
impl VideoProbe {
    pub fn timestamps_ns(&self) -> anyhow::Result<Vec<i64>> {
        let stream = self.streams.first().context("video stream missing")?;
        let (num, den) = stream
            .time_base
            .split_once('/')
            .context("invalid video timebase")?;
        let num: i128 = num.parse()?;
        let den: i128 = den.parse()?;
        ensure!(num > 0 && den > 0, "invalid video timebase");
        let first = self
            .frames
            .first()
            .context("no decoded video frames")?
            .best_effort_timestamp;
        let result = self
            .frames
            .iter()
            .map(|f| {
                let numerator = (f.best_effort_timestamp as i128 - first as i128)
                    .checked_mul(num)
                    .and_then(|v| v.checked_mul(1_000_000_000))
                    .context("PTS arithmetic overflow")?;
                ensure!(numerator >= 0, "negative PTS is unsupported");
                // Rerun 0.38.1 re_video::Time::into_nanos rounds to nearest.
                i64::try_from(
                    numerator
                        .checked_add(den / 2)
                        .context("PTS rounding overflow")?
                        / den,
                )
                .context("PTS overflow")
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        ensure!(
            result.windows(2).all(|p| p[0] < p[1]),
            "video PTS must be strictly increasing"
        );
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn native_volume_uses_each_body_height_center_and_full_rotation() {
        let mut extension = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        extension.available = 6;
        let mut actor: ets2la::Traffic = bytemuck::Zeroable::zeroed();
        actor.id = 42;
        actor.position = [0.0, 2.0, 0.0];
        actor.size = [2.0, 2.0, 6.0];
        actor.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        actor.trailer_count = 1;
        let mut trailers = actor.trailers;
        trailers[0] = ets2la::Trailer {
            position: [0.0, 8.0, 10.0],
            size: [2.0, 4.0, 8.0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        actor.trailers = trailers;
        extension.traffic[..std::mem::size_of_val(&actor)]
            .copy_from_slice(bytemuck::bytes_of(&actor));
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let parked = ets2la::Parked {
            position: [20.0, 5.0, 0.0],
            size: [2.0, 3.0, 4.0],
            quaternion_wxyz: [s, 0.0, 0.0, s],
            id: 7,
            is_trailer: 0,
        };
        extension.parked[..std::mem::size_of_val(&parked)]
            .copy_from_slice(bytemuck::bytes_of(&parked));
        let boxes = native_boxes(&extension);
        assert_eq!(boxes.len(), 3);
        let trailer = boxes
            .iter()
            .find(|body| body.label == "traffic/42/trailer/0")
            .unwrap();
        assert_eq!(trailer.center, [0.0, -10.0, 8.0]);
        assert_eq!(trailer.edges().count(), 12);
        let extent = |body: &NativeBox| {
            body.corners
                .iter()
                .map(|p| p[2])
                .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), z| {
                    (min.min(z), max.max(z))
                })
        };
        assert_eq!(extent(trailer), (6.0, 10.0));
        let rotated = boxes.iter().find(|body| body.label == "parked/7").unwrap();
        let (min, max) = extent(rotated);
        assert!((min - 4.0).abs() < 1e-5 && (max - 6.0).abs() < 1e-5);
    }
    #[test]
    fn missing_route_node_never_creates_a_shortcut() {
        let nodes = BTreeMap::from([
            (1, [1., 0., 0.]),
            (2, [2., 0., 0.]),
            (4, [4., 0., 0.]),
            (5, [5., 0., 0.]),
        ]);
        let (segments, resolved) = route_segments(&[1, 2, 3, 4, 5], &nodes);
        assert_eq!(resolved, 4);
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0].len(), 2);
    }
    #[test]
    fn rational_video_pts_preserves_variable_frame_cadence() {
        let p = VideoProbe {
            streams: vec![VideoStream {
                width: 1,
                height: 1,
                time_base: "1/90000".into(),
                codec_name: "h264".into(),
            }],
            frames: [4500, 6000, 9000]
                .map(|v| VideoPts {
                    best_effort_timestamp: v,
                })
                .to_vec(),
        };
        assert_eq!(p.timestamps_ns().unwrap(), vec![0, 16_666_667, 50_000_000]);
    }
}
