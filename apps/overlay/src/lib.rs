//! Uncalibrated diagnostic rectangles only. This crate has no driving-input
//! client. Its sole published game mapping is the existing OverlayFrame ABI.
use anyhow::{Context, ensure};
use etsads_game::{
    camera::{Convention, bbox_corners_scs, project_bbox},
    ets2la::{self, Camera, Parked, Traffic},
};
use etsads_msgs::protocol::{
    BridgeInfo, ExtensionFrame, OVERLAY_MAPPING, OverlayFrame, OverlayRect, SCHEMA_VERSION,
    TelemetryFrame,
};
use etsads_shm::{SharedCell, SingleWriterGuard};
use serde::Serialize;

#[derive(Clone, Copy, Debug, Serialize)]
pub struct ProjectionOptions {
    pub hypothesis: u8,
    pub maximum_distance_m: f64,
    pub maximum_boxes: usize,
    pub line_width_px: f32,
}
impl ProjectionOptions {
    pub fn validate(self) -> anyhow::Result<Self> {
        ensure!(
            self.hypothesis < 8,
            "hypothesis must be explicitly chosen in 0..7"
        );
        ensure!(
            self.maximum_distance_m.is_finite() && (1.0..=500.0).contains(&self.maximum_distance_m),
            "maximum distance must be 1..500 metres"
        );
        ensure!(
            (1..=64).contains(&self.maximum_boxes),
            "maximum boxes must be 1..64"
        );
        ensure!(
            self.line_width_px.is_finite() && (1.0..=8.0).contains(&self.line_width_px),
            "line width must be 1..8 pixels"
        );
        Ok(self)
    }
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct CaptureStamp {
    pub width: u32,
    pub height: u32,
    pub frame_index: u64,
    pub timestamp_qpc: u64,
    pub publish_qpc: Option<u64>,
    pub generation: Option<u32>,
}
impl CaptureStamp {
    pub fn valid(self) -> bool {
        (160..=16384).contains(&self.width)
            && (80..=16384).contains(&self.height)
            && self.frame_index > 0
            && self.timestamp_qpc > 0
    }
}
impl From<etsads_game::capture::FrameMetadata> for CaptureStamp {
    fn from(m: etsads_game::capture::FrameMetadata) -> Self {
        Self {
            width: m.width,
            height: m.height,
            frame_index: m.frame_index,
            timestamp_qpc: m.timestamp_qpc,
            publish_qpc: Some(m.publish_qpc),
            generation: Some(m.generation),
        }
    }
}
#[derive(Clone, Serialize)]
pub struct CameraEvidence {
    pub fov: f32,
    pub position_scs: [f64; 3],
    pub quaternion_wxyz: [f32; 4],
    pub projection: [f32; 16],
}
impl From<Camera> for CameraEvidence {
    fn from(c: Camera) -> Self {
        Self {
            fov: c.fov,
            position_scs: ets2la::sector_world(c.position, c.sector),
            quaternion_wxyz: c.quaternion_wxyz,
            projection: c.projection,
        }
    }
}
#[derive(Clone, Serialize)]
pub struct ActorEvidence {
    pub source_id: String,
    pub center_scs: [f64; 3],
    pub size: [f32; 3],
    pub quaternion_wxyz: [f32; 4],
    pub camera_distance_m: f64,
    pub screen_bounds: [f32; 4],
    pub visibility_verified: bool,
}
#[derive(Serialize)]
pub struct ProjectionEvidence {
    pub fully_calibrated: bool,
    pub hypothesis: u8,
    pub transpose: bool,
    pub quaternion_world_to_camera: bool,
    pub positive_z_forward: bool,
    pub source_frame_id: u64,
    pub telemetry_qpc: u64,
    pub extension_qpc: Option<u64>,
    pub capture: CaptureStamp,
    pub write_qpc: u64,
    pub enabled: bool,
    pub reason: String,
    pub label: String,
    pub rectangle_count: u32,
    pub projected_actors: Vec<ActorEvidence>,
    pub invalid_actors: usize,
    pub rejected_projection_or_distance: usize,
    pub omitted_capacity: usize,
    pub camera: Option<CameraEvidence>,
    /// Live capture metadata describes an older GPU readback, not the image
    /// onto which the next Present will draw this overlay.
    pub timing_context: TimingContext,
    pub live_timing: Option<LiveTiming>,
}
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TimingContext {
    GeometryOnly,
    ReplayImageAligned,
    LiveLatestForPresentation,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct LiveTiming {
    pub telemetry_age_ms: f64,
    pub camera_age_ms: f64,
    pub metadata_publication_age_ms: f64,
    pub readback_delay_ms: f64,
    pub captured_image_age_ms: f64,
    pub camera_minus_captured_image_ms: f64,
    pub capture_is_image_match: bool,
}
/// Replay aligns a recorded image with its historical camera. Live presentation
/// deliberately does not use this rule: GPU readback metadata arrives later.
pub fn replay_image_aligned(camera_qpc: u64, capture_qpc: u64, frequency: u64) -> bool {
    frequency != 0
        && camera_qpc != 0
        && capture_qpc != 0
        && camera_qpc.abs_diff(capture_qpc) as u128 * 1000 <= frequency as u128 * 16
}
pub struct ProjectionOutput {
    pub frame: Box<OverlayFrame>,
    pub evidence: ProjectionEvidence,
}
pub fn disabled(now: u64, source_frame_id: u64) -> Box<OverlayFrame> {
    let mut frame = etsads_shm::zeroed_box::<OverlayFrame>();
    frame.write_qpc = now;
    frame.source_frame_id = source_frame_id;
    frame
}
fn raw_actor(
    id: String,
    position: [f32; 3],
    size: [f32; 3],
    q: [f32; 4],
    camera: Camera,
    stamp: CaptureStamp,
    options: ProjectionOptions,
) -> Option<ActorEvidence> {
    let center = position.map(|v| v as f64);
    let corners = bbox_corners_scs(center, size, q)?;
    let origin = ets2la::sector_world(camera.position, camera.sector);
    let distance = center
        .iter()
        .zip(origin)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt();
    if distance > options.maximum_distance_m {
        return None;
    }
    let screen = project_bbox(
        camera,
        corners,
        stamp.width,
        stamp.height,
        Convention::candidates()[options.hypothesis as usize],
    )?;
    Some(ActorEvidence {
        source_id: id,
        center_scs: center,
        size,
        quaternion_wxyz: q,
        camera_distance_m: distance,
        screen_bounds: screen,
        visibility_verified: false,
    })
}
/// Pure geometric producer, shared by recorded and live inputs. Native traffic
/// positions are centers; full quaternions are retained. Unknown occlusion and
/// the chosen projection convention remain explicit in every evidence record.
pub fn generate(
    raw: &TelemetryFrame,
    extension: Option<&ExtensionFrame>,
    stamp: CaptureStamp,
    options: ProjectionOptions,
    now: u64,
) -> anyhow::Result<ProjectionOutput> {
    let options = options.validate()?;
    let convention = Convention::candidates()[options.hypothesis as usize];
    let mut out = ProjectionOutput {
        frame: disabled(now, raw.header.frame_id),
        evidence: ProjectionEvidence {
            fully_calibrated: false,
            hypothesis: options.hypothesis,
            transpose: convention.transpose,
            quaternion_world_to_camera: convention.quaternion_world_to_camera,
            positive_z_forward: convention.positive_z_forward,
            source_frame_id: raw.header.frame_id,
            telemetry_qpc: raw.header.qpc,
            extension_qpc: extension.map(|e| e.header.qpc),
            capture: stamp,
            write_qpc: now,
            enabled: false,
            reason: String::new(),
            label: format!("UNCAL C{}", options.hypothesis),
            rectangle_count: 0,
            projected_actors: Vec::new(),
            invalid_actors: 0,
            rejected_projection_or_distance: 0,
            omitted_capacity: 0,
            camera: None,
            timing_context: TimingContext::GeometryOnly,
            live_timing: None,
        },
    };
    let failure = if !stamp.valid() {
        Some("invalid_capture_dimensions_or_timing")
    } else if raw.paused != 0 {
        Some("game_paused")
    } else if extension.is_none() {
        Some("extension_missing")
    } else {
        None
    };
    if let Some(reason) = failure {
        out.evidence.reason = reason.into();
        return Ok(out);
    }
    let ext = extension.unwrap();
    if ext.header.frame_id != raw.header.frame_id
        || ext.header.sim_time_us != raw.header.sim_time_us
    {
        out.evidence.reason = "extension_frame_mismatch".into();
        return Ok(out);
    }
    if ext.available & 1 == 0 {
        out.evidence.reason = "camera_unavailable".into();
        return Ok(out);
    }
    let camera = bytemuck::pod_read_unaligned::<Camera>(&ext.camera);
    let ce = CameraEvidence::from(camera);
    if !ets2la::valid_quaternion(ce.quaternion_wxyz)
        || !ce
            .position_scs
            .iter()
            .all(|v| v.is_finite() && v.abs() < 1e7)
        || !ce.projection.iter().all(|v| v.is_finite())
        || ce.projection.iter().all(|v| *v == 0.0)
        || !ce.fov.is_finite()
        || ce.fov <= 0.0
        || ce.fov >= 180.0
    {
        out.evidence.reason = "invalid_camera_fields".into();
        return Ok(out);
    }
    out.evidence.camera = Some(ce);
    let mut actors = Vec::new();
    let mut add = |id: String, position: [f32; 3], size: [f32; 3], q: [f32; 4]| {
        if bbox_corners_scs(position.map(|v| v as f64), size, q).is_none() {
            out.evidence.invalid_actors += 1;
            return;
        }
        if let Some(actor) = raw_actor(id, position, size, q, camera, stamp, options) {
            actors.push(actor);
        } else {
            out.evidence.rejected_projection_or_distance += 1;
        }
    };
    if ext.available & 2 != 0 {
        for item in ets2la::records::<Traffic>(&ext.traffic) {
            let id = item.id;
            if id < 0 || bytemuck::bytes_of(&item).iter().all(|b| *b == 0) {
                continue;
            }
            add(
                format!("traffic:{id}"),
                item.position,
                item.size,
                item.quaternion_wxyz,
            );
            // These trailers carry real native center, size (including height),
            // and quaternion. SDK trailer placements without size are not used.
            let trailer_count = item.trailer_count;
            if (0..=3).contains(&trailer_count) {
                for (index, t) in item
                    .trailers
                    .into_iter()
                    .take(trailer_count as usize)
                    .enumerate()
                {
                    add(
                        format!("traffic:{id}:trailer:{index}"),
                        t.position,
                        t.size,
                        t.quaternion_wxyz,
                    );
                }
            }
        }
    }
    if ext.available & 4 != 0 {
        for item in ets2la::records::<Parked>(&ext.parked) {
            let id = item.id;
            if id < 0 || bytemuck::bytes_of(&item).iter().all(|b| *b == 0) {
                continue;
            }
            add(
                format!("parked:{id}"),
                item.position,
                item.size,
                item.quaternion_wxyz,
            );
        }
    }
    actors.sort_by(|a, b| {
        a.camera_distance_m
            .total_cmp(&b.camera_distance_m)
            .then(a.source_id.cmp(&b.source_id))
    });
    let label = label_rectangles(&out.evidence.label, stamp.width, stamp.height);
    let capacity = ((256 - label.len()) / 4).min(options.maximum_boxes);
    let mut rectangles = Vec::new();
    out.evidence.omitted_capacity = actors.len().saturating_sub(capacity);
    actors.truncate(capacity);
    for actor in &actors {
        let [l, t, r, b] = actor.screen_bounds;
        let w = options.line_width_px;
        let color = if actor.source_id.starts_with("parked:") {
            [0.8, 0.3, 1.0, 1.0]
        } else {
            [0.0, 0.9, 1.0, 1.0]
        };
        for bounds in [
            [l, t, r, (t + w).min(b)],
            [l, (b - w).max(t), r, b],
            [l, t, (l + w).min(r), b],
            [(r - w).max(l), t, r, b],
        ] {
            rectangles.push(OverlayRect {
                bounds,
                color_rgba: color,
            });
        }
    }
    // ClearView is ordered and opaque: draw the badge last so even a large,
    // incorrect hypothesis rectangle cannot cover the uncalibrated label.
    rectangles.extend(label);
    out.frame.enabled = 1;
    out.frame.count = rectangles.len() as u32;
    out.frame.rectangles[..rectangles.len()].copy_from_slice(&rectangles);
    out.evidence.enabled = true;
    out.evidence.rectangle_count = out.frame.count;
    out.evidence.reason = "uncalibrated_diagnostic_occlusion_unknown".into();
    out.evidence.projected_actors = actors;
    Ok(out)
}

fn glyph(c: char) -> [u8; 7] {
    match c {
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'N' => [17, 25, 25, 21, 19, 19, 17],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        _ => [0; 7],
    }
}
fn label_rectangles(text: &str, width: u32, height: u32) -> Vec<OverlayRect> {
    let scale = 3.0_f32;
    let x = 10.;
    let y = 10.;
    let mut rects = vec![OverlayRect {
        bounds: [
            x - 4.,
            y - 4.,
            (x + text.len() as f32 * 6. * scale + 3.).min(width as f32),
            (y + 7. * scale + 4.).min(height as f32),
        ],
        color_rgba: [0.08, 0.05, 0.0, 1.0],
    }];
    for (index, c) in text.chars().enumerate() {
        for (row, bits) in glyph(c).into_iter().enumerate() {
            let mut col = 0;
            while col < 5 {
                if bits & (1 << (4 - col)) == 0 {
                    col += 1;
                    continue;
                }
                let first = col;
                while col < 5 && bits & (1 << (4 - col)) != 0 {
                    col += 1;
                }
                let left = x + (index * 6 + first) as f32 * scale;
                let top = y + row as f32 * scale;
                let right = (x + (index * 6 + col) as f32 * scale).min(width as f32);
                let bottom = (top + scale).min(height as f32);
                if left < right && top < bottom {
                    rects.push(OverlayRect {
                        bounds: [left, top, right, bottom],
                        color_rgba: [1.0, 0.65, 0.05, 1.0],
                    });
                }
            }
        }
    }
    rects
}

#[derive(Default)]
pub struct FreshnessGate {
    session: Option<u64>,
    viewport: Option<(u32, u32, u32)>,
    camera_epoch_qpc: u64,
}
impl FreshnessGate {
    pub fn reset(&mut self) {
        self.session = None;
        self.viewport = None;
        self.camera_epoch_qpc = 0;
    }
    pub fn check(
        &mut self,
        raw: &TelemetryFrame,
        ext: Option<&ExtensionFrame>,
        capture: CaptureStamp,
        bridge: Option<&BridgeInfo>,
        now: u64,
        frequency: u64,
    ) -> Result<LiveTiming, &'static str> {
        let Some(info) = bridge else {
            return Err("bridge_info_missing");
        };
        if info.schema_version != SCHEMA_VERSION
            || info.telemetry_api_version == 0
            || info.session_start_qpc == 0
        {
            return Err("bridge_info_invalid");
        }
        if self.session != Some(info.session_start_qpc) {
            self.session = Some(info.session_start_qpc);
            self.viewport = None;
            self.camera_epoch_qpc = 0;
            return Err("session_changed_clear");
        }
        let Some(generation) = capture.generation else {
            return Err("capture_generation_unknown");
        };
        let viewport = (capture.width, capture.height, generation);
        if self.viewport != Some(viewport) {
            self.viewport = Some(viewport);
            self.camera_epoch_qpc = capture.publish_qpc.unwrap_or(now);
            return Err("viewport_changed_clear");
        }
        if !capture.valid() || frequency == 0 || raw.qpc_frequency != frequency {
            return Err("clock_or_dimensions_invalid");
        }
        if raw.paused != 0 {
            return Err("game_paused");
        }
        let Some(ext) = ext else {
            return Err("extension_missing");
        };
        if ext.header.frame_id != raw.header.frame_id
            || ext.header.sim_time_us != raw.header.sim_time_us
        {
            return Err("extension_frame_mismatch");
        }
        let publish_qpc = capture.publish_qpc.ok_or("capture_publish_time_unknown")?;
        // Source freshness and producer activity are independent clocks. The
        // captured image can be several staging slots older than publication.
        for stamp in [raw.header.qpc, ext.header.qpc, publish_qpc] {
            if stamp == 0 || stamp > now || stamp < info.session_start_qpc {
                return Err("future_or_previous_session_sample");
            }
            if (now - stamp) as u128 * 1000 > frequency as u128 * 100 {
                return Err("sample_older_than_100ms");
            }
        }
        if capture.timestamp_qpc < info.session_start_qpc {
            return Err("capture_from_previous_session");
        }
        if publish_qpc < capture.timestamp_qpc {
            return Err("capture_publish_before_exposure");
        }
        if ext.header.qpc < self.camera_epoch_qpc {
            return Err("camera_predates_viewport_publication");
        }
        let ms = |ticks: u64| ticks as f64 * 1000.0 / frequency as f64;
        Ok(LiveTiming {
            telemetry_age_ms: ms(now - raw.header.qpc),
            camera_age_ms: ms(now - ext.header.qpc),
            metadata_publication_age_ms: ms(now - publish_qpc),
            readback_delay_ms: ms(publish_qpc - capture.timestamp_qpc),
            captured_image_age_ms: ms(now - capture.timestamp_qpc),
            camera_minus_captured_image_ms: (ext.header.qpc as i128 - capture.timestamp_qpc as i128)
                as f64
                * 1000.0
                / frequency as f64,
            capture_is_image_match: false,
        })
    }
}

pub struct DiagnosticPublisher {
    mapping: SharedCell<OverlayFrame>,
    _guard: SingleWriterGuard,
}
impl DiagnosticPublisher {
    pub fn acquire_live() -> anyhow::Result<Self> {
        Self::acquire(OVERLAY_MAPPING, "ETSADS_Overlay_Writer")
    }
    fn acquire(mapping: &str, guard: &str) -> anyhow::Result<Self> {
        let lease = SingleWriterGuard::acquire(guard)
            .context("another diagnostic overlay publisher owns the writer lease")?;
        let this = Self {
            mapping: SharedCell::create(mapping, SCHEMA_VERSION)?,
            _guard: lease,
        };
        this.clear()?;
        Ok(this)
    }
    pub fn publish(&self, frame: &OverlayFrame) -> anyhow::Result<()> {
        ensure!(frame.count <= 256, "overlay rectangle budget exceeded");
        self.mapping.publish(frame)?;
        Ok(())
    }
    pub fn clear(&self) -> anyhow::Result<()> {
        self.publish(&disabled(etsads_shm::qpc(), 0))
    }
}
impl Drop for DiagnosticPublisher {
    fn drop(&mut self) {
        let _ = self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bytemuck::Zeroable;
    fn fixture() -> (
        TelemetryFrame,
        Box<ExtensionFrame>,
        CaptureStamp,
        ProjectionOptions,
    ) {
        let mut raw = TelemetryFrame {
            qpc_frequency: 1000,
            ..Default::default()
        };
        raw.header.frame_id = 4;
        raw.header.qpc = 1000;
        let mut ext = etsads_shm::zeroed_box::<ExtensionFrame>();
        ext.header = raw.header;
        ext.available = 7;
        let mut camera = Camera::zeroed();
        camera.fov = 80.;
        camera.quaternion_wxyz = [1., 0., 0., 0.];
        camera.projection =
            glam::Mat4::perspective_rh_gl(1.0, 16. / 9., 0.1, 1000.).to_cols_array();
        ext.camera.copy_from_slice(bytemuck::bytes_of(&camera));
        let stamp = CaptureStamp {
            width: 1920,
            height: 1080,
            frame_index: 1,
            timestamp_qpc: 995,
            publish_qpc: Some(999),
            generation: Some(1),
        };
        (
            raw,
            ext,
            stamp,
            ProjectionOptions {
                hypothesis: 0,
                maximum_distance_m: 200.,
                maximum_boxes: 64,
                line_width_px: 2.,
            },
        )
    }
    #[test]
    fn annotation_is_always_present_and_capacity_is_bounded_near_first() {
        let (raw, mut ext, stamp, options) = fixture();
        for i in 0..40 {
            let actor = Parked {
                position: [0., 0., -20. - i as f32],
                quaternion_wxyz: [1., 0., 0., 0.],
                size: [2., 3., 4.],
                id: i as i16,
                is_trailer: 0,
            };
            ext.parked[i * 43..i * 43 + 43].copy_from_slice(bytemuck::bytes_of(&actor));
            let mut moving = Traffic::zeroed();
            moving.id = i as i16;
            moving.position = actor.position;
            moving.size = actor.size;
            moving.quaternion_wxyz = actor.quaternion_wxyz;
            ext.traffic[i * 174..i * 174 + 174].copy_from_slice(bytemuck::bytes_of(&moving));
        }
        let out = generate(&raw, Some(&ext), stamp, options, 1000).unwrap();
        assert_eq!(out.frame.enabled, 1);
        assert!(!out.evidence.fully_calibrated);
        assert_eq!(out.evidence.label, "UNCAL C0");
        assert!(out.frame.count <= 256);
        assert!(out.evidence.omitted_capacity > 0);
        assert!(!out.evidence.projected_actors.is_empty());
        assert_eq!(out.evidence.projected_actors[0].source_id, "parked:0");
        assert!(
            out.evidence
                .projected_actors
                .windows(2)
                .all(|a| a[0].camera_distance_m <= a[1].camera_distance_m)
        );
        assert!(
            out.frame.rectangles[..out.frame.count as usize]
                .iter()
                .all(|r| r.bounds.iter().all(|v| v.is_finite())
                    && r.bounds[0] >= 0.
                    && r.bounds[2] <= 1920.
                    && r.bounds[1] >= 0.
                    && r.bounds[3] <= 1080.)
        );
    }
    #[test]
    fn near_crossings_and_invalid_native_height_do_not_invent_boxes() {
        let (raw, mut ext, stamp, options) = fixture();
        for (i, (z, height)) in [(-1., 3.), (-20., 0.)].into_iter().enumerate() {
            let actor = Parked {
                position: [0., 0., z],
                quaternion_wxyz: [1., 0., 0., 0.],
                size: [2., height, 4.],
                id: i as i16,
                is_trailer: 0,
            };
            ext.parked[i * 43..i * 43 + 43].copy_from_slice(bytemuck::bytes_of(&actor));
        }
        let out = generate(&raw, Some(&ext), stamp, options, 1000).unwrap();
        assert!(out.evidence.projected_actors.is_empty());
        assert_eq!(out.evidence.invalid_actors, 1);
    }
    #[test]
    fn pause_clock_resize_session_and_readback_timing() {
        let (mut raw, ext, mut stamp, _) = fixture();
        let mut gate = FreshnessGate::default();
        let mut bridge = BridgeInfo {
            schema_version: SCHEMA_VERSION,
            session_start_qpc: 900,
            telemetry_api_version: 1,
            ..Default::default()
        };
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000),
            Err("session_changed_clear")
        );
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000),
            Err("viewport_changed_clear")
        );
        assert!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000)
                .is_ok()
        );
        raw.paused = 1;
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000),
            Err("game_paused")
        );
        raw.paused = 0;
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1101, 1000),
            Err("sample_older_than_100ms")
        );
        stamp.width = 1280;
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000),
            Err("viewport_changed_clear")
        );
        bridge.session_start_qpc = 950;
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000),
            Err("session_changed_clear")
        );
        gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000)
            .unwrap_err();
        stamp.timestamp_qpc = 970;
        let timing = gate
            .check(&raw, Some(&ext), stamp, Some(&bridge), 1000, 1000)
            .unwrap();
        assert_eq!(timing.camera_minus_captured_image_ms, 30.0);
        assert!(!timing.capture_is_image_match);
    }
    #[test]
    fn three_staging_slots_do_not_force_a_historical_live_camera() {
        let (raw, ext, mut stamp, options) = fixture();
        let bridge = BridgeInfo {
            schema_version: SCHEMA_VERSION,
            session_start_qpc: 500,
            telemetry_api_version: 1,
            ..Default::default()
        };
        // Three slots at 60 Hz ~=50 ms, or at 20 Hz =150 ms. Source camera
        // and metadata publication are still fresh in both cases.
        for delay in [50, 150] {
            let mut gate = FreshnessGate::default();
            stamp.timestamp_qpc = 1000 - delay;
            stamp.publish_qpc = Some(1000);
            assert_eq!(
                gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000),
                Err("session_changed_clear")
            );
            assert_eq!(
                gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000),
                Err("viewport_changed_clear")
            );
            let timing = gate
                .check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000)
                .unwrap();
            assert_eq!(timing.camera_age_ms, 2.0);
            assert_eq!(timing.metadata_publication_age_ms, 2.0);
            assert_eq!(timing.readback_delay_ms, delay as f64);
            assert_eq!(timing.camera_minus_captured_image_ms, delay as f64);
            assert!(!timing.capture_is_image_match);
            assert!(!replay_image_aligned(
                ext.header.qpc,
                stamp.timestamp_qpc,
                1000
            ));
            let output = generate(&raw, Some(&ext), stamp, options, 1002).unwrap();
            assert_eq!(output.frame.source_frame_id, raw.header.frame_id);
            assert_eq!(output.evidence.extension_qpc, Some(1000));
        }
    }
    #[test]
    fn delayed_metadata_resize_requires_a_new_camera_and_staleness_still_clears() {
        let (mut raw, mut ext, mut stamp, _) = fixture();
        let bridge = BridgeInfo {
            schema_version: SCHEMA_VERSION,
            session_start_qpc: 500,
            telemetry_api_version: 1,
            ..Default::default()
        };
        let mut gate = FreshnessGate::default();
        stamp.timestamp_qpc = 950;
        stamp.publish_qpc = Some(1000);
        gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000)
            .unwrap_err();
        gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000)
            .unwrap_err();
        assert!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1002, 1000)
                .is_ok()
        );
        stamp.width = 1280;
        stamp.generation = Some(2);
        stamp.timestamp_qpc = 953;
        stamp.publish_qpc = Some(1003);
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1004, 1000),
            Err("viewport_changed_clear")
        );
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1004, 1000),
            Err("camera_predates_viewport_publication")
        );
        raw.header.qpc = 1004;
        raw.header.frame_id += 1;
        ext.header = raw.header;
        assert!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1005, 1000)
                .is_ok()
        );
        // Latest camera is exactly 100 ms old, but producer publication is 101 ms old.
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1104, 1000),
            Err("sample_older_than_100ms")
        );
        raw.header.qpc = 904;
        ext.header = raw.header;
        assert_eq!(
            gate.check(&raw, Some(&ext), stamp, Some(&bridge), 1005, 1000),
            Err("sample_older_than_100ms")
        );
    }
    #[test]
    fn replay_image_alignment_keeps_the_16ms_boundary() {
        let capture = 1_000_000_u64;
        let frequency = 10_000_000;
        assert!(replay_image_aligned(capture + 160_000, capture, frequency));
        assert!(!replay_image_aligned(capture + 160_001, capture, frequency));
        assert!(!replay_image_aligned(capture + 500_000, capture, frequency));
        assert!(!replay_image_aligned(capture, capture, 0));
    }
    #[cfg(windows)]
    #[test]
    fn isolated_mapping_single_writer_and_drop_clear() {
        let prefix = format!(
            "ETSADS_Overlay_Test_{}_{}",
            std::process::id(),
            etsads_shm::qpc()
        );
        let guard = format!("{prefix}_Writer");
        let publisher = DiagnosticPublisher::acquire(&prefix, &guard).unwrap();
        let reader = SharedCell::<OverlayFrame>::open(&prefix, SCHEMA_VERSION).unwrap();
        assert!(DiagnosticPublisher::acquire(&prefix, &guard).is_err());
        let (raw, ext, stamp, options) = fixture();
        let output = generate(&raw, Some(&ext), stamp, options, 1000).unwrap();
        publisher.publish(&output.frame).unwrap();
        assert_eq!(reader.latest().unwrap().enabled, 1);
        drop(publisher);
        let cleared = reader.latest().unwrap();
        assert_eq!(cleared.enabled, 0);
        assert_eq!(cleared.count, 0);
    }
    #[cfg(windows)]
    #[test]
    fn separate_process_observes_overlay_and_cannot_steal_writer() {
        let prefix = format!(
            "ETSADS_Overlay_IPC_{}_{}",
            std::process::id(),
            etsads_shm::qpc()
        );
        let publisher = DiagnosticPublisher::acquire(&prefix, &format!("{prefix}_Writer")).unwrap();
        let (raw, ext, stamp, options) = fixture();
        publisher
            .publish(
                &generate(&raw, Some(&ext), stamp, options, 1000)
                    .unwrap()
                    .frame,
            )
            .unwrap();
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "tests::overlay_ipc_reader", "--ignored"])
            .env("ETSADS_OVERLAY_TEST_MAPPING", &prefix)
            .spawn()
            .unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        let status = loop {
            if let Some(status) = child.try_wait().unwrap() {
                break status;
            }
            if std::time::Instant::now() > deadline {
                child.kill().unwrap();
                panic!("isolated reader timed out");
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        };
        assert!(status.success());
        drop(publisher);
    }
    #[cfg(windows)]
    #[test]
    #[ignore = "only spawned by isolated cross-process test"]
    fn overlay_ipc_reader() {
        let prefix = std::env::var("ETSADS_OVERLAY_TEST_MAPPING").expect("helper-only invocation");
        assert!(prefix.starts_with("ETSADS_Overlay_IPC_"));
        assert!(DiagnosticPublisher::acquire(&prefix, &format!("{prefix}_Writer")).is_err());
        let reader = SharedCell::<OverlayFrame>::open(&prefix, SCHEMA_VERSION).unwrap();
        let frame = reader.latest().unwrap();
        assert_eq!(frame.source_frame_id, 4);
        assert_eq!(frame.enabled, 1);
        assert!(frame.count > 0 && frame.count <= 256);
    }
}
