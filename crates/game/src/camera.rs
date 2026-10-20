//! Projection hypotheses are explicit until calibrated against recorded game
//! images. A caller must choose a convention; no unverified default is hidden.
use crate::ets2la::{sector_world, valid_quaternion, Camera};
use etsads_msgs::protocol::*;
use glam::{DQuat, DVec3};
use glam::{Mat4, Quat, Vec3, Vec4};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Convention {
    pub transpose: bool,
    pub quaternion_world_to_camera: bool,
    pub positive_z_forward: bool,
}
impl Convention {
    /// C1 supported by three visually reviewed cam8 traffic observations from
    /// ETS2 1.61.1.9 with the plugin hashes pinned in the exterior fixture.
    ///
    /// This is an explicit convention, not a default or a fully-calibrated
    /// camera capability. It does not certify other views, vehicles, patches,
    /// pixel accuracy, occlusion, or any driving permission. See the persistent
    /// `tests/reviewed_projection.rs` fixture and its provenance notes.
    pub const ETS2LA_1_61_REVIEWED_EXTERIOR: Self = Self {
        transpose: true,
        quaternion_world_to_camera: false,
        positive_z_forward: false,
    };

    pub fn candidates() -> [Self; 8] {
        std::array::from_fn(|i| Self {
            transpose: i & 1 != 0,
            quaternion_world_to_camera: i & 2 != 0,
            positive_z_forward: i & 4 != 0,
        })
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewKind {
    /// Legacy meaning, retained for consumers; `classify` never emits it.
    Interior,
    Chase,
    Other,
    NearCabinUnclassified,
}
pub fn project(
    camera: Camera,
    world_scs: [f64; 3],
    width: u32,
    height: u32,
    convention: Convention,
) -> Option<[f32; 2]> {
    if width == 0 || height == 0 || !valid_quaternion(camera.quaternion_wxyz) {
        return None;
    }
    let origin = sector_world(camera.position, camera.sector);
    let delta = Vec3::new(
        (world_scs[0] - origin[0]) as f32,
        (world_scs[1] - origin[1]) as f32,
        (world_scs[2] - origin[2]) as f32,
    );
    let q = camera.quaternion_wxyz;
    let rotation = Quat::from_xyzw(q[1], q[2], q[3], q[0]).normalize();
    let mut local = if convention.quaternion_world_to_camera {
        rotation * delta
    } else {
        rotation.conjugate() * delta
    };
    if convention.positive_z_forward {
        local.z = -local.z
    }
    if !local.is_finite() || local.z >= -0.001 {
        return None;
    }
    let coefficients = camera.projection;
    let mut projection = Mat4::from_cols_array(&coefficients);
    if convention.transpose {
        projection = projection.transpose()
    }
    let clip = projection * Vec4::new(local.x, local.y, local.z, 1.0);
    if !clip.is_finite() || clip.w <= 0.0 {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    Some([
        (ndc.x + 1.0) * 0.5 * width as f32,
        (1.0 - ndc.y) * 0.5 * height as f32,
    ])
}
/// A coarse spatial diagnostic, not a camera-id or measured cabin enclosure.
/// In particular, bumper/wheel/side cameras can occupy the same neighborhood as
/// the driver. Identifying C1 requires separate vehicle/view calibration evidence.
pub fn classify(relative_to_cabin: Vec3) -> ViewKind {
    if relative_to_cabin.x.abs() < 1.8
        && relative_to_cabin.y.abs() < 2.0
        && relative_to_cabin.z.abs() < 2.5
    {
        ViewKind::NearCabinUnclassified
    } else if relative_to_cabin.z > 3.0
        && relative_to_cabin.y > 0.0
        && (5.0..=30.0).contains(&relative_to_cabin.length())
    {
        ViewKind::Chase
    } else {
        ViewKind::Other
    }
}

/// Stateful diagnostic using official SDK physical placement/cabin offsets.
/// ETS2LA's interpolated truck_position is a bounding-box center, so it cannot
/// be substituted for the SDK reference point without a measured offset.
#[derive(Default)]
pub struct CameraViewTracker {
    previous: Option<CameraViewFrame>,
    selected: Option<u32>,
}
impl CameraViewTracker {
    pub fn with_orientation_hypothesis(selected: u32) -> Result<Self, &'static str> {
        if selected > 1 {
            return Err("camera orientation hypothesis must be 0 or 1");
        }
        Ok(Self {
            previous: None,
            selected: Some(selected),
        })
    }
    pub fn reset(&mut self) {
        self.previous = None;
    }
    pub fn update(
        &mut self,
        telemetry: &TelemetryFrame,
        extension: Option<&ExtensionFrame>,
        config: Option<&ConfigurationFrame>,
        dynamics: Option<&DynamicsFrame>,
        session_start_qpc: u64,
    ) -> CameraViewFrame {
        let mut output = CameraViewFrame {
            header: telemetry.header,
            selected_hypothesis: self.selected.unwrap_or(u32::MAX),
            session_start_qpc,
            flags: CAMERA_VIEW_PHYSICS_REFERENCE | CAMERA_VIEW_HEURISTIC_BOUNDS,
            ..Default::default()
        };
        let fail = |tracker: &mut Self, mut output: CameraViewFrame, flag: u32| {
            output.flags |= flag;
            tracker.previous = None;
            output
        };
        let Some(extension) = extension.filter(|e| {
            e.available & 1 != 0
                && e.header.frame_id == telemetry.header.frame_id
                && e.header.qpc == telemetry.header.qpc
                && e.header.sim_time_us == telemetry.header.sim_time_us
        }) else {
            return fail(self, output, CAMERA_VIEW_MISSING_CAMERA);
        };
        let camera: Camera = bytemuck::pod_read_unaligned(&extension.camera);
        let camera_position = camera.position;
        let camera_rotation = camera.quaternion_wxyz;
        if !camera_position.iter().all(|x| x.is_finite())
            || !valid_quaternion(camera_rotation)
            || !telemetry.position_scs.iter().all(|x| x.is_finite())
            || !telemetry.orientation_turns.iter().all(|x| x.is_finite())
        {
            return fail(self, output, CAMERA_VIEW_INVALID_CAMERA);
        }
        let Some(config) = config.filter(|c| {
            c.effective_from_frame_id <= telemetry.header.frame_id
                && c.header.qpc <= telemetry.header.qpc
                && c.truck_fields & CONFIG_HAS_CABIN as u64 != 0
        }) else {
            return fail(self, output, CAMERA_VIEW_MISSING_CONFIG);
        };
        output.config_seq = config.config_seq;
        let cabin_position = config.truck.cabin_position.map(f64::from);
        if !cabin_position.iter().all(|x| x.is_finite()) {
            return fail(self, output, CAMERA_VIEW_MISSING_CONFIG);
        }
        let truck_rotation = scs_rotation(telemetry.orientation_turns);
        let (offset, offset_rotation) = if let Some(d) = dynamics.filter(|d| {
            d.header.frame_id == telemetry.header.frame_id
                && d.header.qpc == telemetry.header.qpc
                && d.header.sim_time_us == telemetry.header.sim_time_us
                && d.valid & DYNAMICS_CABIN_OFFSET != 0
        }) {
            if !d.cabin_offset_position.iter().all(|x| x.is_finite())
                || !d
                    .cabin_offset_orientation_turns
                    .iter()
                    .all(|x| x.is_finite())
            {
                return fail(self, output, CAMERA_VIEW_INVALID_CAMERA);
            }
            (
                DVec3::from_array(d.cabin_offset_position),
                scs_rotation(d.cabin_offset_orientation_turns),
            )
        } else {
            output.flags |= CAMERA_VIEW_STATIC_CABIN;
            (DVec3::ZERO, DQuat::IDENTITY)
        };
        let cabin_world = DVec3::from_array(telemetry.position_scs)
            + truck_rotation * (DVec3::from_array(cabin_position) + offset);
        let cabin_rotation = truck_rotation * offset_rotation;
        let position = cabin_rotation.conjugate()
            * (DVec3::from_array(sector_world(camera.position, camera.sector)) - cabin_world);
        if !position.is_finite() {
            return fail(self, output, CAMERA_VIEW_INVALID_CAMERA);
        }
        output.relative_position_scs = position.to_array();
        output.flags |= CAMERA_VIEW_POSITION_VALID | CAMERA_VIEW_ORIENTATION_CANDIDATES_VALID;
        output.view_kind = match classify(position.as_vec3()) {
            ViewKind::Interior => VIEW_INTERIOR,
            ViewKind::Chase => VIEW_CHASE,
            ViewKind::Other => VIEW_OTHER,
            ViewKind::NearCabinUnclassified => VIEW_NEAR_CABIN_UNCLASSIFIED,
        };
        let q = DQuat::from_xyzw(
            camera_rotation[1] as f64,
            camera_rotation[2] as f64,
            camera_rotation[3] as f64,
            camera_rotation[0] as f64,
        )
        .normalize();
        for (index, world_rotation) in [q, q.conjugate()].into_iter().enumerate() {
            output.relative_quaternion_candidates_wxyz[index] =
                to_wxyz((cabin_rotation.conjugate() * world_rotation).normalize());
        }
        if let Some(previous) = self.previous {
            output.previous_frame_id = previous.header.frame_id;
            let same_reference = previous.session_start_qpc == session_start_qpc
                && previous.config_seq == output.config_seq
                && previous.flags & CAMERA_VIEW_STATIC_CABIN
                    == output.flags & CAMERA_VIEW_STATIC_CABIN;
            if same_reference
                && previous.header.frame_id.checked_add(1) == Some(output.header.frame_id)
                && output.header.qpc > previous.header.qpc
                && telemetry.header.flags & 1 == 0
            {
                output.flags |= CAMERA_VIEW_HAS_PREVIOUS;
                output.position_delta_m =
                    position.distance(DVec3::from_array(previous.relative_position_scs));
                for i in 0..2 {
                    let a = from_wxyz(previous.relative_quaternion_candidates_wxyz[i]);
                    let b = from_wxyz(output.relative_quaternion_candidates_wxyz[i]);
                    output.rotation_delta_rad[i] = 2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos();
                    if output.position_delta_m > 0.5
                        || output.rotation_delta_rad[i] > 10.0f64.to_radians()
                    {
                        output.switched_mask |= 1 << i;
                    }
                }
            } else {
                output.flags |= CAMERA_VIEW_DISCONTINUITY;
            }
        }
        self.previous = Some(output);
        output
    }
}
fn scs_rotation(turns: [f32; 3]) -> DQuat {
    let [heading, pitch, roll] = turns.map(|v| v as f64 * std::f64::consts::TAU);
    DQuat::from_rotation_y(heading) * DQuat::from_rotation_x(pitch) * DQuat::from_rotation_z(roll)
}
fn to_wxyz(q: DQuat) -> [f64; 4] {
    let [x, y, z, w] = q.to_array();
    [w, x, y, z]
}
fn from_wxyz(q: [f64; 4]) -> DQuat {
    DQuat::from_xyzw(q[1], q[2], q[3], q[0])
}
/// Native actor positions are bounding-box centers. This helper must not be
/// used with the SDK truck reference point as though it were a box center.
pub fn bbox_corners_scs(
    center: [f64; 3],
    size: [f32; 3],
    quaternion_wxyz: [f32; 4],
) -> Option<[[f64; 3]; 8]> {
    if !center
        .iter()
        .all(|v| v.is_finite() && v.abs() < 10_000_000.0)
        || !size.iter().all(|v| v.is_finite() && *v > 0.1 && *v < 50.0)
        || !valid_quaternion(quaternion_wxyz)
    {
        return None;
    }
    let q = quaternion_wxyz;
    let rotation = Quat::from_xyzw(q[1], q[2], q[3], q[0]).normalize();
    Some(std::array::from_fn(|index| {
        let local = Vec3::from_array(std::array::from_fn(|axis| {
            size[axis] * if index & (1 << axis) == 0 { -0.5 } else { 0.5 }
        }));
        let offset = (rotation * local).to_array();
        std::array::from_fn(|axis| center[axis] + offset[axis] as f64)
    }))
}

/// An axis-aligned screen diagnostic for an actor box, clipped to the viewport.
/// Occlusion is unknown. A near-plane crossing is omitted instead of producing
/// an unstable full-screen rectangle; a calibrated visibility test is separate.
pub fn project_bbox(
    camera: Camera,
    corners: [[f64; 3]; 8],
    width: u32,
    height: u32,
    convention: Convention,
) -> Option<[f32; 4]> {
    let mut bounds = [
        f32::INFINITY,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NEG_INFINITY,
    ];
    for corner in corners {
        let [x, y] = project(camera, corner, width, height, convention)?;
        bounds[0] = bounds[0].min(x);
        bounds[1] = bounds[1].min(y);
        bounds[2] = bounds[2].max(x);
        bounds[3] = bounds[3].max(y);
    }
    bounds[0] = bounds[0].clamp(0.0, width as f32);
    bounds[2] = bounds[2].clamp(0.0, width as f32);
    bounds[1] = bounds[1].clamp(0.0, height as f32);
    bounds[3] = bounds[3].clamp(0.0, height as f32);
    (bounds.iter().all(|x| x.is_finite()) && bounds[0] < bounds[2] && bounds[1] < bounds[3])
        .then_some(bounds)
}
#[cfg(test)]
mod tests {
    use super::*;
    use bytemuck::Zeroable;
    fn view_fixture(
        frame: u64,
        relative: [f64; 3],
        turns: [f32; 3],
    ) -> (
        TelemetryFrame,
        Box<ExtensionFrame>,
        ConfigurationFrame,
        DynamicsFrame,
    ) {
        let t = TelemetryFrame {
            header: WireHeader {
                frame_id: frame,
                sim_time_us: frame * 16667,
                qpc: frame * 1000,
                ..Default::default()
            },
            position_scs: [128512.0, 20.0, -256512.0],
            orientation_turns: turns,
            qpc_frequency: 60000,
            ..Default::default()
        };
        let mut config = ConfigurationFrame {
            effective_from_frame_id: 1,
            config_seq: 1,
            truck_fields: CONFIG_HAS_CABIN as u64,
            ..Default::default()
        };
        config.truck.cabin_position = [0.0, 1.0, 0.0];
        config.truck.config_seq = 1;
        let d = DynamicsFrame {
            header: t.header,
            valid: DYNAMICS_CABIN_OFFSET,
            cabin_offset_position: [0.1, 0.2, 0.3],
            ..Default::default()
        };
        let rotation = scs_rotation(turns);
        let world = DVec3::from_array(t.position_scs)
            + rotation
                * (DVec3::from_array(config.truck.cabin_position.map(f64::from))
                    + DVec3::from_array(d.cabin_offset_position)
                    + DVec3::from_array(relative));
        let mut c = Camera::zeroed();
        let sector = [251, -501];
        c.sector = sector;
        c.position = [
            (world.x - sector[0] as f64 * 512.0) as f32,
            world.y as f32,
            (world.z - sector[1] as f64 * 512.0) as f32,
        ];
        c.quaternion_wxyz = to_wxyz(rotation).map(|v| v as f32);
        // Deliberately unrelated bbox center: it must not be used as SDK origin.
        c.truck_position = [-9999.0; 3];
        c.truck_quaternion_wxyz = [0.0; 4];
        let mut ext = etsads_shm::zeroed_box::<ExtensionFrame>();
        ext.header = t.header;
        ext.available = 1;
        ext.camera.copy_from_slice(bytemuck::bytes_of(&c));
        (t, ext, config, d)
    }
    #[test]
    fn cabin_relative_pose_rotates_translates_and_does_not_use_bbox_center() {
        let (t, e, c, d) = view_fixture(1, [0.5, 0.6, -0.2], [0.25, 0.01, -0.02]);
        let output = CameraViewTracker::default().update(&t, Some(&e), Some(&c), Some(&d), 99);
        for (a, b) in output
            .relative_position_scs
            .into_iter()
            .zip([0.5, 0.6, -0.2])
        {
            assert!((a - b).abs() < 0.001);
        }
        assert_eq!(output.view_kind, VIEW_NEAR_CABIN_UNCLASSIFIED);
        assert_eq!(output.selected_hypothesis, u32::MAX);
        assert_ne!(output.flags & CAMERA_VIEW_POSITION_VALID, 0);
        assert_eq!(output.switched_mask, 0);
        let (t, e, c, d) = view_fixture(2, [0.0, 3.0, 10.0], [0.25, 0.0, 0.0]);
        assert_eq!(
            CameraViewTracker::default()
                .update(&t, Some(&e), Some(&c), Some(&d), 99)
                .view_kind,
            VIEW_CHASE
        );
    }
    #[test]
    fn recorded_r5_near_cabin_views_never_claim_calibrated_c1() {
        // Extracted from immutable runs/live-r5-camera-steer-01; full provenance
        // and matching exact video frames are in artifacts/...-near-cabin-comparison.json.
        // These are regression inputs, not reusable truck calibration values.
        let cases = [
            (
                "cam1",
                [-0.7375358031471836, 1.5937670950439926, 0.6480048027322693],
                [
                    -0.9984406474300437,
                    0.05582354964689118,
                    0.0000038001463233869575,
                    -0.00006964090119434977,
                ],
            ),
            (
                "cam5",
                [-1.567502081091639, 1.7837612931027407, 0.547993364713659],
                [
                    -0.11185774541164599,
                    0.019194328760905505,
                    -0.9775269286481463,
                    -0.1776528252028763,
                ],
            ),
            (
                "cam6",
                [
                    0.000131277412147357,
                    0.28129663912041214,
                    2.4016957902218214,
                ],
                [
                    -0.999999999990296,
                    0.000003448273036298078,
                    -0.0000002174510446812406,
                    0.0000027331957555562854,
                ],
            ),
            (
                "cam7",
                [-1.7998893563015046, -0.2184757430836488, 1.8015967081939956],
                [
                    -0.999999999985006,
                    -0.000002768233983101122,
                    0.00000008667179661431502,
                    -0.000004724149605344828,
                ],
            ),
            (
                "cam1-return",
                [-0.7374863550109921, 1.59377356767062, 0.6479797068580928],
                [
                    -0.9984409014857233,
                    0.05581900497408639,
                    0.0000037306473027731973,
                    -0.00007007254574998472,
                ],
            ),
        ];
        let mut tracker = CameraViewTracker::default();
        for (index, (name, relative, quaternion)) in cases.into_iter().enumerate() {
            let (t, mut e, c, d) = view_fixture(index as u64 + 1, relative, [0.0; 3]);
            let mut camera: Camera = bytemuck::pod_read_unaligned(&e.camera);
            camera.quaternion_wxyz = quaternion.map(|v| v as f32);
            e.camera.copy_from_slice(bytemuck::bytes_of(&camera));
            let output = tracker.update(&t, Some(&e), Some(&c), Some(&d), 99);
            assert_eq!(output.view_kind, VIEW_NEAR_CABIN_UNCLASSIFIED, "{name}");
            assert_ne!(output.view_kind, VIEW_INTERIOR);
            assert_eq!(output.selected_hypothesis, u32::MAX);
            let expected = CAMERA_VIEW_POSITION_VALID
                | CAMERA_VIEW_ORIENTATION_CANDIDATES_VALID
                | CAMERA_VIEW_PHYSICS_REFERENCE
                | CAMERA_VIEW_HEURISTIC_BOUNDS;
            assert_eq!(output.flags & expected, expected);
            for (a, b) in output.relative_position_scs.into_iter().zip(relative) {
                assert!((a - b).abs() < 0.001);
            }
            assert_eq!(
                output.switched_mask,
                if index == 0 { 0 } else { 3 },
                "{name}: classification must not suppress real pose switches"
            );
            // A projection-orientation choice is not a camera identity calibration.
            let selected = CameraViewTracker::with_orientation_hypothesis(0)
                .unwrap()
                .update(&t, Some(&e), Some(&c), Some(&d), 99);
            assert_eq!(selected.view_kind, VIEW_NEAR_CABIN_UNCLASSIFIED);
            assert_eq!(selected.selected_hypothesis, 0);
        }
    }
    #[test]
    fn new_near_label_preserves_far_diagnostics_invalidity_and_legacy_raw_codes() {
        for (position, expected) in [
            ([0.0, 3.0, 10.0], VIEW_CHASE),
            ([0.0, 13.0, 1.8], VIEW_OTHER),
            ([22.1, 10.4, -13.4], VIEW_OTHER),
        ] {
            let (t, e, c, d) = view_fixture(1, position, [0.0; 3]);
            assert_eq!(
                CameraViewTracker::default()
                    .update(&t, Some(&e), Some(&c), Some(&d), 99)
                    .view_kind,
                expected
            );
        }
        let (t, mut e, c, d) = view_fixture(1, [0.0; 3], [0.0; 3]);
        let mut camera: Camera = bytemuck::pod_read_unaligned(&e.camera);
        camera.quaternion_wxyz = [0.0; 4];
        e.camera.copy_from_slice(bytemuck::bytes_of(&camera));
        let invalid = CameraViewTracker::default().update(&t, Some(&e), Some(&c), Some(&d), 99);
        assert_eq!(invalid.view_kind, VIEW_UNKNOWN);
        assert_ne!(invalid.flags & CAMERA_VIEW_INVALID_CAMERA, 0);
        assert_eq!(invalid.flags & CAMERA_VIEW_POSITION_VALID, 0);
        assert_eq!(
            [
                VIEW_UNKNOWN,
                VIEW_INTERIOR,
                VIEW_CHASE,
                VIEW_OTHER,
                VIEW_NEAR_CABIN_UNCLASSIFIED
            ],
            [0, 1, 2, 3, 4]
        );
        let old = CameraViewFrame {
            view_kind: VIEW_INTERIOR,
            flags: CAMERA_VIEW_HEURISTIC_BOUNDS,
            ..Default::default()
        };
        let decoded: CameraViewFrame = bytemuck::pod_read_unaligned(bytemuck::bytes_of(&old));
        assert_eq!(std::mem::size_of_val(&old), 192);
        assert_eq!(
            decoded.view_kind, 1,
            "recorded legacy payload must not be reinterpreted as the new code4"
        );
    }
    #[test]
    fn camera_switches_have_frame_continuity_and_never_use_future_config() {
        let mut tracker = CameraViewTracker::with_orientation_hypothesis(0).unwrap();
        let (t, e, c, d) = view_fixture(1, [0.0, 0.0, 0.0], [0.0; 3]);
        tracker.update(&t, Some(&e), Some(&c), Some(&d), 99);
        let (t, e, c, d) = view_fixture(2, [0.6, 0.0, 0.0], [0.0; 3]);
        let moved = tracker.update(&t, Some(&e), Some(&c), Some(&d), 99);
        assert_eq!(moved.switched_mask, 3);
        assert!(moved.position_delta_m > 0.5);
        let (t, mut e, c, d) = view_fixture(3, [0.6, 0.0, 0.0], [0.0; 3]);
        let mut camera: Camera = bytemuck::pod_read_unaligned(&e.camera);
        camera.quaternion_wxyz =
            to_wxyz(DQuat::from_rotation_y(20f64.to_radians())).map(|v| v as f32);
        e.camera.copy_from_slice(bytemuck::bytes_of(&camera));
        let rotated = tracker.update(&t, Some(&e), Some(&c), Some(&d), 99);
        assert_eq!(rotated.switched_mask, 3);
        assert!((rotated.rotation_delta_rad[0] - 20f64.to_radians()).abs() < 1e-6);
        let (t, e, c, d) = view_fixture(8, [10.0, 0.0, 0.0], [0.0; 3]);
        let gap = tracker.update(&t, Some(&e), Some(&c), Some(&d), 99);
        assert_eq!(gap.switched_mask, 0);
        assert_ne!(gap.flags & CAMERA_VIEW_DISCONTINUITY, 0);
        let mut future = c;
        future.header.qpc = t.header.qpc + 1;
        let unknown = tracker.update(&t, Some(&e), Some(&future), Some(&d), 99);
        assert_eq!(unknown.view_kind, VIEW_UNKNOWN);
        assert_ne!(unknown.flags & CAMERA_VIEW_MISSING_CONFIG, 0);
        let old = tracker.update(&t, Some(&e), None, None, 99);
        assert_ne!(old.flags & CAMERA_VIEW_MISSING_CONFIG, 0);
    }
    #[test]
    fn synthetic_camera_projects_center_and_rejects_behind() {
        let mut c = Camera::zeroed();
        c.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        c.projection = Mat4::perspective_rh_gl(1.0, 16.0 / 9.0, 0.1, 1000.0).to_cols_array();
        let convention = Convention {
            transpose: false,
            quaternion_world_to_camera: false,
            positive_z_forward: false,
        };
        assert_eq!(
            project(c, [0.0, 0.0, -10.0], 1920, 1080, convention),
            Some([960.0, 540.0])
        );
        assert_eq!(project(c, [0.0, 0.0, 10.0], 1920, 1080, convention), None);
        assert_eq!(Convention::candidates().len(), 8);
    }
    #[test]
    fn actor_bounds_are_centered_clipped_and_reject_near_plane_crossings() {
        let mut camera = Camera::zeroed();
        camera.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        camera.projection = Mat4::perspective_rh_gl(1.0, 16.0 / 9.0, 0.1, 1000.0).to_cols_array();
        let corners =
            bbox_corners_scs([0.0, 0.0, -10.0], [2.0, 2.0, 4.0], camera.quaternion_wxyz).unwrap();
        let rect = project_bbox(camera, corners, 1920, 1080, Convention::candidates()[0]).unwrap();
        assert!((rect[0] + rect[2] - 1920.0).abs() < 0.01);
        assert!((rect[1] + rect[3] - 1080.0).abs() < 0.01);
        let crossing =
            bbox_corners_scs([0.0, 0.0, -1.0], [2.0, 2.0, 4.0], camera.quaternion_wxyz).unwrap();
        assert!(project_bbox(camera, crossing, 1920, 1080, Convention::candidates()[0]).is_none());
        assert!(bbox_corners_scs([f64::NAN, 0.0, 0.0], [2.0; 3], camera.quaternion_wxyz).is_none());
    }
}
