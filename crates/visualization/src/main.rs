//! Offline projection audit. Intentionally has no live/capture/input commands.
use ab_glyph::{Font, FontRef, PxScale, ScaleFont, point};
use anyhow::{Context, ensure};
use clap::Parser;
use etsads_game::{
    camera::project,
    ets2la::{self, Camera, Parked, Traffic},
};
use etsads_msgs::protocol::{ConfigBlock, ExtensionFrame, TelemetryFrame, VideoIndex};
use etsads_visualization::{Candidate, implied_fov, vanishing_point, vertical_edge_error};
use glam::{Quat, Vec3};
use image::{Rgb, RgbImage};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::Read,
    path::{Path, PathBuf},
    process::Command,
};

#[derive(Parser)]
#[command(
    about = "Generate eight camera-convention comparisons from an immutable MCAP/video recording"
)]
struct Args {
    #[arg(long)]
    run: PathBuf,
    #[arg(long)]
    map: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long, value_delimiter = ',')]
    frames: Vec<u64>,
    #[arg(long)]
    annotations: Option<PathBuf>,
    #[arg(long, default_value = "ffmpeg")]
    ffmpeg: String,
    #[arg(long, default_value = "C:/Windows/Fonts/segoeui.ttf")]
    font: PathBuf,
}

#[derive(Clone, Serialize, Deserialize)]
struct BoxEvidence {
    id: String,
    source: String,
    position_scs: [f64; 3],
    quaternion_wxyz: [f32; 4],
    size: [f32; 3],
}
struct CameraFrame {
    frame_id: u64,
    qpc: u64,
    sim_time_us: u64,
    camera: Camera,
    boxes: Vec<BoxEvidence>,
}
#[derive(Serialize)]
struct CameraEvidence {
    fov: f32,
    position_scs: [f64; 3],
    quaternion_wxyz: [f32; 4],
    projection: [f32; 16],
    render_truck_position: [f32; 3],
    render_truck_quaternion: [f32; 4],
}
impl From<Camera> for CameraEvidence {
    fn from(camera: Camera) -> Self {
        Self {
            fov: camera.fov,
            position_scs: ets2la::sector_world(camera.position, camera.sector),
            quaternion_wxyz: camera.quaternion_wxyz,
            projection: camera.projection,
            render_truck_position: camera.truck_position,
            render_truck_quaternion: camera.truck_quaternion_wxyz,
        }
    }
}
#[derive(Clone)]
struct WorldLine {
    id: String,
    points: Vec<[f64; 3]>,
    color: [u8; 3],
    source: String,
}
#[derive(Deserialize, Default)]
struct Annotations {
    #[serde(default)]
    interior_front_view: bool,
    #[serde(default)]
    description: String,
    #[serde(default)]
    frames: BTreeMap<String, Vec<VerticalEdge>>,
}
#[derive(Clone, Deserialize, Serialize)]
struct VerticalEdge {
    label: String,
    pixels: [[f64; 2]; 2],
    #[serde(default)]
    held_out: bool,
}
#[derive(Serialize)]
struct CandidateReport {
    candidate: Candidate,
    score: f64,
    vertical_edge_rms_px: Option<f64>,
    held_out_vertical_rms_px: Option<f64>,
    front_view_prior_passed: Option<bool>,
    intrinsic_reference_consistency: f64,
    implied_vertical_fov_deg: Option<f64>,
    implied_4_3_reference_fov_deg: Option<f64>,
    world_up_vanishing_point: Option<[f64; 2]>,
    actor_centers_in_image: usize,
    actor_centers_front: usize,
    projected_map_segments: usize,
    actor_visibility_not_verified: bool,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let input_meta: serde_json::Value =
        serde_json::from_slice(&fs::read(args.run.join("meta.json"))?)?;
    ensure!(
        input_meta["source"] == "game" && input_meta["completed"] == true,
        "only completed real-game recordings are accepted"
    );
    fs::create_dir_all(&args.output)?;
    let annotations: Annotations = if let Some(path) = &args.annotations {
        serde_json::from_slice(&fs::read(path)?)?
    } else {
        Annotations::default()
    };
    let font_bytes = fs::read(&args.font).context("provide --font with a local TrueType font")?;
    let font =
        FontRef::try_from_slice(&font_bytes).map_err(|_| anyhow::anyhow!("unsupported font"))?;
    let mut telemetry = BTreeMap::new();
    let mut cameras = Vec::new();
    let mut video = Vec::new();
    let mut configs = Vec::new();
    let replay = etsads_recorder::replay(&args.run, |topic, record| {
        match topic {
            "/telemetry" => {
                let t = record.decode_wire::<TelemetryFrame>()?;
                telemetry.insert(t.header.frame_id, t);
            }
            "/video_index" => video.push(record.decode_wire::<VideoIndex>()?),
            "/config" | "/configuration" => {
                configs.push((record.qpc, record.decode_wire::<ConfigBlock>()?))
            }
            "/extension" => {
                ensure!(
                    record.raw_payload.len() == std::mem::size_of::<ExtensionFrame>(),
                    "extension ABI size mismatch"
                );
                let data = &record.raw_payload;
                let available = u32::from_le_bytes(data[40..44].try_into()?);
                if available & 1 != 0 {
                    let offset = std::mem::offset_of!(ExtensionFrame, camera);
                    let camera: Camera = bytemuck::pod_read_unaligned(&data[offset..offset + 128]);
                    if ets2la::valid_quaternion(camera.quaternion_wxyz) {
                        let mut boxes = Vec::new();
                        if available & 2 != 0 {
                            let offset = std::mem::offset_of!(ExtensionFrame, traffic);
                            for item in ets2la::records::<Traffic>(&data[offset..offset + 6960]) {
                                let id = item.id;
                                if id >= 0
                                    && valid_box(item.position, item.size, item.quaternion_wxyz)
                                {
                                    boxes.push(BoxEvidence {
                                        id: format!("T{id}"),
                                        source: "native_traffic_bbox_center".into(),
                                        position_scs: item.position.map(f64::from),
                                        quaternion_wxyz: item.quaternion_wxyz,
                                        size: item.size,
                                    });
                                }
                            }
                        }
                        if available & 4 != 0 {
                            let offset = std::mem::offset_of!(ExtensionFrame, parked);
                            for item in ets2la::records::<Parked>(&data[offset..offset + 1720]) {
                                let id = item.id;
                                if id >= 0
                                    && valid_box(item.position, item.size, item.quaternion_wxyz)
                                {
                                    boxes.push(BoxEvidence {
                                        id: format!("P{id}"),
                                        source: "native_parked_bbox_center".into(),
                                        position_scs: item.position.map(f64::from),
                                        quaternion_wxyz: item.quaternion_wxyz,
                                        size: item.size,
                                    });
                                }
                            }
                        }
                        cameras.push(CameraFrame {
                            frame_id: record.frame_id,
                            qpc: record.qpc,
                            sim_time_us: record.sim_time_us,
                            camera,
                            boxes,
                        });
                    }
                }
            }
            _ => {}
        };
        Ok(())
    })?;
    ensure!(
        cameras.len() >= 3 && video.len() >= 3,
        "recording needs camera extensions and video indices"
    );
    cameras.sort_by_key(|camera| camera.qpc);
    video.sort_by_key(|index| index.video_frame);
    let frequency = telemetry
        .values()
        .next()
        .context("no telemetry")?
        .qpc_frequency;
    ensure!(frequency > 0, "invalid QPC frequency");
    let targets = if args.frames.is_empty() {
        [0.1, 0.5, 0.9]
            .into_iter()
            .map(|fraction| {
                video[((video.len() - 1) as f64 * fraction).round() as usize].video_frame
            })
            .collect()
    } else {
        args.frames.clone()
    };
    ensure!(
        targets.len() >= 3,
        "at least three recorded frames are required"
    );
    ensure!(
        targets.iter().copied().collect::<BTreeSet<_>>().len() >= 3,
        "at least three distinct recorded frames are required"
    );
    let map: serde_json::Value = serde_json::from_slice(&fs::read(&args.map)?)?;
    let mut frame_reports = Vec::new();
    let mut score_sums = [0.0; 8];
    for target in targets {
        let index = video
            .iter()
            .find(|index| index.video_frame == target)
            .context("requested video frame has no recorded index")?;
        let camera = cameras
            .iter()
            .min_by_key(|camera| camera.qpc.abs_diff(index.capture_qpc))
            .context("no camera")?;
        let physics = telemetry
            .get(&camera.frame_id)
            .or_else(|| {
                telemetry
                    .values()
                    .min_by_key(|t| t.header.qpc.abs_diff(camera.qpc))
            })
            .context("no matched telemetry")?;
        let image_name = format!("source-{target:06}.png");
        let image_path = args.output.join(&image_name);
        extract_frame(
            &args.ffmpeg,
            &args.run.join("video-0000.mp4"),
            target,
            &image_path,
        )?;
        let source = image::open(&image_path)?.to_rgb8();
        let (width, height) = source.dimensions();
        let origin = ets2la::sector_world(camera.camera.position, camera.camera.sector);
        let mut lines = map_lines(&map, origin);
        if let Some((_, config)) = configs
            .iter()
            .filter(|(qpc, _)| *qpc <= camera.qpc)
            .max_by_key(|(qpc, _)| *qpc)
        {
            let q = camera.camera.truck_quaternion_wxyz;
            let rotation = Quat::from_xyzw(q[1], q[2], q[3], q[0]);
            for i in 0..16 {
                if config.wheel_radius[i] > 0.0 {
                    let offset = rotation * Vec3::from_array(config.wheel_positions[i]);
                    let p = std::array::from_fn(|axis| {
                        physics.position_scs[axis] + offset[axis] as f64
                    });
                    lines.push(WorldLine {
                        id: format!("W{i}"),
                        points: vec![p],
                        color: [220, 130, 255],
                        source: "native_config_wheel_center_using_render_rotation".into(),
                    });
                }
            }
        }
        let edges = annotations
            .frames
            .get(&target.to_string())
            .cloned()
            .unwrap_or_default();
        let mut candidates = Vec::new();
        let mut overview = RgbImage::new(width, height / 2 + 100);
        for candidate in Candidate::all() {
            let mut canvas = source.clone();
            let up = vanishing_point(camera.camera, [0.0, 1.0, 0.0], width, height, candidate);
            let fitting_errors: Vec<_> = edges
                .iter()
                .filter(|edge| !edge.held_out)
                .filter_map(|edge| vertical_edge_error(edge.pixels, up?))
                .collect();
            let held_errors: Vec<_> = edges
                .iter()
                .filter(|edge| edge.held_out)
                .filter_map(|edge| vertical_edge_error(edge.pixels, up?))
                .collect();
            let rms = |values: &[f64]| {
                if values.is_empty() {
                    None
                } else {
                    Some((values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt())
                }
            };
            let vertical_error = rms(&fitting_errors);
            let held_error = rms(&held_errors);
            let fov = implied_fov(camera.camera, candidate);
            let source_fov = camera.camera.fov as f64;
            let intrinsic = fov
                .map(|(v, reference)| {
                    let horizontal = 2.0
                        * ((width as f64 / height as f64) * (v.to_radians() / 2.0).tan())
                            .atan()
                            .to_degrees();
                    let error = (source_fov - v)
                        .abs()
                        .min((source_fov - reference).abs())
                        .min((source_fov - horizontal).abs());
                    1.0 / (1.0 + (error / 5.0).powi(2))
                })
                .unwrap_or(0.0);
            // Independent of the extension camera quaternion hypothesis: the
            // SCS SDK defines heading=0 as north, increasing counter-clockwise.
            let heading = physics.orientation_turns[0] as f64 * std::f64::consts::TAU;
            let forward = Vec3::new(-heading.sin() as f32, 0.0, -heading.cos() as f32);
            let forward_point =
                std::array::from_fn(|axis| origin[axis] + 20.0 * forward[axis] as f64);
            let forward_pixel = project(
                camera.camera,
                forward_point,
                width,
                height,
                candidate.convention(),
            );
            let front = annotations
                .interior_front_view
                .then(|| forward_pixel.is_some_and(|p| inside(p, width, height)));
            let mut score = 0.2 * intrinsic;
            let mut total = 0.2;
            if let Some(pass) = front {
                score += 0.25 * f64::from(pass);
                total += 0.25;
            }
            if let Some(error) = vertical_error {
                score += 0.55 / (1.0 + (error / 2.0).powi(2));
                total += 0.55;
            }
            score /= total;
            score_sums[candidate.id] += score;
            let mut actor_centers_front = 0;
            let mut actor_centers_in_image = 0;
            for bbox in &camera.boxes {
                if let Some(pixel) = project(
                    camera.camera,
                    bbox.position_scs,
                    width,
                    height,
                    candidate.convention(),
                ) {
                    actor_centers_front += 1;
                    if inside(pixel, width, height) {
                        actor_centers_in_image += 1;
                        cross(&mut canvas, pixel, [255, 178, 66], 5);
                        label(
                            &mut canvas,
                            &font,
                            pixel[0] + 7.0,
                            pixel[1] - 8.0,
                            &bbox.id,
                            [255, 178, 66],
                            16.0,
                        );
                    }
                }
                draw_box(&mut canvas, camera.camera, candidate, bbox);
            }
            let mut segments = 0;
            for world_line in &lines {
                let pixels: Vec<_> = world_line
                    .points
                    .iter()
                    .map(|p| project(camera.camera, *p, width, height, candidate.convention()))
                    .collect();
                for pair in pixels.windows(2) {
                    if let [Some(a), Some(b)] = pair
                        && line(&mut canvas, *a, *b, world_line.color)
                    {
                        segments += 1;
                    }
                }
                if world_line.points.len() == 1
                    && let Some(pixel) = pixels[0]
                    && inside(pixel, width, height)
                {
                    cross(&mut canvas, pixel, world_line.color, 7);
                    label(
                        &mut canvas,
                        &font,
                        pixel[0] + 8.0,
                        pixel[1],
                        &world_line.id,
                        world_line.color,
                        16.0,
                    );
                }
            }
            if let Some(pixel) = forward_pixel
                && inside(pixel, width, height)
            {
                cross(&mut canvas, pixel, [100, 240, 140], 10);
                label(
                    &mut canvas,
                    &font,
                    pixel[0] + 12.0,
                    pixel[1],
                    "body heading (SDK direction only)",
                    [100, 240, 140],
                    16.0,
                );
            }
            for edge in &edges {
                let a = edge.pixels[0].map(|v| v as f32);
                let b = edge.pixels[1].map(|v| v as f32);
                line(
                    &mut canvas,
                    a,
                    b,
                    if edge.held_out {
                        [255, 210, 110]
                    } else {
                        [250, 250, 250]
                    },
                );
                if let Some(vp) = up {
                    let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
                    let dx = vp[0] - mid[0] as f64;
                    let dy = vp[1] - mid[1] as f64;
                    let norm = dx.hypot(dy);
                    let delta = [(dx / norm * 110.0) as f32, (dy / norm * 110.0) as f32];
                    line(
                        &mut canvas,
                        [mid[0] - delta[0], mid[1] - delta[1]],
                        [mid[0] + delta[0], mid[1] + delta[1]],
                        [255, 65, 140],
                    );
                    label(
                        &mut canvas,
                        &font,
                        a[0] + 6.0,
                        a[1],
                        edge.label.split_whitespace().next().unwrap_or("edge"),
                        [255, 255, 255],
                        14.0,
                    );
                }
            }
            dark_band(&mut canvas, 0, 60);
            dark_band(&mut canvas, height - 56, 56);
            label(
                &mut canvas,
                &font,
                15.0,
                8.0,
                &format!(
                    "C{}  transpose={}  world_to_camera={}  +Z={}  score={:.3}",
                    candidate.id,
                    candidate.transpose,
                    candidate.quaternion_world_to_camera,
                    candidate.positive_z_forward,
                    score
                ),
                [245, 248, 255],
                22.0,
            );
            label(
                &mut canvas,
                &font,
                15.0,
                34.0,
                &format!(
                    "video #{target} / camera frame {} / offline delta {:.2} ms / fit error {:?} px",
                    camera.frame_id,
                    camera.qpc.abs_diff(index.capture_qpc) as f64 * 1000.0 / frequency as f64,
                    vertical_error.map(|v| (v * 100.0).round() / 100.0)
                ),
                [180, 205, 230],
                16.0,
            );
            label(
                &mut canvas,
                &font,
                15.0,
                height as f32 - 48.0,
                "cyan: road centre | purple: map-display ground (not physical bounds) | amber: native actor boxes (occlusion unknown)",
                [230, 235, 245],
                17.0,
            );
            label(
                &mut canvas,
                &font,
                15.0,
                height as f32 - 24.0,
                "white/yellow: image verticals (fit/held-out) | pink: predicted vertical direction | no fabricated ego bounding box",
                [230, 235, 245],
                17.0,
            );
            canvas.save(
                args.output
                    .join(format!("frame-{target:06}-c{}.png", candidate.id)),
            )?;
            let tile = image::imageops::resize(
                &canvas,
                width / 4,
                height / 4,
                image::imageops::FilterType::Triangle,
            );
            image::imageops::replace(
                &mut overview,
                &tile,
                (candidate.id % 4) as i64 * (width / 4) as i64,
                (candidate.id / 4) as i64 * (height / 4) as i64,
            );
            label(
                &mut overview,
                &font,
                (candidate.id % 4) as f32 * (width / 4) as f32 + 8.0,
                (candidate.id / 4) as f32 * (height / 4) as f32 + 5.0,
                &format!("C{}  {:.3}", candidate.id, score),
                [255, 255, 255],
                22.0,
            );
            candidates.push(CandidateReport {
                candidate,
                score,
                vertical_edge_rms_px: vertical_error,
                held_out_vertical_rms_px: held_error,
                front_view_prior_passed: front,
                intrinsic_reference_consistency: intrinsic,
                implied_vertical_fov_deg: fov.map(|f| f.0),
                implied_4_3_reference_fov_deg: fov.map(|f| f.1),
                world_up_vanishing_point: up,
                actor_centers_in_image,
                actor_centers_front,
                projected_map_segments: segments,
                actor_visibility_not_verified: true,
            });
        }
        label(
            &mut overview,
            &font,
            15.0,
            height as f32 / 2.0 + 12.0,
            &format!(
                "Recorded frame {target}. Compare with source-{target:06}.png. Counts of projected actors are not visibility/IoU validation."
            ),
            [230, 235, 245],
            20.0,
        );
        label(
            &mut overview,
            &font,
            15.0,
            height as f32 / 2.0 + 42.0,
            "Use the full-resolution candidate PNGs for geometric review. Evidence is limited to the recorded view.",
            [230, 235, 245],
            20.0,
        );
        overview.save(args.output.join(format!("comparison-{target:06}.png")))?;
        fs::write(
            args.output
                .join(format!("frame-{target:06}-world-markers.json")),
            serde_json::to_vec_pretty(
                &serde_json::json!({"actors":camera.boxes,"map_features":lines.iter().map(|l|serde_json::json!({"id":l.id,"source":l.source,"scs_points":l.points})).collect::<Vec<_>>()}),
            )?,
        )?;
        frame_reports.push(serde_json::json!({"video_frame":target,"capture_frame":index.capture_frame,"capture_qpc":index.capture_qpc,
            "original_physics_frame":index.physics_frame,"original_alignment_error_ms":index.alignment_error_qpc as f64*1000.0/frequency as f64,
            "camera_frame_id":camera.frame_id,"camera_qpc":camera.qpc,"camera_sim_time_us":camera.sim_time_us,
            "camera_alignment_error_ms":camera.qpc.abs_diff(index.capture_qpc)as f64*1000.0/frequency as f64,
            "camera_from_future":camera.qpc>index.capture_qpc,"camera":CameraEvidence::from(camera.camera),
            "telemetry_frame_id":physics.header.frame_id,"telemetry_position_scs":physics.position_scs,"telemetry_speed_mps":physics.speed_mps,
            "telemetry_orientation_turns":physics.orientation_turns,"front_prior_source":"independent SCS SDK heading projected along the horizontal body-forward direction",
            "source_image":image_name,"source_image_sha256":sha256(&image_path)?,"image_width":width,"image_height":height,
            "vertical_annotations":edges,"candidates":candidates}));
    }
    let mut rankings: Vec<_> = score_sums
        .into_iter()
        .enumerate()
        .map(|(id, sum)| (id, sum / frame_reports.len() as f64))
        .collect();
    rankings.sort_by(|a, b| b.1.total_cmp(&a.1));
    let recommended = (rankings[0].1 - rankings[1].1 > 0.08
        && annotations.interior_front_view
        && !annotations.frames.is_empty())
    .then_some(rankings[0].0);
    let mut mcap_hashes = Vec::new();
    for entry in fs::read_dir(&args.run)? {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "mcap") {
            mcap_hashes.push(serde_json::json!({"path":path,"sha256":sha256(&path)?}));
        }
    }
    let report = serde_json::json!({"source":"recorded_game_only","run":args.run,"recording_metadata":input_meta,"replay":replay,
        "mcap_files":mcap_hashes,"map_sha256":sha256(&args.map)?,
        "qpc_frequency":frequency,"video_sha256":sha256(&args.run.join("video-0000.mp4"))?,"annotations_description":annotations.description,
        "scoring":"0.55 vertical-edge fit + 0.25 manually declared front-interior prior + 0.20 intrinsic consistency; absent terms excluded. Actor/map visibility is not scored.",
        "ranking":rankings.iter().map(|(id,score)|serde_json::json!({"candidate":Candidate::all()[*id],"mean_score":score})).collect::<Vec<_>>(),
        "preferred_candidate":recommended.map(|id|Candidate::all()[id]),"fully_calibrated":false,"frames":frame_reports,
        "limitations":["Recorded video uses offline nearest-QPC pairing, not proof of zero-latency capture.","Vertical-edge labels are manual pixel observations; the 4:3 FOV match is an inference from recorded coefficients.","Native actor boxes may be occluded or outside the main camera; no bounding-box IoU labels were fabricated.","Map-display polygons are ground references, not collision geometry or known painted-line correspondences.","No true ego bounding-box dimensions are available in this protocol; only native wheel anchors may be rendered.","The reviewed three frames repeat one stationary front-interior pose; they are not independent camera poses.","No exterior/chase-view calibration is implied; absolute 3D-to-pixel correspondences are still needed."]});
    fs::write(
        args.output.join("report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"output":args.output,"frames":frame_reports.len(),"ranking":report["ranking"],"preferred_candidate":report["preferred_candidate"],"fully_calibrated":false})
        )?
    );
    Ok(())
}

fn valid_box(position: [f32; 3], size: [f32; 3], quaternion: [f32; 4]) -> bool {
    position.into_iter().all(f32::is_finite)
        && size
            .into_iter()
            .all(|v| v.is_finite() && v > 0.1 && v < 50.0)
        && ets2la::valid_quaternion(quaternion)
}
fn sha256(path: &Path) -> anyhow::Result<String> {
    let mut file = fs::File::open(path)?;
    let mut hash = Sha256::new();
    let mut block = [0_u8; 65536];
    loop {
        let n = file.read(&mut block)?;
        if n == 0 {
            break;
        }
        hash.update(&block[..n]);
    }
    Ok(format!("{:x}", hash.finalize()))
}
fn extract_frame(ffmpeg: &str, video: &Path, frame: u64, out: &Path) -> anyhow::Result<()> {
    let mut command = Command::new(ffmpeg);
    command
        .args(["-v", "error", "-y", "-i"])
        .arg(video)
        .args([
            "-vf",
            &format!("select=eq(n\\,{frame})"),
            "-frames:v",
            "1",
            "-fps_mode",
            "passthrough",
        ])
        .arg(out);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    let output = command.output()?;
    ensure!(
        output.status.success(),
        "ffmpeg extraction failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    ensure!(out.exists(), "video frame not present");
    Ok(())
}
fn point_from(value: &serde_json::Value) -> Option<[f64; 3]> {
    Some([
        value["x"].as_f64()?,
        value["z"].as_f64()?,
        -value["y"].as_f64()?,
    ])
}
fn map_lines(map: &serde_json::Value, origin: [f64; 3]) -> Vec<WorldLine> {
    let near = |p: [f64; 3]| (p[0] - origin[0]).hypot(p[2] - origin[2]) < 180.0;
    let mut result = Vec::new();
    if let Some(lanes) = map["lanes"].as_array() {
        for lane in lanes {
            if let Some(points) = lane["center"].as_array() {
                let points: Vec<_> = points.iter().step_by(3).filter_map(point_from).collect();
                if points.iter().any(|p| near(*p)) {
                    result.push(WorldLine {
                        id: lane["id"].as_str().unwrap_or("lane").into(),
                        points,
                        color: [65, 205, 230],
                        source: "native_or_calibrated_lane_center".into(),
                    });
                }
            }
        }
    }
    if let Some(polygons) = map["boundary_candidates"].as_array() {
        for polygon in polygons {
            if let Some(points) = polygon["points"].as_array() {
                let mut points: Vec<_> = points.iter().filter_map(point_from).collect();
                if points.iter().any(|p| near(*p)) {
                    if let Some(p) = points.first().copied() {
                        points.push(p);
                    }
                    result.push(WorldLine {
                        id: polygon["id"].as_str().unwrap_or("display").into(),
                        points,
                        color: [175, 100, 220],
                        source: "ui_map_display_ground_not_physical_boundary".into(),
                    });
                }
            }
        }
    }
    for name in ["pois", "slots"] {
        if let Some(items) = map[name].as_array() {
            for item in items {
                if let Some(p) = point_from(item)
                    && near(p)
                {
                    result.push(WorldLine {
                        id: format!(
                            "{} {}",
                            item["kind"].as_str().unwrap_or(name),
                            item["id"].as_str().unwrap_or("")
                        ),
                        points: vec![p],
                        color: [80, 240, 155],
                        source: format!("native_map_{name}"),
                    });
                }
            }
        }
    }
    result
}
fn inside(p: [f32; 2], w: u32, h: u32) -> bool {
    p[0].is_finite()
        && p[1].is_finite()
        && p[0] >= 0.0
        && p[1] >= 0.0
        && p[0] < w as f32
        && p[1] < h as f32
}
fn line(image: &mut RgbImage, a: [f32; 2], b: [f32; 2], color: [u8; 3]) -> bool {
    if !a.into_iter().chain(b).all(f32::is_finite) {
        return false;
    }
    let (w, h) = image.dimensions();
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let mut lo = 0.0_f32;
    let mut hi = 1.0_f32;
    for (p, q) in [
        (-dx, a[0]),
        (dx, w as f32 - 1.0 - a[0]),
        (-dy, a[1]),
        (dy, h as f32 - 1.0 - a[1]),
    ] {
        if p.abs() < 1e-9 {
            if q < 0.0 {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                lo = lo.max(t);
            } else {
                hi = hi.min(t);
            }
            if lo > hi {
                return false;
            }
        }
    }
    let start = [a[0] + lo * dx, a[1] + lo * dy];
    let end = [a[0] + hi * dx, a[1] + hi * dy];
    let steps = (end[0] - start[0])
        .abs()
        .max((end[1] - start[1]).abs())
        .ceil() as usize;
    for i in 0..=steps {
        let t = i as f32 / steps.max(1) as f32;
        let x = (start[0] + t * (end[0] - start[0])).round() as i32;
        let y = (start[1] + t * (end[1] - start[1])).round() as i32;
        if x >= 0 && y >= 0 && x < w as i32 && y < h as i32 {
            image.put_pixel(x as u32, y as u32, Rgb(color));
        }
    }
    true
}
fn cross(image: &mut RgbImage, p: [f32; 2], color: [u8; 3], radius: i32) {
    line(
        image,
        [p[0] - radius as f32, p[1]],
        [p[0] + radius as f32, p[1]],
        color,
    );
    line(
        image,
        [p[0], p[1] - radius as f32],
        [p[0], p[1] + radius as f32],
        color,
    );
}
fn label(
    image: &mut RgbImage,
    font: &FontRef<'_>,
    x: f32,
    y: f32,
    text: &str,
    color: [u8; 3],
    size: f32,
) {
    let scaled = font.as_scaled(PxScale::from(size));
    let mut cursor = x;
    for ch in text.chars() {
        let id = font.glyph_id(ch);
        let glyph = id.with_scale_and_position(size, point(cursor, y + scaled.ascent()));
        if let Some(outline) = font.outline_glyph(glyph) {
            let bounds = outline.px_bounds();
            outline.draw(|px, py, alpha| {
                let ix = bounds.min.x as i32 + px as i32;
                let iy = bounds.min.y as i32 + py as i32;
                if ix >= 0 && iy >= 0 && ix < image.width() as i32 && iy < image.height() as i32 {
                    let pixel = image.get_pixel_mut(ix as u32, iy as u32);
                    for c in 0..3 {
                        pixel[c] =
                            (pixel[c] as f32 * (1.0 - alpha) + color[c] as f32 * alpha) as u8;
                    }
                }
            });
        }
        cursor += scaled.h_advance(id);
    }
}
fn dark_band(image: &mut RgbImage, start: u32, height: u32) {
    for y in start..(start + height).min(image.height()) {
        for x in 0..image.width() {
            let p = image.get_pixel_mut(x, y);
            for c in 0..3 {
                p[c] = (p[c] as f32 * 0.2) as u8;
            }
        }
    }
}
fn draw_box(image: &mut RgbImage, camera: Camera, candidate: Candidate, bbox: &BoxEvidence) {
    let q = bbox.quaternion_wxyz;
    let rotation = Quat::from_xyzw(q[1], q[2], q[3], q[0]);
    let mut vertices = [None; 8];
    for (i, vertex) in vertices.iter_mut().enumerate() {
        let local = Vec3::new(
            if i & 1 == 0 { -0.5 } else { 0.5 },
            if i & 2 == 0 { -0.5 } else { 0.5 },
            if i & 4 == 0 { -0.5 } else { 0.5 },
        ) * Vec3::from_array(bbox.size);
        let delta = rotation * local;
        let world = std::array::from_fn(|axis| bbox.position_scs[axis] + delta[axis] as f64);
        *vertex = project(
            camera,
            world,
            image.width(),
            image.height(),
            candidate.convention(),
        );
    }
    for i in 0..8 {
        for bit in [1, 2, 4] {
            let j = i ^ bit;
            if i < j
                && let (Some(a), Some(b)) = (vertices[i], vertices[j])
            {
                line(image, a, b, [255, 178, 66]);
            }
        }
    }
}
