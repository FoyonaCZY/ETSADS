//! Host validation of an exact road-operation evidence bundle. This is a
//! development evidence check, not external certification or phase acceptance.
use super::*;
use etsads_map::importer::{RegionPolicy, RoadEvidence};
use etsads_runtime::navigation::{NativeGpsSettings, NavigationRegion, RoadBoundary, RouteSource};

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadMeasurement {
    /// Explain the actual measurement/inspection method and its limitations.
    pub method: String,
    pub source_lane_ids: Vec<String>,
    pub maximum_centerline_error_m: Option<f64>,
    /// Clearance outside the measured swept vehicle envelope, not half width.
    pub minimum_envelope_clearance_m: Option<f64>,
    pub vehicle_access_observation: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoadEvidenceDocument {
    pub schema_version: u32,
    pub game_version: String,
    pub vehicle_id: String,
    pub config_sha256: String,
    pub calibration_sha256: String,
    pub map_manifest_sha256: String,
    pub map_data_sha256: String,
    pub policy_sha256: String,
    pub native_source_sha256: String,
    pub annotations_source_sha256: String,
    pub calibration_recording: RecordingHashes,
    pub supporting_files: Vec<SupportingFile>,
    /// Each file is an EvidenceReference JSON; config's three RoadEvidence
    /// paths must resolve to these same files. Reviews are separately hashed.
    pub evidence: BTreeMap<EvidencePurpose, SupportingFile>,
}

pub struct VerifiedRoadEvidence {
    pub document: RoadEvidenceDocument,
    pub digest: [u8; 32],
    binding: NavigationBinding,
    road: RoadEvidence,
    road_boundary: RoadBoundary,
}
impl VerifiedRoadEvidence {
    pub fn validate_navigation_scope(
        &self,
        config: &RuntimeConfig,
        region: &NavigationRegion,
    ) -> anyhow::Result<()> {
        let RouteSource::NativeGps(settings) = &config.route_source else {
            anyhow::bail!("road evidence requires native GPS navigation");
        };
        self.binding.validate_identity(
            &settings.expected_source_id,
            &region.provenance.source_id,
            &region.provenance.annotations_source_id,
            &region.authorization,
        )?;
        ensure!(
            settings.road_evidence.as_ref() == Some(&self.road)
                && region.road_evidence.as_ref() == Some(&self.road),
            "road measurement references changed after verification"
        );
        verify_boundary_scope(&self.road_boundary, region.road_boundary.as_ref())?;
        region.validate(settings).map_err(anyhow::Error::msg)?;
        Ok(())
    }
}

fn verify_boundary_scope(
    expected: &RoadBoundary,
    actual: Option<&RoadBoundary>,
) -> anyhow::Result<()> {
    let actual = actual.context("navigation region is missing the hash-bound road boundary")?;
    ensure!(
        serde_json::to_value(expected)? == serde_json::to_value(actual)?,
        "navigation road boundary geometry or source scope changed after evidence verification"
    );
    Ok(())
}

/// Parse precisely the bytes whose digest the evidence document binds. The
/// loader may independently read the file; its typed body must then compare
/// equal before Runtime receives the road capability.
fn verify_boundary_file(
    config_base: &Path,
    settings: &NativeGpsSettings,
    policy: &RegionPolicy,
    evidence_base: &Path,
    supporting_files: &[SupportingFile],
) -> anyhow::Result<RoadBoundary> {
    use std::io::Read;
    let declared = settings
        .road_boundary_path
        .as_deref()
        .filter(|path| !path.trim().is_empty())
        .context("road mode requires road_boundary_path naming a measured boundary JSON")?;
    let path = resolve(config_base, Path::new(declared))
        .canonicalize()
        .context("measured road boundary file is missing")?;
    ensure!(path.is_file(), "measured road boundary path is not a file");
    let mut digest = None;
    for supporting in supporting_files {
        if resolve(evidence_base, &supporting.path).canonicalize()? == path {
            let current = hash_bytes(&supporting.sha256)?;
            ensure!(
                digest.is_none_or(|old| old == current),
                "conflicting road boundary hashes in supporting_files"
            );
            digest = Some(current);
        }
    }
    let expected = digest
        .context("road boundary file must appear in supporting_files with its exact SHA-256")?;
    let mut bytes = Vec::new();
    std::fs::File::open(&path)?
        .take(8 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= 8 * 1024 * 1024,
        "road boundary JSON exceeds 8 MiB"
    );
    let actual: [u8; 32] = Sha256::digest(&bytes).into();
    ensure!(
        actual == expected,
        "road boundary content hash mismatch; review the actual polygon before updating evidence"
    );
    let boundary: RoadBoundary =
        serde_json::from_slice(&bytes).context("parse hash-bound road boundary")?;
    boundary
        .validate(settings, policy)
        .map_err(anyhow::Error::msg)?;
    Ok(boundary)
}

fn review_measurement(
    purpose: EvidencePurpose,
    review: &EvidenceReview,
    policy: &RegionPolicy,
) -> anyhow::Result<()> {
    let measured = review
        .road_measurement
        .as_ref()
        .context("road review lacks a measurement method and source-lane scope")?;
    let actual: BTreeSet<_> = measured.source_lane_ids.iter().collect();
    let expected: BTreeSet<_> = policy.verified_lane_ids.iter().collect();
    ensure!(
        !measured.method.trim().is_empty()
            && !actual.is_empty()
            && actual.len() == measured.source_lane_ids.len()
            && expected.len() == policy.verified_lane_ids.len()
            && actual == expected,
        "road review must measure the exact original source-lane whitelist and explain its method"
    );
    match purpose {
        EvidencePurpose::RoadCenterline => ensure!(
            measured
                .maximum_centerline_error_m
                .is_some_and(|e| e.is_finite() && e >= 0.0 && e <= policy.maximum_lateral_error_m),
            "road centerline error is missing or exceeds the reviewed lateral-error limit"
        ),
        EvidencePurpose::PhysicalBoundary => ensure!(
            measured
                .minimum_envelope_clearance_m
                .is_some_and(|c| c.is_finite() && c > 0.0),
            "physical-boundary review must report positive measured clearance outside the swept vehicle envelope"
        ),
        EvidencePurpose::RoadVehicleAccess => ensure!(
            measured
                .vehicle_access_observation
                .as_ref()
                .is_some_and(|s| !s.trim().is_empty()),
            "vehicle-access review must describe observed truck permissions/restrictions for the exact lanes"
        ),
        _ => anyhow::bail!("unexpected road evidence purpose"),
    }
    Ok(())
}

/// Every reviewed video association must have its own actual camera payload.
/// Text review cannot turn absent/other-frame camera state into measurement.
fn verify_camera_observations(run: &Path, interval: [u64; 2]) -> anyhow::Result<()> {
    let mut observed_videos = 0usize;
    etsads_replay::ReplayAdapter::run(run, |frame| {
        if !(interval[0]..=interval[1]).contains(&frame.snapshot.raw.header.frame_id) {
            return Ok(());
        }
        if frame.video.is_empty() {
            return Ok(());
        }
        let ext = frame
            .snapshot
            .extension
            .as_ref()
            .context("road measurement video has no same-frame extension")?;
        ensure!(
            ext.available & 1 != 0,
            "road measurement video has no available camera"
        );
        let camera: etsads_game::ets2la::Camera = bytemuck::pod_read_unaligned(&ext.camera);
        ensure!(
            camera
                .position
                .into_iter()
                .chain(camera.projection)
                .all(f32::is_finite)
                && etsads_game::ets2la::valid_quaternion(camera.quaternion_wxyz),
            "road measurement camera payload is invalid"
        );
        observed_videos += frame.video.len();
        Ok(())
    })?;
    ensure!(
        observed_videos > 0,
        "road measurement interval has no same-frame video/camera observations"
    );
    Ok(())
}

pub fn verify_road_evidence(
    options: &DriveOptions,
    config: &RuntimeConfig,
    calibration: &VehicleCalibration,
    game_version: &str,
) -> anyhow::Result<VerifiedRoadEvidence> {
    let RouteSource::NativeGps(settings) = &config.route_source else {
        anyhow::bail!("--road-evidence requires native GPS road mode");
    };
    let road = settings.road_evidence.as_ref().context(
        "road evidence requires all three typed road measurement references in RuntimeConfig",
    )?;
    let path=options.road_evidence.as_ref().context("road mode requires --road-evidence with a hash-bound road evidence document; files/true alone are insufficient")?;
    let base = path.parent().unwrap_or(Path::new("."));
    let document: RoadEvidenceDocument = read_config(path, false)?;
    ensure!(
        document.schema_version == 1
            && document.game_version == game_version
            && document.vehicle_id == config.vehicle_id,
        "road evidence schema/game/vehicle identity mismatch"
    );
    verify_hash(&options.config, &document.config_sha256)?;
    verify_hash(&options.calibration, &document.calibration_sha256)?;
    let map = options
        .map
        .as_ref()
        .context("road evidence requires a complete partition map")?;
    let policy_path = options
        .policy
        .as_ref()
        .context("road evidence requires its original policy")?;
    verify_hash(&map_manifest_path(map), &document.map_manifest_sha256)?;
    verify_hash(policy_path, &document.policy_sha256)?;
    let partition = map_artifacts::verify_partition(map)?;
    ensure!(
        hash_bytes(&partition.data_hash)? == hash_bytes(&document.map_data_sha256)?
            && hash_bytes(&partition.source_id)? == hash_bytes(&document.native_source_sha256)?
            && settings.expected_source_id == partition.source_id,
        "road map content or source identity differs from review"
    );
    let annotations = partition
        .annotations_source_id
        .context("road mode requires traffic/native-route partition data")?;
    ensure!(
        hash_bytes(&annotations)? == hash_bytes(&document.annotations_source_sha256)?,
        "road annotation content differs from review"
    );
    let policy: RegionPolicy = read_config(policy_path, false)?;
    let range = calibration.road_speed_range()?;
    ensure!(
        policy.maximum_speed_mps.is_finite()
            && policy.maximum_speed_mps > 0.0
            && policy.maximum_speed_mps <= range[1],
        "road speed limit exceeds recorded dynamic calibration applicability"
    );
    calibration.validate_measured_envelope()?;
    let calibration_base = options.calibration.parent().unwrap_or(Path::new("."));
    let source = Path::new(&calibration.source_run);
    let source = if source.exists() {
        source.to_owned()
    } else {
        resolve(calibration_base, source)
    };
    verify_recording_files(
        &source,
        &document.calibration_recording.meta_sha256,
        &document.calibration_recording.segments,
        game_version,
    )?;
    let mut speed_min = f64::INFINITY;
    let mut speed_max = 0.0f64;
    let report = etsads_recorder::replay(&source, |topic, record| {
        if topic == "/telemetry" {
            let t: TelemetryFrame = record.decode_wire()?;
            let speed = (t.speed_mps as f64).abs();
            ensure!(
                speed.is_finite(),
                "calibration recording contains invalid speed"
            );
            speed_min = speed_min.min(speed);
            speed_max = speed_max.max(speed);
        }
        Ok(())
    })?;
    ensure!(
        report.telemetry_frames > 1
            && report.frame_gaps == 0
            && report.nonmonotonic == 0
            && range[0] >= speed_min
            && range[1] <= speed_max,
        "road calibration applicability is unsupported by its complete recorded speed range"
    );
    let mut support = BTreeSet::new();
    for file in &document.supporting_files {
        let target = resolve(base, &file.path);
        verify_hash(&target, &file.sha256)?;
        support.insert(target.canonicalize()?);
    }
    let require = |owner: &Path, path: &str, label: &str| -> anyhow::Result<PathBuf> {
        ensure!(!path.trim().is_empty(), "{label} evidence is empty");
        let target = resolve(owner, Path::new(path))
            .canonicalize()
            .with_context(|| format!("{label} evidence file is missing"))?;
        ensure!(
            support.contains(&target),
            "{label} must be hash-bound in road supporting_files"
        );
        Ok(target)
    };
    require(
        policy_path.parent().unwrap_or(Path::new(".")),
        &policy.evidence_path,
        "road policy",
    )?;
    if let Some(r) = &calibration.reference {
        require(calibration_base, &r.evidence_path, "rear axle reference")?;
    }
    if let Some(c) = &calibration.conditions {
        require(
            calibration_base,
            &c.evidence_path,
            "trailer/load applicability",
        )?;
    }
    if let Some(f) = &calibration.front_bumper {
        require(calibration_base, &f.evidence_path, "front bumper")?;
    }
    if let Some(m) = &calibration.envelope_measurement {
        require(calibration_base, &m.evidence_path, "measured envelope")?;
    }
    let config_base = options.config.parent().unwrap_or(Path::new("."));
    let road_boundary = verify_boundary_file(
        config_base,
        settings,
        &policy,
        base,
        &document.supporting_files,
    )?;
    let fingerprint = calibration
        .geometry_fingerprint
        .as_deref()
        .context("road calibration lacks geometry fingerprint")?;
    for (purpose, declared) in [
        (
            EvidencePurpose::RoadCenterline,
            &road.centerline_evidence_path,
        ),
        (
            EvidencePurpose::PhysicalBoundary,
            &road.physical_boundary_evidence_path,
        ),
        (
            EvidencePurpose::RoadVehicleAccess,
            &road.vehicle_access_evidence_path,
        ),
    ] {
        let declared = require(config_base, declared, "road measurement reference")?;
        let bound = document
            .evidence
            .get(&purpose)
            .with_context(|| format!("missing {purpose:?} road evidence reference"))?;
        let reference_path = resolve(base, &bound.path);
        verify_hash(&reference_path, &bound.sha256)?;
        ensure!(
            reference_path.canonicalize()? == declared,
            "road config and evidence document reference different {purpose:?} artifacts"
        );
        let reference: EvidenceReference = read_config(&reference_path, false)?;
        let reference_base = reference_path.parent().unwrap_or(Path::new("."));
        verify_recorded_review(
            reference_base,
            &reference,
            purpose,
            ReviewIdentity {
                game_version,
                vehicle_id: &config.vehicle_id,
            },
            &[],
            None,
            fingerprint,
        )?;
        let review: EvidenceReview =
            read_config(&resolve(reference_base, &reference.review), false)?;
        review_measurement(purpose, &review, &policy)?;
        verify_camera_observations(&resolve(reference_base, &reference.run), review.frame_range)?;
    }
    Ok(VerifiedRoadEvidence {
        digest: hash_bytes(&plugin_hash(path)?)?,
        document,
        binding: NavigationBinding {
            source_id: partition.source_id,
            annotations_source_id: annotations,
            authorization: policy,
        },
        road: road.clone(),
        road_boundary,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn boundary_fixture() -> (NativeGpsSettings, RegionPolicy, RoadBoundary) {
        let road = RoadEvidence {
            centerline_evidence_path: "centerline-reference.json".into(),
            physical_boundary_evidence_path: "physical-reference.json".into(),
            vehicle_access_evidence_path: "access-reference.json".into(),
        };
        let settings = NativeGpsSettings {
            expected_source_id: "01".repeat(32),
            corridor_policy: Default::default(),
            tolls: vec![],
            roundabout_exits: vec![],
            road_evidence: Some(road),
            road_boundary_path: Some("boundary.json".into()),
        };
        let policy = RegionPolicy {
            region_id: "synthetic boundary fixture".into(),
            evidence_path: "review.json".into(),
            verified_lane_ids: vec!["lane-a".into()],
            allow_navigation_width: false,
            require_fresh_telemetry_speed_limit: true,
            maximum_speed_mps: 12.0,
            maximum_lateral_error_m: 0.3,
        };
        let body = RoadBoundary {
            schema_version: 1,
            map_source_id: settings.expected_source_id.clone(),
            coordinate_system: etsads_runtime::navigation::RoadBoundaryCoordinates::EnuMetres,
            verified_source_lane_ids: policy.verified_lane_ids.clone(),
            boundary: etsads_runtime::parking_mission::VerifiedBoundary {
                polygon: vec![(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (0.0, 10.0)],
                physical_boundaries_verified: true,
                evidence_path: "physical-reference.json".into(),
            },
        };
        (settings, policy, body)
    }
    #[test]
    fn road_boundary_requires_actual_hash_bound_file_and_exact_review_scope() {
        let (settings, policy, body) = boundary_fixture();
        let root = std::env::temp_dir().join(format!(
            "etsads-road-boundary-{}-{}",
            std::process::id(),
            etsads_shm::qpc()
        ));
        std::fs::create_dir(&root).unwrap();
        assert!(verify_boundary_file(&root, &settings, &policy, &root, &[]).is_err());
        let path = root.join("boundary.json");
        std::fs::write(&path, serde_json::to_vec(&body).unwrap()).unwrap();
        assert!(
            verify_boundary_file(&root, &settings, &policy, &root, &[])
                .unwrap_err()
                .to_string()
                .contains("supporting_files")
        );
        let mut support = vec![SupportingFile {
            path: "boundary.json".into(),
            sha256: plugin_hash(&path).unwrap(),
        }];
        let verified = verify_boundary_file(&root, &settings, &policy, &root, &support).unwrap();
        verify_boundary_scope(&body, Some(&verified)).unwrap();
        let mut changed = body.clone();
        changed.boundary.polygon[1].0 += 1.0;
        std::fs::write(&path, serde_json::to_vec(&changed).unwrap()).unwrap();
        assert!(
            verify_boundary_file(&root, &settings, &policy, &root, &support)
                .unwrap_err()
                .to_string()
                .contains("hash mismatch")
        );
        changed
            .verified_source_lane_ids
            .push("unreviewed-lane".into());
        std::fs::write(&path, serde_json::to_vec(&changed).unwrap()).unwrap();
        support[0].sha256 = plugin_hash(&path).unwrap();
        assert!(
            verify_boundary_file(&root, &settings, &policy, &root, &support).is_err(),
            "even an updated file hash cannot expand the original lane review"
        );
        changed = body;
        changed.boundary.evidence_path = "different-physical-reference.json".into();
        std::fs::write(&path, serde_json::to_vec(&changed).unwrap()).unwrap();
        support[0].sha256 = plugin_hash(&path).unwrap();
        assert!(verify_boundary_file(&root, &settings, &policy, &root, &support).is_err());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn road_boundary_scope_compares_entire_geometry_not_only_source_labels() {
        let (settings, policy, body) = boundary_fixture();
        verify_boundary_scope(&body, Some(&body)).unwrap();
        assert!(verify_boundary_scope(&body, None).is_err());
        let mut changed = body.clone();
        changed.boundary.polygon[1].0 += 0.1;
        changed.validate(&settings, &policy).unwrap(); // structurally valid, same source/review
        assert!(verify_boundary_scope(&body, Some(&changed)).is_err());
        changed = body.clone();
        changed.map_source_id = "02".repeat(32);
        assert!(verify_boundary_scope(&body, Some(&changed)).is_err());
        changed = body.clone();
        changed.verified_source_lane_ids.push("unreviewed".into());
        assert!(verify_boundary_scope(&body, Some(&changed)).is_err());
    }
    #[test]
    fn road_review_needs_specific_measured_method_and_exact_lane_scope() {
        let policy = RegionPolicy {
            region_id: "test".into(),
            evidence_path: "measured.json".into(),
            verified_lane_ids: vec!["lane-a".into()],
            allow_navigation_width: false,
            require_fresh_telemetry_speed_limit: true,
            maximum_speed_mps: 15.0,
            maximum_lateral_error_m: 0.3,
        };
        let mut review = EvidenceReview {
            schema_version: 1,
            purpose: EvidencePurpose::RoadCenterline,
            decision: ReviewDecision::AllowBoundedDevelopmentExperiment,
            game_version: "synthetic".into(),
            vehicle_id: "fixture".into(),
            frame_range: [1, 2],
            reviewed_by: "isolated test".into(),
            observations: "fixture".into(),
            limitations: "not game evidence".into(),
            observed_actions: vec![],
            road_measurement: None,
        };
        assert!(review_measurement(EvidencePurpose::RoadCenterline, &review, &policy).is_err());
        review.road_measurement = Some(RoadMeasurement {
            method: "synthetic geometry measurement fixture".into(),
            source_lane_ids: vec!["lane-a".into()],
            maximum_centerline_error_m: Some(0.2),
            minimum_envelope_clearance_m: Some(0.5),
            vehicle_access_observation: Some("synthetic observation".into()),
        });
        for purpose in [
            EvidencePurpose::RoadCenterline,
            EvidencePurpose::PhysicalBoundary,
            EvidencePurpose::RoadVehicleAccess,
        ] {
            review_measurement(purpose, &review, &policy).unwrap();
        }
        review
            .road_measurement
            .as_mut()
            .unwrap()
            .source_lane_ids
            .push("lane-b".into());
        assert!(review_measurement(EvidencePurpose::PhysicalBoundary, &review, &policy).is_err());
        review
            .road_measurement
            .as_mut()
            .unwrap()
            .source_lane_ids
            .pop();
        review
            .road_measurement
            .as_mut()
            .unwrap()
            .maximum_centerline_error_m = Some(0.4);
        assert!(review_measurement(EvidencePurpose::RoadCenterline, &review, &policy).is_err());
        review
            .road_measurement
            .as_mut()
            .unwrap()
            .minimum_envelope_clearance_m = Some(0.0);
        assert!(review_measurement(EvidencePurpose::PhysicalBoundary, &review, &policy).is_err());
        review
            .road_measurement
            .as_mut()
            .unwrap()
            .vehicle_access_observation = None;
        assert!(review_measurement(EvidencePurpose::RoadVehicleAccess, &review, &policy).is_err());
    }
    #[test]
    fn road_video_measurement_never_uses_missing_wrong_time_or_invalid_camera() {
        use bytemuck::Zeroable;
        use etsads_recorder::{Record, Recorder};
        let root = std::env::temp_dir().join(format!(
            "etsads-road-camera-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&root).unwrap();
        for scenario in ["missing", "wrong_time", "invalid", "matched"] {
            let dir = root.join(scenario);
            let mut recorder = Recorder::create(
                &dir,
                RunMetadata::new("synthetic", "camera association fixture only"),
            )
            .unwrap();
            let first = TelemetryFrame {
                header: WireHeader {
                    frame_id: 1,
                    qpc: 100,
                    sim_time_us: 100,
                    ..Default::default()
                },
                qpc_frequency: 1000,
                ..Default::default()
            };
            recorder
                .record("/telemetry", &Record::telemetry(&first))
                .unwrap();
            let second = TelemetryFrame {
                header: WireHeader {
                    frame_id: 2,
                    qpc: 200,
                    sim_time_us: 200,
                    ..Default::default()
                },
                ..first
            };
            recorder
                .record("/telemetry", &Record::telemetry(&second))
                .unwrap();
            if scenario != "missing" {
                let mut ext = etsads_shm::zeroed_box::<ExtensionFrame>();
                ext.header = first.header;
                ext.available = 1;
                if scenario == "wrong_time" {
                    ext.header.qpc += 1;
                }
                let mut camera = etsads_game::ets2la::Camera::zeroed();
                if scenario != "invalid" {
                    camera.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
                }
                ext.camera.copy_from_slice(bytemuck::bytes_of(&camera));
                recorder
                    .record(
                        "/extension",
                        &Record::wire(
                            ext.as_ref(),
                            ext.header.frame_id,
                            ext.header.sim_time_us,
                            ext.header.qpc,
                        ),
                    )
                    .unwrap();
            }
            let video = VideoIndex {
                capture_frame: 1,
                capture_qpc: 100,
                physics_frame: 1,
                physics_qpc: 100,
                sim_time_us: 100,
                ..Default::default()
            };
            recorder
                .record("/video_index", &Record::wire(&video, 1, 100, 100))
                .unwrap();
            let alignment = VideoAlignment {
                capture_frame: 1,
                capture_qpc: 100,
                ..Default::default()
            };
            recorder
                .record("/video_alignment", &Record::wire(&alignment, 1, 100, 100))
                .unwrap();
            recorder.finish().unwrap();
            let result = verify_camera_observations(&dir, [1, 2]);
            assert_eq!(
                result.is_ok(),
                scenario == "matched",
                "{scenario}: {result:?}"
            );
            // This association helper never turns the synthetic source into
            // road authorization: outer verification still rejects it.
            if scenario == "matched" {
                let meta = plugin_hash(&dir.join("meta.json")).unwrap();
                let files = std::fs::read_dir(&dir)
                    .unwrap()
                    .map(|entry| entry.unwrap().path())
                    .filter(|p| p.extension().is_some_and(|e| e == "mcap"))
                    .map(|p| {
                        (
                            p.file_name().unwrap().to_string_lossy().into_owned(),
                            plugin_hash(&p).unwrap(),
                        )
                    })
                    .collect();
                assert!(
                    verify_recording_files(&dir, &meta, &files, "camera association fixture only")
                        .is_err()
                );
            }
        }
        std::fs::remove_dir_all(root).unwrap();
    }
}
