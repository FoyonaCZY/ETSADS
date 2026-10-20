//! Explicit host trust boundary for supervised development experiments. The
//! CLI verifies immutable game recordings, review artifacts and raw file hashes
//! before constructing this capability. A capability is not stage acceptance.
use crate::{DrivingMode, ExecutionMode, RuntimeConfig, RuntimeError};
use etsads_control::identification::VehicleCalibration;
use etsads_map::importer::RegionGraph;
use sha2::{Digest, Sha256};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LiveExperimentKind {
    ControlledRoute,
    Parking,
}

/// Not deserializable and not constructible from an acceptance boolean. The
/// private binding prevents reuse for another configuration or calibration.
#[derive(Clone, Debug)]
pub struct VerifiedLiveExperiment {
    kind: LiveExperimentKind,
    binding: [u8; 32],
    evidence_sha256: [u8; 32],
    safety_profile_sha256: Option<[u8; 32]>,
    navigation_scope: Option<[u8; 32]>,
}
impl VerifiedLiveExperiment {
    /// Call only after the host has verified every evidence artifact against
    /// its source game MCAP segments, identity, frame ranges and SHA-256 hashes.
    /// Parking also requires a fresh independent supervisor status with the
    /// exact safety-profile digest on every live frame. This library does not
    /// perform filesystem or supervisor IPC verification for the host.
    pub fn from_host_verification(
        config: &RuntimeConfig,
        calibration: &VehicleCalibration,
        region: Option<&RegionGraph>,
        kind: LiveExperimentKind,
        evidence_sha256: [u8; 32],
        safety_profile_sha256: Option<[u8; 32]>,
    ) -> Result<Self, RuntimeError> {
        let invalid = |reason: &str| RuntimeError::Configuration(reason.into());
        calibration
            .validate_measured_envelope()
            .map_err(|e| invalid(&e.to_string()))?;
        if config.execution != ExecutionMode::Live || mode_kind(&config.mode) != Some(kind) {
            return Err(invalid(
                "experiment capability mode does not match the requested live mission",
            ));
        }
        if evidence_sha256 == [0; 32]
            || safety_profile_sha256 == Some([0; 32])
            || kind == LiveExperimentKind::Parking && safety_profile_sha256.is_none()
        {
            return Err(invalid(
                "live experiment requires a verified evidence digest; parking also requires an independently running safety profile",
            ));
        }
        Ok(Self {
            kind,
            binding: binding(config, calibration, region)?,
            evidence_sha256,
            safety_profile_sha256,
            navigation_scope: None,
        })
    }
    /// Extend a host-verified experiment to the exact partition source and
    /// original authorization whitelist. The host must have authenticated all
    /// referenced map/traffic/topology shards and the policy file in the review.
    pub fn with_native_scope(mut self, scope: [u8; 32]) -> Result<Self, RuntimeError> {
        if scope == [0; 32] || self.kind != LiveExperimentKind::ControlledRoute {
            return Err(RuntimeError::Configuration(
                "invalid native navigation experiment scope".into(),
            ));
        }
        self.navigation_scope = Some(scope);
        Ok(self)
    }
    pub(crate) fn navigation_scope(&self) -> Option<[u8; 32]> {
        self.navigation_scope
    }
    pub(crate) fn validates(
        &self,
        config: &RuntimeConfig,
        calibration: &VehicleCalibration,
        region: Option<&RegionGraph>,
    ) -> Result<(), RuntimeError> {
        if config.execution != ExecutionMode::Live
            || mode_kind(&config.mode) != Some(self.kind)
            || self.binding != binding(config, calibration, region)?
            || self.evidence_sha256 == [0; 32]
            || self.kind == LiveExperimentKind::Parking && self.safety_profile_sha256.is_none()
        {
            return Err(RuntimeError::Configuration(
                "live experiment evidence is not bound to this mission and vehicle calibration"
                    .into(),
            ));
        }
        Ok(())
    }
}
fn mode_kind(mode: &DrivingMode) -> Option<LiveExperimentKind> {
    match mode {
        DrivingMode::ControlledRoute { .. } => Some(LiveExperimentKind::ControlledRoute),
        DrivingMode::Parking { .. } => Some(LiveExperimentKind::Parking),
        _ => None,
    }
}
fn binding(
    config: &RuntimeConfig,
    calibration: &VehicleCalibration,
    region: Option<&RegionGraph>,
) -> Result<[u8; 32], RuntimeError> {
    let region_data = region
        .map(|r| {
            r.imported.graph.to_json().map(|graph| {
                (
                    graph,
                    &r.imported.runtime_to_source,
                    &r.imported.native_owners,
                    &r.policy,
                    r.physical_boundaries_verified,
                )
            })
        })
        .transpose()
        .map_err(|e| RuntimeError::Configuration(format!("experiment map binding: {e}")))?;
    let bytes = serde_json::to_vec(&(
        "ETSADS_LIVE_EXPERIMENT_V1",
        config,
        calibration,
        region_data,
    ))
    .map_err(|e| RuntimeError::Configuration(format!("experiment binding serialization: {e}")))?;
    Ok(Sha256::digest(bytes).into())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn capability_is_bound_to_exact_mission_and_calibration() {
        let (mut config, mut region) = crate::replay::controlled_fixture(false, false, false);
        config.execution = ExecutionMode::Live;
        let mut calibration = crate::replay::with_measured_envelope(crate::replay::calibration());
        let token = VerifiedLiveExperiment::from_host_verification(
            &config,
            &calibration,
            Some(&region),
            LiveExperimentKind::ControlledRoute,
            [1; 32],
            None,
        )
        .unwrap();
        token
            .validates(&config, &calibration, Some(&region))
            .unwrap();
        region.imported.graph.lanes.get_mut(&1).unwrap().speed_limit -= 0.5;
        assert!(
            token
                .validates(&config, &calibration, Some(&region))
                .is_err()
        );
        calibration.delay_s = Some(0.2);
        assert!(
            token
                .validates(&config, &calibration, Some(&region))
                .is_err()
        );
        assert!(
            VerifiedLiveExperiment::from_host_verification(
                &config,
                &calibration,
                Some(&region),
                LiveExperimentKind::Parking,
                [1; 32],
                Some([2; 32])
            )
            .is_err()
        );
        assert!(
            VerifiedLiveExperiment::from_host_verification(
                &config,
                &calibration,
                Some(&region),
                LiveExperimentKind::ControlledRoute,
                [0; 32],
                None
            )
            .is_err()
        );
    }
    #[test]
    fn parking_defaults_closed_and_explicit_experiment_keeps_semantic_checks() {
        use crate::{
            Runtime,
            parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary},
        };
        let mut config = crate::replay::config(ExecutionMode::Live);
        config.mode = DrivingMode::Parking {
            mission: ParkingMission {
                target: ParkingTarget {
                    source_id: "synthetic-slot".into(),
                    pose: etsads_msgs::Pose2::default(),
                    reference: SlotReference::TrailerAxle,
                },
                boundary: VerifiedBoundary {
                    polygon: vec![
                        (-100.0, -100.0),
                        (100.0, -100.0),
                        (100.0, 100.0),
                        (-100.0, 100.0),
                    ],
                    physical_boundaries_verified: true,
                    evidence_path: "synthetic-test-boundary".into(),
                },
                static_obstacles: vec![],
                max_expansions: 1000,
                maximum_duration_s: 30.0,
                recovery: Default::default(),
            },
        };
        let calibration = crate::replay::with_measured_envelope(crate::replay::calibration());
        assert!(Runtime::new(config.clone(), calibration.clone(), None).is_err());
        assert!(
            VerifiedLiveExperiment::from_host_verification(
                &config,
                &calibration,
                None,
                LiveExperimentKind::Parking,
                [1; 32],
                None
            )
            .is_err()
        );
        let token = VerifiedLiveExperiment::from_host_verification(
            &config,
            &calibration,
            None,
            LiveExperimentKind::Parking,
            [1; 32],
            Some([2; 32]),
        )
        .unwrap();
        assert!(
            Runtime::new_live_experiment(config.clone(), calibration.clone(), None, token).is_ok()
        );
        if let DrivingMode::Parking { mission } = &mut config.mode {
            mission.target.reference = SlotReference::Unknown;
        }
        let token = VerifiedLiveExperiment::from_host_verification(
            &config,
            &calibration,
            None,
            LiveExperimentKind::Parking,
            [1; 32],
            Some([2; 32]),
        )
        .unwrap();
        assert!(Runtime::new_live_experiment(config, calibration, None, token).is_err());
    }
    #[test]
    fn legacy_geometry_remains_replay_only_and_measurement_values_must_match_evidence() {
        let original = crate::replay::calibration();
        let mut json = serde_json::to_value(&original).unwrap();
        json.as_object_mut().unwrap().remove("envelope_measurement");
        json["geometry"]
            .as_object_mut()
            .unwrap()
            .remove("measured_envelope");
        let legacy: VehicleCalibration = serde_json::from_value(json).unwrap();
        let runtime = crate::Runtime::new(
            crate::replay::config(ExecutionMode::Replay),
            legacy.clone(),
            None,
        )
        .unwrap();
        assert!(
            runtime
                .capabilities()
                .iter()
                .any(|c| c.contains("synthetic body-offset fallback"))
        );
        let error = crate::Runtime::new(crate::replay::config(ExecutionMode::Live), legacy, None)
            .err()
            .unwrap();
        assert!(error.to_string().contains("missing measured tractor"));
        let measured = crate::replay::with_measured_envelope(original);
        measured.validate_measured_envelope().unwrap();
        let mut changed = measured.clone();
        let envelope = changed.geometry.measured_envelope.as_mut().unwrap();
        envelope.tractor_front_from_rear_axle_m += 0.4;
        envelope.tractor_rear_overhang_m -= 0.4;
        assert!(changed.geometry.valid());
        assert!(changed.validate_measured_envelope().is_err());
        let mut changed = measured.clone();
        changed
            .envelope_measurement
            .as_mut()
            .unwrap()
            .evidence_path
            .clear();
        assert!(changed.validate_measured_envelope().is_err());
        let mut changed = measured;
        changed.geometry.measured_envelope.as_mut().unwrap().trailer = None;
        assert!(!changed.geometry.valid());
    }
}
