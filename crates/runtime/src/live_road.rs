//! Host-verified engineering evidence for road operation, separate from the
//! commissioning speed policy and from advanced-manoeuvre experiments.
use crate::{ExecutionMode, RouteSource, RuntimeConfig, RuntimeError};
use etsads_control::identification::VehicleCalibration;
use sha2::{Digest, Sha256};

#[derive(Clone, Debug)]
pub struct VerifiedRoadOperation {
    binding: [u8; 32],
    pub(crate) scope: [u8; 32],
    evidence_sha256: [u8; 32],
}
impl VerifiedRoadOperation {
    /// The host first verifies current map/policy/calibration hashes and the
    /// complete source-game recordings and road measurement artifacts. This
    /// capability contains no stage-passed flag and cannot be deserialized.
    pub fn from_host_verification(
        config: &RuntimeConfig,
        calibration: &VehicleCalibration,
        scope: [u8; 32],
        evidence_sha256: [u8; 32],
    ) -> Result<Self, RuntimeError> {
        if config.execution != ExecutionMode::Live
            || !matches!(&config.route_source,RouteSource::NativeGps(s) if s.road_evidence.is_some())
            || scope == [0; 32]
            || evidence_sha256 == [0; 32]
        {
            return Err(RuntimeError::Configuration("road permission requires exact live road configuration and verified nonzero source/evidence digests".into()));
        }
        calibration
            .road_speed_range()
            .map_err(|e| RuntimeError::Configuration(e.to_string()))?;
        Ok(Self {
            binding: binding(config, calibration, scope)?,
            scope,
            evidence_sha256,
        })
    }
    pub(crate) fn validates(
        &self,
        config: &RuntimeConfig,
        calibration: &VehicleCalibration,
    ) -> Result<(), RuntimeError> {
        if config.execution != ExecutionMode::Live
            || self.evidence_sha256 == [0; 32]
            || self.binding != binding(config, calibration, self.scope)?
        {
            return Err(RuntimeError::Configuration(
                "road evidence belongs to a different configuration or calibration".into(),
            ));
        }
        Ok(())
    }
}
fn binding(
    config: &RuntimeConfig,
    calibration: &VehicleCalibration,
    scope: [u8; 32],
) -> Result<[u8; 32], RuntimeError> {
    let bytes = serde_json::to_vec(&("ETSADS_LIVE_ROAD_V1", config, calibration, scope))
        .map_err(|e| RuntimeError::Configuration(e.to_string()))?;
    Ok(Sha256::digest(bytes).into())
}
