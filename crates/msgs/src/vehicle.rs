//! Pure physical-configuration identity shared by calibration and the isolated
//! supervisor. No planner/controller dependency is introduced.
use crate::protocol::ConfigBlock;
use sha2::{Digest, Sha256};
pub fn geometry_fingerprint(config: &ConfigBlock) -> Result<String, &'static str> {
    if config.wheel_count < 2 || config.wheel_count > 16 {
        return Err("vehicle wheel count unavailable");
    }
    let mut hash = Sha256::new();
    hash.update(b"ETSADS_GEOMETRY_V1");
    hash.update(config.wheel_count.to_le_bytes());
    for i in 0..config.wheel_count as usize {
        for value in config.wheel_positions[i] {
            if !value.is_finite() || value.abs() > 50.0 {
                return Err("invalid wheel geometry");
            }
            hash.update(value.to_le_bytes());
        }
        let radius = config.wheel_radius[i];
        if !radius.is_finite() || radius <= 0.0 || radius > 3.0 {
            return Err("wheel radius unavailable");
        }
        hash.update(radius.to_le_bytes());
        hash.update(config.wheel_steerable[i].to_le_bytes());
    }
    for value in config.hook_position {
        if !value.is_finite() || value.abs() > 50.0 {
            return Err("invalid hitch geometry");
        }
        hash.update(value.to_le_bytes());
    }
    Ok(format!("{:x}", hash.finalize()))
}
