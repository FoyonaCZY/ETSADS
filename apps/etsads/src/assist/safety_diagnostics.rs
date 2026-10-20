//! Explain the independent supervisor's unchanged raw-box policy. Each
//! candidate is evaluated by that policy itself, not a second collision model.
use etsads_msgs::protocol::{ExtensionFrame, TelemetryFrame};
use etsads_safety::primary_obstacle;
use serde_json::{Value, json};

pub(super) fn inspect(ego: &TelemetryFrame, extension: &ExtensionFrame) -> Value {
    match candidates(ego, extension) {
        Ok(value) => value,
        Err(error) => json!({"error":error}),
    }
}
fn candidates(ego: &TelemetryFrame, extension: &ExtensionFrame) -> Result<Value, &'static str> {
    let selected = primary_obstacle(ego, extension)?;
    let mut isolated = Box::<ExtensionFrame>::default();
    isolated.header = extension.header;
    isolated.available = extension.available & 6;
    let mut observations = Vec::<(f32, Value)>::new();
    for (parked, bytes, stride, id_offset) in [
        (false, extension.traffic.as_slice(), 174, 50),
        (true, extension.parked.as_slice(), 43, 40),
    ] {
        if parked && extension.available & 4 == 0 {
            continue;
        }
        for record in bytes.chunks_exact(stride) {
            let id = i16::from_le_bytes([record[id_offset], record[id_offset + 1]]);
            if id < 0 || record.iter().all(|v| *v == 0) {
                continue;
            }
            if parked {
                isolated.parked[..stride].copy_from_slice(record);
            } else {
                isolated.traffic[..stride].copy_from_slice(record);
            }
            if let Some(obstacle) = primary_obstacle(ego, &isolated)? {
                let f = |offset: usize| {
                    f32::from_le_bytes(record[offset..offset + 4].try_into().unwrap())
                };
                let risk = obstacle.distance_m.max(0.01) / obstacle.closing_speed_mps.max(0.1);
                observations.push((
                    risk,
                    json!({"object_id":id,"parked":parked,"includes_trailers":!parked,
                    "position_scs":[f(0),f(4),f(8)],"size_xyz_m":[f(28),f(32),f(36)],
                    "distance_m":obstacle.distance_m,"closing_speed_mps":obstacle.closing_speed_mps,
                    "center_height_difference_m":f(4) as f64-ego.position_scs[1]}),
                ));
            }
            if parked {
                isolated.parked[..stride].fill(0);
            } else {
                isolated.traffic[..stride].fill(0);
            }
        }
    }
    observations.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(
        json!({"policy":"independent_primary_obstacle","physics_frame":ego.header.frame_id,"extension_frame":extension.header.frame_id,
        "ego_heading_turns":ego.orientation_turns[0],"ego_yaw_rate_turns_s":ego.angular_velocity_local[1],
        "selected":observations.first().map(|o|&o.1),"aggregate":selected.map(|o|json!({"distance_m":o.distance_m,"closing_speed_mps":o.closing_speed_mps})),
        "candidates":observations.into_iter().map(|o|o.1).collect::<Vec<_>>(),"height_filter":"none_without_verified_body_profile"}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use bytemuck::Zeroable;
    #[test]
    fn identifies_raw_obstacle_with_the_same_result_as_supervisor() {
        let mut ego = TelemetryFrame {
            qpc_frequency: 1000,
            speed_mps: 5.0,
            ..Default::default()
        };
        ego.header.qpc = 100;
        ego.header.frame_id = 10;
        let mut extension = Box::<ExtensionFrame>::default();
        extension.header = ego.header;
        extension.available = 6;
        let mut actor = etsads_game::ets2la::Traffic::zeroed();
        actor.id = 11;
        actor.position = [0.0, 1.0, -15.0];
        actor.quaternion_wxyz = [1.0, 0.0, 0.0, 0.0];
        actor.size = [2.0, 2.0, 4.0];
        extension.traffic[..174].copy_from_slice(bytemuck::bytes_of(&actor));
        let value = inspect(&ego, &extension);
        assert_eq!(value["selected"]["object_id"], 11);
        assert_eq!(
            value["selected"]["distance_m"],
            value["aggregate"]["distance_m"]
        );
        assert_eq!(
            value["selected"]["closing_speed_mps"],
            value["aggregate"]["closing_speed_mps"]
        );
        assert!((value["selected"]["distance_m"].as_f64().unwrap() - 9.8).abs() < 1e-5);
        actor.position[0] = 20.0;
        extension.traffic[..174].copy_from_slice(bytemuck::bytes_of(&actor));
        assert!(inspect(&ego, &extension)["selected"].is_null());
    }
}
