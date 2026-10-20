//! Planner-side privileged obstacle decoding. Independent safety uses its own
//! raw parser and sweep implementation; this module never substitutes for AEB.
use crate::{ExecutionMode, prediction_bodies};
use etsads_game::{adapter::GameSnapshot, ets2la};
use etsads_msgs::{OrientedBox, Pose2};

pub(crate) fn validate_source(snapshot: &GameSnapshot, mode: ExecutionMode) -> Result<(), String> {
    if mode != ExecutionMode::Live {
        return Ok(());
    }
    let ext = snapshot
        .extension
        .as_ref()
        .ok_or("live parking requires privileged traffic and parked-object channels")?;
    let now = etsads_shm::qpc();
    let frequency = etsads_shm::qpc_frequency();
    if ext.available & 6 != 6
        || ext.header.frame_id != snapshot.ego.header.frame_id
        || ext.header.qpc == 0
        || ext.header.qpc > now
        || frequency == 0
        || (now - ext.header.qpc) as u128 * 1000 > frequency as u128 * 200
    {
        return Err(
            "live parking requires fresh same-frame traffic and parked-object channels".into(),
        );
    }
    Ok(())
}

/// Project every 3D box axis into an enclosing planar box, preserving tilt and
/// height contributions. Unknown vertical clearance cannot remove an obstacle.
fn body(position: [f32; 3], quaternion: [f32; 4], size: [f32; 3]) -> Result<OrientedBox, String> {
    if position.iter().any(|v| !v.is_finite())
        || size
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0 || *v > 100.0)
    {
        return Err("invalid raw parking-obstacle position or dimensions".into());
    }
    let yaw =
        ets2la::yaw_scs_quaternion(quaternion).ok_or("invalid raw parking-obstacle quaternion")?;
    let q = glam::DQuat::from_xyzw(
        quaternion[1] as f64,
        quaternion[2] as f64,
        quaternion[3] as f64,
        quaternion[0] as f64,
    )
    .normalize();
    let forward = glam::DVec2::new(yaw.cos(), yaw.sin());
    let left = glam::DVec2::new(-yaw.sin(), yaw.cos());
    let mut half_length = 0.0;
    let mut half_width = 0.0;
    for (axis, extent) in [glam::DVec3::X, glam::DVec3::Y, glam::DVec3::Z]
        .into_iter()
        .zip(size)
    {
        let world = q * axis;
        let planar = glam::DVec2::new(world.x, -world.z);
        half_length += planar.dot(forward).abs() * extent as f64 / 2.0;
        half_width += planar.dot(left).abs() * extent as f64 / 2.0;
    }
    let result = OrientedBox {
        pose: Pose2 {
            x: position[0] as f64,
            y: -position[2] as f64,
            yaw,
        },
        length: 2.0 * half_length,
        width: 2.0 * half_width,
    };
    result
        .valid()
        .then_some(result)
        .ok_or_else(|| "invalid projected parking-obstacle box".into())
}
fn with_motion(mut body: OrientedBox, speed: f64) -> Result<OrientedBox, String> {
    if !speed.is_finite() || speed.abs() > 80.0 {
        return Err("invalid raw parking-obstacle speed".into());
    }
    // Conservative occupation union over the next 2 s, refreshed every plan.
    // The controller stops when that union enters its upcoming swept path.
    let distance = speed * 2.0;
    body.pose.x += distance * body.pose.yaw.cos() / 2.0;
    body.pose.y += distance * body.pose.yaw.sin() / 2.0;
    body.length += distance.abs();
    body.valid().then_some(body).ok_or_else(|| {
        "moving parking-obstacle occupation exceeds supported geometry bounds".into()
    })
}
#[derive(Default)]
pub(crate) struct ParkingObstacles {
    pub current: Vec<OrientedBox>,
    pub occupancy: Vec<OrientedBox>,
}
impl ParkingObstacles {
    fn push(&mut self, object: OrientedBox, speed: f64) -> Result<(), String> {
        self.occupancy.push(with_motion(object, speed)?);
        self.current.push(object);
        Ok(())
    }
}
pub(crate) fn collect(snapshot: &GameSnapshot) -> Result<ParkingObstacles, String> {
    let mut result = ParkingObstacles::default();
    if let Some(ext) = snapshot.extension.as_ref().filter(|e| e.available & 2 != 0) {
        for value in ets2la::records::<ets2la::Traffic>(&ext.traffic) {
            if value.id < 0 || bytemuck::bytes_of(&value).iter().all(|b| *b == 0) {
                continue;
            }
            let count = value.trailer_count;
            if !(0..=3).contains(&count) {
                return Err("invalid raw traffic trailer count during parking".into());
            }
            let speed = value.speed as f64;
            result.push(
                body(value.position, value.quaternion_wxyz, value.size)?,
                speed,
            )?;
            for trailer in value.trailers.into_iter().take(count as usize) {
                result.push(
                    body(trailer.position, trailer.quaternion_wxyz, trailer.size)?,
                    speed,
                )?;
            }
        }
    } else {
        for agent in prediction_bodies(&snapshot.scene.agents) {
            result.push(
                OrientedBox {
                    pose: agent.pose,
                    length: agent.length,
                    width: agent.width,
                },
                agent.speed,
            )?;
        }
    }
    if let Some(ext) = snapshot.extension.as_ref().filter(|e| e.available & 4 != 0) {
        for value in ets2la::records::<ets2la::Parked>(&ext.parked) {
            if value.id < 0 || bytemuck::bytes_of(&value).iter().all(|b| *b == 0) {
                continue;
            }
            result.push(
                body(value.position, value.quaternion_wxyz, value.size)?,
                0.0,
            )?;
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn live_requires_both_fresh_channels_and_replay_can_use_scene() {
        let mut snapshot = crate::replay::snapshot(8);
        assert!(validate_source(&snapshot, ExecutionMode::Live).is_err());
        assert!(validate_source(&snapshot, ExecutionMode::Replay).is_ok());
        let mut ext = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        ext.header.frame_id = 8;
        ext.header.qpc = etsads_shm::qpc();
        ext.available = 2;
        snapshot.extension = Some(ext);
        assert!(validate_source(&snapshot, ExecutionMode::Live).is_err());
        snapshot.extension.as_mut().unwrap().available = 6;
        assert!(validate_source(&snapshot, ExecutionMode::Live).is_ok());
        snapshot.extension.as_mut().unwrap().header.frame_id = 7;
        assert!(validate_source(&snapshot, ExecutionMode::Live).is_err());
        snapshot.extension.as_mut().unwrap().header.frame_id = 8;
        snapshot.extension.as_mut().unwrap().header.qpc =
            etsads_shm::qpc() + etsads_shm::qpc_frequency();
        assert!(validate_source(&snapshot, ExecutionMode::Live).is_err());
    }
    #[test]
    fn raw_parked_object_is_used_and_invalid_nonempty_record_is_not_discarded() {
        let mut snapshot = crate::replay::snapshot(2);
        let mut ext = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        ext.available = 6;
        let parked = ets2la::Parked {
            position: [10.0, 3.0, -5.0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            size: [2.5, 4.0, 12.0],
            id: 1,
            is_trailer: 1,
        };
        ext.parked[..43].copy_from_slice(bytemuck::bytes_of(&parked));
        snapshot.extension = Some(ext);
        let objects = collect(&snapshot).unwrap();
        assert_eq!(objects.current.len(), 1);
        assert_eq!(
            (objects.current[0].pose.x, objects.current[0].pose.y),
            (10.0, 5.0)
        );
        assert!((objects.current[0].length - 12.0).abs() < 1e-6);
        assert_eq!(objects.current, objects.occupancy);
        snapshot.extension.as_mut().unwrap().parked[12..16].copy_from_slice(&0.0_f32.to_le_bytes());
        assert!(collect(&snapshot).is_err());
    }
    #[test]
    fn runtime_parking_rejects_raw_parked_body_even_when_fused_scene_is_empty() {
        use crate::{
            DrivingMode, Runtime, RuntimeStatus,
            parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary},
        };
        let mut config = crate::replay::config(ExecutionMode::Replay);
        config.mode = DrivingMode::Parking {
            mission: ParkingMission {
                target: ParkingTarget {
                    source_id: "synthetic-raw-obstacle-test".into(),
                    pose: Pose2 {
                        x: -18.0,
                        y: 0.0,
                        yaw: 0.0,
                    },
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
                    evidence_path: "synthetic-boundary".into(),
                },
                static_obstacles: vec![],
                max_expansions: 1000,
                maximum_duration_s: 60.0,
                recovery: Default::default(),
            },
        };
        let mut runtime =
            Runtime::new(config, crate::replay::low_speed_calibration(), None).unwrap();
        let mut output = None;
        for i in 0..2 {
            let mut frame = crate::replay::snapshot(i);
            frame.ego.speed = 0.0;
            frame.scene.ego = frame.ego;
            let mut ext = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
            ext.available = 6;
            ext.header = frame.raw.header;
            let parked = ets2la::Parked {
                position: [0.0, 0.0, 0.0],
                quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
                size: [2.5, 4.0, 12.0],
                id: 1,
                is_trailer: 1,
            };
            ext.parked[..43].copy_from_slice(bytemuck::bytes_of(&parked));
            frame.extension = Some(ext);
            assert!(frame.scene.agents.is_empty());
            output = Some(runtime.step(&frame));
        }
        let output = output.unwrap();
        assert_eq!(output.status, RuntimeStatus::Fault);
        assert!(
            output.reason.contains("envelope intersects"),
            "{}",
            output.reason
        );
        assert_eq!(output.actuation.throttle, 0.0);
    }
    #[test]
    fn approaching_traffic_brakes_instead_of_reporting_a_future_overlap_as_a_current_collision() {
        use crate::parking_mission::{
            ParkingEngine, ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary,
        };
        let mut engine = ParkingEngine::new(
            ParkingMission {
                target: ParkingTarget {
                    source_id: "synthetic-incoming-traffic".into(),
                    pose: Pose2 {
                        x: -18.0,
                        y: 0.0,
                        yaw: 0.0,
                    },
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
                    evidence_path: "synthetic-boundary".into(),
                },
                static_obstacles: vec![],
                max_expansions: 1000,
                maximum_duration_s: 60.0,
                recovery: Default::default(),
            },
            etsads_msgs::VehicleGeometry::default(),
        )
        .unwrap();
        let current = OrientedBox {
            pose: Pose2 {
                x: 15.0,
                y: 0.0,
                yaw: std::f64::consts::PI,
            },
            length: 4.0,
            width: 2.5,
        };
        let forecast = with_motion(current, 10.0).unwrap();
        let (trajectory, intent) = engine
            .update(
                etsads_msgs::EgoState::default(),
                0,
                false,
                &[current],
                &[forecast],
            )
            .unwrap();
        assert!(intent.hold_brake);
        assert!(intent.reason.contains("incoming obstacle"));
        assert!(trajectory.points.iter().all(|p| p.speed == 0.0));
    }
}
