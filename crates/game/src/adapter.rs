use crate::ets2la;
use etsads_msgs::{
    model::{Agent, EgoState, Header, OrientedBox, Pose2, Scene},
    protocol::*,
};
use etsads_shm::{SharedCell, SharedRing};

#[derive(Clone)]
pub struct GameSnapshot {
    pub ego: EgoState,
    pub raw: TelemetryFrame,
    pub extension: Option<Box<ExtensionFrame>>,
    pub config: Option<ConfigBlock>,
    pub events: Vec<GameEvent>,
    pub scene: Scene,
    pub local_origin: [f64; 3],
    pub origin_generation: u64,
    pub session_start_qpc: u64,
}
#[derive(Default)]
pub struct SnapshotSidecars {
    pub control_configuration: Option<ControlConfiguration>,
    pub dynamics: Option<DynamicsFrame>,
    pub configuration: Option<Box<ConfigurationFrame>>,
    pub camera_view: Option<CameraViewFrame>,
}
#[derive(Default)]
pub struct SidecarReader {
    control_configuration: Option<SharedRing<ControlConfiguration, 64>>,
    dynamics: Option<SharedRing<DynamicsFrame, 64>>,
    configuration: Option<SharedRing<ConfigurationFrame, 64>>,
    camera_view: Option<SharedRing<CameraViewFrame, 64>>,
}
impl SidecarReader {
    pub fn connect() -> Self {
        Self {
            control_configuration: SharedRing::open(CONTROL_CONFIGURATION_MAPPING, SCHEMA_VERSION)
                .ok(),
            dynamics: SharedRing::open(DYNAMICS_MAPPING, SCHEMA_VERSION).ok(),
            configuration: SharedRing::open(CONFIG_FULL_MAPPING, SCHEMA_VERSION).ok(),
            camera_view: SharedRing::open(CAMERA_VIEW_MAPPING, SCHEMA_VERSION).ok(),
        }
    }
    pub fn read(&self, frame: &TelemetryFrame, session_start_qpc: u64) -> SnapshotSidecars {
        let matches = |header: WireHeader| {
            header.frame_id == frame.header.frame_id
                && header.qpc == frame.header.qpc
                && header.sim_time_us == frame.header.sim_time_us
                && header.qpc >= session_start_qpc
        };
        let dynamics = self.dynamics.as_ref().and_then(|ring| {
            let end = ring.latest_index();
            (end.saturating_sub(63).max(1)..=end)
                .rev()
                .filter_map(|i| ring.read(i))
                .find(|value| matches(value.header))
        });
        let camera_view = self.camera_view.as_ref().and_then(|ring| {
            let end = ring.latest_index();
            (end.saturating_sub(63).max(1)..=end)
                .rev()
                .filter_map(|i| ring.read(i))
                .find(|value| matches(value.header) && value.session_start_qpc == session_start_qpc)
        });
        let configuration = self.configuration_for(frame, session_start_qpc);
        SnapshotSidecars {
            control_configuration: self.control_configuration_for(frame, session_start_qpc),
            dynamics,
            configuration,
            camera_view,
        }
    }
    /// Historical association only. Control authorization must additionally
    /// inspect the newest SDK generation to revoke a superseded safe mode.
    pub fn control_configuration_for(
        &self,
        frame: &TelemetryFrame,
        session_start_qpc: u64,
    ) -> Option<ControlConfiguration> {
        self.control_configuration.as_ref().and_then(|ring| {
            let end = ring.latest_index();
            (end.saturating_sub(63).max(1)..=end)
                .rev()
                .filter_map(|index| ring.read(index))
                .find(|value| {
                    value.session_start_qpc == session_start_qpc
                        && value.header.qpc >= session_start_qpc
                        && value.header.qpc <= frame.header.qpc
                        && value.effective_from_frame_id <= frame.header.frame_id
                })
        })
    }
    fn configuration_for(
        &self,
        frame: &TelemetryFrame,
        session_start_qpc: u64,
    ) -> Option<Box<ConfigurationFrame>> {
        self.configuration.as_ref().and_then(|ring| {
            let end = ring.latest_index();
            let mut value = etsads_shm::zeroed_box::<ConfigurationFrame>();
            for index in (end.saturating_sub(63).max(1)..=end).rev() {
                if ring.read_into(index, &mut value)
                    && value.effective_from_frame_id <= frame.header.frame_id
                    && value.header.qpc <= frame.header.qpc
                    && value.header.qpc >= session_start_qpc
                {
                    return Some(value);
                }
            }
            None
        })
    }
}
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Transport(#[from] etsads_shm::Error),
    #[error("invalid telemetry: {0}")]
    Invalid(&'static str),
}
pub struct GameAdapter {
    telemetry: SharedRing<TelemetryFrame, TELEMETRY_SLOTS>,
    extension: Option<SharedRing<ExtensionFrame, 4>>,
    config: Option<SharedCell<ConfigBlock>>,
    events: Option<SharedRing<GameEvent, EVENT_SLOTS>>,
    info: Option<SharedCell<BridgeInfo>>,
    session_start_qpc: u64,
    last_frame: u64,
    last_time: u64,
    last_event: u64,
    origin: [f64; 3],
    origin_generation: u64,
    pub invalid_count: u64,
    /// Consecutive frames whose clock lies before the last accepted one.
    regressions: u32,
    /// Set true when planning requires privileged traffic/camera state. When
    /// false, telemetry-only operation is explicit and `extension` is None.
    pub require_extension: bool,
    sidecars: SidecarReader,
}
impl GameAdapter {
    pub fn connect() -> Result<Self, Error> {
        Ok(Self {
            telemetry: SharedRing::open(TELEMETRY_MAPPING, SCHEMA_VERSION)?,
            extension: SharedRing::open(EXTENSION_MAPPING, SCHEMA_VERSION).ok(),
            config: SharedCell::open(CONFIG_MAPPING, SCHEMA_VERSION).ok(),
            events: SharedRing::open(EVENTS_MAPPING, SCHEMA_VERSION).ok(),
            info: SharedCell::open(INFO_MAPPING, SCHEMA_VERSION).ok(),
            session_start_qpc: 0,
            last_frame: 0,
            last_time: 0,
            last_event: 0,
            origin: [0.0; 3],
            origin_generation: 0,
            invalid_count: 0,
            regressions: 0,
            require_extension: false,
            sidecars: SidecarReader::connect(),
        })
    }
    /// Missing old-DLL sidecars stay None; values from a different frame/session
    /// are never silently substituted for the snapshot being consumed.
    pub fn sidecars(&self, frame: &TelemetryFrame) -> SnapshotSidecars {
        self.sidecars.read(frame, self.session_start_qpc)
    }
    pub fn poll(&mut self) -> Result<Option<GameSnapshot>, Error> {
        if let Some(info) = self.info.as_ref().and_then(|r| r.latest()) {
            if info.telemetry_api_version == 0 {
                return Ok(None);
            }
            if info.session_start_qpc != self.session_start_qpc {
                self.session_start_qpc = info.session_start_qpc;
                self.last_frame = 0;
                self.last_time = 0;
                self.last_event = 0;
                self.origin_generation = 0;
            }
        }
        let Some(mut raw) = self.telemetry.latest() else {
            return Ok(None);
        };
        if raw.header.qpc < self.session_start_qpc {
            return Ok(None);
        }
        let extension = self
            .extension
            .as_ref()
            .and_then(|ring| ring.latest_boxed())
            .filter(|e| e.available != 0 && e.header.qpc >= self.session_start_qpc);
        if let Some(extension) = &extension {
            let last = self.telemetry.latest_index();
            let matched = (last.saturating_sub(TELEMETRY_SLOTS as u64 - 1).max(1)..=last)
                .rev()
                .filter_map(|i| self.telemetry.read(i))
                .find(|t| t.header.frame_id == extension.header.frame_id);
            let Some(matched) = matched else {
                return Ok(None);
            };
            raw = matched;
        } else if self.require_extension {
            return Ok(None);
        }
        // A frame older than the last one used (the extension can match an
        // older telemetry frame after a frame without extension data) is
        // stale, not a clock regression (L2 exited on it).
        if raw.header.frame_id <= self.last_frame {
            return Ok(None);
        }
        let ego = ego_from_telemetry(&raw);
        let regression =
            self.last_time > raw.paused_simulation_time_us && raw.header.flags & 1 == 0;
        if !ego.valid() || raw.speed_mps.abs() > 70.0 || raw.qpc_frequency == 0 || regression {
            self.invalid_count += 1;
            // A clock that stays behind for a second of frames is the new
            // timeline (a game-side reset), not a transient glitch.
            if regression {
                self.regressions += 1;
                if self.regressions >= 60 {
                    self.last_time = raw.paused_simulation_time_us;
                    self.regressions = 0;
                }
            }
            return Err(Error::Invalid(
                "nonfinite state, speed, clock or time regression",
            ));
        }
        // Extension bridge copies the previous physical frame at next frame_start.
        // Missing extension is explicit; never combine it with another frame ID.
        self.last_frame = raw.header.frame_id;
        self.last_time = raw.paused_simulation_time_us;
        self.regressions = 0;
        if self.origin_generation == 0
            || (ego.pose.x - self.origin[0]).hypot(ego.pose.y - self.origin[1]) > 2000.0
        {
            self.origin = [
                (ego.pose.x / 1000.0).floor() * 1000.0 + 500.0,
                (ego.pose.y / 1000.0).floor() * 1000.0 + 500.0,
                0.0,
            ];
            self.origin_generation += 1;
        }
        let scene = scene_from_data(ego, &raw, extension.as_deref());
        let mut events = Vec::new();
        if let Some(ring) = &self.events {
            let latest = ring.latest_index();
            for i in self
                .last_event
                .saturating_add(1)
                .max(latest.saturating_sub(EVENT_SLOTS as u64 - 1))..=latest
            {
                if let Some(e) = ring.read(i) {
                    if e.header.qpc >= self.session_start_qpc
                        && (e.header.frame_id > raw.header.frame_id
                            || e.header.qpc > raw.header.qpc)
                    {
                        break;
                    }
                    if e.header.qpc >= self.session_start_qpc {
                        events.push(e);
                    }
                    self.last_event = i;
                }
            }
        }
        // New bridges carry exact SDK event boundaries. A missing retained
        // boundary is unknown, not permission to use a newer legacy snapshot.
        let config = if self.sidecars.configuration.is_some() {
            self.sidecars
                .configuration_for(&raw, self.session_start_qpc)
                .map(|full| full.truck)
        } else {
            self.config.as_ref().and_then(|x| x.latest())
        };
        Ok(Some(GameSnapshot {
            ego,
            raw,
            extension,
            config,
            events,
            scene,
            local_origin: self.origin,
            origin_generation: self.origin_generation,
            session_start_qpc: self.session_start_qpc,
        }))
    }
}
/// Convert recorded wire data with the same units and scene conversion as Live.
/// This pure function never connects to a mapping. Recording event streams and
/// stateful origin continuity are the replay caller's responsibility.
pub fn snapshot_from_recorded_frame(
    raw: TelemetryFrame,
    extension: Option<Box<ExtensionFrame>>,
    config: Option<ConfigBlock>,
) -> Result<GameSnapshot, Error> {
    let ego = ego_from_telemetry(&raw);
    if !ego.valid() || raw.speed_mps.abs() > 70.0 || raw.qpc_frequency == 0 {
        return Err(Error::Invalid("nonfinite state, speed or clock"));
    }
    if extension.as_ref().is_some_and(|e| {
        e.header.frame_id != raw.header.frame_id || e.header.sim_time_us != raw.header.sim_time_us
    }) {
        return Err(Error::Invalid(
            "extension belongs to another physical frame",
        ));
    }
    let scene = scene_from_data(ego, &raw, extension.as_deref());
    Ok(GameSnapshot {
        ego,
        raw,
        extension,
        config,
        events: Vec::new(),
        scene,
        local_origin: [
            (ego.pose.x / 1000.0).floor() * 1000.0 + 500.0,
            (ego.pose.y / 1000.0).floor() * 1000.0 + 500.0,
            0.0,
        ],
        origin_generation: 1,
        session_start_qpc: 0,
    })
}
fn scene_from_data(
    ego: EgoState,
    raw: &TelemetryFrame,
    extension: Option<&ExtensionFrame>,
) -> Scene {
    let mut scene = Scene {
        ego,
        speed_limit: raw.navigation_speed_limit_mps as f64,
        ..Default::default()
    };
    if let Some(ext) = extension {
        if ext.available & 2 != 0 {
            for item in ets2la::records::<ets2la::Traffic>(&ext.traffic) {
                if item.id < 0 || bytemuck::bytes_of(&item).iter().all(|b| *b == 0) {
                    continue;
                }
                let Some(yaw) = ets2la::yaw_scs_quaternion(item.quaternion_wxyz) else {
                    continue;
                };
                let mut trailers = Vec::new();
                for trailer in item
                    .trailers
                    .into_iter()
                    .take(item.trailer_count.clamp(0, 3) as usize)
                {
                    if let Some(yaw) = ets2la::yaw_scs_quaternion(trailer.quaternion_wxyz) {
                        trailers.push(OrientedBox {
                            pose: Pose2 {
                                x: trailer.position[0] as f64,
                                y: -trailer.position[2] as f64,
                                yaw,
                            },
                            length: trailer.size[2] as f64,
                            width: trailer.size[0] as f64,
                        });
                    }
                }
                let agent = Agent {
                    id: item.id as u64,
                    pose: Pose2 {
                        x: item.position[0] as f64,
                        y: -item.position[2] as f64,
                        yaw,
                    },
                    z: item.position[1] as f64,
                    speed: item.speed as f64,
                    // A collision spikes the reported acceleration (L2: −77 m/s²);
                    // the body is still there and must stay an obstacle.
                    acceleration: if item.acceleration.is_finite() {
                        (item.acceleration as f64).clamp(-30.0, 30.0)
                    } else {
                        0.0
                    },
                    length: item.size[2] as f64,
                    width: item.size[0] as f64,
                    lane_id: None,
                    trailers,
                };
                if agent.valid() {
                    scene.agents.push(agent);
                }
            }
        }
        if ext.available & 16 != 0 {
            scene.route = ets2la::records::<ets2la::RouteNode>(&ext.route)
                .filter_map(|r| {
                    if r.node_uid != 0 {
                        Some(r.node_uid as u64)
                    } else {
                        None
                    }
                })
                .collect();
        }
    }
    scene
}
pub fn ego_from_telemetry(raw: &TelemetryFrame) -> EgoState {
    let yaw = etsads_msgs::frames::heading_to_yaw(raw.orientation_turns[0] as f64);
    let trailer_yaw = raw
        .trailers
        .iter()
        .find(|t| t.connected != 0)
        .map(|t| etsads_msgs::frames::heading_to_yaw(t.orientation_turns[0] as f64))
        .unwrap_or(yaw);
    EgoState {
        header: Header {
            sim_time_us: raw.paused_simulation_time_us,
            frame_id: raw.header.frame_id,
            seq: raw.header.seq,
        },
        pose: Pose2 {
            x: raw.position_scs[0],
            y: -raw.position_scs[2],
            yaw,
        },
        z: raw.position_scs[1],
        speed: raw.speed_mps as f64,
        acceleration: -raw.acceleration_local[2] as f64,
        yaw_rate: raw.angular_velocity_local[1] as f64 * std::f64::consts::TAU,
        wheel_angle: raw.wheel_steering[0] as f64 * std::f64::consts::TAU,
        trailer_yaw,
        paused: raw.paused != 0,
    }
}

#[cfg(test)]
mod recorded_tests {
    use super::*;

    #[test]
    fn recorded_snapshot_uses_live_coordinates_units_and_frame_checks() {
        let mut raw = TelemetryFrame {
            position_scs: [1250.0, 6.0, -2150.0],
            qpc_frequency: 10_000_000,
            paused_simulation_time_us: 99,
            ..Default::default()
        };
        raw.header.frame_id = 7;
        raw.header.sim_time_us = 100;
        raw.angular_velocity_local[1] = 0.25;
        raw.wheel_steering[0] = 0.125;
        let mut ext = Box::<ExtensionFrame>::default();
        ext.header = raw.header;
        let snapshot = snapshot_from_recorded_frame(raw, Some(ext.clone()), None).unwrap();
        assert_eq!(snapshot.ego.pose.x, 1250.0);
        assert_eq!(snapshot.ego.pose.y, 2150.0);
        assert_eq!(snapshot.local_origin, [1500.0, 2500.0, 0.0]);
        assert_eq!(snapshot.ego.header.sim_time_us, 99);
        assert_eq!(snapshot.ego.yaw_rate, std::f64::consts::FRAC_PI_2);
        assert_eq!(snapshot.ego.wheel_angle, std::f64::consts::FRAC_PI_4);
        ext.header.frame_id += 1;
        assert!(snapshot_from_recorded_frame(raw, Some(ext), None).is_err());
        raw.position_scs[0] = f64::NAN;
        assert!(snapshot_from_recorded_frame(raw, None, None).is_err());
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    #[test]
    fn sidecar_reader_matches_physical_frame_and_rejects_future_configuration() {
        let suffix = format!("{}_{}", std::process::id(), etsads_shm::qpc());
        let dynamics =
            SharedRing::create(&format!("ETSADS_Sidecars_D_{suffix}"), SCHEMA_VERSION).unwrap();
        let configuration =
            SharedRing::create(&format!("ETSADS_Sidecars_C_{suffix}"), SCHEMA_VERSION).unwrap();
        let camera_view =
            SharedRing::create(&format!("ETSADS_Sidecars_V_{suffix}"), SCHEMA_VERSION).unwrap();
        let reader = SidecarReader {
            control_configuration: Some(
                SharedRing::create(
                    &format!("ETSADS_Sidecars_Controls_{suffix}"),
                    SCHEMA_VERSION,
                )
                .unwrap(),
            ),
            dynamics: Some(dynamics),
            configuration: Some(configuration),
            camera_view: Some(camera_view),
        };
        let raw = TelemetryFrame {
            header: WireHeader {
                frame_id: 2,
                qpc: 200,
                sim_time_us: 2000,
                ..Default::default()
            },
            ..Default::default()
        };
        let controls = reader.control_configuration.as_ref().unwrap();
        let safe = ControlConfiguration {
            header: WireHeader {
                qpc: 100,
                seq: 1,
                ..Default::default()
            },
            session_start_qpc: 100,
            generation: 1,
            effective_from_frame_id: 1,
            shifter_type: SHIFTER_AUTOMATIC,
            valid: 1,
        };
        controls.publish(&safe).unwrap();
        controls
            .publish(&ControlConfiguration {
                header: WireHeader {
                    qpc: 201,
                    frame_id: 1,
                    seq: 2,
                    ..Default::default()
                },
                generation: 2,
                effective_from_frame_id: 2,
                valid: 0,
                shifter_type: SHIFTER_UNKNOWN,
                ..safe
            })
            .unwrap();
        assert_eq!(
            reader
                .control_configuration_for(&raw, 100)
                .unwrap()
                .generation,
            1
        );
        assert!(reader.control_configuration_for(&raw, 101).is_none());
        let mut next = raw;
        next.header.frame_id = 3;
        next.header.qpc = 300;
        assert_eq!(
            reader
                .control_configuration_for(&next, 100)
                .unwrap()
                .generation,
            2
        );
        reader
            .dynamics
            .as_ref()
            .unwrap()
            .publish(&DynamicsFrame {
                header: raw.header,
                valid: DYNAMICS_ANGULAR_ACCELERATION,
                ..Default::default()
            })
            .unwrap();
        reader
            .dynamics
            .as_ref()
            .unwrap()
            .publish(&DynamicsFrame {
                header: WireHeader {
                    frame_id: 3,
                    qpc: 300,
                    ..raw.header
                },
                ..Default::default()
            })
            .unwrap();
        reader
            .configuration
            .as_ref()
            .unwrap()
            .publish(&ConfigurationFrame {
                header: WireHeader {
                    frame_id: 1,
                    qpc: 150,
                    ..Default::default()
                },
                effective_from_frame_id: 2,
                config_seq: 1,
                ..Default::default()
            })
            .unwrap();
        reader
            .configuration
            .as_ref()
            .unwrap()
            .publish(&ConfigurationFrame {
                header: WireHeader {
                    frame_id: 1,
                    qpc: 250,
                    ..Default::default()
                },
                effective_from_frame_id: 2,
                config_seq: 2,
                ..Default::default()
            })
            .unwrap();
        reader
            .camera_view
            .as_ref()
            .unwrap()
            .publish(&CameraViewFrame {
                header: raw.header,
                session_start_qpc: 50,
                ..Default::default()
            })
            .unwrap();
        let matched = reader.read(&raw, 50);
        assert_eq!(matched.dynamics.unwrap().header.frame_id, 2);
        assert_eq!(matched.configuration.unwrap().config_seq, 1);
        assert_eq!(matched.camera_view.unwrap().header.qpc, 200);
        assert!(reader.read(&raw, 100).camera_view.is_none());
        let legacy = SidecarReader::default().read(&raw, 0);
        assert!(
            legacy.dynamics.is_none()
                && legacy.configuration.is_none()
                && legacy.camera_view.is_none()
        );
    }
    #[test]
    fn aligns_previous_extension_with_retained_telemetry_and_freezes_clock() {
        let suffix = format!("{}_{}", std::process::id(), etsads_shm::qpc());
        let telemetry =
            SharedRing::create(&format!("ETSADS_Adapter_T_{suffix}"), SCHEMA_VERSION).unwrap();
        let extension =
            SharedRing::create(&format!("ETSADS_Adapter_E_{suffix}"), SCHEMA_VERSION).unwrap();
        let events =
            SharedRing::create(&format!("ETSADS_Adapter_Events_{suffix}"), SCHEMA_VERSION).unwrap();
        let config =
            SharedCell::create(&format!("ETSADS_Adapter_Config_{suffix}"), SCHEMA_VERSION).unwrap();
        config
            .publish(&ConfigBlock {
                config_seq: 2,
                ..Default::default()
            })
            .unwrap();
        let full =
            SharedRing::create(&format!("ETSADS_Adapter_Full_{suffix}"), SCHEMA_VERSION).unwrap();
        for seq in 1..=2 {
            full.publish(&ConfigurationFrame {
                header: WireHeader {
                    qpc: seq * 100 - 50,
                    ..Default::default()
                },
                effective_from_frame_id: seq,
                config_seq: seq,
                truck: ConfigBlock {
                    config_seq: seq,
                    ..Default::default()
                },
                ..Default::default()
            })
            .unwrap();
        }
        let mut adapter = GameAdapter {
            telemetry,
            extension: Some(extension),
            config: Some(config),
            events: Some(events),
            info: None,
            session_start_qpc: 0,
            last_frame: 0,
            last_time: 0,
            last_event: 0,
            origin: [0.0; 3],
            origin_generation: 0,
            invalid_count: 0,
            regressions: 0,
            require_extension: true,
            sidecars: SidecarReader {
                configuration: Some(full),
                ..Default::default()
            },
        };
        let mut first = TelemetryFrame {
            qpc_frequency: 1000,
            paused_simulation_time_us: 10,
            ..Default::default()
        };
        first.header.frame_id = 1;
        first.header.sim_time_us = 100;
        first.header.qpc = 100;
        let mut second = first;
        second.header.frame_id = 2;
        second.header.sim_time_us = 200;
        second.header.qpc = 200;
        second.paused = 1;
        adapter.telemetry.publish(&first).unwrap();
        adapter.telemetry.publish(&second).unwrap();
        let ext = ExtensionFrame {
            header: first.header,
            available: 2,
            ..Default::default()
        };
        adapter.extension.as_ref().unwrap().publish(&ext).unwrap();
        adapter
            .events
            .as_ref()
            .unwrap()
            .publish(&GameEvent {
                header: WireHeader {
                    qpc: 150,
                    ..first.header
                },
                ..Default::default()
            })
            .unwrap();
        let snapshot = adapter.poll().unwrap().unwrap();
        assert_eq!(snapshot.raw.header.frame_id, 1);
        assert_eq!(snapshot.config.unwrap().config_seq, 1);
        assert_eq!(snapshot.extension.unwrap().header.frame_id, 1);
        assert_eq!(snapshot.ego.header.sim_time_us, 10);
        assert!(
            snapshot.events.is_empty(),
            "gameplay events after frame_start must wait for the next aligned ego sample"
        );
        assert!(adapter.poll().unwrap().is_none());
        adapter
            .extension
            .as_ref()
            .unwrap()
            .publish(&ExtensionFrame {
                header: second.header,
                ..ext
            })
            .unwrap();
        let snapshot = adapter.poll().unwrap().unwrap();
        assert_eq!(snapshot.raw.header.frame_id, 2);
        assert_eq!(snapshot.config.unwrap().config_seq, 2);
        assert_eq!(snapshot.ego.header.sim_time_us, 10);
        assert!(snapshot.ego.paused);
        assert_eq!(snapshot.events.len(), 1);
        // A full SDK reload resets frame/time counters while retained mappings
        // still contain previous-session slots. Metadata starts a new epoch.
        let info =
            SharedCell::create(&format!("ETSADS_Adapter_Info_{suffix}"), SCHEMA_VERSION).unwrap();
        info.publish(&BridgeInfo {
            telemetry_api_version: 0x10001,
            session_start_qpc: 301,
            ..Default::default()
        })
        .unwrap();
        adapter.info = Some(info);
        assert!(adapter.poll().unwrap().is_none());
        first.header.qpc = 302;
        first.header.frame_id = 1;
        first.paused_simulation_time_us = 1;
        adapter.telemetry.publish(&first).unwrap();
        let mut new_extension = etsads_shm::zeroed_box::<ExtensionFrame>();
        new_extension.header = first.header;
        new_extension.available = 2;
        adapter
            .extension
            .as_ref()
            .unwrap()
            .publish(&new_extension)
            .unwrap();
        let snapshot = adapter.poll().unwrap().unwrap();
        assert_eq!(snapshot.raw.header.frame_id, 1);
        assert_eq!(snapshot.session_start_qpc, 301);
        assert_eq!(snapshot.ego.header.sim_time_us, 1);
        assert!(
            snapshot.config.is_none(),
            "no retained configuration for the new session"
        );
    }
}
