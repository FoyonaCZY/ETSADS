//! Causal checks on recorded wire observations, independent of review prose.
use anyhow::{Context, ensure};
use etsads_msgs::protocol::*;
use etsads_recorder::Record;
use std::collections::BTreeMap;

#[derive(Default)]
pub(super) struct Observations {
    pub telemetry: BTreeMap<u64, TelemetryFrame>,
    pub config: BTreeMap<u64, ConfigBlock>,
    pub frame_ends: BTreeMap<u64, u64>,
    pub commands: Vec<ControlCmd>,
    pub actions: Vec<InputActionFrame>,
    pub applied: Vec<AppliedControl>,
    pub gates: Vec<Gate>,
    pub status: Vec<SupervisorStatus>,
    pub events: Vec<GameEvent>,
    pub video: BTreeMap<u64, VideoIndex>,
    pub alignment: BTreeMap<u64, VideoAlignment>,
    pub all_video_count: u64,
}
impl Observations {
    pub fn ingest(&mut self, topic: &str, record: &Record, range: [u64; 2]) -> anyhow::Result<()> {
        if topic == "/config" {
            self.config.insert(record.frame_id, record.decode_wire()?);
        }
        if topic == "/video_index" {
            self.all_video_count += 1;
        }
        if !(range[0]..=range[1]).contains(&record.frame_id) {
            return Ok(());
        }
        match topic {
            "/telemetry" => {
                self.telemetry
                    .insert(record.frame_id, record.decode_wire()?);
            }
            "/frame_timing" => {
                let timing: FrameTiming = record.decode_wire()?;
                if timing.kind == 2 {
                    ensure!(
                        timing.physics_frame_id == record.frame_id
                            && timing.frame_end_qpc >= timing.frame_start_qpc,
                        "invalid frame-end evidence"
                    );
                    self.frame_ends
                        .insert(timing.physics_frame_id, timing.frame_end_qpc);
                }
            }
            "/control_cmd" => self.commands.push(record.decode_wire()?),
            "/input_actions" => self.actions.push(record.decode_wire()?),
            "/applied" => self.applied.push(record.decode_wire()?),
            "/gate" => self.gates.push(record.decode_wire()?),
            "/supervisor_status" => self.status.push(record.decode_wire()?),
            "/events" => self.events.push(record.decode_wire()?),
            "/video_index" => {
                let index: VideoIndex = record.decode_wire()?;
                ensure!(
                    self.video.insert(index.video_frame, index).is_none(),
                    "duplicate video index"
                );
            }
            "/video_alignment" => {
                let a: VideoAlignment = record.decode_wire()?;
                ensure!(
                    self.alignment.insert(a.video_frame, a).is_none(),
                    "duplicate video alignment"
                );
            }
            _ => {}
        }
        Ok(())
    }
    fn observed_time(&self, frame: u64) -> Option<u64> {
        self.frame_ends.get(&frame).copied()
    }
    pub fn validate_input_action(&self, name: &str) -> anyhow::Result<()> {
        let index = etsads_game::input::PULSE_NAMES
            .iter()
            .position(|n| *n == name)
            .context("unknown semantic action")?;
        ensure!(
            !self.actions.is_empty(),
            "semantic proof requires /input_actions from the current bridge, not just host pulse requests"
        );
        ensure!(
            !self.frame_ends.is_empty(),
            "semantic proof requires /frame_timing frame_end observations"
        );
        let confirmed = self
            .actions
            .iter()
            .filter(|a| {
                a.reason == 0
                    && a.command_seq > 0
                    && a.pulse_mask & (1 << index) != 0
                    && a.completed_qpc >= a.prepared_qpc
                    && a.prepared_qpc > 0
            })
            .any(|action| {
                let Some(before) = self
                    .telemetry
                    .iter()
                    .rfind(|(id, _)| {
                        self.observed_time(**id)
                            .is_some_and(|q| q < action.prepared_qpc)
                    })
                    .map(|(_, t)| t)
                else {
                    return false;
                };
                if !self.commands.iter().any(|c| {
                    c.seq >= action.command_seq && c.pulses[index] >= action.pulse_counters[index]
                }) {
                    return false;
                }
                let timeout = if name == "lblinker" || name == "rblinker" {
                    1.0
                } else {
                    3.0
                };
                let after = self.telemetry.iter().filter(|(id, t)| {
                    self.observed_time(**id).is_some_and(|q| {
                        q >= action.completed_qpc
                            && (q - action.completed_qpc) as f64 / t.qpc_frequency.max(1) as f64
                                <= timeout
                    })
                });
                match name {
                    "lblinker" => {
                        before.lights & (1 << 6) == 0
                            && after.into_iter().any(|(_, t)| t.lights & (1 << 6) != 0)
                    }
                    "rblinker" => {
                        before.lights & (1 << 7) == 0
                            && after.into_iter().any(|(_, t)| t.lights & (1 << 7) != 0)
                    }
                    "geardrive" => before.gear <= 0 && after.into_iter().any(|(_, t)| t.gear > 0),
                    "gearreverse" => before.gear >= 0 && after.into_iter().any(|(_, t)| t.gear < 0),
                    "activate" => self.events.iter().any(|event| {
                        event.name.split(|b| *b == 0).next()
                            == Some(b"player.tollgate.paid".as_slice())
                            && self.observed_time(event.header.frame_id).is_some_and(|q| {
                                q >= action.completed_qpc
                                    && (q - action.completed_qpc) as f64
                                        / before.qpc_frequency.max(1) as f64
                                        <= 3.0
                            })
                    }),
                    _ => false,
                }
            });
        ensure!(
            confirmed,
            "{name} lacks a completed ACTIVE SDK button edge followed by a measured state transition/event; an earlier matching state is not acknowledgement"
        );
        Ok(())
    }
    pub fn validate_video(&self, total_encoded_frames: u64) -> anyhow::Result<()> {
        ensure!(
            !self.video.is_empty() && total_encoded_frames == self.all_video_count,
            "encoded MP4 frame count differs from complete video-index count"
        );
        let indexes: Vec<_> = self.video.values().collect();
        ensure!(
            indexes
                .windows(2)
                .all(|pair| pair[1].video_frame == pair[0].video_frame + 1),
            "review video indices are not a contiguous encoded sequence"
        );
        for index in indexes {
            let physical = self
                .telemetry
                .get(&index.physics_frame)
                .context("video index points outside reviewed telemetry interval")?;
            ensure!(
                index.physics_qpc == physical.header.qpc
                    && index.sim_time_us == physical.header.sim_time_us
                    && index.alignment_error_qpc == index.capture_qpc.abs_diff(physical.header.qpc)
                    && index.video_frame < total_encoded_frames,
                "video index physics frame/QPC/time/error or encoded frame number is inconsistent"
            );
            let alignment = self
                .alignment
                .get(&index.video_frame)
                .context("video index has no completed alignment record")?;
            ensure!(
                alignment.status == 0
                    && alignment.capture_frame == index.capture_frame
                    && alignment.capture_qpc == index.capture_qpc,
                "camera capture and video alignment disagree"
            );
        }
        Ok(())
    }
    pub fn validate_traffic_extension(&self, ext: &ExtensionFrame) -> anyhow::Result<()> {
        use etsads_game::ets2la;
        let telemetry = self
            .telemetry
            .get(&ext.header.frame_id)
            .context("traffic extension has no reviewed physical frame")?;
        ensure!(
            ext.available & 10 == 10,
            "traffic review requires available traffic and signal channels, not a topic name alone"
        );
        etsads_safety::primary_obstacle(telemetry, ext).map_err(anyhow::Error::msg)?;
        for signal in ets2la::records::<ets2la::Signal>(&ext.signals) {
            if bytemuck::bytes_of(&signal).iter().all(|b| *b == 0) || signal.id < 0 {
                continue;
            }
            let position = signal.position;
            ensure!(
                position.iter().all(|v| v.is_finite())
                    && signal.time_remaining.is_finite()
                    && ets2la::valid_quaternion(signal.quaternion_wxyz),
                "invalid signal observation in traffic review"
            );
        }
        Ok(())
    }
    pub fn reverse_aeb_effective(
        &self,
        ext: &ExtensionFrame,
        profile: &etsads_safety::SafetyGeometryProfile,
        profile_hash: [u8; 32],
    ) -> anyhow::Result<bool> {
        let Some(t) = self.telemetry.get(&ext.header.frame_id) else {
            return Ok(false);
        };
        if t.gear >= 0 {
            return Ok(false);
        }
        let config = self
            .config
            .range(..=ext.header.frame_id)
            .next_back()
            .context("reverse evidence lacks raw configuration")?
            .1;
        let Some(obstacle) = etsads_safety::reverse::swept_obstacle(t, ext, config, profile)
            .map_err(anyhow::Error::msg)?
        else {
            return Ok(false);
        };
        let triggers = obstacle.distance_m <= 0.0
            || obstacle.closing_speed_mps > 0.0
                && (obstacle.distance_m / obstacle.closing_speed_mps < 1.0
                    || obstacle.closing_speed_mps.powi(2)
                        / (2.0 * (obstacle.distance_m - 0.5).max(0.1))
                        > 1.0);
        if !triggers {
            return Ok(false);
        }
        let frequency = t.qpc_frequency.max(1) as f64;
        Ok(self
            .gates
            .iter()
            .filter(|g| {
                g.enabled == GATE_ACTIVE
                    && g.aeb_brake > 0.05
                    && g.heartbeat_qpc >= t.header.qpc
                    && (g.heartbeat_qpc - t.header.qpc) as f64 / frequency <= 0.2
            })
            .any(|gate| {
                let ready = self.status.iter().any(|s| {
                    s.schema_version == 1
                        && s.process_id != 0
                        && s.reverse_ready == 1
                        && s.profile_sha256 == profile_hash
                        && s.configuration_sequence == config.config_seq
                        && s.heartbeat_qpc.abs_diff(gate.heartbeat_qpc) as f64 / frequency <= 0.2
                });
                ready
                    && self
                        .applied
                        .iter()
                        .filter(|a| {
                            a.reason == 0
                                && a.axes[1] == 0.0
                                && a.axes[2] >= gate.aeb_brake - 0.01
                                && a.applied_qpc >= gate.heartbeat_qpc
                                && (a.applied_qpc - gate.heartbeat_qpc) as f64 / frequency <= 0.2
                        })
                        .any(|a| {
                            let host_not_braking = self.commands.iter().any(|c| {
                                c.seq == a.applied_seq
                                    && c.brake == 0.0
                                    && c.write_qpc <= a.applied_qpc
                            });
                            host_not_braking
                                && self.telemetry.iter().any(|(id, observed)| {
                                    observed.gear < 0
                                        && observed.effective[2] > 0.05
                                        && self.observed_time(*id).is_some_and(|q| {
                                            q >= a.applied_qpc
                                                && (q - a.applied_qpc) as f64 / frequency <= 0.3
                                        })
                                })
                        })
            }))
    }
}

pub(super) fn mp4_frame_count(path: &std::path::Path) -> anyhow::Result<u64> {
    let executable = std::env::var_os("ETSADS_FFPROBE")
        .or_else(|| {
            let encoder = std::path::PathBuf::from(std::env::var_os("ETSADS_FFMPEG")?);
            let probe = encoder.parent()?.join(if cfg!(windows) {
                "ffprobe.exe"
            } else {
                "ffprobe"
            });
            probe.is_file().then(|| probe.into_os_string())
        })
        .unwrap_or_else(|| "ffprobe".into());
    let mut command = std::process::Command::new(executable);
    command
        .args([
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=nb_frames",
            "-of",
            "json",
        ])
        .arg(path)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    let mut child = command.spawn().context(
        "video evidence needs ffprobe; set ETSADS_FFPROBE to the installed ffprobe executable",
    )?;
    let start = std::time::Instant::now();
    while child.try_wait()?.is_none() {
        if start.elapsed() > std::time::Duration::from_secs(30) {
            let _ = child.kill();
            let _ = child.wait();
            anyhow::bail!("video metadata probe timed out: {}", path.display());
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    let result = child.wait_with_output()?;
    ensure!(
        result.status.success(),
        "ffprobe rejected reviewed video {}",
        path.display()
    );
    let parsed: serde_json::Value = serde_json::from_slice(&result.stdout)?;
    parsed["streams"][0]["nb_frames"]
        .as_str()
        .context("MP4 has no authoritative video frame count")?
        .parse()
        .context("invalid MP4 frame count")
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(id: u64, qpc: u64) -> TelemetryFrame {
        TelemetryFrame {
            header: WireHeader {
                frame_id: id,
                qpc,
                sim_time_us: id * 10000,
                ..Default::default()
            },
            qpc_frequency: 1000,
            ..Default::default()
        }
    }
    #[test]
    fn an_existing_gear_state_is_not_causal_sdk_acknowledgement() {
        let mut observed = Observations::default();
        let mut before = frame(1, 90);
        before.gear = -1;
        let mut after = frame(2, 190);
        after.gear = -1;
        observed.telemetry.insert(1, before);
        observed.telemetry.insert(2, after);
        observed.frame_ends.extend([(1, 100), (2, 200)]);
        let index = etsads_game::input::PULSE_NAMES
            .iter()
            .position(|n| *n == "gearreverse")
            .unwrap();
        let mut action = InputActionFrame {
            batch_id: 1,
            command_seq: 9,
            prepared_qpc: 120,
            completed_qpc: 130,
            pulse_mask: 1 << index,
            ..Default::default()
        };
        action.pulse_counters[index] = 1;
        observed.actions.push(action);
        let mut command = ControlCmd {
            seq: 9,
            write_qpc: 110,
            ..Default::default()
        };
        command.pulses[index] = 1;
        observed.commands.push(command);
        assert!(observed.validate_input_action("gearreverse").is_err());
        observed.telemetry.get_mut(&1).unwrap().gear = 0;
        observed.validate_input_action("gearreverse").unwrap();
        observed.actions[0].completed_qpc = 250;
        assert!(observed.validate_input_action("gearreverse").is_err());
    }
    #[test]
    fn video_tuple_cannot_override_wrong_physics_or_encoded_frame_count() {
        let mut observed = Observations::default();
        let t = frame(1, 100);
        observed.telemetry.insert(1, t);
        observed.all_video_count = 1;
        observed.video.insert(
            0,
            VideoIndex {
                video_frame: 0,
                capture_frame: 7,
                capture_qpc: 105,
                physics_frame: 1,
                physics_qpc: 100,
                sim_time_us: t.header.sim_time_us,
                alignment_error_qpc: 5,
                ..Default::default()
            },
        );
        observed.alignment.insert(
            0,
            VideoAlignment {
                video_frame: 0,
                capture_frame: 7,
                capture_qpc: 105,
                ..Default::default()
            },
        );
        observed.validate_video(1).unwrap();
        assert!(observed.validate_video(2).is_err());
        observed.video.get_mut(&0).unwrap().physics_qpc = 99;
        assert!(observed.validate_video(1).is_err());
        let mut extension = Box::<ExtensionFrame>::default();
        extension.header = t.header;
        assert!(observed.validate_traffic_extension(&extension).is_err());
        extension.available = 10;
        observed.validate_traffic_extension(&extension).unwrap();
        let signal = etsads_game::ets2la::Signal {
            position: [f32::NAN, 0.0, 0.0],
            sector: [0, 0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            kind: 0,
            time_remaining: 1.0,
            state: 0,
            id: 1,
        };
        extension.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
        assert!(observed.validate_traffic_extension(&extension).is_err());
    }
    #[test]
    fn off_gate_brake_intent_never_proves_reverse_aeb_effectiveness() {
        use etsads_msgs::{Point3, VehicleGeometry};
        let mut observed = Observations::default();
        let mut t = frame(1, 1000);
        t.gear = -1;
        t.speed_mps = -0.2;
        t.orientation_turns[0] = 0.75;
        let mut after = t;
        after.header = frame(2, 1060).header;
        after.effective[2] = 0.3;
        observed.telemetry.extend([(1, t), (2, after)]);
        observed.frame_ends.extend([(1, 1010), (2, 1070)]);
        let mut config = ConfigBlock {
            config_seq: 1,
            wheel_count: 4,
            ..Default::default()
        };
        config.wheel_positions[0] = [-1.0, 0.5, -3.8];
        config.wheel_positions[1] = [1.0, 0.5, -3.8];
        config.wheel_positions[2] = [-1.0, 0.5, 0.0];
        config.wheel_positions[3] = [1.0, 0.5, 0.0];
        config.wheel_radius[..4].fill(0.5);
        config.wheel_steerable[..2].fill(1);
        config.hook_position = [0.0, 1.0, -0.5];
        observed.config.insert(1, config);
        let profile = etsads_safety::SafetyGeometryProfile {
            schema_version: 1,
            validated_for_reverse: true,
            evidence_path: "synthetic checker fixture only".into(),
            geometry_fingerprint: etsads_msgs::vehicle::geometry_fingerprint(&config).unwrap(),
            configuration_sequence: 1,
            cargo_mass_range_kg: [0.0, 100.0],
            vertical_extent_from_rear_m: None,
            rear_wheel_indices: vec![2, 3],
            sdk_to_rear_axle_scs: Point3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            geometry: VehicleGeometry {
                has_trailer: false,
                hitch_offset: -0.5,
                trailer_wheelbase: 0.0,
                trailer_length: 0.0,
                ..Default::default()
            },
            tractor_front_from_rear_axle_m: 4.5,
            tractor_rear_overhang_m: 1.7,
            trailer: None,
        };
        let mut extension = Box::<ExtensionFrame>::default();
        extension.header = t.header;
        extension.available = 6;
        let parked = etsads_game::ets2la::Parked {
            position: [-2.0, 0.5, 0.0],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            size: [2.0, 2.0, 2.0],
            id: 1,
            is_trailer: 0,
        };
        extension.parked[..43].copy_from_slice(bytemuck::bytes_of(&parked));
        observed.gates.push(Gate {
            enabled: GATE_OFF,
            heartbeat_qpc: 1040,
            aeb_brake: 0.3,
            ..Default::default()
        });
        observed.status.push(SupervisorStatus {
            schema_version: 1,
            process_id: 42,
            heartbeat_qpc: 1040,
            profile_sha256: [2; 32],
            configuration_sequence: 1,
            reverse_ready: 1,
            reserved: 0,
        });
        observed.commands.push(ControlCmd {
            seq: 5,
            write_qpc: 1010,
            ..Default::default()
        });
        observed.applied.push(AppliedControl {
            applied_seq: 5,
            applied_qpc: 1050,
            axes: [0.0, 0.0, 0.3, 0.0],
            ..Default::default()
        });
        assert!(
            !observed
                .reverse_aeb_effective(&extension, &profile, [2; 32])
                .unwrap()
        );
        observed.gates[0].enabled = GATE_ACTIVE;
        assert!(
            observed
                .reverse_aeb_effective(&extension, &profile, [2; 32])
                .unwrap()
        );
        observed.applied[0].reason = etsads_game::input::REASON_BRAKE_ONLY;
        assert!(
            !observed
                .reverse_aeb_effective(&extension, &profile, [2; 32])
                .unwrap()
        );
        observed.applied[0].reason = 0;
        observed.telemetry.get_mut(&2).unwrap().effective[2] = 0.0;
        assert!(
            !observed
                .reverse_aeb_effective(&extension, &profile, [2; 32])
                .unwrap()
        );
    }
}
