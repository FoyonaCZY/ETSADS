//! Metrics derived from immutable recordings. Missing evidence stays missing;
//! no simulation value or absent channel can pass a live-game exit gate.
use etsads_msgs::protocol::*;
use etsads_recorder::{Record, ReplayReport};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, VecDeque},
    path::Path,
};

#[derive(Default, Debug, Serialize, Deserialize)]
pub struct Distribution {
    pub samples: usize,
    pub minimum: Option<f64>,
    pub p05: Option<f64>,
    pub median: Option<f64>,
    pub p95: Option<f64>,
    pub maximum: Option<f64>,
}
impl Distribution {
    fn from_values(mut values: Vec<f64>) -> Self {
        values.retain(|v| v.is_finite());
        values.sort_by(f64::total_cmp);
        let percentile = |p: f64| {
            values
                .get(((values.len().saturating_sub(1)) as f64 * p).round() as usize)
                .copied()
        };
        Self {
            samples: values.len(),
            minimum: values.first().copied(),
            p05: percentile(0.05),
            median: percentile(0.5),
            p95: percentile(0.95),
            maximum: values.last().copied(),
        }
    }
}
#[derive(Debug, Serialize, Deserialize)]
pub struct RunMetrics {
    pub schema_version: u32,
    pub evidence: String,
    pub integrity: ReplayReport,
    pub active_simulation_seconds: f64,
    pub physical_world_km: f64,
    pub scaled_game_km: Option<f64>,
    pub teleports_or_discontinuities: u64,
    pub invalid_telemetry: u64,
    pub event_counts: BTreeMap<String, u64>,
    pub fine_offences: BTreeMap<String, u64>,
    pub observed_fines: u64,
    pub fines_per_100_scaled_km: Option<f64>,
    pub fines_enabled_verified: bool,
    pub gate_releases: u64,
    pub lateral_error_rms_m: Option<f64>,
    pub moving_lateral_error_rms_m: Option<f64>,
    pub verified_following_headway_seconds: Distribution,
    pub synthetic_following_headway_seconds: Distribution,
    pub rejected_following_samples: u64,
    pub lateral_acceleration_mps2: Distribution,
    pub jerk_mps3: Distribution,
    pub command_delay_ms: Distribution,
    pub video_alignment_error_ms: Distribution,
    pub video_frames: u64,
    pub dropped_video_frames: u64,
    pub video_alignment_status_counts: BTreeMap<u32, u64>,
    pub every_video_frame_bracketed: Option<bool>,
    pub limitations: Vec<String>,
}
#[derive(Default)]
struct Accumulator {
    previous: Option<TelemetryFrame>,
    last_acceleration: Option<f64>,
    active: f64,
    distance: f64,
    scaled: f64,
    unknown_scale: bool,
    discontinuities: u64,
    invalid: u64,
    events: BTreeMap<String, u64>,
    offences: BTreeMap<String, u64>,
    releases: u64,
    gate_enabled: bool,
    error_squared: f64,
    error_count: u64,
    moving_error_squared: f64,
    moving_error_count: u64,
    verified_headway: Vec<f64>,
    synthetic_headway: Vec<f64>,
    rejected_following: u64,
    motion: BTreeMap<(u64, u64), Option<(f64, bool)>>,
    following_candidates: Vec<FollowingSample>,
    following_counts: BTreeMap<(u64, u64), u64>,
    lat: Vec<f64>,
    jerk: Vec<f64>,
    delays: Vec<f64>,
    video_alignment: Vec<f64>,
    video_frames: u64,
    video_dropped: u64,
    frequency: u64,
    commands: VecDeque<(u64, u64)>,
    last_applied: u64,
    alignment_status: BTreeMap<u32, u64>,
    video_records: BTreeMap<u64, (u64, u64)>,
    alignment_records: BTreeMap<u64, (u64, u64, u32)>,
    duplicate_video_record: bool,
}
struct FollowingSample {
    key: (u64, u64),
    speed: f64,
    headway: f64,
    verified_geometry: bool,
}
impl Accumulator {
    fn consume(&mut self, topic: &str, record: &Record) -> anyhow::Result<()> {
        match topic {
            "/telemetry" => self.telemetry(record.decode_wire()?),
            "/events" => {
                let e = record.decode_wire::<GameEvent>()?;
                let name = text(&e.name);
                *self.events.entry(name.clone()).or_default() += 1;
                if name == "player.fined" {
                    *self.offences.entry(text(&e.detail)).or_default() += 1;
                }
            }
            "/gate" => {
                let gate = record.decode_wire::<Gate>()?;
                let enabled = gate.enabled == 1;
                if self.gate_enabled && !enabled {
                    self.releases += 1;
                }
                self.gate_enabled = enabled;
            }
            "/control_cmd" => {
                let command = record.decode_wire::<ControlCmd>()?;
                if self
                    .commands
                    .back()
                    .is_none_or(|(seq, _)| *seq != command.seq)
                {
                    self.commands.push_back((command.seq, command.write_qpc));
                    if self.commands.len() > 256 {
                        self.commands.pop_front();
                    }
                }
            }
            "/applied" => {
                let applied = record.decode_wire::<AppliedControl>()?;
                if applied.applied_seq != self.last_applied
                    && self.frequency > 0
                    && applied.reason == 0
                {
                    if let Some((_, qpc)) = self
                        .commands
                        .iter()
                        .rev()
                        .find(|(seq, _)| *seq == applied.applied_seq)
                        && applied.applied_qpc >= *qpc
                    {
                        self.delays.push(
                            (applied.applied_qpc - *qpc) as f64 * 1000.0 / self.frequency as f64,
                        );
                    }
                    self.last_applied = applied.applied_seq;
                }
            }
            "/video_alignment" => {
                let alignment = record.decode_wire::<VideoAlignment>()?;
                *self.alignment_status.entry(alignment.status).or_default() += 1;
                self.duplicate_video_record |= self
                    .alignment_records
                    .insert(
                        alignment.video_frame,
                        (
                            alignment.capture_frame,
                            alignment.capture_qpc,
                            alignment.status,
                        ),
                    )
                    .is_some();
            }
            "/video_index" => {
                let index = record.decode_wire::<VideoIndex>()?;
                self.duplicate_video_record |= self
                    .video_records
                    .insert(index.video_frame, (index.capture_frame, index.capture_qpc))
                    .is_some();
                self.video_frames += 1;
                self.video_dropped += index.dropped_before;
                if self.frequency > 0 {
                    self.video_alignment
                        .push(index.alignment_error_qpc as f64 * 1000.0 / self.frequency as f64);
                }
            }
            "/stack" => {
                let output: serde_json::Value = serde_json::from_slice(&record.raw_payload)?;
                if output["request_control"] == true
                    && let Some(error) = output["lane"]["d"].as_f64().filter(|v| v.is_finite())
                {
                    self.error_squared += error * error;
                    self.error_count += 1;
                    if output["control_ego"]["speed"]
                        .as_f64()
                        .is_some_and(|speed| speed.is_finite() && speed.abs() > 0.5)
                    {
                        self.moving_error_squared += error * error;
                        self.moving_error_count += 1;
                    }
                }
                self.following(&output, record.frame_id, record.qpc);
            }
            _ => {}
        }
        Ok(())
    }
    fn following(&mut self, output: &serde_json::Value, frame: u64, qpc: u64) {
        let Some(sample) = output.get("following").filter(|v| !v.is_null()) else {
            return;
        };
        let key = (frame, qpc);
        *self.following_counts.entry(key).or_default() += 1;
        let valid = (|| {
            if output["request_control"] != true || output["status"] != "Running" {
                return None;
            }
            sample["leader_id"].as_u64()?;
            if sample["frame_id"].as_u64()? != frame
                || output["header"]["frame_id"].as_u64()? != frame
            {
                return None;
            }
            let gap = sample["bumper_gap_m"].as_f64()?;
            let speed = sample["ego_speed_mps"].as_f64()?;
            let reported_speed = output["control_ego"]["speed"].as_f64()?;
            let headway = sample["headway_seconds"].as_f64()?;
            if !gap.is_finite()
                || !(0.0..=5000.0).contains(&gap)
                || !speed.is_finite()
                || speed <= 0.05
                || speed > 100.0
                || !reported_speed.is_finite()
                || (reported_speed - speed).abs() > 1e-4
                || !headway.is_finite()
                || headway < 0.0
                || (headway - gap / speed).abs() > 1e-6 * headway.max(1.0)
            {
                return None;
            }
            match sample["quality"].as_str()? {
                "verified_geometry" => Some(FollowingSample {
                    key,
                    speed,
                    headway,
                    verified_geometry: true,
                }),
                "synthetic_replay" => Some(FollowingSample {
                    key,
                    speed,
                    headway,
                    verified_geometry: false,
                }),
                _ => None,
            }
        })();
        match valid {
            Some(candidate) => self.following_candidates.push(candidate),
            None => self.rejected_following += 1,
        }
    }
    fn finish_following(&mut self, source: &str) {
        for candidate in std::mem::take(&mut self.following_candidates) {
            let matched = self
                .motion
                .get(&candidate.key)
                .and_then(|v| *v)
                .is_some_and(|(speed, paused)| {
                    !paused && speed > 0.05 && (speed - candidate.speed).abs() <= 1e-4
                });
            if !matched
                || self.following_counts.get(&candidate.key) != Some(&1)
                || (candidate.verified_geometry && source != "game")
            {
                self.rejected_following += 1;
            } else if candidate.verified_geometry {
                self.verified_headway.push(candidate.headway);
            } else {
                self.synthetic_headway.push(candidate.headway);
            }
        }
    }
    fn telemetry(&mut self, t: TelemetryFrame) {
        let invalid = !t.position_scs.iter().all(|v| v.is_finite())
            || !t.speed_mps.is_finite()
            || t.speed_mps.abs() > 70.0
            || !t.acceleration_local.iter().all(|v| v.is_finite())
            || !t.angular_velocity_local.iter().all(|v| v.is_finite())
            || t.qpc_frequency == 0;
        self.motion
            .entry((t.header.frame_id, t.header.qpc))
            .and_modify(|v| *v = None)
            .or_insert_with(|| (!invalid).then_some((t.speed_mps as f64, t.paused != 0)));
        if invalid {
            self.invalid += 1;
            self.previous = None;
            self.last_acceleration = None;
            return;
        }
        self.frequency = t.qpc_frequency;
        if let Some(previous) = self.previous {
            let elapsed = t
                .paused_simulation_time_us
                .checked_sub(previous.paused_simulation_time_us)
                .map(|v| v as f64 / 1e6);
            if let Some(dt) = elapsed.filter(|dt| *dt > 0.0 && *dt <= 0.2) {
                if t.paused == 0
                    && previous.paused == 0
                    && t.header.frame_id == previous.header.frame_id + 1
                {
                    let distance = (t.position_scs[0] - previous.position_scs[0])
                        .hypot(t.position_scs[2] - previous.position_scs[2]);
                    if distance <= 70.0 * dt + 0.5 {
                        self.active += dt;
                        self.distance += distance;
                        if t.local_scale.is_finite() && (1.0..=100.0).contains(&t.local_scale) {
                            self.scaled += distance * t.local_scale as f64;
                        } else {
                            self.unknown_scale = true;
                        }
                        let acceleration = -t.acceleration_local[2] as f64;
                        self.lat.push(
                            (t.speed_mps as f64
                                * t.angular_velocity_local[1] as f64
                                * std::f64::consts::TAU)
                                .abs(),
                        );
                        if let Some(a) = self.last_acceleration {
                            self.jerk.push(((acceleration - a) / dt).abs());
                        }
                        self.last_acceleration = Some(acceleration);
                    } else {
                        self.discontinuities += 1;
                        self.last_acceleration = None;
                    }
                } else {
                    self.last_acceleration = None;
                }
            } else if elapsed.is_none() || elapsed.is_some_and(|dt| dt > 0.2) {
                self.discontinuities += 1;
                self.last_acceleration = None;
            }
        }
        self.previous = Some(t);
    }
    fn every_video_frame_bracketed(&self) -> Option<bool> {
        if self.alignment_records.is_empty() {
            return None;
        }
        Some(
            !self.duplicate_video_record
                && self.video_records.len() == self.alignment_records.len()
                && self.video_records.iter().all(|(id, (capture, qpc))| {
                    self.alignment_records.get(id)
                        == Some(&(*capture, *qpc, VIDEO_ALIGNMENT_BRACKETED))
                }),
        )
    }
}
fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(&bytes[..bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len())])
        .into_owned()
}
pub fn analyze(run: &Path) -> anyhow::Result<RunMetrics> {
    let mut accumulator = Accumulator::default();
    let integrity =
        etsads_recorder::replay(run, |topic, record| accumulator.consume(topic, record))?;
    // Join by physical frame and original QPC after reading the entire stream;
    // a /stack record is allowed to precede its telemetry in the MCAP.
    accumulator.finish_following(&integrity.source);
    let scaled = if !accumulator.unknown_scale && accumulator.active > 0.0 {
        Some(accumulator.scaled / 1000.0)
    } else {
        None
    };
    let fines = accumulator.events.get("player.fined").copied().unwrap_or(0);
    let bracketed = accumulator.every_video_frame_bracketed();
    Ok(RunMetrics{
        schema_version:1,
        evidence:format!("Computed from {} MCAP; no live phase acceptance inferred",integrity.source),
        integrity,
        active_simulation_seconds:accumulator.active,
        physical_world_km:accumulator.distance/1000.0,
        scaled_game_km:scaled,
        teleports_or_discontinuities:accumulator.discontinuities,
        invalid_telemetry:accumulator.invalid,
        event_counts:accumulator.events,
        fine_offences:accumulator.offences,
        observed_fines:fines,
        fines_per_100_scaled_km:scaled.filter(|v|*v>0.0).map(|v|fines as f64/v*100.0),
        fines_enabled_verified:false,
        gate_releases:accumulator.releases,
        lateral_error_rms_m:if accumulator.error_count>0{Some((accumulator.error_squared/accumulator.error_count as f64).sqrt())}else{None},
        moving_lateral_error_rms_m:if accumulator.moving_error_count>0{Some((accumulator.moving_error_squared/accumulator.moving_error_count as f64).sqrt())}else{None},
        verified_following_headway_seconds:Distribution::from_values(accumulator.verified_headway),
        synthetic_following_headway_seconds:Distribution::from_values(accumulator.synthetic_headway),
        rejected_following_samples:accumulator.rejected_following,
        lateral_acceleration_mps2:Distribution::from_values(accumulator.lat),
        jerk_mps3:Distribution::from_values(accumulator.jerk),
        command_delay_ms:Distribution::from_values(accumulator.delays),
        video_alignment_error_ms:Distribution::from_values(accumulator.video_alignment),
        video_frames:accumulator.video_frames,
        dropped_video_frames:accumulator.video_dropped,
        video_alignment_status_counts:accumulator.alignment_status,
        every_video_frame_bracketed:bracketed,
        limitations:vec![
            "scaled_game_km integrates world displacement × local.scale; odometer cross-check still required".into(),
            "zero observed fines is not proof of zero violations unless g_police and event capture are verified".into(),
            "gate_releases includes manual stops, pause and faults; it is not automatically a human-intervention count".into(),
            "Lane RMS requires recorded /stack lane matches; moving RMS excludes speeds <=0.5m/s. Following requires same-frame observed bumper gap and speed, with verified and synthetic geometry reported separately. No delivery success is invented from missing data".into()
        ]
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    fn following_output(frame: u64, quality: &str) -> serde_json::Value {
        serde_json::json!({"header":{"frame_id":frame},"status":"Running","request_control":true,"control_ego":{"speed":20.0},"following":{"leader_id":9,"frame_id":frame,"bumper_gap_m":36.0,"ego_speed_mps":20.0,"headway_seconds":1.8,"quality":quality}})
    }
    fn following_telemetry(frame: u64) -> TelemetryFrame {
        let mut t = sample(frame, frame * 100_000, 0.0);
        t.header.qpc = frame * 100;
        t.speed_mps = 20.0;
        t
    }
    #[test]
    fn following_requires_causal_measured_gap_and_separates_geometry_quality() {
        let mut a = Accumulator::default();
        // The asynchronous /stack can precede its raw telemetry in the file.
        a.following(&following_output(1, "verified_geometry"), 1, 100);
        a.following(&following_output(2, "synthetic_replay"), 2, 200);
        a.telemetry(following_telemetry(1));
        a.telemetry(following_telemetry(2));
        let mut output = following_output(3, "verified_geometry");
        output["following"]["headway_seconds"] = 2.0.into(); // Desired target is not an observation.
        a.following(&output, 3, 300);
        output["following"]["headway_seconds"] = 1.8.into();
        a.following(&output, 4, 400); // A different physical frame is not silently aligned.
        output["following"]["ego_speed_mps"] = (-20.0).into();
        a.following(&output, 5, 500);
        a.following(&serde_json::json!({"following":null}), 6, 600);
        a.finish_following("game");
        assert_eq!(a.rejected_following, 3);
        assert_eq!(a.verified_headway, vec![1.8]);
        assert_eq!(a.synthetic_headway, vec![1.8]);
    }
    #[test]
    fn following_rejects_orphans_faults_stopped_raw_frames_and_duplicates() {
        for case in [
            "missing",
            "paused",
            "stationary",
            "wrong_qpc",
            "duplicate_raw",
            "duplicate_stack",
            "fault",
            "synthetic_source",
        ] {
            let mut a = Accumulator::default();
            let mut output = following_output(42, "verified_geometry");
            let mut raw = following_telemetry(42);
            if case == "paused" {
                raw.paused = 1;
            }
            if case == "stationary" {
                raw.speed_mps = 0.0;
            }
            if case == "wrong_qpc" {
                raw.header.qpc += 1;
            }
            if case == "fault" {
                output["status"] = "Fault".into();
                output["request_control"] = false.into();
            }
            if case != "missing" {
                a.telemetry(raw);
            }
            if case == "duplicate_raw" {
                a.telemetry(raw);
            }
            a.following(&output, 42, 4200);
            if case == "duplicate_stack" {
                a.following(&output, 42, 4200);
            }
            a.finish_following(if case == "synthetic_source" {
                "synthetic"
            } else {
                "game"
            });
            assert!(a.verified_headway.is_empty(), "{case}");
            assert!(a.rejected_following > 0, "{case}");
        }
    }
    #[test]
    fn alignment_requires_unique_matching_frame_evidence() {
        let mut a = Accumulator::default();
        assert_eq!(a.every_video_frame_bracketed(), None);
        let index = VideoIndex {
            video_frame: 0,
            capture_frame: 42,
            capture_qpc: 1000,
            ..Default::default()
        };
        let proof = VideoAlignment {
            video_frame: 0,
            capture_frame: 42,
            capture_qpc: 1000,
            status: VIDEO_ALIGNMENT_BRACKETED,
            ..Default::default()
        };
        a.consume("/video_index", &Record::wire(&index, 1, 1, 1000))
            .unwrap();
        a.consume("/video_alignment", &Record::wire(&proof, 1, 1, 1000))
            .unwrap();
        assert_eq!(a.every_video_frame_bracketed(), Some(true));
        let mut bad = proof;
        bad.capture_qpc = 999;
        let mut mismatch = Accumulator::default();
        mismatch
            .consume("/video_index", &Record::wire(&index, 1, 1, 1000))
            .unwrap();
        mismatch
            .consume("/video_alignment", &Record::wire(&bad, 1, 1, 1000))
            .unwrap();
        assert_eq!(mismatch.every_video_frame_bracketed(), Some(false));
        a.consume("/video_alignment", &Record::wire(&proof, 1, 1, 1000))
            .unwrap();
        assert_eq!(a.every_video_frame_bracketed(), Some(false));
    }
    fn sample(frame: u64, time: u64, x: f64) -> TelemetryFrame {
        TelemetryFrame {
            header: WireHeader {
                frame_id: frame,
                sim_time_us: time,
                ..Default::default()
            },
            paused_simulation_time_us: time,
            position_scs: [x, 0.0, 0.0],
            speed_mps: 10.0,
            local_scale: 20.0,
            qpc_frequency: 1000,
            ..Default::default()
        }
    }
    #[test]
    fn pause_and_teleport_do_not_inflate_distance() {
        let mut a = Accumulator::default();
        a.telemetry(sample(1, 100_000, 0.0));
        a.telemetry(sample(2, 200_000, 1.0));
        let mut paused = sample(3, 200_000, 1.0);
        paused.paused = 1;
        a.telemetry(paused);
        a.telemetry(sample(4, 300_000, 1.0));
        a.telemetry(sample(5, 400_000, 10001.0));
        assert_eq!(a.distance, 1.0);
        assert_eq!(a.scaled, 20.0);
        assert_eq!(a.discontinuities, 1);
        assert!((a.active - 0.1).abs() < 1e-9);
    }
    #[test]
    fn invalid_values_reset_derivative_history() {
        let mut a = Accumulator::default();
        a.telemetry(sample(1, 100_000, 0.0));
        let mut bad = sample(2, 200_000, 1.0);
        bad.position_scs[0] = f64::NAN;
        a.telemetry(bad);
        a.telemetry(sample(3, 300_000, 2.0));
        assert_eq!(a.invalid, 1);
        assert!(a.jerk.is_empty());
        assert_eq!(a.distance, 0.0);
    }
    #[test]
    fn percentile_reports_missing_instead_of_zero() {
        assert_eq!(Distribution::from_values(vec![]).p95, None);
        let d = Distribution::from_values(vec![3.0, 1.0, 2.0, f64::NAN]);
        assert_eq!(d.samples, 3);
        assert_eq!(d.minimum, Some(1.0));
        assert_eq!(d.median, Some(2.0));
        assert_eq!(d.p95, Some(3.0));
    }
}
