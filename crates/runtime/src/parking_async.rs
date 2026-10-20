//! Stationary parking search lifecycle. Road/moving trajectory freshness stays
//! 200 ms. A slow geometric result needs a new observation and a second worker
//! request before it can become a command; its original timestamp is not edited.
use super::*;
use std::{
    sync::{Arc, atomic::Ordering},
    time::{Duration, Instant},
};

const FRESH: Duration = Duration::from_millis(200);
const WAIT_LIMIT: Duration = Duration::from_secs(30);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ParkingPlanningPhase {
    Searching,
    Revalidating,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingPlanningDiagnostic {
    pub phase: ParkingPlanningPhase,
    pub original_frame_id: u64,
    pub request_frame_id: u64,
    pub waited_seconds: f64,
    pub timeout_seconds: f64,
}
#[derive(Clone, Copy)]
pub(crate) struct Anchor {
    ego: EgoState,
    session: u64,
    configuration: u64,
    qpc: u64,
}
impl Anchor {
    fn new(snapshot: &GameSnapshot) -> Self {
        Self {
            ego: snapshot.ego,
            session: snapshot.session_start_qpc,
            configuration: snapshot.config.map_or(0, |c| c.config_seq),
            qpc: snapshot.raw.header.qpc,
        }
    }
    fn stationary(self) -> bool {
        self.ego.speed.abs() <= 0.05
    }
    fn identity(self, current: &GameSnapshot) -> Result<(), String> {
        if self.session != current.session_start_qpc
            || self.configuration != current.config.map_or(0, |c| c.config_seq)
            || current.ego.header.frame_id < self.ego.header.frame_id
            || current.raw.header.qpc < self.qpc
        {
            return Err("parking request session/configuration/frame changed".into());
        }
        Ok(())
    }
    fn unchanged_standstill(self, current: &GameSnapshot) -> Result<(), String> {
        self.identity(current)?;
        let ego = current.ego;
        if !self.stationary()
            || ego.speed.abs() > 0.05
            || ego.pose.distance(self.ego.pose) > 0.1
            || (ego.z - self.ego.z).abs() > 0.1
            || wrap_angle(ego.pose.yaw - self.ego.pose.yaw).abs() > 0.02
            || wrap_angle(ego.trailer_yaw - self.ego.trailer_yaw).abs() > 0.02
            || (ego.wheel_angle - self.ego.wheel_angle).abs() > 0.03
        {
            return Err(
                "truck moved or changed articulation/steering while awaiting a parking plan".into(),
            );
        }
        Ok(())
    }
}
#[derive(Clone, Copy)]
pub(crate) struct ParkingJobTag {
    pub id: u64,
    pub phase: ParkingPlanningPhase,
    pub anchor: Anchor,
}
struct Pending {
    tag: ParkingJobTag,
    original: Anchor,
    started: Instant,
    dispatched: Instant,
}
#[derive(Default)]
pub(crate) struct ParkingAsyncState {
    serial: u64,
    pending: Option<Pending>,
}
pub(crate) struct Progress {
    pub planned: bool,
    pub seconds: f64,
    pub waiting: Option<ParkingPlanningDiagnostic>,
}
impl Runtime {
    fn parking_candidate_safe(
        &self,
        snapshot: &GameSnapshot,
        states: &[TruckState],
    ) -> Result<(), String> {
        let DrivingMode::Parking { mission } = &self.config.mode else {
            return Err("stationary wait is parking-only".into());
        };
        let obstacles = parking_obstacles::collect(snapshot)?;
        parking_mission::validate_adoption_path(
            snapshot.ego,
            states,
            self.calibration.geometry,
            mission,
            &obstacles.current,
            &obstacles.occupancy,
            Some(&self.planner_cancel),
        )
    }
    fn check_parking_wait(&self, pending: &Pending, snapshot: &GameSnapshot) -> Result<(), String> {
        if pending.started.elapsed() >= WAIT_LIMIT {
            return Err("stationary parking search exceeded 30 second wall-clock budget".into());
        }
        pending.original.unchanged_standstill(snapshot)?;
        self.parking_candidate_safe(snapshot, &[])
    }
    fn dispatch_parking(
        &mut self,
        snapshot: &GameSnapshot,
        phase: ParkingPlanningPhase,
        original: Anchor,
        started: Instant,
    ) -> Result<(), String> {
        self.parking_async.serial = self
            .parking_async
            .serial
            .checked_add(1)
            .ok_or("parking request sequence overflow")?;
        let tag = ParkingJobTag {
            id: self.parking_async.serial,
            phase,
            anchor: Anchor::new(snapshot),
        };
        let copy = GameSnapshot {
            events: self.pending_events.clone(),
            ..snapshot.clone()
        };
        self.planner_tx
            .as_ref()
            .ok_or("planner worker unavailable")?
            .try_send(PlanJob {
                generation: self.generation,
                snapshot: copy,
                lane: None,
                context: None,
                parking: Some(tag),
                cancellation: Arc::clone(&self.planner_cancel),
                #[cfg(test)]
                delay: if phase == ParkingPlanningPhase::Searching {
                    self.parking_test_delay
                } else {
                    self.parking_test_validation_delay
                },
            })
            .map_err(|_| "parking worker queue unavailable; no stale request may be queued")?;
        self.parking_async.pending = Some(Pending {
            tag,
            original,
            started,
            dispatched: Instant::now(),
        });
        self.last_request = Some(snapshot.ego.header.sim_time_us);
        self.first_request
            .get_or_insert(snapshot.ego.header.sim_time_us);
        self.pending_events.clear();
        Ok(())
    }
    pub(crate) fn poll_live_parking(
        &mut self,
        snapshot: &GameSnapshot,
    ) -> Result<Progress, String> {
        let mut progress = Progress {
            planned: false,
            seconds: 0.0,
            waiting: None,
        };
        if let Some(pending) = &self.parking_async.pending {
            pending.tag.anchor.identity(snapshot)?;
            if pending.dispatched.elapsed() >= FRESH
                || pending.tag.phase == ParkingPlanningPhase::Revalidating
            {
                self.check_parking_wait(pending, snapshot)?;
            }
        }
        let completed: Vec<_> = self
            .planner_rx
            .as_ref()
            .ok_or("planner receiver unavailable")?
            .try_iter()
            .collect();
        for completed in completed {
            if completed.generation != self.generation {
                continue;
            }
            let Some(tag) = completed.parking else {
                return Err("parking worker returned an unrelated plan".into());
            };
            let Some(pending) = self.parking_async.pending.take() else {
                continue;
            };
            if pending.tag.id != tag.id || pending.tag.phase != tag.phase {
                self.parking_async.pending = Some(pending);
                continue;
            }
            tag.anchor.identity(snapshot)?;
            let trajectory = completed.result?;
            let wall_age = pending.dispatched.elapsed();
            let now = etsads_shm::qpc();
            let frequency = snapshot.raw.qpc_frequency;
            let fresh_qpc = frequency > 0
                && tag.anchor.qpc > 0
                && now >= tag.anchor.qpc
                && (now - tag.anchor.qpc) as u128 * 1000 <= frequency as u128 * 200;
            let stale = wall_age > FRESH
                || completed.seconds > FRESH.as_secs_f64()
                || snapshot
                    .ego
                    .header
                    .sim_time_us
                    .saturating_sub(trajectory.header.sim_time_us)
                    > 200_000
                || !fresh_qpc;
            if tag.phase == ParkingPlanningPhase::Searching && stale {
                self.check_parking_wait(&pending, snapshot)?;
                self.latest = None; // old trajectory AND old semantic intents are discarded.
                self.intent = PlanningIntent::default();
                self.dispatch_parking(
                    snapshot,
                    ParkingPlanningPhase::Revalidating,
                    pending.original,
                    pending.started,
                )?;
                continue;
            }
            if stale {
                return Err("fresh parking adoption missed the unchanged 200 ms deadline".into());
            }
            if tag.phase == ParkingPlanningPhase::Revalidating {
                self.check_parking_wait(&pending, snapshot)?;
                tag.anchor.unchanged_standstill(snapshot)?;
                let proof = completed
                    .parking_path
                    .as_ref()
                    .ok_or("parking adoption has no validated remaining path")?;
                let obstacles = parking_obstacles::collect(snapshot)?;
                if !parking_certificate::adoption_environment_safe(
                    proof,
                    &obstacles.current,
                    &obstacles.occupancy,
                )? {
                    self.latest = None;
                    self.intent = PlanningIntent::default();
                    self.dispatch_parking(
                        snapshot,
                        ParkingPlanningPhase::Revalidating,
                        pending.original,
                        pending.started,
                    )?;
                    continue;
                }
                // Include this frame's small standstill drift, not just the
                // original worker pose, in the first remaining connection.
                self.parking_candidate_safe(snapshot, proof.first.as_slice())?;
                let now = etsads_shm::qpc();
                if pending.dispatched.elapsed() > FRESH
                    || now < tag.anchor.qpc
                    || (now - tag.anchor.qpc) as u128 * 1000 > frequency as u128 * 200
                {
                    return Err(
                        "fresh parking adoption exceeded 200 ms during local sweep validation"
                            .into(),
                    );
                }
            }
            // This header belongs to an actual fresh worker request. Never
            // retimestamp the original slow trajectory to make it look fresh.
            if trajectory.header.frame_id != tag.anchor.ego.header.frame_id
                || trajectory.header.sim_time_us != tag.anchor.ego.header.sim_time_us
            {
                return Err("parking result header differs from its request".into());
            }
            self.last_plan = Some(trajectory.header.sim_time_us);
            self.latest = Some(trajectory);
            self.target_acceleration = completed.acceleration;
            self.intent = completed.intent;
            self.prediction_diagnostic = completed.prediction;
            progress.planned = true;
            progress.seconds = completed.seconds;
        }
        if self.parking_async.pending.is_none()
            && self
                .last_request
                .is_none_or(|last| snapshot.ego.header.sim_time_us.saturating_sub(last) >= 100_000)
        {
            self.dispatch_parking(
                snapshot,
                ParkingPlanningPhase::Searching,
                Anchor::new(snapshot),
                Instant::now(),
            )?;
        }
        if let Some(pending) = &self.parking_async.pending {
            let fresh_trajectory = self.latest.as_ref().is_some_and(|t| {
                snapshot
                    .ego
                    .header
                    .sim_time_us
                    .saturating_sub(t.header.sim_time_us)
                    <= 200_000
            });
            if pending.original.stationary()
                && (!fresh_trajectory || pending.tag.phase == ParkingPlanningPhase::Revalidating)
            {
                self.check_parking_wait(pending, snapshot)?;
                progress.waiting = Some(ParkingPlanningDiagnostic {
                    phase: pending.tag.phase,
                    original_frame_id: pending.original.ego.header.frame_id,
                    request_frame_id: pending.tag.anchor.ego.header.frame_id,
                    waited_seconds: pending.started.elapsed().as_secs_f64(),
                    timeout_seconds: WAIT_LIMIT.as_secs_f64(),
                });
            } else if !fresh_trajectory && pending.dispatched.elapsed() > FRESH {
                return Err("moving parking trajectory missed the 200 ms deadline".into());
            }
        }
        Ok(progress)
    }
    pub(crate) fn parking_wait_output(
        &mut self,
        ego: EgoState,
        diagnostic: ParkingPlanningDiagnostic,
    ) -> RuntimeOutput {
        self.latest = None;
        self.last_plan = None;
        self.controller.reset();
        self.mpc.reset();
        let curvature = ego.wheel_angle.tan() / self.calibration.geometry.wheelbase;
        let steer = match self.calibration.steering_axis(curvature, ego.speed.abs()) {
            Ok(axis) => axis,
            Err(error) => return self.stopped(ego, RuntimeStatus::Fault, error.to_string()),
        };
        let mut output = self.stopped(
            ego,
            RuntimeStatus::WaitingForParkingPlan,
            "stationary parking search; no motion/action authority until fresh adoption",
        );
        output.request_control = false; // Never an arm request; host may only retain an already-granted brake hold.
        output.actuation = Actuation {
            steer,
            brake: 0.4,
            ..Default::default()
        };
        output.intent = PlanningIntent {
            hold_brake: true,
            ..Default::default()
        };
        output.parking_planning = Some(diagnostic);
        output
    }
}
impl Drop for Runtime {
    fn drop(&mut self) {
        self.planner_cancel.store(true, Ordering::Release);
        // No synchronous join: channels close, cancellation interrupts search,
        // and a late result cannot reach a dropped controlling runtime.
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    use crate::parking_mission::{ParkingMission, ParkingTarget, SlotReference, VerifiedBoundary};
    fn runtime(delay: Duration) -> Runtime {
        let mut config = crate::replay::config(ExecutionMode::Live);
        config.mode = DrivingMode::Parking {
            mission: ParkingMission {
                target: ParkingTarget {
                    source_id: "synthetic async lifecycle fixture; no game evidence".into(),
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
                    evidence_path: "synthetic isolated boundary".into(),
                },
                static_obstacles: vec![],
                max_expansions: 1000,
                maximum_duration_s: 60.0,
                recovery: Default::default(),
            },
        };
        let base = crate::replay::calibration();
        let mut calibration = crate::replay::low_speed_calibration();
        calibration.reference = base.reference;
        calibration.geometry_fingerprint = base.geometry_fingerprint;
        let calibration = crate::replay::with_measured_envelope(calibration);
        let token = VerifiedLiveExperiment::from_host_verification(
            &config,
            &calibration,
            None,
            LiveExperimentKind::Parking,
            [1; 32],
            Some([2; 32]),
        )
        .unwrap();
        let mut runtime = Runtime::new_live_experiment(config, calibration, None, token).unwrap();
        runtime.parking_test_delay = delay;
        runtime
    }
    fn frame(n: u64) -> GameSnapshot {
        let state = TruckState {
            pose: Pose2::default(),
            trailer_yaw: 0.0,
            speed: 0.0,
            steer: 0.0,
        };
        let mut snapshot = crate::replay::simulated_frame(state, n, None, false);
        snapshot.raw.header.qpc = etsads_shm::qpc();
        snapshot.raw.qpc_frequency = etsads_shm::qpc_frequency();
        snapshot.raw.header.sim_time_us = snapshot.ego.header.sim_time_us;
        snapshot.raw.paused_simulation_time_us = snapshot.ego.header.sim_time_us;
        let ext = snapshot.extension.as_mut().unwrap();
        ext.header = snapshot.raw.header;
        ext.available = 6;
        snapshot
    }
    fn parked(snapshot: &mut GameSnapshot, x: f32, y: f32) {
        let object = etsads_game::ets2la::Parked {
            position: [x, 0.0, -y],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            size: [1.0, 2.0, 1.0],
            id: 1,
            is_trailer: 0,
        };
        snapshot.extension.as_mut().unwrap().parked[..43]
            .copy_from_slice(bytemuck::bytes_of(&object));
    }
    #[test]
    fn mock_slow_search_requires_new_fresh_request_and_ignores_outside_path_traffic() {
        let mut runtime = runtime(Duration::from_millis(260));
        let mut original = None;
        let mut revalidation = None;
        let mut accepted = None;
        for n in 0..180 {
            let mut snapshot = frame(n);
            // Inside the old whole-path AABB, but outside the actual swept
            // bodies. Its changing position must not cause endless retries.
            parked(&mut snapshot, -18.0 + n as f32 * 0.01, 2.4);
            let output = runtime.step(&snapshot);
            assert_ne!(output.status, RuntimeStatus::Fault, "{}", output.reason);
            if let Some(wait) = output.parking_planning {
                assert_eq!(output.status, RuntimeStatus::WaitingForParkingPlan);
                assert!(
                    !output.request_control
                        && output.actuation.throttle == 0.0
                        && output.actuation.brake >= 0.4
                );
                assert!(output.intent.gear_request.is_none() && !output.intent.activate);
                original.get_or_insert(wait.original_frame_id);
                if wait.phase == ParkingPlanningPhase::Revalidating {
                    revalidation = Some(wait.request_frame_id);
                }
            }
            if output.status == RuntimeStatus::Running {
                accepted = output.trajectory;
                assert!(output.intent.parking_search.is_some());
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let trajectory = accepted.expect("stationary candidate should be adopted");
        assert!(revalidation.unwrap() > original.unwrap());
        assert_eq!(
            trajectory.header.frame_id,
            revalidation.unwrap(),
            "no retimestamping of the original result"
        );
    }
    #[test]
    fn movement_pause_session_and_wall_timeout_cancel_without_waiting_for_worker() {
        for scenario in [
            "moving",
            "drift",
            "pause",
            "session",
            "configuration",
            "extension",
            "timeout",
        ] {
            let mut runtime = runtime(Duration::from_millis(300));
            runtime.step(&frame(0));
            let waiting = runtime.step(&frame(1));
            assert_eq!(
                waiting.status,
                RuntimeStatus::WaitingForParkingPlan,
                "{}",
                waiting.reason
            );
            let cancel = runtime.planner_cancel.clone();
            let mut current = frame(2);
            match scenario {
                "moving" => {
                    current.ego.speed = 0.2;
                    current.raw.speed_mps = 0.2;
                }
                "drift" => {
                    current.raw.position_scs[0] += 0.2;
                    current.ego.pose.x += 0.2;
                }
                "pause" => {
                    current.ego.paused = true;
                    current.raw.paused = 1;
                }
                "session" => current.session_start_qpc += 1,
                "configuration" => current.config.as_mut().unwrap().config_seq += 1,
                "extension" => current.extension.as_mut().unwrap().available = 4,
                "timeout" => {
                    runtime.parking_async.pending.as_mut().unwrap().started =
                        Instant::now() - Duration::from_secs(31)
                }
                _ => unreachable!(),
            }
            let output = runtime.step(&current);
            assert!(
                matches!(output.status, RuntimeStatus::Fault | RuntimeStatus::Paused),
                "{scenario}: {}",
                output.reason
            );
            assert!(cancel.load(Ordering::Acquire));
            assert!(runtime.parking_async.pending.is_none());
            let start = Instant::now();
            drop(runtime);
            assert!(
                start.elapsed() < Duration::from_millis(100),
                "Drop must not join a slow search"
            );
        }
    }
    #[test]
    fn path_obstacle_change_forces_new_validation_and_never_reuses_old_candidate() {
        let mut runtime = runtime(Duration::from_millis(260));
        let mut add_obstacle = false;
        let mut rejected = false;
        let mut serial = 0;
        let mut maximum_serial = 0;
        for n in 0..180 {
            let mut snapshot = frame(n);
            if add_obstacle {
                parked(&mut snapshot, -18.0, 0.0);
            }
            let output = runtime.step(&snapshot);
            maximum_serial = maximum_serial.max(runtime.parking_async.serial);
            assert_ne!(
                output.status,
                RuntimeStatus::Running,
                "changed path must not gain motion authority"
            );
            if output
                .parking_planning
                .as_ref()
                .is_some_and(|d| d.phase == ParkingPlanningPhase::Revalidating)
            {
                if !add_obstacle {
                    serial = runtime.parking_async.serial;
                }
                add_obstacle = true;
            }
            if output.status == RuntimeStatus::Fault {
                assert!(
                    output.reason.contains("sweep") || output.reason.contains("obstacle"),
                    "{}",
                    output.reason
                );
                rejected = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(rejected && add_obstacle);
        assert!(
            maximum_serial > serial,
            "changed current geometry must produce an actual third validation request"
        );
    }
    #[test]
    fn adoption_and_moving_parking_keep_200ms_and_nonparking_has_no_exception() {
        let mut slow = runtime(Duration::from_millis(260));
        slow.parking_test_validation_delay = Duration::from_millis(260);
        let mut failed = false;
        for n in 0..180 {
            let output = slow.step(&frame(n));
            if output.status == RuntimeStatus::Fault {
                assert!(output.reason.contains("adoption"), "{}", output.reason);
                failed = true;
                break;
            }
            assert_ne!(output.status, RuntimeStatus::Running);
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(failed);
        let mut moving = runtime(Duration::from_millis(300));
        let mut stopped = false;
        for n in 0..20 {
            let mut s = frame(n);
            s.ego.speed = 0.2;
            s.raw.speed_mps = 0.2;
            let output = moving.step(&s);
            assert_ne!(output.status, RuntimeStatus::WaitingForParkingPlan);
            if output.status == RuntimeStatus::Fault {
                assert!(output.reason.contains("200 ms"));
                stopped = true;
                break;
            }
        }
        assert!(stopped);
        let mut config = crate::replay::config(ExecutionMode::Live);
        config.mode = DrivingMode::CommissioningCircle {
            radius_m: 50.0,
            speed_mps: 6.0,
            clockwise: false,
            verified_clear_area: vec![
                (-200.0, -200.0),
                (200.0, -200.0),
                (200.0, 200.0),
                (-200.0, 200.0),
            ],
        };
        let calibration = crate::replay::with_measured_envelope(crate::replay::calibration());
        let mut road = Runtime::new(config, calibration, None).unwrap();
        let (requests, _unconsumed) = std::sync::mpsc::sync_channel(1);
        road.planner_tx = Some(requests);
        let (_sender, receiver) = std::sync::mpsc::channel();
        road.planner_rx = Some(receiver);
        let mut fault = false;
        for n in 0..20 {
            let output = road.step(&frame(n));
            assert_ne!(output.status, RuntimeStatus::WaitingForParkingPlan);
            if output.status == RuntimeStatus::Fault {
                assert!(output.reason.contains("200 ms"), "{}", output.reason);
                fault = true;
                break;
            }
        }
        assert!(fault);
    }
    #[test]
    fn gear_change_after_fresh_adoption_request_cannot_receive_motion_throttle() {
        let mut runtime = runtime(Duration::from_millis(260));
        let mut changed = false;
        let mut rejected = false;
        for n in 0..180 {
            let mut snapshot = frame(n);
            snapshot.raw.gear = if changed { 1 } else { -1 };
            let output = runtime.step(&snapshot);
            assert_ne!(
                output.status,
                RuntimeStatus::Running,
                "old reverse acknowledgement cannot authorize drive gear"
            );
            if output
                .parking_planning
                .as_ref()
                .is_some_and(|p| p.phase == ParkingPlanningPhase::Revalidating)
            {
                changed = true;
            }
            if output.status == RuntimeStatus::Fault {
                assert!(output.reason.contains("measured gear"), "{}", output.reason);
                assert!(!output.request_control && output.actuation.throttle == 0.0);
                rejected = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(changed && rejected);
    }
}
