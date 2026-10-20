use super::{RecordedCase, ScenarioLibrary, graph_hash, nearest, seed, source_hashes};
use anyhow::{Context, ensure};
use etsads_control::{TruckState, identification::VehicleCalibration};
use etsads_game::adapter::GameSnapshot;
use etsads_map::{LaneGraph, importer::RegionGraph};
use etsads_msgs::{Actuation, Agent, Header, OrientedBox, Point3, Pose2, frames::wrap_angle};
use etsads_planning::{
    geometry::{footprints, overlaps},
    speed::IdmConfig,
};
use etsads_runtime::{
    ExecutionMode, RouteSource, Runtime, RuntimeConfig, RuntimeOutput, RuntimeStatus,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeSet, VecDeque},
    path::Path,
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CaseMetrics {
    pub simulated_seconds: f64,
    pub distance_m: f64,
    pub collision_frames: u64,
    pub maximum_deceleration_mps2: f64,
    pub maximum_lateral_error_m: f64,
    pub minimum_agent_gap_m: Option<f64>,
    pub reactive_ego_braking_steps: u64,
    pub unmapped_agents: usize,
    pub fault: Option<String>,
}
#[derive(Serialize)]
pub struct CaseReport {
    pub schema_version: u32,
    pub case: RecordedCase,
    pub execution: &'static str,
    pub source: String,
    pub source_hashes: std::collections::BTreeMap<String, String>,
    pub map_sha256: String,
    pub calibration: VehicleCalibration,
    pub config: RuntimeConfig,
    pub metrics: CaseMetrics,
    pub completed: bool,
    pub phase_acceptance: bool,
    pub control_written: bool,
    pub limitations: Vec<String>,
}
#[derive(Serialize)]
pub struct SimStep {
    pub elapsed_s: f64,
    pub state: TruckState,
    pub agents: Vec<Agent>,
    pub runtime: RuntimeOutput,
    pub applied_axis: f64,
    pub applied_acceleration: f64,
}
#[derive(Clone)]
struct Actor {
    agent: Agent,
    lane: Option<u64>,
    s: f64,
    lateral_offset: f64,
    parked: bool,
    desired: f64,
    raw: Option<etsads_game::ets2la::Traffic>,
}
struct World {
    actors: Vec<Actor>,
}
impl World {
    fn new(snapshot: &GameSnapshot, graph: &LaneGraph) -> anyhow::Result<Self> {
        ensure!(
            snapshot.scene.agents.len() <= 200,
            "too many initial actors"
        );
        let mut actors = snapshot
            .scene
            .agents
            .iter()
            .map(|agent| {
                ensure!(agent.valid(), "invalid recorded actor");
                let p = nearest(graph, agent.pose, agent.z).filter(|p| {
                    p.d.abs() <= (graph.lanes[&p.lane_id].width - agent.width).max(0.0) / 2.0
                        && wrap_angle(p.heading - agent.pose.yaw).abs() <= 0.2
                });
                let mut simulated = agent.clone();
                if p.is_none() {
                    simulated.speed = 0.0;
                    simulated.acceleration = 0.0;
                }
                Ok(Actor {
                    agent: simulated,
                    lane: p.map(|p| p.lane_id),
                    s: p.map_or(0.0, |p| p.s),
                    lateral_offset: p.map_or(0.0, |p| p.d),
                    parked: false,
                    desired: p.map_or(agent.speed.max(1.0), |p| {
                        graph.lanes[&p.lane_id].speed_limit
                    }),
                    raw: snapshot.extension.as_ref().and_then(|e| {
                        etsads_game::ets2la::records::<etsads_game::ets2la::Traffic>(&e.traffic)
                            .find(|item| item.id >= 0 && item.id as u64 == agent.id)
                    }),
                })
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        if let Some(ext) = snapshot.extension.as_ref().filter(|e| e.available & 4 != 0) {
            for parked in etsads_game::ets2la::records::<etsads_game::ets2la::Parked>(&ext.parked) {
                if bytemuck::bytes_of(&parked).iter().all(|b| *b == 0) || parked.id < 0 {
                    continue;
                }
                let Some(yaw) = etsads_game::ets2la::yaw_scs_quaternion(parked.quaternion_wxyz)
                else {
                    continue;
                };
                let agent = Agent {
                    id: (1u64 << 32) | parked.id as u64,
                    pose: Pose2 {
                        x: parked.position[0] as f64,
                        y: -parked.position[2] as f64,
                        yaw,
                    },
                    z: parked.position[1] as f64,
                    speed: 0.0,
                    acceleration: 0.0,
                    length: parked.size[2] as f64,
                    width: parked.size[0] as f64,
                    lane_id: None,
                    trailers: vec![],
                };
                ensure!(agent.valid(), "invalid parked actor");
                actors.push(Actor {
                    agent,
                    lane: None,
                    s: 0.0,
                    lateral_offset: 0.0,
                    parked: true,
                    desired: 1.0,
                    raw: None,
                });
            }
        }
        ensure!(
            actors.len() <= 200,
            "combined traffic and parked actors exceed Runtime budget"
        );
        Ok(Self { actors })
    }
    fn advance(
        &mut self,
        graph: &LaneGraph,
        ego: TruckState,
        ego_z: f64,
        g: etsads_msgs::VehicleGeometry,
        dt: f64,
    ) -> anyhow::Result<(u64, Option<f64>)> {
        let old = self.actors.clone();
        let mut reactions = 0;
        let mut minimum = None::<f64>;
        let ego_boxes = footprints(ego, g);
        for actor in &mut self.actors {
            let Some(lane) = actor.lane else { continue };
            if actor.parked {
                continue;
            }
            let mut lead = None::<(f64, f64)>;
            let mut ego_lead = false;
            for body in &ego_boxes {
                if let Some(gap) = forward_gap(
                    graph,
                    lane,
                    actor.s,
                    &actor.agent,
                    body,
                    ego_z,
                    ego.speed.max(0.0),
                ) && lead.is_none_or(|l| gap.0 < l.0)
                {
                    lead = Some(gap);
                    ego_lead = true;
                }
            }
            for other in &old {
                if other.agent.id == actor.agent.id {
                    continue;
                }
                let mut bodies = vec![OrientedBox {
                    pose: other.agent.pose,
                    length: other.agent.length,
                    width: other.agent.width,
                }];
                bodies.extend_from_slice(&other.agent.trailers);
                for body in bodies {
                    if let Some(gap) = forward_gap(
                        graph,
                        lane,
                        actor.s,
                        &actor.agent,
                        &body,
                        other.agent.z,
                        other.agent.speed.max(0.0),
                    ) && lead.is_none_or(|l| gap.0 < l.0)
                    {
                        lead = Some(gap);
                        ego_lead = false;
                    }
                }
            }
            // A missing map successor is a stopping boundary, not a teleport.
            let remaining = graph.lanes[&lane].length() - actor.s;
            if graph.lanes[&lane].successors.is_empty() && lead.is_none_or(|l| remaining < l.0) {
                lead = Some((remaining, 0.0));
                ego_lead = false;
            }
            let a = IdmConfig {
                desired_speed: actor.desired,
                minimum_gap: 3.0,
                time_headway: 1.5,
                ..Default::default()
            }
            .acceleration(actor.agent.speed.max(0.0), lead)?;
            if ego_lead && a < 0.0 {
                reactions += 1;
            }
            if let Some((gap, _)) = lead {
                minimum = Some(minimum.map_or(gap, |m| m.min(gap)));
            }
            let speed = actor.agent.speed.max(0.0);
            let a = a.max(-speed / dt);
            let distance = (speed * dt + 0.5 * a * dt * dt).max(0.0);
            actor.agent.speed = (speed + a * dt).max(0.0);
            actor.agent.acceleration = a;
            actor.s += distance;
            let mut id = lane;
            for _ in 0..16 {
                let current = &graph.lanes[&id];
                if actor.s <= current.length() {
                    break;
                }
                let heading = current.sample(current.length()).unwrap().1;
                let next = current.successors.iter().copied().min_by(|a, b| {
                    let ah = wrap_angle(graph.lanes[a].sample(0.0).unwrap().1 - heading).abs();
                    let bh = wrap_angle(graph.lanes[b].sample(0.0).unwrap().1 - heading).abs();
                    ah.total_cmp(&bh).then(a.cmp(b))
                });
                let Some(next) = next else {
                    actor.s = current.length();
                    actor.agent.speed = 0.0;
                    break;
                };
                actor.s -= current.length();
                id = next;
            }
            ensure!(
                actor.s <= graph.lanes[&id].length() + 1e-6,
                "actor exceeded route traversal budget"
            );
            actor.lane = Some(id);
            actor.agent.lane_id = Some(id);
            let (p, yaw) = graph.lanes[&id]
                .sample(actor.s)
                .context("actor map sample")?;
            let old_pose = actor.agent.pose;
            let rotation = wrap_angle(yaw - old_pose.yaw);
            actor.agent.pose = Pose2 {
                x: p.x - yaw.sin() * actor.lateral_offset,
                y: p.y + yaw.cos() * actor.lateral_offset,
                yaw,
            };
            actor.agent.z = p.z;
            // Target hitch geometry is absent from ETS2LA. Preserve its rigid
            // compound envelope rather than invent articulated trailer axles.
            for trailer in &mut actor.agent.trailers {
                let dx = trailer.pose.x - old_pose.x;
                let dy = trailer.pose.y - old_pose.y;
                trailer.pose.x = actor.agent.pose.x + rotation.cos() * dx - rotation.sin() * dy;
                trailer.pose.y = actor.agent.pose.y + rotation.sin() * dx + rotation.cos() * dy;
                trailer.pose.yaw = wrap_angle(trailer.pose.yaw + rotation);
            }
        }
        Ok((reactions, minimum))
    }
    fn publish(&self, extension: &mut etsads_msgs::protocol::ExtensionFrame) -> anyhow::Result<()> {
        extension.traffic.fill(0);
        let mut offset = 0;
        for actor in &self.actors {
            let Some(mut raw) = actor.raw else { continue };
            let a = &actor.agent;
            raw.position = [a.pose.x as f32, a.z as f32, -a.pose.y as f32];
            let angle = (a.pose.yaw - std::f64::consts::FRAC_PI_2) / 2.0;
            raw.quaternion_wxyz = [angle.cos() as f32, 0.0, angle.sin() as f32, 0.0];
            raw.speed = a.speed as f32;
            raw.acceleration = a.acceleration as f32;
            let mut trailers = raw.trailers;
            for (value, body) in trailers.iter_mut().zip(&a.trailers) {
                value.position = [body.pose.x as f32, a.z as f32, -body.pose.y as f32];
                let angle = (body.pose.yaw - std::f64::consts::FRAC_PI_2) / 2.0;
                value.quaternion_wxyz = [angle.cos() as f32, 0.0, angle.sin() as f32, 0.0];
            }
            raw.trailers = trailers;
            let bytes = bytemuck::bytes_of(&raw);
            ensure!(
                offset + bytes.len() <= extension.traffic.len(),
                "simulated traffic wire capacity exceeded"
            );
            extension.traffic[offset..offset + bytes.len()].copy_from_slice(bytes);
            offset += bytes.len();
        }
        extension.available |= 2;
        Ok(())
    }
}
fn forward_gap(
    graph: &LaneGraph,
    lane: u64,
    s: f64,
    actor: &Agent,
    body: &OrientedBox,
    body_z: f64,
    speed: f64,
) -> Option<(f64, f64)> {
    if (body_z - actor.z).abs() >= 3.0 {
        return None;
    }
    let mut pending = VecDeque::from([(lane, -s)]);
    let mut seen = BTreeSet::new();
    while let Some((id, offset)) = pending.pop_front() {
        if !seen.insert(id) || seen.len() > 32 {
            continue;
        }
        let p = graph
            .project(
                id,
                Point3 {
                    x: body.pose.x,
                    y: body.pose.y,
                    z: body_z,
                },
            )
            .ok()?;
        // Projecting beyond the endpoint must not put a remote body at the
        // clipped endpoint. Compare Euclidean distance and vertical level.
        if p.distance < (actor.width + body.width) / 2.0 + 0.2 && (p.z - body_z).abs() < 3.0 {
            let along = offset + p.s;
            if along > 0.0 {
                return Some((along - (actor.length + body.length) / 2.0, speed));
            }
        }
        let next_offset = offset + graph.lanes[&id].length();
        if next_offset < 150.0 {
            for next in &graph.lanes[&id].successors {
                pending.push_back((*next, next_offset));
            }
        }
    }
    None
}
fn curvature(cal: &VehicleCalibration, axis: f64, speed: f64) -> anyhow::Result<f64> {
    ensure!(
        speed >= cal.steering_rows[0].speed - 1.0
            && speed <= cal.steering_rows.last().unwrap().speed + 1.0,
        "simulation speed left calibrated steering support"
    );
    let eval = |row: &etsads_control::identification::SteeringRow| -> anyhow::Result<f64> {
        ensure!(row.samples.len() >= 2, "steering row too short");
        let mut points = row.samples.clone();
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        ensure!(
            axis >= points[0].0 - 1e-9 && axis <= points.last().unwrap().0 + 1e-9,
            "simulation axis outside calibration"
        );
        for pair in points.windows(2) {
            ensure!(pair[1].0 > pair[0].0, "repeated steering axis");
            if axis <= pair[1].0 {
                let t = (axis - pair[0].0) / (pair[1].0 - pair[0].0);
                return Ok(pair[0].1 + t * (pair[1].1 - pair[0].1));
            }
        }
        Ok(points.last().unwrap().1)
    };
    for pair in cal.steering_rows.windows(2) {
        if speed <= pair[1].speed {
            let t = ((speed - pair[0].speed) / (pair[1].speed - pair[0].speed)).clamp(0.0, 1.0);
            return Ok(eval(&pair[0])? * (1.0 - t) + eval(&pair[1])? * t);
        }
    }
    eval(cal.steering_rows.last().unwrap())
}
fn update_snapshot(
    snapshot: &mut GameSnapshot,
    state: TruckState,
    cal: &VehicleCalibration,
    index: u64,
    elapsed: f64,
    acceleration: f64,
) {
    let time = (elapsed * 1e6).round() as u64;
    let previous = TruckState::from(snapshot.ego);
    if cal.geometry.has_trailer {
        let old_axle = previous.trailer_axle(cal.geometry);
        let axle = state.trailer_axle(cal.geometry);
        let rotation = wrap_angle(state.trailer_yaw - previous.trailer_yaw);
        for trailer in snapshot
            .raw
            .trailers
            .iter_mut()
            .filter(|t| t.connected != 0)
        {
            let old = trailer.position_scs;
            let dx = old[0] - old_axle.x;
            let dy = -old[2] - old_axle.y;
            trailer.position_scs = [
                axle.x + rotation.cos() * dx - rotation.sin() * dy,
                old[1],
                -(axle.y + rotation.sin() * dx + rotation.cos() * dy),
            ];
            trailer.orientation_turns = [
                ((state.trailer_yaw - std::f64::consts::FRAC_PI_2) / std::f64::consts::TAU)
                    .rem_euclid(1.0) as f32,
                0.0,
                0.0,
            ];
            trailer.velocity_scs =
                std::array::from_fn(|i| ((trailer.position_scs[i] - old[i]) / 0.02) as f32);
        }
    }
    snapshot.ego.header = Header {
        frame_id: index + 1,
        sim_time_us: time,
        seq: index + 1,
    };
    snapshot.ego.pose = state.pose;
    snapshot.ego.speed = state.speed;
    snapshot.ego.acceleration = acceleration;
    snapshot.ego.wheel_angle = state.steer;
    snapshot.ego.trailer_yaw = state.trailer_yaw;
    snapshot.ego.paused = false;
    snapshot.ego.yaw_rate = state.speed * state.steer.tan() / cal.geometry.wheelbase;
    snapshot.raw.header.frame_id = index + 1;
    snapshot.raw.header.seq = index + 1;
    snapshot.raw.header.sim_time_us = time;
    snapshot.raw.header.qpc = time + 1;
    snapshot.raw.header.source = 0x0053_494d; // SIM: model output, not game input.
    snapshot.raw.qpc_frequency = 1_000_000;
    snapshot.raw.paused_simulation_time_us = time;
    snapshot.raw.paused = 0;
    snapshot.raw.orientation_turns = [
        ((state.pose.yaw - std::f64::consts::FRAC_PI_2) / std::f64::consts::TAU).rem_euclid(1.0)
            as f32,
        0.0,
        0.0,
    ];
    let offset = cal
        .reference
        .as_ref()
        .map_or(Point3::default(), |r| r.sdk_to_rear_axle_scs);
    // At flat-road yaw, SDK local -Z is forward and +X is right.
    let dx = -offset.z * state.pose.yaw.cos() + offset.x * state.pose.yaw.sin();
    let dy = -offset.z * state.pose.yaw.sin() - offset.x * state.pose.yaw.cos();
    snapshot.raw.position_scs = [
        state.pose.x - dx,
        snapshot.ego.z - offset.y,
        -(state.pose.y - dy),
    ];
    snapshot.raw.speed_mps = state.speed as f32;
    snapshot.raw.velocity_local = [0.0, 0.0, -state.speed as f32];
    snapshot.raw.acceleration_local = [0.0, 0.0, -acceleration as f32];
    snapshot
        .raw
        .wheel_steering
        .fill((state.steer / std::f64::consts::TAU) as f32);
    snapshot.scene.ego = snapshot.ego;
    snapshot.events.clear();
    if let Some(ext) = &mut snapshot.extension {
        ext.header = snapshot.raw.header;
    }
}
/// Run the production Runtime against calibrated ego dynamics and interactive
/// traffic. Emits every step to a caller-owned streaming writer; no game IPC.
pub fn run_case(
    run: &Path,
    library: &ScenarioLibrary,
    case: &RecordedCase,
    config: RuntimeConfig,
    calibration: VehicleCalibration,
    region: RegionGraph,
    mut receive: impl FnMut(&SimStep) -> anyhow::Result<()>,
) -> anyhow::Result<CaseReport> {
    ensure!(
        library.schema_version == 1 && !library.phase_acceptance,
        "invalid scenario library"
    );
    ensure!(
        library.cases.contains(case),
        "case does not belong to library"
    );
    ensure!(
        config.execution == ExecutionMode::Replay
            && matches!(config.route_source, RouteSource::Explicit),
        "recording simulation requires Replay and an explicit regional route"
    );
    ensure!(
        !matches!(config.mode, etsads_runtime::DrivingMode::Parking { .. }),
        "recording traffic simulator does not model parking gear/contact events"
    );
    ensure!(
        source_hashes(run)? == library.source_hashes,
        "recording changed since extraction"
    );
    let graph = region.imported.graph.clone();
    ensure!(
        graph_hash(&graph)? == library.map_sha256,
        "scenario graph changed since extraction"
    );
    let mut seed = seed(run, case)?;
    ensure!(
        seed.source == library.source,
        "library source disagrees with source recording"
    );
    ensure!(
        seed.snapshot
            .extension
            .as_ref()
            .is_some_and(|e| e.available & 2 != 0),
        "initial traffic unavailable; cannot assume an empty scene"
    );
    ensure!(
        seed.snapshot.ego.speed >= 0.0,
        "reverse initial state unsupported"
    );
    ensure!(
        seed.snapshot
            .raw
            .trailers
            .iter()
            .filter(|t| t.connected != 0)
            .count()
            == usize::from(calibration.geometry.has_trailer),
        "seed attachment count does not match single-trailer simulation model"
    );
    let mut runtime = Runtime::new(config.clone(), calibration.clone(), Some(region))?;
    let model = calibration
        .longitudinal
        .context("identified longitudinal model required")?;
    ensure!(
        model.coefficients.iter().all(|x| x.is_finite()),
        "invalid identified model"
    );
    let delay = calibration
        .delay_s
        .context("identified steering delay required")?;
    let tau = calibration
        .steering_tau_s
        .context("identified steering lag required")?;
    ensure!(
        (0.0..=2.0).contains(&delay)
            && (0.001..=5.0).contains(&tau)
            && !calibration.steering_rows.is_empty(),
        "invalid steering dynamics"
    );
    let mut state = TruckState::from(seed.snapshot.ego);
    if let Some(reference) = &calibration.reference {
        let raw = &seed.snapshot.raw;
        let p = etsads_msgs::frames::placement_offset_to_enu(
            Point3 {
                x: raw.position_scs[0],
                y: raw.position_scs[1],
                z: raw.position_scs[2],
            },
            raw.orientation_turns.map(|x| x as f64),
            reference.sdk_to_rear_axle_scs,
        )
        .map_err(anyhow::Error::msg)?;
        state.pose.x = p.x;
        state.pose.y = p.y;
        seed.snapshot.ego.z = p.z;
    }
    seed.snapshot.ego.pose = state.pose;
    let mut world = World::new(&seed.snapshot, &graph)?;
    let mut metrics = CaseMetrics {
        simulated_seconds: 0.0,
        distance_m: 0.0,
        collision_frames: 0,
        maximum_deceleration_mps2: 0.0,
        maximum_lateral_error_m: 0.0,
        minimum_agent_gap_m: None,
        reactive_ego_braking_steps: 0,
        unmapped_agents: world.actors.iter().filter(|a| a.lane.is_none()).count(),
        fault: None,
    };
    let dt = 0.02;
    let duration = (case.end_sim_time_us - case.start_sim_time_us) as f64 / 1e6;
    let steps = (duration / dt).floor() as u64;
    let mut pending = VecDeque::new();
    let mut applied = Actuation::default();
    let mut acceleration = 0.0;
    let mut signal_index = 0;
    seed.snapshot.session_start_qpc = 1;
    seed.snapshot.raw.gear = 1;
    if let Some(ext) = &mut seed.snapshot.extension {
        ext.available &= 12;
    }
    for index in 0..=steps {
        let elapsed = index as f64 * dt;
        update_snapshot(
            &mut seed.snapshot,
            state,
            &calibration,
            index,
            elapsed,
            acceleration,
        );
        seed.snapshot.raw.input = [
            applied.steer as f32,
            applied.throttle as f32,
            applied.brake as f32,
            0.0,
        ];
        seed.snapshot.raw.effective = seed.snapshot.raw.input;
        let source_time = case.start_sim_time_us + (elapsed * 1e6) as u64;
        while signal_index + 1 < seed.signals.len()
            && seed.signals[signal_index + 1].0 <= source_time
        {
            signal_index += 1;
        }
        if let Some(ext) = &mut seed.snapshot.extension {
            world.publish(ext)?;
            let signals = &seed.signals[signal_index].1;
            if signals.len() == ext.signals.len() {
                ext.signals.copy_from_slice(signals);
                ext.available |= 8;
            } else {
                ext.available &= !8;
            }
        }
        seed.snapshot.scene.agents = world.actors.iter().map(|a| a.agent.clone()).collect();
        let output = runtime.step(&seed.snapshot);
        let fault = (output.status == RuntimeStatus::Fault).then(|| output.reason.clone());
        let done = output.status == RuntimeStatus::Completed;
        let command = if output.request_control {
            output.actuation
        } else {
            Actuation::default()
        };
        pending.push_back((elapsed, command));
        while pending
            .front()
            .is_some_and(|(t, _)| elapsed - *t >= delay - 1e-9)
        {
            applied = pending.pop_front().unwrap().1;
        }
        ensure!(
            pending.len() <= 102,
            "actuator queue exceeded bounded delay"
        );
        let k = curvature(&calibration, applied.steer, state.speed)?;
        let target = (k * calibration.geometry.wheelbase).atan().clamp(
            -calibration.geometry.max_steer,
            calibration.geometry.max_steer,
        );
        let steer = state.steer + (target - state.steer) * (1.0 - (-dt / tau).exp());
        let c = model.coefficients;
        acceleration = (c[0] + c[1] * applied.throttle + c[2] * applied.brake + c[3] * state.speed)
            .max(-state.speed / dt);
        ensure!(
            acceleration.is_finite() && acceleration.abs() <= 15.0,
            "identified model left bounded simulation range"
        );
        let bodies = footprints(state, calibration.geometry);
        if world.actors.iter().any(|a| {
            (a.agent.z - seed.snapshot.ego.z).abs() < 3.0
                && bodies.iter().any(|b| {
                    overlaps(
                        *b,
                        OrientedBox {
                            pose: a.agent.pose,
                            length: a.agent.length,
                            width: a.agent.width,
                        },
                        0.0,
                    ) || a.agent.trailers.iter().any(|t| overlaps(*b, *t, 0.0))
                })
        }) {
            metrics.collision_frames += 1;
        }
        metrics.maximum_deceleration_mps2 = metrics.maximum_deceleration_mps2.max(-acceleration);
        if let Some(p) = nearest(&graph, state.pose, seed.snapshot.ego.z) {
            metrics.maximum_lateral_error_m = metrics.maximum_lateral_error_m.max(p.d.abs());
        }
        receive(&SimStep {
            elapsed_s: elapsed,
            state,
            agents: seed.snapshot.scene.agents.clone(),
            runtime: output,
            applied_axis: applied.steer,
            applied_acceleration: acceleration,
        })?;
        metrics.simulated_seconds = elapsed;
        if let Some(fault) = fault {
            metrics.fault = Some(fault);
            break;
        }
        if done || index == steps {
            break;
        }
        let (reactive, gap) =
            world.advance(&graph, state, seed.snapshot.ego.z, calibration.geometry, dt)?;
        metrics.reactive_ego_braking_steps += reactive;
        if let Some(gap) = gap {
            metrics.minimum_agent_gap_m =
                Some(metrics.minimum_agent_gap_m.map_or(gap, |g| g.min(gap)));
        }
        let next = state.step(calibration.geometry, steer, acceleration, dt)?;
        metrics.distance_m += state.pose.distance(next.pose);
        state = next;
    }
    ensure!(
        source_hashes(run)? == library.source_hashes,
        "source changed during simulation"
    );
    Ok(CaseReport{schema_version:1,case:case.clone(),execution:"recording_seeded_closed_loop",source:library.source.clone(),source_hashes:library.source_hashes.clone(),map_sha256:library.map_sha256.clone(),calibration,config,completed:metrics.fault.is_none(),metrics,phase_acceptance:false,control_written:false,limitations:vec!["Offline model output, never game acceptance; road grade is held flat.".into(),"Observed traffic initializes once, then IDM responds to ego. Unmapped actors remain static. Forks choose the smallest heading change deterministically.".into(),"Signals follow their recorded schedule; gate activation and paid/fine/job events are not replayed as consequences of simulated ego.".into(),"Other vehicles' trailers use a rigid compound envelope because their hitch geometry is not recorded.".into()]})
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn initial_lateral_offset_is_preserved_and_unmapped_motion_is_frozen() {
        let (region, _, _) = crate::recording::tests::fixture();
        let raw = etsads_msgs::protocol::TelemetryFrame {
            qpc_frequency: 1_000_000,
            orientation_turns: [0.75, 0.0, 0.0],
            ..Default::default()
        };
        let mut snapshot =
            etsads_game::adapter::snapshot_from_recorded_frame(raw, None, None).unwrap();
        let agent = Agent {
            id: 1,
            pose: Pose2 {
                x: 0.0,
                y: 0.5,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 4.0,
            acceleration: 1.0,
            length: 4.0,
            width: 2.0,
            lane_id: None,
            trailers: vec![],
        };
        snapshot.scene.agents = vec![
            agent.clone(),
            Agent {
                id: 2,
                pose: Pose2 {
                    x: 0.0,
                    y: 7.0,
                    yaw: 0.0,
                },
                ..agent
            },
        ];
        let mut world = World::new(&snapshot, &region.imported.graph).unwrap();
        assert_eq!(world.actors[1].agent.speed, 0.0);
        assert_eq!(world.actors[1].agent.acceleration, 0.0);
        world
            .advance(
                &region.imported.graph,
                TruckState {
                    pose: Pose2 {
                        x: 200.0,
                        y: 0.0,
                        yaw: 0.0,
                    },
                    ..Default::default()
                },
                0.0,
                etsads_msgs::VehicleGeometry::default(),
                0.02,
            )
            .unwrap();
        assert!((world.actors[0].agent.pose.y - 0.5).abs() < 1e-9);
        assert_eq!(world.actors[1].agent.pose.y, 7.0);
    }
    #[test]
    fn raw_trailer_reference_moves_with_its_model_body() {
        let (_, _, mut calibration) = crate::recording::tests::fixture();
        calibration.geometry = etsads_msgs::VehicleGeometry::default();
        let mut raw = etsads_msgs::protocol::TelemetryFrame {
            qpc_frequency: 1_000_000,
            orientation_turns: [0.75, 0.0, 0.0],
            ..Default::default()
        };
        raw.trailers[0].connected = 1;
        raw.trailers[0].orientation_turns = [0.75, 0.0, 0.0];
        raw.trailers[0].position_scs = [-7.4, 0.0, 0.0];
        let mut snapshot =
            etsads_game::adapter::snapshot_from_recorded_frame(raw, None, None).unwrap();
        let state = TruckState {
            pose: Pose2 {
                x: 1.0,
                y: 2.0,
                yaw: 0.3,
            },
            trailer_yaw: 0.1,
            ..Default::default()
        };
        update_snapshot(&mut snapshot, state, &calibration, 1, 0.02, 0.0);
        let axle = state.trailer_axle(calibration.geometry);
        let actual = snapshot.raw.trailers[0].position_scs;
        assert!((actual[0] - axle.x - 0.1_f64.cos()).abs() < 1e-9);
        assert!((-actual[2] - axle.y - 0.1_f64.sin()).abs() < 1e-9);
        assert!(
            (etsads_msgs::frames::heading_to_yaw(
                snapshot.raw.trailers[0].orientation_turns[0] as f64
            ) - 0.1)
                .abs()
                < 1e-6
        );
    }
    #[test]
    fn bridge_height_is_not_a_leader_and_actor_wire_follows_simulation() {
        let (region, _, _) = crate::recording::tests::fixture();
        let graph = &region.imported.graph;
        let actor = Agent {
            id: 4,
            pose: Pose2 {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 8.0,
            acceleration: 0.0,
            length: 4.0,
            width: 2.0,
            lane_id: Some(1),
            trailers: vec![],
        };
        let body = OrientedBox {
            pose: Pose2 {
                x: 10.0,
                y: 0.0,
                yaw: 0.0,
            },
            length: 6.0,
            width: 2.5,
        };
        assert!(forward_gap(graph, 1, 200.0, &actor, &body, 8.0, 0.0).is_none());
        assert_eq!(
            forward_gap(graph, 1, 200.0, &actor, &body, 0.0, 0.0)
                .unwrap()
                .0,
            5.0
        );
        let mut raw: etsads_game::ets2la::Traffic = bytemuck::Zeroable::zeroed();
        raw.id = 4;
        raw.size = [2.0, 3.25, 4.0];
        let world = World {
            actors: vec![Actor {
                agent: Agent {
                    pose: Pose2 {
                        x: 42.0,
                        y: 7.0,
                        yaw: 0.3,
                    },
                    ..actor
                },
                lane: Some(1),
                s: 242.0,
                lateral_offset: 0.0,
                parked: false,
                desired: 8.0,
                raw: Some(raw),
            }],
        };
        let mut ext = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        world.publish(&mut ext).unwrap();
        let current = etsads_game::ets2la::records::<etsads_game::ets2la::Traffic>(&ext.traffic)
            .next()
            .unwrap();
        let position = current.position;
        let size = current.size;
        assert_eq!(position, [42.0, 0.0, -7.0]);
        assert_eq!(size, [2.0, 3.25, 4.0]);
        assert_eq!(ext.available & 2, 2);
        assert!(
            (etsads_game::ets2la::yaw_scs_quaternion(current.quaternion_wxyz).unwrap() - 0.3).abs()
                < 1e-6
        );
    }
}
