//! Bounded Hybrid A* over articulated tractor states with forward/reverse
//! primitives. The target is the trailer axle, never the tractor centre.
mod connection;
mod holonomic;
pub mod reeds_shepp;
use crate::{
    PlanningError,
    geometry::{box_in_polygon, footprints, overlaps, swept_collision, swept_in_polygon},
};
use etsads_control::TruckState;
use etsads_msgs::{Direction, OrientedBox, Pose2, VehicleGeometry, frames::wrap_angle};
use serde::{Deserialize, Serialize};
use std::collections::{BinaryHeap, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
fn cancelled(token: Option<&AtomicBool>) -> bool {
    token.is_some_and(|t| t.load(Ordering::Acquire))
}
fn check_cancel(token: Option<&AtomicBool>) -> Result<(), PlanningError> {
    if cancelled(token) {
        Err(PlanningError::Invalid("parking search cancelled"))
    } else {
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct ParkingRequest<'a> {
    pub start: TruckState,
    pub target: Pose2,
    pub geometry: VehicleGeometry,
    pub obstacles: &'a [OrientedBox],
    pub drivable: &'a [(f64, f64)],
    pub max_expansions: usize,
    pub position_tolerance: f64,
    pub angle_tolerance: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ParkingPoint {
    pub state: TruckState,
    pub direction: Direction,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingPlan {
    pub points: Vec<ParkingPoint>,
    pub expansions: usize,
    pub cost: f64,
    #[serde(default)]
    pub search: ParkingSearchStatistics,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct ParkingSearchStatistics {
    pub initial_reeds_shepp_m: f64,
    pub initial_dijkstra_m: Option<f64>,
    pub dijkstra_resolution_m: f64,
    pub dijkstra_cells: usize,
    pub dijkstra_settled: usize,
    pub reeds_shepp_candidates_checked: usize,
    pub accepted_connection: String,
    pub analytic_curved: bool,
    pub analytic_length_correction: bool,
    pub analytic_cusps: usize,
    pub analytic_reference: String,
}
#[derive(Clone)]
struct Node {
    state: TruckState,
    parent: Option<usize>,
    direction: Direction,
    cost: f64,
    segment: Vec<TruckState>,
}
#[derive(Clone, Copy)]
struct Open {
    estimate: f64,
    index: usize,
}
impl PartialEq for Open {
    fn eq(&self, o: &Self) -> bool {
        self.index == o.index && self.estimate == o.estimate
    }
}
impl Eq for Open {}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.estimate
            .total_cmp(&self.estimate)
            .then_with(|| o.index.cmp(&self.index))
    }
}
type Key = (i32, i32, i16, i16, i16, bool);
fn key(s: TruckState, d: Direction, origin: Pose2) -> Key {
    (
        ((s.pose.x - origin.x) / 0.6).round() as i32,
        ((s.pose.y - origin.y) / 0.6).round() as i32,
        (wrap_angle(s.pose.yaw) / 0.12).round() as i16,
        (s.articulation() / 0.12).round() as i16,
        (s.steer / 0.12).round() as i16,
        d == Direction::Reverse,
    )
}
fn done(s: TruckState, r: &ParkingRequest<'_>) -> bool {
    let axle = s.trailer_axle(r.geometry);
    axle.distance(r.target) < r.position_tolerance
        && wrap_angle(axle.yaw - r.target.yaw).abs() < r.angle_tolerance
        && (!r.geometry.has_trailer || s.articulation().abs() < 0.2)
}
fn rs_heuristic(s: TruckState, r: &ParkingRequest<'_>, goal: Pose2, radius: f64) -> f64 {
    let reeds_shepp =
        reeds_shepp::distance(s.pose, goal, radius).unwrap_or_else(|_| s.pose.distance(goal));
    if !r.geometry.has_trailer {
        return reeds_shepp;
    }
    let g = r.geometry;
    let hitch_curvature = g.hitch_offset.abs() / radius;
    let denominator = g.trailer_wheelbase
        * (g.max_articulation.cos() - hitch_curvature * g.max_articulation.sin());
    let trailer_radius = if denominator > 0.0 {
        denominator / (g.max_articulation.sin() + hitch_curvature)
    } else {
        0.001
    };
    let trailer = reeds_shepp::distance(s.trailer_axle(g), r.target, trailer_radius).unwrap_or(0.0)
        / (1.0 + hitch_curvature);
    reeds_shepp.max(trailer)
}
fn heuristic(
    s: TruckState,
    r: &ParkingRequest<'_>,
    goal: Pose2,
    radius: f64,
    grid: &holonomic::Potential,
) -> f64 {
    let reeds_shepp = rs_heuristic(s, r, goal, radius);
    reeds_shepp.max(grid.query(s.pose).unwrap_or_else(|| s.pose.distance(goal)))
}
fn safe(from: TruckState, to: TruckState, r: &ParkingRequest<'_>) -> bool {
    if r.geometry.has_trailer && to.articulation().abs() > r.geometry.max_articulation {
        return false;
    }
    let a = footprints(from, r.geometry);
    let b = footprints(to, r.geometry);
    let envelope = r.geometry.resolved_envelope();
    let tractor_angle = wrap_angle(to.pose.yaw - from.pose.yaw);
    let trailer_angle = wrap_angle(to.trailer_yaw - from.trailer_yaw);
    for j in 0..b.len() {
        // Body centres rotate around the measured rear axle/hitch. Their
        // deviation from chord interpolation is bounded by r*angle²/8.
        let centre_error = if j == 0 {
            envelope.tractor_center_from_rear_axle_m().abs() * tractor_angle.powi(2) / 8.0
        } else {
            (r.geometry.hitch_offset.abs() * tractor_angle.powi(2)
                + envelope
                    .trailer
                    .unwrap()
                    .center_from_hitch_m(r.geometry.trailer_wheelbase)
                    .abs()
                    * trailer_angle.powi(2))
                / 8.0
        };
        let before = OrientedBox {
            length: a[j].length + 2.0 * centre_error,
            width: a[j].width + 2.0 * centre_error,
            ..a[j]
        };
        let after = OrientedBox {
            length: b[j].length + 2.0 * centre_error,
            width: b[j].width + 2.0 * centre_error,
            ..b[j]
        };
        if !box_in_polygon(b[j], r.drivable)
            || !swept_in_polygon(before, after, r.drivable)
            || r.obstacles
                .iter()
                .any(|o| overlaps(b[j], *o, 0.3) || swept_collision(before, after, *o, 0.3))
        {
            return false;
        }
    }
    true
}
pub fn plan(r: ParkingRequest<'_>) -> Result<ParkingPlan, PlanningError> {
    plan_internal(r, None)
}
pub fn plan_cancellable(
    r: ParkingRequest<'_>,
    cancelled: &AtomicBool,
) -> Result<ParkingPlan, PlanningError> {
    plan_internal(r, Some(cancelled))
}
fn plan_internal(
    r: ParkingRequest<'_>,
    cancellation: Option<&AtomicBool>,
) -> Result<ParkingPlan, PlanningError> {
    check_cancel(cancellation)?;
    if !r.start.valid()
        || !r.target.valid()
        || !r.geometry.valid()
        || r.drivable.len() < 3
        || r.drivable.len() > 10000
        || r.obstacles.len() > 4096
        || r.start.steer.abs() > r.geometry.max_steer
        || r.drivable
            .iter()
            .any(|(x, y)| !x.is_finite() || !y.is_finite())
        || r.obstacles.iter().any(|b| !b.valid())
        || r.max_expansions == 0
        || r.max_expansions > 1_000_000
        || !r.position_tolerance.is_finite()
        || r.position_tolerance <= 0.0
        || r.position_tolerance > 1.0
        || !r.angle_tolerance.is_finite()
        || r.angle_tolerance <= 0.0
        || r.angle_tolerance > 0.2
    {
        return Err(PlanningError::Invalid("parking request"));
    }
    if !safe(r.start, r.start, &r) {
        return Err(PlanningError::NoPath);
    }
    let start = TruckState {
        speed: 0.0,
        trailer_yaw: if r.geometry.has_trailer {
            r.start.trailer_yaw
        } else {
            r.start.pose.yaw
        },
        ..r.start
    };
    let goal = connection::tractor_goal(&r);
    let radius = r.geometry.wheelbase / r.geometry.max_steer.tan();
    let envelope = r.geometry.resolved_envelope();
    let clearance = (r.geometry.width / 2.0)
        .min(envelope.tractor_front_from_rear_axle_m)
        .min(envelope.tractor_rear_overhang_m);
    let grid = holonomic::Potential::build(r.drivable, r.obstacles, goal, clearance, cancellation)?;
    let mut search = ParkingSearchStatistics {
        initial_reeds_shepp_m: rs_heuristic(start, &r, goal, radius),
        initial_dijkstra_m: grid.query(start.pose),
        dijkstra_resolution_m: grid.resolution,
        dijkstra_cells: grid.cells(),
        dijkstra_settled: grid.settled,
        ..Default::default()
    };
    let mut nodes = vec![Node {
        state: start,
        parent: None,
        direction: Direction::Forward,
        cost: 0.0,
        segment: vec![start],
    }];
    let mut open = BinaryHeap::new();
    open.push(Open {
        estimate: heuristic(start, &r, goal, radius, &grid),
        index: 0,
    });
    let mut visited = HashMap::new();
    visited.insert(key(start, Direction::Forward, start.pose), 0.0);
    let mut expansions = 0;
    while let Some(entry) = open.pop() {
        check_cancel(cancellation)?;
        let current = nodes[entry.index].clone();
        if current.cost
            > *visited
                .get(&key(current.state, current.direction, start.pose))
                .unwrap_or(&f64::INFINITY)
                + 1e-9
        {
            continue;
        }
        expansions += 1;
        if done(current.state, &r) {
            search.accepted_connection = "search_goal".into();
            return Ok(reconstruct(&nodes, entry.index, expansions, search));
        }
        if expansions >= r.max_expansions {
            break;
        }
        let mut connection = straight_connection(current.state, &r, cancellation);
        if connection.is_some() {
            search.accepted_connection = "straight".into();
        }
        if connection.is_none() && expansions % 20 == 1 {
            check_cancel(cancellation)?;
            if let Some(analytic) = connection::connect(
                current.state,
                &r,
                &mut search.reeds_shepp_candidates_checked,
                expansions % 200 == 1,
                cancellation,
            ) {
                search.accepted_connection = "reeds_shepp_articulated".into();
                search.analytic_curved = analytic.curved;
                search.analytic_length_correction = analytic.corrected;
                search.analytic_cusps = analytic.cusps;
                search.analytic_reference = if analytic.trailer_reference {
                    "trailer_axle"
                } else {
                    "tractor_rear_axle"
                }
                .into();
                connection = Some(analytic.states);
            } else if r.geometry.has_trailer {
                check_cancel(cancellation)?;
                connection = feedback_connection(current.state, &r, cancellation);
                if connection.is_some() {
                    search.accepted_connection = "reverse_feedback".into();
                }
            }
        }
        check_cancel(cancellation)?;
        if let Some(segment) = connection {
            let direction = if segment.last().unwrap().speed < 0.0 {
                Direction::Reverse
            } else {
                Direction::Forward
            };
            let end = *segment.last().unwrap();
            let cost =
                current.cost + segment_cost(current.state, current.direction, &segment, r.geometry);
            nodes.push(Node {
                state: end,
                parent: Some(entry.index),
                direction,
                cost,
                segment,
            });
            return Ok(reconstruct(&nodes, nodes.len() - 1, expansions, search));
        }
        for direction in [Direction::Reverse, Direction::Forward] {
            let velocity = if direction == Direction::Reverse {
                -1.0
            } else {
                1.0
            };
            for steer in [-1.0, -0.5, 0.0, 0.5, 1.0] {
                let delta = steer * r.geometry.max_steer;
                let mut state = TruckState {
                    speed: velocity,
                    ..current.state
                };
                let mut segment = Vec::new();
                let mut valid = true;
                let primitive_steps = if current.state.pose.distance(goal) > 12.0 {
                    10
                } else {
                    5
                };
                for _ in 0..primitive_steps {
                    let bounded_delta = delta.clamp(state.steer - 0.06, state.steer + 0.06);
                    match state.step(r.geometry, bounded_delta, 0.0, 0.2) {
                        Ok(next) if safe(state, next, &r) => {
                            segment.push(next);
                            state = next;
                        }
                        _ => {
                            valid = false;
                            break;
                        }
                    }
                }
                if !valid {
                    continue;
                }
                let cost = current.cost
                    + segment_cost(current.state, current.direction, &segment, r.geometry);
                let k = key(state, direction, start.pose);
                if cost >= *visited.get(&k).unwrap_or(&f64::INFINITY) {
                    continue;
                }
                visited.insert(k, cost);
                let estimate = cost + heuristic(state, &r, goal, radius, &grid);
                let index = nodes.len();
                nodes.push(Node {
                    state,
                    parent: Some(entry.index),
                    direction,
                    cost,
                    segment,
                });
                open.push(Open { estimate, index });
            }
        }
    }
    Err(PlanningError::NoPath)
}
/// A bounded reverse feedback rollout is an additional non-optimal connection
/// attempt. Every intermediate truck envelope must remain feasible.
fn feedback_connection(
    state: TruckState,
    r: &ParkingRequest<'_>,
    cancellation: Option<&AtomicBool>,
) -> Option<Vec<TruckState>> {
    let axle = state.trailer_axle(r.geometry);
    let heading_error = wrap_angle(axle.yaw - r.target.yaw);
    let along =
        (axle.x - r.target.x) * r.target.yaw.cos() + (axle.y - r.target.y) * r.target.yaw.sin();
    if !(2.0..=25.0).contains(&along) || heading_error.abs() > 0.5 {
        return None;
    }
    let mut current = TruckState {
        speed: -1.0,
        ..state
    };
    let mut segment = Vec::new();
    for _ in 0..300 {
        if cancelled(cancellation) {
            return None;
        }
        let axle = current.trailer_axle(r.geometry);
        let lateral = -(axle.x - r.target.x) * r.target.yaw.sin()
            + (axle.y - r.target.y) * r.target.yaw.cos();
        let heading = wrap_angle(axle.yaw - r.target.yaw);
        let desired_phi = (-0.12 * lateral + 1.2 * heading).clamp(-0.6, 0.6);
        let phi = current.articulation();
        let desired_rate = -1.5 * (phi - desired_phi);
        let desired = (r.geometry.wheelbase
            * (-desired_rate + phi.sin() / r.geometry.trailer_wheelbase)
            / (1.0 + r.geometry.hitch_offset * phi.cos() / r.geometry.trailer_wheelbase))
            .atan();
        let steer = desired
            .clamp(current.steer - 0.03, current.steer + 0.03)
            .clamp(-r.geometry.max_steer, r.geometry.max_steer);
        let next = current.step(r.geometry, steer, 0.0, 0.1).ok()?;
        if !safe(current, next, r) {
            return None;
        }
        segment.push(next);
        current = next;
        if done(current, r) {
            return Some(segment);
        }
        let remaining = (current.trailer_axle(r.geometry).x - r.target.x) * r.target.yaw.cos()
            + (current.trailer_axle(r.geometry).y - r.target.y) * r.target.yaw.sin();
        if remaining < -r.position_tolerance {
            return None;
        }
    }
    None
}
fn straight_connection(
    state: TruckState,
    r: &ParkingRequest<'_>,
    cancellation: Option<&AtomicBool>,
) -> Option<Vec<TruckState>> {
    let axle = state.trailer_axle(r.geometry);
    let dx = r.target.x - axle.x;
    let dy = r.target.y - axle.y;
    let along = dx * axle.yaw.cos() + dy * axle.yaw.sin();
    let cross = -dx * axle.yaw.sin() + dy * axle.yaw.cos();
    if cross.abs() > r.position_tolerance * 0.5
        || wrap_angle(axle.yaw - r.target.yaw).abs() > r.angle_tolerance * 0.5
        || state.articulation().abs() > 0.01
        || state.steer.abs() > 0.01
        || along.abs() > 30.0
        || along.abs() < 0.01
    {
        return None;
    }
    let velocity = along.signum();
    let n = (along.abs() / 0.2).ceil() as usize;
    let dt = along.abs() / n as f64;
    let mut current = TruckState {
        speed: velocity,
        ..state
    };
    let mut segment = Vec::new();
    for _ in 0..n {
        if cancelled(cancellation) {
            return None;
        }
        let steer = 0.0_f64.clamp(current.steer - 0.3 * dt, current.steer + 0.3 * dt);
        let next = current.step(r.geometry, steer, 0.0, dt).ok()?;
        if !safe(current, next, r) {
            return None;
        }
        segment.push(next);
        current = next;
    }
    if done(current, r) {
        Some(segment)
    } else {
        None
    }
}
fn segment_cost(
    start: TruckState,
    mut direction: Direction,
    states: &[TruckState],
    geometry: VehicleGeometry,
) -> f64 {
    let mut cost = 0.0;
    let mut previous = start;
    for state in states {
        let next_direction = if state.speed < 0.0 {
            Direction::Reverse
        } else {
            Direction::Forward
        };
        let distance = previous.pose.distance(state.pose);
        cost += distance
            * (if next_direction == Direction::Reverse {
                1.2
            } else {
                1.0
            } + 0.2 * (state.steer / geometry.max_steer).abs()
                + 0.1 * state.articulation().abs());
        if next_direction != direction {
            cost += 2.0;
        }
        direction = next_direction;
        previous = *state;
    }
    cost
}
fn reconstruct(
    nodes: &[Node],
    index: usize,
    expansions: usize,
    search: ParkingSearchStatistics,
) -> ParkingPlan {
    let cost = nodes[index].cost;
    let mut chain = Vec::new();
    let mut i = Some(index);
    while let Some(index) = i {
        chain.push(index);
        i = nodes[index].parent;
    }
    chain.reverse();
    let mut points = Vec::new();
    for i in chain {
        for state in &nodes[i].segment {
            points.push(ParkingPoint {
                state: *state,
                direction: if state.speed < 0.0 {
                    Direction::Reverse
                } else if state.speed > 0.0 {
                    Direction::Forward
                } else {
                    nodes[i].direction
                },
            });
        }
    }
    ParkingPlan {
        points,
        expansions,
        cost,
        search,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancellation_interrupts_an_active_search_without_waiting_for_expansion_budget() {
        use std::sync::{Arc, mpsc};
        let token = Arc::new(AtomicBool::new(false));
        let worker_token = token.clone();
        let (tx, rx) = mpsc::channel();
        let (ready_tx, ready_rx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let polygon = [
                (-200.0, -200.0),
                (200.0, -200.0),
                (200.0, 200.0),
                (-200.0, 200.0),
            ];
            let obstacles = [-180.0, -90.0, 0.0, 90.0, 180.0].map(|y| OrientedBox {
                pose: Pose2 {
                    x: 50.0,
                    y,
                    yaw: 0.0,
                },
                length: 2.0,
                width: 92.0,
            });
            ready_tx.send(()).unwrap();
            let result = plan_cancellable(
                ParkingRequest {
                    start: TruckState::default(),
                    target: Pose2 {
                        x: 100.0,
                        ..Default::default()
                    },
                    geometry: VehicleGeometry::default(),
                    obstacles: &obstacles,
                    drivable: &polygon,
                    max_expansions: 1_000_000,
                    position_tolerance: 0.25,
                    angle_tolerance: 0.025,
                },
                &worker_token,
            );
            tx.send(result).unwrap();
        });
        ready_rx.recv().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        token.store(true, Ordering::Release);
        let error = rx
            .recv_timeout(std::time::Duration::from_secs(2))
            .expect("cancel must not wait for a long Hybrid A* search")
            .unwrap_err();
        assert!(
            error.to_string().contains("parking search cancelled"),
            "{error}"
        );
        worker.join().unwrap();
    }
    #[test]
    fn false_cancellation_token_preserves_the_actual_planner_output() {
        let polygon = [(-40.0, -15.0), (40.0, -15.0), (40.0, 15.0), (-40.0, 15.0)];
        let request = ParkingRequest {
            start: TruckState::default(),
            target: Pose2 {
                x: -18.0,
                ..Default::default()
            },
            geometry: VehicleGeometry::default(),
            obstacles: &[],
            drivable: &polygon,
            max_expansions: 1000,
            position_tolerance: 0.25,
            angle_tolerance: 0.025,
        };
        let plain = plan(request.clone()).unwrap();
        let token = AtomicBool::new(false);
        let cancellable = plan_cancellable(request, &token).unwrap();
        assert_eq!(plain.expansions, cancellable.expansions);
        assert_eq!(plain.cost, cancellable.cost);
        assert_eq!(plain.points.len(), cancellable.points.len());
        assert!(
            plain
                .points
                .iter()
                .zip(&cancellable.points)
                .all(|(a, b)| a.state == b.state && a.direction == b.direction)
        );
        token.store(true, Ordering::Release);
    }
    #[test]
    fn curved_connection_rejects_occupied_terminal_even_after_endpoint_correction() {
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_length: 0.0,
            trailer_wheelbase: 0.0,
            ..Default::default()
        };
        let polygon = [(-30.0, -30.0), (30.0, -30.0), (30.0, 30.0), (-30.0, 30.0)];
        let target = Pose2 {
            x: 10.0,
            y: 10.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let obstacle = OrientedBox {
            pose: target,
            length: 1.0,
            width: 1.0,
        };
        let request = ParkingRequest {
            start: TruckState::default(),
            target,
            geometry,
            obstacles: &[obstacle],
            drivable: &polygon,
            max_expansions: 100,
            position_tolerance: 0.25,
            angle_tolerance: 0.025,
        };
        let mut attempts = 0;
        assert!(connection::connect(request.start, &request, &mut attempts, true, None).is_none());
        assert!(attempts > 0);
    }
    #[test]
    fn feedback_connector_obeys_the_same_steering_rate_as_search_and_rs() {
        let geometry = VehicleGeometry::default();
        let polygon = [(-50.0, -30.0), (50.0, -30.0), (50.0, 30.0), (-50.0, 30.0)];
        let request = ParkingRequest {
            start: TruckState::default(),
            target: Pose2 {
                x: -18.0,
                y: 0.5,
                yaw: 0.0,
            },
            geometry,
            obstacles: &[],
            drivable: &polygon,
            max_expansions: 100,
            position_tolerance: 0.5,
            angle_tolerance: 0.05,
        };
        let states = feedback_connection(request.start, &request, None).unwrap();
        let mut previous = request.start;
        for state in states {
            assert!((state.steer - previous.steer).abs() <= 0.03000001);
            assert!(safe(previous, state, &request));
            previous = state;
        }
        assert!(done(previous, &request));
    }
    #[test]
    fn curved_reeds_shepp_connection_is_executed_with_bounded_steering_rate() {
        let geometry = VehicleGeometry {
            has_trailer: false,
            trailer_length: 0.0,
            trailer_wheelbase: 0.0,
            ..Default::default()
        };
        let polygon = [(-30.0, -30.0), (30.0, -30.0), (30.0, 30.0), (-30.0, 30.0)];
        let target = Pose2 {
            x: 10.0,
            y: 10.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let request = ParkingRequest {
            start: TruckState::default(),
            target,
            geometry,
            obstacles: &[],
            drivable: &polygon,
            max_expansions: 5000,
            position_tolerance: 0.25,
            angle_tolerance: 0.025,
        };
        let result = plan(request.clone()).unwrap();
        assert_eq!(
            result.search.accepted_connection, "reeds_shepp_articulated",
            "{:?}",
            result.search
        );
        assert!(result.search.analytic_curved && result.search.reeds_shepp_candidates_checked > 0);
        assert!(done(result.points.last().unwrap().state, &request));
        for pair in result.points.windows(2) {
            let travel = pair[0].state.pose.distance(pair[1].state.pose);
            assert!((pair[1].state.steer - pair[0].state.steer).abs() <= 0.3 * travel + 0.0001);
            assert!(safe(pair[0].state, pair[1].state, &request));
            assert_eq!(
                pair[1].direction,
                if pair[1].state.speed < 0.0 {
                    Direction::Reverse
                } else {
                    Direction::Forward
                }
            );
        }
    }
    #[test]
    fn straight_reverse_reaches_trailer_target() {
        let g = VehicleGeometry::default();
        let start = TruckState::default();
        let axle = start.trailer_axle(g);
        let polygon = [(-40.0, -15.0), (40.0, -15.0), (40.0, 15.0), (-40.0, 15.0)];
        let goal = Pose2 {
            x: axle.x - 8.0,
            ..axle
        };
        let p = plan(ParkingRequest {
            start,
            target: goal,
            geometry: g,
            obstacles: &[],
            drivable: &polygon,
            max_expansions: 1000,
            position_tolerance: 0.5,
            angle_tolerance: 3.0_f64.to_radians(),
        })
        .unwrap();
        assert!(
            p.points
                .last()
                .unwrap()
                .state
                .trailer_axle(g)
                .distance(goal)
                < 0.5
        );
        assert!(p.points.iter().any(|p| p.direction == Direction::Reverse));
    }
    #[test]
    fn initial_overlap_is_rejected() {
        let g = VehicleGeometry::default();
        let obstacle = OrientedBox {
            pose: Pose2::default(),
            length: 2.0,
            width: 2.0,
        };
        let polygon = [(-40.0, -15.0), (40.0, -15.0), (40.0, 15.0), (-40.0, 15.0)];
        assert!(
            plan(ParkingRequest {
                start: TruckState::default(),
                target: Pose2 {
                    x: -20.0,
                    ..Default::default()
                },
                geometry: g,
                obstacles: &[obstacle],
                drivable: &polygon,
                max_expansions: 100,
                position_tolerance: 0.5,
                angle_tolerance: 0.05
            })
            .is_err()
        );
    }
}
