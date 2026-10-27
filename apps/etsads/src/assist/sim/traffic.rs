//! AI traffic for the simulation: cars and trucks spawned on the map's lanes
//! around the truck (some on the route ahead), driving along their lanes
//! with an intelligent-driver model, choosing successors at random and
//! braking for the vehicle ahead, the player's truck included, as the
//! game's AI does. They stop at red lights and give way to the truck where
//! the map makes their lane yield; among themselves only following and
//! merging are modelled. Collisions with the truck are counted.
use etsads_map::importer::ExportLane;
use etsads_map::traffic::{RightOfWay, SemaphoreKind, TrafficContext};
use etsads_msgs::{Agent, Point3, Pose2, frames::wrap_angle};
use std::collections::HashMap;

/// Small deterministic generator (xorshift64*), so a run can be repeated.
#[derive(Clone)]
pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    pub fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next_f64()
    }
}
struct Lane {
    points: Vec<Point3>,
    stations: Vec<f64>,
    successors: Vec<String>,
}
impl Lane {
    fn length(&self) -> f64 {
        *self.stations.last().unwrap_or(&0.0)
    }
    /// Point and heading at station `s`.
    fn at(&self, s: f64) -> (Point3, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self
            .stations
            .partition_point(|v| *v <= s)
            .clamp(1, self.points.len() - 1);
        let (a, b) = (self.points[i - 1], self.points[i]);
        let span = (self.stations[i] - self.stations[i - 1]).max(1e-9);
        let t = (s - self.stations[i - 1]) / span;
        (
            Point3 {
                x: a.x + t * (b.x - a.x),
                y: a.y + t * (b.y - a.y),
                z: a.z + t * (b.z - a.z),
            },
            (b.y - a.y).atan2(b.x - a.x),
        )
    }
    /// Curvature magnitude around station `s` (1/m) over ±5 m.
    fn curvature(&self, s: f64) -> f64 {
        let (_, h0) = self.at(s - 5.0);
        let (_, h1) = self.at(s + 5.0);
        wrap_angle(h1 - h0).abs() / 10.0
    }
}
/// What a lane makes the AI do before entering it: wait for its signal
/// group (phase intervals and delay), stop at `stop_s` on it, and give way
/// to traffic on other lanes at conflict points.
#[derive(Default)]
struct Rule {
    signal: Option<([f64; 4], f64)>,
    stop_s: f64,
    yields_to: Vec<(String, Point3)>,
}
pub struct Car {
    pub id: u64,
    lane: String,
    /// Station of the car's centre on its lane.
    s: f64,
    pub speed: f64,
    acceleration: f64,
    desired: f64,
    pub length: f64,
    pub width: f64,
    /// Lanes chosen ahead (the first is the one after `lane`).
    next: Vec<String>,
    /// How long it has stood waiting for the stopped truck (s).
    held_s: f64,
    /// A semi-trailer behind the tractor (its length, 0 for none), and the
    /// lanes before the current one (the latest last), where the trailer
    /// may still be: junction lanes can be shorter than it.
    trailer_m: f64,
    previous: Vec<String>,
    /// When it appeared (s).
    born_s: f64,
    /// On roads it drifts this far (m) toward the centre line and back over
    /// `WANDER_WAVELENGTH_M` (0: keeps to the lane centre).
    wander_m: f64,
}
impl Car {
    /// From the centre back to the rear end of the combination.
    fn rear_extent(&self) -> f64 {
        0.5 * self.length
            + if self.trailer_m > 0.0 {
                self.trailer_m - KINGPIN_AHEAD_OF_REAR_M - TRAILER_FRONT_OVERHANG_M
            } else {
                0.0
            }
    }
}
/// The truck as the traffic sees it: rear-axle pose and body size.
#[derive(Clone, Copy)]
pub struct Body {
    pub rear: Pose2,
    pub z: f64,
    pub speed: f64,
    /// Rear axle to front bumper and to rear end (m), and width.
    pub front_m: f64,
    pub back_m: f64,
    pub width: f64,
    /// A semi-trailer's box: centre x, y, yaw, half length, half width.
    pub trailer: Option<[f64; 5]>,
}
impl Body {
    fn centre(&self) -> (f64, f64) {
        let mid = 0.5 * (self.front_m - self.back_m);
        (
            self.rear.x + mid * self.rear.yaw.cos(),
            self.rear.y + mid * self.rear.yaw.sin(),
        )
    }
    fn length(&self) -> f64 {
        self.front_m + self.back_m
    }
}
pub struct Traffic {
    rng: Rng,
    lanes: HashMap<String, Lane>,
    /// Lane ids by 50 m cell, to spawn near the truck.
    cells: HashMap<(i64, i64), Vec<String>>,
    loaded_at: Option<Point3>,
    pub cars: Vec<Car>,
    next_id: u64,
    /// Cars kept within `radius_m` of the truck.
    pub count: usize,
    pub radius_m: f64,
    spawn_timer: f64,
    rules: HashMap<String, Rule>,
    /// Simulated time (s), for the signal phases.
    time_s: f64,
    /// See `wandering`.
    wander_m: f64,
}
const CELL_M: f64 = 50.0;
/// Comfortable acceleration, braking, gap and headway of the AI (IDM).
const IDM_A: f64 = 1.5;
const IDM_B: f64 = 2.5;
const IDM_GAP_M: f64 = 2.5;
const IDM_HEADWAY_S: f64 = 1.4;
const AI_LATERAL_ACCELERATION: f64 = 2.0;
/// Semi-trailer geometry: the kingpin ahead of the tractor's rear, the
/// trailer's front ahead of the kingpin, and its axle behind the kingpin as
/// a share of its length.
const KINGPIN_AHEAD_OF_REAR_M: f64 = 1.0;
/// Lanes remembered behind a car, for its trailer.
const PREVIOUS_LANES: usize = 12;
const TRAILER_FRONT_OVERHANG_M: f64 = 1.2;
const TRAILER_AXLE_SHARE: f64 = 0.75;
/// How far ahead the AI looks for lights and give-way lines (m), and how
/// soon the truck must reach a conflict point to be given way (s).
const RULE_LOOKAHEAD_M: f64 = 90.0;
/// One drift toward the centre line and back per this much road (m).
const WANDER_WAVELENGTH_M: f64 = 200.0;
const YIELD_TIME_S: f64 = 6.0;
/// How far ahead (s) the AI looks at where the truck is going.
const TRUCK_PREVIEW_S: f64 = 1.0;
impl Traffic {
    pub fn new(seed: u64, count: usize) -> Self {
        Self {
            rng: Rng::new(seed),
            lanes: HashMap::new(),
            cells: HashMap::new(),
            loaded_at: None,
            cars: Vec::new(),
            next_id: 1,
            count,
            radius_m: 250.0,
            spawn_timer: 0.0,
            rules: HashMap::new(),
            time_s: 0.0,
            wander_m: 0.0,
        }
    }
    /// Cars drift up to `metres` toward the centre line on roads, as some
    /// do in the game (live: 1.0–1.7 m off their lane 100 m ahead).
    pub fn wandering(mut self, metres: f64) -> Self {
        self.wander_m = metres.max(0.0);
        self
    }
    /// Whether the lanes around `center` need loading (first time, or the
    /// truck moved 600 m on).
    pub fn needs_lanes(&self, center: Point3) -> bool {
        self.loaded_at
            .is_none_or(|at| at.distance_xy(center) > 600.0)
    }
    pub fn load_lanes(
        &mut self,
        center: Point3,
        lanes: Vec<ExportLane>,
        controls: &TrafficContext,
    ) {
        self.lanes.clear();
        self.cells.clear();
        self.rules.clear();
        for lane in lanes {
            if lane.truck_access == Some(false) || lane.center.len() < 2 {
                continue;
            }
            let mut stations = vec![0.0];
            for w in lane.center.windows(2) {
                stations.push(stations.last().unwrap() + w[0].distance_xy(w[1]));
            }
            if *stations.last().unwrap() < 1.0 {
                continue;
            }
            for p in lane.center.iter().step_by(4) {
                let key = ((p.x / CELL_M).floor() as i64, (p.y / CELL_M).floor() as i64);
                let ids = self.cells.entry(key).or_default();
                if ids.last() != Some(&lane.id) {
                    ids.push(lane.id.clone());
                }
            }
            self.lanes.insert(
                lane.id.clone(),
                Lane {
                    points: lane.center,
                    stations,
                    successors: lane.successors,
                },
            );
        }
        self.load_rules(controls);
        // Cars on lanes no longer loaded are dropped.
        let lanes = &self.lanes;
        self.cars.retain(|c| lanes.contains_key(&c.lane));
        self.loaded_at = Some(center);
    }
    /// A lane's lights come from the same records the assist reads: its
    /// stop lines' signal groups, its lane control's group and the signal
    /// links (the links alone missed most junction lanes, and AI ran red
    /// lights into the truck).
    fn load_rules(&mut self, controls: &TrafficContext) {
        let phases = |group: &str| {
            controls
                .signals
                .iter()
                .find(|s| {
                    s.id == group
                        && matches!(
                            s.kind,
                            SemaphoreKind::UseProfile
                                | SemaphoreKind::TrafficLight
                                | SemaphoreKind::TrafficLightMinor
                                | SemaphoreKind::TrafficLightMajor
                        )
                        && s.interval_0 + s.interval_1 + s.interval_2 + s.interval_3 > 0.0
                })
                .map(|s| {
                    (
                        [s.interval_0, s.interval_1, s.interval_2, s.interval_3],
                        s.cycle_delay_s,
                    )
                })
        };
        let groups = controls
            .signal_lanes
            .iter()
            .map(|link| (&link.lane_id, Some(&link.signal_id)))
            .chain(
                controls
                    .lane_controls
                    .iter()
                    .map(|c| (&c.lane_id, c.signal_group_id.as_ref())),
            )
            .chain(
                controls
                    .stop_lines
                    .iter()
                    .map(|s| (&s.lane_id, s.signal_id.as_ref())),
            );
        for (lane, group) in groups {
            if let Some(group) = group
                && self.lanes.contains_key(lane)
                && let Some(signal) = phases(group)
            {
                self.rules
                    .entry(lane.clone())
                    .or_default()
                    .signal
                    .get_or_insert(signal);
            }
        }
        for stop in &controls.stop_lines {
            let Some(lane) = self.lanes.get(&stop.lane_id) else {
                continue;
            };
            let at = stop.position();
            let nearest = lane
                .points
                .iter()
                .zip(&lane.stations)
                .min_by(|a, b| a.0.distance_xy(at).total_cmp(&b.0.distance_xy(at)));
            if let Some((p, s)) = nearest
                && p.distance_xy(at) < 6.0
            {
                self.rules.entry(stop.lane_id.clone()).or_default().stop_s = *s;
            }
        }
        for pair in &controls.conflicts {
            for (mine, other) in [(&pair.lane_a, &pair.lane_b), (&pair.lane_b, &pair.lane_a)] {
                if self.lanes.contains_key(mine)
                    && pair.priority_for(mine) == Some(RightOfWay::Yield)
                {
                    self.rules
                        .entry(mine.clone())
                        .or_default()
                        .yields_to
                        .push((other.clone(), pair.position()));
                }
            }
        }
    }
    /// Distance from a car's front to where a red light or a truck it gives
    /// way to makes it stop, and whether the truck is the reason. A stop
    /// point the car can no longer stop at is driven through.
    fn rule_stop(&self, car: &Car, truck: &Body, truck_lanes: &[String]) -> Option<(f64, bool)> {
        let v = car.speed;
        let mut start = -car.s;
        let mut before = car.previous.last();
        for id in std::iter::once(&car.lane).chain(car.next.iter()) {
            let Some(lane) = self.lanes.get(id) else {
                break;
            };
            // Lights are obeyed entering a junction; its inner lanes carry
            // the same groups, and cars stopping on them blocked the
            // junction for the truck (simulation).
            let entering =
                before.is_none_or(|b| junction(b).is_none() || junction(b) != junction(id));
            before = Some(id);
            if start > RULE_LOOKAHEAD_M {
                break;
            }
            let gap = self
                .rules
                .get(id)
                .map(|rule| (rule, start + rule.stop_s - 0.5 * car.length))
                .filter(|(_, gap)| *gap > -0.5);
            if let Some((rule, gap)) = gap {
                if let Some((intervals, delay)) = rule.signal.filter(|_| entering) {
                    // Amber: stop where comfortable; red: unless too late.
                    let braking = match super::signal_state(intervals, delay, self.time_s) {
                        8 => None,
                        1 => Some(3.0),
                        _ => Some(7.0),
                    };
                    if braking.is_some_and(|b| gap > v * v / (2.0 * b)) {
                        return Some((gap.max(0.0), false));
                    }
                }
                let (tx, ty) = truck.centre();
                let (c, s) = (truck.rear.yaw.cos(), truck.rear.yaw.sin());
                for (other, point) in &rule.yields_to {
                    if !truck_lanes.contains(other) {
                        continue;
                    }
                    let (dx, dy) = (point.x - tx, point.y - ty);
                    let along = dx * c + dy * s;
                    // The truck in the conflict area, or driving to it soon. A
                    // truck standing back from it is itself giving way: cars
                    // waiting for it there deadlocked the junction.
                    let trailer_in = truck
                        .trailer
                        .is_some_and(|t| (point.x - t[0]).hypot(point.y - t[1]) < t[3] + 2.0);
                    let coming = trailer_in
                        || along > -(0.5 * truck.length() + 2.0)
                            && (dx.hypot(dy) < 0.5 * truck.length() + 2.0
                                || (truck.speed > 1.0 && along / truck.speed < YIELD_TIME_S));
                    if coming && gap > v * v / (2.0 * 6.0) {
                        return Some((gap.max(0.0), true));
                    }
                }
            }
            start += lane.length();
        }
        None
    }
    fn spawn_one(&mut self, truck: &Body, route_lanes: &[String], limit: &dyn Fn(Pose2) -> f64) {
        // Some on the route ahead (leaders), the rest on any lane nearby.
        let on_route = !route_lanes.is_empty() && self.rng.next_f64() < 0.35;
        let candidates: Vec<String> = if on_route {
            route_lanes.to_vec()
        } else {
            let (cx, cy) = (
                (truck.rear.x / CELL_M).floor() as i64,
                (truck.rear.y / CELL_M).floor() as i64,
            );
            let reach = (self.radius_m / CELL_M).ceil() as i64;
            let mut ids = Vec::new();
            for dx in -reach..=reach {
                for dy in -reach..=reach {
                    if let Some(cell) = self.cells.get(&(cx + dx, cy + dy)) {
                        ids.extend(cell.iter().cloned());
                    }
                }
            }
            ids
        };
        if candidates.is_empty() {
            return;
        }
        for _ in 0..8 {
            let id = &candidates
                [(self.rng.next_f64() * candidates.len() as f64) as usize % candidates.len()];
            let Some(lane) = self.lanes.get(id) else {
                continue;
            };
            let s = self.rng.range(0.0, lane.length());
            let (p, heading) = lane.at(s);
            let d = (p.x - truck.rear.x).hypot(p.y - truck.rear.y);
            if !(60.0..self.radius_m).contains(&d) || (p.z - truck.z).abs() > 6.0 {
                continue;
            }
            let clear = self
                .cars
                .iter()
                .all(|c| c.lane != *id || (c.s - s).abs() > 25.0);
            if !clear {
                continue;
            }
            let heavy = self.rng.next_f64() < 0.25;
            // The game's AI keeps near the limit.
            let desired = limit(Pose2 {
                x: p.x,
                y: p.y,
                yaw: heading,
            }) * self.rng.range(0.85, 1.05);
            self.cars.push(Car {
                id: self.next_id,
                lane: id.clone(),
                s,
                speed: desired * self.rng.range(0.5, 1.0),
                acceleration: 0.0,
                desired,
                length: if heavy { 6.0 } else { 4.5 },
                width: if heavy { 2.5 } else { 1.8 },
                next: Vec::new(),
                held_s: 0.0,
                trailer_m: if heavy { 10.5 } else { 0.0 },
                previous: Vec::new(),
                born_s: self.time_s,
                // Drawn only when wandering, so other runs keep their traffic.
                wander_m: if self.wander_m > 0.0 {
                    self.rng.range(0.0, self.wander_m)
                } else {
                    0.0
                },
            });
            self.next_id += 1;
            return;
        }
    }
    /// The lanes ahead of a car, extending its choice of successors.
    fn plan(&mut self, index: usize, ahead_m: f64) {
        let mut length = {
            let car = &self.cars[index];
            self.lanes[&car.lane].length() - car.s
        };
        let mut last = self.cars[index]
            .next
            .last()
            .cloned()
            .unwrap_or_else(|| self.cars[index].lane.clone());
        for id in &self.cars[index].next {
            length += self.lanes.get(id).map_or(0.0, Lane::length);
        }
        while length < ahead_m {
            let car = &self.cars[index];
            let all: Vec<&String> = self.lanes[&last]
                .successors
                .iter()
                .filter(|s| self.lanes.contains_key(*s))
                .collect();
            // Lanes not driven lately first: picked at random, cars
            // circled roundabouts for minutes and never let the truck in.
            let fresh: Vec<&String> = all
                .iter()
                .copied()
                .filter(|s| !car.previous.contains(s) && !car.next.contains(s) && **s != car.lane)
                .collect();
            let options = if fresh.is_empty() { all } else { fresh };
            if options.is_empty() {
                break;
            }
            let pick = options
                [(self.rng.next_f64() * options.len() as f64) as usize % options.len()]
            .clone();
            length += self.lanes[&pick].length();
            self.cars[index].next.push(pick.clone());
            last = pick;
        }
    }
    /// Points along a car's path ahead, every 2 m up to `ahead_m`, with
    /// the distance from its centre.
    fn path(&self, car: &Car, ahead_m: f64) -> Vec<(f64, Point3)> {
        let mut out = Vec::new();
        let mut lane = &self.lanes[&car.lane];
        let mut s = car.s;
        let mut d = 0.0;
        let mut next = car.next.iter();
        while d <= ahead_m {
            if s > lane.length() {
                let Some(id) = next
                    .next()
                    .and_then(|id| self.lanes.get(id).map(|l| (id, l)))
                else {
                    // No lane linked on (the export misses some links into
                    // junctions): look straight on, where the car will go.
                    let (end, heading) = lane.at(lane.length());
                    let beyond = s - lane.length();
                    while d <= ahead_m.min(d + 30.0 - beyond) {
                        let k = s - lane.length();
                        out.push((
                            d,
                            Point3 {
                                x: end.x + k * heading.cos(),
                                y: end.y + k * heading.sin(),
                                z: end.z,
                            },
                        ));
                        s += 2.0;
                        d += 2.0;
                    }
                    break;
                };
                s -= lane.length();
                lane = id.1;
            }
            out.push((d, lane.at(s).0));
            s += 2.0;
            d += 2.0;
        }
        out
    }
    /// One step of `dt` seconds. Returns the ids of cars touching the truck.
    /// `limit` is the speed limit (m/s) at a pose.
    pub fn step(
        &mut self,
        dt: f64,
        truck: &Body,
        route_lanes: &[String],
        limit: &dyn Fn(Pose2) -> f64,
    ) -> Vec<u64> {
        self.time_s += dt;
        // Spawn toward the target count, at most a car per 0.5 s.
        self.spawn_timer -= dt;
        if self.cars.len() < self.count && self.spawn_timer <= 0.0 && !self.lanes.is_empty() {
            self.spawn_one(truck, route_lanes, limit);
            self.spawn_timer = 0.5;
        }
        for i in 0..self.cars.len() {
            self.plan(i, 120.0);
        }
        let (tx, ty) = truck.centre();
        // Gap to the nearest vehicle ahead on each car's path (the truck's
        // body included) and its speed.
        let mut gaps = Vec::with_capacity(self.cars.len());
        for car in &self.cars {
            let path = self.path(car, 80.0);
            let heading = self.position(car).1;
            let mut gap = None::<(f64, f64)>;
            let mut by_truck = false;
            for other in &self.cars {
                if other.id == car.id {
                    continue;
                }
                let (op, oh) = self.position(other);
                let (cp, _) = self.position(car);
                let reach = 0.5 * car.length + other.rear_extent();
                // Side by side after two lanes merged: the later one yields.
                let beside = (op.x - cp.x).hypot(op.y - cp.y) < reach
                    && wrap_angle(oh - heading).abs() < 0.6
                    && other.id < car.id;
                let g = if beside {
                    Some(0.0)
                } else if other.lane == car.lane && other.s > car.s {
                    Some(other.s - car.s - reach)
                } else {
                    path.iter()
                        .find(|(d, p)| *d > 0.0 && (p.x - op.x).hypot(p.y - op.y) < 2.0)
                        .map(|(d, _)| d - reach)
                };
                if let Some(g) = g
                    && gap.is_none_or(|(best, _)| g < best)
                {
                    gap = Some((g, other.speed));
                }
            }
            // The truck: any part of its body, now or a second on, near the
            // path. Seen only where it stood, a truck creeping round a
            // junction turn was noticed too late to stop for (simulation).
            let half = 0.5 * truck.length();
            let ahead = truck.speed.max(0.0) * TRUCK_PREVIEW_S;
            let (c, s) = (truck.rear.yaw.cos(), truck.rear.yaw.sin());
            // Its own body over the next second too: turning away from a
            // truck whose cab swung into its lane, a car whose path cleared
            // the truck clipped it with a corner (simulation).
            let truck_box = [tx, ty, truck.rear.yaw, half + 0.3, 0.5 * truck.width + 0.3];
            let reach_m = car.speed * TRUCK_PREVIEW_S + 1.0;
            let clipped = path.windows(2).find(|w| {
                w[0].0 <= reach_m && {
                    let (p, q) = (w[0].1, w[1].1);
                    let heading = (q.y - p.y).atan2(q.x - p.x);
                    let car_box = [p.x, p.y, heading, 0.5 * car.length, 0.5 * car.width];
                    overlap(car_box, truck_box)
                        || truck.trailer.is_some_and(|t| {
                            overlap(car_box, [t[0], t[1], t[2], t[3] + 0.3, t[4] + 0.3])
                        })
                }
            });
            if let Some((d, _)) = clipped.map(|w| &w[0]).or_else(|| {
                path.iter().find(|(_, p)| {
                    let (dx, dy) = (p.x - tx, p.y - ty);
                    let along = dx * c + dy * s;
                    let across = -dx * s + dy * c;
                    // A trailer cuts inside bends: the combination sweeps wider.
                    let reach = 0.5 * car.width + if car.trailer_m > 0.0 { 2.0 } else { 1.0 };
                    let near_trailer = truck.trailer.is_some_and(|t| {
                        let (dx, dy) = (p.x - t[0], p.y - t[1]);
                        let (along, across) = (
                            dx * t[2].cos() + dy * t[2].sin(),
                            -dx * t[2].sin() + dy * t[2].cos(),
                        );
                        along.abs() < t[3] + reach && across.abs() < t[4] + reach
                    });
                    near_trailer
                        || along > -(half + reach)
                            && along < half + ahead + reach
                            && across.abs() < 0.5 * truck.width + reach
                })
            }) {
                let g = (d - 0.5 * car.length).max(0.0);
                if gap.is_none_or(|(best, _)| g < best) {
                    // Moving the car's way, the truck is followed; across
                    // its path it is waited for.
                    let along = (truck.speed * wrap_angle(truck.rear.yaw - heading).cos()).max(0.0);
                    gap = Some((g, along));
                    by_truck = true;
                }
            }
            // Curves ahead set a speed the car slows for.
            let lane = &self.lanes[&car.lane];
            let curve = lane.curvature(car.s + 10.0).max(lane.curvature(car.s));
            let v_curve = (AI_LATERAL_ACCELERATION / curve.max(1e-4)).sqrt();
            // A red light or a truck it gives way to: a standing obstacle.
            let mut by_rule = false;
            if let Some((g, truck_rule)) = self.rule_stop(car, truck, route_lanes)
                && gap.is_none_or(|(best, _)| g < best)
            {
                gap = Some((g, 0.0));
                by_truck = truck_rule;
                by_rule = !truck_rule;
            }
            gaps.push((gap, car.desired.min(v_curve), by_truck, by_rule));
        }
        for (car, (gap, desired, by_truck, by_rule)) in self.cars.iter_mut().zip(gaps) {
            // Standing still behind something: the stopped truck counts
            // more (the game removes AI that stays stuck); waiting at a
            // red light does not count.
            car.held_s = if by_rule {
                car.held_s
            } else if gap.is_some() && car.speed < 0.5 {
                car.held_s
                    + if by_truck && truck.speed < 0.5 {
                        2.5 * dt
                    } else {
                        dt
                    }
            } else {
                0.0
            };
            let v = car.speed;
            let free = 1.0 - (v / desired.max(0.5)).powi(4);
            let interaction = gap.map_or(0.0, |(g, lead)| {
                let s_star =
                    IDM_GAP_M + v * IDM_HEADWAY_S + v * (v - lead) / (2.0 * (IDM_A * IDM_B).sqrt());
                (s_star.max(0.0) / g.max(0.1)).powi(2)
            });
            let a = (IDM_A * (free - interaction)).clamp(-8.0, IDM_A);
            car.acceleration = a;
            car.speed = (v + a * dt).max(0.0);
            car.s += car.speed * dt;
        }
        // A car kept waiting by the stopped truck gives up (the game removes
        // stuck AI), so the two never wait for each other for ever.
        self.cars.retain(|c| c.held_s < 20.0);
        // Lane ends: on to the chosen successor, or gone.
        let lanes = &self.lanes;
        self.cars.retain_mut(|car| {
            while car.s > lanes[&car.lane].length() {
                if car.next.is_empty() {
                    return false;
                }
                car.s -= lanes[&car.lane].length();
                let left = std::mem::replace(&mut car.lane, car.next.remove(0));
                car.previous.push(left);
                if car.previous.len() > PREVIOUS_LANES {
                    car.previous.remove(0);
                }
            }
            true
        });
        // Too far away: gone.
        let radius = self.radius_m + 80.0;
        let far: Vec<u64> = self
            .cars
            .iter()
            .filter(|c| {
                let (p, _) = self.position(c);
                (p.x - truck.rear.x).hypot(p.y - truck.rear.y) > radius
            })
            .map(|c| c.id)
            .collect();
        self.cars.retain(|c| !far.contains(&c.id));
        self.collisions(truck)
    }
    /// A car's lane, station, lanes before and ahead, age, the nearest
    /// path point to the truck with its distance, and which of its bodies
    /// touch the truck (diagnostics).
    pub fn describe(&self, id: u64, truck: &Body) -> String {
        let Some(car) = self.cars.iter().find(|c| c.id == id) else {
            return String::new();
        };
        let (x, y) = truck.centre();
        let near = self
            .path(car, 80.0)
            .into_iter()
            .map(|(d, p)| (d, (p.x - x).hypot(p.y - y)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        let truck_box = [
            x,
            y,
            truck.rear.yaw,
            0.5 * truck.length(),
            0.5 * truck.width,
        ];
        let touching: Vec<&str> = self
            .boxes(car)
            .iter()
            .zip(["tractor", "trailer"])
            .filter(|((b, _), _)| overlap(truck_box, *b))
            .map(|(_, name)| name)
            .collect();
        // The rules of its lanes: signal state, stop station, give-ways.
        let rules: Vec<String> = car
            .previous
            .iter()
            .chain(std::iter::once(&car.lane))
            .chain(car.next.iter())
            .map(|id| match self.rules.get(id) {
                Some(rule) => format!(
                    "{}:{}@{:.0}/{}",
                    id.rsplit(':').next().unwrap_or(""),
                    rule.signal
                        .map_or(0, |(i, d)| super::signal_state(i, d, self.time_s)),
                    rule.stop_s,
                    rule.yields_to.len()
                ),
                None => "-".into(),
            })
            .collect();
        format!(
            "rules {rules:?} lane {} s {:.1} previous {:?} next {:?} age {:.1} s nearest {:?} held {:.1} touching {:?}",
            car.lane,
            car.s,
            car.previous,
            car.next,
            self.time_s - car.born_s,
            near,
            car.held_s,
            touching
        )
    }
    /// A car's lane and the lanes it will take on (diagnostics).
    pub fn lanes_of(&self, id: u64) -> Vec<String> {
        self.cars
            .iter()
            .find(|c| c.id == id)
            .map(|c| std::iter::once(&c.lane).chain(&c.next).cloned().collect())
            .unwrap_or_default()
    }
    /// Removes the cars within `radius_m` of `pose` (the driver got past them).
    pub fn clear_near(&mut self, pose: Pose2, radius_m: f64) {
        let near: Vec<u64> = self
            .cars
            .iter()
            .filter(|c| {
                let (p, _) = self.position(c);
                (p.x - pose.x).hypot(p.y - pose.y) < radius_m
            })
            .map(|c| c.id)
            .collect();
        self.cars.retain(|c| !near.contains(&c.id));
    }
    fn position(&self, car: &Car) -> (Point3, f64) {
        let (p, heading) = self.lanes[&car.lane].at(car.s);
        if car.wander_m <= 0.0 || !car.lane.contains(":road:") {
            return (p, heading);
        }
        let phase = car.id as f64 * 1.7;
        let off = car.wander_m
            * (std::f64::consts::TAU * car.s / WANDER_WAVELENGTH_M + phase)
                .sin()
                .max(0.0);
        (
            Point3 {
                x: p.x - off * heading.sin(),
                y: p.y + off * heading.cos(),
                z: p.z,
            },
            heading,
        )
    }
    /// The trailer's centre and heading, following the tractor's path.
    /// The trailer's centre and heading. The kingpin and the trailer's
    /// axle both follow the tractor's path, so the body lies on the chord
    /// between them and cuts inside a bend, as a real trailer does; set on
    /// the path's tangent it swung outward into a truck waiting beside a
    /// junction turn (simulation).
    fn trailer(&self, car: &Car) -> Option<(Point3, f64)> {
        if car.trailer_m <= 0.0 {
            return None;
        }
        let kingpin = self.behind(car, 0.5 * car.length - KINGPIN_AHEAD_OF_REAR_M);
        let axle = self.behind(
            car,
            0.5 * car.length - KINGPIN_AHEAD_OF_REAR_M + TRAILER_AXLE_SHARE * car.trailer_m,
        );
        let heading = (kingpin.y - axle.y).atan2(kingpin.x - axle.x);
        let back = 0.5 * car.trailer_m - TRAILER_FRONT_OVERHANG_M;
        Some((
            Point3 {
                x: kingpin.x - back * heading.cos(),
                y: kingpin.y - back * heading.sin(),
                z: 0.5 * (kingpin.z + axle.z),
            },
            heading,
        ))
    }
    /// The point on a car's path `distance` behind its centre: on its lane,
    /// the lanes before, or straight back from where it joined.
    fn behind(&self, car: &Car, distance: f64) -> Point3 {
        let mut s = car.s - distance;
        let mut lane = &self.lanes[&car.lane];
        for id in car.previous.iter().rev() {
            if s >= 0.0 {
                break;
            }
            let Some(before) = self.lanes.get(id) else {
                break;
            };
            s += before.length();
            lane = before;
        }
        if s >= 0.0 {
            return lane.at(s).0;
        }
        let (start, heading) = lane.at(0.0);
        Point3 {
            x: start.x + s * heading.cos(),
            y: start.y + s * heading.sin(),
            z: start.z,
        }
    }
    /// Every box of a vehicle: [x, y, yaw, half length, half width].
    fn boxes(&self, car: &Car) -> Vec<([f64; 5], f64)> {
        let (p, h) = self.position(car);
        let mut out = vec![([p.x, p.y, h, 0.5 * car.length, 0.5 * car.width], p.z)];
        if let Some((t, th)) = self.trailer(car) {
            out.push(([t.x, t.y, th, 0.5 * car.trailer_m, 0.5 * car.width], t.z));
        }
        out
    }
    /// Cars whose body overlaps the truck's.
    fn collisions(&self, truck: &Body) -> Vec<u64> {
        let (tx, ty) = truck.centre();
        let truck_box = [
            tx,
            ty,
            truck.rear.yaw,
            0.5 * truck.length(),
            0.5 * truck.width,
        ];
        self.cars
            .iter()
            .filter(|c| {
                self.boxes(c).iter().any(|(b, z)| {
                    (z - truck.z).abs() < 3.0
                        && (overlap(truck_box, *b) || truck.trailer.is_some_and(|t| overlap(t, *b)))
                })
            })
            .map(|c| c.id)
            .collect()
    }
    /// The cars as the game's traffic list reports them.
    pub fn agents(&self) -> Vec<Agent> {
        self.cars
            .iter()
            .map(|c| {
                let (p, h) = self.position(c);
                Agent {
                    id: c.id,
                    pose: Pose2 {
                        x: p.x,
                        y: p.y,
                        yaw: h,
                    },
                    z: p.z,
                    speed: c.speed,
                    acceleration: c.acceleration,
                    length: c.length,
                    width: c.width,
                    lane_id: None,
                    trailers: self
                        .trailer(c)
                        .map(|(t, th)| etsads_msgs::OrientedBox {
                            pose: Pose2 {
                                x: t.x,
                                y: t.y,
                                yaw: th,
                            },
                            length: c.trailer_m,
                            width: c.width,
                        })
                        .into_iter()
                        .collect(),
                }
            })
            .collect()
    }
}
/// The junction (prefab) a lane belongs to, if any.
fn junction(lane: &str) -> Option<&str> {
    lane.find(":prefab:").map(|i| &lane[..i])
}
/// Separating-axis test of two boxes [x, y, yaw, half length, half width].
fn overlap(a: [f64; 5], b: [f64; 5]) -> bool {
    let axes = [
        a[2],
        a[2] + std::f64::consts::FRAC_PI_2,
        b[2],
        b[2] + std::f64::consts::FRAC_PI_2,
    ];
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    axes.iter().all(|&t| {
        let (c, s) = (t.cos(), t.sin());
        let r = |q: [f64; 5]| {
            q[3] * (c * q[2].cos() + s * q[2].sin()).abs()
                + q[4]
                    * (c * (q[2] + std::f64::consts::FRAC_PI_2).cos()
                        + s * (q[2] + std::f64::consts::FRAC_PI_2).sin())
                    .abs()
        };
        (dx * c + dy * s).abs() <= r(a) + r(b)
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    fn straight(id: &str, x0: f64, successors: &[&str]) -> ExportLane {
        serde_json::from_value(serde_json::json!({
            "id": id, "owner_uid": 1, "node_start": 1, "node_end": 2,
            "center": (0..=50).map(|i| serde_json::json!({"x": x0 + 4.0 * i as f64, "y": 0.0, "z": 0.0})).collect::<Vec<_>>(),
            "width": 3.5, "speed_limit": null, "geometry_status": "ok",
            "navigation_width": null, "centerline_validated": true,
            "physical_boundaries_verified": false, "truck_access": true,
            "successors": successors, "predecessors": [], "left": null, "right": null,
            "can_change_left": false, "can_change_right": false
        }))
        .unwrap()
    }
    #[test]
    fn a_car_brakes_for_the_truck_standing_in_its_lane() {
        let mut traffic = Traffic::new(7, 0);
        traffic.load_lanes(
            Point3::default(),
            vec![straight("a", 0.0, &["b"]), straight("b", 200.0, &[])],
            &TrafficContext::default(),
        );
        traffic.cars.push(Car {
            id: 1,
            lane: "a".into(),
            s: 20.0,
            speed: 15.0,
            acceleration: 0.0,
            desired: 15.0,
            length: 4.5,
            width: 1.8,
            next: Vec::new(),
            held_s: 0.0,
            trailer_m: 0.0,
            previous: Vec::new(),
            born_s: 0.0,
            wander_m: 0.0,
        });
        let truck = Body {
            trailer: None,
            rear: Pose2 {
                x: 150.0,
                y: 0.0,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 0.0,
            front_m: 5.0,
            back_m: 1.5,
            width: 2.5,
        };
        for _ in 0..(20.0 * 60.0) as usize {
            assert!(
                traffic.step(1.0 / 60.0, &truck, &[], &|_| 15.0).is_empty(),
                "collided"
            );
        }
        let car = &traffic.cars[0];
        assert!(
            car.speed < 0.1 && car.s > 100.0 && car.s < 147.0,
            "{} {}",
            car.s,
            car.speed
        );
    }
    #[test]
    fn a_car_stops_at_a_red_light_and_goes_on_green() {
        let mut traffic = Traffic::new(7, 0);
        traffic.load_lanes(
            Point3::default(),
            vec![straight("a", 0.0, &["b"]), straight("b", 200.0, &[])],
            &TrafficContext::default(),
        );
        // Red for the first 30 s, then green.
        traffic.rules.insert(
            "b".into(),
            Rule {
                signal: Some(([10.0, 3.0, 30.0, 0.0], 13.0)),
                ..Rule::default()
            },
        );
        traffic.cars.push(Car {
            id: 1,
            lane: "a".into(),
            s: 60.0,
            speed: 15.0,
            acceleration: 0.0,
            desired: 15.0,
            length: 4.5,
            width: 1.8,
            next: Vec::new(),
            held_s: 0.0,
            trailer_m: 0.0,
            previous: Vec::new(),
            born_s: 0.0,
            wander_m: 0.0,
        });
        let truck = Body {
            trailer: None,
            rear: Pose2 {
                x: 0.0,
                y: 50.0,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 0.0,
            front_m: 5.0,
            back_m: 1.5,
            width: 2.5,
        };
        for _ in 0..(25.0 * 60.0) as usize {
            traffic.step(1.0 / 60.0, &truck, &[], &|_| 15.0);
        }
        let car = &traffic.cars[0];
        assert!(
            car.lane == "a" && car.speed < 0.1 && car.s > 190.0 && car.s < 198.0,
            "{} {} {}",
            car.lane,
            car.s,
            car.speed
        );
        for _ in 0..(10.0 * 60.0) as usize {
            traffic.step(1.0 / 60.0, &truck, &[], &|_| 15.0);
        }
        assert!(traffic.cars[0].lane == "b", "drove on at green");
    }
    #[test]
    fn boxes_overlap_only_when_touching() {
        let a = [0.0, 0.0, 0.0, 3.0, 1.2];
        assert!(overlap(a, [5.0, 0.0, 0.3, 2.25, 0.9]));
        assert!(!overlap(a, [5.5, 0.0, 0.0, 2.25, 0.9]));
        assert!(
            overlap(a, [0.0, 3.0, 1.0, 2.25, 0.9]),
            "rotated, it reaches down to y = 0.6"
        );
        assert!(!overlap(a, [0.0, 4.0, 1.0, 2.25, 0.9]));
    }
}
