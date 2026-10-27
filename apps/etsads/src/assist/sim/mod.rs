//! Offline closed-loop driving: the assist's own route preparation, speed
//! planning, steering and pedal control run on the exported map against a
//! truck modelled on recorded drives (`plant`), with a GPS route captured
//! from the game. A driver model takes over where the assist hands back
//! (and after a safety stop) and rejoins the route at the next checkpoint,
//! so one run covers a whole haul. Simulation only: it cannot establish
//! acceptance in the game.
pub mod fit;
pub mod plant;
pub mod routes;
pub mod traffic;
use super::core::MAX_SPEED;
use super::core::{DRIFT_WARN_M, DriftHold, Feedback, Output, RelativeHold, SteeringMap};
use super::drift_speed;
use super::io::{SteeringServo, clutch};
use super::navigation::{Guidance, Navigator};
use anyhow::{Context, ensure};
use etsads_map::traffic::SignalObservation;
use etsads_msgs::{Point3, Pose2};
use plant::{Input, Profile, Truck};
use serde::Serialize;
use std::{
    collections::{HashMap, VecDeque},
    io::Write,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

pub struct Options {
    pub map: PathBuf,
    /// GPS route records captured from the game (`testdata/r17-route.txt`).
    pub route: PathBuf,
    /// Measured steering table of the truck.
    pub steering: PathBuf,
    pub profile: Profile,
    pub maximum_kmh: f64,
    /// Stop after driving this far (world km, not the game's scaled
    /// navigation km); 0 drives to the end of the route.
    pub distance_km: f64,
    /// Assist logs whose recorded speed limits are reused where the truck
    /// passes the same places; elsewhere `default_limit_kmh`.
    pub limits: Option<PathBuf>,
    pub default_limit_kmh: f64,
    /// Report, incident windows and a 1 Hz track are written here.
    pub out: PathBuf,
    /// Start at this checkpoint instead of the recorded start pose.
    pub start_checkpoint: Option<usize>,
    /// Track period (s): 1 s by default, 0.1 s to compare with a drive.
    pub track_period_s: f64,
    /// AI vehicles kept around the truck (0: none) and the random seed.
    pub traffic: usize,
    pub seed: u64,
    /// The assist is told a semi-trailer is attached (the truck model
    /// itself stays a solo tractor): routes end before rounded corners.
    pub trailer: bool,
    /// Every this many km while engaged the truck is yawed by `kick_rad`,
    /// alternately left and right (0: never): a drift to recover from,
    /// which the model alone never produces past 0.6 m.
    pub kick_every_km: f64,
    pub kick_rad: f64,
}
/// The GPS route as ETS2LA lists it, and where the capture started.
pub struct Route {
    pub nodes: Vec<u64>,
    pub distance_to_end: Vec<f64>,
    pub start: Option<(Pose2, f64, f64)>,
}
impl Route {
    pub fn read(path: &Path) -> anyhow::Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("读取路线 {}", path.display()))?;
        let mut route = Route {
            nodes: Vec::new(),
            distance_to_end: Vec::new(),
            start: None,
        };
        for line in text.lines() {
            let fields: Vec<&str> = line.split_whitespace().collect();
            if let Some(rest) = line.strip_prefix("# start sdk_pose ") {
                let v: Vec<f64> = rest
                    .split_whitespace()
                    .filter_map(|f| f.parse().ok())
                    .collect();
                if v.len() >= 5 {
                    route.start = Some((
                        Pose2 {
                            x: v[0],
                            y: v[1],
                            yaw: v[2],
                        },
                        v[3],
                        v[4],
                    ));
                }
                continue;
            }
            if line.starts_with('#') || fields.len() < 2 {
                continue;
            }
            let node: u64 = fields[0].parse()?;
            if node == 0 || route.nodes.last() == Some(&node) {
                continue;
            }
            route.nodes.push(node);
            route.distance_to_end.push(fields[1].parse()?);
        }
        ensure!(route.nodes.len() >= 2, "路线节点不足");
        Ok(route)
    }
    /// The last checkpoint passed with `remaining_m` of the route left.
    fn passed(&self, remaining_m: f64) -> usize {
        self.distance_to_end
            .iter()
            .position(|d| *d < remaining_m)
            .map_or(0, |i| i.saturating_sub(1))
    }
}
/// Speed limits the game showed, by place and heading, from assist logs.
#[derive(Default)]
struct Limits {
    cells: HashMap<(i64, i64), Vec<(f64, f64)>>,
}
const LIMIT_CELL_M: f64 = 25.0;
impl Limits {
    fn read(dir: &Path) -> Self {
        #[derive(serde::Deserialize)]
        struct Line {
            position_enu: [f64; 3],
            heading_rad: f64,
            limit_kmh: f64,
        }
        let mut limits = Self::default();
        let Ok(entries) = std::fs::read_dir(dir) else {
            return limits;
        };
        for path in entries.filter_map(|e| e.ok().map(|e| e.path())) {
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            for line in text.lines() {
                let Ok(l) = serde_json::from_str::<Line>(line) else {
                    continue;
                };
                if l.limit_kmh > 5.0 {
                    let key = (
                        (l.position_enu[0] / LIMIT_CELL_M).floor() as i64,
                        (l.position_enu[1] / LIMIT_CELL_M).floor() as i64,
                    );
                    let cell = limits.cells.entry(key).or_default();
                    if cell.len() < 8 {
                        cell.push((l.heading_rad, l.limit_kmh / 3.6));
                    }
                }
            }
        }
        limits
    }
    fn at(&self, pose: Pose2) -> Option<f64> {
        let (cx, cy) = (
            (pose.x / LIMIT_CELL_M).floor() as i64,
            (pose.y / LIMIT_CELL_M).floor() as i64,
        );
        (-1..=1)
            .flat_map(|dx| (-1..=1).map(move |dy| (cx + dx, cy + dy)))
            .filter_map(|k| self.cells.get(&k))
            .flatten()
            .find(|(h, _)| etsads_msgs::frames::wrap_angle(h - pose.yaw).abs() < 0.6)
            .map(|(_, v)| *v)
    }
}
/// Traffic lights cycle through their profile's phases: green, amber, red,
/// red-amber (ETS2LA states 8, 1, 2, 4).
fn signal_state(intervals: [f64; 4], delay: f64, t: f64) -> i32 {
    let total: f64 = intervals.iter().sum();
    if total <= 0.0 || !total.is_finite() {
        return 8;
    }
    let mut phase = (t + delay).rem_euclid(total);
    for (interval, state) in intervals.iter().zip([8, 1, 2, 4]) {
        if phase < *interval {
            return state;
        }
        phase -= interval;
    }
    8
}
#[derive(Clone, Debug, Serialize)]
pub struct Incident {
    pub kind: String,
    pub reason: String,
    /// Where on the route (world km driven, and the game's navigation km).
    pub driven_km: f64,
    pub route_km: f64,
    pub sim_s: f64,
    pub x: f64,
    pub y: f64,
    pub speed_kmh: f64,
    pub lateral_m: f64,
    pub lane: String,
    /// Recent 10 Hz history written for this incident.
    pub window: Option<String>,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct Report {
    pub evidence: &'static str,
    /// The route's length and the share covered, in the game's scaled
    /// navigation km (about 19× world distance on ETS2 roads).
    pub route_km: f64,
    pub route_covered_km: f64,
    /// Distances in world km.
    pub driven_km: f64,
    pub assisted_km: f64,
    pub handed_over_km: f64,
    pub sim_hours: f64,
    pub wall_s: f64,
    pub engagements: u32,
    pub mean_assisted_kmh: f64,
    pub peak_lateral_m: f64,
    pub rms_lateral_m: f64,
    /// Share of assisted time beyond 0.3 / 0.6 m of the lane centre.
    pub over_0_3: f64,
    pub over_0_6: f64,
    /// Changes between throttle and brake per assisted km (a speed target
    /// that jumps about shows here).
    pub pedal_reversals_per_km: f64,
    /// Hard braking: episodes beyond 2.5 m/s² for 0.3 s, per 100 assisted
    /// km, and the strongest deceleration (m/s², over 0.3 s).
    pub hard_brakes_per_100km: f64,
    pub peak_deceleration: f64,
    /// Strongest lateral acceleration while assisted (m/s²) and the share
    /// of time beyond 2 m/s².
    pub peak_lateral_acceleration: f64,
    pub over_2_lateral: f64,
    /// Where the reversals happened: by what bound the speed target, and
    /// by speed (10 km/h bands).
    pub reversals_by_binding: Vec<(String, u32)>,
    pub reversals_by_speed: Vec<(u32, u32)>,
    /// Mean change of the speed target per second (km/h/s).
    pub target_jitter_kmh_s: f64,
    pub collisions: u32,
    pub incidents: Vec<Incident>,
    pub handovers_by_reason: Vec<(String, u32)>,
    pub finished: String,
}
#[derive(Clone, Copy, Serialize)]
struct Row {
    t: f64,
    x: f64,
    y: f64,
    yaw: f64,
    kmh: f64,
    target_kmh: f64,
    lateral_m: f64,
    position: f64,
    command: f64,
    input: f64,
    throttle: f64,
    brake: f64,
    gear: usize,
    rpm: f64,
    station_m: f64,
    curvature: f64,
    binding: super::navigation::Binding,
}
/// The assist's state while engaged.
struct Engaged {
    guidance: Guidance,
    feedback: Feedback,
    servo: SteeringServo,
    hold: RelativeHold,
    drift: DriftHold,
    extension_tried_at: f64,
    ending: bool,
}
const DT: f64 = 1.0 / 60.0;
const TRAFFIC_DT: f64 = 3.0 * DT;
pub fn drive(options: &Options) -> anyhow::Result<Report> {
    let wall = Instant::now();
    std::fs::create_dir_all(&options.out)?;
    std::fs::write(
        options.out.join("profile.json"),
        serde_json::to_vec_pretty(&options.profile)?,
    )?;
    let navigator = Navigator::open(&options.map)?;
    let route = Route::read(&options.route)?;
    let map = SteeringMap::read(&options.steering)?;
    let profile = &options.profile;
    let limits = options
        .limits
        .as_deref()
        .map(Limits::read)
        .unwrap_or_default();
    let ceiling = (options.maximum_kmh / 3.6).min(MAX_SPEED);
    let axles = super::core::Axles {
        rear_forward: profile.rear_forward_m,
        rear_left: 0.0,
        wheelbase: profile.wheelbase_m,
    };
    let steering_mode = match profile.relative_rate_per_s {
        Some(rate_per_s) => crate::setup::SteeringMode::Relative { rate_per_s },
        None => crate::setup::SteeringMode::Absolute,
    };
    // Where to begin: the captured pose, or a checkpoint's lane.
    let (mut truck, mut checkpoint) = match (options.start_checkpoint, route.start) {
        (None, Some((sdk, z, remaining))) => {
            let rear = axles.rear_pose(sdk);
            (Truck::new(rear, z, 0.0, 0.0), route.passed(remaining))
        }
        _ => anyhow::bail!("需要路线起点位姿（或改用 --start-checkpoint 并给出位姿）"),
    };
    let total_m = route.distance_to_end[0];
    let goal_km = if options.distance_km > 0.0 {
        options.distance_km
    } else {
        f64::INFINITY
    };
    let mut track =
        std::io::BufWriter::new(std::fs::File::create(options.out.join("track.jsonl"))?);
    let mut report = Report {
        evidence: "simulation_only_not_game_acceptance",
        route_km: total_m / 1000.0,
        ..Default::default()
    };
    let mut history: VecDeque<Row> = VecDeque::new();
    let mut engaged: Option<Engaged> = None;
    let mut pending = Input::default();
    let begin = Instant::now();
    let (mut t, mut next_row, mut next_track) = (0.0_f64, 0.0_f64, 0.0_f64);
    let mut assisted_s = 0.0;
    let (mut lateral_sq, mut over3, mut over6) = (0.0, 0.0, 0.0);
    let mut last_station = None::<f64>;
    let mut handovers: HashMap<String, u32> = HashMap::new();
    let mut window_count = 0;
    let last_checkpoint = route.nodes.len() - 1;
    let mut last_progress = (0.0_f64, 0.0_f64);
    // A drift episode: since when and its peak offset.
    let mut drift = None::<(f64, f64)>;
    let mut next_kick_km = options.kick_every_km;
    let mut kick_side = 1.0;
    let mut traffic = traffic::Traffic::new(options.seed, options.traffic);
    let combination = if options.trailer {
        super::trailer::Combination {
            trailers: 1,
            behind_hitch_m: 13.0,
            first_wheelbase_m: 8.0,
        }
    } else {
        super::trailer::Combination::default()
    };
    let mut collided: Vec<u64> = Vec::new();
    let mut stopped_since = None::<f64>;
    // Why the driver must take over at the next standby (the assist handed
    // back), if it must.
    let mut force_handover = None::<String>;
    // Loop turns without simulated time passing (a run that cannot move on).
    let (mut idle_turns, mut last_turn_t) = (0_u32, -1.0_f64);
    // AI traffic moves at 20 Hz (collisions are checked as often).
    let mut traffic_clock = 0.0_f64;
    let mut plan_reason = String::new();
    let (mut pedal, mut reversals) = (0_i8, 0_u32);
    let mut reversal_bindings: HashMap<String, u32> = HashMap::new();
    let mut reversal_speeds: std::collections::BTreeMap<u32, u32> = Default::default();
    let (mut last_target, mut jitter) = (None::<f64>, 0.0_f64);
    // Speeds over the last 0.3 s, a hard-braking episode under way, and the
    // count of episodes; time beyond 2 m/s² lateral.
    let mut recent_speeds: VecDeque<f64> = VecDeque::new();
    let (mut braking_hard, mut hard_brakes, mut over2) = (false, 0_u32, 0.0_f64);
    let body = |truck: &Truck| traffic::Body {
        rear: truck.rear,
        z: truck.z,
        speed: truck.speed,
        front_m: 5.0,
        back_m: 1.5,
        width: 2.5,
    };
    macro_rules! incident {
        ($kind:expr, $reason:expr, $lateral:expr, $lane:expr) => {{
            let window = if window_count < 200 {
                window_count += 1;
                let name = format!("incident-{window_count:03}.txt");
                let mut f = std::fs::File::create(options.out.join(&name))?;
                writeln!(f, "# {} {}: t x y yaw kmh target_kmh lateral position command input throttle brake gear rpm station", $kind, $reason)?;
                for r in &history {
                    writeln!(f, "{:.3} {:.3} {:.3} {:.5} {:.2} {:.2} {:.3} {:.4} {:.4} {:.4} {:.3} {:.3} {} {:.0} {:.2} {:.4} {:?}", r.t, r.x, r.y, r.yaw, r.kmh, r.target_kmh, r.lateral_m, r.position, r.command, r.input, r.throttle, r.brake, r.gear, r.rpm, r.station_m, r.curvature, r.binding)?;
                }
                Some(name)
            } else {
                None
            };
            report.incidents.push(Incident {
                kind: $kind.into(),
                reason: $reason.to_string(),
                driven_km: report.driven_km,
                route_km: (total_m - route.distance_to_end[checkpoint]) / 1000.0,
                sim_s: t,
                x: truck.rear.x,
                y: truck.rear.y,
                speed_kmh: truck.speed * 3.6,
                lateral_m: $lateral,
                lane: $lane,
                window,
            });
        }};
    }
    let finished = loop {
        if t == last_turn_t {
            idle_turns += 1;
            if idle_turns > 200 {
                break format!("仿真时间停滞，停在 {:.2} km", report.driven_km);
            }
        } else {
            (idle_turns, last_turn_t) = (0, t);
        }
        if report.driven_km >= goal_km {
            break "到达设定里程".to_owned();
        }
        if checkpoint >= last_checkpoint {
            break "已到路线终点".to_owned();
        }
        // No progress for 10 simulated minutes: the run is stuck.
        if report.driven_km > last_progress.1 + 0.02 {
            last_progress = (t, report.driven_km);
        } else if t - last_progress.0 > 600.0 {
            break format!("10 分钟没有进展，停在 {:.2} km", report.driven_km);
        }
        let now = begin + Duration::from_secs_f64(t);
        let rear = truck.rear;
        let Some(state) = engaged.as_mut() else {
            drift = None;
            recent_speeds.clear();
            braking_hard = false;
            stopped_since = None;
            // Standby: prepare from the truck as it stands, as the driver
            // pressing F8 once the assist reports ready.
            let prepared = if let Some(reason) = force_handover.take() {
                Err(anyhow::anyhow!(reason))
            } else {
                navigator
                    .prepare_from(rear, truck.z, truck.speed, &route.nodes, checkpoint, None)
                    .and_then(|mut guidance| {
                        let road = guidance.observe(rear, truck.z, truck.speed, &route.nodes)?;
                        ensure!(
                            road.lateral_m.abs() < 0.55 && road.heading_error_rad.abs() < 0.35,
                            "车辆未对准车道"
                        );
                        guidance.set_relative_steering(profile.relative_rate_per_s);
                        guidance.set_combination(combination);
                        let plan = guidance.speed(
                            &road,
                            rear,
                            truck.speed,
                            limits.at(rear).unwrap_or(options.default_limit_kmh / 3.6),
                            ceiling,
                            &[],
                            t,
                        )?;
                        ensure!(plan.acceleration > -1.0, "前方路段终点过近");
                        Ok(guidance)
                    })
            };
            match prepared {
                Ok(guidance) => {
                    let mut servo = SteeringServo::new(steering_mode);
                    servo.observe(truck.effective(), truck.speed, now);
                    engaged = Some(Engaged {
                        feedback: Feedback::new(axles, map.clone())
                            .with_steering(Some(truck.position), servo.max_rate()),
                        guidance,
                        servo,
                        hold: RelativeHold::default(),
                        drift: DriftHold::default(),
                        extension_tried_at: f64::NEG_INFINITY,
                        ending: false,
                    });
                    report.engagements += 1;
                }
                Err(error) => {
                    // The driver takes over and drives to the next checkpoint.
                    let reason = format!("{error:#}");
                    *handovers.entry(short(&reason)).or_default() += 1;
                    incident!("handover", reason, 0.0, String::new());
                    let (pose, z, reached, driven) =
                        match navigator.drive_through(rear, truck.z, &route.nodes, checkpoint) {
                            Ok(found) => found,
                            Err(error) => {
                                break format!("驾驶员模型无法接续路线：{error:#}（{reason}）");
                            }
                        };
                    checkpoint = reached;
                    report.handed_over_km += driven / 1000.0;
                    report.driven_km += driven / 1000.0;
                    truck = Truck::new(pose, z, 0.0, 0.0);
                    // The driver joins in a gap in the traffic.
                    traffic.clear_near(pose, 30.0);
                    pending = Input::default();
                    history.clear();
                    last_station = None;
                    t += driven / 10.0;
                }
            }
            continue;
        };
        // Telemetry of this frame.
        state.servo.observe(truck.effective(), truck.speed, now);
        let road = match state
            .guidance
            .observe(rear, truck.z, truck.speed, &route.nodes)
        {
            Ok(road) => road,
            Err(error) => {
                let lane = state.guidance.lane(last_station.unwrap_or(0.0)).to_owned();
                incident!("safety_stop", format!("{error:#}"), f64::NAN, lane);
                stop_in_place(&mut truck, profile, &map);
                engaged = None;
                // The driver steers back onto the lane where it stopped.
                recentre(&navigator, &mut truck);
                traffic.clear_near(truck.rear, 30.0);
                continue;
            }
        };
        if let Some(previous) = last_station {
            let advanced = (road.station_m - previous).max(0.0);
            report.driven_km += advanced / 1000.0;
            report.assisted_km += advanced / 1000.0;
        }
        last_station = Some(road.station_m);
        // The road under the truck, read before the route may be swapped
        // below (stations differ between routes).
        let grade = grade_at(&state.guidance, road.station_m);
        if let Some((p, _)) = state.guidance.sample(road.station_m) {
            truck.z = p.z;
        }
        checkpoint = state
            .guidance
            .passed_checkpoint(road.station_m)
            .max(checkpoint);
        // Held up for 90 s (a deadlock with AI traffic, or a vehicle that
        // will not move): the driver takes over and drives on.
        if truck.speed > 0.5 {
            stopped_since = None;
        } else if t - *stopped_since.get_or_insert(t) > 90.0 {
            // Who is around: the nearest AI vehicles and their state.
            let mut near: Vec<(f64, u64)> = traffic
                .agents()
                .iter()
                .map(|a| ((a.pose.x - rear.x).hypot(a.pose.y - rear.y), a.id))
                .filter(|(d, _)| *d < 60.0)
                .collect();
            near.sort_by(|a, b| a.0.total_cmp(&b.0));
            let around: Vec<String> = near
                .iter()
                .take(4)
                .map(|(d, id)| format!("[{id} {d:.0} m: {}]", traffic.describe(*id, &body(&truck))))
                .collect();
            incident!(
                "blocked",
                format!("{plan_reason} {}", around.join(" ")),
                road.lateral_m,
                road.lane.clone()
            );
            stopped_since = None;
            engaged = None;
            traffic.clear_near(truck.rear, 40.0);
            force_handover = Some("被前方车辆堵住，司机接管绕行".into());
            continue;
        }
        // Rolling horizon, prepared at once (the assist does it on a thread).
        if road.remaining_m - 18.0 < 800.0 && t - state.extension_tried_at > 3.0 {
            state.extension_tried_at = t;
            if let Ok(next) = navigator.prepare_from(
                rear,
                truck.z,
                truck.speed,
                &route.nodes,
                state.guidance.passed_checkpoint(road.station_m),
                Some(state.guidance.lane(road.station_m)),
            ) && state
                .guidance
                .adopt(next, rear, truck.z, truck.speed)
                .is_ok_and(|e| e.is_some())
            {
                // A new route: stations start afresh.
                last_station = None;
            }
        }
        state.guidance.set_relative_steering(state.servo.max_rate());
        state.guidance.set_combination(combination);
        let signals = state
            .guidance
            .signal_heads()
            .into_iter()
            .filter(|(_, p, _, _)| {
                p.distance_xy(Point3 {
                    x: rear.x,
                    y: rear.y,
                    z: truck.z,
                }) < 300.0
            })
            .map(|(_, position, intervals, delay)| SignalObservation {
                position,
                raw_state: signal_state(intervals, delay, t),
                age_us: 0,
            })
            .collect();
        state.guidance.observe_signals(signals);
        // AI traffic moves on, then is seen as the game lists it.
        let agents = if options.traffic > 0 {
            let here = Point3 {
                x: rear.x,
                y: rear.y,
                z: truck.z,
            };
            if traffic.needs_lanes(here)
                && let Ok((lanes, controls)) = navigator.region_lanes(here)
            {
                traffic.load_lanes(here, lanes, &controls);
            }
            // The truck's lanes from just behind it to 200 m ahead: AI gives
            // way to them and some spawn on them as leaders.
            let mut ahead: Vec<String> = Vec::new();
            for k in 0..22 {
                let lane = state.guidance.lane(road.station_m - 10.0 + 10.0 * k as f64);
                if lane != "gps" && ahead.last().is_none_or(|l| l != lane) {
                    ahead.push(lane.to_owned());
                }
            }
            let limit_at = |pose: Pose2| limits.at(pose).unwrap_or(options.default_limit_kmh / 3.6);
            traffic_clock += DT;
            let touching = if traffic_clock >= TRAFFIC_DT - 1e-9 {
                traffic_clock -= TRAFFIC_DT;
                traffic.step(TRAFFIC_DT, &body(&truck), &ahead, &limit_at)
            } else {
                Vec::new()
            };
            for id in touching {
                if !collided.contains(&id) {
                    collided.push(id);
                    report.collisions += 1;
                    let car = traffic.agents().into_iter().find(|a| a.id == id);
                    let what = car.map_or(String::new(), |a| {
                        format!(
                            "（对方 {:.1} m 长，{:.0} km/h，位于 {:.1}, {:.1}，朝向差 {:.0}°）",
                            a.length,
                            a.speed * 3.6,
                            a.pose.x,
                            a.pose.y,
                            etsads_msgs::frames::wrap_angle(a.pose.yaw - rear.yaw).to_degrees()
                        )
                    });
                    let detail = traffic.describe(id, &body(&truck));
                    {
                        let name = format!("path-{:03}.txt", window_count + 1);
                        let mut f = std::fs::File::create(options.out.join(name))?;
                        writeln!(
                            f,
                            "# station x y curvature (truck at station {:.1}, rear {:.2} {:.2} yaw {:.3}); AI boxes x y yaw half_length half_width",
                            road.station_m, rear.x, rear.y, rear.yaw
                        )?;
                        for (s, p, k) in state.guidance.path_window(
                            road.station_m - 30.0,
                            road.station_m + 30.0,
                            0.5,
                        ) {
                            writeln!(f, "{s:.1} {:.2} {:.2} {k:.4}", p.x, p.y)?;
                        }
                        for a in traffic.agents().iter().filter(|a| a.id == id) {
                            writeln!(
                                f,
                                "AI {:.2} {:.2} {:.3} {:.2} {:.2}",
                                a.pose.x,
                                a.pose.y,
                                a.pose.yaw,
                                a.length / 2.0,
                                a.width / 2.0
                            )?;
                        }
                    }
                    let lane = state.guidance.lane(road.station_m).to_owned();
                    incident!(
                        "collision",
                        format!("与 AI 车辆 {id} 相撞{what} {detail}"),
                        road.lateral_m,
                        lane
                    );
                }
            }
            traffic.agents()
        } else {
            Vec::new()
        };
        let limit = limits.at(rear).unwrap_or(options.default_limit_kmh / 3.6);
        let plan = match state
            .guidance
            .speed(&road, rear, truck.speed, limit, ceiling, &agents, t)
        {
            Ok(plan) => plan,
            Err(error) => {
                incident!(
                    "safety_stop",
                    format!("{error:#}"),
                    road.lateral_m,
                    road.lane.clone()
                );
                stop_in_place(&mut truck, profile, &map);
                engaged = None;
                continue;
            }
        };
        plan_reason.clone_from(&plan.reason);
        // End of the usable route: stop, then the driver takes over.
        if road.remaining_m < 21.0 && truck.speed.abs() < 0.15 {
            let terminal = state.guidance.terminal.clone();
            engaged = None;
            if terminal.starts_with("即将到达目的地") && checkpoint + 3 >= last_checkpoint {
                break "已到达目的地".to_owned();
            }
            // Handed back short of the destination: the driver drives on.
            force_handover = Some(format!("路线可用段结束：{terminal}"));
            continue;
        }
        // The path around the truck when a drift begins (diagnostics).
        if road.lateral_m.abs() >= DRIFT_WARN_M && drift.is_none() && window_count < 200 {
            let name = format!("path-{:03}.txt", window_count + 1);
            let mut f = std::fs::File::create(options.out.join(name))?;
            writeln!(
                f,
                "# station x y curvature (truck at station {:.1})",
                road.station_m
            )?;
            for (s, p, k) in
                state
                    .guidance
                    .path_window(road.station_m - 60.0, road.station_m + 60.0, 1.0)
            {
                writeln!(f, "{s:.1} {:.2} {:.2} {k:.4}", p.x, p.y)?;
            }
        }
        let drift_cap = state.drift.cap(
            road.lateral_m,
            truck.speed,
            drift_speed(state.servo.max_rate().is_some()),
        );
        let drifting = drift_cap.is_some();
        let (target, planned) = match drift_cap {
            Some(cap) => (
                plan.target.min(cap),
                if truck.speed > cap + super::DRIFT_BRAKE_ABOVE {
                    plan.acceleration.min(-1.0)
                } else {
                    plan.acceleration
                },
            ),
            None => (plan.target, plan.acceleration),
        };
        let (target, planned) = match state.servo.max_rate() {
            Some(_) => {
                match state.hold.cap(
                    state.feedback.tracking_lag(state.servo.position(now)),
                    road.lateral_m,
                    truck.speed,
                ) {
                    Some(held) => (target.min(held), planned.min(0.0)),
                    None => (target, planned),
                }
            }
            None => (target, planned),
        };
        state.feedback.coast = plan.approach;
        state.feedback.observe_position(state.servo.position(now));
        let output = match state.feedback.step(
            &road,
            rear,
            truck.speed,
            if state.ending { 0.0 } else { target },
            planned,
            DT,
        ) {
            Ok(output) => output,
            Err(error) => {
                incident!(
                    "safety_stop",
                    format!("{error:#}"),
                    road.lateral_m,
                    road.lane.clone()
                );
                stop_in_place(&mut truck, profile, &map);
                engaged = None;
                recentre(&navigator, &mut truck);
                traffic.clear_near(truck.rear, 30.0);
                continue;
            }
        };
        let steer = state.servo.input(output.steer, now);
        let command = Input {
            steer,
            throttle: output.throttle,
            brake: output.brake,
            clutch: clutch(output.throttle, output.brake, truck.speed, truck.rpm) as f64,
        };
        // Statistics while assisted.
        assisted_s += DT;
        let now_pedal = if output.throttle > 0.05 {
            1
        } else if output.brake > 0.05 {
            -1
        } else {
            pedal
        };
        if now_pedal != pedal && pedal != 0 {
            reversals += 1;
            *reversal_bindings
                .entry(format!("{:?}", plan.binding))
                .or_default() += 1;
            *reversal_speeds
                .entry((truck.speed * 3.6 / 10.0) as u32 * 10)
                .or_default() += 1;
        }
        pedal = now_pedal;
        if let Some(last) = last_target {
            jitter += (plan.target - last).abs() * 3.6;
        }
        last_target = Some(plan.target);
        lateral_sq += road.lateral_m * road.lateral_m * DT;
        recent_speeds.push_back(truck.speed);
        if recent_speeds.len() > (0.3 / DT) as usize {
            let decel = (recent_speeds.pop_front().unwrap_or(truck.speed) - truck.speed) / 0.3;
            report.peak_deceleration = report.peak_deceleration.max(decel);
            if decel > 2.5 && !braking_hard {
                hard_brakes += 1;
                incident!(
                    "hard_brake",
                    format!("减速 {decel:.1} m/s²（{}）", plan.reason),
                    road.lateral_m,
                    road.lane.clone()
                );
            }
            braking_hard = decel > 2.0 && (braking_hard || decel > 2.5);
        }
        let lateral_acceleration = (truck.speed * truck.yaw_rate).abs();
        report.peak_lateral_acceleration =
            report.peak_lateral_acceleration.max(lateral_acceleration);
        over2 += if lateral_acceleration > 2.0 { DT } else { 0.0 };
        over3 += if road.lateral_m.abs() > 0.3 { DT } else { 0.0 };
        over6 += if road.lateral_m.abs() > 0.6 { DT } else { 0.0 };
        if road.lateral_m.abs() > report.peak_lateral_m {
            report.peak_lateral_m = road.lateral_m.abs();
        }
        // One incident per drift, written when it has settled (below 0.3 m).
        if drifting {
            let (since, peak) = drift.get_or_insert((t, 0.0));
            if road.lateral_m.abs() > peak.abs() {
                *peak = road.lateral_m;
            }
            let _ = since;
        } else if let Some((since, peak)) = drift.filter(|_| road.lateral_m.abs() < 0.3) {
            drift = None;
            incident!(
                "drift",
                format!("偏离峰值 {:.2} m，持续 {:.1} s", peak, t - since),
                peak,
                road.lane.clone()
            );
        }
        if t >= next_row {
            next_row = t + 0.1;
            history.push_back(Row {
                curvature: state.guidance.curvature(road.station_m),
                binding: plan.binding,
                ..row(t, &truck, &road, &output, steer, plan.target)
            });
            while history.len() > 300 {
                history.pop_front();
            }
        }
        if t >= next_track {
            next_track = t + options.track_period_s;
            serde_json::to_writer(
                &mut track,
                &serde_json::json!({"t":t,"km":report.driven_km,"route_km":(total_m-route.distance_to_end[checkpoint])/1000.0,"x":truck.rear.x,"y":truck.rear.y,"kmh":truck.speed*3.6,"target_kmh":plan.target*3.6,"limit_kmh":limit*3.6,"lateral_m":road.lateral_m,"binding":plan.binding,"reason":plan.reason,"gear":truck.gear,"throttle":output.throttle,"brake":output.brake,"brake_gain":state.feedback.brake_gain(),"accel":plan.acceleration,"heading":road.heading_error_rad,"path_k":state.guidance.curvature(road.station_m),"wheel":truck.wheel,"lead":plan.lead,"yielding":plan.yielding,"lead_lanes":plan.lead.map(|l| traffic.lanes_of(l.agent_id))}),
            )?;
            writeln!(track)?;
        }
        if options.kick_every_km > 0.0 && report.driven_km >= next_kick_km {
            truck.rear.yaw += options.kick_rad * kick_side;
            kick_side = -kick_side;
            next_kick_km += options.kick_every_km;
        }
        // The game applies the bridge's input on the next frame.
        truck.step(pending, grade, profile, &map, DT);
        pending = command;
        t += DT;
    };
    track.flush()?;
    report.finished = finished;
    report.route_covered_km = (total_m - route.distance_to_end[checkpoint]) / 1000.0;
    report.sim_hours = t / 3600.0;
    report.wall_s = wall.elapsed().as_secs_f64();
    report.mean_assisted_kmh = report.assisted_km / (assisted_s / 3600.0).max(1e-9);
    report.rms_lateral_m = (lateral_sq / assisted_s.max(1e-9)).sqrt();
    report.over_0_3 = over3 / assisted_s.max(1e-9);
    report.over_0_6 = over6 / assisted_s.max(1e-9);
    report.pedal_reversals_per_km = reversals as f64 / report.assisted_km.max(1e-9);
    report.hard_brakes_per_100km = 100.0 * hard_brakes as f64 / report.assisted_km.max(1e-9);
    report.over_2_lateral = over2 / assisted_s.max(1e-9);
    report.target_jitter_kmh_s = jitter / assisted_s.max(1e-9);
    let mut bindings: Vec<_> = reversal_bindings.into_iter().collect();
    bindings.sort_by_key(|r| std::cmp::Reverse(r.1));
    report.reversals_by_binding = bindings;
    report.reversals_by_speed = reversal_speeds.into_iter().collect();
    let mut reasons: Vec<_> = handovers.into_iter().collect();
    reasons.sort_by_key(|r| std::cmp::Reverse(r.1));
    report.handovers_by_reason = reasons;
    std::fs::write(
        options.out.join("report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(report)
}
fn row(
    t: f64,
    truck: &Truck,
    road: &super::core::RoadState,
    output: &Output,
    input: f64,
    target: f64,
) -> Row {
    Row {
        t,
        x: truck.rear.x,
        y: truck.rear.y,
        yaw: truck.rear.yaw,
        kmh: truck.speed * 3.6,
        target_kmh: target * 3.6,
        lateral_m: road.lateral_m,
        position: truck.position,
        command: output.steer,
        input,
        throttle: output.throttle,
        brake: output.brake,
        gear: truck.gear,
        rpm: truck.rpm,
        station_m: road.station_m,
        curvature: 0.0,
        binding: super::navigation::Binding::Limit,
    }
}
fn grade_at(guidance: &Guidance, s: f64) -> f64 {
    match (guidance.sample(s - 4.0), guidance.sample(s + 4.0)) {
        (Some((a, _)), Some((b, _))) if a.distance_xy(b) > 1.0 => (b.z - a.z) / a.distance_xy(b),
        _ => 0.0,
    }
}
/// The minimal-risk stop: wheel held, braking to a standstill.
fn stop_in_place(truck: &mut Truck, profile: &Profile, map: &SteeringMap) {
    let input = Input {
        steer: 0.0,
        throttle: 0.0,
        brake: 0.55,
        clutch: 1.0,
    };
    for _ in 0..(30.0 / DT) as usize {
        if truck.speed <= 0.0 {
            break;
        }
        let hold = Input {
            steer: if profile.relative_rate_per_s.is_some() {
                0.0
            } else {
                truck.position
            },
            ..input
        };
        truck.step(hold, 0.0, profile, map, DT);
    }
}
/// After a safety stop the driver puts the truck back on its lane, where it
/// stopped (the next preparation matches it there).
fn recentre(navigator: &Navigator, truck: &mut Truck) {
    if let Ok((pose, z)) = navigator.lane_centre(truck.rear, truck.z) {
        *truck = Truck::new(pose, z, 0.0, 0.0);
    }
}
/// A handover reason without its numbers, for counting.
fn short(reason: &str) -> String {
    reason
        .chars()
        .filter(|c| !c.is_ascii_digit() && *c != '.')
        .take(40)
        .collect()
}
/// The measured steering table at `path`.
pub fn steering_map(path: &Path) -> anyhow::Result<SteeringMap> {
    SteeringMap::read(path)
}
