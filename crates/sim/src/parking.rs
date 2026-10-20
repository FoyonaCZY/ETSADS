//! Closed-loop parking evaluation, separate from the easier planner-only suite.
use crate::SimError;
use etsads_control::{Controller, ControllerConfig, TruckState};
use etsads_msgs::{
    BehaviorKind, Direction, EgoState, Header, OrientedBox, Pose2, TrajPoint, Trajectory,
    VehicleGeometry, frames::wrap_angle,
};
use etsads_planning::{
    geometry::{footprints, overlaps, swept_collision},
    parking::{ParkingPoint, ParkingRequest, plan},
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug)]
pub struct ParkingConfig {
    pub start: TruckState,
    pub target: Pose2,
    pub obstacles: Vec<OrientedBox>,
    pub drivable: Vec<(f64, f64)>,
    pub max_expansions: usize,
    pub initial_position_error: f64,
    pub actuator_lag_s: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingSample {
    pub time_s: f64,
    pub state: TruckState,
    pub reference: TruckState,
    pub direction: Direction,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingReport {
    pub evidence: String,
    pub success: bool,
    pub position_error_m: f64,
    pub heading_error_rad: f64,
    pub collision_count: usize,
    pub interventions: usize,
    pub elapsed_sim_s: f64,
    pub planning_expansions: usize,
    #[serde(default)]
    pub planning_search: etsads_planning::parking::ParkingSearchStatistics,
    #[serde(default)]
    pub planning_seconds: f64,
    pub gear_changes: usize,
    pub max_articulation_rad: f64,
    pub samples: Vec<ParkingSample>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingTrial {
    pub index: usize,
    pub target: Pose2,
    pub report: Option<ParkingReport>,
    pub failure: Option<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingSuite {
    pub evidence: String,
    pub attempts: usize,
    pub successes: usize,
    pub success_rate: f64,
    pub passed_offline_threshold: bool,
    pub trials: Vec<ParkingTrial>,
}
/// Fifty deterministic offset-docking trials beside two parked trailers. The
/// synthetic suite varies geometry, world orientation and actuator disturbance.
/// It is deliberately not labelled as the game's difficult parking benchmark.
pub fn run_parking_suite() -> ParkingSuite {
    let mut trials = Vec::new();
    let mut successes = 0;
    for i in 0..50 {
        let angle = (i % 8) as f64 * std::f64::consts::FRAC_PI_4;
        let rotate = |p: Pose2| Pose2 {
            x: p.x * angle.cos() - p.y * angle.sin(),
            y: p.x * angle.sin() + p.y * angle.cos(),
            yaw: wrap_angle(p.yaw + angle),
        };
        let target = Pose2 {
            x: -17.5 - 0.5 * (i % 3) as f64,
            y: 0.8 + 0.1 * (i % 5) as f64,
            yaw: ((i % 5) as f64 - 2.0) * 0.005,
        };
        let obstacles = vec![
            OrientedBox {
                pose: rotate(Pose2 {
                    x: target.x - 6.0,
                    y: target.y - 3.2,
                    yaw: target.yaw,
                }),
                length: 14.0,
                width: 2.5,
            },
            OrientedBox {
                pose: rotate(Pose2 {
                    x: target.x - 6.0,
                    y: target.y + 3.5,
                    yaw: target.yaw,
                }),
                length: 14.0,
                width: 2.5,
            },
        ];
        let drivable = [(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)]
            .map(|(x, y)| {
                let p = rotate(Pose2 { x, y, yaw: 0.0 });
                (p.x, p.y)
            })
            .to_vec();
        let target = rotate(target);
        let result = run_parking(ParkingConfig {
            start: TruckState {
                pose: rotate(Pose2::default()),
                trailer_yaw: angle,
                ..Default::default()
            },
            target,
            obstacles,
            drivable,
            max_expansions: 80_000,
            initial_position_error: ((i % 5) as f64 - 2.0) * 0.02,
            actuator_lag_s: 0.04 + 0.02 * (i % 5) as f64,
        });
        match result {
            Ok(mut report) => {
                if report.success {
                    successes += 1;
                }
                if i != 0 && report.success {
                    report.samples.clear();
                }
                trials.push(ParkingTrial {
                    index: i,
                    target,
                    report: Some(report),
                    failure: None,
                });
            }
            Err(e) => trials.push(ParkingTrial {
                index: i,
                target,
                report: None,
                failure: Some(e.to_string()),
            }),
        }
    }
    let success_rate = successes as f64 / 50.0;
    ParkingSuite{evidence:"50 synthetic obstacle-docking closed-loop trials; not the 50 difficult ETS2 game deliveries required by P4".into(),attempts:50,successes,success_rate,passed_offline_threshold:success_rate>=0.9,trials}
}
fn trajectory(points: &[ParkingPoint], direction: Direction, g: VehicleGeometry) -> Trajectory {
    let mut total = 0.0;
    let mut distances = vec![0.0];
    for pair in points.windows(2) {
        total += pair[0].state.pose.distance(pair[1].state.pose);
        distances.push(total);
    }
    let sign = if direction == Direction::Reverse {
        -1.0
    } else {
        1.0
    };
    let points = points
        .iter()
        .zip(distances)
        .enumerate()
        .map(|(i, (p, s))| TrajPoint {
            t: i as f64 * 0.2,
            pose: p.state.pose,
            speed: sign * ((2.0 * 0.6 * (total - s)).sqrt().min(3.0 / 3.6)),
            acceleration: if total - s < 0.7 { -0.8 * sign } else { 0.0 },
            curvature: p.state.steer.tan() / g.wheelbase,
            articulation: p.state.articulation(),
        })
        .collect();
    Trajectory {
        header: Header::default(),
        points,
        direction,
        behavior: BehaviorKind::Parking,
    }
}
pub fn run_parking(config: ParkingConfig) -> Result<ParkingReport, SimError> {
    if !config.initial_position_error.is_finite()
        || config.initial_position_error.abs() > 0.5
        || !config.actuator_lag_s.is_finite()
        || !(0.0..=0.5).contains(&config.actuator_lag_s)
    {
        return Err(SimError::Invalid);
    }
    let g = VehicleGeometry::default();
    let planning_start = std::time::Instant::now();
    let plan = plan(ParkingRequest {
        start: config.start,
        target: config.target,
        geometry: g,
        obstacles: &config.obstacles,
        drivable: &config.drivable,
        max_expansions: config.max_expansions,
        position_tolerance: 0.25,
        angle_tolerance: 1.5_f64.to_radians(),
    })?;
    let planning_seconds = planning_start.elapsed().as_secs_f64();
    let mut segments: Vec<(Direction, Vec<ParkingPoint>)> = Vec::new();
    for p in &plan.points[1..] {
        if segments.last().is_none_or(|(d, _)| *d != p.direction) {
            let previous = segments
                .last()
                .and_then(|(_, p)| p.last())
                .copied()
                .unwrap_or(plan.points[0]);
            segments.push((p.direction, vec![previous]));
        }
        segments.last_mut().unwrap().1.push(*p);
    }
    let mut state = TruckState {
        speed: 0.0,
        ..config.start
    };
    state.pose.y += config.initial_position_error;
    let mut controller = Controller::new(ControllerConfig {
        delay: config.actuator_lag_s,
        ..Default::default()
    })?;
    let mut samples = Vec::new();
    let mut time = 0.0;
    let mut collision_count = 0;
    let mut interventions = 0;
    let mut max_articulation: f64 = 0.0;
    let mut completed = 0;
    let mut actuator = state.steer;
    'segments: for (direction, points) in &segments {
        let traj = trajectory(points, *direction, g);
        if traj.points.len() < 2 {
            continue;
        }
        controller.reset();
        let end = points.last().unwrap().state;
        let mut segment_completed = false;
        for step in 0..9000 {
            if state.pose.distance(end.pose) < 0.35 && state.speed.abs() < 0.08 {
                completed += 1;
                segment_completed = true;
                break;
            }
            let ego = EgoState {
                pose: state.pose,
                speed: state.speed,
                wheel_angle: state.steer,
                trailer_yaw: state.trailer_yaw,
                yaw_rate: state.speed * state.steer.tan() / g.wheelbase,
                ..Default::default()
            };
            let command = controller.track(ego, &traj, 0.02)?;
            if command.brake == 1.0 && state.articulation().abs() > 0.6 {
                interventions += 1;
                break 'segments;
            }
            let sign = if *direction == Direction::Reverse {
                -1.0
            } else {
                1.0
            };
            // Brake opposes actual motion, including the residual velocity at a
            // gear change. Applying it in the selected gear direction can turn
            // braking into acceleration when crossing a cusp.
            let acceleration = sign * command.throttle * 1.2
                - state.speed.signum() * (command.brake * 4.0).min(state.speed.abs() / 0.02);
            let desired = command.steer * g.max_steer;
            actuator = if config.actuator_lag_s > 0.0 {
                actuator + (desired - actuator) * (1.0 - (-0.02 / config.actuator_lag_s).exp())
            } else {
                desired
            };
            let next = match state.step(g, actuator, acceleration, 0.02) {
                Ok(next) => next,
                Err(_) => {
                    interventions += 1;
                    break 'segments;
                }
            };
            let from = footprints(state, g);
            let to = footprints(next, g);
            if config.obstacles.iter().any(|o| {
                (0..to.len())
                    .any(|i| overlaps(to[i], *o, 0.0) || swept_collision(from[i], to[i], *o, 0.0))
            }) {
                collision_count += 1;
                break 'segments;
            }
            if to
                .iter()
                .any(|b| !etsads_planning::geometry::box_in_polygon(*b, &config.drivable))
            {
                interventions += 1;
                break 'segments;
            }
            state = next;
            time += 0.02;
            max_articulation = max_articulation.max(state.articulation().abs());
            if step % 5 == 0 {
                let reference = points
                    .iter()
                    .min_by(|a, b| {
                        a.state
                            .pose
                            .distance(state.pose)
                            .total_cmp(&b.state.pose.distance(state.pose))
                    })
                    .unwrap()
                    .state;
                samples.push(ParkingSample {
                    time_s: time,
                    state,
                    reference,
                    direction: *direction,
                });
            }
            if time > 240.0 {
                interventions += 1;
                break 'segments;
            }
        }
        if !segment_completed {
            interventions += 1;
            break;
        }
    }
    let axle = state.trailer_axle(g);
    let position_error = axle.distance(config.target);
    let heading_error = wrap_angle(axle.yaw - config.target.yaw).abs();
    Ok(ParkingReport{evidence:"synthetic closed-loop articulated truck, bounded steering actuator; not an ETS2 delivery event".into(),success:completed==segments.len()&&position_error<0.5&&heading_error<3.0_f64.to_radians()&&collision_count==0&&interventions==0,position_error_m:position_error,heading_error_rad:heading_error,collision_count,interventions,elapsed_sim_s:time,planning_expansions:plan.expansions,planning_search:plan.search,planning_seconds,gear_changes:segments.len().saturating_sub(1),max_articulation_rad:max_articulation,samples})
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn offset_docking_between_parked_trailers_closes_the_loop() {
        let report = run_parking(ParkingConfig {
            start: TruckState::default(),
            target: Pose2 {
                x: -18.0,
                y: 1.0,
                yaw: 0.0,
            },
            obstacles: vec![
                OrientedBox {
                    pose: Pose2 {
                        x: -24.0,
                        y: -2.2,
                        yaw: 0.0,
                    },
                    length: 14.0,
                    width: 2.5,
                },
                OrientedBox {
                    pose: Pose2 {
                        x: -24.0,
                        y: 4.5,
                        yaw: 0.0,
                    },
                    length: 14.0,
                    width: 2.5,
                },
            ],
            drivable: vec![(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)],
            max_expansions: 10000,
            initial_position_error: 0.03,
            actuator_lag_s: 0.08,
        })
        .unwrap();
        assert!(
            report.success,
            "position {} heading {} interventions {}",
            report.position_error_m, report.heading_error_rad, report.interventions
        );
        assert!(report.gear_changes >= 1);
        assert_eq!(report.collision_count, 0);
    }
}
