use etsads_control::TruckState;
use etsads_msgs::{OrientedBox, Pose2, VehicleGeometry};
use etsads_planning::parking::{ParkingRequest, plan};
fn main() {
    let geometry = VehicleGeometry::default();
    let polygon = [(-40.0, -25.0), (40.0, -25.0), (40.0, 25.0), (-40.0, 25.0)];
    let cases = std::env::args()
        .nth(1)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(4);
    let with_obstacles = std::env::args().any(|s| s == "obstacles");
    for (offset, yaw) in [(1.0, 0.0), (2.0, 0.1), (3.0, 0.2), (-2.0, -0.15)]
        .into_iter()
        .take(cases)
    {
        let start = TruckState::default();
        let target = Pose2 {
            x: -18.0,
            y: offset,
            yaw,
        };
        let time = std::time::Instant::now();
        let obstacles = if with_obstacles {
            vec![
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
            ]
        } else {
            vec![]
        };
        let result = plan(ParkingRequest {
            start,
            target,
            geometry,
            obstacles: &obstacles,
            drivable: &polygon,
            max_expansions: 30_000,
            position_tolerance: 0.5,
            angle_tolerance: 3.0_f64.to_radians(),
        });
        match result {
            Ok(p) => println!(
                "offset={offset} yaw={yaw}: success expansions={} seconds={:.3} points={}",
                p.expansions,
                time.elapsed().as_secs_f64(),
                p.points.len()
            ),
            Err(e) => println!(
                "offset={offset} yaw={yaw}: {e}, seconds={:.3}",
                time.elapsed().as_secs_f64()
            ),
        }
        let result = etsads_sim::parking::run_parking(etsads_sim::parking::ParkingConfig {
            start,
            target,
            obstacles,
            drivable: polygon.to_vec(),
            max_expansions: 150_000,
            initial_position_error: 0.03,
            actuator_lag_s: 0.08,
        });
        match result {
            Ok(mut report) => {
                for s in report.samples.iter().take(250).step_by(25) {
                    println!(
                        "t={:.1} pos=({:.2},{:.2}) yaw={:.2}/{:.2} ref=({:.2},{:.2}) yaw={:.2}/{:.2} speed={:.2}",
                        s.time_s,
                        s.state.pose.x,
                        s.state.pose.y,
                        s.state.pose.yaw,
                        s.state.trailer_yaw,
                        s.reference.pose.x,
                        s.reference.pose.y,
                        s.reference.pose.yaw,
                        s.reference.trailer_yaw,
                        s.state.speed
                    );
                }
                if let Some(last) = report.samples.last() {
                    println!("last={}", serde_json::to_string(last).unwrap());
                }
                report.samples.clear();
                println!("{}", serde_json::to_string(&report).unwrap());
            }
            Err(e) => println!("closed-loop: {e}"),
        }
    }
}
