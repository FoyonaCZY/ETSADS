//! Distinct synthetic geometries, not rotations of the offset-docking suite.
use etsads_control::TruckState;
use etsads_msgs::{OrientedBox, Pose2};
use etsads_sim::parking::{ParkingConfig, run_parking};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("output JSON path required")?;
    let selected = std::env::args()
        .nth(2)
        .map(|v| v.parse::<usize>())
        .transpose()?;
    let mut reports = vec![];
    for (index, (name, target, start_x, barrier)) in [
        (
            "perpendicular_bay",
            Pose2 {
                x: -15.0,
                y: 8.0,
                yaw: std::f64::consts::FRAC_PI_2,
            },
            0.0,
            false,
        ),
        (
            "diagonal_bay",
            Pose2 {
                x: -20.0,
                y: 12.0,
                yaw: std::f64::consts::FRAC_PI_4,
            },
            0.0,
            false,
        ),
        (
            "blocked_straight_corridor",
            Pose2 {
                x: -25.0,
                y: 0.0,
                yaw: 0.0,
            },
            10.0,
            true,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        if selected.is_some_and(|selected| selected != index) {
            continue;
        }
        let obstacles = if barrier {
            vec![OrientedBox {
                pose: Pose2 {
                    x: -7.0,
                    y: 0.0,
                    yaw: 0.0,
                },
                length: 2.0,
                width: 8.0,
            }]
        } else {
            [-3.3, 3.3]
                .map(|side| OrientedBox {
                    pose: Pose2 {
                        x: target.x + 2.2 * target.yaw.cos() - side * target.yaw.sin(),
                        y: target.y + 2.2 * target.yaw.sin() + side * target.yaw.cos(),
                        yaw: target.yaw,
                    },
                    length: 13.6,
                    width: 2.5,
                })
                .to_vec()
        };
        let config = ParkingConfig {
            start: TruckState {
                pose: Pose2 {
                    x: start_x,
                    ..Default::default()
                },
                ..Default::default()
            },
            target,
            obstacles,
            drivable: vec![(-50.0, -40.0), (50.0, -40.0), (50.0, 50.0), (-50.0, 50.0)],
            max_expansions: 80_000,
            initial_position_error: 0.03,
            actuator_lag_s: 0.08,
        };
        let inputs = serde_json::json!({"start":config.start,"target":target,"obstacles":config.obstacles,"drivable":config.drivable,"max_expansions":config.max_expansions});
        let begin = std::time::Instant::now();
        let result = run_parking(config);
        eprintln!(
            "{name}: elapsed={:.3}s success={}",
            begin.elapsed().as_secs_f64(),
            result.as_ref().is_ok_and(|r| r.success)
        );
        reports.push(serde_json::json!({"name":name,"evidence":"synthetic articulated closed loop; not game delivery acceptance","inputs":inputs,"wall_seconds":begin.elapsed().as_secs_f64(),"report":result.as_ref().ok(),"error":result.as_ref().err().map(ToString::to_string)}));
        std::fs::write(&path, serde_json::to_vec_pretty(&reports)?)?;
    }
    Ok(())
}
