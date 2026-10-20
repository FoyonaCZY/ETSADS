//! Read-only commissioning snapshot. No input/control/gate mapping is written.
use etsads_game::{adapter::GameAdapter, ets2la};
use etsads_msgs::protocol::*;
use etsads_shm::SharedCell;
use serde_json::json;
fn main() -> anyhow::Result<()> {
    let mut adapter = GameAdapter::connect()?;
    let started = std::time::Instant::now();
    loop {
        if let Some(s) = adapter.poll()? {
            let camera=s.extension.as_ref().and_then(|e|ets2la::records::<ets2la::Camera>(&e.camera).next()).map(|c|{let fov=c.fov;let pos=c.position;let sector=c.sector;let q=c.quaternion_wxyz;let projection=c.projection;let truck=c.truck_position;let truck_q=c.truck_quaternion_wxyz;json!({"fov":fov,"position_scs":ets2la::sector_world(pos,sector),"quaternion_wxyz":q,"projection":projection,"render_truck_position":truck,"render_truck_quaternion":truck_q})});
            let config=s.config.map(|c|json!({"wheel_count":c.wheel_count,"wheel_positions":c.wheel_positions,"wheel_radius":c.wheel_radius,"wheel_steerable":c.wheel_steerable,"hook":c.hook_position,"cabin":c.cabin_position,"cargo_mass_kg":c.cargo_mass_kg}));
            let applied=SharedCell::<AppliedControl>::open(APPLIED_MAPPING,SCHEMA_VERSION).ok().and_then(|r|r.latest()).map(|a|json!({"seq":a.applied_seq,"qpc":a.applied_qpc,"frame":a.frame_index,"axes":a.axes,"reason":a.reason}));
            let gate=SharedCell::<Gate>::open(GATE_MAPPING,SCHEMA_VERSION).ok().and_then(|r|r.latest()).map(|g|json!({"seq":g.seq,"qpc":g.heartbeat_qpc,"enabled":g.enabled,"aeb":g.aeb_brake,"throttle_limit":g.max_throttle,"steer_rate":g.max_steer_rate}));
            eprintln!(
                "{}",
                json!({"now_qpc":etsads_shm::qpc(),"applied":applied,"gate":gate})
            );
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &json!({"source":"game","ego":s.ego,"position_scs":s.raw.position_scs,"orientation_turns":s.raw.orientation_turns,"input":s.raw.input,"effective":s.raw.effective,"wheel_steering_turns":s.raw.wheel_steering,"navigation_speed_limit_mps":s.raw.navigation_speed_limit_mps,"parking_brake":s.raw.parking_brake,"qpc":s.raw.header.qpc,"qpc_frequency":s.raw.qpc_frequency,"local_scale":s.raw.local_scale,"configuration":config,"camera":camera,"agents":s.scene.agents,"extension_available":s.extension.as_ref().map(|e|e.available),"route_nodes":s.scene.route.len()})
                )?
            );
            break;
        }
        anyhow::ensure!(
            started.elapsed().as_secs() < 5,
            "no coherent snapshot in five seconds"
        );
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    Ok(())
}
