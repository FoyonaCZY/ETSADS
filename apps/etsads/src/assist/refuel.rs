//! Refuelling on the way. Below a fifth of the tank the route is turned
//! through a fuel station the GPS route passes (`Navigator::plan_refuel`),
//! the truck stops beside a pump, the game's `activate` button (Enter by
//! default) is held while the tank fills, and the drive continues.
//!
//! In the game the fuel runs only while the button is held; the engine stops
//! meanwhile. About 35 L/s (live: 161 → 380 L in 6.5 s, the driver had
//! stopped with the cab level with the pump); afterwards the stall restart
//! starts the engine again.

use anyhow::{Context, ensure};
use etsads_msgs::Point3;
use serde::Deserialize;
use std::path::Path;

/// Written into the runtime map by the exporter (`fuel-stations`).
pub const FILE: &str = "fuel_stations.json";
/// Below this share of the tank the assist looks for a station.
pub const LOW_SHARE: f64 = 0.20;

/// A fuel-station spawn point: the truck stops with its own position (the
/// SDK's, near the cab) beside it.
#[derive(Clone, Debug, Deserialize)]
pub struct Pump {
    pub id: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Pump {
    pub fn position(&self) -> Point3 {
        Point3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}
#[derive(Deserialize)]
struct Stations {
    schema_version: u32,
    pumps: Vec<Pump>,
}
pub struct Pumps(Vec<Pump>);
impl Pumps {
    /// `None` for a map generated before pumps were exported.
    pub fn load(map: &Path) -> anyhow::Result<Option<Self>> {
        let path = map.join(FILE);
        if !path.is_file() {
            return Ok(None);
        }
        let bytes = std::fs::read(&path).with_context(|| format!("读取 {}", path.display()))?;
        let stations: Stations = serde_json::from_slice(&bytes).context("加油站数据无法解析")?;
        ensure!(stations.schema_version == 1, "加油站数据版本不支持");
        Ok(Some(Self(stations.pumps)))
    }
    pub fn near(&self, p: Point3, radius: f64) -> Vec<Pump> {
        self.0
            .iter()
            .filter(|pump| (pump.x - p.x).hypot(pump.y - p.y) <= radius)
            .cloned()
            .collect()
    }
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}
/// Whether the tank is low enough to turn into a station.
pub fn wants_fuel(fuel_l: f64, capacity_l: f64) -> bool {
    capacity_l > 0.0 && fuel_l.is_finite() && fuel_l < LOW_SHARE * capacity_l
}

/// Stopped beside the pump: settle, then hold the button while the tank
/// fills, until it is nearly full or stops filling. If it does not start,
/// let go, switch a running engine off and hold again; three tries.
pub struct Refuel {
    capacity_l: f64,
    stage: Stage,
    /// When the stage began (s).
    at: f64,
    start_fuel: f64,
    best: f64,
    last_rise: f64,
    tries: u8,
    engine_switched: bool,
    filled: bool,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    Settling,
    Holding,
    /// Let go between tries.
    Pause,
    EngineOff,
}
#[derive(Debug, Default, PartialEq)]
pub struct RefuelStep {
    /// Hold the `activate` button this frame.
    pub hold: bool,
    /// Press the engine switch once.
    pub engine: bool,
    /// Litres added once done, or why it did not refuel.
    pub finished: Option<Result<f64, &'static str>>,
}
/// Standing still this long before holding the button (s).
const SETTLE_S: f64 = 1.0;
/// Held this long with no fuel: let go and try again (s).
const ANSWER_S: f64 = 4.0;
/// Let go this long between tries (s).
const PAUSE_S: f64 = 1.0;
/// The engine is given this long to stop before the next try (s).
const ENGINE_OFF_S: f64 = 2.0;
/// Tries without fuel before giving up.
const MAX_TRIES: u8 = 3;
/// Done once the tank rose no more for this long while held (s).
const FULL_S: f64 = 2.0;
/// Done at this share of a known tank.
const FULL_SHARE: f64 = 0.98;
/// However long the game takes, a stop ends after this (s).
const LIMIT_S: f64 = 180.0;
impl Refuel {
    /// `capacity_l` 0 when unknown.
    pub fn new(t: f64, fuel_l: f64, capacity_l: f64) -> Self {
        Self {
            capacity_l,
            stage: Stage::Settling,
            at: t,
            start_fuel: fuel_l,
            best: fuel_l,
            last_rise: t,
            tries: 0,
            engine_switched: false,
            filled: false,
        }
    }
    /// Fuel has been seen coming in.
    pub fn filling(&self) -> bool {
        self.filled
    }
    pub fn step(&mut self, t: f64, speed: f64, fuel_l: f64, running: bool) -> RefuelStep {
        let mut out = RefuelStep::default();
        if speed.abs() > 0.5 {
            out.finished = Some(Err("加油时车辆移动"));
            return out;
        }
        if fuel_l > self.best + 0.2 {
            self.best = fuel_l;
            self.last_rise = t;
            self.filled = true;
        }
        let waited = t - self.at;
        let full = self.capacity_l > 0.0 && self.best >= FULL_SHARE * self.capacity_l;
        match self.stage {
            Stage::Settling if speed.abs() >= 0.1 => self.at = t,
            Stage::Settling if waited >= SETTLE_S => {
                self.stage = Stage::Holding;
                self.tries = 1;
                self.at = t;
                self.last_rise = t;
                out.hold = true;
            }
            Stage::Settling => {}
            Stage::Holding => {
                let quiet = t - self.last_rise;
                if self.filled && (full || quiet >= FULL_S || waited >= LIMIT_S) {
                    out.finished = Some(Ok(self.best - self.start_fuel));
                } else if !self.filled && quiet >= ANSWER_S {
                    if self.tries >= MAX_TRIES {
                        out.finished = Some(Err("加油站没有开始加油"));
                    } else if running && !self.engine_switched {
                        out.engine = true;
                        self.engine_switched = true;
                        self.stage = Stage::EngineOff;
                        self.at = t;
                    } else {
                        self.stage = Stage::Pause;
                        self.at = t;
                    }
                } else {
                    out.hold = true;
                }
            }
            Stage::Pause | Stage::EngineOff => {
                let wait = if self.stage == Stage::Pause {
                    PAUSE_S
                } else {
                    ENGINE_OFF_S
                };
                if waited >= wait {
                    self.stage = Stage::Holding;
                    self.tries += 1;
                    self.at = t;
                    self.last_rise = t;
                    out.hold = true;
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn low_fuel_is_a_fifth_of_a_known_tank() {
        assert!(wants_fuel(79.0, 400.0));
        assert!(!wants_fuel(81.0, 400.0));
        assert!(!wants_fuel(10.0, 0.0), "an unknown tank never turns off");
    }
    /// Runs the machine against a pump that fills at 35 L/s while the
    /// button is held, once `after` presses have been made; the litres and
    /// how many times the button was taken up.
    fn run(r: &mut Refuel, mut fuel: f64, capacity: f64, works_from_try: u8) -> (f64, u8, bool) {
        let (mut t, mut tries, mut held, mut engine) = (0.0, 0, false, false);
        loop {
            t += 0.05;
            let step = r.step(t, 0.0, fuel, false);
            if step.hold && !held {
                tries += 1;
            }
            held = step.hold;
            engine |= step.engine;
            if held && tries >= works_from_try {
                fuel = (fuel + 35.0 * 0.05).min(capacity);
            }
            if let Some(done) = step.finished {
                return (done.unwrap_or(-1.0), tries, engine);
            }
            assert!(t < 300.0);
        }
    }
    #[test]
    fn refuel_holds_the_button_until_the_tank_is_full() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        assert!(!r.step(0.5, 0.0, 60.0, true).hold);
        let (added, tries, _) = run(&mut r, 60.0, 400.0, 1);
        assert_eq!(tries, 1, "held once, not let go");
        assert!(added >= 0.98 * 400.0 - 60.0 - 2.0, "added {added}");
    }
    #[test]
    fn an_unknown_tank_is_filled_until_the_fuel_stops_rising() {
        let mut r = Refuel::new(0.0, 60.0, 0.0);
        let (added, tries, _) = run(&mut r, 60.0, 400.0, 1);
        assert_eq!(tries, 1);
        assert!((added - 340.0).abs() < 1.0, "added {added}");
    }
    #[test]
    fn refuel_tries_again_with_the_engine_off_then_gives_up() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        assert!(r.step(1.0, 0.0, 60.0, true).hold);
        assert!(r.step(4.9, 0.0, 60.0, true).hold);
        let off = r.step(5.1, 0.0, 60.0, true);
        assert!(off.engine && !off.hold, "let go and switch the engine off");
        assert!(!r.step(6.0, 0.0, 60.0, false).hold);
        assert!(r.step(7.2, 0.0, 60.0, false).hold, "second try");
        assert!(!r.step(11.3, 0.0, 60.0, false).hold);
        assert!(r.step(12.4, 0.0, 60.0, false).hold, "third try");
        assert_eq!(
            r.step(16.5, 0.0, 60.0, false).finished,
            Some(Err("加油站没有开始加油"))
        );
    }
    #[test]
    fn a_pump_that_answers_on_the_second_try_fills_the_tank() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        let (added, tries, engine) = run(&mut r, 60.0, 400.0, 2);
        assert_eq!(tries, 2);
        assert!(!engine, "the engine was already off");
        assert!(added > 320.0);
    }
    #[test]
    fn refuel_stops_if_the_truck_moves() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        assert!(
            r.step(0.3, 1.0, 60.0, true)
                .finished
                .is_some_and(|f| f.is_err())
        );
    }
}
