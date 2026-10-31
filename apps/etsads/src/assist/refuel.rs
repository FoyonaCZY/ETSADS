//! Refuelling on the way. Below a fifth of the tank the route is turned
//! through a fuel station the GPS route passes (`Navigator::plan_refuel`),
//! the truck stops beside a pump, the game is asked to refuel (the `activate`
//! input, Enter by default) and the drive continues.
//!
//! In the game the engine stops while the tank fills, about 35 L/s (live:
//! 161 → 380 L in 6.5 s, the driver had stopped with the cab level with the
//! pump); afterwards the stall restart starts it again.

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

/// Stopped beside the pump: settle, ask the game to refuel, wait while the
/// tank fills. If nothing happens with the engine running it is switched off
/// and asked again (the game may want it off). A tank that stops filling
/// well short of full is asked again: one press may fill only while held.
pub struct Refuel {
    capacity_l: f64,
    top_ups: u8,
    stage: Stage,
    /// When the stage began (s).
    at: f64,
    start_fuel: f64,
    best: f64,
    last_rise: f64,
    presses: u8,
    engine_switched: bool,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    Settling,
    Waiting,
    EngineOff,
    Filling,
}
#[derive(Debug, Default, PartialEq)]
pub struct RefuelStep {
    pub activate: bool,
    pub engine: bool,
    /// Litres added when the tank has stopped filling, or why it did not.
    pub finished: Option<Result<f64, &'static str>>,
}
/// Standing still this long before asking (s).
const SETTLE_S: f64 = 1.0;
/// No fuel this long after asking: ask again, or switch the engine off (s).
const ANSWER_S: f64 = 3.0;
/// The engine is given this long to stop before asking again (s).
const ENGINE_OFF_S: f64 = 2.0;
/// Asked this often without fuel: give up.
const MAX_PRESSES: u8 = 3;
/// The tank has stopped filling when it rose no more for this long (s).
const FULL_S: f64 = 2.0;
/// However long the game takes, a stop ends after this (s).
const LIMIT_S: f64 = 180.0;
/// Stopped filling below this share of a known tank: asked again, at most
/// `MAX_TOP_UPS` times.
const FULL_SHARE: f64 = 0.95;
const MAX_TOP_UPS: u8 = 12;
impl Refuel {
    /// `capacity_l` 0 when unknown.
    pub fn new(t: f64, fuel_l: f64, capacity_l: f64) -> Self {
        Self {
            capacity_l,
            top_ups: 0,
            stage: Stage::Settling,
            at: t,
            start_fuel: fuel_l,
            best: fuel_l,
            last_rise: t,
            presses: 0,
            engine_switched: false,
        }
    }
    pub fn filling(&self) -> bool {
        self.stage == Stage::Filling
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
            if self.stage != Stage::Filling {
                self.stage = Stage::Filling;
                self.at = t;
            }
        }
        let waited = t - self.at;
        match self.stage {
            Stage::Settling if speed.abs() < 0.1 && waited >= SETTLE_S => {
                out.activate = true;
                self.presses = 1;
                self.stage = Stage::Waiting;
                self.at = t;
            }
            Stage::Settling if speed.abs() >= 0.1 => self.at = t,
            Stage::Waiting if waited >= ANSWER_S => {
                if running && !self.engine_switched {
                    out.engine = true;
                    self.engine_switched = true;
                    self.stage = Stage::EngineOff;
                    self.at = t;
                } else if self.presses < MAX_PRESSES {
                    out.activate = true;
                    self.presses += 1;
                    self.at = t;
                } else {
                    out.finished = Some(Err("加油站没有开始加油"));
                }
            }
            Stage::EngineOff if waited >= ENGINE_OFF_S => {
                out.activate = true;
                self.presses += 1;
                self.stage = Stage::Waiting;
                self.at = t;
            }
            Stage::Filling if t - self.last_rise >= FULL_S || waited >= LIMIT_S => {
                if waited < LIMIT_S
                    && self.best < FULL_SHARE * self.capacity_l
                    && self.top_ups < MAX_TOP_UPS
                {
                    out.activate = true;
                    self.top_ups += 1;
                    self.last_rise = t;
                } else {
                    out.finished = Some(Ok(self.best - self.start_fuel));
                }
            }
            _ => {}
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
    #[test]
    fn refuel_asks_once_settled_and_finishes_when_the_tank_stops_filling() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        assert!(!r.step(0.5, 0.0, 60.0, true).activate);
        assert!(r.step(1.0, 0.0, 60.0, true).activate);
        assert!(!r.step(1.5, 0.0, 60.0, true).activate);
        // The game stops the engine and fills.
        let mut fuel = 60.0;
        let mut t = 1.5;
        while fuel < 400.0 {
            t += 0.1;
            fuel += 3.5;
            assert_eq!(r.step(t, 0.0, fuel, false).finished, None);
        }
        assert_eq!(r.step(t + 1.0, 0.0, 400.0, false).finished, None);
        let done = r.step(t + 2.1, 0.0, 400.0, false).finished;
        assert!(matches!(done, Some(Ok(added)) if (added - 340.0).abs() < 4.0));
    }
    #[test]
    fn refuel_switches_the_engine_off_then_gives_up_after_three_asks() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        assert!(r.step(1.0, 0.0, 60.0, true).activate);
        let off = r.step(4.0, 0.0, 60.0, true);
        assert!(off.engine && !off.activate);
        assert!(r.step(6.0, 0.0, 60.0, false).activate);
        assert!(r.step(9.0, 0.0, 60.0, false).activate);
        assert_eq!(
            r.step(12.0, 0.0, 60.0, false).finished,
            Some(Err("加油站没有开始加油"))
        );
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
    #[test]
    fn a_tank_that_fills_only_a_little_per_press_is_asked_again() {
        let mut r = Refuel::new(0.0, 60.0, 400.0);
        let (mut fuel, mut t, mut presses) = (60.0, 0.0, 0);
        let mut rise = 0.0_f64;
        let finished = loop {
            t += 0.1;
            if rise > 0.0 {
                fuel += 3.5;
                rise -= 3.5;
            }
            let step = r.step(t, 0.0, fuel, false);
            if step.activate {
                presses += 1;
                rise = 35.0;
            }
            if let Some(done) = step.finished {
                break done;
            }
            assert!(t < 120.0);
        };
        assert_eq!(presses, 10, "until 95% of the tank");
        assert!(fuel >= 380.0 && finished.is_ok());
    }
}
