//! Ferries and train transports on the route. The GPS route leaves the map
//! at a port; the route is driven on to the port's activation point (the
//! map's ferry item, past the end of the port's lanes), where the game is
//! asked for the crossing with the `activate` button (Enter by default).
//! The driver confirms the crossing in the game's menu; the truck appears at
//! the far port, and the route is prepared afresh from there.
//!
//! Live (manual crossings Naantali → Kapellskär and Gedser → Rostock): the
//! truck stopped with its own position within 2.2 m of the departure port's
//! point, and appeared on the destination port's point.

use anyhow::{Context, ensure};
use etsads_msgs::Point3;
use serde::Deserialize;
use std::path::Path;

/// Written into the runtime map by the exporter (`ferries`).
pub const FILE: &str = "ferries.json";
/// The SDK pulse asking for the ferry menu.
pub const ACTIVATE_PULSE: usize = etsads_game::input::ACTIVATE_PULSE;

#[derive(Clone, Debug, Deserialize)]
pub struct FerryPoint {
    /// The port's name in the game (`/def/ferry.sii`).
    pub port: String,
    /// A train transport rather than a ship.
    #[serde(default)]
    pub train: bool,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl FerryPoint {
    pub fn position(&self) -> Point3 {
        Point3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}
#[derive(Deserialize)]
struct File {
    schema_version: u32,
    ferries: Vec<FerryPoint>,
}
pub struct Ferries(Vec<FerryPoint>);
impl Ferries {
    /// `None` for a map generated before ferries were exported.
    pub fn load(map: &Path) -> anyhow::Result<Option<Self>> {
        let path = map.join(FILE);
        if !path.is_file() {
            return Ok(None);
        }
        let bytes = std::fs::read(&path).with_context(|| format!("读取 {}", path.display()))?;
        let file: File = serde_json::from_slice(&bytes).context("渡轮数据无法解析")?;
        ensure!(file.schema_version == 1, "渡轮数据版本不支持");
        Ok(Some(Self(file.ferries)))
    }
    #[cfg(test)]
    pub fn from_points(points: Vec<FerryPoint>) -> Self {
        Self(points)
    }
    /// The nearest point within `radius` (m) satisfying `fits`.
    pub fn nearest(
        &self,
        p: Point3,
        radius: f64,
        fits: impl Fn(&FerryPoint) -> bool,
    ) -> Option<&FerryPoint> {
        self.0
            .iter()
            .map(|f| (f, (f.x - p.x).hypot(f.y - p.y)))
            .filter(|(f, d)| *d <= radius && fits(f))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(f, _)| f)
    }
}

/// Stopped at the port: settle, press `activate` for the menu, wait while
/// the driver picks the crossing in it (the game is paused meanwhile), and
/// notice the truck has arrived somewhere else.
pub struct Crossing {
    pub port: String,
    /// The truck's own position where it asked for the crossing.
    pub from: Point3,
    stage: Stage,
    /// When the stage began (s).
    at: f64,
    presses: u8,
    /// The game has been paused (the menu, or the crossing itself) since the
    /// last press.
    menu_seen: bool,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    Settling,
    Asked,
    Arrived,
}
#[derive(Debug, Default, PartialEq)]
pub struct CrossingStep {
    /// Press `activate` once.
    pub press: bool,
    /// The truck has just been found at the far port.
    pub arrived: bool,
    pub failed: Option<&'static str>,
}
/// Standing still this long before asking (s).
const SETTLE_S: f64 = 1.0;
/// No menu this long after a press: press again (s); this often at most.
const MENU_S: f64 = 4.0;
const MAX_PRESSES: u8 = 3;
/// The menu closed with the truck still here this long: the crossing was
/// not taken (s).
const CANCELLED_S: f64 = 8.0;
/// Further than this (m) from where it asked, the truck has crossed.
const ARRIVED_M: f64 = 300.0;
/// A route from the far port not prepared within this (s): hand over.
const PLAN_S: f64 = 30.0;
impl Crossing {
    pub fn new(t: f64, port: String, from: Point3) -> Self {
        Self {
            port,
            from,
            stage: Stage::Settling,
            at: t,
            presses: 0,
            menu_seen: false,
        }
    }
    pub fn arrived(&self) -> bool {
        self.stage == Stage::Arrived
    }
    /// Waiting for the driver to confirm the crossing in the menu.
    pub fn asked(&self) -> bool {
        self.stage == Stage::Asked
    }
    /// A frame with the game paused: the menu is open, or the crossing is
    /// under way.
    pub fn paused(&mut self, t: f64) {
        self.menu_seen = true;
        if self.stage == Stage::Asked {
            self.at = t;
        }
    }
    /// A frame with the game running, the truck's own position `here`.
    pub fn step(&mut self, t: f64, speed: f64, here: Point3) -> CrossingStep {
        let mut out = CrossingStep::default();
        if self.stage != Stage::Arrived && here.distance_xy(self.from) > ARRIVED_M {
            self.stage = Stage::Arrived;
            self.at = t;
            out.arrived = true;
            return out;
        }
        let waited = t - self.at;
        match self.stage {
            Stage::Arrived => {
                if waited >= PLAN_S {
                    out.failed = Some("渡轮到岸后无法规划路线");
                }
            }
            _ if speed.abs() > 0.5 => out.failed = Some("在渡口时车辆移动"),
            Stage::Settling if speed.abs() >= 0.1 => self.at = t,
            Stage::Settling if waited >= SETTLE_S => {
                out.press = true;
                self.presses = 1;
                self.menu_seen = false;
                self.stage = Stage::Asked;
                self.at = t;
            }
            Stage::Settling => {}
            Stage::Asked if self.menu_seen && waited >= CANCELLED_S => {
                out.failed = Some("渡轮没有出发");
            }
            Stage::Asked if !self.menu_seen && waited >= MENU_S => {
                if self.presses >= MAX_PRESSES {
                    out.failed = Some("渡轮界面没有出现");
                } else {
                    out.press = true;
                    self.presses += 1;
                    self.at = t;
                }
            }
            Stage::Asked => {}
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn at(x: f64) -> Point3 {
        Point3 { x, y: 0.0, z: 0.0 }
    }
    #[test]
    fn crossing_asks_once_settled_waits_through_the_menu_and_notices_arrival() {
        let mut c = Crossing::new(0.0, "naantali".into(), at(0.0));
        assert!(!c.step(0.5, 0.0, at(0.0)).press);
        assert!(c.step(1.0, 0.0, at(0.0)).press);
        // The driver takes a while in the menu: no further press.
        for k in 0..200 {
            c.paused(1.1 + k as f64 * 0.1);
        }
        assert_eq!(c.step(21.2, 0.0, at(0.0)), CrossingStep::default());
        let arrived = c.step(23.0, 0.0, at(8600.0));
        assert!(arrived.arrived && c.arrived());
        assert_eq!(c.step(30.0, 0.0, at(8600.0)), CrossingStep::default());
        assert_eq!(
            c.step(53.1, 0.0, at(8600.0)).failed,
            Some("渡轮到岸后无法规划路线")
        );
    }
    #[test]
    fn no_menu_is_asked_for_three_times_then_handed_over() {
        let mut c = Crossing::new(0.0, "gedser".into(), at(0.0));
        assert!(c.step(1.0, 0.0, at(0.0)).press);
        assert!(c.step(5.0, 0.0, at(0.0)).press);
        assert!(c.step(9.0, 0.0, at(0.0)).press);
        assert_eq!(c.step(13.0, 0.0, at(0.0)).failed, Some("渡轮界面没有出现"));
    }
    #[test]
    fn a_menu_closed_without_crossing_hands_over() {
        let mut c = Crossing::new(0.0, "gedser".into(), at(0.0));
        assert!(c.step(1.0, 0.0, at(0.0)).press);
        c.paused(2.0);
        assert_eq!(c.step(9.0, 0.0, at(0.0)).failed, None);
        assert_eq!(c.step(10.1, 0.0, at(0.0)).failed, Some("渡轮没有出发"));
    }
    #[test]
    fn the_nearest_fitting_point_is_found() {
        let point = |port: &str, x: f64| FerryPoint {
            port: port.into(),
            train: false,
            x,
            y: 0.0,
            z: 0.0,
        };
        let ferries = Ferries::from_points(vec![point("a", 30.0), point("b", 10.0)]);
        assert_eq!(ferries.nearest(at(0.0), 50.0, |_| true).unwrap().port, "b");
        assert_eq!(
            ferries
                .nearest(at(0.0), 50.0, |f| f.port == "a")
                .unwrap()
                .port,
            "a"
        );
        assert!(ferries.nearest(at(0.0), 5.0, |_| true).is_none());
    }
}
