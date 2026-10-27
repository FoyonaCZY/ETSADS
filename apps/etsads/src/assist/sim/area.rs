//! The drivable area as the exported map gives it: every lane centre of the
//! region, whatever may use it. A point further than half a lane from all of
//! them is off the road (a kerb, a pavement, a verge): the map has no kerb
//! geometry, so this is how far a trailer's wheels run over one.
use etsads_map::importer::ExportLane;
use etsads_msgs::Point3;
use std::collections::HashMap;

const CELL_M: f64 = 25.0;
/// Half a lane (m): beyond this from every lane centre a wheel is off it.
pub const HALF_LANE_M: f64 = 1.75;

#[derive(Default)]
pub struct Area {
    lanes: Vec<Vec<Point3>>,
    /// Segments (lane, index of its first point) by grid cell.
    cells: HashMap<(i64, i64), Vec<(usize, usize)>>,
    loaded_at: Option<Point3>,
}
impl Area {
    /// Whether the lanes around `center` need loading (first time, or 600 m
    /// on from where they were loaded).
    pub fn needs_lanes(&self, center: Point3) -> bool {
        self.loaded_at
            .is_none_or(|at| at.distance_xy(center) > 600.0)
    }
    pub fn load(&mut self, center: Point3, lanes: &[ExportLane]) {
        self.lanes.clear();
        self.cells.clear();
        self.loaded_at = Some(center);
        for lane in lanes.iter().filter(|l| l.center.len() >= 2) {
            let index = self.lanes.len();
            for (i, w) in lane.center.windows(2).enumerate() {
                let mid = ((w[0].x + w[1].x) / 2.0, (w[0].y + w[1].y) / 2.0);
                let cell = (
                    (mid.0 / CELL_M).floor() as i64,
                    (mid.1 / CELL_M).floor() as i64,
                );
                self.cells.entry(cell).or_default().push((index, i));
            }
            self.lanes.push(lane.center.clone());
        }
    }
    /// Distance (m) from (x, y) to the nearest lane centre, up to one cell
    /// away; infinite where none is that near.
    pub fn clearance(&self, x: f64, y: f64) -> f64 {
        let (cx, cy) = ((x / CELL_M).floor() as i64, (y / CELL_M).floor() as i64);
        let mut best = f64::INFINITY;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &(lane, i) in self.cells.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                    let (a, b) = (self.lanes[lane][i], self.lanes[lane][i + 1]);
                    let (ux, uy) = (b.x - a.x, b.y - a.y);
                    let length2 = ux * ux + uy * uy;
                    let t = if length2 > 1e-12 {
                        (((x - a.x) * ux + (y - a.y) * uy) / length2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    best = best.min((a.x + t * ux - x).hypot(a.y + t * uy - y));
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clearance_is_the_distance_to_the_nearest_lane_centre() {
        let lane = |y: f64| ExportLane {
            id: format!("{y}"),
            center: (0..=10)
                .map(|i| Point3 {
                    x: i as f64 * 10.0,
                    y,
                    z: 0.0,
                })
                .collect(),
            ..Default::default()
        };
        let mut area = Area::default();
        area.load(Point3::default(), &[lane(0.0), lane(3.5)]);
        assert!((area.clearance(50.0, 1.0) - 1.0).abs() < 1e-9);
        assert!((area.clearance(50.0, 6.0) - 2.5).abs() < 1e-9);
        assert!((area.clearance(105.0, 0.0) - 5.0).abs() < 1e-9);
        assert!(area.clearance(500.0, 0.0).is_infinite());
        assert!(!area.needs_lanes(Point3::default()));
    }
}
