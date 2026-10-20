//! Obstacle-aware, bounded 8-connected Dijkstra potential. This is search
//! guidance, never a substitute for articulated continuous collision checks.
use crate::{
    PlanningError,
    geometry::{corners, point_in_polygon},
};
use etsads_msgs::{OrientedBox, Pose2};
use std::collections::BinaryHeap;
const MAX_CELLS: usize = 250_000;
#[derive(Clone, Copy)]
struct Entry {
    cost: f64,
    index: usize,
}
impl Eq for Entry {}
impl PartialEq for Entry {
    fn eq(&self, o: &Self) -> bool {
        self.cost == o.cost && self.index == o.index
    }
}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.cost
            .total_cmp(&self.cost)
            .then_with(|| o.index.cmp(&self.index))
    }
}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
pub(super) struct Potential {
    origin: (f64, f64),
    pub resolution: f64,
    width: usize,
    height: usize,
    blocked: Vec<bool>,
    distance: Vec<f64>,
    pub settled: usize,
    goal_offset: f64,
}
impl Potential {
    pub fn build(
        polygon: &[(f64, f64)],
        obstacles: &[OrientedBox],
        goal: Pose2,
        clearance: f64,
        cancellation: Option<&std::sync::atomic::AtomicBool>,
    ) -> Result<Self, PlanningError> {
        let mut min = (f64::INFINITY, f64::INFINITY);
        let mut max = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in polygon {
            min.0 = min.0.min(p.0);
            min.1 = min.1.min(p.1);
            max.0 = max.0.max(p.0);
            max.1 = max.1.max(p.1);
        }
        let extent = (max.0 - min.0, max.1 - min.1);
        if !clearance.is_finite()
            || clearance < 0.0
            || clearance > 20.0
            || !goal.valid()
            || !extent.0.is_finite()
            || !extent.1.is_finite()
            || extent.0 <= 0.0
            || extent.1 <= 0.0
            || extent.0 > 10000.0
            || extent.1 > 10000.0
        {
            return Err(PlanningError::Invalid("parking Dijkstra extent"));
        }
        let mut resolution = 0.5_f64.max((extent.0 * extent.1 / MAX_CELLS as f64).sqrt());
        let (width, height) = loop {
            let w = (extent.0 / resolution).ceil() as usize;
            let h = (extent.1 / resolution).ceil() as usize;
            if w.saturating_mul(h) <= MAX_CELLS {
                break (w.max(1), h.max(1));
            }
            resolution *= 1.01;
        };
        let count = width * height;
        let mut grid = Self {
            origin: min,
            resolution,
            width,
            height,
            blocked: vec![false; count],
            distance: vec![f64::INFINITY; count],
            settled: 0,
            goal_offset: 0.0,
        };
        for i in 0..count {
            if i % 256 == 0 {
                super::check_cancel(cancellation)?;
            }
            grid.blocked[i] = !point_in_polygon(grid.point(i), polygon);
        }
        for obstacle in obstacles {
            super::check_cancel(cancellation)?;
            let cs = corners(*obstacle);
            let margin = clearance + 0.3;
            let low = (
                cs.iter().map(|p| p.0).fold(f64::INFINITY, f64::min) - margin,
                cs.iter().map(|p| p.1).fold(f64::INFINITY, f64::min) - margin,
            );
            let high = (
                cs.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max) + margin,
                cs.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max) + margin,
            );
            let x0 = ((low.0 - min.0) / resolution).floor().max(0.0) as usize;
            let y0 = ((low.1 - min.1) / resolution).floor().max(0.0) as usize;
            let x1 = ((high.0 - min.0) / resolution).ceil().max(0.0) as usize;
            let y1 = ((high.1 - min.1) / resolution).ceil().max(0.0) as usize;
            for y in y0.min(height)..y1.saturating_add(1).min(height) {
                super::check_cancel(cancellation)?;
                for x in x0.min(width)..x1.saturating_add(1).min(width) {
                    let i = y * width + x;
                    let p = grid.point(i);
                    let dx = p.0 - obstacle.pose.x;
                    let dy = p.1 - obstacle.pose.y;
                    let local_x = dx * obstacle.pose.yaw.cos() + dy * obstacle.pose.yaw.sin();
                    let local_y = -dx * obstacle.pose.yaw.sin() + dy * obstacle.pose.yaw.cos();
                    if local_x.abs() <= obstacle.length / 2.0 + margin
                        && local_y.abs() <= obstacle.width / 2.0 + margin
                    {
                        grid.blocked[i] = true;
                    }
                }
            }
        }
        // Rasterization can close a narrow but continuously feasible pocket.
        // Leave unreachable queries explicit; Hybrid A* must not prune them.
        if let Some(goal_index) = grid.index(goal.x, goal.y).filter(|i| !grid.blocked[*i]) {
            let p = grid.point(goal_index);
            grid.goal_offset = (p.0 - goal.x).hypot(p.1 - goal.y);
            grid.distance[goal_index] = 0.0;
            let mut queue = BinaryHeap::from([Entry {
                cost: 0.0,
                index: goal_index,
            }]);
            let mut popped = 0_usize;
            while let Some(entry) = queue.pop() {
                if popped.is_multiple_of(256) {
                    super::check_cancel(cancellation)?;
                }
                popped += 1;
                if entry.cost > grid.distance[entry.index] + 1e-10 {
                    continue;
                }
                grid.settled += 1;
                let x = entry.index % width;
                let y = entry.index / width;
                for dx in -1_i32..=1 {
                    for dy in -1_i32..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;
                        if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                            continue;
                        }
                        let next = ny as usize * width + nx as usize;
                        if grid.blocked[next]
                            || (dx != 0
                                && dy != 0
                                && (grid.blocked[y * width + nx as usize]
                                    || grid.blocked[ny as usize * width + x]))
                        {
                            continue;
                        }
                        let cost = entry.cost
                            + resolution
                                * if dx != 0 && dy != 0 {
                                    std::f64::consts::SQRT_2
                                } else {
                                    1.0
                                };
                        if cost + 1e-10 < grid.distance[next] {
                            grid.distance[next] = cost;
                            queue.push(Entry { cost, index: next });
                        }
                    }
                }
            }
        }
        Ok(grid)
    }
    fn index(&self, x: f64, y: f64) -> Option<usize> {
        let ix = ((x - self.origin.0) / self.resolution).floor();
        let iy = ((y - self.origin.1) / self.resolution).floor();
        (ix >= 0.0 && iy >= 0.0 && ix < self.width as f64 && iy < self.height as f64)
            .then(|| iy as usize * self.width + ix as usize)
    }
    fn point(&self, index: usize) -> (f64, f64) {
        (
            self.origin.0 + (index % self.width) as f64 * self.resolution + self.resolution / 2.0,
            self.origin.1 + (index / self.width) as f64 * self.resolution + self.resolution / 2.0,
        )
    }
    pub fn query(&self, pose: Pose2) -> Option<f64> {
        let i = self.index(pose.x, pose.y)?;
        let d = self.distance[i];
        let p = self.point(i);
        d.is_finite()
            .then_some(d + (p.0 - pose.x).hypot(p.1 - pose.y) + self.goal_offset)
    }
    pub fn cells(&self) -> usize {
        self.distance.len()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn potential_detours_wall_and_does_not_cut_diagonal_corners() {
        let polygon = [(-12.0, -10.0), (12.0, -10.0), (12.0, 10.0), (-12.0, 10.0)];
        let start = Pose2 {
            x: -8.0,
            y: 0.0,
            yaw: 0.0,
        };
        let goal = Pose2 {
            x: 8.0,
            y: 0.0,
            yaw: 0.0,
        };
        let wall = OrientedBox {
            pose: Pose2 {
                x: 0.0,
                y: -3.0,
                yaw: 0.0,
            },
            length: 2.0,
            width: 14.0,
        };
        let free = Potential::build(&polygon, &[], goal, 0.0, None).unwrap();
        let blocked = Potential::build(&polygon, &[wall], goal, 0.0, None).unwrap();
        assert!(blocked.query(start).unwrap() > free.query(start).unwrap() + 2.0);
        let closed = Potential::build(
            &polygon,
            &[OrientedBox {
                width: 30.0,
                ..wall
            }],
            goal,
            0.0,
            None,
        )
        .unwrap();
        assert!(closed.query(start).is_none());
        assert!(closed.cells() <= MAX_CELLS);
    }
}
