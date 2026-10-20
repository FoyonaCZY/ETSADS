//! Lane graph, spatial projection, topological matching and route search.
use etsads_msgs::{Point3, Pose2, frames::wrap_angle};
use rstar::{AABB, RTree, RTreeObject};
use serde::{Deserialize, Serialize};
use std::collections::{BinaryHeap, HashMap, HashSet};
pub mod annotations;
pub mod corridor;
pub mod importer;
pub mod sector;
pub mod traffic;

#[derive(Debug, thiserror::Error)]
pub enum MapError {
    #[error("invalid map: {0}")]
    Invalid(String),
    #[error("lane {0} not found")]
    Missing(u64),
    #[error("no connected route")]
    NoRoute,
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Lane {
    pub id: u64,
    pub center: Vec<Point3>,
    pub width: f64,
    pub speed_limit: f64,
    #[serde(default)]
    pub successors: Vec<u64>,
    #[serde(default)]
    pub predecessors: Vec<u64>,
    pub left: Option<u64>,
    pub right: Option<u64>,
    #[serde(default)]
    pub can_change_left: bool,
    #[serde(default)]
    pub can_change_right: bool,
    #[serde(default)]
    pub toll_cost: f64,
    #[serde(default)]
    pub difficulty_cost: f64,
}

/// Minimum horizontal chord used to estimate direction. Actual exported
/// Float32 world coordinates can leave 2–9 mm endpoint segments whose angle
/// is dominated by quantization. Coordinates and arc lengths remain unchanged.
pub const MIN_TANGENT_BASELINE_M: f64 = 0.25;

/// Direction of a segment using only this polyline. Ordinary segments retain
/// their exact direction; short ones expand to nearby vertices until their
/// chord has a useful baseline. This never borrows a successor/neighbor lane's
/// direction, which could belong to the opposite branch at a junction.
pub fn polyline_tangent(center: &[Point3], segment: usize) -> Result<f64, MapError> {
    let invalid = || MapError::Invalid("insufficient or invalid polyline tangent baseline".into());
    let end = segment.checked_add(1).ok_or_else(invalid)?;
    let (Some(a), Some(b)) = (center.get(segment), center.get(end)) else {
        return Err(invalid());
    };
    let length = a.distance_xy(*b);
    if !a.valid() || !b.valid() || !length.is_finite() || length < 1e-6 {
        return Err(invalid());
    }
    let (mut left, mut right) = (segment, end);
    let (mut before, mut after) = (length / 2.0, length / 2.0);
    while center[left].distance_xy(center[right]) < MIN_TANGENT_BASELINE_M {
        if left == 0 && right == center.len() - 1 {
            return Err(invalid());
        }
        // Grow the side with less existing arc support, unless at an endpoint.
        // A deterministic tie prefers earlier points of the same directed lane.
        if left > 0 && (right == center.len() - 1 || before <= after) {
            let next = center[left - 1];
            let step = next.distance_xy(center[left]);
            if !next.valid() || !step.is_finite() || step < 1e-6 {
                return Err(invalid());
            }
            if next.distance_xy(center[right]) >= MIN_TANGENT_BASELINE_M {
                return extension_tangent(center[left], next, center[right], true);
            }
            before += step;
            left -= 1;
        } else {
            let next = center[right + 1];
            let step = center[right].distance_xy(next);
            if !next.valid() || !step.is_finite() || step < 1e-6 {
                return Err(invalid());
            }
            if next.distance_xy(center[left]) >= MIN_TANGENT_BASELINE_M {
                return extension_tangent(center[right], next, center[left], false);
            }
            after += step;
            right += 1;
        }
    }
    if !center[left].distance_xy(center[right]).is_finite() {
        return Err(invalid());
    }
    Ok((center[right].y - center[left].y).atan2(center[right].x - center[left].x))
}
fn extension_tangent(
    anchor: Point3,
    next: Point3,
    opposite: Point3,
    left: bool,
) -> Result<f64, MapError> {
    // Use only the portion of the supporting segment required to reach the
    // baseline. Taking its entire (possibly long) edge would over-smooth bends.
    let (dx, dy) = (next.x - anchor.x, next.y - anchor.y);
    let (rx, ry) = (anchor.x - opposite.x, anchor.y - opposite.y);
    let a = dx * dx + dy * dy;
    let b = 2.0 * (rx * dx + ry * dy);
    let c = rx * rx + ry * ry - MIN_TANGENT_BASELINE_M.powi(2);
    let t = (-b + (b * b - 4.0 * a * c).max(0.0).sqrt()) / (2.0 * a);
    if !t.is_finite() || !(0.0..=1.0 + 1e-9).contains(&t) {
        return Err(MapError::Invalid(
            "invalid tangent baseline interpolation".into(),
        ));
    }
    let sign = if left { -1.0 } else { 1.0 };
    Ok((sign * (ry + t * dy)).atan2(sign * (rx + t * dx)))
}
impl Lane {
    pub fn length(&self) -> f64 {
        self.center.windows(2).map(|p| p[0].distance_xy(p[1])).sum()
    }
    pub fn sample(&self, s: f64) -> Option<(Point3, f64)> {
        if !s.is_finite() || self.center.len() < 2 {
            return None;
        }
        let mut remain = s.max(0.0);
        for (i, w) in self.center.windows(2).enumerate() {
            let len = w[0].distance_xy(w[1]);
            if !len.is_finite() || len < 1e-6 {
                return None;
            }
            if remain <= len || i == self.center.len() - 2 {
                let a = (remain / len).clamp(0.0, 1.0);
                return Some((
                    Point3 {
                        x: w[0].x + a * (w[1].x - w[0].x),
                        y: w[0].y + a * (w[1].y - w[0].y),
                        z: w[0].z + a * (w[1].z - w[0].z),
                    },
                    polyline_tangent(&self.center, i).ok()?,
                ));
            }
            remain -= len;
        }
        None
    }
}
#[derive(Clone, Copy, Debug)]
struct Segment {
    lane: u64,
    envelope: AABB<[f64; 2]>,
}
impl RTreeObject for Segment {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Projection {
    pub lane_id: u64,
    pub s: f64,
    pub d: f64,
    pub heading: f64,
    pub z: f64,
    pub distance: f64,
}
/// Geometry-only projection shared by qualified lane graphs and read-only
/// source diagnostics. It supplies no width, speed, or driving permission.
pub fn project_polyline(
    lane_id: u64,
    center: &[Point3],
    pos: Point3,
) -> Result<(Projection, Point3), MapError> {
    if !pos.valid() {
        return Err(MapError::Invalid("nonfinite query".into()));
    }
    let mut best: Option<(Projection, Point3, usize)> = None;
    let mut s = 0.0;
    for (segment, p) in center.windows(2).enumerate() {
        let dx = p[1].x - p[0].x;
        let dy = p[1].y - p[0].y;
        let len = dx.hypot(dy);
        if !p[0].valid() || !p[1].valid() || len < 1e-6 {
            return Err(MapError::Invalid("invalid projection centerline".into()));
        }
        let t = (((pos.x - p[0].x) * dx + (pos.y - p[0].y) * dy) / (len * len)).clamp(0.0, 1.0);
        let rx = pos.x - p[0].x - t * dx;
        let ry = pos.y - p[0].y - t * dy;
        let distance = rx.hypot(ry);
        let point = Point3 {
            x: p[0].x + t * dx,
            y: p[0].y + t * dy,
            z: p[0].z + t * (p[1].z - p[0].z),
        };
        let projection = Projection {
            lane_id,
            s: s + t * len,
            d: (dx * ry - dy * rx) / len,
            heading: dy.atan2(dx),
            z: point.z,
            distance,
        };
        if best
            .as_ref()
            .is_none_or(|(old, _, _)| distance < old.distance)
        {
            best = Some((projection, point, segment));
        }
        s += len;
    }
    let (mut projection, point, segment) =
        best.ok_or_else(|| MapError::Invalid("empty lane".into()))?;
    projection.heading = polyline_tangent(center, segment)?;
    // Keep the exact closest point/distance/station, but give signed offset the
    // same stable normal used by matching and reference sampling.
    projection.d =
        projection.heading.cos() * (pos.y - point.y) - projection.heading.sin() * (pos.x - point.x);
    Ok((projection, point))
}
#[derive(Clone, Debug)]
pub struct LaneGraph {
    pub lanes: HashMap<u64, Lane>,
    index: RTree<Segment>,
}
impl LaneGraph {
    pub fn new(lanes: Vec<Lane>) -> Result<Self, MapError> {
        let mut map = HashMap::new();
        let mut segments = Vec::new();
        for lane in lanes {
            if lane.center.len() < 2
                || lane.center.len() > 1_000_000
                || !lane.center.iter().all(|p| p.valid())
                || lane
                    .center
                    .windows(2)
                    .any(|p| p[0].distance_xy(p[1]) < 1e-6)
                || !lane.width.is_finite()
                || !(0.5..=30.0).contains(&lane.width)
                || !lane.speed_limit.is_finite()
                || !(0.1..=60.0).contains(&lane.speed_limit)
                || !lane.toll_cost.is_finite()
                || lane.toll_cost < 0.0
                || !lane.difficulty_cost.is_finite()
                || lane.difficulty_cost < 0.0
            {
                return Err(MapError::Invalid(format!(
                    "lane {} invalid geometry or cost",
                    lane.id
                )));
            }
            if map.contains_key(&lane.id) {
                return Err(MapError::Invalid("duplicate lane id".into()));
            }
            for (segment, p) in lane.center.windows(2).enumerate() {
                if p[0].distance_xy(p[1]) < MIN_TANGENT_BASELINE_M {
                    polyline_tangent(&lane.center, segment).map_err(|error| {
                        MapError::Invalid(format!("lane {} segment {segment}: {error}", lane.id))
                    })?;
                }
                segments.push(Segment {
                    lane: lane.id,
                    envelope: AABB::from_corners(
                        [p[0].x.min(p[1].x), p[0].y.min(p[1].y)],
                        [p[0].x.max(p[1].x), p[0].y.max(p[1].y)],
                    ),
                });
            }
            map.insert(lane.id, lane);
        }
        if map.is_empty() {
            return Err(MapError::Invalid("empty graph".into()));
        }
        for l in map.values() {
            if l.can_change_left && l.left.is_none()
                || l.can_change_right && l.right.is_none()
                || l.left == Some(l.id)
                || l.right == Some(l.id)
                || l.left.is_some() && l.left == l.right
            {
                return Err(MapError::Invalid(format!(
                    "lane {} has inconsistent lateral neighbors",
                    l.id
                )));
            }
            for id in l
                .successors
                .iter()
                .chain(l.predecessors.iter())
                .chain(l.left.iter())
                .chain(l.right.iter())
            {
                if !map.contains_key(id) {
                    return Err(MapError::Missing(*id));
                }
            }
        }
        Ok(Self {
            lanes: map,
            index: RTree::bulk_load(segments),
        })
    }
    pub fn from_json(json: &str) -> Result<Self, MapError> {
        Self::new(serde_json::from_str(json)?)
    }
    pub fn to_json(&self) -> Result<String, MapError> {
        let mut lanes: Vec<_> = self.lanes.values().collect();
        lanes.sort_by_key(|l| l.id);
        Ok(serde_json::to_string_pretty(&lanes)?)
    }
    pub fn project(&self, lane: u64, pos: Point3) -> Result<Projection, MapError> {
        if !pos.valid() {
            return Err(MapError::Invalid("nonfinite query".into()));
        }
        let l = self.lanes.get(&lane).ok_or(MapError::Missing(lane))?;
        project_polyline(lane, &l.center, pos).map(|(projection, _)| projection)
    }
    pub fn nearest_lanes(&self, pos: Point3, radius: f64) -> Vec<Projection> {
        if !pos.valid() || !radius.is_finite() || radius <= 0.0 || radius > 10_000.0 {
            return Vec::new();
        }
        let envelope = AABB::from_corners(
            [pos.x - radius, pos.y - radius],
            [pos.x + radius, pos.y + radius],
        );
        let mut seen = HashSet::new();
        let mut result = Vec::new();
        for segment in self.index.locate_in_envelope_intersecting(&envelope) {
            if seen.insert(segment.lane)
                && let Ok(p) = self.project(segment.lane, pos)
                && p.distance <= radius
            {
                result.push(p);
            }
        }
        result.sort_by(|a, b| {
            a.distance
                .total_cmp(&b.distance)
                .then(a.lane_id.cmp(&b.lane_id))
        });
        result
    }
    /// Connect independently exported road/prefab endpoints; report unmatched exits.
    pub fn stitch(&mut self, max_distance: f64, max_angle: f64) -> Result<Vec<u64>, MapError> {
        if !max_distance.is_finite()
            || max_distance <= 0.0
            || !max_angle.is_finite()
            || max_angle <= 0.0
            || max_angle > std::f64::consts::FRAC_PI_2
        {
            return Err(MapError::Invalid("invalid stitch tolerances".into()));
        }
        let mut additions = Vec::new();
        for a in self.lanes.values() {
            let (end, ha) = a.sample(a.length()).unwrap();
            for b in self.lanes.values() {
                if a.id == b.id {
                    continue;
                }
                let (start, hb) = b.sample(0.0).unwrap();
                if end.distance_xy(start) < max_distance
                    && (end.z - start.z).abs() < max_distance
                    && wrap_angle(ha - hb).abs() < max_angle
                    && !a.successors.contains(&b.id)
                {
                    additions.push((a.id, b.id));
                }
            }
        }
        for (a, b) in additions {
            self.lanes.get_mut(&a).unwrap().successors.push(b);
            self.lanes.get_mut(&b).unwrap().predecessors.push(a);
        }
        let mut unmatched: Vec<_> = self
            .lanes
            .values()
            .filter(|l| l.successors.is_empty())
            .map(|l| l.id)
            .collect();
        unmatched.sort_unstable();
        Ok(unmatched)
    }
    /// Dijkstra is A* with an admissible zero heuristic. This remains correct for
    /// discontinuous exported nodes and arbitrary nonnegative toll penalties.
    pub fn route(&self, start: u64, goal: u64, change_penalty: f64) -> Result<Vec<u64>, MapError> {
        if !change_penalty.is_finite() || change_penalty < 0.0 {
            return Err(MapError::Invalid("negative/nonfinite route penalty".into()));
        }
        if !self.lanes.contains_key(&start) {
            return Err(MapError::Missing(start));
        }
        if !self.lanes.contains_key(&goal) {
            return Err(MapError::Missing(goal));
        }
        let mut open = BinaryHeap::new();
        let mut costs = HashMap::new();
        let mut parent = HashMap::new();
        costs.insert(start, 0.0);
        open.push(Queue {
            cost: 0.0,
            id: start,
        });
        while let Some(q) = open.pop() {
            if q.cost > *costs.get(&q.id).unwrap_or(&f64::INFINITY) {
                continue;
            }
            if q.id == goal {
                let mut route = vec![goal];
                while *route.last().unwrap() != start {
                    route.push(parent[route.last().unwrap()]);
                }
                route.reverse();
                return Ok(route);
            }
            let l = &self.lanes[&q.id];
            let mut neighbors: Vec<_> = l.successors.iter().map(|id| (*id, 0.0)).collect();
            if l.can_change_left
                && let Some(id) = l.left
            {
                neighbors.push((id, change_penalty));
            }
            if l.can_change_right
                && let Some(id) = l.right
            {
                neighbors.push((id, change_penalty));
            }
            for (id, penalty) in neighbors {
                let n = &self.lanes[&id];
                let new =
                    q.cost + n.length() / n.speed_limit + n.toll_cost + n.difficulty_cost + penalty;
                if new < *costs.get(&id).unwrap_or(&f64::INFINITY) {
                    costs.insert(id, new);
                    parent.insert(id, q.id);
                    open.push(Queue { cost: new, id });
                }
            }
        }
        Err(MapError::NoRoute)
    }
    pub fn advance(&self, lane: u64, s: f64, distance: f64) -> Result<(u64, f64), MapError> {
        if !s.is_finite() || !distance.is_finite() || s < 0.0 || distance < 0.0 {
            return Err(MapError::Invalid("invalid advance distance".into()));
        }
        let mut id = lane;
        let mut progress = s + distance;
        for _ in 0..10_000 {
            let l = self.lanes.get(&id).ok_or(MapError::Missing(id))?;
            let length = l.length();
            if progress <= length {
                return Ok((id, progress));
            }
            match l.successors.first() {
                Some(next) => {
                    id = *next;
                    progress -= length;
                }
                None => return Ok((id, length)),
            }
        }
        Err(MapError::Invalid(
            "advance exceeds graph traversal budget".into(),
        ))
    }
}
#[derive(Clone, Copy, Debug)]
struct Queue {
    cost: f64,
    id: u64,
}
impl PartialEq for Queue {
    fn eq(&self, o: &Self) -> bool {
        self.id == o.id && self.cost == o.cost
    }
}
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Queue {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.cost
            .total_cmp(&self.cost)
            .then_with(|| o.id.cmp(&self.id))
    }
}

pub mod matcher {
    use super::*;
    #[derive(Clone, Copy, Debug, Serialize, Deserialize)]
    pub struct LaneState {
        pub lane_id: u64,
        pub s: f64,
        pub d: f64,
        pub heading_err: f64,
        pub confidence: f64,
    }
    #[derive(Clone, Debug, Default)]
    pub struct Matcher {
        history: std::collections::VecDeque<Observation>,
        last_time: Option<f64>,
    }
    #[derive(Clone, Debug)]
    struct Observation {
        time: f64,
        speed: f64,
        candidates: Vec<(Projection, f64)>,
    }
    impl Matcher {
        pub fn reset(&mut self) {
            self.history.clear();
            self.last_time = None;
        }
        /// Two-second sliding-window Viterbi recursion. Costs are normalized at
        /// each observation; old evidence cannot indefinitely lock lane identity.
        pub fn update(
            &mut self,
            graph: &LaneGraph,
            pose: Pose2,
            z: f64,
            speed: f64,
            time: f64,
        ) -> Option<LaneState> {
            if !pose.valid()
                || ![z, speed, time].iter().all(|v| v.is_finite())
                || speed.abs() > 100.0
                || time < 0.0
            {
                self.reset();
                return None;
            }
            let dt = self.last_time.map(|t| time - t).unwrap_or(0.0);
            if self.last_time.is_some() && (dt <= 0.0 || dt > 2.0) {
                self.reset();
            }
            let candidates = graph.nearest_lanes(
                Point3 {
                    x: pose.x,
                    y: pose.y,
                    z,
                },
                5.0,
            );
            let mut scores = Vec::new();
            for p in candidates {
                let heading = wrap_angle(p.heading - pose.yaw);
                if heading.abs() > std::f64::consts::PI / 6.0 || (p.z - z).abs() > 3.0 {
                    continue;
                }
                let emission = p.distance.powi(2) / (2.0 * 0.5_f64.powi(2))
                    + heading.powi(2) / (2.0 * 0.15_f64.powi(2))
                    + (p.z - z).powi(2) / 2.0;
                scores.push((p, emission));
            }
            if scores.is_empty() {
                self.reset();
                return None;
            }
            self.history.push_back(Observation {
                time,
                speed,
                candidates: scores,
            });
            while self
                .history
                .front()
                .is_some_and(|frame| time - frame.time > 2.0)
                || self.history.len() > 2000
            {
                self.history.pop_front();
            }
            let mut previous: Vec<(Projection, f64)> = Vec::new();
            let mut last = None;
            for observation in &self.history {
                let dt = last.map_or(0.0, |time| observation.time - time);
                let mut current = Vec::new();
                for &(p, emission) in &observation.candidates {
                    let transition = previous
                        .iter()
                        .map(|(prev, cost)| {
                            let Some(l) = graph.lanes.get(&prev.lane_id) else {
                                return cost + 50.0;
                            };
                            let (travel, penalty) = if prev.lane_id == p.lane_id {
                                (p.s - prev.s, 0.0)
                            } else if l.successors.contains(&p.lane_id) {
                                (l.length() - prev.s + p.s, 0.2)
                            } else if (l.left == Some(p.lane_id) && l.can_change_left)
                                || (l.right == Some(p.lane_id) && l.can_change_right)
                            {
                                (p.s - prev.s, 2.0)
                            } else {
                                (0.0, 40.0)
                            };
                            cost + penalty + ((travel - observation.speed * dt) / 3.0).powi(2)
                        })
                        .min_by(f64::total_cmp)
                        .unwrap_or(0.0);
                    current.push((p, emission + transition));
                }
                let minimum = current
                    .iter()
                    .map(|(_, cost)| *cost)
                    .min_by(f64::total_cmp)
                    .unwrap_or(0.0);
                for (_, cost) in &mut current {
                    *cost -= minimum;
                }
                previous = current;
                last = Some(observation.time);
            }
            let mut scores = previous;
            scores.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.lane_id.cmp(&b.0.lane_id)));
            let (best, min) = scores[0];
            let sum: f64 = scores.iter().map(|(_, c)| (-(c - min)).exp()).sum();
            let confidence = (1.0 / sum) * (-best.distance.powi(2) / 8.0).exp();
            self.last_time = Some(time);
            Some(LaneState {
                lane_id: best.lane_id,
                s: best.s,
                d: best.d,
                heading_err: wrap_angle(pose.yaw - best.heading),
                confidence,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lane(id: u64, y: f64, z: f64) -> Lane {
        Lane {
            id,
            center: vec![Point3 { x: 0.0, y, z }, Point3 { x: 100.0, y, z }],
            width: 3.7,
            speed_limit: 25.0,
            successors: vec![],
            predecessors: vec![],
            left: None,
            right: None,
            can_change_left: false,
            can_change_right: false,
            toll_cost: 0.0,
            difficulty_cost: 0.0,
        }
    }
    #[test]
    fn projection_large_origin_and_sign() {
        let mut l = lane(1, 0.0, 0.0);
        for p in &mut l.center {
            p.x += 200000.0;
        }
        let g = LaneGraph::new(vec![l]).unwrap();
        let p = g
            .project(
                1,
                Point3 {
                    x: 200025.0,
                    y: 2.0,
                    z: 0.0,
                },
            )
            .unwrap();
        assert!((p.s - 25.0).abs() < 1e-9);
        assert_eq!(p.d, 2.0);
    }
    #[test]
    fn stacked_roads_and_heading() {
        let g = LaneGraph::new(vec![lane(1, 0.0, 0.0), lane(2, 0.0, 8.0)]).unwrap();
        let mut m = matcher::Matcher::default();
        assert_eq!(
            m.update(
                &g,
                Pose2 {
                    x: 10.0,
                    y: 0.0,
                    yaw: 0.0
                },
                8.0,
                10.0,
                0.0
            )
            .unwrap()
            .lane_id,
            2
        );
        assert!(
            m.update(
                &g,
                Pose2 {
                    x: 11.0,
                    y: 0.0,
                    yaw: std::f64::consts::PI
                },
                8.0,
                10.0,
                0.1
            )
            .is_none()
        );
    }
    #[test]
    fn route_obeys_connections_and_costs() {
        let mut a = lane(1, 0.0, 0.0);
        a.successors = vec![2, 3];
        let mut b = lane(2, 4.0, 0.0);
        b.successors = vec![4];
        b.toll_cost = 100.0;
        let mut c = lane(3, 8.0, 0.0);
        c.successors = vec![4];
        let g = LaneGraph::new(vec![a, b, c, lane(4, 12.0, 0.0)]).unwrap();
        assert_eq!(g.route(1, 4, 5.0).unwrap(), vec![1, 3, 4]);
        assert!(g.route(4, 1, 5.0).is_err());
    }
    #[test]
    fn rejects_corrupt_map() {
        let mut l = lane(1, 0.0, 0.0);
        l.center[1].x = f64::NAN;
        assert!(LaneGraph::new(vec![l]).is_err());
        let mut l = lane(1, 0.0, 0.0);
        l.successors.push(999);
        assert!(LaneGraph::new(vec![l]).is_err());
    }
    #[test]
    fn graph_rejects_permission_without_a_distinct_neighbor() {
        let mut l = lane(1, 0.0, 0.0);
        l.can_change_left = true;
        assert!(LaneGraph::new(vec![l.clone()]).is_err());
        l.left = Some(1);
        assert!(LaneGraph::new(vec![l.clone()]).is_err());
        l.left = Some(2);
        l.right = Some(2);
        assert!(LaneGraph::new(vec![l, lane(2, 4.5, 0.0)]).is_err());
    }

    #[test]
    fn real_export_quantized_tail_keeps_match_and_reference_heading_continuous() {
        // Actual tail from junction-samples.json, owner 3890663093387395077,
        // prefab curve 1 -> 15. Its final 4 mm segment points west after world
        // Float32 quantization, while the approximately 1 m support points SW.
        // Width/speed below belong only to this synthetic matcher test graph;
        // the real export's unknown physical width is never changed.
        let mut source = lane(1, 0.0, 0.0);
        source.center = vec![
            Point3 {
                x: -38449.055,
                y: -47981.426,
                z: 15.082031,
            },
            Point3 {
                x: -38449.664,
                y: -47982.22,
                z: 15.082031,
            },
            Point3 {
                x: -38449.668,
                y: -47982.22,
                z: 15.082031,
            },
        ];
        let original = source.center.clone();
        let original_length = source.length();
        let target_heading = (-0.870_f64).atan2(-0.488);
        let naive_heading = (source.center[2].y - source.center[1].y)
            .atan2(source.center[2].x - source.center[1].x);
        assert!(
            wrap_angle(naive_heading - target_heading)
                .abs()
                .to_degrees()
                > 60.0
        );
        let (end, sampled_heading) = source.sample(original_length).unwrap();
        assert!(
            (wrap_angle(sampled_heading - target_heading))
                .abs()
                .to_degrees()
                < 10.0
        );
        assert_eq!(end, original[2]);
        let (projected, point) = project_polyline(source.id, &source.center, end).unwrap();
        assert_eq!(point, end);
        assert!((projected.s - original_length).abs() < 1e-9);
        assert_eq!(projected.heading, sampled_heading);
        let offset = Point3 {
            x: end.x - 0.02 * sampled_heading.sin(),
            y: end.y + 0.02 * sampled_heading.cos(),
            z: end.z,
        };
        let offset_projection = project_polyline(source.id, &source.center, offset)
            .unwrap()
            .0;
        assert!(offset_projection.d > 0.0);
        assert!(offset_projection.d <= offset_projection.distance + 1e-9);
        let graph = LaneGraph::new(vec![source]).unwrap();
        let matched = matcher::Matcher::default()
            .update(
                &graph,
                Pose2 {
                    x: end.x,
                    y: end.y,
                    yaw: target_heading,
                },
                end.z,
                1.0,
                0.0,
            )
            .unwrap();
        assert_eq!(matched.lane_id, 1);
        assert!(matched.heading_err.abs().to_degrees() < 10.0);
        assert_eq!(graph.lanes[&1].center, original);
        assert_eq!(graph.lanes[&1].length(), original_length);
    }

    #[test]
    fn effective_tangent_retains_curves_and_does_not_borrow_opposite_successor() {
        let mut a = lane(1, 0.0, 0.0);
        a.center = vec![
            Point3::default(),
            Point3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point3 {
                x: 1.0,
                y: 1.0,
                z: 0.0,
            },
            Point3 {
                x: 1.0,
                y: 1.004,
                z: 0.0,
            },
        ];
        a.successors = vec![2];
        let mut opposite = lane(2, 0.0, 0.0);
        opposite.center = vec![
            a.center[3],
            Point3 {
                x: 1.0,
                y: -2.0,
                z: 0.0,
            },
        ];
        opposite.predecessors = vec![1];
        let graph = LaneGraph::new(vec![a, opposite]).unwrap();
        let a = &graph.lanes[&1];
        assert_eq!(a.sample(0.5).unwrap().1, 0.0);
        assert_eq!(a.sample(1.5).unwrap().1, std::f64::consts::FRAC_PI_2);
        assert_eq!(a.sample(a.length()).unwrap().1, std::f64::consts::FRAC_PI_2);
        assert_eq!(
            graph.lanes[&2].sample(0.0).unwrap().1,
            -std::f64::consts::FRAC_PI_2
        );
        assert_eq!(a.successors, vec![2]);
    }

    #[test]
    fn tangent_extension_uses_only_needed_support_at_a_bend() {
        let p = [
            Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point3 {
                x: 1.0,
                y: 0.2,
                z: 0.0,
            },
        ];
        let heading = polyline_tangent(&p, 1).unwrap();
        // Baseline is the nearest 0.25 m chord: 0.15 m back along the first
        // edge, not a chord spanning its entire metre and flattening the turn.
        assert!((heading - 0.2_f64.atan2(0.15)).abs() < 1e-10);
        assert_eq!(polyline_tangent(&p, 0).unwrap(), 0.0);
    }

    #[test]
    fn public_tangent_combines_short_samples_but_refuses_insufficient_or_invalid_support() {
        let points: Vec<_> = (0..=5)
            .map(|i| Point3 {
                x: i as f64 * 0.1,
                y: 0.0,
                z: 0.0,
            })
            .collect();
        for segment in 0..5 {
            assert_eq!(polyline_tangent(&points, segment).unwrap(), 0.0);
        }
        assert!(polyline_tangent(&points, usize::MAX).is_err());
        assert!(polyline_tangent(&points, 5).is_err());
        assert!(polyline_tangent(&[], 0).is_err());
        let mut invalid = points.clone();
        invalid[0].x = f64::NAN;
        assert!(polyline_tangent(&invalid, 0).is_err());
        let short = &points[..3];
        assert!(polyline_tangent(short, 0).is_err());
        assert!(project_polyline(1, short, short[0]).is_err());
        let mut l = lane(1, 0.0, 0.0);
        l.center = short.to_vec();
        assert!(l.sample(0.0).is_none());
        let error = LaneGraph::new(vec![l]).unwrap_err().to_string();
        assert!(error.contains("tangent baseline"));
    }
}
