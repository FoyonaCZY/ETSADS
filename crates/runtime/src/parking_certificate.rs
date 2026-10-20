//! Worker-built remaining-path sweep index. The controlling thread checks only
//! newly observed geometry near the sweep, with a bounded amount of work.
use etsads_control::TruckState;
use etsads_msgs::{EgoState, OrientedBox, VehicleGeometry, frames::wrap_angle};
use etsads_planning::geometry::{corners, footprints, swept_collision};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

const CELL: f64 = 4.0;
const MAX_REFERENCES: usize = 1_000_000;
const MAX_WORK: usize = 8192;
const MAX_OBJECTS: usize = 16384;
const BUDGET: Duration = Duration::from_millis(10);
#[derive(Clone, Copy)]
struct Sweep {
    from: OrientedBox,
    to: OrientedBox,
    bounds: [f64; 4],
    work: usize,
}
pub(crate) struct ParkingAdoptionProof {
    pub first: Option<TruckState>,
    sweeps: Vec<Sweep>,
    grid: BTreeMap<(i32, i32), Vec<usize>>,
    validated: BTreeSet<[u64; 5]>,
    bounds: [f64; 4],
}
fn key(b: OrientedBox) -> [u64; 5] {
    [
        b.pose.x.to_bits(),
        b.pose.y.to_bits(),
        b.pose.yaw.to_bits(),
        b.length.to_bits(),
        b.width.to_bits(),
    ]
}
fn intersects(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1]
}
fn body_bounds(b: OrientedBox) -> [f64; 4] {
    corners(b).into_iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |mut a, p| {
            a[0] = a[0].min(p.0);
            a[1] = a[1].min(p.1);
            a[2] = a[2].max(p.0);
            a[3] = a[3].max(p.1);
            a
        },
    )
}
fn cells(bounds: [f64; 4]) -> Result<[i32; 4], String> {
    if bounds.iter().any(|v| !v.is_finite() || v.abs() > 1e8) {
        return Err("parking sweep index coordinate budget exceeded".into());
    }
    Ok(bounds.map(|v| (v / CELL).floor() as i32))
}
pub(crate) fn adoption_proof(
    ego: EgoState,
    states: &[TruckState],
    geometry: VehicleGeometry,
    obstacles: &[OrientedBox],
    occupancy: &[OrientedBox],
    cancelled: Option<&AtomicBool>,
) -> Result<ParkingAdoptionProof, String> {
    if states.len() > 20_000 || obstacles.len() + occupancy.len() > MAX_OBJECTS {
        return Err("parking sweep certificate input budget exceeded".into());
    }
    let mut proof = ParkingAdoptionProof {
        first: states.first().copied(),
        sweeps: vec![],
        grid: BTreeMap::new(),
        validated: obstacles
            .iter()
            .chain(occupancy)
            .copied()
            .map(key)
            .collect(),
        bounds: [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
    };
    let mut references = 0;
    let mut previous = footprints(TruckState::from(ego), geometry);
    for current in
        std::iter::once(previous.clone()).chain(states.iter().map(|s| footprints(*s, geometry)))
    {
        if cancelled.is_some_and(|c| c.load(Ordering::Acquire)) {
            return Err("parking adoption cancelled".into());
        }
        for (from, to) in previous.iter().zip(&current) {
            let travel = from.pose.distance(to.pose);
            let rotation = wrap_angle(to.pose.yaw - from.pose.yaw).abs();
            let radius = from.length.hypot(from.width).max(to.length.hypot(to.width)) / 2.0;
            let work = ((travel + rotation * radius) / 0.25)
                .ceil()
                .clamp(1.0, 10000.0) as usize;
            // A capsule enclosing every orientation, plus the actual swept SAT
            // margin and its sampling inflation. Broad-phase is never narrower
            // than the production swept_collision predicate.
            let padding = radius
                + std::f64::consts::SQRT_2
                    * (0.3 + (travel + rotation * radius) / (2.0 * work as f64));
            let bounds = [
                from.pose.x.min(to.pose.x) - padding,
                from.pose.y.min(to.pose.y) - padding,
                from.pose.x.max(to.pose.x) + padding,
                from.pose.y.max(to.pose.y) + padding,
            ];
            let [x0, y0, x1, y1] = cells(bounds)?;
            let count = (i64::from(x1) - i64::from(x0) + 1) * (i64::from(y1) - i64::from(y0) + 1);
            if count > (MAX_REFERENCES - references) as i64 {
                return Err("parking sweep index reference budget exceeded".into());
            }
            references += count as usize;
            let id = proof.sweeps.len();
            for x in x0..=x1 {
                for y in y0..=y1 {
                    proof.grid.entry((x, y)).or_default().push(id);
                }
            }
            proof.sweeps.push(Sweep {
                from: *from,
                to: *to,
                bounds,
                work,
            });
            proof.bounds[0] = proof.bounds[0].min(bounds[0]);
            proof.bounds[1] = proof.bounds[1].min(bounds[1]);
            proof.bounds[2] = proof.bounds[2].max(bounds[2]);
            proof.bounds[3] = proof.bounds[3].max(bounds[3]);
        }
        previous = current;
    }
    Ok(proof)
}
/// Changed objects are checked geometrically, not rejected merely for moving.
/// False means a genuine new collision: request a new worker validation. Budget
/// exhaustion is an error and cannot grant motion permission.
pub(crate) fn adoption_environment_safe(
    proof: &ParkingAdoptionProof,
    obstacles: &[OrientedBox],
    occupancy: &[OrientedBox],
) -> Result<bool, String> {
    if obstacles.len() + occupancy.len() > MAX_OBJECTS {
        return Err("parking adoption object budget exceeded".into());
    }
    let started = Instant::now();
    let mut work = 0usize;
    let mut charge = |amount: usize| -> Result<(), String> {
        work = work.saturating_add(amount);
        if work > MAX_WORK || started.elapsed() > BUDGET {
            return Err("parking adoption local sweep work/time budget exceeded".into());
        }
        Ok(())
    };
    let mut checked = BTreeSet::new();
    for object in obstacles.iter().chain(occupancy).copied() {
        charge(1)?;
        if !object.valid() {
            return Err("invalid parking adoption obstacle".into());
        }
        let object_key = key(object);
        if proof.validated.contains(&object_key) || !checked.insert(object_key) {
            continue;
        }
        let bounds = body_bounds(object);
        if !intersects(bounds, proof.bounds) {
            continue;
        }
        let clipped = [
            bounds[0].max(proof.bounds[0]),
            bounds[1].max(proof.bounds[1]),
            bounds[2].min(proof.bounds[2]),
            bounds[3].min(proof.bounds[3]),
        ];
        let [x0, y0, x1, y1] = cells(clipped)?;
        let count = (i64::from(x1) - i64::from(x0) + 1) * (i64::from(y1) - i64::from(y0) + 1);
        if count > MAX_WORK as i64 {
            return Err("parking adoption grid query budget exceeded".into());
        }
        let mut candidates = BTreeSet::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                charge(1)?;
                if let Some(ids) = proof.grid.get(&(x, y)) {
                    for id in ids {
                        charge(1)?;
                        candidates.insert(*id);
                    }
                }
            }
        }
        for id in candidates {
            let sweep = proof.sweeps[id];
            if !intersects(bounds, sweep.bounds) {
                continue;
            }
            charge(sweep.work)?;
            if swept_collision(sweep.from, sweep.to, object, 0.3) {
                return Ok(false);
            }
            charge(0)?;
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use etsads_msgs::Pose2;
    fn geometry() -> VehicleGeometry {
        VehicleGeometry {
            has_trailer: false,
            trailer_wheelbase: 0.0,
            trailer_length: 0.0,
            ..Default::default()
        }
    }
    fn state(x: f64, y: f64) -> TruckState {
        TruckState {
            pose: Pose2 { x, y, yaw: 0.0 },
            trailer_yaw: 0.0,
            speed: 0.0,
            steer: 0.0,
        }
    }
    fn object(x: f64, y: f64) -> OrientedBox {
        OrientedBox {
            pose: Pose2 { x, y, yaw: 0.0 },
            length: 1.0,
            width: 1.0,
        }
    }
    #[test]
    fn moving_inside_global_bounds_but_outside_local_sweeps_can_be_adopted() {
        let ego = EgoState::default();
        let states: Vec<_> = (0..=100)
            .map(|i| state(-(i as f64) * 0.2, 0.0))
            .chain((1..=100).map(|i| state(-20.0, i as f64 * 0.2)))
            .collect();
        let proof =
            adoption_proof(ego, &states, geometry(), &[object(-10.0, 10.0)], &[], None).unwrap();
        for i in 1..10 {
            assert!(
                adoption_environment_safe(&proof, &[object(-10.0 + i as f64 * 0.1, 10.0)], &[])
                    .unwrap()
            );
        }
        assert!(!adoption_environment_safe(&proof, &[object(-10.0, 0.0)], &[]).unwrap());
    }
    #[test]
    fn dense_local_index_work_exhaustion_fails_closed() {
        let states = vec![state(0.0, 0.0); 10000];
        let proof =
            adoption_proof(EgoState::default(), &states, geometry(), &[], &[], None).unwrap();
        let error = adoption_environment_safe(&proof, &[object(0.0, 3.0)], &[]).unwrap_err();
        assert!(error.contains("budget"), "{error}");
    }
}
