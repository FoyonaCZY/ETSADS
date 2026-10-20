//! Road containment uses the same measured axle/hitch body envelopes and
//! conservative continuous sweep as controlled routes and parking.
use crate::{RouteSource, Runtime};
use etsads_control::TruckState;
use etsads_msgs::{Trajectory, VehicleGeometry, frames::wrap_angle};
use etsads_planning::geometry::{box_in_polygon, footprints, swept_in_polygon};
impl Runtime {
    fn road_polygon(&self) -> Result<Option<&[(f64, f64)]>, String> {
        if !matches!(&self.config.route_source,RouteSource::NativeGps(s) if s.road_evidence.is_some())
        {
            return Ok(None);
        }
        self.road_boundary
            .as_ref()
            .map(|b| Some(b.boundary.polygon.as_slice()))
            .ok_or("measured road boundary is unavailable".into())
    }
    pub(crate) fn check_road_execution(&mut self, state: TruckState) -> Result<(), String> {
        let Some(polygon) = self.road_polygon()? else {
            return Ok(());
        };
        let current = footprints(state, self.calibration.geometry);
        if current.iter().any(|b| !box_in_polygon(*b, polygon)) {
            return Err("current tractor/trailer body leaves the measured road boundary".into());
        }
        if let Some(previous) = self.road_previous
            && !contained_sweep(previous, state, self.calibration.geometry, polygon)
        {
            return Err("observed tractor/trailer sweep crossed the measured road boundary; control released".into());
        }
        self.road_previous = Some(state);
        Ok(())
    }
    pub(crate) fn check_road_trajectory(
        &self,
        initial: TruckState,
        trajectory: &Trajectory,
    ) -> Result<(), String> {
        let Some(polygon) = self.road_polygon()? else {
            return Ok(());
        };
        let mut previous = initial;
        if footprints(initial, self.calibration.geometry)
            .iter()
            .any(|b| !box_in_polygon(*b, polygon))
        {
            return Err("road trajectory starts outside the measured physical boundary".into());
        }
        for point in &trajectory.points {
            let state = TruckState {
                pose: point.pose,
                trailer_yaw: point.pose.yaw - point.articulation,
                speed: point.speed,
                steer: (point.curvature * self.calibration.geometry.wheelbase).atan(),
            };
            if !contained_sweep(previous, state, self.calibration.geometry, polygon) {
                return Err("planned tractor/trailer continuous sweep leaves the measured road boundary; control stopped before execution".into());
            }
            previous = state;
        }
        Ok(())
    }
}
fn contained_sweep(
    previous: TruckState,
    current: TruckState,
    geometry: VehicleGeometry,
    polygon: &[(f64, f64)],
) -> bool {
    let tractor_angle = wrap_angle(current.pose.yaw - previous.pose.yaw);
    let trailer_angle = wrap_angle(current.trailer_yaw - previous.trailer_yaw);
    let envelope = geometry.resolved_envelope();
    let mut offset_bounds =
        vec![envelope.tractor_center_from_rear_axle_m().abs() * tractor_angle.powi(2) / 8.0];
    if let Some(trailer) = envelope.trailer {
        offset_bounds.push(
            (geometry.hitch_offset.abs() * tractor_angle.powi(2)
                + trailer
                    .center_from_hitch_m(geometry.trailer_wheelbase)
                    .abs()
                    * trailer_angle.powi(2))
                / 8.0,
        );
    }
    // The existing rigid-box sweep interpolates body centres along chords.
    // A measured body centre actually rotates about its rear axle/hitch.
    // Linear-interpolation error is <= max|p''(t)|/8 = r*angle²/8;
    // inflate both endpoint boxes by that bound before reusing the sweep.
    footprints(previous, geometry)
        .iter()
        .zip(footprints(current, geometry))
        .zip(offset_bounds)
        .all(|((a, b), extra)| {
            let a = etsads_msgs::OrientedBox {
                length: a.length + 2.0 * extra,
                width: a.width + 2.0 * extra,
                ..*a
            };
            let b = etsads_msgs::OrientedBox {
                length: b.length + 2.0 * extra,
                width: b.width + 2.0 * extra,
                ..b
            };
            box_in_polygon(b, polygon) && swept_in_polygon(a, b, polygon)
        })
}
