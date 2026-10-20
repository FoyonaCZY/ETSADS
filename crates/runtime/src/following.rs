//! One measured leader selection shared by IDM and recording diagnostics.
use etsads_map::{LaneGraph, matcher::LaneState};
use etsads_msgs::{Agent, EgoState, Point3, frames::wrap_angle};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FollowingQuality {
    VerifiedGeometry,
    SyntheticReplay,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct FollowingDiagnostic {
    pub leader_id: u64,
    pub bumper_gap_m: f64,
    pub ego_speed_mps: f64,
    pub headway_seconds: f64,
    pub frame_id: u64,
    pub quality: FollowingQuality,
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct LeadObservation {
    pub leader_id: Option<u64>,
    pub bumper_gap_m: f64,
    pub speed_mps: f64,
    pub same_direction: bool,
}
impl LeadObservation {
    pub fn idm(self) -> (f64, f64) {
        (self.bumper_gap_m, self.speed_mps)
    }
    pub fn diagnostic(
        self,
        ego: EgoState,
        quality: FollowingQuality,
    ) -> Option<FollowingDiagnostic> {
        if !self.same_direction
            || ego.speed <= 0.05
            || !ego.speed.is_finite()
            || !self.bumper_gap_m.is_finite()
            || self.bumper_gap_m < 0.0
        {
            return None;
        }
        Some(FollowingDiagnostic {
            leader_id: self.leader_id?,
            bumper_gap_m: self.bumper_gap_m,
            ego_speed_mps: ego.speed,
            headway_seconds: self.bumper_gap_m / ego.speed,
            frame_id: ego.header.frame_id,
            quality,
        })
    }
}
pub(crate) fn select_lead(
    graph: &LaneGraph,
    route: &[u64],
    lane: LaneState,
    ego: EgoState,
    agents: &[Agent],
    ego_front_from_rear_axle_m: f64,
    ego_width_m: f64,
) -> Option<LeadObservation> {
    if !ego_front_from_rear_axle_m.is_finite()
        || ego_front_from_rear_axle_m <= 0.0
        || !ego_width_m.is_finite()
        || !(0.5..=10.0).contains(&ego_width_m)
        || !lane.heading_err.is_finite()
    {
        return None;
    }
    let ego_front_support = ego_front_from_rear_axle_m * lane.heading_err.cos()
        + ego_width_m / 2.0 * lane.heading_err.sin().abs();
    let start = route.iter().position(|id| *id == lane.lane_id)?;
    let mut offsets = Vec::new();
    let mut offset = -lane.s;
    for id in &route[start..] {
        offsets.push((*id, offset));
        offset += graph.lanes.get(id)?.length();
        if offset > 300.0 {
            break;
        }
    }
    let mut lead: Option<LeadObservation> = None;
    for agent in agents {
        if !agent.valid() {
            continue;
        }
        let bodies = std::iter::once((agent.pose, agent.length, agent.width)).chain(
            agent
                .trailers
                .iter()
                .map(|body| (body.pose, body.length, body.width)),
        );
        for (pose, length, width) in bodies {
            for (id, offset) in &offsets {
                let Ok(projected) = graph.project(
                    *id,
                    Point3 {
                        x: pose.x,
                        y: pose.y,
                        z: agent.z,
                    },
                ) else {
                    continue;
                };
                let delta = wrap_angle(pose.yaw - projected.heading);
                let body_back_extent =
                    delta.cos().abs() * length / 2.0 + delta.sin().abs() * width / 2.0;
                let body_side_extent =
                    delta.sin().abs() * length / 2.0 + delta.cos().abs() * width / 2.0;
                if projected.d.abs() > graph.lanes[id].width / 2.0 + body_side_extent
                    || (projected.z - agent.z).abs() > 3.0
                {
                    continue;
                }
                // Projection clamps to a polyline endpoint. Preserve the
                // tangent residual so a distant object cannot masquerade as
                // a leader at that endpoint, while a body crossing it remains
                // visible even when its centre lies just beyond the lane.
                let Some((on_lane, _)) = graph.lanes[id].sample(projected.s) else {
                    continue;
                };
                let centre_s = projected.s
                    + (pose.x - on_lane.x) * projected.heading.cos()
                    + (pose.y - on_lane.y) * projected.heading.sin();
                if centre_s + body_back_extent < 0.0
                    || centre_s - body_back_extent > graph.lanes[id].length()
                {
                    continue;
                }
                let center = offset + centre_s;
                if center <= 0.0 {
                    continue;
                }
                let gap = center - body_back_extent - ego_front_support;
                let same_direction = delta.abs() <= 30f64.to_radians() && agent.speed >= 0.0;
                if lead.is_none_or(|previous| {
                    gap < previous.bumper_gap_m
                        || gap == previous.bumper_gap_m && Some(agent.id) < previous.leader_id
                }) {
                    lead = Some(LeadObservation {
                        leader_id: Some(agent.id),
                        bumper_gap_m: gap,
                        speed_mps: (agent.speed * delta.cos()).max(0.0),
                        same_direction,
                    });
                }
            }
        }
    }
    // A route endpoint is an IDM stop constraint, never a measured vehicle.
    lead.or(Some(LeadObservation {
        leader_id: None,
        bumper_gap_m: (offset - ego_front_support - ego.speed.max(0.0) * 0.3).max(0.0),
        speed_mps: 0.0,
        same_direction: false,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ExecutionMode, Runtime,
        replay::{calibration, controlled_fixture, snapshot},
    };
    fn lane() -> LaneState {
        LaneState {
            lane_id: 1,
            s: 0.0,
            d: 0.0,
            heading_err: 0.0,
            confidence: 1.0,
        }
    }
    fn agent() -> Agent {
        Agent {
            id: 42,
            pose: etsads_msgs::Pose2 {
                x: 50.0,
                y: 0.0,
                yaw: 0.0,
            },
            z: 0.0,
            speed: 4.0,
            acceleration: 0.0,
            length: 10.0,
            width: 2.5,
            lane_id: Some(1),
            trailers: vec![],
        }
    }
    #[test]
    fn idm_and_diagnostic_share_measured_bumper_and_current_frame() {
        let (config, region) = controlled_fixture(false, false, false);
        let mut cal = calibration();
        cal.front_bumper.as_mut().unwrap().front_from_rear_axle_m = 4.2;
        let runtime = Runtime::new(config, cal, Some(region.clone())).unwrap();
        let mut sample = snapshot(22);
        sample.scene.agents = vec![agent()];
        let lead = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            sample.ego,
            &sample.scene.agents,
            4.2,
            2.55,
        )
        .unwrap();
        assert!((lead.idm().0 - 40.8).abs() < 1e-12);
        let diagnostic = runtime
            .following_diagnostic(&sample, sample.ego, Some(lane()))
            .unwrap();
        assert_eq!(diagnostic.frame_id, 22);
        assert_eq!(diagnostic.leader_id, 42);
        assert_eq!(diagnostic.bumper_gap_m, lead.idm().0);
        assert_eq!(diagnostic.headway_seconds, lead.idm().0 / sample.ego.speed);
        assert_eq!(diagnostic.quality, FollowingQuality::SyntheticReplay);
        assert_eq!(
            serde_json::to_value(diagnostic).unwrap()["quality"],
            "synthetic_replay"
        );
        assert_eq!(runtime.config.execution, ExecutionMode::Replay);
    }
    #[test]
    fn route_end_stationary_reverse_opposing_and_unknown_geometry_are_not_headway_samples() {
        let (config, region) = controlled_fixture(false, false, false);
        let mut sample = snapshot(2);
        let endpoint = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            sample.ego,
            &[],
            4.2,
            2.55,
        )
        .unwrap();
        assert!(
            endpoint
                .diagnostic(sample.ego, FollowingQuality::SyntheticReplay)
                .is_none()
        );
        sample.scene.agents = vec![agent()];
        let mut cal = calibration();
        cal.front_bumper = None;
        let runtime = Runtime::new(config, cal, Some(region.clone())).unwrap();
        assert!(
            runtime
                .following_diagnostic(&sample, sample.ego, Some(lane()))
                .is_none()
        );
        for speed in [0.0, -1.0] {
            sample.ego.speed = speed;
            assert!(
                select_lead(
                    &region.imported.graph,
                    &[1],
                    lane(),
                    sample.ego,
                    &sample.scene.agents,
                    4.2,
                    2.55
                )
                .unwrap()
                .diagnostic(sample.ego, FollowingQuality::SyntheticReplay)
                .is_none()
            );
        }
        sample.ego.speed = 6.0;
        sample.scene.agents[0].pose.yaw = std::f64::consts::PI;
        let lead = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            sample.ego,
            &sample.scene.agents,
            4.2,
            2.55,
        )
        .unwrap();
        assert_eq!(lead.speed_mps, 0.0);
        assert!(
            lead.diagnostic(sample.ego, FollowingQuality::SyntheticReplay)
                .is_none()
        );
    }
    #[test]
    fn attached_traffic_trailer_rear_is_the_following_gap() {
        let (_, region) = controlled_fixture(false, false, false);
        let mut lead = agent();
        lead.trailers.push(etsads_msgs::OrientedBox {
            pose: etsads_msgs::Pose2 {
                x: 40.0,
                y: 0.0,
                yaw: 0.0,
            },
            length: 12.0,
            width: 2.5,
        });
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            snapshot(2).ego,
            &[lead],
            4.2,
            2.55,
        )
        .unwrap();
        assert!((selected.bumper_gap_m - 29.8).abs() < 1e-12);
        assert_eq!(selected.leader_id, Some(42));
    }
    #[test]
    fn rotated_ego_front_corner_participates_in_measured_gap() {
        let (_, region) = controlled_fixture(false, false, false);
        let mut matched = lane();
        matched.heading_err = 30_f64.to_radians();
        let mut ego = snapshot(2).ego;
        ego.pose.yaw = matched.heading_err;
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            matched,
            ego,
            &[agent()],
            4.2,
            2.55,
        )
        .unwrap();
        let expected = 45.0 - (4.2 * matched.heading_err.cos() + 1.275 * matched.heading_err.sin());
        assert!((selected.bumper_gap_m - expected).abs() < 1e-12);
    }
    #[test]
    fn legacy_calibration_or_foreign_bumper_evidence_cannot_certify_gap() {
        let measured = calibration();
        assert!(measured.measured_front_bumper().is_some());
        let mut legacy = serde_json::to_value(&measured).unwrap();
        legacy.as_object_mut().unwrap().remove("front_bumper");
        let legacy: etsads_control::identification::VehicleCalibration =
            serde_json::from_value(legacy).unwrap();
        assert!(legacy.measured_front_bumper().is_none());
        let mut foreign = measured.clone();
        foreign.front_bumper.as_mut().unwrap().geometry_fingerprint = "another-truck".into();
        assert!(foreign.measured_front_bumper().is_none());
        for invalid in [f64::NAN, 0.0, 100.0] {
            let mut invalid_measurement = measured.clone();
            invalid_measurement
                .front_bumper
                .as_mut()
                .unwrap()
                .front_from_rear_axle_m = invalid;
            assert!(invalid_measurement.measured_front_bumper().is_none());
        }
    }
    #[test]
    fn cut_in_and_rotated_trailer_are_leaders_when_their_bodies_enter_the_lane() {
        let (_, mut region) = controlled_fixture(false, false, false);
        region.imported.graph.lanes.get_mut(&1).unwrap().width = 3.5;
        let mut cut_in = agent();
        cut_in.pose.y = 2.2;
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            snapshot(2).ego,
            &[cut_in],
            4.2,
            2.55,
        )
        .unwrap();
        assert_eq!(selected.leader_id, Some(42));
        assert!(
            selected
                .diagnostic(snapshot(2).ego, FollowingQuality::SyntheticReplay)
                .is_some()
        );
        let mut truck = agent();
        truck.pose.y = 4.5;
        truck.trailers.push(etsads_msgs::OrientedBox {
            pose: etsads_msgs::Pose2 {
                x: 40.0,
                y: 4.0,
                yaw: 30_f64.to_radians(),
            },
            length: 12.0,
            width: 2.5,
        });
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            snapshot(2).ego,
            &[truck],
            4.2,
            2.55,
        )
        .unwrap();
        let expected =
            40.0 - 6.0 * 30_f64.to_radians().cos() - 1.25 * 30_f64.to_radians().sin() - 4.2;
        assert_eq!(selected.leader_id, Some(42));
        assert!((selected.bumper_gap_m - expected).abs() < 1e-12);
    }
    #[test]
    fn fully_adjacent_and_separated_height_layers_are_not_following_leaders() {
        let (_, mut region) = controlled_fixture(false, false, false);
        region.imported.graph.lanes.get_mut(&1).unwrap().width = 3.5;
        for (lateral, height) in [(4.5, 0.0), (0.0, 8.0), (0.0, -8.0)] {
            let mut other = agent();
            other.pose.y = lateral;
            other.z = height;
            let selected = select_lead(
                &region.imported.graph,
                &[1],
                lane(),
                snapshot(2).ego,
                &[other],
                4.2,
                2.55,
            )
            .unwrap();
            assert_eq!(
                selected.leader_id, None,
                "lateral={lateral} height={height}"
            );
        }
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            snapshot(2).ego,
            &[agent()],
            4.2,
            2.55,
        )
        .unwrap();
        assert_eq!(selected.leader_id, Some(42));
    }
    #[test]
    fn body_overlap_at_lane_end_keeps_true_longitudinal_distance() {
        let (_, region) = controlled_fixture(false, false, false);
        let mut other = agent();
        other.pose.x = 350.0;
        assert_eq!(
            select_lead(
                &region.imported.graph,
                &[1],
                lane(),
                snapshot(2).ego,
                &[other.clone()],
                4.2,
                2.55
            )
            .unwrap()
            .leader_id,
            None
        );
        other.pose.x = 302.0;
        let selected = select_lead(
            &region.imported.graph,
            &[1],
            lane(),
            snapshot(2).ego,
            &[other],
            4.2,
            2.55,
        )
        .unwrap();
        assert!((selected.bumper_gap_m - (302.0 - 5.0 - 4.2)).abs() < 1e-12);
    }
}
