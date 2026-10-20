//! Native GPS route ownership and regional rebinding. Runtime IDs never carry
//! identity across map windows; source lane IDs and native checkpoint UIDs do.
use crate::{
    DrivingMode,
    controlled_route::{LaneChangeMission, RoundaboutExit, gates::derive_native_gates},
};
use etsads_map::{
    corridor::{
        BoundCorridor, ChannelKind, CorridorPolicy, GlobalNativeRouteIndex, NativeRouteIndex,
    },
    importer::{RegionGraph, RegionPolicy},
    sector::ReadStats,
    traffic::TrafficContext,
};
use etsads_msgs::Point3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RouteSource {
    #[default]
    Explicit,
    NativeGps(Box<NativeGpsSettings>),
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeGpsSettings {
    /// SHA-256 identity of the partition index. The host verifies its shards.
    pub expected_source_id: String,
    #[serde(default)]
    pub corridor_policy: CorridorPolicy,
    /// Explicitly reviewed toll interactions, indexed by stable source lane ID.
    /// A map barrier alone is not authority to issue an activate event.
    #[serde(default)]
    pub tolls: Vec<SourceTollStop>,
    #[serde(default)]
    pub roundabout_exits: Vec<SourceRoundaboutExit>,
    /// None retains the bounded commissioning policy and its 30 km/h cap.
    #[serde(default)]
    pub road_evidence: Option<etsads_map::importer::RoadEvidence>,
    #[serde(default)]
    pub road_boundary_path: Option<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum RoadBoundaryCoordinates {
    #[serde(rename = "ENU_metres")]
    EnuMetres,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoadBoundary {
    pub schema_version: u32,
    pub map_source_id: String,
    pub coordinate_system: RoadBoundaryCoordinates,
    pub verified_source_lane_ids: Vec<String>,
    pub boundary: crate::parking_mission::VerifiedBoundary,
}
impl RoadBoundary {
    pub fn validate(
        &self,
        settings: &NativeGpsSettings,
        authorization: &RegionPolicy,
    ) -> Result<(), String> {
        let measured = settings
            .road_evidence
            .as_ref()
            .ok_or("road boundary supplied without a road evidence mode")?;
        let unique: std::collections::BTreeSet<_> = self.verified_source_lane_ids.iter().collect();
        let allowed: std::collections::BTreeSet<_> =
            authorization.verified_lane_ids.iter().collect();
        if self.schema_version != 1
            || self.map_source_id != settings.expected_source_id
            || !self.boundary.valid()
            || unique.len() != self.verified_source_lane_ids.len()
            || unique != allowed
            || self.boundary.evidence_path != measured.physical_boundary_evidence_path
        {
            return Err("road boundary must be a measured ENU polygon bound to the exact map, lane scope and physical-boundary evidence".into());
        }
        let p = &self.boundary.polygon;
        let signed_area: f64 = p
            .iter()
            .zip(p.iter().cycle().skip(1))
            .take(p.len())
            .map(|(a, b)| a.0 * b.1 - b.0 * a.1)
            .sum();
        if !signed_area.is_finite() || signed_area.abs() < 1e-6 {
            return Err("road boundary polygon has no finite enclosed area".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceTollStop {
    pub id: String,
    pub source_lane_id: String,
    pub barrier_position: Point3,
    pub stop_s: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceRoundaboutExit {
    pub source_lane_id: String,
    pub exit_s: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MapProvenance {
    pub source_id: String,
    pub annotations_source_id: String,
    pub center: Point3,
    pub radius_m: f64,
    pub generation: u64,
    pub reason: String,
    pub reads: ReadStats,
}
/// Produced by a map host after authenticating one source and applying the
/// original, unchanged policy to the loaded window. Construction grants no
/// supervisor permission and does not certify the source measurements.
#[derive(Clone, Debug)]
pub struct NavigationRegion {
    pub region: RegionGraph,
    pub native: NativeRouteIndex,
    pub global: Arc<GlobalNativeRouteIndex>,
    pub traffic: TrafficContext,
    pub gate_candidates: Vec<etsads_map::annotations::GateCandidate>,
    pub authorization: RegionPolicy,
    pub road_evidence: Option<etsads_map::importer::RoadEvidence>,
    pub road_boundary: Option<RoadBoundary>,
    pub provenance: MapProvenance,
}
impl NavigationRegion {
    pub fn scope_digest(&self) -> Result<[u8; 32], String> {
        let data = serde_json::to_vec(&(
            "ETSADS_NATIVE_SCOPE_V1",
            &self.provenance.source_id,
            &self.provenance.annotations_source_id,
            &self.authorization,
            &self.road_evidence,
            &self.road_boundary,
        ))
        .map_err(|e| e.to_string())?;
        Ok(Sha256::digest(data).into())
    }
    pub fn validate(&self, settings: &NativeGpsSettings) -> Result<(), String> {
        if settings.road_evidence.is_some() {
            if settings
                .road_boundary_path
                .as_ref()
                .is_none_or(|p| p.trim().is_empty())
            {
                return Err("road operation requires an explicit measured boundary file".into());
            }
            self.road_boundary
                .as_ref()
                .ok_or("road operation lacks the measured boundary data")?
                .validate(settings, &self.authorization)?;
        } else if self.road_boundary.is_some() || settings.road_boundary_path.is_some() {
            return Err("measured road boundary requires road operation mode".into());
        }
        if self.road_evidence != settings.road_evidence {
            return Err("road-region evidence differs from the configured operating mode".into());
        }
        if self.road_evidence.as_ref().is_some_and(|e| {
            [
                &e.centerline_evidence_path,
                &e.physical_boundary_evidence_path,
                &e.vehicle_access_evidence_path,
            ]
            .iter()
            .any(|p| p.trim().is_empty())
        }) {
            return Err("road-region measurement evidence references are incomplete".into());
        }
        if self.road_evidence.is_some()
            && (!self.region.physical_boundaries_verified
                || self.authorization.allow_navigation_width)
        {
            return Err("road operation requires measured physical boundaries and widths".into());
        }
        if self.road_evidence.is_none() && self.region.policy.maximum_speed_mps > 30.0 / 3.6 {
            return Err("commissioning region speed exceeds 30 km/h; select the independent road-region evidence mode".into());
        }
        if settings.expected_source_id.is_empty()
            || self.provenance.source_id != settings.expected_source_id
            || self.provenance.annotations_source_id.is_empty()
            || !self.provenance.center.valid()
            || self.provenance.radius_m != 2000.0
            || self.provenance.reason.is_empty()
        {
            return Err(
                "native GPS requires the pinned map source, annotations and a 2 km window".into(),
            );
        }
        self.traffic.validate().map_err(|e| e.to_string())?;
        self.native
            .bind(&self.region.imported)
            .map_err(|e| e.to_string())?;
        let local = &self.region.policy;
        let mut policy = self.authorization.clone();
        policy.verified_lane_ids.retain(|id| {
            self.region
                .imported
                .runtime_to_source
                .values()
                .any(|s| s == id)
        });
        if serde_json::to_value(local).map_err(|e| e.to_string())?
            != serde_json::to_value(policy).map_err(|e| e.to_string())?
        {
            return Err(
                "regional policy is not the exact intersection of the original authorization"
                    .into(),
            );
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NavigationDiagnostic {
    pub frame_id: u64,
    pub source: String,
    pub map: MapProvenance,
    pub gps_node_uids: Vec<u64>,
    pub consumed_prefix_nodes: usize,
    pub corridor_source_ids: Vec<String>,
    pub corridor_runtime_ids: Vec<u64>,
    pub allowed_channels: Vec<etsads_map::corridor::AllowedChannel>,
    pub next_boundary_checkpoint: Option<u64>,
    pub terminal_distance_m: Option<f64>,
    pub reason: String,
}
pub(crate) struct NativeNavigation {
    pub map: NavigationRegion,
    pub template: DrivingMode,
    pub gps: Vec<u64>,
    pub corridor: Option<BoundCorridor>,
    pub next_boundary_checkpoint: Option<u64>,
    pub consumed: usize,
    pub corridor_consumed_base: usize,
    pub diagnostic: Option<NavigationDiagnostic>,
    pub scope: [u8; 32],
    pub dirty: bool,
}
impl NativeNavigation {
    pub fn new(map: NavigationRegion, template: DrivingMode) -> Result<Self, String> {
        let scope = map.scope_digest()?;
        Ok(Self {
            map,
            template,
            scope,
            gps: vec![],
            corridor: None,
            next_boundary_checkpoint: None,
            consumed: 0,
            corridor_consumed_base: 0,
            diagnostic: None,
            dirty: true,
        })
    }
    pub fn observe_gps(&mut self, route: &[u64], source_lane: &str) -> Result<(), String> {
        let mut route = route.to_vec();
        route.dedup();
        if route.is_empty() || route.len() > 4096 || route.contains(&0) {
            return Err("native GPS route is missing, invalid or oversized".into());
        }
        if !self.gps.is_empty() && self.gps != route {
            let Some(offset) = (1..self.gps.len()).find(|&n| self.gps[n..] == route) else {
                return Err("native GPS rerouted or changed destination; old corridor invalidated, explicit replanning/rearm required".into());
            };
            // Only a prefix supported by already traversed corridor steps may
            // disappear. A jumped route cannot silently skip a mandatory exit.
            let traversed = self
                .corridor
                .as_ref()
                .and_then(|c| {
                    c.native
                        .steps
                        .iter()
                        .find(|s| s.source_lane_id == source_lane)
                })
                .map(|s| self.corridor_consumed_base + s.checkpoint_index)
                .unwrap_or(0);
            if self.consumed + offset > traversed {
                return Err("GPS prefix consumed checkpoints ahead of the observed vehicle; route progress is unverified".into());
            }
            self.consumed += offset;
        }
        self.gps = route;
        Ok(())
    }
    pub fn bind_mode(&self, settings: &NativeGpsSettings) -> Result<DrivingMode, String> {
        let corridor = self
            .corridor
            .as_ref()
            .ok_or("native corridor unavailable")?;
        let route = corridor.runtime_lane_ids.clone();
        let source_to_id: BTreeMap<_, _> = self
            .map
            .region
            .imported
            .runtime_to_source
            .iter()
            .map(|(id, source)| (source.as_str(), *id))
            .collect();
        if !corridor.native.all_physical_boundaries_verified {
            return Err("native GPS corridor has unverified physical boundaries".into());
        }
        let sources: Vec<_> = corridor
            .native
            .steps
            .iter()
            .map(|s| s.source_lane_id.clone())
            .collect();
        if !self.map.traffic.has_annotations_for_route(&sources) {
            return Err("native GPS corridor lacks complete traffic annotations".into());
        }
        let gates = derive_native_gates(
            &self.map.region.imported.graph,
            &self.map.region.imported.runtime_to_source,
            corridor,
            &self.map.gate_candidates,
            &settings.tolls,
        )?;
        match self.template.clone() {
            DrivingMode::LaneFollow {
                maximum_speed_mps, ..
            } => {
                if sources.iter().any(|s| !s.contains(":road:"))
                    || corridor
                        .native
                        .allowed_channels
                        .iter()
                        .any(|c| c.kind != ChannelKind::Successor)
                    || !self.map.traffic.upcoming_stop_controls(&sources).is_empty()
                    || settings
                        .tolls
                        .iter()
                        .any(|t| sources.contains(&t.source_lane_id))
                    || !gates.is_empty()
                {
                    return Err(format!(
                        "native corridor requires controlled-route junction, lane-change or toll handling: sources={sources:?}, channels={:?}, stops={}",
                        corridor
                            .native
                            .allowed_channels
                            .iter()
                            .map(|c| c.kind)
                            .collect::<Vec<_>>(),
                        self.map.traffic.upcoming_stop_controls(&sources).len()
                    ));
                }
                Ok(DrivingMode::LaneFollow {
                    route,
                    maximum_speed_mps,
                })
            }
            DrivingMode::ControlledRoute { mut mission } => {
                mission.route = route;
                mission.traffic = self.map.traffic.clone();
                mission.lane_changes = corridor
                    .native
                    .allowed_channels
                    .iter()
                    .filter_map(|c| {
                        c.lane_change.as_ref().map(|change| LaneChangeMission {
                            from_lane: source_to_id[c.from_source_lane.as_str()],
                            to_lane: source_to_id[c.to_source_lane.as_str()],
                            earliest_s: change.earliest_start_s_m,
                            // Runtime's existing deadline denotes completion, not start.
                            deadline_s: change.latest_start_s_m + change.minimum_distance_m,
                            latest_start_s: Some(change.latest_start_s_m),
                        })
                    })
                    .collect();
                mission.tolls = gates;
                mission.roundabout_exits = settings
                    .roundabout_exits
                    .iter()
                    .filter_map(|e| {
                        source_to_id
                            .get(e.source_lane_id.as_str())
                            .map(|&id| RoundaboutExit {
                                lane_id: id,
                                exit_s: e.exit_s,
                            })
                    })
                    .collect();
                Ok(DrivingMode::ControlledRoute { mission })
            }
            _ => Err(
                "native GPS is only available for lane-follow or controlled-route missions".into(),
            ),
        }
    }
    pub fn terminal_distance(&self, lane: u64, s: f64) -> Option<f64> {
        let c = self.corridor.as_ref()?;
        let index = c.runtime_lane_ids.iter().position(|id| *id == lane)?;
        let steps = &c.native.steps;
        Some(
            steps[index].exit_s_m - s
                + steps[index + 1..]
                    .iter()
                    .map(|step| step.exit_s_m - step.entry_s_m)
                    .sum::<f64>(),
        )
    }
}

impl crate::Runtime {
    pub fn report_navigation_failure(&mut self, reason: impl Into<String>) {
        let reason = reason.into();
        if let Some(diagnostic) = self.navigation.as_mut().and_then(|n| n.diagnostic.as_mut()) {
            diagnostic.reason = reason.clone();
        }
        self.fault = Some(reason);
    }
    pub fn can_replace_native_region(&self) -> bool {
        !self.intent.active_lane_change && self.fault.is_none()
    }

    /// Accept a completed background read. The next physical frame re-matches
    /// and derives a new corridor before publishing any newly planned control.
    pub fn install_native_region(
        &mut self,
        map: NavigationRegion,
    ) -> Result<(), crate::RuntimeError> {
        let invalid = |e: String| crate::RuntimeError::Configuration(e);
        let RouteSource::NativeGps(settings) = &self.config.route_source else {
            return Err(invalid(
                "native map supplied to an explicit-route runtime".into(),
            ));
        };
        map.validate(settings).map_err(invalid)?;
        let scope = map.scope_digest().map_err(invalid)?;
        if self.config.execution == crate::ExecutionMode::Live
            && settings.road_evidence.is_some()
            && self.live_road_scope != Some(scope)
        {
            return Err(invalid(
                "road map replacement is outside the host-verified measurement and policy scope"
                    .into(),
            ));
        }
        if self.config.execution == crate::ExecutionMode::Live
            && matches!(self.config.mode, DrivingMode::ControlledRoute { .. })
            && self.live_navigation_scope != Some(scope)
        {
            return Err(invalid(
                "native map is outside the host-verified live experiment source/policy scope"
                    .into(),
            ));
        }
        if !self.can_replace_native_region() {
            return Err(invalid(
                "region replacement is blocked by a fault or active lane change".into(),
            ));
        }
        if let Some(navigation) = &mut self.navigation {
            if scope != navigation.scope
                || map.provenance.generation <= navigation.map.provenance.generation
            {
                return Err(invalid(
                    "map replacement changed source authorization or regressed generation".into(),
                ));
            }
            navigation.map = map;
            navigation.dirty = true;
        } else {
            self.navigation =
                Some(NativeNavigation::new(map, self.config.mode.clone()).map_err(invalid)?);
        }
        self.matcher.reset();
        Ok(())
    }

    pub(crate) fn apply_plan_context(&mut self, context: crate::PlanContext) -> Result<(), String> {
        let mut graph = context.region.imported.graph.clone();
        if let DrivingMode::ControlledRoute { mission } = &context.mode {
            if let Some(engine) = &mut self.route_engine {
                engine.rebind(
                    mission.clone(),
                    &graph,
                    context.region.imported.runtime_to_source.clone(),
                    self.calibration.geometry,
                )?;
            } else {
                self.route_engine = Some(crate::controlled_route::RouteEngine::new(
                    mission.clone(),
                    &graph,
                    context.region.imported.runtime_to_source.clone(),
                    self.calibration.geometry,
                    self.interactions_authorized,
                )?);
            }
        }
        let route = match &context.mode {
            DrivingMode::LaneFollow { route, .. } => route,
            DrivingMode::ControlledRoute { mission } => &mission.route,
            _ => return Err("native planning context is not a route".into()),
        };
        for id in route {
            graph
                .lanes
                .get_mut(id)
                .ok_or("bound corridor lane disappeared")?
                .successors
                .clear();
        }
        for pair in route.windows(2) {
            if context.region.imported.graph.lanes[&pair[0]]
                .successors
                .contains(&pair[1])
            {
                graph.lanes.get_mut(&pair[0]).unwrap().successors = vec![pair[1]];
            }
        }
        self.config.mode = context.mode;
        self.planning_graph = Some(graph);
        self.region = Some(context.region);
        self.native_terminal = context.terminal;
        self.road_boundary = context.road_boundary;
        Ok(())
    }

    pub(crate) fn update_native_route(
        &mut self,
        snapshot: &etsads_game::adapter::GameSnapshot,
    ) -> Result<etsads_map::matcher::LaneState, String> {
        let RouteSource::NativeGps(settings) = self.config.route_source.clone() else {
            unreachable!()
        };
        let ego = snapshot.ego;
        if snapshot
            .extension
            .as_ref()
            .is_none_or(|e| e.header.frame_id != ego.header.frame_id || e.available & 16 == 0)
        {
            return Err("native GPS requires same-frame route extension observations".into());
        }
        let nav = self
            .navigation
            .as_mut()
            .ok_or("native GPS map source has not been installed")?;
        if (Point3 {
            x: ego.pose.x,
            y: ego.pose.y,
            z: ego.z,
        })
        .distance_xy(nav.map.provenance.center)
            >= nav.map.provenance.radius_m
        {
            return Err("vehicle left the loaded native map window".into());
        }
        if nav.diagnostic.is_none() {
            nav.diagnostic = Some(NavigationDiagnostic {
                frame_id: ego.header.frame_id,
                source: "native_gps".into(),
                map: nav.map.provenance.clone(),
                gps_node_uids: snapshot.scene.route.clone(),
                consumed_prefix_nodes: nav.consumed,
                corridor_source_ids: vec![],
                corridor_runtime_ids: vec![],
                allowed_channels: vec![],
                next_boundary_checkpoint: None,
                terminal_distance_m: None,
                reason: "resolving observed native GPS route".into(),
            });
        }
        let matched = self
            .matcher
            .update(
                &nav.map.region.imported.graph,
                ego.pose,
                ego.z,
                ego.speed,
                ego.header.sim_time_us as f64 / 1e6,
            )
            .ok_or("native GPS cannot match the current verified region")?;
        if !self.intent.active_lane_change
            && (matched.confidence < 0.6
                || matched.d.abs() > nav.map.region.policy.maximum_lateral_error_m)
        {
            return Err(
                "native GPS map match confidence or approved lateral deviation failed".into(),
            );
        }
        let source = nav
            .map
            .region
            .imported
            .runtime_to_source
            .get(&matched.lane_id)
            .ok_or("matched lane lacks source identity")?
            .clone();
        // Record the observed route even when the change is rejected.
        if let Some(diagnostic) = &mut nav.diagnostic {
            diagnostic.frame_id = ego.header.frame_id;
            diagnostic.gps_node_uids = snapshot.scene.route.clone();
        }
        if let Err(reason) = nav.observe_gps(&snapshot.scene.route, &source) {
            if let Some(diagnostic) = &mut nav.diagnostic {
                diagnostic.reason = reason.clone();
            }
            return Err(reason);
        }
        let mut context = None;
        if nav.dirty {
            let resolved = nav
                .map
                .native
                .resolve_local_prefix(
                    &nav.map.global,
                    &nav.gps,
                    &source,
                    matched.s,
                    &settings.corridor_policy,
                )
                .map_err(|e| e.to_string())?;
            let bound = resolved
                .corridor
                .bind(&nav.map.native, &nav.map.region.imported)
                .map_err(|e| e.to_string())?;
            let old_remaining = nav.corridor.as_ref().and_then(|c| {
                c.native
                    .steps
                    .iter()
                    .position(|s| s.source_lane_id == source)
                    .map(|index| {
                        c.native.steps[index..]
                            .iter()
                            .map(|s| s.source_lane_id.clone())
                            .collect::<Vec<_>>()
                    })
            });
            let new_sources: Vec<_> = bound
                .native
                .steps
                .iter()
                .map(|s| s.source_lane_id.clone())
                .collect();
            let unique: std::collections::HashSet<_> = new_sources.iter().collect();
            if unique.len() != new_sources.len() {
                return Err("native GPS corridor revisits a source lane; occurrence-aware execution is required before this loop can be driven".into());
            }
            if old_remaining
                .as_ref()
                .is_some_and(|old| !new_sources.starts_with(old))
            {
                return Err("replacement map changes the committed corridor; stop and explicitly replan before using a different branch".into());
            }
            nav.corridor = Some(bound);
            nav.next_boundary_checkpoint = resolved.next_boundary_checkpoint;
            nav.corridor_consumed_base = nav.consumed;
            let mode = nav.bind_mode(&settings)?;
            context = Some(crate::PlanContext {
                mode,
                region: nav.map.region.clone(),
                terminal: nav.corridor.clone(),
                road_boundary: nav.map.road_boundary.clone(),
            });
            nav.dirty = false;
        }
        let bound = nav
            .corridor
            .as_ref()
            .ok_or("native GPS corridor unavailable")?;
        if !bound.runtime_lane_ids.contains(&matched.lane_id) {
            return Err("vehicle left the native GPS corridor".into());
        }
        let terminal = nav.terminal_distance(matched.lane_id, matched.s);
        nav.diagnostic=Some(NavigationDiagnostic {
            frame_id:ego.header.frame_id,source:"native_gps".into(),map:nav.map.provenance.clone(),
            gps_node_uids:nav.gps.clone(),consumed_prefix_nodes:nav.consumed,
            corridor_source_ids:bound.native.steps.iter().map(|s|s.source_lane_id.clone()).collect(),
            corridor_runtime_ids:bound.runtime_lane_ids.clone(),allowed_channels:bound.native.allowed_channels.clone(),
            next_boundary_checkpoint:nav.next_boundary_checkpoint,terminal_distance_m:terminal,
            reason:if nav.next_boundary_checkpoint.is_some(){"known remote GPS suffix; stop at verified local terminal until a compatible next region is installed"}else{"GPS corridor bound to this map window"}.into(),
        });
        if let Some(context) = context {
            self.apply_plan_context(context)?;
            self.generation = self.generation.wrapping_add(1);
            self.planner_context_pending = true;
            self.last_request = None;
            self.last_plan = None;
        }
        Ok(matched)
    }
    pub(crate) fn native_stop_distance(&self, lane: etsads_map::matcher::LaneState) -> Option<f64> {
        let c = self.native_terminal.as_ref()?;
        let index = c
            .runtime_lane_ids
            .iter()
            .position(|id| *id == lane.lane_id)?;
        let steps = &c.native.steps;
        Some(
            steps[index].exit_s_m - lane.s
                + steps[index + 1..]
                    .iter()
                    .map(|s| s.exit_s_m - s.entry_s_m)
                    .sum::<f64>()
                - self
                    .calibration
                    .geometry
                    .resolved_envelope()
                    .tractor_front_from_rear_axle_m
                - 3.0,
        )
    }
}
