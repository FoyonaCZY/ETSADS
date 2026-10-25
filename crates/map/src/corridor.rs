//! Native navigation route -> constrained lane corridor. This proves topology
//! and explicit native permissions, not physical boundaries or dynamic safety.
use crate::importer::{ExportedMap, ImportedGraph};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BinaryHeap, HashMap};
use std::path::Path;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeLaneReference {
    pub source_lane_id: String,
    pub owner_uid: u64,
    /// Zero is an internal/unknown endpoint, never a native route checkpoint.
    pub node_start: u64,
    pub node_end: u64,
    pub length_m: f64,
    pub truck_access: Option<bool>,
    pub centerline_validated: bool,
    pub physical_boundaries_verified: bool,
    pub successors: Vec<String>,
    pub left: Option<String>,
    pub right: Option<String>,
    pub can_change_left: bool,
    pub can_change_right: bool,
    /// A same-direction neighbour whose road template records no lane-change
    /// rule (neither allowed nor forbidden).
    #[serde(default)]
    pub unmarked_change_left: bool,
    #[serde(default)]
    pub unmarked_change_right: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeRouteIndex {
    #[serde(default)]
    pub source_id: Option<String>,
    /// Endpoints present in the loaded source window, before authorization
    /// filtering. Distinguishes unapproved lanes from genuinely remote nodes.
    #[serde(default)]
    pub available_node_uids: Vec<u64>,
    pub lanes: BTreeMap<String, NativeLaneReference>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CorridorPolicy {
    /// Positive distance-equivalent cost discourages unnecessary lateral
    /// manoeuvres independently of the longitudinal distance they consume.
    #[serde(default = "default_change_penalty")]
    pub lane_change_penalty_m: f64,
    /// Planning distance; execution must separately verify curvature, clearance,
    /// swept bounds and the dynamic gap. No native geometry is invented here.
    pub minimum_lane_change_distance_m: f64,
    pub exit_buffer_m: f64,
    pub maximum_lane_changes: usize,
    pub maximum_search_states: usize,
    pub require_validated_centerline: bool,
    /// Lanes closed to AI trucks only (service-area roads the game's own GPS
    /// routes the player through) are usable at this extra cost per lane;
    /// `None` keeps them forbidden. Unknown access is never usable.
    #[serde(default)]
    pub ai_restricted_penalty_m: Option<f64>,
    /// Changes between neighbours whose rule is unrecorded are usable at
    /// this extra cost; `None` keeps them forbidden. An explicitly forbidden
    /// change never is. r13 stopped before a right turn: the turn lane was
    /// the unmarked neighbour of the truck's lane.
    #[serde(default)]
    pub unmarked_change_penalty_m: Option<f64>,
}
fn default_change_penalty() -> f64 {
    20.0
}
impl Default for CorridorPolicy {
    fn default() -> Self {
        Self {
            lane_change_penalty_m: default_change_penalty(),
            minimum_lane_change_distance_m: 40.0,
            exit_buffer_m: 15.0,
            maximum_lane_changes: 4,
            maximum_search_states: 50_000,
            require_validated_centerline: true,
            ai_restricted_penalty_m: None,
            unmarked_change_penalty_m: None,
        }
    }
}
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum CorridorError {
    #[error(
        "native route checkpoint {node_uid} is loaded but not authorized by the current region policy"
    )]
    UnauthorizedNativeCheckpoint { node_uid: u64 },
    #[error("native route references unknown source node UID {node_uid}")]
    UnknownNativeUid { node_uid: u64 },
    #[error("native route checkpoint pair has no directed source connection: {from} -> {to}")]
    DisconnectedNativeNodes { from: u64, to: u64 },
    #[error("invalid native route: {0}")]
    Invalid(String),
    #[error("native route needs unloaded node UIDs {node_uids:?}")]
    NeedsMoreMap { node_uids: Vec<u64> },
    #[error("no connected permitted corridor for the ordered native route")]
    NoCorridor,
    #[error("native route search exceeded its state budget")]
    SearchBudget,
    #[error("corridor lane is absent from the current runtime graph: {0}")]
    MissingRuntimeLane(String),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelKind {
    Successor,
    ChangeLeft,
    ChangeRight,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LaneChangeConstraint {
    pub earliest_start_s_m: f64,
    pub latest_start_s_m: f64,
    pub planned_completion_s_m: f64,
    pub minimum_distance_m: f64,
    pub requires_verified_physical_boundaries: bool,
    pub requires_dynamic_gap: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AllowedChannel {
    pub from_source_lane: String,
    pub to_source_lane: String,
    pub kind: ChannelKind,
    /// Number of requested native checkpoints consumed before this transition.
    pub checkpoint_index: usize,
    pub lane_change: Option<LaneChangeConstraint>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CorridorStep {
    pub source_lane_id: String,
    pub owner_uid: u64,
    pub node_start: u64,
    pub node_end: u64,
    pub entry_s_m: f64,
    pub exit_s_m: f64,
    pub checkpoint_index: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeCorridor {
    pub policy: CorridorPolicy,
    pub ordered_node_uids: Vec<u64>,
    pub steps: Vec<CorridorStep>,
    /// These are the chosen path's only allowed exits. Other graph successors
    /// must not be substituted when following this game navigation route.
    pub allowed_channels: Vec<AllowedChannel>,
    pub terminal_lane_s_m: f64,
    pub all_physical_boundaries_verified: bool,
    pub topology_only: bool,
}
#[derive(Clone, Debug)]
pub struct BoundCorridor {
    pub native: NativeCorridor,
    pub runtime_lane_ids: Vec<u64>,
    pub native_lanes: BTreeMap<u64, NativeLaneReference>,
}
#[derive(Clone, Debug)]
pub struct GlobalNativeRouteIndex {
    pub lane_index_sha256: String,
    pub source_manifest_sha256: String,
    pub source_cache_key: String,
    next: BTreeMap<u64, Vec<u64>>,
}
#[derive(Deserialize)]
struct GlobalDocument {
    schema_version: u32,
    source_manifest_sha256: String,
    source_cache_key: String,
    topology_only: bool,
    nodes: Vec<GlobalNode>,
}
#[derive(Deserialize)]
struct GlobalNode {
    uid: u64,
    next: Vec<u64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LocalCorridor {
    pub corridor: NativeCorridor,
    pub next_boundary_checkpoint: Option<u64>,
    pub remaining_node_uids: Vec<u64>,
    pub must_stop_at_local_terminal: bool,
}
impl GlobalNativeRouteIndex {
    /// Parse topology whose source identity has already been checked by the
    /// caller (also useful for explicit synthetic fixtures). This is not an
    /// evidence authorization; production file loading uses load_source.
    pub fn from_json(
        json: &str,
        expected_manifest: &str,
        expected_cache_key: &str,
        expected_lane_index_sha256: &str,
    ) -> Result<Self, CorridorError> {
        if expected_lane_index_sha256.len() != 64
            || !expected_lane_index_sha256
                .bytes()
                .all(|b| b.is_ascii_hexdigit())
        {
            return Err(CorridorError::Invalid(
                "invalid expected lane index digest".into(),
            ));
        }
        let mut index = Self::from_bytes(json.as_bytes(), expected_manifest, expected_cache_key)?;
        index.lane_index_sha256 = expected_lane_index_sha256.to_ascii_lowercase();
        Ok(index)
    }
    /// Read the compact native-node topology distributed with a sector map.
    /// Checks the lane index identity and checksum before accepting its nodes.
    pub fn load_source(path: impl AsRef<Path>) -> Result<Self, CorridorError> {
        let root = path
            .as_ref()
            .canonicalize()
            .map_err(|e| CorridorError::Invalid(e.to_string()))?;
        let index = crate::annotations::read_index(&root)
            .map_err(|e| CorridorError::Invalid(e.to_string()))?;
        let bytes = crate::annotations::checked_compressed(
            &root,
            &index.native_route_index,
            256 * 1024 * 1024,
        )
        .map_err(|e| CorridorError::Invalid(e.to_string()))?;
        let mut result = Self::from_bytes(
            &bytes,
            &index.source_manifest_sha256,
            &index.source_cache_key,
        )?;
        result.lane_index_sha256 = index.lane_index_sha256;
        Ok(result)
    }
    fn from_bytes(
        bytes: &[u8],
        expected_manifest: &str,
        expected_cache_key: &str,
    ) -> Result<Self, CorridorError> {
        let document: GlobalDocument =
            serde_json::from_slice(bytes).map_err(|e| CorridorError::Invalid(e.to_string()))?;
        if document.schema_version != 1
            || !document.topology_only
            || document.source_manifest_sha256 != expected_manifest
            || document.source_cache_key != expected_cache_key
            || document.nodes.is_empty()
            || document.nodes.len() > 2_000_000
        {
            return Err(CorridorError::Invalid(
                "invalid/global native topology source identity or count".into(),
            ));
        }
        let mut next = BTreeMap::new();
        let mut edges = 0usize;
        for node in document.nodes {
            edges = edges.saturating_add(node.next.len());
            if node.uid == 0
                || node.next.len() > 4096
                || edges > 8_000_000
                || node.next.contains(&0)
                || node.next.contains(&node.uid)
                || node.next.windows(2).any(|w| w[0] >= w[1])
                || next.insert(node.uid, node.next).is_some()
            {
                return Err(CorridorError::Invalid(
                    "malformed/duplicate global native node".into(),
                ));
            }
        }
        if next.values().flatten().any(|id| !next.contains_key(id)) {
            return Err(CorridorError::Invalid("dangling global native node".into()));
        }
        Ok(Self {
            lane_index_sha256: String::new(),
            source_manifest_sha256: document.source_manifest_sha256,
            source_cache_key: document.source_cache_key,
            next,
        })
    }
    pub fn node_count(&self) -> usize {
        self.next.len()
    }
    pub fn contains(&self, node_uid: u64) -> bool {
        self.next.contains_key(&node_uid)
    }
    /// Coarse source topology only. Exact lane direction, native permission and
    /// all physical/dynamic checks remain local corridor/execution obligations.
    pub fn validate_route(&self, route: &[u64]) -> Result<Vec<u64>, CorridorError> {
        if route.is_empty() || route.len() > 4096 {
            return Err(CorridorError::Invalid(
                "empty/oversized native route".into(),
            ));
        }
        let mut route = route.to_vec();
        route.dedup();
        for node in &route {
            if !self.next.contains_key(node) {
                return Err(CorridorError::UnknownNativeUid { node_uid: *node });
            }
        }
        for pair in route.windows(2) {
            if !self.next[&pair[0]].contains(&pair[1]) {
                return Err(CorridorError::DisconnectedNativeNodes {
                    from: pair[0],
                    to: pair[1],
                });
            }
        }
        Ok(route)
    }
    /// As `validate_route`, but a node the exported map does not know ends
    /// the route there instead of rejecting all of it: the route is driven
    /// up to that node, which is returned as a boundary the truck must stop
    /// before. One unexported node hundreds of kilometres ahead (a ferry, a
    /// newer map item) left the assist unavailable for the whole trip (live).
    pub fn validate_prefix(&self, route: &[u64]) -> Result<(Vec<u64>, Option<u64>), CorridorError> {
        let mut deduped = route.to_vec();
        deduped.dedup();
        match deduped.iter().position(|n| !self.next.contains_key(n)) {
            Some(0) => Err(CorridorError::UnknownNativeUid {
                node_uid: deduped[0],
            }),
            Some(cut) => Ok((self.validate_route(&deduped[..cut])?, Some(deduped[cut]))),
            None => Ok((self.validate_route(&deduped)?, None)),
        }
    }
}
impl NativeRouteIndex {
    pub fn for_region_graph(
        &self,
        region: &crate::importer::RegionGraph,
    ) -> Result<Self, CorridorError> {
        let bound = self.bind(&region.imported)?;
        let approved: std::collections::HashSet<_> = region
            .imported
            .runtime_to_source
            .values()
            .cloned()
            .collect();
        let mut result = self.clone();
        result.lanes.retain(|id, _| approved.contains(id));
        for (runtime, native) in bound {
            let lane = result.lanes.get_mut(&native.source_lane_id).unwrap();
            let graph = &region.imported.graph.lanes[&runtime];
            let source_of = |id: u64| region.imported.runtime_to_source.get(&id).cloned();
            let successors: Vec<_> = graph
                .successors
                .iter()
                .map(|id| {
                    source_of(*id).ok_or_else(|| {
                        CorridorError::Invalid("approved graph has unmapped successor".into())
                    })
                })
                .collect::<Result<_, _>>()?;
            if successors.iter().any(|id| !lane.successors.contains(id)) {
                return Err(CorridorError::Invalid(
                    "approved graph added a nonnative successor".into(),
                ));
            }
            lane.successors = successors;
            let left = graph.left.and_then(source_of);
            let right = graph.right.and_then(source_of);
            if left.is_some() && left != lane.left || right.is_some() && right != lane.right {
                return Err(CorridorError::Invalid(
                    "approved graph changed native neighbor identity".into(),
                ));
            }
            lane.left = left;
            lane.right = right;
            lane.can_change_left &= lane.left.is_some() && graph.can_change_left;
            lane.can_change_right &= lane.right.is_some() && graph.can_change_right;
        }
        Ok(result)
    }
    pub fn from_region(region: &crate::sector::RegionLoad) -> Result<Self, CorridorError> {
        let mut index = Self::from_exported(&region.map)?;
        index.source_id = Some(region.source_id.clone());
        Ok(index)
    }
    /// Validate the entire GPS route against the same-source native topology,
    /// then execute only a complete local prefix. A remote suffix always has
    /// an explicit stopping boundary; it can never count as route completion.
    pub fn resolve_local_prefix(
        &self,
        global: &GlobalNativeRouteIndex,
        route: &[u64],
        start_source: &str,
        start_s_m: f64,
        policy: &CorridorPolicy,
    ) -> Result<LocalCorridor, CorridorError> {
        if self.source_id.as_deref() != Some(global.lane_index_sha256.as_str())
            || global.lane_index_sha256.is_empty()
        {
            return Err(CorridorError::Invalid(
                "local/global map source identity mismatch; build native index with from_region"
                    .into(),
            ));
        }
        let (route, unknown) = global.validate_prefix(route)?;
        let local: std::collections::HashSet<_> = self
            .lanes
            .values()
            .flat_map(|l| [l.node_start, l.node_end])
            .filter(|n| *n != 0)
            .collect();
        let count = route.iter().take_while(|n| local.contains(n)).count();
        if let Some(node) = route
            .get(count)
            .filter(|n| self.available_node_uids.contains(n))
        {
            return Err(CorridorError::UnauthorizedNativeCheckpoint { node_uid: *node });
        }
        if count == 0 {
            return Err(CorridorError::NeedsMoreMap {
                node_uids: vec![route[0]],
            });
        }
        let corridor = self.resolve(&route[..count], start_source, start_s_m, policy)?;
        let boundary = route.get(count).copied().or(unknown);
        if let Some(next) = boundary
            && corridor
                .steps
                .iter()
                .map(|s| (s.exit_s_m - s.entry_s_m).max(0.0))
                .sum::<f64>()
                < 0.001
        {
            return Err(CorridorError::NeedsMoreMap {
                node_uids: vec![next],
            });
        }
        Ok(LocalCorridor {
            corridor,
            next_boundary_checkpoint: boundary,
            remaining_node_uids: route[count..].iter().copied().chain(unknown).collect(),
            must_stop_at_local_terminal: boundary.is_some(),
        })
    }
    pub fn from_exported(map: &ExportedMap) -> Result<Self, CorridorError> {
        let mut lanes = BTreeMap::new();
        for l in &map.lanes {
            let length = l
                .center
                .windows(2)
                .map(|p| p[0].distance_xy(p[1]))
                .sum::<f64>();
            if l.id.is_empty()
                || l.center.len() < 2
                || !l.center.iter().all(|p| p.valid())
                || !length.is_finite()
                || length <= 0.001
                || (l.can_change_left
                    && (l.left.is_none() || l.left_change_allowed_raw != Some(true)))
                || (l.can_change_right
                    && (l.right.is_none() || l.right_change_allowed_raw != Some(true)))
            {
                return Err(CorridorError::Invalid(format!("malformed lane {}", l.id)));
            }
            let native = NativeLaneReference {
                source_lane_id: l.id.clone(),
                owner_uid: l.owner_uid,
                node_start: l.node_start,
                node_end: l.node_end,
                length_m: length,
                truck_access: l.truck_access,
                centerline_validated: l.centerline_validated,
                physical_boundaries_verified: l.physical_boundaries_verified,
                successors: l.successors.clone(),
                left: l.left.clone(),
                right: l.right.clone(),
                can_change_left: l.can_change_left,
                can_change_right: l.can_change_right,
                unmarked_change_left: l.left.is_some() && l.left_change_allowed_raw.is_none(),
                unmarked_change_right: l.right.is_some() && l.right_change_allowed_raw.is_none(),
            };
            if lanes.insert(l.id.clone(), native).is_some() {
                return Err(CorridorError::Invalid("duplicate source lane".into()));
            }
        }
        for l in lanes.values() {
            if l.successors
                .iter()
                .chain(l.left.iter())
                .chain(l.right.iter())
                .any(|id| id == &l.source_lane_id || !lanes.contains_key(id))
            {
                return Err(CorridorError::Invalid("dangling or self lane edge".into()));
            }
        }
        Ok(Self {
            available_node_uids: lanes
                .values()
                .flat_map(|l| [l.node_start, l.node_end])
                .filter(|n| *n != 0)
                .collect::<std::collections::BTreeSet<_>>()
                .into_iter()
                .collect(),
            lanes,
            source_id: None,
        })
    }
    /// Preserve source identities and endpoints when binding a freshly loaded
    /// region; runtime IDs must not be carried across regional re-imports.
    pub fn bind(
        &self,
        imported: &ImportedGraph,
    ) -> Result<BTreeMap<u64, NativeLaneReference>, CorridorError> {
        imported
            .runtime_to_source
            .iter()
            .map(|(id, source)| {
                let lane = self
                    .lanes
                    .get(source)
                    .ok_or_else(|| CorridorError::MissingRuntimeLane(source.clone()))?;
                if imported.native_owners.get(id) != Some(&lane.owner_uid)
                    || !imported.graph.lanes.contains_key(id)
                {
                    return Err(CorridorError::Invalid(
                        "runtime/source identity mismatch".into(),
                    ));
                }
                Ok((*id, lane.clone()))
            })
            .collect()
    }
    /// `route` begins at the current or next native checkpoint. The caller must
    /// trim already-passed route nodes using observed route progress, not guess.
    /// Adjacent duplicates collapse; nonadjacent repeats retain loop semantics.
    pub fn resolve(
        &self,
        route: &[u64],
        start_source: &str,
        start_s_m: f64,
        policy: &CorridorPolicy,
    ) -> Result<NativeCorridor, CorridorError> {
        if route.is_empty()
            || route.len() > 4096
            || route.contains(&0)
            || !start_s_m.is_finite()
            || start_s_m < 0.0
            || !policy.minimum_lane_change_distance_m.is_finite()
            || policy.minimum_lane_change_distance_m <= 0.0
            || !policy.lane_change_penalty_m.is_finite()
            || policy.lane_change_penalty_m <= 0.0
            || policy.lane_change_penalty_m > 1_000_000.0
            || !policy.exit_buffer_m.is_finite()
            || policy.exit_buffer_m < 0.0
            || policy.maximum_lane_changes > 32
            || !(1..=1_000_000).contains(&policy.maximum_search_states)
        {
            return Err(CorridorError::Invalid("invalid route/policy/start".into()));
        }
        let mut route = route.to_vec();
        route.dedup();
        let endpoints: std::collections::HashSet<_> = self
            .lanes
            .values()
            .flat_map(|l| [l.node_start, l.node_end])
            .filter(|n| *n != 0)
            .collect();
        let missing: Vec<_> = route
            .iter()
            .copied()
            .filter(|n| !endpoints.contains(n))
            .collect();
        if !missing.is_empty() {
            return Err(CorridorError::NeedsMoreMap { node_uids: missing });
        }
        let names: Vec<_> = self.lanes.keys().cloned().collect();
        let ids: HashMap<_, _> = names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.as_str(), i))
            .collect();
        let start = *ids
            .get(start_source)
            .ok_or_else(|| CorridorError::MissingRuntimeLane(start_source.into()))?;
        let first = &self.lanes[start_source];
        if start_s_m > first.length_m || !eligible(first, policy) {
            return Err(CorridorError::NoCorridor);
        }
        // The live GPS may retain the previous junction after the truck has
        // entered the next road. Anchor only to an exact, uniquely occurring
        // directed start/end pair; never skip ahead by nearest-node distance.
        let anchors: Vec<_> = route
            .windows(2)
            .enumerate()
            .filter(|(_, p)| {
                first.node_start != 0
                    && first.node_end != 0
                    && p[0] == first.node_start
                    && p[1] == first.node_end
            })
            .map(|(i, _)| i)
            .collect();
        if anchors.len() > 1 && anchors[0] != 0 {
            return Err(CorridorError::Invalid(
                "starting road occurs more than once in GPS; occurrence is ambiguous".into(),
            ));
        }
        let initial_checkpoint = anchors
            .first()
            .map_or_else(|| usize::from(first.node_start == route[0]), |i| i + 1);
        // Initial start can be behind the vehicle. Subsequent endpoints must
        // match the exact requested order; shared start/end is consumed once.
        let key = State {
            lane: start,
            next: initial_checkpoint,
            last: first.node_start,
            s_mm: (start_s_m * 1000.0).ceil() as u64,
            changes: 0,
        };
        let mut records = vec![Record {
            key,
            cost: 0.0,
            parent: None,
            via: None,
        }];
        let mut best = HashMap::from([(key, (0.0, 0usize))]);
        let mut queue = BinaryHeap::from([std::cmp::Reverse((0u64, 0usize))]);
        let mut goal: Option<(f64, usize, f64)> = None;
        while let Some(std::cmp::Reverse((_, rid))) = queue.pop() {
            let state = records[rid].key;
            let cost = records[rid].cost;
            if best.get(&state).is_none_or(|(_, i)| *i != rid) {
                continue;
            }
            // Queue keys are costs to lane entry, not to its end. A terminal
            // can only win once no remaining entry can improve its full cost.
            if goal.is_some_and(|(total, _, _)| cost >= total) {
                break;
            }
            let lane = &self.lanes[&names[state.lane]];
            let entry = (state.s_mm as f64 / 1000.0).min(lane.length_m);
            if state.next == route.len() {
                goal = Some((cost, rid, entry));
                continue;
            }
            if let Some((next, last)) = consume(lane.node_end, state.next, state.last, &route) {
                if next == route.len() {
                    let terminal_cost = cost + lane.length_m - entry;
                    if goal.is_none_or(|(total, _, _)| terminal_cost < total) {
                        goal = Some((terminal_cost, rid, lane.length_m));
                    }
                } else {
                    for successor in &lane.successors {
                        let target = &self.lanes[successor];
                        if !eligible(target, policy) {
                            continue;
                        }
                        let Some((next, last)) = consume(target.node_start, next, last, &route)
                        else {
                            continue;
                        };
                        let next_state = State {
                            lane: ids[successor.as_str()],
                            next,
                            last,
                            s_mm: 0,
                            changes: state.changes,
                        };
                        push(
                            next_state,
                            cost + lane.length_m - entry + restricted_cost(target, policy),
                            rid,
                            ChannelKind::Successor,
                            &mut records,
                            &mut best,
                            &mut queue,
                            policy,
                        )?;
                    }
                }
            }
            if state.changes >= policy.maximum_lane_changes {
                continue;
            }
            for (neighbor, allowed, unmarked, kind) in [
                (
                    &lane.left,
                    lane.can_change_left,
                    lane.unmarked_change_left,
                    ChannelKind::ChangeLeft,
                ),
                (
                    &lane.right,
                    lane.can_change_right,
                    lane.unmarked_change_right,
                    ChannelKind::ChangeRight,
                ),
            ] {
                let unmarked_cost = match (allowed, unmarked, policy.unmarked_change_penalty_m) {
                    (true, _, _) => 0.0,
                    (false, true, Some(cost)) => cost,
                    _ => continue,
                };
                let Some(name) = neighbor.as_ref() else {
                    continue;
                };
                let target = &self.lanes[name];
                // Only same-road, same-direction native lane pairs have a
                // meaningful shared longitudinal fraction. Internal curves do
                // not acquire a fabricated lateral connection.
                if !eligible(target, policy)
                    || lane.owner_uid != target.owner_uid
                    || lane.node_start == 0
                    || lane.node_end == 0
                    || lane.node_start != target.node_start
                    || lane.node_end != target.node_end
                {
                    continue;
                }
                let fraction = entry / lane.length_m;
                let completion = fraction
                    + policy.minimum_lane_change_distance_m / lane.length_m.min(target.length_m);
                let buffer = policy.exit_buffer_m / lane.length_m.min(target.length_m);
                if completion + buffer > 1.0 {
                    continue;
                }
                let next_state = State {
                    lane: ids[name.as_str()],
                    next: state.next,
                    last: state.last,
                    s_mm: (completion * target.length_m * 1000.0).ceil() as u64,
                    changes: state.changes + 1,
                };
                let target_completion = next_state.s_mm as f64 / 1000.0;
                let source_advance = target_completion / target.length_m * lane.length_m - entry;
                let target_advance = target_completion - fraction * target.length_m;
                push(
                    next_state,
                    cost + source_advance.max(target_advance)
                        + policy.lane_change_penalty_m
                        + unmarked_cost
                        + restricted_cost(target, policy),
                    rid,
                    kind,
                    &mut records,
                    &mut best,
                    &mut queue,
                    policy,
                )?;
            }
        }
        let (_, goal, terminal) = goal.ok_or(CorridorError::NoCorridor)?;
        let mut chain = vec![goal];
        while let Some(parent) = records[*chain.last().unwrap()].parent {
            chain.push(parent);
        }
        chain.reverse();
        let mut steps = Vec::new();
        let mut channels = Vec::new();
        for (i, rid) in chain.iter().enumerate() {
            let state = records[*rid].key;
            let lane = &self.lanes[&names[state.lane]];
            let entry = (state.s_mm as f64 / 1000.0).min(lane.length_m);
            let mut exit = if i + 1 == chain.len() {
                terminal
            } else {
                lane.length_m
            };
            if let Some(next_id) = chain.get(i + 1) {
                let next = &records[*next_id];
                let target = &self.lanes[&names[next.key.lane]];
                let kind = next.via.unwrap();
                let change = if kind != ChannelKind::Successor {
                    // Reserve distance for every consecutive required lateral
                    // transition before this road's exit; deadline is in this
                    // source lane's native arc length.
                    let mut fraction_cost = 0.0;
                    let mut minimum_length = lane.length_m;
                    for pair in chain[i..].windows(2) {
                        if records[pair[1]].via == Some(ChannelKind::Successor) {
                            break;
                        }
                        let a = &self.lanes[&names[records[pair[0]].key.lane]];
                        let b = &self.lanes[&names[records[pair[1]].key.lane]];
                        minimum_length = minimum_length.min(a.length_m).min(b.length_m);
                        fraction_cost +=
                            policy.minimum_lane_change_distance_m / a.length_m.min(b.length_m);
                    }
                    let latest = (1.0 - fraction_cost - policy.exit_buffer_m / minimum_length)
                        * lane.length_m;
                    let completion =
                        (next.key.s_mm as f64 / 1000.0) / target.length_m * lane.length_m;
                    if entry > latest + 0.002 {
                        return Err(CorridorError::NoCorridor);
                    }
                    exit = completion;
                    Some(LaneChangeConstraint {
                        earliest_start_s_m: entry,
                        latest_start_s_m: latest,
                        planned_completion_s_m: completion,
                        minimum_distance_m: policy.minimum_lane_change_distance_m,
                        requires_verified_physical_boundaries: true,
                        requires_dynamic_gap: true,
                    })
                } else {
                    None
                };
                channels.push(AllowedChannel {
                    from_source_lane: lane.source_lane_id.clone(),
                    to_source_lane: target.source_lane_id.clone(),
                    kind,
                    checkpoint_index: state.next,
                    lane_change: change,
                });
            }
            steps.push(CorridorStep {
                source_lane_id: lane.source_lane_id.clone(),
                owner_uid: lane.owner_uid,
                node_start: lane.node_start,
                node_end: lane.node_end,
                entry_s_m: entry,
                exit_s_m: exit,
                checkpoint_index: state.next,
            });
        }
        Ok(NativeCorridor {
            policy: policy.clone(),
            ordered_node_uids: route,
            all_physical_boundaries_verified: steps
                .iter()
                .all(|s| self.lanes[&s.source_lane_id].physical_boundaries_verified),
            steps,
            allowed_channels: channels,
            terminal_lane_s_m: terminal,
            topology_only: true,
        })
    }
}
impl NativeCorridor {
    pub fn bind(
        self,
        index: &NativeRouteIndex,
        imported: &ImportedGraph,
    ) -> Result<BoundCorridor, CorridorError> {
        let reverse: HashMap<_, _> = imported
            .runtime_to_source
            .iter()
            .map(|(id, s)| (s.as_str(), *id))
            .collect();
        let runtime_lane_ids = self
            .steps
            .iter()
            .map(|s| {
                reverse
                    .get(s.source_lane_id.as_str())
                    .copied()
                    .ok_or_else(|| CorridorError::MissingRuntimeLane(s.source_lane_id.clone()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        for channel in &self.allowed_channels {
            let from = reverse
                .get(channel.from_source_lane.as_str())
                .ok_or_else(|| {
                    CorridorError::MissingRuntimeLane(channel.from_source_lane.clone())
                })?;
            let to = reverse
                .get(channel.to_source_lane.as_str())
                .ok_or_else(|| CorridorError::MissingRuntimeLane(channel.to_source_lane.clone()))?;
            let lane = imported.graph.lanes.get(from).ok_or_else(|| {
                CorridorError::MissingRuntimeLane(channel.from_source_lane.clone())
            })?;
            let permitted = match channel.kind {
                ChannelKind::Successor => lane.successors.contains(to),
                ChannelKind::ChangeLeft => lane.left == Some(*to) && lane.can_change_left,
                ChannelKind::ChangeRight => lane.right == Some(*to) && lane.can_change_right,
            };
            if !permitted {
                return Err(CorridorError::Invalid(
                    "current runtime graph does not permit a corridor channel".into(),
                ));
            }
        }
        Ok(BoundCorridor {
            native: self,
            runtime_lane_ids,
            native_lanes: index.bind(imported)?,
        })
    }
}
fn eligible(l: &NativeLaneReference, p: &CorridorPolicy) -> bool {
    (l.truck_access == Some(true)
        || (l.truck_access == Some(false) && p.ai_restricted_penalty_m.is_some()))
        && (!p.require_validated_centerline || l.centerline_validated)
}
/// Extra search cost of entering a lane that AI trucks may not use.
fn restricted_cost(l: &NativeLaneReference, p: &CorridorPolicy) -> f64 {
    if l.truck_access == Some(true) {
        0.0
    } else {
        p.ai_restricted_penalty_m.unwrap_or(0.0)
    }
}
fn consume(node: u64, next: usize, last: u64, route: &[u64]) -> Option<(usize, u64)> {
    if node == 0 || node == last {
        return Some((next, last));
    }
    (route.get(next) == Some(&node)).then_some((next + 1, node))
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct State {
    lane: usize,
    next: usize,
    last: u64,
    s_mm: u64,
    changes: usize,
}
struct Record {
    key: State,
    cost: f64,
    parent: Option<usize>,
    via: Option<ChannelKind>,
}
#[allow(clippy::too_many_arguments)]
fn push(
    key: State,
    cost: f64,
    parent: usize,
    via: ChannelKind,
    records: &mut Vec<Record>,
    best: &mut HashMap<State, (f64, usize)>,
    queue: &mut BinaryHeap<std::cmp::Reverse<(u64, usize)>>,
    policy: &CorridorPolicy,
) -> Result<(), CorridorError> {
    if best.get(&key).is_some_and(|(old, _)| *old <= cost) {
        return Ok(());
    }
    if records.len() >= policy.maximum_search_states {
        return Err(CorridorError::SearchBudget);
    }
    let id = records.len();
    records.push(Record {
        key,
        cost,
        parent: Some(parent),
        via: Some(via),
    });
    best.insert(key, (cost, id));
    queue.push(std::cmp::Reverse((cost.to_bits(), id)));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::importer::ExportLane;
    use etsads_msgs::Point3;
    fn lane(id: &str, owner: u64, start: u64, end: u64, len: f64, next: &[&str]) -> ExportLane {
        ExportLane {
            id: id.into(),
            owner_uid: owner,
            node_start: start,
            node_end: end,
            center: vec![
                Point3::default(),
                Point3 {
                    x: len,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            truck_access: Some(true),
            centerline_validated: true,
            successors: next.iter().map(|s| s.to_string()).collect(),
            ..Default::default()
        }
    }
    fn index(lanes: Vec<ExportLane>) -> NativeRouteIndex {
        NativeRouteIndex::from_exported(&ExportedMap {
            schema_version: 1,
            accepted_for_autonomous_driving: false,
            source: "synthetic_test_only".into(),
            lanes,
        })
        .unwrap()
    }
    #[test]
    fn gps_retained_previous_checkpoint_anchors_to_observed_directed_road() {
        let idx = index(vec![
            lane("a", 1, 20, 30, 100.0, &["b"]),
            lane("b", 2, 30, 40, 100.0, &[]),
            lane("previous", 3, 10, 20, 30.0, &["a"]),
        ]);
        let corridor = idx
            .resolve(&[10, 20, 30, 40], "a", 16.0, &CorridorPolicy::default())
            .unwrap();
        assert_eq!(corridor.steps[0].source_lane_id, "a");
        assert_eq!(corridor.steps[0].checkpoint_index, 2);
        assert_eq!(corridor.ordered_node_uids, vec![10, 20, 30, 40]);
        assert!(
            idx.resolve(&[10, 30, 20, 40], "a", 16.0, &CorridorPolicy::default())
                .is_err()
        );
        assert!(
            idx.resolve(
                &[20, 30, 40, 20, 30, 40],
                "a",
                16.0,
                &CorridorPolicy::default()
            )
            .is_err()
        );
        assert!(
            idx.resolve(
                &[10, 20, 30, 40, 20, 30, 40],
                "a",
                16.0,
                &CorridorPolicy::default()
            )
            .is_err()
        );
    }
    #[test]
    fn ordered_native_nodes_exclude_wrong_branch_and_preserve_u64() {
        let n = u64::MAX - 2;
        let idx = index(vec![
            lane("a", 1, n, n + 1, 100.0, &["wrong", "internal"]),
            lane("wrong", 2, n + 1, 44, 100.0, &[]),
            lane("internal", 3, n + 1, 0, 8.0, &["out"]),
            lane("out", 3, 0, n + 2, 10.0, &[]),
        ]);
        let route = idx
            .resolve(&[n, n, n + 1, n + 2], "a", 20.0, &CorridorPolicy::default())
            .unwrap();
        assert_eq!(
            route
                .steps
                .iter()
                .map(|s| s.source_lane_id.as_str())
                .collect::<Vec<_>>(),
            vec!["a", "internal", "out"]
        );
        assert_eq!(route.ordered_node_uids, vec![n, n + 1, n + 2]);
        assert_eq!(route.steps[0].node_start, n);
        assert!(route.topology_only);
        assert!(!route.all_physical_boundaries_verified);
    }
    #[test]
    fn mandatory_lane_change_has_deadline_and_unknown_permission_is_closed() {
        let mut a = lane("a", 1, 10, 20, 200.0, &[]);
        a.left = Some("b".into());
        a.left_change_allowed_raw = Some(true);
        a.can_change_left = true;
        let b = lane("b", 1, 10, 20, 200.0, &["exit"]);
        let exit = lane("exit", 2, 20, 30, 50.0, &[]);
        let idx = index(vec![a.clone(), b.clone(), exit.clone()]);
        let route = idx
            .resolve(&[10, 20, 30], "a", 40.0, &CorridorPolicy::default())
            .unwrap();
        let change = route.allowed_channels[0].lane_change.as_ref().unwrap();
        assert_eq!(route.allowed_channels[0].kind, ChannelKind::ChangeLeft);
        assert_eq!(change.earliest_start_s_m, 40.0);
        assert!((change.latest_start_s_m - 145.0).abs() < 1e-9);
        assert!(change.requires_dynamic_gap && change.requires_verified_physical_boundaries);
        assert_eq!(
            idx.resolve(&[10, 20, 30], "a", 146.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
        a.can_change_left = false;
        a.left_change_allowed_raw = None;
        assert_eq!(
            index(vec![a, b, exit])
                .resolve(&[10, 20, 30], "a", 0.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
    }
    #[test]
    fn missing_nodes_and_unknown_truck_access_are_explicit_errors() {
        let mut a = lane("a", 1, 1, 2, 100.0, &[]);
        let idx = index(vec![a.clone()]);
        assert_eq!(
            idx.resolve(&[1, 2, 999], "a", 0.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NeedsMoreMap {
                node_uids: vec![999]
            }
        );
        a.truck_access = None;
        assert_eq!(
            index(vec![a])
                .resolve(&[1, 2], "a", 0.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
    }
    #[test]
    fn ai_restricted_lanes_are_used_only_when_the_policy_prices_them() {
        let a = lane("a", 1, 1, 2, 100.0, &["service"]);
        let mut service = lane("service", 2, 2, 3, 50.0, &[]);
        service.truck_access = Some(false);
        let idx = index(vec![a, service]);
        assert_eq!(
            idx.resolve(&[1, 2, 3], "a", 0.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
        let priced = CorridorPolicy {
            ai_restricted_penalty_m: Some(200.0),
            ..Default::default()
        };
        let route = idx.resolve(&[1, 2, 3], "a", 0.0, &priced).unwrap();
        assert_eq!(route.steps.last().unwrap().source_lane_id, "service");
    }
    #[test]
    fn unmarked_lane_changes_are_priced_but_forbidden_ones_stay_closed() {
        // r13: the right-turn lane beside the truck's lane, no rule recorded.
        let mut a = lane("a", 1, 10, 20, 200.0, &[]);
        a.left = Some("b".into());
        let b = lane("b", 1, 10, 20, 200.0, &["exit"]);
        let exit = lane("exit", 2, 20, 30, 50.0, &[]);
        let priced = CorridorPolicy {
            unmarked_change_penalty_m: Some(100.0),
            ..Default::default()
        };
        let idx = index(vec![a.clone(), b.clone(), exit.clone()]);
        assert_eq!(
            idx.resolve(&[10, 20, 30], "a", 0.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
        let route = idx.resolve(&[10, 20, 30], "a", 0.0, &priced).unwrap();
        assert_eq!(route.allowed_channels[0].kind, ChannelKind::ChangeLeft);
        a.left_change_allowed_raw = Some(false);
        assert_eq!(
            index(vec![a, b, exit])
                .resolve(&[10, 20, 30], "a", 0.0, &priced)
                .unwrap_err(),
            CorridorError::NoCorridor
        );
    }
    #[test]
    fn nonadjacent_native_repeats_retain_loop_and_state_budget_is_bounded() {
        let idx = index(vec![
            lane("a", 1, 1, 2, 50.0, &["b"]),
            lane("b", 2, 2, 1, 50.0, &["a"]),
        ]);
        let route = idx
            .resolve(&[1, 2, 1, 2], "a", 0.0, &CorridorPolicy::default())
            .unwrap();
        assert_eq!(
            route
                .steps
                .iter()
                .map(|s| s.source_lane_id.as_str())
                .collect::<Vec<_>>(),
            vec!["a", "b", "a"]
        );
        let p = CorridorPolicy {
            maximum_search_states: 1,
            ..Default::default()
        };
        assert_eq!(
            idx.resolve(&[1, 2, 1], "a", 0.0, &p).unwrap_err(),
            CorridorError::SearchBudget
        );
    }
    #[test]
    fn two_required_lateral_moves_reserve_both_distances_before_exit() {
        let mut a = lane("a", 1, 1, 2, 200.0, &[]);
        let mut b = lane("b", 1, 1, 2, 200.0, &[]);
        let c = lane("c", 1, 1, 2, 200.0, &["exit"]);
        let exit = lane("exit", 2, 2, 3, 30.0, &[]);
        for (l, n) in [(&mut a, "b"), (&mut b, "c")] {
            l.left = Some(n.into());
            l.can_change_left = true;
            l.left_change_allowed_raw = Some(true);
        }
        let idx = index(vec![a, b, c, exit]);
        let r = idx
            .resolve(&[1, 2, 3], "a", 0.0, &CorridorPolicy::default())
            .unwrap();
        assert!(
            (r.allowed_channels[0]
                .lane_change
                .as_ref()
                .unwrap()
                .latest_start_s_m
                - 105.0)
                .abs()
                < 1e-9
        );
        assert_eq!(
            idx.resolve(&[1, 2, 3], "a", 110.0, &CorridorPolicy::default())
                .unwrap_err(),
            CorridorError::NoCorridor
        );
    }
    #[test]
    fn straight_route_never_buys_progress_with_out_and_back_lane_changes() {
        let mut a = lane("a", 1, 1, 2, 1000.0, &["straight"]);
        let mut b = lane("b", 1, 1, 2, 1000.0, &["exit"]);
        a.left = Some("b".into());
        a.can_change_left = true;
        a.left_change_allowed_raw = Some(true);
        b.right = Some("a".into());
        b.can_change_right = true;
        b.right_change_allowed_raw = Some(true);
        let idx = index(vec![
            a,
            b,
            lane("straight", 2, 2, 3, 200.0, &[]),
            lane("exit", 3, 2, 4, 200.0, &[]),
        ]);
        for start in [20.0, 20.00001, 20.12345] {
            let straight = idx
                .resolve(&[1, 2, 3], "a", start, &CorridorPolicy::default())
                .unwrap();
            assert_eq!(
                straight
                    .steps
                    .iter()
                    .map(|s| s.source_lane_id.as_str())
                    .collect::<Vec<_>>(),
                vec!["a", "straight"]
            );
            let exit = idx
                .resolve(&[1, 2, 4], "a", start, &CorridorPolicy::default())
                .unwrap();
            assert_eq!(
                exit.steps
                    .iter()
                    .map(|s| s.source_lane_id.as_str())
                    .collect::<Vec<_>>(),
                vec!["a", "b", "exit"]
            );
        }
    }
    #[test]
    fn terminal_cost_includes_remaining_lane_distance() {
        let idx = index(vec![
            lane("start", 1, 1, 2, 10.0, &["a_long", "b_short"]),
            lane("a_long", 2, 2, 3, 100.0, &[]),
            lane("b_short", 3, 2, 0, 20.0, &["end"]),
            lane("end", 3, 0, 3, 5.0, &[]),
        ]);
        let route = idx
            .resolve(&[1, 2, 3], "start", 0.0, &CorridorPolicy::default())
            .unwrap();
        assert_eq!(
            route
                .steps
                .iter()
                .map(|s| s.source_lane_id.as_str())
                .collect::<Vec<_>>(),
            vec!["start", "b_short", "end"]
        );
    }
}
