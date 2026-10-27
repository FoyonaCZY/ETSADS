//! Traffic annotations from the offline map. Positions are ENU metres.
//!
//! Static priority is never permission to ignore moving traffic. Unknown rules,
//! missing signal observations and ambiguous spatial matches remain restrictive.
//! ETS2LA runtime IDs are not SCS prefab semaphore IDs; live lights are matched
//! to the exported control group by position after the adapter converts to ENU.
use etsads_msgs::Point3;
use serde::{Deserialize, Serialize};
use std::{collections::HashSet, path::Path};

#[derive(Debug, thiserror::Error)]
pub enum TrafficError {
    #[error("invalid traffic annotations: {0}")]
    Invalid(String),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlKind {
    Signal,
    Stop,
    Yield,
    PriorityRoad,
    Uncontrolled,
    #[default]
    #[serde(other)]
    Unknown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConflictKind {
    Cross,
    CrossSharp,
    Merge,
    Fork,
    #[default]
    #[serde(other)]
    Unknown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConflictSource {
    NativePpdGroup,
    DerivedGeometry,
    #[default]
    #[serde(other)]
    Unknown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConflictRelation {
    AYieldsB,
    BYieldsA,
    SignalStateRequired,
    ZipMerge,
    EqualPriority,
    #[default]
    #[serde(other)]
    Unknown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RightOfWay {
    HasPriority,
    Yield,
    SignalRequired,
    ZipMerge,
    EqualPriority,
    #[default]
    Unknown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemaphoreKind {
    UseProfile,
    ModelOnly,
    TrafficLight,
    TrafficLightMinor,
    TrafficLightMajor,
    BarrierManualTimed,
    BarrierDistanceActivated,
    BarrierAutomatic,
    BarrierGas,
    TrafficLightBlockable,
    TrafficLightVirtual,
    #[default]
    #[serde(other)]
    Unknown,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConflictPair {
    pub id: String,
    pub owner_uid: u64,
    pub lane_a: String,
    pub lane_b: String,
    pub kind: ConflictKind,
    pub source: ConflictSource,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub radius_m: f64,
    pub geometry_gap_m: f64,
    pub priority_a: Option<i32>,
    pub priority_b: Option<i32>,
    pub relation: ConflictRelation,
    pub conservative_wait: bool,
}
impl ConflictPair {
    pub fn position(&self) -> Point3 {
        Point3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn priority_for(&self, lane: &str) -> Option<RightOfWay> {
        let is_a = lane == self.lane_a;
        if !is_a && lane != self.lane_b {
            return None;
        }
        if self.source == ConflictSource::Unknown {
            return Some(RightOfWay::Unknown);
        }
        Some(match self.relation {
            ConflictRelation::AYieldsB if is_a => RightOfWay::Yield,
            ConflictRelation::AYieldsB => RightOfWay::HasPriority,
            ConflictRelation::BYieldsA if is_a => RightOfWay::HasPriority,
            ConflictRelation::BYieldsA => RightOfWay::Yield,
            ConflictRelation::SignalStateRequired => RightOfWay::SignalRequired,
            ConflictRelation::ZipMerge => RightOfWay::ZipMerge,
            ConflictRelation::EqualPriority => RightOfWay::EqualPriority,
            ConflictRelation::Unknown => RightOfWay::Unknown,
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LaneControl {
    pub lane_id: String,
    pub traffic_rule: String,
    pub control_kind: ControlKind,
    pub priority_modifier: Option<i32>,
    pub additive_priority: bool,
    pub signal_group_id: Option<String>,
    pub evidence_source: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Signal {
    /// A group may have several physical lamp heads sharing this ID.
    pub id: String,
    pub owner_uid: u64,
    pub semaphore_id: u32,
    #[serde(rename = "type")]
    pub kind: SemaphoreKind,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    pub profile: String,
    pub cycle_delay_s: f64,
    pub interval_0: f64,
    pub interval_1: f64,
    pub interval_2: f64,
    pub interval_3: f64,
    #[serde(default)]
    pub raw_type: Option<String>,
    #[serde(default)]
    pub raw_profile: Option<String>,
    #[serde(default)]
    pub effective_profile: Option<String>,
    #[serde(default)]
    pub profile_unit: Option<String>,
    #[serde(default)]
    pub profile_country: Option<String>,
    #[serde(default)]
    pub profile_resolution: Option<String>,
    #[serde(default)]
    pub profile_evidence_id: Option<String>,
    #[serde(default)]
    pub timing_resolved: bool,
    #[serde(default)]
    pub resolved_id_map: Option<u32>,
}
impl Signal {
    pub fn position(&self) -> Point3 {
        Point3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SignalLane {
    pub signal_id: String,
    pub lane_id: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StopLine {
    pub id: String,
    pub lane_id: String,
    pub signal_id: Option<String>,
    pub rule: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// `curve_start_proxy` is an entry target, not a certified painted line.
    pub geometry_status: String,
    pub control_kind: ControlKind,
}
impl StopLine {
    pub fn position(&self) -> Point3 {
        Point3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UpcomingStopControl {
    pub route_index: usize,
    pub lane_id: String,
    pub position: Point3,
    pub control_kind: ControlKind,
    pub signal_group_id: Option<String>,
    pub priority: RightOfWay,
    /// Offset upstream from `position`. Vehicle reference-point compensation is
    /// still the controller's responsibility.
    pub stop_before_m: f64,
    pub geometry_source: String,
    pub conflict_lane_ids: Vec<String>,
    pub requires_conservative_wait: bool,
}

/// A light seen nearer to one group's head than to any other group's by
/// at least this much (m) is that group's.
const SIGNAL_HEAD_MARGIN_M: f64 = 0.4;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SignalObservation {
    pub position: Point3,
    /// ETS2LA traffic-light state. Gate observations (kind=2) must not be passed.
    pub raw_state: i32,
    pub age_us: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "decision", content = "reason", rename_all = "snake_case")]
pub enum SignalDecision {
    Proceed,
    Hold(SignalHoldReason),
}
impl SignalDecision {
    pub fn can_proceed(&self) -> bool {
        matches!(self, Self::Proceed)
    }
}
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalHoldReason {
    UnresolvedProfile,
    NotTrafficLight,
    MissingGroup,
    MissingObservation,
    Stale,
    AmbiguousGeometry,
    ConflictingStates,
    NotGreen(i32),
    InvalidSettings,
}

/// Serializable in RuntimeConfig. Call `validate()` after deserializing that
/// config. `from_json`/`from_path` validate automatically.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrafficContext {
    pub schema_version: u32,
    pub source_lane_ids: Vec<String>,
    /// Cross-window conflict identities have no complete control annotation
    /// coverage here. They must not make has_annotations_for_route succeed.
    #[serde(default)]
    pub reference_only_lane_ids: Vec<String>,
    pub annotations_present: bool,
    pub conflicts: Vec<ConflictPair>,
    pub lane_controls: Vec<LaneControl>,
    pub signals: Vec<Signal>,
    pub signal_lanes: Vec<SignalLane>,
    pub stop_lines: Vec<StopLine>,
}
impl Default for TrafficContext {
    fn default() -> Self {
        Self {
            schema_version: 1,
            source_lane_ids: vec![],
            reference_only_lane_ids: vec![],
            annotations_present: false,
            conflicts: vec![],
            lane_controls: vec![],
            signals: vec![],
            signal_lanes: vec![],
            stop_lines: vec![],
        }
    }
}

#[derive(Deserialize)]
struct LaneReference {
    id: String,
}
#[derive(Deserialize)]
struct Document {
    schema_version: u32,
    annotations_present: Option<bool>,
    #[serde(default)]
    lanes: Vec<LaneReference>,
    #[serde(default)]
    source_lane_ids: Vec<String>,
    #[serde(default)]
    reference_only_lane_ids: Vec<String>,
    conflicts: Option<Vec<ConflictPair>>,
    lane_controls: Option<Vec<LaneControl>>,
    signals: Option<Vec<Signal>>,
    signal_lanes: Option<Vec<SignalLane>>,
    stop_lines: Option<Vec<StopLine>>,
}

impl TrafficContext {
    pub fn from_json(json: &str) -> Result<Self, TrafficError> {
        let document: Document = serde_json::from_str(json)?;
        let arrays_present = document.conflicts.is_some()
            && document.lane_controls.is_some()
            && document.signals.is_some()
            && document.signal_lanes.is_some()
            && document.stop_lines.is_some();
        let annotations_present =
            document.annotations_present.unwrap_or(arrays_present) && arrays_present;
        let mut source_lane_ids = document.source_lane_ids;
        source_lane_ids.extend(document.lanes.into_iter().map(|lane| lane.id));
        let context = Self {
            schema_version: document.schema_version,
            source_lane_ids,
            reference_only_lane_ids: document.reference_only_lane_ids,
            annotations_present,
            conflicts: document.conflicts.unwrap_or_default(),
            lane_controls: document.lane_controls.unwrap_or_default(),
            signals: document.signals.unwrap_or_default(),
            signal_lanes: document.signal_lanes.unwrap_or_default(),
            stop_lines: document.stop_lines.unwrap_or_default(),
        };
        context.validate()?;
        Ok(context)
    }

    pub fn from_path(path: impl AsRef<Path>) -> Result<Self, TrafficError> {
        Self::from_json(&std::fs::read_to_string(path)?)
    }

    pub fn validate(&self) -> Result<(), TrafficError> {
        let invalid = |text: &str| TrafficError::Invalid(text.into());
        if self.schema_version != 1 {
            return Err(invalid("unsupported schema"));
        }
        let lanes: HashSet<_> = self.source_lane_ids.iter().map(String::as_str).collect();
        if lanes.len() != self.source_lane_ids.len() || lanes.contains("") {
            return Err(invalid("duplicate or empty source lane IDs"));
        }
        let references: HashSet<_> = self.reference_only_lane_ids.iter().collect();
        if references.len() != self.reference_only_lane_ids.len()
            || references.iter().any(|id| !lanes.contains(id.as_str()))
        {
            return Err(invalid("invalid reference-only lane identity"));
        }
        let mut conflict_ids = HashSet::new();
        for pair in &self.conflicts {
            if pair.id.is_empty()
                || !conflict_ids.insert(pair.id.as_str())
                || pair.lane_a == pair.lane_b
                || !lanes.contains(pair.lane_a.as_str())
                || !lanes.contains(pair.lane_b.as_str())
                || !finite(pair.position())
                || !pair.radius_m.is_finite()
                || pair.radius_m < 0.0
                || !pair.geometry_gap_m.is_finite()
                || pair.geometry_gap_m < 0.0
            {
                return Err(invalid("invalid conflict geometry, ID or lane reference"));
            }
        }
        let mut controlled = HashSet::new();
        for control in &self.lane_controls {
            if !lanes.contains(control.lane_id.as_str())
                || !controlled.insert(control.lane_id.as_str())
            {
                return Err(invalid("unknown or duplicate control lane"));
            }
        }
        for signal in &self.signals {
            if signal.id.is_empty()
                || !finite(signal.position())
                || !signal.yaw.is_finite()
                || !signal.cycle_delay_s.is_finite()
                || [
                    signal.interval_0,
                    signal.interval_1,
                    signal.interval_2,
                    signal.interval_3,
                ]
                .iter()
                .any(|value| !value.is_finite() || *value < 0.0)
            {
                return Err(invalid("invalid semaphore geometry or timing"));
            }
            if signal
                .profile_resolution
                .as_deref()
                .is_some_and(|s| s.starts_with("resolved_"))
                && (signal.raw_type.as_deref() != Some("UseProfile")
                    || signal
                        .effective_profile
                        .as_deref()
                        .is_none_or(str::is_empty)
                    || signal.profile_evidence_id.as_deref().is_none_or(|id| {
                        id.len() != 64 || !id.bytes().all(|c| c.is_ascii_hexdigit())
                    }))
            {
                return Err(invalid(
                    "resolved profile is missing its original type or provenance identity",
                ));
            }
        }
        for link in &self.signal_lanes {
            if !lanes.contains(link.lane_id.as_str()) || link.signal_id.is_empty() {
                return Err(invalid("invalid signal-to-lane reference"));
            }
        }
        let mut stops = HashSet::new();
        for stop in &self.stop_lines {
            if !lanes.contains(stop.lane_id.as_str())
                || !finite(stop.position())
                || stop.id.is_empty()
                || !stops.insert(stop.id.as_str())
            {
                return Err(invalid("invalid stop target"));
            }
        }
        Ok(())
    }

    pub fn control_for_lane(&self, lane_id: &str) -> Option<&LaneControl> {
        self.lane_controls
            .iter()
            .find(|control| control.lane_id == lane_id)
    }
    /// Call before enabling traffic-control-dependent runtime behaviour. An old
    /// map without annotation arrays is not evidence that the road has no lights.
    pub fn has_annotations_for_route(&self, route_source_ids: &[String]) -> bool {
        self.annotations_present
            && !route_source_ids.is_empty()
            && route_source_ids.iter().all(|id| {
                self.source_lane_ids.contains(id) && !self.reference_only_lane_ids.contains(id)
            })
    }
    pub fn stops_for_lane(&self, lane_id: &str) -> Vec<&StopLine> {
        self.stop_lines
            .iter()
            .filter(|stop| stop.lane_id == lane_id)
            .collect()
    }
    pub fn conflicts_for_lane(&self, lane_id: &str) -> Vec<&ConflictPair> {
        self.conflicts
            .iter()
            .filter(|pair| pair.lane_a == lane_id || pair.lane_b == lane_id)
            .collect()
    }
    pub fn signals_for_lane(&self, lane_id: &str) -> Vec<&Signal> {
        let groups: HashSet<_> = self
            .signal_lanes
            .iter()
            .filter(|link| link.lane_id == lane_id)
            .map(|link| link.signal_id.as_str())
            .collect();
        self.signals
            .iter()
            .filter(|signal| groups.contains(signal.id.as_str()))
            .collect()
    }

    /// Route order is preserved. Within a lane, the caller projects targets onto
    /// its route centreline to determine longitudinal order and distance.
    pub fn upcoming_stop_controls(&self, route_source_ids: &[String]) -> Vec<UpcomingStopControl> {
        let mut output = Vec::new();
        for (route_index, lane_id) in route_source_ids.iter().enumerate() {
            for stop in self.stops_for_lane(lane_id) {
                output.push(UpcomingStopControl {
                    route_index,
                    lane_id: lane_id.clone(),
                    position: stop.position(),
                    control_kind: stop.control_kind,
                    signal_group_id: stop.signal_id.clone(),
                    priority: match stop.control_kind {
                        ControlKind::Signal => RightOfWay::SignalRequired,
                        ControlKind::Yield => RightOfWay::Yield,
                        ControlKind::PriorityRoad => RightOfWay::HasPriority,
                        _ => RightOfWay::Unknown,
                    },
                    stop_before_m: 0.0,
                    geometry_source: stop.geometry_status.clone(),
                    conflict_lane_ids: vec![],
                    requires_conservative_wait: !matches!(
                        stop.control_kind,
                        ControlKind::PriorityRoad | ControlKind::Uncontrolled
                    ),
                });
            }
            for pair in self.conflicts_for_lane(lane_id) {
                let priority = pair.priority_for(lane_id).unwrap_or(RightOfWay::Unknown);
                let other = if pair.lane_a == *lane_id {
                    &pair.lane_b
                } else {
                    &pair.lane_a
                };
                let group = self
                    .control_for_lane(lane_id)
                    .and_then(|control| control.signal_group_id.clone());
                output.push(UpcomingStopControl {
                    route_index,
                    lane_id: lane_id.clone(),
                    position: pair.position(),
                    control_kind: match priority {
                        RightOfWay::SignalRequired => ControlKind::Signal,
                        RightOfWay::HasPriority => ControlKind::PriorityRoad,
                        RightOfWay::Yield | RightOfWay::ZipMerge | RightOfWay::EqualPriority => {
                            ControlKind::Yield
                        }
                        RightOfWay::Unknown => ControlKind::Unknown,
                    },
                    signal_group_id: group,
                    priority,
                    stop_before_m: pair.radius_m,
                    geometry_source: match pair.source {
                        ConflictSource::NativePpdGroup => "native_ppd_group",
                        ConflictSource::DerivedGeometry => "derived_geometry",
                        ConflictSource::Unknown => "unknown",
                    }
                    .into(),
                    conflict_lane_ids: vec![other.clone()],
                    requires_conservative_wait: pair.conservative_wait
                        || priority != RightOfWay::HasPriority,
                });
            }
        }
        output
    }

    /// Only traffic-light observations are accepted here. Gate OPEN=3 belongs to
    /// a separate gate state machine. We require every matched fresh lamp to
    /// agree on GREEN=8. Orange, flashing/off and unknown states hold by default.
    pub fn signal_decision(
        &self,
        group_id: &str,
        observations: &[SignalObservation],
        max_distance_m: f64,
        max_age_us: u64,
    ) -> SignalDecision {
        use SignalHoldReason as Reason;
        if !max_distance_m.is_finite() || max_distance_m <= 0.0 || max_age_us == 0 {
            return SignalDecision::Hold(Reason::InvalidSettings);
        }
        if !self.signals.iter().any(|signal| signal.id == group_id) {
            return SignalDecision::Hold(Reason::MissingGroup);
        }
        let unresolved =
            |s: &Signal| matches!(s.kind, SemaphoreKind::UseProfile | SemaphoreKind::Unknown);
        let heads = || self.signals.iter().filter(|s| s.id == group_id);
        // Lights whose profile's country unit the exporter did not find
        // (1352 heads, 1108 of them cr_1x1) are still read from the game's
        // own lamp states; any other unresolved head may be a barrier.
        let unresolved_lights = heads().any(unresolved);
        if heads().any(|s| unresolved(s) && !unresolved_light_profile(s)) {
            return SignalDecision::Hold(Reason::UnresolvedProfile);
        }
        if !unresolved_lights
            && !heads().any(|s| {
                matches!(
                    s.kind,
                    SemaphoreKind::TrafficLight
                        | SemaphoreKind::TrafficLightMinor
                        | SemaphoreKind::TrafficLightMajor
                        | SemaphoreKind::TrafficLightBlockable
                        | SemaphoreKind::TrafficLightVirtual
                )
            })
        {
            return SignalDecision::Hold(Reason::NotTrafficLight);
        }
        let mut states = HashSet::new();
        let mut stale = false;
        for observation in observations {
            if !finite(observation.position) {
                continue;
            }
            let groups: HashSet<_> = self
                .signals
                .iter()
                .filter(|signal| {
                    distance3(signal.position(), observation.position) <= max_distance_m
                })
                .map(|signal| signal.id.as_str())
                .collect();
            if !groups.contains(group_id) {
                continue;
            }
            if groups.len() != 1 {
                // Heads of other groups nearby: the observation is the
                // nearest group's when that is clearly nearest. Junctions
                // can carry two groups 1 m apart (the same lights mapped
                // twice), which as a plain ambiguity held a green for ever.
                let nearest = |id: &str| {
                    self.signals
                        .iter()
                        .filter(|s| s.id == id)
                        .map(|s| distance3(s.position(), observation.position))
                        .fold(f64::INFINITY, f64::min)
                };
                let own = nearest(group_id);
                let other = groups
                    .iter()
                    .filter(|g| **g != group_id)
                    .map(|g| nearest(g))
                    .fold(f64::INFINITY, f64::min);
                if other < own - SIGNAL_HEAD_MARGIN_M {
                    continue;
                }
                if other < own + SIGNAL_HEAD_MARGIN_M {
                    return SignalDecision::Hold(Reason::AmbiguousGeometry);
                }
            }
            if observation.age_us > max_age_us {
                stale = true;
                continue;
            }
            states.insert(observation.raw_state);
        }
        // Unread, a light of unresolved profile is not taken as dark.
        if states.is_empty() && unresolved_lights {
            return SignalDecision::Hold(Reason::UnresolvedProfile);
        }
        if states.is_empty() {
            return SignalDecision::Hold(if stale {
                Reason::Stale
            } else {
                Reason::MissingObservation
            });
        }
        if states.len() != 1 {
            return SignalDecision::Hold(Reason::ConflictingStates);
        }
        let state = *states.iter().next().expect("nonempty states checked");
        if state == 8 {
            SignalDecision::Proceed
        } else {
            SignalDecision::Hold(Reason::NotGreen(state))
        }
    }
}

/// A head of a traffic light whose profile (crossroads `cr_*`, crossings,
/// two- to four-phase) is known by name although not resolved.
fn unresolved_light_profile(signal: &Signal) -> bool {
    let name = signal.profile.as_str();
    signal.raw_type.as_deref() == Some("UseProfile")
        && (name.starts_with("cr_")
            || name.starts_with("cross")
            || matches!(name, "2ph" | "3ph" | "4ph"))
}
fn finite(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}
fn distance3(a: Point3, b: Point3) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn context() -> TrafficContext {
        TrafficContext::from_json(r#"{"schema_version":1,"lanes":[{"id":"a"},{"id":"b"}],
          "conflicts":[{"id":"c","owner_uid":18446744073709551615,"lane_a":"a","lane_b":"b","kind":"cross","source":"native_ppd_group","x":5,"y":0,"z":0,"radius_m":4,"geometry_gap_m":0,"priority_a":4,"priority_b":12,"relation":"a_yields_b","conservative_wait":false}],
          "lane_controls":[],"signals":[{"id":"g","owner_uid":18446744073709551615,"semaphore_id":0,"type":"TrafficLight","x":0,"y":0,"z":2,"yaw":0,"profile":"x","cycle_delay_s":0,"interval_0":10,"interval_1":2,"interval_2":10,"interval_3":2}],
          "signal_lanes":[{"signal_id":"g","lane_id":"a"}],
          "stop_lines":[{"id":"s","lane_id":"a","signal_id":"g","rule":"","x":0,"y":0,"z":0,"geometry_status":"curve_start_proxy","control_kind":"signal"}]}"#).unwrap()
    }
    #[test]
    fn unresolved_profile_or_gate_never_uses_traffic_light_green_permission() {
        let mut context = context();
        let observed = [SignalObservation {
            position: Point3 {
                x: 0.0,
                y: 0.0,
                z: 2.0,
            },
            raw_state: 8,
            age_us: 0,
        }];
        for kind in [SemaphoreKind::UseProfile, SemaphoreKind::Unknown] {
            context.signals[0].kind = kind;
            assert_eq!(
                context.signal_decision("g", &observed, 1.0, 100),
                SignalDecision::Hold(SignalHoldReason::UnresolvedProfile)
            );
        }
        // A crossroads light whose country profile was not found is read
        // from its observed lamps; unread it still holds as unresolved.
        context.signals[0].kind = SemaphoreKind::Unknown;
        context.signals[0].raw_type = Some("UseProfile".into());
        context.signals[0].profile = "cr_1x1".into();
        assert_eq!(
            context.signal_decision("g", &observed, 1.0, 100),
            SignalDecision::Proceed
        );
        assert_eq!(
            context.signal_decision("g", &[], 1.0, 100),
            SignalDecision::Hold(SignalHoldReason::UnresolvedProfile)
        );
        // A gate's profile is not one of them.
        context.signals[0].profile = "barrier_toll".into();
        assert_eq!(
            context.signal_decision("g", &observed, 1.0, 100),
            SignalDecision::Hold(SignalHoldReason::UnresolvedProfile)
        );
        context.signals[0].raw_type = None;
        context.signals[0].profile = "x".into();
        for kind in [
            SemaphoreKind::BarrierManualTimed,
            SemaphoreKind::BarrierDistanceActivated,
            SemaphoreKind::BarrierAutomatic,
            SemaphoreKind::BarrierGas,
            SemaphoreKind::ModelOnly,
        ] {
            context.signals[0].kind = kind;
            assert_eq!(
                context.signal_decision("g", &observed, 1.0, 100),
                SignalDecision::Hold(SignalHoldReason::NotTrafficLight)
            );
        }
    }
    #[test]
    fn source_ids_priority_and_stop_targets_are_lossless() {
        let context = context();
        assert_eq!(context.conflicts[0].owner_uid, u64::MAX);
        assert_eq!(
            context.conflicts[0].priority_for("a"),
            Some(RightOfWay::Yield)
        );
        assert_eq!(
            context.conflicts[0].priority_for("b"),
            Some(RightOfWay::HasPriority)
        );
        let targets = context.upcoming_stop_controls(&["a".into()]);
        assert_eq!(targets.len(), 2);
        assert_eq!(targets[0].stop_before_m, 0.0);
        assert_eq!(targets[1].stop_before_m, 4.0);
        assert!(targets[1].requires_conservative_wait);
        let restored: TrafficContext =
            serde_json::from_str(&serde_json::to_string(&context).unwrap()).unwrap();
        restored.validate().unwrap();
        assert_eq!(restored.signals_for_lane("a").len(), 1);
    }
    #[test]
    fn only_fresh_unambiguous_green_passes() {
        let mut context = context();
        let mut observed = vec![SignalObservation {
            position: Point3 {
                x: 0.0,
                y: 0.0,
                z: 2.0,
            },
            raw_state: 8,
            age_us: 10,
        }];
        assert!(
            context
                .signal_decision("g", &observed, 0.5, 100)
                .can_proceed()
        );
        for state in [0, 1, 2, 3, 4, 32, 99] {
            observed[0].raw_state = state;
            assert!(
                !context
                    .signal_decision("g", &observed, 0.5, 100)
                    .can_proceed()
            );
        }
        observed[0].raw_state = 8;
        observed[0].age_us = 101;
        assert_eq!(
            context.signal_decision("g", &observed, 0.5, 100),
            SignalDecision::Hold(SignalHoldReason::Stale)
        );
        observed[0].age_us = 10;
        let mut other = context.signals[0].clone();
        other.id = "other-direction".into();
        other.x = 0.1;
        context.signals.push(other);
        assert_eq!(
            context.signal_decision("g", &observed, 0.5, 100),
            SignalDecision::Hold(SignalHoldReason::AmbiguousGeometry)
        );
    }
    #[test]
    fn a_head_clearly_nearest_decides_between_groups_a_metre_apart() {
        let mut context = context();
        let mut other = context.signals[0].clone();
        other.id = "same-lights-mapped-twice".into();
        other.x = 1.0;
        context.signals.push(other);
        let at = |x: f64, raw_state: i32| SignalObservation {
            position: Point3 { x, y: 0.0, z: 2.0 },
            raw_state,
            age_us: 10,
        };
        assert!(
            context
                .signal_decision("g", &[at(0.0, 8)], 1.5, 100)
                .can_proceed()
        );
        // The other head's red is not ours; ours is still unseen.
        assert_eq!(
            context.signal_decision("g", &[at(1.0, 2)], 1.5, 100),
            SignalDecision::Hold(SignalHoldReason::MissingObservation)
        );
        assert_eq!(
            context.signal_decision("g", &[at(0.0, 2), at(1.0, 8)], 1.5, 100),
            SignalDecision::Hold(SignalHoldReason::NotGreen(2))
        );
        assert_eq!(
            context.signal_decision("g", &[at(0.5, 8)], 1.5, 100),
            SignalDecision::Hold(SignalHoldReason::AmbiguousGeometry)
        );
    }
    #[test]
    fn missing_and_contradictory_signals_hold() {
        let context = context();
        assert_eq!(
            context.signal_decision("g", &[], 0.5, 100),
            SignalDecision::Hold(SignalHoldReason::MissingObservation)
        );
        let position = context.signals[0].position();
        let observations = [
            SignalObservation {
                position,
                raw_state: 8,
                age_us: 0,
            },
            SignalObservation {
                position,
                raw_state: 2,
                age_us: 0,
            },
        ];
        assert_eq!(
            context.signal_decision("g", &observations, 0.5, 100),
            SignalDecision::Hold(SignalHoldReason::ConflictingStates)
        );
    }
    #[test]
    fn unknown_semantics_never_inherit_priority() {
        let mut context = context();
        context.conflicts[0].source = ConflictSource::Unknown;
        assert_eq!(
            context.conflicts[0].priority_for("b"),
            Some(RightOfWay::Unknown)
        );
        context.conflicts[0].lane_b = "absent".into();
        assert!(context.validate().is_err());
        let old =
            TrafficContext::from_json(r#"{"schema_version":1,"lanes":[{"id":"a"}]}"#).unwrap();
        assert!(!old.annotations_present);
        let roundtrip = TrafficContext::from_json(&serde_json::to_string(&old).unwrap()).unwrap();
        assert!(!roundtrip.has_annotations_for_route(&["a".into()]));
    }
    #[test]
    #[ignore = "set ETSADS_TRAFFIC_FIXTURE to a generated real runtime-map.json"]
    fn real_export_fixture() {
        let path =
            std::env::var("ETSADS_TRAFFIC_FIXTURE").expect("real export fixture path required");
        let context = TrafficContext::from_path(path).unwrap();
        assert!(context.annotations_present);
        assert!(!context.conflicts.is_empty());
        assert!(!context.signals.is_empty());
        assert!(!context.stop_lines.is_empty());
        for pair in &context.conflicts {
            assert!(pair.priority_for(&pair.lane_a).is_some());
            assert!(pair.priority_for(&pair.lane_b).is_some());
        }
        println!(
            "Real traffic export: {} lanes, {} conflicts, {} controls, {} signal locators, {} stop targets",
            context.source_lane_ids.len(),
            context.conflicts.len(),
            context.lane_controls.len(),
            context.signals.len(),
            context.stop_lines.len()
        );
    }
}
