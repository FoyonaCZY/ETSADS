use crate::PlanningError;
use etsads_msgs::{BarrierState, SignalState};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GateMode {
    Manual,
    Automatic,
    Distance,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GatePhase {
    Approach,
    Settling,
    Waiting,
    Passing,
    AwaitPayment,
    Complete,
    Takeover,
}
#[derive(Clone, Copy, Debug)]
pub struct GateInput {
    pub time_s: f64,
    pub distance_m: f64,
    pub speed_mps: f64,
    pub barrier: Option<BarrierState>,
    pub rear_cleared: bool,
    pub activation_zone: bool,
    pub paid_event: bool,
    pub activation_authorized: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GateAction {
    pub phase: GatePhase,
    pub target_speed: f64,
    pub activate: bool,
    pub attempts: u8,
    pub payment_observed: bool,
    pub passage_observed: bool,
    pub observation_missing: bool,
    pub activation_authorized: bool,
}
/// One active barrier, driven solely by fresh runtime state and monotonic time.
/// Definition intervals are deliberately absent from this interface.
#[derive(Clone, Debug)]
pub struct GateMachine {
    mode: GateMode,
    payment_required: bool,
    phase: GatePhase,
    since: f64,
    first_wait: Option<f64>,
    last_time: Option<f64>,
    attempts: u8,
    paid: bool,
    passed: bool,
}
impl GateMachine {
    pub fn new(mode: GateMode, payment_required: bool) -> Self {
        Self {
            mode,
            payment_required,
            phase: GatePhase::Approach,
            since: 0.0,
            first_wait: None,
            last_time: None,
            attempts: 0,
            paid: false,
            passed: false,
        }
    }
    pub fn phase(&self) -> GatePhase {
        self.phase
    }
    pub fn committed(&self) -> bool {
        matches!(
            self.phase,
            GatePhase::Settling | GatePhase::Waiting | GatePhase::Passing | GatePhase::AwaitPayment
        )
    }
    pub fn update(&mut self, input: GateInput) -> Result<GateAction, PlanningError> {
        let i = input;
        if ![i.time_s, i.distance_m, i.speed_mps]
            .iter()
            .all(|n| n.is_finite())
            || i.time_s < 0.0
            || i.speed_mps < 0.0
            || self.last_time.is_some_and(|t| i.time_s < t)
        {
            return Err(PlanningError::Invalid("gate time/state"));
        }
        self.last_time = Some(i.time_s);
        let stopped = i.speed_mps <= 0.05;
        let mut activate = false;
        // Payment belongs only to a committed, nearby interaction. A paid
        // event seen while approaching another gate is not its receipt.
        if self.committed()
            && (self.mode != GateMode::Manual || self.attempts > 0)
            && i.paid_event
            && i.distance_m.abs() <= 15.0
        {
            self.paid = true;
        }
        if self.phase == GatePhase::Complete || self.phase == GatePhase::Takeover {
            return Ok(self.action(false, i.barrier.is_none(), i.activation_authorized));
        }
        if self
            .first_wait
            .is_some_and(|start| i.time_s - start >= 41.0)
            && self.phase != GatePhase::Passing
        {
            self.phase = GatePhase::Takeover;
            return Ok(self.action(false, i.barrier.is_none(), i.activation_authorized));
        }
        match self.phase {
            GatePhase::Approach => {
                if i.distance_m < -1.0 {
                    // Never infer a pass just because the first observation
                    // starts behind a barrier without having seen OPEN.
                    self.phase = GatePhase::Takeover;
                } else if i.distance_m <= 20.0
                    && i.barrier == Some(BarrierState::Open)
                    && self.mode != GateMode::Manual
                {
                    self.phase = GatePhase::Passing;
                    self.since = i.time_s;
                } else if i.distance_m <= 3.5 && stopped {
                    self.phase = GatePhase::Settling;
                    self.since = i.time_s;
                    self.first_wait = Some(i.time_s);
                }
            }
            GatePhase::Settling => {
                if !stopped || !i.activation_zone {
                    self.since = i.time_s;
                } else if i.time_s - self.since >= 1.0 {
                    if i.barrier == Some(BarrierState::Open) {
                        self.phase = GatePhase::Passing;
                        self.since = i.time_s;
                    } else if i.barrier.is_some() {
                        self.phase = GatePhase::Waiting;
                        self.since = i.time_s;
                        if self.mode == GateMode::Manual
                            && i.barrier == Some(BarrierState::Closed)
                            && i.activation_authorized
                        {
                            activate = true;
                            self.attempts = 1;
                        }
                    }
                }
            }
            GatePhase::Waiting => {
                if i.barrier == Some(BarrierState::Open) {
                    self.phase = GatePhase::Passing;
                    self.since = i.time_s;
                } else if self.mode == GateMode::Manual
                    && i.barrier == Some(BarrierState::Closed)
                    && stopped
                    && i.activation_zone
                    && i.activation_authorized
                    && (self.attempts == 0 || self.attempts == 1 && i.time_s - self.since >= 20.0)
                {
                    activate = true;
                    self.attempts += 1;
                    self.since = i.time_s;
                }
            }
            GatePhase::Passing => {
                if i.rear_cleared {
                    self.passed = true;
                    if !self.payment_required || self.paid {
                        self.phase = GatePhase::Complete;
                    } else {
                        self.phase = GatePhase::AwaitPayment;
                        self.since = i.time_s;
                        self.first_wait = Some(i.time_s);
                    }
                } else if i.barrier != Some(BarrierState::Open) {
                    // Hold in place and do not re-activate while the truck may
                    // still occupy the plane. A missing state is not CLOSED.
                    if i.time_s - self.since >= 20.0 {
                        self.phase = GatePhase::Takeover;
                    }
                }
            }
            GatePhase::AwaitPayment => {
                if self.paid {
                    self.phase = GatePhase::Complete;
                } else if i.time_s - self.since >= 10.0 {
                    self.phase = GatePhase::Takeover;
                }
            }
            GatePhase::Complete | GatePhase::Takeover => {}
        }
        if self.phase == GatePhase::Passing
            && self.mode != GateMode::Manual
            && i.paid_event
            && i.distance_m.abs() <= 15.0
        {
            self.paid = true;
        }
        let mut action = self.action(activate, i.barrier.is_none(), i.activation_authorized);
        action.target_speed = match self.phase {
            GatePhase::Approach => (2.0 * 1.5 * (i.distance_m - 3.0).max(0.0)).sqrt().min(8.0),
            GatePhase::Passing if i.barrier == Some(BarrierState::Open) => 5.0,
            GatePhase::Complete => 5.0,
            _ => 0.0,
        };
        Ok(action)
    }
    fn action(&self, activate: bool, missing: bool, authorized: bool) -> GateAction {
        GateAction {
            phase: self.phase,
            target_speed: if self.phase == GatePhase::Complete {
                5.0
            } else {
                0.0
            },
            activate,
            attempts: self.attempts,
            payment_observed: self.paid,
            passage_observed: self.passed,
            observation_missing: missing,
            activation_authorized: authorized,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignalDecision {
    Stop,
    Proceed,
    Yield,
}
pub fn signal_decision(
    state: SignalState,
    distance: f64,
    speed: f64,
    remaining: f64,
    clearance_time: f64,
) -> Result<SignalDecision, PlanningError> {
    if ![distance, speed, remaining, clearance_time]
        .iter()
        .all(|x| x.is_finite())
        || speed < 0.0
        || remaining < 0.0
        || clearance_time < 0.0
    {
        return Err(PlanningError::Invalid("signal state"));
    }
    let stopping = speed * 0.3 + speed * speed / (2.0 * 1.5) + 3.0;
    Ok(match state {
        SignalState::Red | SignalState::AmberToGreen => SignalDecision::Stop,
        SignalState::AmberToRed => {
            if stopping <= distance {
                SignalDecision::Stop
            } else {
                SignalDecision::Proceed
            }
        }
        SignalState::Green => {
            if remaining > 0.0 && remaining < clearance_time && stopping <= distance {
                SignalDecision::Stop
            } else {
                SignalDecision::Proceed
            }
        }
        SignalState::Off | SignalState::Flashing => SignalDecision::Yield,
    })
}
pub fn must_yield(ego_arrival: f64, other_arrivals: &[f64], unprotected_left: bool) -> bool {
    if !ego_arrival.is_finite()
        || ego_arrival < 0.0
        || other_arrivals.iter().any(|t| !t.is_finite() || *t < 0.0)
    {
        return true;
    }
    let gap = if unprotected_left { 8.0 } else { 4.0 };
    other_arrivals
        .iter()
        .any(|t| (ego_arrival - *t).abs() < gap)
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TollPhase {
    Approach,
    Settling,
    Waiting,
    Passing,
    Complete,
    Takeover,
}
#[derive(Clone, Copy, Debug)]
pub struct TollAction {
    pub phase: TollPhase,
    pub target_speed: f64,
    pub activate: bool,
}
#[derive(Clone, Debug)]
pub struct TollBooth {
    phase: TollPhase,
    since: f64,
    last_time: Option<f64>,
    attempts: u8,
}
impl Default for TollBooth {
    fn default() -> Self {
        Self {
            phase: TollPhase::Approach,
            since: 0.0,
            last_time: None,
            attempts: 0,
        }
    }
}
impl TollBooth {
    pub fn update(
        &mut self,
        time: f64,
        distance: f64,
        speed: f64,
        barrier: BarrierState,
        paid: bool,
    ) -> Result<TollAction, PlanningError> {
        if ![time, distance, speed].iter().all(|v| v.is_finite())
            || time < 0.0
            || speed < 0.0
            || self.last_time.is_some_and(|t| time < t)
        {
            return Err(PlanningError::Invalid("toll time/state"));
        }
        self.last_time = Some(time);
        let mut activate = false;
        match self.phase {
            TollPhase::Approach => {
                if distance <= 3.5 && speed < 0.1 {
                    self.phase = TollPhase::Settling;
                    self.since = time;
                }
            }
            TollPhase::Settling => {
                if time - self.since >= 1.0 {
                    activate = true;
                    self.attempts = 1;
                    self.since = time;
                    self.phase = TollPhase::Waiting;
                }
            }
            TollPhase::Waiting => {
                if barrier == BarrierState::Open {
                    self.phase = TollPhase::Passing;
                } else if time - self.since >= 20.0 {
                    if self.attempts < 2 {
                        activate = true;
                        self.attempts += 1;
                        self.since = time;
                    } else {
                        self.phase = TollPhase::Takeover;
                    }
                }
            }
            TollPhase::Passing => {
                if paid {
                    self.phase = TollPhase::Complete;
                } else if barrier != BarrierState::Open && distance >= 0.0 {
                    self.phase = TollPhase::Waiting;
                    self.since = time;
                }
            }
            _ => {}
        }
        let target_speed = match self.phase {
            TollPhase::Approach => (2.0 * 1.5 * (distance - 3.0).max(0.0)).sqrt().min(8.0),
            TollPhase::Passing | TollPhase::Complete => 5.0,
            _ => 0.0,
        };
        Ok(TollAction {
            phase: self.phase,
            target_speed,
            activate,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn gate_input(time: f64, state: Option<BarrierState>) -> GateInput {
        GateInput {
            time_s: time,
            distance_m: 3.0,
            speed_mps: 0.0,
            barrier: state,
            rear_cleared: false,
            activation_zone: true,
            paid_event: false,
            activation_authorized: true,
        }
    }
    #[test]
    fn typed_manual_gate_requires_observation_permission_dwell_and_bounded_retry() {
        let mut gate = GateMachine::new(GateMode::Manual, true);
        assert!(
            !gate
                .update(gate_input(0.0, Some(BarrierState::Closed)))
                .unwrap()
                .activate
        );
        assert!(!gate.update(gate_input(1.0, None)).unwrap().activate);
        let mut input = gate_input(2.0, Some(BarrierState::Closed));
        input.activation_authorized = false;
        assert!(!gate.update(input).unwrap().activate);
        assert!(
            gate.update(gate_input(3.0, Some(BarrierState::Closed)))
                .unwrap()
                .activate
        );
        assert!(
            !gate
                .update(gate_input(4.0, Some(BarrierState::Closed)))
                .unwrap()
                .activate
        );
        assert!(
            gate.update(gate_input(23.0, Some(BarrierState::Closed)))
                .unwrap()
                .activate
        );
        let final_action = gate
            .update(gate_input(41.0, Some(BarrierState::Closed)))
            .unwrap();
        assert_eq!(final_action.phase, GatePhase::Takeover);
        assert!(!final_action.activate);
        assert_eq!(final_action.attempts, 2);
    }
    #[test]
    fn automatic_and_distance_gates_never_activate_and_need_open_then_rear_clearance() {
        for mode in [GateMode::Automatic, GateMode::Distance] {
            let mut gate = GateMachine::new(mode, false);
            for time in [0.0, 1.0, 2.0] {
                assert!(
                    !gate
                        .update(gate_input(time, Some(BarrierState::Closed)))
                        .unwrap()
                        .activate
                );
            }
            let open = gate
                .update(gate_input(3.0, Some(BarrierState::Open)))
                .unwrap();
            assert_eq!(open.phase, GatePhase::Passing);
            assert!(open.target_speed > 0.0);
            let lost = gate.update(gate_input(4.0, None)).unwrap();
            assert_eq!(lost.target_speed, 0.0);
            assert!(!lost.activate);
            let mut input = gate_input(5.0, Some(BarrierState::Open));
            input.rear_cleared = true;
            let done = gate.update(input).unwrap();
            assert_eq!(done.phase, GatePhase::Complete);
            assert!(!done.payment_observed);
            assert!(done.passage_observed);
        }
    }
    #[test]
    fn receipt_is_distinct_from_physical_passage_and_cannot_precede_interaction() {
        let mut gate = GateMachine::new(GateMode::Manual, true);
        let mut early = gate_input(0.0, Some(BarrierState::Closed));
        early.paid_event = true;
        assert!(!gate.update(early).unwrap().payment_observed);
        gate.update(gate_input(1.0, Some(BarrierState::Closed)))
            .unwrap();
        gate.update(gate_input(2.0, Some(BarrierState::Open)))
            .unwrap();
        let mut passed = gate_input(3.0, Some(BarrierState::Open));
        passed.rear_cleared = true;
        assert_eq!(gate.update(passed).unwrap().phase, GatePhase::AwaitPayment);
        let mut receipt = gate_input(4.0, None);
        receipt.paid_event = true;
        let done = gate.update(receipt).unwrap();
        assert_eq!(done.phase, GatePhase::Complete);
        assert!(done.payment_observed && done.passage_observed);
    }
    #[test]
    fn amber_stopping_dilemma() {
        assert_eq!(
            signal_decision(SignalState::AmberToRed, 10.0, 20.0, 0.0, 4.0).unwrap(),
            SignalDecision::Proceed
        );
        assert_eq!(
            signal_decision(SignalState::AmberToRed, 200.0, 20.0, 0.0, 4.0).unwrap(),
            SignalDecision::Stop
        );
        assert_eq!(
            signal_decision(SignalState::Red, 10.0, 20.0, 0.0, 4.0).unwrap(),
            SignalDecision::Stop
        );
    }
    #[test]
    fn toll_retries_once_and_requires_paid_event() {
        let mut t = TollBooth::default();
        assert!(
            !t.update(0.0, 3.0, 0.0, BarrierState::Closed, false)
                .unwrap()
                .activate
        );
        assert!(
            t.update(1.0, 3.0, 0.0, BarrierState::Closed, false)
                .unwrap()
                .activate
        );
        assert!(
            !t.update(2.0, 3.0, 0.0, BarrierState::Closed, false)
                .unwrap()
                .activate
        );
        assert!(
            t.update(21.0, 3.0, 0.0, BarrierState::Closed, false)
                .unwrap()
                .activate
        );
        assert_eq!(
            t.update(41.0, 3.0, 0.0, BarrierState::Closed, false)
                .unwrap()
                .phase,
            TollPhase::Takeover
        );
    }
    #[test]
    fn crossing_traffic_gap() {
        assert!(must_yield(5.0, &[7.0], false));
        assert!(!must_yield(1.0, &[10.0], true));
        assert!(must_yield(1.0, &[7.0], true));
    }
}
