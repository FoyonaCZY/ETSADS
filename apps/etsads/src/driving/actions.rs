//! Feedback-driven semantic actions. No shared memory or game calls occur here.
use super::experiment::ExperimentKind;
use anyhow::{Context, bail, ensure};
use etsads_msgs::{Direction, protocol::TelemetryFrame};
use etsads_runtime::RuntimeOutput;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct ActionEvent {
    pub kind: &'static str,
    pub detail: String,
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use etsads_runtime::{RuntimeStatus, parking_mission::PlanningIntent};
    pub(crate) fn output() -> RuntimeOutput {
        RuntimeOutput {
            lateral_control: None,
            navigation: None,
            prediction: None,
            parking_planning: None,
            header: Default::default(),
            control_ego: Default::default(),
            request_control: true,
            status: RuntimeStatus::Running,
            reason: "synthetic action fixture".into(),
            actuation: Default::default(),
            lane: None,
            trajectory: None,
            planned_this_frame: true,
            planning_seconds: 0.0,
            supported_capabilities: vec!["synthetic-only".into()],
            intent: PlanningIntent::default(),
            following: None,
            following_unavailable_reason: None,
        }
    }
    fn raw() -> TelemetryFrame {
        TelemetryFrame {
            qpc_frequency: 1000,
            ..Default::default()
        }
    }
    fn manual_gate_diagnostic() -> Box<etsads_runtime::controlled_route::GateDiagnostic> {
        Box::new(serde_json::from_value(serde_json::json!({"id":"synthetic-gate","source_lane_id":"synthetic-lane","native_signal_group_id":null,"mode":"manual",
            "barrier":"Closed","distance_m":3.0,"physical_front_distance_m":3.0,"rear_cleared":false,"payment_required":true,
            "action":{"phase":"waiting","target_speed":0.0,"activate":true,"attempts":1,"payment_observed":false,"passage_observed":false,"observation_missing":false,"activation_authorized":true},
            "geometry_source":"synthetic test only","profile_evidence_ids":[],"observation_positions":[{"x":0.0,"y":0.0,"z":0.0}]})).unwrap())
    }
    #[test]
    fn current_gate_state_and_typed_mode_are_checked_before_activation_publication() {
        let mut out = output();
        out.intent.activate = true;
        out.actuation.activate = true;
        out.intent.hold_brake = true;
        out.actuation.brake = 0.4;
        out.intent.gate = Some(manual_gate_diagnostic());
        let mut raw = raw();
        raw.header.frame_id = 1;
        raw.header.qpc = 1000;
        let mut ext = Box::<etsads_msgs::protocol::ExtensionFrame>::default();
        ext.header = raw.header;
        ext.available = 8;
        let mut signal: etsads_game::ets2la::Signal = bytemuck::Zeroable::zeroed();
        signal.kind = 2;
        signal.state = 1;
        ext.signals[..48].copy_from_slice(bytemuck::bytes_of(&signal));
        let mut snapshot =
            etsads_game::adapter::snapshot_from_recorded_frame(raw, Some(ext), None).unwrap();
        assert!(validate_gate_activation(&snapshot, &out).is_ok());
        snapshot.extension.as_mut().unwrap().header.frame_id = 0;
        assert!(validate_gate_activation(&snapshot, &out).is_err());
        snapshot.extension.as_mut().unwrap().header.frame_id = 1;
        signal.state = 3;
        snapshot.extension.as_mut().unwrap().signals[..48]
            .copy_from_slice(bytemuck::bytes_of(&signal));
        assert!(validate_gate_activation(&snapshot, &out).is_err());
        signal.state = 1;
        snapshot.extension.as_mut().unwrap().signals[..48]
            .copy_from_slice(bytemuck::bytes_of(&signal));
        out.intent.gate.as_mut().unwrap().mode =
            etsads_runtime::controlled_route::gates::GateMode::Automatic;
        assert!(validate_gate_activation(&snapshot, &out).is_err());
        assert!(
            ActionExecutor::default()
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &snapshot.raw,
                    true,
                    true,
                    1000
                )
                .is_err()
        );
    }
    #[test]
    fn shadow_consumes_no_actions_and_gate_grant_cannot_lose_one_shot_gear() {
        let mut executor = ActionExecutor::default();
        let mut out = output();
        out.intent.gear_request = Some(Direction::Reverse);
        out.intent.hold_brake = true;
        for _ in 0..5 {
            assert!(
                executor
                    .prepare(
                        Some(ExperimentKind::Parking),
                        &out,
                        &raw(),
                        false,
                        false,
                        1000
                    )
                    .unwrap()
                    .pulses
                    .is_empty()
            );
        }
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::Parking),
                    &out,
                    &raw(),
                    true,
                    false,
                    1000
                )
                .unwrap()
                .pulses
                .is_empty()
        );
        out.intent.gear_request = None;
        let action = executor
            .prepare(
                Some(ExperimentKind::Parking),
                &out,
                &raw(),
                true,
                true,
                1010,
            )
            .unwrap();
        assert_eq!(action.pulses, vec!["gearreverse"]);
        assert!(action.hold_brake);
        for time in [1020, 1100, 2000] {
            assert!(
                executor
                    .prepare(
                        Some(ExperimentKind::Parking),
                        &out,
                        &raw(),
                        true,
                        true,
                        time
                    )
                    .unwrap()
                    .pulses
                    .is_empty()
            );
        }
        let mut measured = raw();
        measured.gear = -1;
        let acknowledged = executor
            .prepare(
                Some(ExperimentKind::Parking),
                &out,
                &measured,
                true,
                true,
                2010,
            )
            .unwrap();
        assert!(
            acknowledged
                .events
                .iter()
                .any(|e| e.kind == "gear_acknowledged")
        );
        assert!(!acknowledged.hold_brake);
    }
    #[test]
    fn indicator_switch_waits_for_measured_off_then_on_without_toggle_flood() {
        let mut executor = ActionExecutor::default();
        let mut out = output();
        let mut measured = raw();
        out.intent.indicator = 1;
        out.actuation.indicator = 1;
        measured.lights = 1 << 7;
        let first = executor
            .prepare(
                Some(ExperimentKind::ControlledRoute),
                &out,
                &measured,
                true,
                true,
                1000,
            )
            .unwrap();
        assert_eq!(first.pulses, vec!["rblinker"]);
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &measured,
                    true,
                    true,
                    1020
                )
                .unwrap()
                .pulses
                .is_empty()
        );
        measured.lights = 0;
        assert_eq!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &measured,
                    true,
                    true,
                    1040
                )
                .unwrap()
                .pulses,
            vec!["lblinker"]
        );
        measured.lights = 1 << 6;
        out.intent.active_lane_change = true;
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &measured,
                    true,
                    true,
                    1060
                )
                .unwrap()
                .pulses
                .is_empty()
        );
        measured.lights = 0;
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &measured,
                    true,
                    true,
                    1080
                )
                .is_err()
        );
    }
    #[test]
    fn missing_gear_ack_times_out_and_activate_is_one_rising_edge_at_standstill() {
        let mut executor = ActionExecutor::default();
        let mut out = output();
        out.intent.gear_request = Some(Direction::Reverse);
        out.intent.hold_brake = true;
        executor
            .prepare(
                Some(ExperimentKind::Parking),
                &out,
                &raw(),
                true,
                true,
                1000,
            )
            .unwrap();
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::Parking),
                    &out,
                    &raw(),
                    true,
                    true,
                    4001
                )
                .is_err()
        );
        let mut executor = ActionExecutor::default();
        let mut out = output();
        out.intent.activate = true;
        out.actuation.activate = true;
        out.intent.gate = Some(manual_gate_diagnostic());
        out.intent.hold_brake = true;
        out.actuation.brake = 0.4;
        assert_eq!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &raw(),
                    true,
                    true,
                    1000
                )
                .unwrap()
                .pulses,
            vec!["activate"]
        );
        assert!(
            executor
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &raw(),
                    true,
                    true,
                    1020
                )
                .unwrap()
                .pulses
                .is_empty()
        );
        let mut moving = raw();
        moving.speed_mps = 0.2;
        assert!(
            ActionExecutor::default()
                .prepare(
                    Some(ExperimentKind::ControlledRoute),
                    &out,
                    &moving,
                    true,
                    true,
                    1000
                )
                .is_err()
        );
    }
}
#[derive(Default, Debug)]
pub struct ActionBatch {
    pub pulses: Vec<&'static str>,
    pub events: Vec<ActionEvent>,
    pub hold_brake: bool,
}
impl ActionBatch {
    fn event(&mut self, kind: &'static str, detail: impl Into<String>) {
        self.events.push(ActionEvent {
            kind,
            detail: detail.into(),
        });
    }
    fn pulse(&mut self, name: &'static str) {
        self.pulses.push(name);
        self.event("pulse", name);
    }
}
#[derive(Default)]
pub struct ActionExecutor {
    indicator_pending: Option<(i8, u64)>,
    gear_pending: Option<(Direction, Option<u64>)>,
    activate_pending: bool,
    last_activate: bool,
}
impl ActionExecutor {
    pub fn prepare(
        &mut self,
        kind: Option<ExperimentKind>,
        output: &RuntimeOutput,
        raw: &TelemetryFrame,
        enabled: bool,
        granted: bool,
        now: u64,
    ) -> anyhow::Result<ActionBatch> {
        let mut batch = ActionBatch::default();
        ensure!(
            output.actuation.retarder == 0,
            "retarder actuation is not commissioned"
        );
        let intent = &output.intent;
        if kind.is_none() {
            ensure!(
                intent.indicator == 0
                    && !intent.activate
                    && intent.gear_request.is_none()
                    && !intent.active_lane_change
                    && output.actuation.indicator == 0
                    && !output.actuation.activate,
                "maneuver actions require --experiment and verified experiment evidence"
            );
            return Ok(batch);
        }
        ensure!(
            raw.qpc_frequency > 0 && raw.speed_mps.is_finite(),
            "invalid action feedback clock/speed"
        );
        ensure!(
            (-1..=1).contains(&intent.indicator),
            "invalid indicator target"
        );
        ensure!(
            output.actuation.indicator == intent.indicator
                && output.actuation.activate == intent.activate,
            "runtime actuation and intent disagree"
        );
        if !enabled {
            // Shadow mode consumes no counters and cannot queue an action for a
            // later run. Observed intent is already in the mandatory stack trace.
            return Ok(batch);
        }
        if let Some(direction) = intent.gear_request {
            ensure!(
                kind == Some(ExperimentKind::Parking),
                "gear changes require the parking experiment"
            );
            ensure!(
                raw.speed_mps.abs() <= 0.05 && intent.hold_brake,
                "gear request requires measured standstill <=0.05 m/s and hold_brake"
            );
            if let Some((pending, _)) = self.gear_pending {
                ensure!(
                    pending == direction,
                    "gear target changed before acknowledgement"
                );
            } else {
                self.gear_pending = Some((direction, None));
                batch.event("gear_requested", format!("{direction:?}"));
            }
        }
        if intent.activate && !self.last_activate {
            self.activate_pending = true;
        }
        self.last_activate = intent.activate;
        if !granted {
            batch.hold_brake = self.gear_pending.is_some();
            return Ok(batch);
        }
        let elapsed = |since: u64| -> anyhow::Result<f64> {
            ensure!(now >= since, "action acknowledgement clock regressed");
            Ok((now - since) as f64 / raw.qpc_frequency as f64)
        };
        if let Some((direction, sent)) = self.gear_pending {
            let acknowledged = match direction {
                Direction::Forward => raw.gear > 0,
                Direction::Reverse => raw.gear < 0,
            };
            if acknowledged {
                self.gear_pending = None;
                batch.event(
                    "gear_acknowledged",
                    format!("{direction:?}: raw gear {}", raw.gear),
                );
            } else if let Some(sent) = sent {
                ensure!(
                    elapsed(sent)? <= 3.0,
                    "gear was not acknowledged within 3 seconds; check transmission mode and semantic gear evidence"
                );
                batch.hold_brake = true;
            } else {
                ensure!(
                    raw.speed_mps.abs() <= 0.05,
                    "truck moved before gear request could be emitted"
                );
                batch.pulse(match direction {
                    Direction::Forward => "geardrive",
                    Direction::Reverse => "gearreverse",
                });
                self.gear_pending = Some((direction, Some(now)));
                batch.hold_brake = true;
            }
        }
        let left = raw.lights & (1 << 6) != 0;
        let right = raw.lights & (1 << 7) != 0;
        ensure!(
            !(left && right) && raw.lights & (1 << 8) == 0,
            "hazard/ambiguous blinker state: stop experiment for operator review"
        );
        let observed = if left {
            1
        } else if right {
            -1
        } else {
            0
        };
        if let Some((expected, sent)) = self.indicator_pending {
            if observed == expected {
                self.indicator_pending = None;
                batch.event("indicator_acknowledged", expected.to_string());
            } else {
                ensure!(
                    elapsed(sent)? <= 1.0,
                    "indicator state not acknowledged within 1 second"
                );
            }
        }
        ensure!(
            !intent.active_lane_change || observed == intent.indicator,
            "lane change started before measured indicator acknowledgement"
        );
        if self.indicator_pending.is_none() && observed != intent.indicator {
            // Switching sides is two confirmed transitions, so the host never
            // assumes a particular game's interaction between left/right toggles.
            let toggled = if observed != 0 {
                observed
            } else {
                intent.indicator
            };
            let expected = if observed != 0 { 0 } else { intent.indicator };
            batch.pulse(if toggled == 1 { "lblinker" } else { "rblinker" });
            self.indicator_pending = Some((expected, now));
        }
        if self.activate_pending {
            let gate = intent
                .gate
                .as_ref()
                .context("activate lacks typed gate evidence")?;
            ensure!(
                gate.mode == etsads_runtime::controlled_route::gates::GateMode::Manual
                    && gate.action.activate
                    && gate.action.activation_authorized
                    && !gate.action.observation_missing
                    && gate.barrier == Some(etsads_msgs::BarrierState::Closed),
                "activate requires an authorized, observed closed manual gate"
            );
            ensure!(
                kind == Some(ExperimentKind::ControlledRoute),
                "activate is only commissioned for controlled-route toll interactions"
            );
            ensure!(
                raw.speed_mps.abs() <= 0.05 && intent.hold_brake && output.actuation.brake >= 0.2,
                "activate requires measured standstill and commanded hold brake"
            );
            if self.gear_pending.is_some() {
                bail!("cannot activate while gear acknowledgement is pending");
            }
            batch.pulse("activate");
            self.activate_pending = false;
        }
        Ok(batch)
    }
}
pub(super) fn validate_gate_activation(
    snapshot: &etsads_game::adapter::GameSnapshot,
    output: &RuntimeOutput,
) -> anyhow::Result<()> {
    let gate = output
        .intent
        .gate
        .as_ref()
        .context("activate lacks a source-bound gate diagnostic")?;
    ensure!(
        gate.mode == etsads_runtime::controlled_route::gates::GateMode::Manual
            && gate.action.activate
            && gate.action.activation_authorized,
        "only authorized manual gates may issue activate"
    );
    ensure!(
        etsads_runtime::controlled_route::gate_barrier_state(
            snapshot.extension.as_deref(),
            snapshot.ego.header.frame_id,
            &gate.observation_positions
        ) == Some(etsads_msgs::BarrierState::Closed),
        "current same-frame gate state changed or is missing; activate rejected before publication"
    );
    Ok(())
}
