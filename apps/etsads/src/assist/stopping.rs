//! A planned finish keeps service braking until a single parking-brake request
//! has both SDK completion evidence and a later stopped telemetry observation.
//! Manual takeover and transport failure are handled outside this state machine.
use anyhow::ensure;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    Brake(f64),
    RequestParking,
    Complete,
}
pub struct StopSequence {
    started: f64,
    initial_brake: f64,
    stable_since: Option<f64>,
    requested_at: Option<f64>,
}
impl StopSequence {
    pub fn new(time: f64) -> Self {
        Self {
            started: time,
            initial_brake: 0.15,
            stable_since: None,
            requested_at: None,
        }
    }
    /// Minimal-risk stop after a tracking fault: start at about 3 m/s².
    pub fn urgent(time: f64) -> Self {
        Self {
            initial_brake: 0.35,
            ..Self::new(time)
        }
    }
    pub fn update(
        &mut self,
        time: f64,
        speed: f64,
        parked: bool,
        acknowledged: bool,
    ) -> anyhow::Result<Action> {
        ensure!(
            time.is_finite() && speed.is_finite() && time >= self.started,
            "停车时钟或车速无效"
        );
        ensure!(
            time - self.started < 15.0,
            "限时停车未完成，请手动制动；控制已释放"
        );
        if speed.abs() < 0.05 {
            self.stable_since.get_or_insert(time);
        } else {
            self.stable_since = None;
        }
        let stable = self.stable_since.is_some_and(|t| time - t >= 0.35);
        if let Some(requested) = self.requested_at {
            if stable && parked && acknowledged {
                return Ok(Action::Complete);
            }
            ensure!(
                time - requested < 2.0,
                "没有确认到驻车输入完成和停车状态，请手动制动"
            );
        } else if stable {
            if parked {
                return Ok(Action::Complete);
            }
            self.requested_at = Some(time);
            return Ok(Action::RequestParking);
        }
        Ok(Action::Brake(
            (self.initial_brake + (time - self.started) * 0.3).min(0.55),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stopped_observation_and_own_ack_are_both_required() {
        let mut s = StopSequence::new(10.0);
        assert_eq!(
            s.update(10.0, 4.0, false, false).unwrap(),
            Action::Brake(0.15)
        );
        assert!(matches!(
            s.update(11.0, 0.0, false, false).unwrap(),
            Action::Brake(_)
        ));
        assert_eq!(
            s.update(11.4, 0.0, false, false).unwrap(),
            Action::RequestParking
        );
        assert!(matches!(
            s.update(11.5, 0.0, true, false).unwrap(),
            Action::Brake(_)
        ));
        assert_eq!(s.update(11.6, 0.0, true, true).unwrap(), Action::Complete);
    }
    #[test]
    fn parking_toggle_is_never_retried_and_rollbacks_reset_stability() {
        let mut s = StopSequence::new(0.0);
        s.update(0.0, 0.0, false, false).unwrap();
        s.update(0.2, -0.06, false, false).unwrap();
        s.update(0.4, 0.0, false, false).unwrap();
        assert!(matches!(
            s.update(0.6, 0.0, false, false).unwrap(),
            Action::Brake(_)
        ));
        assert_eq!(
            s.update(0.8, 0.0, false, false).unwrap(),
            Action::RequestParking
        );
        assert!(matches!(
            s.update(1.8, 0.0, false, false).unwrap(),
            Action::Brake(_)
        ));
        assert!(s.update(2.81, 0.0, false, false).is_err());
    }
    #[test]
    fn existing_parking_never_gets_toggled_off_and_moving_vehicle_cannot_complete() {
        let mut s = StopSequence::new(0.0);
        s.update(0.0, 2.0, true, true).unwrap();
        s.update(1.0, 0.0, true, false).unwrap();
        assert_eq!(s.update(1.4, 0.0, true, false).unwrap(), Action::Complete);
        let mut moving = StopSequence::new(0.0);
        assert!(moving.update(15.1, 2.0, false, false).is_err());
    }
}
