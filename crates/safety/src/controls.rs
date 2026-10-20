//! The SDK's loaded controls mode determines what `abackward` means. In
//! arcade/simple-automatic mode it can engage reverse and become throttle.
use etsads_msgs::protocol::{ControlConfiguration, TelemetryFrame};

pub fn validate_control_configuration(
    configuration: Option<&ControlConfiguration>,
    session_start_qpc: u64,
    telemetry: &TelemetryFrame,
    now: u64,
) -> Result<(), &'static str> {
    let c = configuration.ok_or("SDK controls configuration missing; current bridge required")?;
    if session_start_qpc == 0 || c.session_start_qpc != session_start_qpc {
        return Err("SDK controls configuration belongs to a different or unknown session");
    }
    if !c.safe_shifter() {
        return Err(
            "SDK shifter mode is arcade/simple automatic or unknown; all input authority is disabled",
        );
    }
    if c.generation == 0
        || c.header.seq != c.generation
        || c.header.frame_id.checked_add(1) != Some(c.effective_from_frame_id)
        || c.header.qpc < session_start_qpc
        || c.header.qpc > now
        || c.header.qpc > telemetry.header.qpc
        || c.effective_from_frame_id > telemetry.header.frame_id
        || c.header.frame_id > telemetry.header.frame_id
    {
        return Err(
            "SDK controls configuration is not yet associated with the current physical frame",
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use etsads_msgs::protocol::*;
    #[test]
    fn loaded_mode_and_session_are_required_not_disk_settings() {
        let mut t = TelemetryFrame::default();
        t.header.qpc = 200;
        t.header.frame_id = 5;
        let mut c = ControlConfiguration {
            header: WireHeader {
                qpc: 110,
                frame_id: 2,
                seq: 1,
                ..Default::default()
            },
            session_start_qpc: 100,
            generation: 1,
            effective_from_frame_id: 3,
            shifter_type: SHIFTER_AUTOMATIC,
            valid: 1,
        };
        assert!(validate_control_configuration(None, 100, &t, 200).is_err());
        for mode in [SHIFTER_AUTOMATIC, SHIFTER_MANUAL, SHIFTER_HSHIFTER] {
            c.shifter_type = mode;
            assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_ok());
        }
        for mode in [SHIFTER_UNKNOWN, SHIFTER_ARCADE, 99] {
            c.shifter_type = mode;
            assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_err());
        }
        c.shifter_type = SHIFTER_AUTOMATIC;
        assert!(validate_control_configuration(Some(&c), 101, &t, 200).is_err());
        c.header.qpc = 201;
        assert!(validate_control_configuration(Some(&c), 100, &t, 205).is_err());
        c.header.qpc = 110;
        c.effective_from_frame_id = 6;
        assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_err());
        c.effective_from_frame_id = 3;
        c.header.seq = 0;
        assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_err());
        c.header.seq = 1;
        c.effective_from_frame_id = 2;
        assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_err());
        c.effective_from_frame_id = 3;
        c.valid = 0;
        assert!(validate_control_configuration(Some(&c), 100, &t, 200).is_err());
    }
}
