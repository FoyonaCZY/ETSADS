//! Owned snapshots of official SDK configuration attributes, with presence bits.
use super::*;

/// Unknown, absent, duplicate, or wrongly typed attributes revoke the previous
/// mode. The value comes only from the official SDK controls configuration.
pub(super) fn controls(
    previous: ControlConfiguration,
    config: &scs_telemetry_configuration_t,
    mut header: WireHeader,
) -> ControlConfiguration {
    let generation = previous.generation.saturating_add(1);
    header.seq = generation;
    let mut output = ControlConfiguration {
        header,
        session_start_qpc: previous.session_start_qpc,
        generation,
        effective_from_frame_id: header.frame_id.saturating_add(1),
        ..Default::default()
    };
    let mut seen = false;
    for attr in attributes(config.attributes) {
        if unsafe { CStr::from_ptr(attr.name) }.to_bytes() != b"shifter.type" {
            continue;
        }
        if seen {
            output.valid = 0;
            output.shifter_type = SHIFTER_UNKNOWN;
            return output;
        }
        seen = true;
        if attr.value.type_ != SCS_VALUE_TYPE_string {
            continue;
        }
        let pointer = unsafe { attr.value.__bindgen_anon_1.value_string.value };
        if pointer.is_null() {
            continue;
        }
        output.shifter_type = match unsafe { CStr::from_ptr(pointer) }.to_bytes() {
            b"arcade" => SHIFTER_ARCADE,
            b"automatic" => SHIFTER_AUTOMATIC,
            b"manual" => SHIFTER_MANUAL,
            b"hshifter" => SHIFTER_HSHIFTER,
            _ => SHIFTER_UNKNOWN,
        };
        output.valid = u32::from(output.shifter_type != SHIFTER_UNKNOWN);
    }
    output
}

fn finite_vector(value: &scs_value_t) -> Option<[f32; 3]> {
    if !matches!(value.type_, SCS_VALUE_TYPE_fvector | SCS_VALUE_TYPE_dvector) {
        return None;
    }
    let vector = vector(value);
    vector.iter().all(|x| x.is_finite()).then_some(vector)
}
fn uint(value: &scs_value_t) -> Option<u32> {
    if value.type_ == SCS_VALUE_TYPE_u32 {
        Some(unsafe { value.__bindgen_anon_1.value_u32.value })
    } else {
        None
    }
}
pub(super) fn update(
    output: &mut ConfigurationFrame,
    truck: ConfigBlock,
    config: &scs_telemetry_configuration_t,
    header: WireHeader,
) {
    if config.id.is_null() {
        return;
    }
    let group = unsafe { CStr::from_ptr(config.id) }.to_bytes();
    output.header = header;
    output.header.seq = truck.config_seq;
    output.config_seq = truck.config_seq;
    output.effective_from_frame_id = header.frame_id.saturating_add(1);
    output.truck = truck;
    let trailer = if group == b"trailer" {
        Some(0)
    } else {
        group.strip_prefix(b"trailer.").and_then(|suffix| {
            (suffix.len() == 1 && suffix[0].is_ascii_digit()).then(|| (suffix[0] - b'0') as usize)
        })
    };
    if group == b"truck" {
        output.truck_fields = 0;
        output.truck_wheel_position_mask = 0;
        output.truck_wheel_radius_mask = 0;
        output.truck_wheel_steerable_mask = 0;
        for attr in attributes(config.attributes) {
            let name = unsafe { CStr::from_ptr(attr.name) }.to_bytes();
            let i = attr.index as usize;
            let v = &attr.value;
            match name {
                b"cabin.position" if finite_vector(v).is_some() => {
                    output.truck_fields |= CONFIG_HAS_CABIN as u64
                }
                b"head.position" if finite_vector(v).is_some() => {
                    output.truck_fields |= CONFIG_HAS_HEAD as u64
                }
                b"hook.position" if finite_vector(v).is_some() => {
                    output.truck_fields |= CONFIG_HAS_HOOK as u64
                }
                b"wheels.count" if uint(v).is_some() => {
                    output.truck_fields |= CONFIG_HAS_WHEEL_COUNT as u64;
                    if uint(v).unwrap() > 16 {
                        output.truck_fields |= CONFIG_TRUNCATED_WHEELS as u64;
                    }
                }
                b"wheel.position" if i < 16 && finite_vector(v).is_some() => {
                    output.truck_wheel_position_mask |= 1 << i
                }
                b"wheel.radius"
                    if i < 16
                        && v.type_ == SCS_VALUE_TYPE_float
                        && scalar(v).is_finite()
                        && scalar(v) > 0.0 =>
                {
                    output.truck_wheel_radius_mask |= 1 << i
                }
                b"wheel.steerable" if i < 16 && v.type_ == SCS_VALUE_TYPE_bool => {
                    output.truck_wheel_steerable_mask |= 1 << i
                }
                _ => {}
            }
        }
    } else if let Some(index) = trailer {
        let data = &mut output.trailers[index];
        *data = TrailerConfiguration {
            update_qpc: header.qpc,
            config_seq: truck.config_seq,
            ..Default::default()
        };
        output.reported_trailer_mask |= 1 << index;
        output.present_trailer_mask &= !(1 << index);
        for attr in attributes(config.attributes) {
            data.present = 1;
            output.present_trailer_mask |= 1 << index;
            let name = unsafe { CStr::from_ptr(attr.name) }.to_bytes();
            let i = attr.index as usize;
            let v = &attr.value;
            match name {
                b"id" if v.type_ == SCS_VALUE_TYPE_string => {
                    copy_string(
                        unsafe { v.__bindgen_anon_1.value_string.value },
                        &mut data.id,
                    );
                    data.flags |= CONFIG_HAS_ID;
                }
                b"hook.position" => {
                    if let Some(p) = finite_vector(v) {
                        data.hook_position = p;
                        data.flags |= CONFIG_HAS_HOOK;
                    }
                }
                b"wheels.count" => {
                    if let Some(count) = uint(v) {
                        data.wheel_count = count;
                        data.flags |= CONFIG_HAS_WHEEL_COUNT;
                        if count > 16 {
                            data.flags |= CONFIG_TRUNCATED_WHEELS;
                        }
                    }
                }
                b"wheel.position" if i < 16 => {
                    if let Some(p) = finite_vector(v) {
                        data.wheel_positions[i] = p;
                        data.wheel_position_mask |= 1 << i;
                    }
                }
                b"wheel.radius"
                    if i < 16
                        && v.type_ == SCS_VALUE_TYPE_float
                        && scalar(v).is_finite()
                        && scalar(v) > 0.0 =>
                {
                    data.wheel_radius[i] = scalar(v);
                    data.wheel_radius_mask |= 1 << i;
                }
                b"wheel.steerable" if i < 16 && v.type_ == SCS_VALUE_TYPE_bool => {
                    data.wheel_steerable[i] =
                        u32::from(unsafe { v.__bindgen_anon_1.value_bool.value } != 0);
                    data.wheel_steerable_mask |= 1 << i;
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn controls_events_preserve_causality_and_revoke_missing_invalid_or_duplicate_mode() {
        let header = WireHeader {
            frame_id: 10,
            qpc: 300,
            sim_time_us: 120,
            ..Default::default()
        };
        let mut previous = ControlConfiguration {
            session_start_qpc: 100,
            ..Default::default()
        };
        for (mode, kind, safe) in [
            (c"arcade", SHIFTER_ARCADE, false),
            (c"automatic", SHIFTER_AUTOMATIC, true),
            (c"manual", SHIFTER_MANUAL, true),
            (c"hshifter", SHIFTER_HSHIFTER, true),
            (c"unknown", SHIFTER_UNKNOWN, false),
        ] {
            let mut value = scs_value_t {
                type_: SCS_VALUE_TYPE_string,
                ..Default::default()
            };
            value.__bindgen_anon_1.value_string = scs_value_string_t {
                value: mode.as_ptr(),
            };
            let attr = attribute(c"shifter.type", u32::MAX, value);
            let attrs = [attr, scs_named_value_t::default()];
            let config = scs_telemetry_configuration_t {
                id: c"controls".as_ptr(),
                attributes: attrs.as_ptr(),
            };
            let result = controls(previous, &config, header);
            assert_eq!(result.generation, previous.generation + 1);
            assert_eq!(result.header.qpc, 300);
            assert_eq!(result.effective_from_frame_id, 11);
            assert_eq!(result.session_start_qpc, 100);
            assert_eq!(result.shifter_type, kind);
            assert_eq!(result.safe_shifter(), safe);
            previous = result;
            let duplicate = [attr, attr, scs_named_value_t::default()];
            assert_eq!(
                controls(
                    previous,
                    &scs_telemetry_configuration_t {
                        attributes: duplicate.as_ptr(),
                        ..config
                    },
                    header
                )
                .valid,
                0
            );
            let mut wrong = attr;
            wrong.value.type_ = SCS_VALUE_TYPE_float;
            let wrong = [wrong, scs_named_value_t::default()];
            assert_eq!(
                controls(
                    previous,
                    &scs_telemetry_configuration_t {
                        attributes: wrong.as_ptr(),
                        ..config
                    },
                    header
                )
                .valid,
                0
            );
            assert_eq!(
                controls(
                    previous,
                    &scs_telemetry_configuration_t {
                        attributes: ptr::null(),
                        ..config
                    },
                    header
                )
                .valid,
                0
            );
        }
    }
    fn attribute(name: &'static CStr, index: u32, value: scs_value_t) -> scs_named_value_t {
        scs_named_value_t {
            name: name.as_ptr(),
            index,
            value,
            ..Default::default()
        }
    }
    #[test]
    fn official_trailer_arrays_presence_empty_slot_and_timestamps_are_preserved() {
        let mut vector = scs_value_t {
            type_: SCS_VALUE_TYPE_fvector,
            ..Default::default()
        };
        vector.__bindgen_anon_1.value_fvector = scs_value_fvector_t {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let mut count = scs_value_t {
            type_: SCS_VALUE_TYPE_u32,
            ..Default::default()
        };
        count.__bindgen_anon_1.value_u32 = scs_value_u32_t { value: 18 };
        let attrs = [
            attribute(c"hook.position", u32::MAX, vector),
            attribute(c"wheels.count", u32::MAX, count),
            attribute(c"wheel.position", 15, vector),
            attribute(c"wheel.position", 16, vector),
            scs_named_value_t::default(),
        ];
        let config = scs_telemetry_configuration_t {
            id: c"trailer.9".as_ptr(),
            attributes: attrs.as_ptr(),
        };
        let truck = ConfigBlock {
            config_seq: 7,
            ..Default::default()
        };
        let header = WireHeader {
            frame_id: 41,
            qpc: 1000,
            sim_time_us: 500,
            ..Default::default()
        };
        let mut output = ConfigurationFrame::default();
        update(&mut output, truck, &config, header);
        assert_eq!(output.effective_from_frame_id, 42);
        assert_eq!(output.header.qpc, 1000);
        assert_eq!(output.config_seq, output.truck.config_seq);
        assert_eq!(output.present_trailer_mask, 1 << 9);
        let t = output.trailers[9];
        assert_eq!(t.hook_position, [1.0, 2.0, 3.0]);
        assert_eq!(t.wheel_position_mask, 1 << 15);
        assert_eq!(t.wheel_count, 18);
        assert_ne!(t.flags & CONFIG_TRUNCATED_WHEELS, 0);
        assert_eq!(t.wheel_radius_mask, 0);
        let empty = [scs_named_value_t::default()];
        let removed = scs_telemetry_configuration_t {
            attributes: empty.as_ptr(),
            ..config
        };
        update(
            &mut output,
            ConfigBlock {
                config_seq: 8,
                ..truck
            },
            &removed,
            WireHeader {
                qpc: 1200,
                ..header
            },
        );
        assert_eq!(output.present_trailer_mask, 0);
        assert_eq!(output.trailers[9].present, 0);
        assert_eq!(output.trailers[9].wheel_positions, [[0.0; 3]; 16]);
        assert_eq!(output.trailers[9].update_qpc, 1200);
        let alias = scs_telemetry_configuration_t {
            id: c"trailer".as_ptr(),
            attributes: attrs.as_ptr(),
        };
        update(&mut output, truck, &alias, header);
        assert_eq!(output.trailers[0].present, 1);
    }
}
