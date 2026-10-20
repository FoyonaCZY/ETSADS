//! Read-only camera/video correspondence audit of an immutable recording.
use anyhow::{Context, ensure};
use etsads_msgs::protocol::{ExtensionFrame, TelemetryFrame, VideoIndex, WireHeader};
use std::collections::BTreeMap;
fn main() -> anyhow::Result<()> {
    let run = std::env::args()
        .nth(1)
        .context("usage: camera_alignment_probe RUN")?;
    let mut telemetry = BTreeMap::new();
    let mut extensions = BTreeMap::new();
    let mut video = Vec::new();
    let mut frequency = 0;
    etsads_recorder::replay(std::path::Path::new(&run), |topic, record| {
        match topic {
            "/telemetry" => {
                let raw: TelemetryFrame = record.decode_wire()?;
                frequency = raw.qpc_frequency;
                telemetry.insert(raw.header.frame_id, raw.header.qpc);
            }
            "/extension" => {
                ensure!(
                    record.raw_payload.len() == std::mem::size_of::<ExtensionFrame>(),
                    "invalid extension size"
                );
                let header: WireHeader = bytemuck::pod_read_unaligned(&record.raw_payload[..40]);
                let available = u32::from_le_bytes(record.raw_payload[40..44].try_into()?);
                extensions.insert(header.frame_id, (header.qpc, available));
            }
            "/video_index" => video.push(record.decode_wire::<VideoIndex>()?),
            _ => {}
        }
        Ok(())
    })?;
    let mut missing = Vec::new();
    for frame in &video {
        if !extensions
            .get(&frame.physics_frame)
            .is_some_and(|(_, available)| available & 1 != 0)
        {
            let before = extensions.range(..=frame.physics_frame).next_back();
            let after = extensions.range(frame.physics_frame..).next();
            missing.push(serde_json::json!({"video_frame":frame.video_frame,"physics_frame":frame.physics_frame,"capture_frame":frame.capture_frame,"capture_qpc":frame.capture_qpc,"telemetry_present":telemetry.contains_key(&frame.physics_frame),"extension_record_available":extensions.get(&frame.physics_frame),"nearest_extension_before":before,"nearest_extension_after":after,"first_extension_minus_capture_ms":extensions.first_key_value().map(|(_,e)|(e.0 as i128-frame.capture_qpc as i128)as f64*1000.0/frequency.max(1)as f64)}));
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"source":"recorded_game_audit","run":run,"telemetry_frames":telemetry.len(),"extension_frames":extensions.len(),"camera_frames":extensions.values().filter(|(_,a)|a&1!=0).count(),"video_frames":video.len(),"first_telemetry":telemetry.first_key_value(),"first_extension":extensions.first_key_value(),"last_telemetry":telemetry.last_key_value(),"last_extension":extensions.last_key_value(),"missing_camera_count":missing.len(),"missing":missing})
        )?
    );
    Ok(())
}
