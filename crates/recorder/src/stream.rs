//! Bounded, checksum-validated MCAP reading. Own the input buffers instead of
//! relying on an unsafe promise that another process cannot mutate an mmap.
use anyhow::{Context, ensure};
use mcap::{
    records::Record as McapRecord,
    sans_io::linear_reader::{LinearReadEvent, LinearReader, LinearReaderOptions},
};
use std::{collections::BTreeMap, io::Read};

pub(crate) fn messages(
    mut input: impl Read,
    mut receive: impl FnMut(&str, &mcap::records::MessageHeader, &[u8]) -> anyhow::Result<()>,
) -> anyhow::Result<()> {
    const MAX_RECORD: usize = 16 * 1024 * 1024;
    let options = LinearReaderOptions::default()
        .with_prevalidate_chunk_crcs(true)
        .with_validate_data_section_crc(true)
        .with_validate_summary_section_crc(true)
        .with_check_finishes_after_end_magic(true)
        .with_record_length_limit(MAX_RECORD);
    let mut reader = LinearReader::new_with_options(options);
    let mut schemas = BTreeMap::new();
    let expected_descriptor = crate::descriptor();
    let mut channels = BTreeMap::<u16, mcap::records::Channel>::new();
    let mut metadata_bytes = 0usize;
    while let Some(event) = reader.next_event() {
        match event? {
            LinearReadEvent::ReadRequest(need) => {
                // Variable-sized chunk-header fields can request bytes before
                // the upstream reader applies its outer record-size limit.
                ensure!(need <= MAX_RECORD, "MCAP internal read exceeds 16 MiB");
                let count = input.read(reader.insert(need.clamp(1, 64 * 1024)))?;
                reader.notify_read(count);
            }
            LinearReadEvent::Record { opcode, data } => match mcap::parse_record(opcode, data)? {
                McapRecord::Schema { header, data } => {
                    ensure!(
                        header.id != 0
                            && header.name == "etsads.Record"
                            && header.encoding == "protobuf",
                        "unsupported MCAP schema"
                    );
                    ensure!(
                        data.as_ref() == expected_descriptor,
                        "MCAP descriptor differs from the declared ETSADS wire schema"
                    );
                    let value = (header.name, header.encoding, data.into_owned());
                    if let Some(previous) = schemas.get(&header.id) {
                        ensure!(previous == &value, "MCAP schema was redefined");
                    } else {
                        ensure!(schemas.len() < 16, "too many MCAP schemas");
                        metadata_bytes += value.0.len() + value.1.len() + value.2.len();
                        ensure!(
                            metadata_bytes <= 1024 * 1024,
                            "MCAP schema/channel metadata exceeds 1 MiB"
                        );
                        schemas.insert(header.id, value);
                    }
                }
                McapRecord::Channel(channel) => {
                    ensure!(
                        channel.message_encoding == "protobuf" && channel.topic.starts_with('/'),
                        "unsupported MCAP channel"
                    );
                    if let Some(previous) = channels.get(&channel.id) {
                        ensure!(previous == &channel, "MCAP channel was redefined");
                    } else {
                        ensure!(channels.len() < 256, "too many MCAP channels");
                        metadata_bytes += channel.topic.len()
                            + channel.message_encoding.len()
                            + channel
                                .metadata
                                .iter()
                                .map(|(k, v)| k.len() + v.len())
                                .sum::<usize>();
                        ensure!(
                            metadata_bytes <= 1024 * 1024,
                            "MCAP schema/channel metadata exceeds 1 MiB"
                        );
                        channels.insert(channel.id, channel);
                    }
                }
                McapRecord::Message { header, data } => {
                    let channel = channels
                        .get(&header.channel_id)
                        .context("message refers to unknown channel")?;
                    ensure!(
                        schemas.contains_key(&channel.schema_id),
                        "channel refers to unknown schema"
                    );
                    receive(&channel.topic, &header, &data)?;
                }
                _ => {}
            },
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn sample() -> Vec<u8> {
        let mut writer = mcap::Writer::new(Cursor::new(Vec::new())).unwrap();
        let schema = writer
            .add_schema("etsads.Record", "protobuf", &crate::descriptor())
            .unwrap();
        let channel = writer
            .add_channel(schema, "/probe", "protobuf", &BTreeMap::new())
            .unwrap();
        writer
            .write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: channel,
                    sequence: 1,
                    log_time: 0,
                    publish_time: 0,
                },
                b"ETSADS-CHECKSUM-PROBE-ORIGINAL",
            )
            .unwrap();
        writer.finish().unwrap();
        writer.into_inner().into_inner()
    }

    #[test]
    fn detects_payload_corruption_before_delivering_chunk_messages() {
        let mut bytes = sample();
        let mut count = 0;
        messages(Cursor::new(&bytes), |topic, _, payload| {
            assert_eq!(topic, "/probe");
            assert_eq!(payload, b"ETSADS-CHECKSUM-PROBE-ORIGINAL");
            count += 1;
            Ok(())
        })
        .unwrap();
        assert_eq!(count, 1);
        let marker = b"ETSADS-CHECKSUM-PROBE-ORIGINAL";
        let position = bytes
            .windows(marker.len())
            .position(|v| v == marker)
            .unwrap();
        bytes[position] ^= 1;
        let mut delivered = false;
        assert!(
            messages(Cursor::new(bytes), |_, _, _| {
                delivered = true;
                Ok(())
            })
            .is_err()
        );
        assert!(!delivered);
    }

    #[test]
    fn detects_truncation_and_limits_malicious_record_lengths() {
        let bytes = sample();
        assert!(messages(Cursor::new(&bytes[..bytes.len() - 3]), |_, _, _| Ok(())).is_err());
        let mut malicious = mcap::MAGIC.to_vec();
        malicious.push(mcap::records::op::HEADER);
        malicious.extend_from_slice(&u64::MAX.to_le_bytes());
        assert!(messages(Cursor::new(malicious), |_, _, _| Ok(())).is_err());
        let mut chunk = mcap::MAGIC.to_vec();
        chunk.push(mcap::records::op::CHUNK);
        chunk.extend_from_slice(&40_u64.to_le_bytes());
        let mut header = [0_u8; 40];
        header[28..32].copy_from_slice(&(1_u32 << 30).to_le_bytes());
        chunk.extend_from_slice(&header);
        let error = messages(Cursor::new(chunk), |_, _, _| Ok(())).unwrap_err();
        assert!(error.to_string().contains("internal read"));
    }
    #[test]
    fn bounds_retained_channel_metadata() {
        let mut writer = mcap::Writer::new(Cursor::new(Vec::new())).unwrap();
        let schema = writer
            .add_schema("etsads.Record", "protobuf", &crate::descriptor())
            .unwrap();
        let metadata = BTreeMap::from([("large".into(), "x".repeat(1024 * 1024))]);
        writer
            .add_channel(schema, "/probe", "protobuf", &metadata)
            .unwrap();
        writer.finish().unwrap();
        let error = messages(Cursor::new(writer.into_inner().into_inner()), |_, _, _| {
            Ok(())
        })
        .unwrap_err();
        assert!(error.to_string().contains("metadata exceeds"));
    }
}
