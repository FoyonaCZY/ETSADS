//! MCAP recording with an embedded protobuf descriptor and loss accounting.
//! Simulation records are labelled separately and cannot satisfy game acceptance.
pub mod loss;
mod stream;
use anyhow::{Context, ensure};
use etsads_msgs::protocol::{SCHEMA_VERSION, TelemetryFrame};
pub use loss::{LOSS_TOPIC, LossPhase, RingLoss};
use prost::Message;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{BufWriter, Read, Write},
    path::{Path, PathBuf},
};

#[derive(Clone, PartialEq, Message)]
pub struct Record {
    #[prost(uint32, tag = "1")]
    pub schema_version: u32,
    #[prost(uint64, tag = "2")]
    pub frame_id: u64,
    #[prost(uint64, tag = "3")]
    pub sim_time_us: u64,
    #[prost(uint64, tag = "4")]
    pub qpc: u64,
    #[prost(bytes = "vec", tag = "5")]
    pub raw_payload: Vec<u8>,
    #[prost(double, tag = "6")]
    pub speed_mps: f64,
    #[prost(double, repeated, tag = "7")]
    pub position_scs: Vec<f64>,
}
impl Record {
    pub fn wire<T: bytemuck::Pod>(value: &T, frame_id: u64, sim_time_us: u64, qpc: u64) -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            frame_id,
            sim_time_us,
            qpc,
            raw_payload: bytemuck::bytes_of(value).to_vec(),
            ..Default::default()
        }
    }
    pub fn telemetry(t: &TelemetryFrame) -> Self {
        Self {
            speed_mps: t.speed_mps as f64,
            position_scs: t.position_scs.to_vec(),
            ..Self::wire(t, t.header.frame_id, t.header.sim_time_us, t.header.qpc)
        }
    }
    pub fn decode_wire<T: bytemuck::Pod>(&self) -> anyhow::Result<T> {
        ensure!(
            self.schema_version == SCHEMA_VERSION,
            "unsupported wire schema"
        );
        ensure!(
            self.raw_payload.len() == std::mem::size_of::<T>(),
            "payload size mismatch"
        );
        Ok(bytemuck::pod_read_unaligned(&self.raw_payload))
    }
}
fn descriptor() -> Vec<u8> {
    use prost_types::{
        DescriptorProto, FieldDescriptorProto, FileDescriptorProto, FileDescriptorSet,
        field_descriptor_proto::{Label, Type},
    };
    let fields = [
        ("schema_version", Type::Uint32, false),
        ("frame_id", Type::Uint64, false),
        ("sim_time_us", Type::Uint64, false),
        ("qpc", Type::Uint64, false),
        ("raw_payload", Type::Bytes, false),
        ("speed_mps", Type::Double, false),
        ("position_scs", Type::Double, true),
    ]
    .into_iter()
    .enumerate()
    .map(|(i, (name, ty, repeated))| FieldDescriptorProto {
        name: Some(name.into()),
        number: Some(i as i32 + 1),
        label: Some(if repeated {
            Label::Repeated
        } else {
            Label::Optional
        } as i32),
        r#type: Some(ty as i32),
        ..Default::default()
    })
    .collect();
    FileDescriptorSet {
        file: vec![FileDescriptorProto {
            name: Some("record.proto".into()),
            package: Some("etsads".into()),
            syntax: Some("proto3".into()),
            message_type: vec![DescriptorProto {
                name: Some("Record".into()),
                field: fields,
                ..Default::default()
            }],
            ..Default::default()
        }],
    }
    .encode_to_vec()
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RunMetadata {
    pub format_version: u32,
    pub source: String,
    pub game_version: String,
    pub started_unix_s: u64,
    pub git_commit: String,
    pub completed: bool,
    pub messages: u64,
    pub dropped_frames: u64,
    pub notes: Vec<String>,
    #[serde(default)]
    pub environment: serde_json::Value,
}
impl RunMetadata {
    pub fn new(source: &str, game_version: &str) -> Self {
        Self {
            format_version: 1,
            source: source.into(),
            game_version: game_version.into(),
            started_unix_s: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            git_commit: option_env!("ETSADS_GIT_COMMIT")
                .unwrap_or("unversioned")
                .into(),
            completed: false,
            messages: 0,
            dropped_frames: 0,
            notes: vec![],
            environment: serde_json::Value::Null,
        }
    }
}

pub struct Recorder {
    dir: PathBuf,
    writer: Option<mcap::Writer<BufWriter<File>>>,
    channels: BTreeMap<String, u16>,
    schema: u16,
    part: u32,
    bytes: u64,
    first_time: Option<u64>,
    meta: RunMetadata,
    loss_sequence: u64,
}
impl Recorder {
    pub fn create(path: &Path, meta: RunMetadata) -> anyhow::Result<Self> {
        std::fs::create_dir_all(path)?;
        let lock = path.join("meta.json");
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock)
            .context("run already exists; use a new output directory")?;
        serde_json::to_writer_pretty(file, &meta)?;
        let mut this = Self {
            dir: path.to_owned(),
            writer: None,
            channels: BTreeMap::new(),
            schema: 0,
            part: 0,
            bytes: 0,
            first_time: None,
            meta,
            loss_sequence: 0,
        };
        this.open_part()?;
        Ok(this)
    }
    fn open_part(&mut self) -> anyhow::Result<()> {
        let f = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(self.dir.join(format!("data-{:04}.mcap", self.part)))?;
        let mut w = mcap::Writer::new(BufWriter::new(f))?;
        self.schema = w.add_schema("etsads.Record", "protobuf", &descriptor())?;
        self.writer = Some(w);
        self.channels.clear();
        self.bytes = 0;
        self.first_time = None;
        Ok(())
    }
    pub fn record(&mut self, topic: &str, record: &Record) -> anyhow::Result<()> {
        ensure!(topic.starts_with('/'), "topic must be absolute");
        ensure!(
            record.schema_version == SCHEMA_VERSION,
            "unsupported record schema"
        );
        let rotate = self.bytes >= 3_900_000_000
            || self
                .first_time
                .is_some_and(|first| record.sim_time_us.saturating_sub(first) >= 1_800_000_000);
        if rotate {
            self.close_data()?;
            self.part += 1;
            self.open_part()?;
        }
        let writer = self.writer.as_mut().context("recorder is closed")?;
        let id = match self.channels.get(topic) {
            Some(id) => *id,
            None => {
                let id = writer.add_channel(self.schema, topic, "protobuf", &BTreeMap::new())?;
                self.channels.insert(topic.into(), id);
                id
            }
        };
        let data = record.encode_to_vec();
        let time = record
            .sim_time_us
            .checked_mul(1000)
            .context("simulation timestamp overflow")?;
        writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: id,
                sequence: self.meta.messages as u32,
                log_time: time,
                publish_time: time,
            },
            &data,
        )?;
        self.bytes += data.len() as u64 + 32;
        self.meta.messages += 1;
        self.first_time.get_or_insert(record.sim_time_us);
        Ok(())
    }
    pub fn add_dropped(&mut self, n: u64) {
        self.meta.dropped_frames += n;
    }
    /// Emit one bounded diagnostic per observed overwritten ring range. Zero
    /// frame/sim-time in this metadata envelope explicitly means unavailable;
    /// only the independently measured observation QPC is populated.
    pub fn record_ring_loss(&mut self, loss: &RingLoss) -> anyhow::Result<()> {
        loss.validate()?;
        let sequence = self
            .loss_sequence
            .checked_add(1)
            .context("loss sequence overflow")?;
        let payload = serde_json::to_vec(&serde_json::json!({
            "encoding":"json", "schema_version":1, "source":self.meta.source,
            "kind":"ring_overrun", "sequence":sequence, "observation":loss,
            "lost_ring_index_range":[loss.last_read+1,loss.earliest_available-1],
            "lost_physical_frame_range":null, "lost_sample_qpc_range":null,
            "timestamp_semantics":"QPC is recorder observation time; lost sample times and physical frames are unknown"
        }))?;
        ensure!(
            payload.len() <= 4096,
            "loss observation exceeds 4 KiB budget"
        );
        if loss.counted_in_dropped_frames {
            self.meta.dropped_frames = self
                .meta
                .dropped_frames
                .checked_add(loss.count)
                .context("dropped-frame aggregate overflow")?;
        }
        self.record(
            LOSS_TOPIC,
            &Record {
                schema_version: SCHEMA_VERSION,
                qpc: loss.observed_qpc,
                raw_payload: payload,
                ..Default::default()
            },
        )?;
        self.loss_sequence = sequence;
        Ok(())
    }
    fn close_data(&mut self) -> anyhow::Result<()> {
        let mut writer = self.writer.take().context("recorder is closed")?;
        let finish = writer.finish();
        let mut file = writer.into_inner();
        let flush = file.flush();
        let sync = file.get_ref().sync_all();
        finish?;
        flush?;
        sync?;
        Ok(())
    }
    fn finalize(mut self, completed: bool, reason: Option<String>) -> anyhow::Result<RunMetadata> {
        if let Some(reason) = reason {
            self.meta.notes.push(reason);
        }
        let closed = self.close_data();
        self.meta.completed = completed && closed.is_ok();
        if let Err(error) = &closed {
            self.meta
                .notes
                .push(format!("MCAP finalization failed: {error:#}"));
        }
        let tmp = self.dir.join("meta.json.tmp");
        let mut file = File::create(&tmp)?;
        serde_json::to_writer_pretty(&mut file, &self.meta)?;
        file.sync_all()?;
        std::fs::rename(tmp, self.dir.join("meta.json"))?;
        closed?;
        Ok(self.meta.clone())
    }
    pub fn finish(self) -> anyhow::Result<RunMetadata> {
        self.finalize(true, None)
    }
    /// Close/flush a partial run without marking its experiment successful.
    /// A structurally readable MCAP remains useful for failure diagnosis.
    pub fn finish_incomplete(self, reason: impl Into<String>) -> anyhow::Result<RunMetadata> {
        self.finalize(false, Some(reason.into()))
    }
}

#[derive(Default, Debug, Serialize, Deserialize)]
pub struct ReplayReport {
    pub source: String,
    pub completed: bool,
    pub messages: u64,
    pub telemetry_frames: u64,
    pub frame_gaps: u64,
    pub nonmonotonic: u64,
    pub duration_sim_s: f64,
    pub topics: BTreeMap<String, u64>,
}
pub fn replay(
    path: &Path,
    mut receive: impl FnMut(&str, &Record) -> anyhow::Result<()>,
) -> anyhow::Result<ReplayReport> {
    let mut metadata_bytes = Vec::new();
    File::open(path.join("meta.json"))?
        .take(8 * 1024 * 1024 + 1)
        .read_to_end(&mut metadata_bytes)?;
    ensure!(
        metadata_bytes.len() <= 8 * 1024 * 1024,
        "run metadata exceeds 8 MiB"
    );
    let meta: RunMetadata = serde_json::from_slice(&metadata_bytes)?;
    let mut parts = std::fs::read_dir(path)?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "mcap"))
        .collect::<Vec<_>>();
    parts.sort();
    ensure!(!parts.is_empty(), "no MCAP data files");
    let mut report = ReplayReport {
        source: meta.source,
        completed: meta.completed,
        ..Default::default()
    };
    let mut last = None;
    let mut first_time = None;
    let mut last_time = 0;
    for part in parts {
        let mut options = OpenOptions::new();
        options.read(true);
        #[cfg(windows)]
        {
            use std::os::windows::fs::OpenOptionsExt;
            options.share_mode(1);
        }
        let file = options
            .open(&part)
            .context("recording must be closed before replay")?;
        stream::messages(file, |topic, message_header, data| {
            let record = Record::decode(data)?;
            let timestamp = record
                .sim_time_us
                .checked_mul(1000)
                .context("simulation timestamp overflow")?;
            ensure!(
                message_header.log_time == timestamp && message_header.publish_time == timestamp,
                "MCAP timestamps disagree with record envelope"
            );
            ensure!(
                record.schema_version == SCHEMA_VERSION,
                "schema mismatch in recording"
            );
            *report.topics.entry(topic.to_owned()).or_default() += 1;
            report.messages += 1;
            if topic == "/telemetry" {
                let raw: TelemetryFrame = record.decode_wire()?;
                ensure!(
                    raw.header.frame_id == record.frame_id
                        && raw.header.sim_time_us == record.sim_time_us
                        && raw.header.qpc == record.qpc,
                    "telemetry envelope disagrees with wire header"
                );
                report.telemetry_frames += 1;
                if let Some(prev) = last {
                    if record.frame_id <= prev || record.sim_time_us < last_time {
                        report.nonmonotonic += 1;
                    } else {
                        report.frame_gaps += record.frame_id - prev - 1;
                    }
                }
                last = Some(record.frame_id);
                first_time.get_or_insert(record.sim_time_us);
                last_time = record.sim_time_us;
            }
            receive(topic, &record)?;
            Ok(())
        })?;
    }
    report.duration_sim_s = last_time.saturating_sub(first_time.unwrap_or(0)) as f64 / 1e6;
    ensure!(
        report.messages == meta.messages || !meta.completed,
        "recorded message count disagrees with manifest"
    );
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip_and_loss_detection() {
        let dir = std::env::temp_dir().join(format!(
            "etsads-record-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = Recorder::create(&dir, RunMetadata::new("synthetic", "none")).unwrap();
        for id in [1, 2, 4] {
            let mut t = TelemetryFrame::default();
            t.header.frame_id = id;
            t.header.sim_time_us = id * 16_667;
            t.speed_mps = 12.0;
            writer.record("/telemetry", &Record::telemetry(&t)).unwrap();
        }
        writer.finish().unwrap();
        let report = replay(&dir, |_, r| {
            assert_eq!(r.decode_wire::<TelemetryFrame>()?.speed_mps, 12.0);
            Ok(())
        })
        .unwrap();
        assert_eq!(report.telemetry_frames, 3);
        assert_eq!(report.frame_gaps, 1);
        assert_eq!(report.source, "synthetic");
        assert!(report.completed);
        std::fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn malformed_wire_rejected() {
        let r = Record {
            schema_version: SCHEMA_VERSION,
            raw_payload: vec![0; 3],
            ..Default::default()
        };
        assert!(r.decode_wire::<TelemetryFrame>().is_err());
    }
    #[test]
    fn rejects_envelope_and_raw_frame_disagreement() {
        let dir = std::env::temp_dir().join(format!(
            "etsads-envelope-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = Recorder::create(&dir, RunMetadata::new("synthetic", "none")).unwrap();
        let mut raw = TelemetryFrame::default();
        raw.header.frame_id = 1;
        let mut record = Record::telemetry(&raw);
        record.frame_id = 2;
        writer.record("/telemetry", &record).unwrap();
        writer.finish().unwrap();
        let error = replay(&dir, |_, _| Ok(())).unwrap_err();
        assert!(error.to_string().contains("envelope"));
        std::fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn rejects_false_descriptor_and_wrong_mcap_clock_independently() {
        for invalid_schema in [true, false] {
            let dir = std::env::temp_dir().join(format!(
                "etsads-mcap-contract-test-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ));
            std::fs::create_dir(&dir).unwrap();
            let mut meta = RunMetadata::new("synthetic", "validator audit");
            meta.completed = true;
            meta.messages = 1;
            std::fs::write(dir.join("meta.json"), serde_json::to_vec(&meta).unwrap()).unwrap();
            let mut writer =
                mcap::Writer::new(File::create(dir.join("data-0000.mcap")).unwrap()).unwrap();
            let schema = writer
                .add_schema(
                    "etsads.Record",
                    "protobuf",
                    &if invalid_schema {
                        vec![0xff]
                    } else {
                        descriptor()
                    },
                )
                .unwrap();
            let channel = writer
                .add_channel(schema, "/telemetry", "protobuf", &BTreeMap::new())
                .unwrap();
            let mut raw = TelemetryFrame::default();
            raw.header.frame_id = 1;
            raw.header.sim_time_us = 100;
            let time = if invalid_schema { 100_000 } else { 999_999 };
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader {
                        channel_id: channel,
                        sequence: 0,
                        log_time: time,
                        publish_time: time,
                    },
                    &Record::telemetry(&raw).encode_to_vec(),
                )
                .unwrap();
            writer.finish().unwrap();
            drop(writer);
            let error = replay(&dir, |_, _| Ok(())).unwrap_err();
            assert!(error.to_string().contains(if invalid_schema {
                "descriptor"
            } else {
                "timestamps"
            }));
            std::fs::remove_dir_all(dir).unwrap();
        }
    }
    #[test]
    fn failed_experiment_is_readable_but_never_complete() {
        let dir = std::env::temp_dir().join(format!(
            "etsads-partial-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = Recorder::create(&dir, RunMetadata::new("synthetic", "none")).unwrap();
        let mut t = TelemetryFrame::default();
        t.header.frame_id = 1;
        writer.record("/telemetry", &Record::telemetry(&t)).unwrap();
        let meta = writer
            .finish_incomplete("planned failure injection")
            .unwrap();
        assert!(!meta.completed);
        assert!(meta.notes.iter().any(|n| n.contains("failure")));
        let report = replay(&dir, |_, _| Ok(())).unwrap();
        assert!(!report.completed);
        assert_eq!(report.messages, 1);
        std::fs::remove_dir_all(dir).unwrap();
    }
}
