//! Explicit stationary acceptance of the same planned-finish parking handshake.
use super::{
    Options, Report,
    core::Output,
    io,
    stopping::{Action, StopSequence},
};
use anyhow::{Context, ensure};
use etsads_game::adapter::GameAdapter;
use etsads_msgs::protocol::*;
use etsads_shm::{SharedRing, SingleWriterGuard, qpc, qpc_frequency};
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

pub(super) fn run(arm: bool, stop: Arc<AtomicBool>) -> anyhow::Result<Report> {
    ensure!(arm, "驻车收尾实测需要显式 --arm，且车辆须熄火驻车");
    let _lease = SingleWriterGuard::acquire("ETSADS_Control_Writer")?;
    let mut trace = io::Trace::open(&Options::default().log_dir)?;
    trace.write(
        &serde_json::json!({"kind":"stationary_finish_check","source":"game","zero_throttle":true}),
    )?;
    let _supervisor = io::SupervisorProcess::ensure()?;
    let mut sink = io::Sink::connect()?;
    let actions = SharedRing::<InputActionFrame, 256>::open(INPUT_ACTIONS_MAPPING, SCHEMA_VERSION)?;
    let mut controls = crate::loaded_controls::LoadedControls::connect("")?;
    let mut game = GameAdapter::connect()?;
    let mut warmup = crate::InputWarmup::default();
    let started = Instant::now();
    let mut last_observation = Instant::now();
    let mut initial = None::<TelemetryFrame>;
    let mut arm_time = None::<Instant>;
    let mut granted = false;
    let mut holding_since = None::<Instant>;
    let mut release = None::<io::ParkingReceipt>;
    let mut sequence = None::<StopSequence>;
    let mut parking = None::<io::ParkingReceipt>;
    let mut frames = 0_u64;
    let outcome = (|| -> anyhow::Result<()> {
        loop {
            ensure!(!stop.load(Ordering::Acquire), "用户停止驻车实测");
            ensure!(
                started.elapsed() < Duration::from_secs(10),
                "驻车实测超时，请检查驻车状态"
            );
            let Some(s) = game.poll()? else {
                ensure!(
                    last_observation.elapsed() < Duration::from_millis(200),
                    "驻车实测遥测中断"
                );
                std::thread::sleep(Duration::from_millis(2));
                continue;
            };
            last_observation = Instant::now();
            let g = crate::loaded_controls::coherent_read(|| sink.gate.latest())
                .context("安全监督不可读")?;
            let a = crate::loaded_controls::coherent_read(|| sink.applied.latest())
                .context("输入回调不可读")?;
            let now = qpc();
            ensure!(
                s.raw.paused == 0 && s.raw.rpm < 100.0 && s.raw.speed_mps.abs() < 0.05,
                "请熄火并停稳，保持驾驶画面"
            );
            ensure!(
                etsads_safety::fresh(now, s.raw.header.qpc, qpc_frequency(), 150)
                    && etsads_safety::fresh(now, g.heartbeat_qpc, qpc_frequency(), 150)
                    && etsads_safety::fresh(now, a.applied_qpc, qpc_frequency(), 150),
                "实测数据过期"
            );
            controls.validate(&s.raw)?;
            if let Some(first) = &initial {
                crate::loaded_controls::validate_stationary_steering(first, &s.raw)?;
            } else {
                ensure!(s.raw.parking_brake != 0, "实测开始前须驻车");
                controls.bind(&s.raw)?;
                initial = Some(s.raw);
            }
            let brake = Output {
                steer: 0.0,
                throttle: 0.0,
                brake: 0.55,
            };
            if arm_time.is_none() {
                sink.publish(s.raw.header.frame_id, brake)?;
                if warmup.observe(true, s.raw.header.frame_id, Instant::now()) {
                    sink.request(true)?;
                    arm_time = Some(Instant::now());
                }
                continue;
            }
            if !granted {
                if g.enabled == GATE_ACTIVE {
                    granted = true;
                } else {
                    ensure!(
                        arm_time.unwrap().elapsed() < Duration::from_millis(300),
                        "安全监督拒绝驻车实测"
                    );
                    sink.publish(s.raw.header.frame_id, brake)?;
                    continue;
                }
            }
            ensure!(
                g.enabled == GATE_ACTIVE,
                "人工接管或安全监督撤销授权，已停止"
            );
            let mut published = false;
            if release.is_none() {
                if a.axes[2] >= 0.5 {
                    holding_since.get_or_insert(Instant::now());
                } else {
                    holding_since = None;
                }
                if holding_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(300)) {
                    release = Some(sink.request_parking(&s.raw, &actions, 0.0, false)?);
                    published = true;
                    trace.write(&serde_json::json!({"event":"parking_release_requested","frame":s.raw.header.frame_id}))?;
                }
            } else if let Some(receipt) = &mut release
                && sequence.is_none()
                && receipt.confirmed(&actions, &s.raw)?
            {
                sequence = Some(StopSequence::new(started.elapsed().as_secs_f64()));
                trace.write(&serde_json::json!({"event":"release_completed_with_service_brake","frame":s.raw.header.frame_id,"sdk_evidence":receipt.evidence()}))?;
            }
            if let Some(sequence) = &mut sequence {
                let acknowledged = if let Some(receipt) = &mut parking {
                    receipt.confirmed(&actions, &s.raw)?
                } else {
                    false
                };
                match sequence.update(
                    started.elapsed().as_secs_f64(),
                    s.raw.speed_mps as f64,
                    s.raw.parking_brake != 0,
                    acknowledged,
                )? {
                    Action::RequestParking => {
                        parking = Some(sink.park(&s.raw, &actions, 0.0)?);
                        published = true;
                        trace.write(&serde_json::json!({"event":"parking_requested_once","frame":s.raw.header.frame_id}))?;
                    }
                    Action::Complete => {
                        trace.write(&serde_json::json!({"event":"parking_confirmed","frame":s.raw.header.frame_id,"qpc":s.raw.header.qpc,"sdk_evidence":parking.as_ref().and_then(|p|p.evidence())}))?;
                        return Ok(());
                    }
                    Action::Brake(_) => {}
                }
            }
            ensure!(!stop.load(Ordering::Acquire), "用户停止驻车实测");
            if !published {
                sink.publish(s.raw.header.frame_id, brake)?;
            }
            frames += 1;
            trace.write(&serde_json::json!({"frame":s.raw.header.frame_id,"qpc":s.raw.header.qpc,"speed":s.raw.speed_mps,"position_scs":s.raw.position_scs,"rpm":s.raw.rpm,"parking_brake":s.raw.parking_brake,"requested_brake":0.55,"applied":a.axes,"gate":g.enabled}))?;
        }
    })();
    drop(sink);
    let success = outcome.is_ok();
    let reason = match outcome {
        Ok(()) => "零油门驻车收尾实测通过：释放与重新驻车均有SDK完成记录，末尾驻车已确认".into(),
        Err(e) => format!("驻车实测未通过：{e:#}"),
    };
    trace.write(&serde_json::json!({"complete":true,"passed":success,"reason":reason}))?;
    trace.flush()?;
    Ok(Report {
        source: "game_stationary_finish_check",
        stop_reason: reason,
        active_frames: frames,
        log: trace.path.clone(),
        full_acceptance: false,
        parking_confirmed: success,
    })
}
