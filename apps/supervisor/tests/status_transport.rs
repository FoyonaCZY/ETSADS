#![cfg(windows)]
use etsads_msgs::protocol::{SCHEMA_VERSION, SUPERVISOR_STATUS_MAPPING, SupervisorStatus};
use etsads_shm::SharedCell;
use std::{
    os::windows::process::CommandExt,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
#[test]
fn independent_process_publishes_fresh_identity_without_claiming_reverse_permission() {
    let namespace = format!("status_{}_{}", std::process::id(), etsads_shm::qpc());
    let mut child = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_etsads-supervisor"))
            .args(["--namespace", &namespace, "--no-hotkey", "--duration", "1"])
            .creation_flags(0x08000000)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap(),
    );
    let started = Instant::now();
    let cell = loop {
        if let Ok(cell) = SharedCell::<SupervisorStatus>::open(
            &format!("{SUPERVISOR_STATUS_MAPPING}_{namespace}"),
            SCHEMA_VERSION,
        ) {
            break cell;
        }
        assert!(
            started.elapsed() < Duration::from_secs(3),
            "isolated supervisor status unavailable"
        );
        assert!(
            child.0.try_wait().unwrap().is_none(),
            "isolated supervisor exited before publishing"
        );
        std::thread::sleep(Duration::from_millis(10));
    };
    let first = loop {
        if let Some(status) = cell.latest() {
            break status;
        }
        assert!(started.elapsed() < Duration::from_secs(3));
        std::thread::sleep(Duration::from_millis(10));
    };
    assert_eq!(first.schema_version, 1);
    assert_eq!(first.process_id, child.0.id());
    assert_eq!(first.profile_sha256, [0; 32]);
    assert_eq!(first.reverse_ready, 0);
    std::thread::sleep(Duration::from_millis(40));
    let next = cell.latest().unwrap();
    assert!(next.heartbeat_qpc > first.heartbeat_qpc);
    assert!(etsads_safety::fresh(
        etsads_shm::qpc(),
        next.heartbeat_qpc,
        etsads_shm::qpc_frequency(),
        200
    ));
    assert!(child.0.wait().unwrap().success());
    let stopped = cell.latest().unwrap();
    assert_eq!(stopped.reverse_ready, 0);
    assert_eq!(stopped.configuration_sequence, 0);
}
