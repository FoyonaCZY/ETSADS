# Recorded snapshot replay

`ReplayAdapter::run(run, callback)` reconstructs the same `GameSnapshot` type consumed by live Runtime. It never opens a shared mapping or publishes input. The entire closed MCAP recording is validated before delivery, then a second streaming pass associates observations with a bounded 64-frame lookahead. A packet arriving after its frame was delivered is an error; it is never attached to the nearest newer frame.

Telemetry is delivered once in physical-frame order. Extension data, angular dynamics and camera-view diagnostics require matching physical timestamps. Missing channels stay `None`. Gameplay events are delivered once, on the first recorded frame whose frame ID and QPC reach the event. A trailing event with no later telemetry remains a reported tail event. Local origin and session identity remain stable across frames; recordings spanning distinct SDK sessions are rejected.

Legacy `/config` is an observation, not a precise SDK event boundary. `/config_full` retains the real callback QPC and the earliest frame allowed to use it (the previously completed frame plus one). Replay also requires the physical-frame QPC to reach the event QPC. Because telemetry is stamped at frame start, a configuration callback during a step can conservatively be deferred until a later snapshot; this is not proof of the SDK's first affected physics step. A newer legacy observation cannot backdate this event. Both the original configuration packet and optional sidecars are exposed to consumers. Existing recordings are not modified or backfilled.

Video associations retain encoded-frame/capture-frame/QPC identity and the original alignment status. This adapter supplies the video index, not fabricated pixels or a nearest-camera substitution; the viewer opens the corresponding encoded video. Missing extensions, unmatched video and absent configuration are counted separately from MCAP integrity.

```powershell
# Existing integrity-only check also supports historical benchmark records.
etsads-cli.exe replay runs/session
# Reconstruct all GameSnapshots and audit associations.
etsads-cli.exe replay runs/session --snapshots
# Run the production controller/planner on recorded observations.
etsads-cli.exe replay-drive runs/session --config replay-runtime.json --calibration vehicle.json --output artifacts/replay-result
```

`replay-drive` requires `execution: "Replay"` in the runtime configuration. Lane modes additionally require `--map` and `--policy`. It writes `runtime.jsonl` and `report.json`, including proposed control, recorded human/game inputs and effective inputs, faults, the controller actually executed and source integrity. Each result explicitly states `control_written=false` and `phase_acceptance=false`. Runtime faults are retained; successful file processing does not mean a driving scenario passed.

This is **open-loop** replay: recorded traffic and ego states do not respond to proposed actuation. Closed-loop vehicle simulation and actual game trials remain separate requirements. Early throughput benchmarks with a missing QPC frequency can still be checked for file integrity; they cannot be turned into usable driving snapshots by guessing a clock.
