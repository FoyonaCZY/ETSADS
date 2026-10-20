# Independent safety supervision

The supervisor starts disarmed. `F8` arms/disarms; Escape or driving keys release control. It reads command → raw extension → configuration → telemetry → QPC to avoid treating a concurrently published command/extension as a future physics frame. Actual future timestamps, stale data, frame mismatch, invalid commands, pause and manual takeover still fail closed.

The source protocol now distinguishes `Gate.enabled = 0` (Off), `1` (Active) and `2` (BrakeOnly). If a previously authorized command is unchanged for more than 200 ms while required safety data stays valid, the supervisor revokes main-stack authority and latches that command's original timestamp. BrakeOnly ignores recovered commands and permits only the supervisor's 0→0.3 brake ramp over the next 500 ms. It ends at 1 s after the original command; explicit rearming is required. Off, manual takeover, pause, invalid safety data and the hotkey during BrakeOnly cancel it immediately. A missing supervisor heartbeat still releases all inputs within 300 ms.

These semantics resolve the conflicting command-loss and gate-release descriptions in the original implementation document: withdrawal of main-stack authority and expiry of the supervisor's bounded braking are separate checks. Old bridge binaries reject mode 2 and release immediately, so a mode-2 Gate recording alone cannot prove an actual brake response. The deployed DLL and physical brake response require separate version/hash and in-game evidence.

Without a separately verified safety profile, reverse gear or negative speed prohibits throttle. Rolling backward also requests braking. A profile is loaded explicitly:

```powershell
cargo run -p etsads-supervisor -- --safety-profile path/to/verified-safety-profile.json
```

The JSON must deserialize as `etsads_safety::reverse::SafetyGeometryProfile`. Its evidence file must exist; relative evidence paths are resolved against the profile file's directory. The profile contains:

- Schema version, `validated_for_reverse`, evidence path, raw configuration SHA-256, exact configuration sequence, and an observed cargo-mass range.
- Explicit non-steering rear-wheel indices and the SDK-origin-to-rear-axle offset.
- `VehicleGeometry`, including mandatory `has_trailer`. Tractor-only models have zero trailer length and wheelbase and `trailer: null`.
- Independently measured tractor front/rear overhangs. For a single attached trailer: hitch-to-front and axle-to-rear overhangs, the SDK trailer-reference-to-axle offset, and the allowed axle-consistency error.
- Optional measured `vertical_extent_from_rear_m: [bottom, top]`. Without this evidence, obstacles are not discarded based on height. With it, separation is checked using full obstacle dimensions and rotation, including tall or tilted objects whose centres lie on another level.

The safety crate validates those measurements directly against raw `/config`, actual connected-trailer count, and trailer world placement/heading. It does not link the controller, planner, fused Scene or their collision routines. Trailer attachment changes latch reverse permission off for that supervisor instance; a changed configuration/load requires a newly verified profile.

Live parking hosts additionally call `SafetyGeometryProfile::validate_planner_envelope` against the planner's explicit measured body offsets. Matching total lengths alone is insufficient: each tractor and trailer front/rear extent must contain its independently measured counterpart. The profile keeps its own measurements and collision implementation; its `geometry.measured_envelope` may be absent because the independent overhang fields already supply that information. This preserves old profile parsing without treating the planner's legacy synthetic offsets as measured.

The independent AEB parses raw traffic and parked-vehicle records, follows measured wheel steering with the signed hitch offset, and checks moving tractor/trailer envelopes continuously between 50 ms prediction samples. It catches rearward approaching traffic and tractor front-corner swing during reversing. Reverse speed is capped at 3 km/h. Missing/corrupt rear obstacle channels cut throttle and brake a rolling vehicle.

The separate `ETSADS_SupervisorStatus` mapping publishes schema version, process identity, a fresh QPC heartbeat, the SHA-256 of the exact profile bytes loaded, current raw configuration sequence and `reverse_ready`. Readiness reuses the safety policy's profile validation, actual attachment count and attachment-change latch; it is false without fresh telemetry or after shutdown. It does not grant the normal Gate or override obstacle AEB. A live experiment host must verify both mappings continuously.

This safety profile does not certify road boundaries or company target semantics and does not enable live parking by itself. Runtime's default constructor refuses live parking. Its separate development-experiment entry requires host-verified game recording evidence and the exact running independent profile; experiment authorization is not stage acceptance.

```powershell
cargo test -p etsads-safety --lib
```

Tests cover forward watchdog behavior, default reverse prohibition, verified reverse limits, attachment/load changes, signed forward hitch geometry, rearward approaching vehicles, front-corner swing, verified cross-layer separation, tall and tilted obstacles, and a vehicle crossing between prediction samples.
