# Driving runtime

`Runtime::new(config, calibration, region)` validates mission permissions and vehicle calibration. `Runtime::step(&GameSnapshot)` returns a serializable `RuntimeOutput`; it never opens the supervisor gate or writes game input memory.

`control_ego` and all output trajectories use the tractor rear axle as their reference. The raw SDK state is transformed using the calibration's measured wheel-centre offset and the official full pitch/roll/heading rotation. The original recording remains unchanged. Live operation checks physical configuration fingerprints and supports at most one attached trailer.

Tractor-only and single-trailer geometry are explicit models. A tractor-only model has no trailer envelope or trailer dynamics. The calibration binds its measured trailer count and cargo-mass applicability; picking up cargo or changing the attachment cannot silently reuse an incompatible model.

Every live Runtime mode requires an explicit measured exterior envelope and matching `EnvelopeMeasurement` evidence. Tractor front/rear offsets use the tractor rear axle; trailer front uses the hitch and trailer rear uses its equivalent axle. The planner, continuous sweeps, Hybrid A* and iLQR all use `VehicleGeometry.resolved_envelope()` through shared footprints. Kinematics retain the same axle/hitch references, so changing body overhangs cannot move the control reference. Independent AEB still reads its own measured profile; `validate_planner_envelope` ensures the planner contains each independently measured front/rear extent.

Older geometry/calibration JSON remains readable: absent envelope fields become `None`. Replay preserves the previous offsets and explicitly reports a synthetic body-offset fallback. Such a model is refused by every live Runtime constructor with the missing measurement identified. Wire telemetry layouts and existing MCAP payloads are unchanged.

Live planning uses a named worker and a capacity-one request queue. Control runs on physical telemetry frames; planning requests are submitted every 100 simulation milliseconds. The first trajectory is awaited with released inputs, and any trajectory older than 200 ms faults. Replay computes planning synchronously for deterministic regression. Events are buffered between planning ticks so a toll-payment or delivery event is not lost.

Forward control is selected explicitly by `RuntimeConfig.forward_lateral`. Omission preserves `{"kind":"pure_pursuit"}`. The MPC setting is `{"kind":"ltv_mpc","lateral_acceleration_limit_mps2":1.2,"maximum_solve_time_ms":20.0}`; choose the deadline for the execution environment and inspect measured solve times. MPC uses a 2 s horizon at 0.1 s spacing, advances the first steering-rate command by the measured physical interval, compensates the measured delay and applies the identified semantic-axis inverse. Reverse control retains articulated LQR.

Each output's `lateral_control` records the actual controller, QP attempt/convergence, solve duration, requested wheel angle and any failure. A hold or reverse-LQR frame is never counted as executed MPC. Nonconvergence, infeasible constraints or an exceeded configured deadline stop the runtime without silently selecting pure pursuit. The preceding control sequence is shifted by the physical interval for the next LTV linearisation. Clarabel 0.11.1 computes its own default primal/dual start, so diagnostics explicitly distinguish this linearisation reuse from a solver warm start.

| Mode | Implemented behavior | Current live gate |
| --- | --- | --- |
| `CommissioningCircle` | Explicit fixed circle inside a verified clear area | Measured calibration, reference transform, current truck fingerprint |
| `LaneFollow` | Verified road-region matching, route following, ACC | Explicit region policy, fresh privileged traffic and telemetry limit; junctions excluded |
| `ControlledRoute` | Signal association, stop/yield conflicts, green clearance, toll interaction, legal gap-checked lane changes, merge/roundabout annotations, full truck sweep and feasible city iLQR refinement | Default constructor refuses live mode; explicit experiment entry requires host-verified recording evidence, physical boundaries and manoeuvre semantics |
| `Parking` | Hybrid A*, articulated feedback, stop-before-direction-change, measured gear acknowledgement, pose/delivery completion | Default constructor refuses live mode; explicit experiment entry additionally requires a running independent rearward AEB with the exact verified safety profile and game target-reference evidence |

Unknown company-slot reference semantics are rejected. A parking target must explicitly represent the tractor rear axle for a tractor-only model, the trailer axle for a trailer model, or provide a measured trailer-reference-to-axle transform. Display polygons are not accepted as physical boundaries.

Live parking also requires fresh, same-frame privileged traffic and parked-object channels on every control frame. The planner decodes parked vehicles and moving vehicle/trailer bodies from raw records, rejects malformed nonempty records, projects tilted 3D boxes conservatively, and checks the upcoming path continuously against their occupancy. This remains separate from the independent supervisor's AEB parser and collision checks. Replay without extension data retains its explicit synthetic Scene fixtures.

`ParkingMission.recovery` bounds articulation-error recovery. Its defaults allow two attempts, 15 m total travel, 45 s, 0.6 m/s forward speed and a 5° articulation target; zero allowed attempts disables recovery. A tracking articulation error above 15° first requests braking. After measured standstill (the existing 0.05 m/s threshold), the runtime searches a collision-free forward pull-up, waits for measured forward gear, follows the verified swept path, stops again, replans to the original target and confirms the next gear before resuming. Occupancy changes, gear-acknowledgement failure, exhausted budgets or no feasible path stop the attempt; moving failures keep braking until the standstill threshold before latching Fault. `intent.parking_recovery` records phase, attempt, travelled distance and failure reason.

Hosts must inspect `request_control`, `status`, `actuation`, and `intent`. `WaitingForSecondFrame` and `WaitingForTrajectory` mean polling should continue with released inputs. `Paused` releases inputs. `Fault` latches until explicit reset/rearm. `Completed` ends the mission. `intent.gear_request` is emitted only after stopping and must be acknowledged by measured gear before movement; interaction requests are not implicit authorization to bypass the supervisor.

`Runtime::new_live_experiment` consumes a non-deserializable `VerifiedLiveExperiment`. The host first verifies source game recordings, frame ranges, review artifacts and all file hashes, then calls `from_host_verification(config, calibration, region, kind, evidence_sha256, safety_profile_sha256)`. The capability binds the exact runtime configuration, calibration, sorted lane graph, source IDs and region policy; changing any of them invalidates it. The runtime does not independently perform the host's filesystem or supervisor IPC checks. The host must keep checking supervisor identity, profile SHA-256, heartbeat, current configuration and normal Gate permissions on every live frame. This is an explicit supervised development entry, not a stage acceptance certificate. No experiment evidence is fabricated by the library.

`following` records the same selected leader and bumper gap used by IDM on the current physical frame. A real, matching front-bumper measurement is required; without it, `following` is null and `following_unavailable_reason` explains the missing evidence. Stationary/reverse motion, opposite traffic and route-end stop constraints do not produce following samples. `quality` is `verified_geometry` for eligible live observations or `synthetic_replay` in replay. `headway_seconds` is the measured `bumper_gap_m / ego_speed_mps`, never the controller's desired headway.

`RuntimeConfig.route_source` defaults to `{"kind":"explicit"}`, preserving existing numeric route configurations. Native navigation uses the current `GameSnapshot.scene.route` and requires the same physical frame's route-extension channel. It validates every GPS UID and directed checkpoint pair against the compact, authenticated global topology, then resolves the exact executable local lane prefix. Unknown nodes, disconnected GPS checkpoints, absent native vehicle/lateral permissions, incomplete traffic annotations or unmeasured physical boundaries stop execution. Missing GPS data never triggers an unconstrained A* fallback.

```json
"route_source": {
  "kind": "native_gps",
  "expected_source_id": "<lowercase SHA-256 of sector_index.json>",
  "corridor_policy": {
    "minimum_lane_change_distance_m": 40.0,
    "exit_buffer_m": 15.0,
    "maximum_lane_changes": 4,
    "maximum_search_states": 50000,
    "require_validated_centerline": true,
    "lane_change_penalty_m": 20.0
  },
  "tolls": [],
  "roundabout_exits": [],
  "road_evidence": null
}
```

With native navigation, leave the mode's numeric `route` empty; the current map binds stable source lane strings to local runtime IDs. `ControlledRoute` retains its verified boundary and control-evidence template, while route, traffic annotations and required lane changes are derived from the current corridor. Native gate candidates become missions only on the selected corridor's actual entry-to-exit lane intervals. Their native semaphore group and resolved profile type choose the interaction; names are never used to infer a gate type. Unknown, contradictory or unsupported service-gate semantics stop execution. Lane-follow mode stops when the corridor needs junction, gate or lateral handling. Allowed successor channels are restricted to the GPS corridor. A necessary lane change has separate latest-start and completion deadlines; an occupied adjacent lane causes braking before the start deadline.

`route_source.tolls` remains an optional reviewed override using `{id, source_lane_id, barrier_position, stop_s}`. `stop_s` denotes the barrier plane's lane station; the state machine targets a stop 3 m before it. An override must agree with the actual source projection and adds the requirement for a real `player.tollgate.paid` event. Generic native barriers do not establish a toll classification: physical passage and observed payment are recorded separately. A completed unclassified gate interaction is never a paid-toll acceptance result.

Manual gates require measured standstill, a one-second dwell, a fresh CLOSED observation and authenticated activation semantics. They issue one pulse, allow at most one retry after 20 seconds, and stop on the bounded timeout. Automatic and distance-activated gates never issue activate. All types need a same-frame, unambiguous OPEN state before passing; missing or contradictory observations hold. Passage completes only when all measured tractor and trailer corners clear the gate plane. Static profile intervals are not runtime countdowns and are not inputs to this state machine. Numeric lane-ID changes retain the source-bound interaction state and attempt count.

`intent.gate` exposes the source lane, native group, profile proof IDs, mode, phase, observation, activation attempts, physical clearance and payment/passage flags. Native `ControlledRoute` live capability creation requires real activate evidence even when the startup override list is empty, since later map windows may contain manual gates. Before publishing a requested pulse, the CLI rechecks CLOSED on the current physical frame and enforces the experiment, supervisor, standstill and brake conditions. Candidate derivation cannot grant any of these permissions.

`drive` uses a capacity-one background map session with 2 km windows, a bounded sector cache, speculative prefetch and replacement requests after 600 m movement. Global policy authorization is intersected with each window; it cannot silently gain new source lanes. Each completed window retains its source/annotation hashes, read statistics, generation and reason in `RuntimeOutput.navigation`, which mandatory `/stack` recording preserves. Changing regional numeric IDs does not change vehicle route identity. Normal GPS prefix consumption is checked against observed checkpoint progress. A changed destination, skipped checkpoint or replacement that changes the committed branch faults and requires explicit replanning/rearm. A known but unloaded remote suffix remains an explicit stopping boundary. Neither a long GPS list nor a successful local prefix counts as completing a long-distance drive.

Native road operation is separate from commissioning. `road_evidence: null` retains the existing 30 km/h policy limit. Setting `road_evidence` to the three explicit `centerline_evidence_path`, `physical_boundary_evidence_path`, and `vehicle_access_evidence_path` references selects `RoadRegionPolicy`. This requires measured widths, validated centerlines and physical boundaries, native truck access and fresh navigation limits; nominal navigation width is refused. The chosen speed cannot exceed the dynamic calibration's `dynamic_speed_range_mps` or its identified steering rows. Identification derives that range from observed longitudinal samples intersected with the steering-table support; old or static calibration without it cannot authorize road operation. A vehicle leaving that observed speed range stops rather than extrapolating an unmeasured model.

Road mode additionally requires `route_source.road_boundary_path`, resolved relative to the runtime configuration file. That file is a `RoadBoundary` with `schema_version: 1`, `coordinate_system: "ENU_metres"`, the exact `map_source_id`, `verified_source_lane_ids` equal to the original complete policy whitelist, and a `boundary` using the existing `VerifiedBoundary` structure. Its `boundary.evidence_path` must exactly equal the configured physical-boundary evidence reference; its polygon vertices must come from the measured drivable area. Display polygons and nominal lane widths are never converted into this file. The complete polygon and evidence scope remain unchanged across 2 km map windows, and the host's road document must hash-bind the actual boundary file.

Both the proposed trajectory and successive observed physical states check tractor and trailer containment with the existing conservative polygon sweep. A body centre rotates about its measured rear axle/hitch; the road check additionally bounds the difference from linear body-centre interpolation by `radius × angle² / 8`. This catches articulated corner excursions even when both endpoint boxes fit. An unsafe proposed sweep faults before it publishes an actuation request. This is withdrawal of automatic control before executing an invalid path, not a claim that releasing a moving vehicle instantly stops its physical motion. The startup `/road_boundary` record preserves the exact measured polygon used by the runtime.

Live road operation also requires `drive --road-evidence <document.json>`. The host verifies the engineering measurement files and their complete source game recordings against the current configuration, calibration, map and policy hashes. It then creates a non-deserializable `VerifiedRoadOperation` and calls `Runtime::new_live_road`. Advanced manoeuvres still need their independent `VerifiedLiveExperiment`; road evidence does not enable them implicitly. Default live constructors refuse road operation without this host verification. Region replacement must match the original road evidence and authorization scope. These engineering checks do not declare a P1 100 km acceptance pass.

The three `RoadEvidence` paths each refer to an `EvidenceReference` JSON. `RoadEvidenceDocument.evidence` binds them as `road_centerline`, `physical_boundary`, and `road_vehicle_access`, including their file hashes and supporting files. Their engineering reviews carry a measurement method, the exact lane scope, maximum centerline error, minimum measured vehicle-envelope clearance, and a concrete vehicle-access observation. Real recording identity, frame coverage, complete MCAP/video hashes and same-frame camera association are checked by the host. The code generates no successful measurement review.

`replay-drive` uses the same native map session with `poll_replay`: offline observation time waits for a requested disk load, so fast replay cannot outrun background disk wall time. The entry still requires `execution=Replay`, writes no input IPC, records its exact configuration/calibration and reports `phase_acceptance=false`.

The current executor rejects a local corridor that revisits the same source lane. A repeated lane needs occurrence-aware state; collapsing `A→B→A→C` into one successor entry could otherwise take the second exit on the first visit. Such a GPS loop faults explicitly instead of driving the wrong branch.

Run the shared offline regression path:

```powershell
cargo test -p etsads-runtime --lib
cargo run -p etsads-sim --example regression_suite --release -- target/core-suite.json
cargo run -p etsads-sim --example runtime_control -- artifacts/runtime-mpc-evidence
cargo run -p etsads-sim --example runtime_recovery -- artifacts/runtime-recovery-evidence
cargo run -p etsads-sim --example runtime_navigation -- artifacts/runtime-navigation-evidence.json
```

The two Runtime examples write parsed Replay configurations, reference trajectories, per-frame actual model states, controls and execution diagnostics. The MPC case includes a lateral disturbance and actuator lag. Recovery cases include successful resumed docking, insufficient space, missing forward-gear acknowledgement and exhausted attempts. These fixtures send no game inputs. Compare debug and optimized solver P50/P95/maximum timings with the original 1 ms goal separately from geometric closed-loop success; a configured deadline is not a replacement for that performance goal.

The suite includes 50 varied obstacle-docking closed loops plus the actual runtime's red/green, toll/payment, lane-change and gear-acknowledged parking scenarios. These are synthetic SDK-message and vehicle-model tests. They do not establish a live 100 km drive, game fine rate, or 50 difficult ETS2 deliveries. Actual map projection, real vehicle calibration and game acceptance remain separate evidence requirements.

The navigation example records actual Runtime trajectories/states for required native-GPS lane changes and stopping before an unavailable gap, a source lane rebound from numeric ID 2 to 1 across windows, unknown-UID/reroute failures, and road operation above 30 km/h within a synthetic dynamic range. The disk map under `crates/map/testdata/navigation-sectors` is explicitly synthetic. It provides no game measurement or live-operation evidence.
