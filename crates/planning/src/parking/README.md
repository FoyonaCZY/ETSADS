# Articulated Hybrid A* parking

The production entry remains `parking::plan(ParkingRequest)`. Runtime and simulation therefore use the new search without a separate demo-only switch. The target is the trailer axle for a coupled vehicle and the tractor rear axle for a tractor-only vehicle.

Long-running hosts can call `plan_cancellable(request, &AtomicBool)`. Cancellation is checked during grid construction/Dijkstra, Hybrid expansion, RS replay/correction and terminal connectors, returning `Invalid("parking search cancelled")`. Use a fresh, one-way cancellation token for each generation; do not clear a token still held by an old search. An uncancelled call produces the same plan as `plan`. The host still discards late results from older generations and must not join a long worker from its control/Drop path.

Search guidance is `max(Reeds–Shepp relaxation, obstacle-aware 2D Dijkstra potential)`. The RS implementation evaluates CSC, CCC, CCCC, CCSC and CCSCC families with reflection, time reversal and backward transformations. Its obstacle-free distance is checked against 128 independently compiled OMPL C++ reference cases, as well as endpoint, symmetry, scaling, forward/reverse, cusp and known lower-bound cases. The articulated guidance combines tractor and trailer axle relaxations; it does not claim to solve the full joint-state optimum analytically.

The Dijkstra map uses 0.5 m cells, increasing that resolution only when needed to remain within 250,000 cells. Obstacles include the tractor rear-axle inscribed-circle clearance and the existing 0.3 m planning margin. Diagonal transitions cannot cut occupied corners. An unreachable grid query is recorded as unavailable and weakens only the heuristic; it never deletes an obstacle or grants permission. All accepted motion still uses the actual articulated geometry and continuous collision checks.

Forward and reverse primitives use five steering choices and 0.2 m collision substeps. A primitive is 2 m outside a 12 m tractor-goal neighbourhood and 1 m inside it. Every twentieth expansion tries RS analytic connections. Candidate curves are replayed through the actual signed-hitch RK4 model at bounded steering rate, including trailer-angle limits. Near an aligned trailer target, a trailer-reference RS seed can also be replayed using the off-axle articulation relation. A bounded least-squares adjustment changes only segment lengths, preserves each segment's direction, and accepts only a fully validated terminal trailer pose. To bound compute, length correction is limited to eight iterations on the first two candidates per radius every 200 expansions. Uncorrected RS candidates are still checked at the ordinary 20-expansion interval.

Straight and reverse-feedback terminal connectors remain additional options. They obey the same 0.3 rad/s steering-rate limit as search and RS rollout. `ParkingPlan.search` identifies the connector actually accepted, whether it is curved, its reference axle, length correction and cusp count, candidate attempts, and initial RS/Dijkstra distances. A feedback result is never reported as an RS result. Mixed-direction RS paths preserve direction on every `ParkingPoint`; Runtime must still stop and obtain measured gear acknowledgement at each cusp.

The complete tractor/trailer envelope is checked for each accepted point and between points. The sweep accounts for body-centre rotation around the measured rear axle/hitch using the interpolation bound `radius × angle² / 8`, in addition to rigid-box sweep checks. Collision, boundary and articulation limits are never relaxed by a successful analytic endpoint solve.

The finite lattice, goal tolerances, steering-rate model, first feasible analytic connection and finite search budget mean this implementation makes no global-optimality guarantee. It can return `NoPath` at the requested budget even when a longer or more expensive solution exists. Seconds-long difficult searches also require a stopped, unarmed planning lifecycle with fresh-scene validation before execution; a faster planner-only result is not evidence of live readiness.

Reproduce the checks:

```powershell
cargo test -p etsads-planning parking
cargo run --release --target-dir target/core-runtime-validation -p etsads-sim --example parking_suite -- artifacts/parking-suite.json
cargo run --release --target-dir target/core-runtime-validation -p etsads-sim --example parking_difficult -- artifacts/parking-difficult.json
```

`parking_suite` retains the 50 deterministic offset-docking trials with parked trailers, world orientation changes and actuator disturbance. `parking_difficult` adds three distinct geometries: perpendicular bay, diagonal bay and a wall blocking the straight corridor. Failures remain in the reports. The current additional perpendicular and diagonal scenarios did not find a path within the configured 80,000-expansion budget; the wall-detour scenario produced a collision-free closed loop. These results do not establish the original P4 requirement of at least 50 difficult ETS2 parking attempts with at least 90% success.

Formula attribution: [OMPL ReedsSheppStateSpace](https://github.com/ompl/ompl/blob/main/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp). The BSD notice is retained in `vendor/licenses/OMPL-ReedsShepp.txt`, which the portable packager copies. Reference-generation details are in `crates/planning/testdata/README.md`.
