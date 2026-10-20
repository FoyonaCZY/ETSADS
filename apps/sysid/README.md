# Vehicle identification

`etsads-sysid` analyzes a completed recording. It never sends game inputs.

For a stationary, zero-throttle steering sweep, use the separate static workflow:

```powershell
cargo run -p etsads-sysid -- static-plan --vehicle-id YOUR_TRUCK --steering-axes=-0.2,-0.1,0,0.1,0.2 --hold-seconds 3 --output artifacts/static-plan.json
cargo run -p etsads-sysid -- static-steering --run recordings/YOUR_STATIC_RUN --vehicle-id YOUR_TRUCK --settle-seconds 0.75 --output artifacts/static-steering.json
```

The plan contains explicit steering levels and zero throttle; it is a reviewable document, not an input executor. The analyzer selects the recorded configuration's steerable wheels, measures each wheel's angle and repeatability after settling, and learns semantic-axis polarity. It rejects moving, paused, nonzero-throttle and invalid observations. Static reports always contain `driving_authorized: false` and identify the missing speed-dependent steering, delay and longitudinal experiments. They cannot be loaded as a complete vehicle calibration.

Both static and dynamic analysis align semantic controls by QPC causality: only the newest active control applied at or before the telemetry measurement is eligible, with a maximum age of 200 ms. A later applied snapshot recorded beside an older telemetry frame is not treated as that frame's input. An inactive gate ends the active interval. Semantic steering and telemetry input axes need not share a sign; the observed wheel response determines the fitted polarity.

For dynamic identification:

```powershell
cargo run -p etsads-sysid -- plan --vehicle-id YOUR_TRUCK --speeds-kmh 20,40,60,80 --steering-axes=-0.2,-0.1,0,0.1,0.2 --hold-seconds 3 --output artifacts/experiment-plan.json
cargo run -p etsads-sysid -- fit --run recordings/YOUR_RUN --vehicle-id YOUR_TRUCK --geometry configs/vehicles/measured-geometry.json --steering-source applied --rear-wheel-indices 2,3 --reference-evidence artifacts/rear-axle-measurement.json --output configs/vehicles/calibration.json
```

The wheel indices in this example must be replaced with the selected truck's actual, non-steering rear wheels. The evidence file must exist. Geometry is a serialized `etsads_msgs::VehicleGeometry`; dimensions and equivalent axle lengths must be measured for the selected truck/trailer configuration. `hitch_offset` is signed: positive behind the tractor rear axle, negative ahead of it.

`has_trailer` is mandatory. For a tractor-only configuration, set it to `false` and set both trailer length and trailer wheelbase to zero. The recording also establishes `CalibrationConditions` (actual attached-trailer count and cargo-mass range). Changing attachment or load during identification makes the result unaccepted; runtime checks the conditions again on every frame.

Required recording topics are `/telemetry`, `/config`, and active `/applied` semantic SDK controls. `input` and `effective` fits can be inspected, but do not authorize semantic-device control. Paused frames, nonmoving samples and unsynchronized applied controls are excluded. Incomplete recordings, time regressions, changing truck geometry and insufficient excitation are rejected or produce an unaccepted calibration.

The output includes steady steering samples at four speed ranges, measured axis polarity, ARX delay/first-order lag, longitudinal regression and residuals. There are no default axis signs or fallback coefficients. A physical configuration SHA-256 covers wheel positions, radii, steerability and hitch position; job fields and sequence counters are excluded.

The rear-axle reference is computed from explicitly selected wheel positions. The SDK only defines `truck.world.placement` as vehicle placement; it does not declare it to be the rear axle. The conversion follows the official SDK `telemetry_position.cpp`: roll about Z, pitch about X, heading about Y, then translate. `reference_from_config` and `validate_reference_geometry` also check the selected axle centre, wheelbase and signed hitch offset.

Changing a configuration file's acceptance flag is not calibration. Runtime additionally checks source provenance, semantic-axis evidence, the current `/config` fingerprint and the measured reference transform. Synthetic recordings remain offline evidence. Independent throttle, brake, coast and grade excitation is required for a useful longitudinal fit; stationary telemetry cannot produce a driving calibration.

Outputs use JSON, or TOML when the output filename ends in `.toml`. Existing output files are preserved; select a new filename for each fit.

Actual following-headway acceptance requires an exterior front-bumper measurement. Optional `fit --front-bumper-measurement FILE.json` imports an existing `FrontBumperMeasurement` containing `front_from_rear_axle_m`, `geometry_fingerprint`, and `evidence_path`. It remains readable for older recordings; a front-bumper measurement alone is insufficient for current live control.

All live Runtime modes now require `VehicleGeometry.measured_envelope`. It contains `tractor_front_from_rear_axle_m`, `tractor_rear_overhang_m`, and `trailer` (null for tractor-only, otherwise `front_from_hitch_m` and `rear_from_axle_m`). The tractor offsets must sum to its measured total length; trailer front + equivalent axle distance + rear must sum to trailer length. Equal total lengths cannot substitute for the actual front/rear distribution.

Use `fit --envelope-measurement FILE.json` to import an existing `EnvelopeMeasurement` with `geometry_fingerprint`, matching `envelope` values, and `evidence_path`. Relative evidence paths resolve beside that measurement file and must exist. The stored measurement values must equal the geometry file, and its fingerprint must match the current recording. Missing or inconsistent measurements leave the fit unaccepted with explicit issues. Wheel positions, hitch position and stationary steering never generate exterior measurements. Advanced live experiments additionally hash-bind this evidence through the host's reviewed supporting files.
