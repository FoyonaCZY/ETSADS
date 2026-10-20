# Runtime map loading

`ExportedMap::from_path` reads exporter JSON; `load_parquet_region` reads a bounded region from normalized Parquet. Both represent the induced graph of loaded lanes: longitudinal links and lateral neighbors outside the region cannot become executable edges. The exporter keeps crossing rows separately as boundary provenance. Source lane strings and native owner UIDs stay lossless; runtime integer lane IDs are deterministic local indices, not native IDs.

`left` and `right` are optional loaded source-lane IDs. `can_change_left/right` default to false and require an explicit `*_change_allowed_raw: true`. Missing or null native permission never grants a lane change. A boundary neighbor loses its runtime reference and effective permission while retaining its raw source permission for auditing. Duplicate Parquet declarations for the same lane side, self-neighbors, dangling references and inconsistent permissions are rejected. Loading native permission does not waive calibration, physical-boundary or live-experiment requirements.

JSON uses exact float round-trip parsing so exported double-precision centreline coordinates match Parquet values. Strict map acceptance continues to reject missing widths, speed limits, unverified geometry and unaccepted exports. Explicit commissioning region policy remains a separate, bounded capability.

Portable checks:

```powershell
cargo test -p etsads-map
```

For installed-game cross-format regression, generate a fresh exporter slice, then set `ETSADS_PARQUET_EXPORT` to its source Parquet directory and `ETSADS_REGION_SLICE` to the slice directory:

```powershell
cargo test -p etsads-map --test real_region_equivalence -- --ignored --nocapture
```

The explicit fixture must contain real internal or boundary neighbor rows and crossing longitudinal links. The test compares every point, source ID, model field, adjacency, effective/raw permission and audit result. It also checks that the source remains unaccepted for autonomous driving. Matching formats is data-contract verification, not a road-driving acceptance result.
