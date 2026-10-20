//! Read real resolved profile metadata and verify its provenance. No game IPC.
use etsads_map::{
    annotations::{AnnotationLimits, AnnotationStore},
    sector::{CacheLimits, SectorStore},
    traffic::{SemaphoreKind, SignalDecision, SignalHoldReason, SignalObservation},
};
use etsads_msgs::Point3;
use serde_json::Value;
use std::{error::Error, fs, path::Path};
fn main() -> Result<(), Box<dyn Error>> {
    let a: Vec<_> = std::env::args().collect();
    if a.len() != 5 {
        return Err("usage: profile_probe BUNDLE X Y REPORT.json".into());
    }
    let center = Point3 {
        x: a[2].parse()?,
        y: a[3].parse()?,
        z: 0.0,
    };
    let root = Path::new(&a[1]);
    let map = SectorStore::open(root, CacheLimits::default())?.load_2km(center)?;
    let annotations = AnnotationStore::open(root, AnnotationLimits::default())?
        .load_for_region(center, 2000.0, &map)?;
    let index: Value = serde_json::from_slice(&fs::read(root.join("traffic_index.json"))?)?;
    let proof: Value = serde_json::from_slice(&fs::read(
        root.join(
            index["signal_profile_resolution"]["evidence_path"]
                .as_str()
                .ok_or("profile proof missing")?,
        ),
    )?)?;
    let mut kinds = std::collections::BTreeMap::<String, usize>::new();
    let mut examples = Vec::new();
    let mut checked = 0;
    for candidate in &annotations.gate_candidates {
        assert!(candidate.semantics_resolved);
        assert!(!candidate.activation_verified);
        assert!(!candidate.toll_classification_verified);
        assert!(
            map.map
                .lanes
                .iter()
                .any(|l| l.id == candidate.source_lane_id)
        );
        let kind = serde_json::to_value(candidate.kind)?;
        *kinds.entry(kind.as_str().unwrap().into()).or_default() += 1;
        if let Some(id) = &candidate.profile_evidence_id {
            let evidence = &proof["resolutions"][id];
            assert!(!evidence.is_null());
            assert_eq!(evidence["resolved_type"], kind);
            checked += 1;
            if examples.len() < 6 && candidate.kind != SemaphoreKind::BarrierGas {
                examples.push(
                    serde_json::json!({"candidate":candidate,"definition_resolution":evidence}),
                );
            }
        }
    }
    let mut unknown_green_probes = 0;
    for signal in &annotations.traffic.signals {
        if matches!(
            signal.kind,
            SemaphoreKind::Unknown | SemaphoreKind::UseProfile
        ) {
            let green = [SignalObservation {
                position: signal.position(),
                raw_state: 8,
                age_us: 0,
            }];
            assert_eq!(
                annotations
                    .traffic
                    .signal_decision(&signal.id, &green, 1.0, 100),
                SignalDecision::Hold(SignalHoldReason::UnresolvedProfile)
            );
            unknown_green_probes += 1;
        }
    }
    let mut timing = 0;
    for value in proof["resolutions"].as_object().unwrap().values() {
        if value["timing_resolved"] == true {
            assert!(
                value["resolved_interval"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|v| v.as_f64().is_some_and(|f| f.is_finite() && f >= 0.0))
            );
            assert!(
                value["resolved_cycle_delay_s"]
                    .as_f64()
                    .is_some_and(|f| f.is_finite() && f >= 0.0)
            );
            timing += 1;
        }
    }
    assert!(
        checked > 0 || unknown_green_probes > 0,
        "real profile-linked gate or unresolved profile fixture required"
    );
    let report = serde_json::json!({"source":"installed-game static definitions and source map; no game actuation", "center":center,"radius_m":2000,
        "map_source_id":map.source_id,"annotations_source_id":annotations.source_id,"map_audit":map.map.audit(),"metadata_reads":annotations.stats,
        "gate_candidate_counts":kinds,"gate_provenance_checked":checked,"resolved_timing_records":timing,
        "unknown_profile_synthetic_green_hold_checks":unknown_green_probes,"examples":examples,"source_definition_files":proof["sources"],
        "native_country_inference":proof["country_summary"],"global_signal_resolution":index["signal_profile_resolution"],
        "static_timing_is_live_remaining_time":false,"activation_verified":false,"phase_acceptance":false});
    fs::write(&a[4], serde_json::to_vec_pretty(&report)?)?;
    println!(
        "profile probe: {} gates with checked provenance; {} timing records; {} unresolved hold checks",
        checked, timing, unknown_green_probes
    );
    Ok(())
}
