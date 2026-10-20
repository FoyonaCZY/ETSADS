//! Real recorded inputs and approximate, visually reviewed image regions.
//! Expected regions are not copied from the projection algorithm's output.
use etsads_game::{
    camera::{bbox_corners_scs, project, project_bbox, Convention},
    ets2la::Camera,
};
use serde_json::Value;

type Rect = [f64; 4];
// One rule for all examples. The 12 px annotation uncertainty is stored with
// each original manual region; the extra 10% permits geometric-box vs visible
// mesh/silhouette offsets. Neither budget claims pixel-accurate calibration.
const MESH_CENTER_ALLOWANCE_FRACTION: f64 = 0.10;
const MIN_UNCERTAINTY_REGION_IOU: f64 = 0.50;
const MIN_IOU_ADVANTAGE: f64 = 0.25;

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/ets2la-1.61.1.9-exterior-projection.json"
    ))
    .expect("persistent real-recording fixture is valid JSON")
}
fn array<const N: usize>(value: &Value) -> [f64; N] {
    let values = value.as_array().expect("numeric array");
    assert_eq!(values.len(), N);
    std::array::from_fn(|i| {
        let n = values[i].as_f64().expect("finite number");
        assert!(n.is_finite());
        n
    })
}
fn camera(case: &Value) -> Camera {
    let c = &case["raw_camera"];
    Camera {
        fov: c["fov"].as_f64().unwrap() as f32,
        position: array::<3>(&c["position"]).map(|v| v as f32),
        sector: std::array::from_fn(|i| c["sector"][i].as_i64().unwrap().try_into().unwrap()),
        quaternion_wxyz: array::<4>(&c["quaternion_wxyz"]).map(|v| v as f32),
        projection: array::<16>(&c["projection"]).map(|v| v as f32),
        truck_position: array::<3>(&c["truck_position"]).map(|v| v as f32),
        truck_quaternion_wxyz: array::<4>(&c["truck_quaternion_wxyz"]).map(|v| v as f32),
    }
}
fn expanded(rect: Rect, uncertainty: f64) -> Rect {
    [
        rect[0] - uncertainty,
        rect[1] - uncertainty,
        rect[2] + uncertainty,
        rect[3] + uncertainty,
    ]
}
fn area(r: Rect) -> f64 {
    (r[2] - r[0]).max(0.0) * (r[3] - r[1]).max(0.0)
}
fn iou(a: Rect, b: Rect) -> f64 {
    let intersection = area([
        a[0].max(b[0]),
        a[1].max(b[1]),
        a[2].min(b[2]),
        a[3].min(b[3]),
    ]);
    intersection / (area(a) + area(b) - intersection)
}
fn contains(r: Rect, p: [f32; 2]) -> bool {
    (r[0]..=r[2]).contains(&(p[0] as f64)) && (r[1]..=r[3]).contains(&(p[1] as f64))
}
fn projected_group(case: &Value, convention: Convention) -> Option<(Rect, Vec<[f32; 2]>)> {
    let camera = camera(case);
    let width: u32 = case["dimensions_px"][0]
        .as_u64()
        .unwrap()
        .try_into()
        .unwrap();
    let height: u32 = case["dimensions_px"][1]
        .as_u64()
        .unwrap()
        .try_into()
        .unwrap();
    let mut union: Option<Rect> = None;
    let mut centers = Vec::new();
    for native in case["native_boxes"].as_array().unwrap() {
        let center = array(&native["center_world_scs"]);
        let corners = bbox_corners_scs(
            center,
            array::<3>(&native["size_xyz_m"]).map(|v| v as f32),
            array::<4>(&native["quaternion_wxyz"]).map(|v| v as f32),
        )
        .expect("raw reviewed native box is valid");
        let bounds = project_bbox(camera, corners, width, height, convention)?.map(f64::from);
        centers.push(project(camera, center, width, height, convention)?);
        union = Some(union.map_or(bounds, |r| {
            [
                r[0].min(bounds[0]),
                r[1].min(bounds[1]),
                r[2].max(bounds[2]),
                r[3].max(bounds[3]),
            ]
        }));
    }
    Some((union?, centers))
}

#[test]
fn fixed_exterior_convention_matches_reviewed_pixels_better_than_all_seven_alternatives() {
    let fixture = fixture();
    let candidates = Convention::candidates();
    assert_eq!(Convention::ETS2LA_1_61_REVIEWED_EXTERIOR, candidates[1]);
    for case in fixture["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let reviewed: Rect = array(&case["review"]["region_px"]);
        let uncertainty = case["review"]["region_uncertainty_px"].as_f64().unwrap();
        let expected = expanded(reviewed, uncertainty);
        let (projected, centers) = projected_group(case, Convention::ETS2LA_1_61_REVIEWED_EXTERIOR)
            .expect("all reviewed traffic bodies must project under C1");
        let score = iou(projected, expected);
        assert!(
            score >= MIN_UNCERTAINTY_REGION_IOU,
            "{name}: C1 overlap {score:.3} does not cover the observed region"
        );
        for axis in 0..2 {
            let center_error = ((projected[axis] + projected[axis + 2])
                - (reviewed[axis] + reviewed[axis + 2]))
                .abs()
                / 2.0;
            let budget = uncertainty
                + MESH_CENTER_ALLOWANCE_FRACTION * (reviewed[axis + 2] - reviewed[axis]);
            assert!(center_error <= budget, "{name}: axis {axis} center error {center_error:.2} exceeds annotation/mesh allowance {budget:.2}");
        }
        for center in centers {
            assert!(
                contains(expected, center),
                "{name}: native body center outside the reviewed image region"
            );
        }
        for (id, convention) in candidates
            .into_iter()
            .enumerate()
            .filter(|(id, _)| *id != 1)
        {
            let other = projected_group(case, convention).map_or(0.0, |(r, _)| iou(r, expected));
            assert!(
                score >= other + MIN_IOU_ADVANTAGE,
                "{name}: C1 IoU {score:.3} is not clearly better than C{id} ({other:.3})"
            );
        }
    }
}

#[test]
fn recording_identity_alignment_and_review_limits_remain_explicit() {
    let data = fixture();
    assert_eq!(data["schema_version"], 1);
    assert_eq!(data["game_patch"], "1.61.1.9");
    assert_eq!(data["source_recording_dropped_frames"], 1);
    assert_eq!(data["fully_calibrated"], false);
    assert_eq!(data["phase_acceptance"], false);
    assert_eq!(data["cases"].as_array().unwrap().len(), 3);
    let identities = [(10100, 24067, 1), (10253, 24220, 2), (10403, 24370, 1)];
    for (case, (video, physical, bodies)) in
        data["cases"].as_array().unwrap().iter().zip(identities)
    {
        assert_eq!(case["video_frame"], video);
        assert_eq!(case["physical_frame"], physical);
        assert_eq!(case["native_boxes"].as_array().unwrap().len(), bodies);
        assert_eq!(array::<2>(&case["dimensions_px"]), [2560.0, 1440.0]);
        assert_eq!(case["review"]["region_uncertainty_px"], 12);
        let hash = case["image_sha256"].as_str().unwrap();
        assert!(hash.len() == 64 && hash.bytes().all(|c| c.is_ascii_hexdigit()));
        let frequency = case["qpc_frequency"].as_u64().unwrap();
        assert_eq!(frequency, 10_000_000);
        let skew = case["capture_qpc"]
            .as_u64()
            .unwrap()
            .abs_diff(case["physical_qpc"].as_u64().unwrap());
        assert!(skew as f64 * 1000.0 / frequency as f64 <= 16.0);
        // Fail if an algorithm-derived pixel output was accidentally copied in
        // as the expected result instead of preserving manual image evidence.
        assert!(case.get("hypotheses").is_none());
        assert!(case.get("projected_union_bbox").is_none());
    }
}
