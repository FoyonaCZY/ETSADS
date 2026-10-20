//! Packed ETS2LA 2026 protocol. Copy packed values before accessing fields.
use bytemuck::{Pod, Zeroable};
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Camera {
    pub fov: f32,
    pub position: [f32; 3],
    pub sector: [i16; 2],
    pub quaternion_wxyz: [f32; 4],
    pub projection: [f32; 16],
    pub truck_position: [f32; 3],
    pub truck_quaternion_wxyz: [f32; 4],
}
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Trailer {
    pub position: [f32; 3],
    pub quaternion_wxyz: [f32; 4],
    pub size: [f32; 3],
}
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Traffic {
    pub position: [f32; 3],
    pub quaternion_wxyz: [f32; 4],
    pub size: [f32; 3],
    pub speed: f32,
    pub acceleration: f32,
    pub trailer_count: i16,
    pub id: i16,
    pub is_tmp: u8,
    pub is_trailer: u8,
    pub trailers: [Trailer; 3],
}
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Parked {
    pub position: [f32; 3],
    pub quaternion_wxyz: [f32; 4],
    pub size: [f32; 3],
    pub id: i16,
    pub is_trailer: u8,
}
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Signal {
    pub position: [f32; 3],
    pub sector: [i16; 2],
    pub quaternion_wxyz: [f32; 4],
    pub kind: i32,
    pub time_remaining: f32,
    pub state: i32,
    pub id: i32,
}
#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct RouteNode {
    pub node_uid: i64,
    pub distance_to_end: f32,
    pub time_to_end: f32,
}
const _: () = assert!(std::mem::size_of::<Camera>() == 128);
const _: () = assert!(std::mem::size_of::<Traffic>() == 174);
const _: () = assert!(std::mem::size_of::<Parked>() == 43);
const _: () = assert!(std::mem::size_of::<Signal>() == 48);
const _: () = assert!(std::mem::size_of::<RouteNode>() == 16);
pub fn sector_world(position: [f32; 3], sector: [i16; 2]) -> [f64; 3] {
    [
        position[0] as f64 + sector[0] as f64 * 512.0,
        position[1] as f64,
        position[2] as f64 + sector[1] as f64 * 512.0,
    ]
}
// as_chunks::<{size_of::<T>()}> requires generic_const_exprs, unavailable on stable.
#[allow(clippy::chunks_exact_to_as_chunks)]
pub fn records<T: Pod>(bytes: &[u8]) -> impl Iterator<Item = T> + '_ {
    bytes
        .chunks_exact(std::mem::size_of::<T>())
        .map(bytemuck::pod_read_unaligned)
}
pub fn valid_quaternion(q: [f32; 4]) -> bool {
    let n = q.iter().map(|x| x * x).sum::<f32>();
    n.is_finite() && (n - 1.0).abs() < 0.05
}
pub fn yaw_scs_quaternion(q: [f32; 4]) -> Option<f64> {
    if !valid_quaternion(q) {
        return None;
    }
    let rotation = glam::Quat::from_xyzw(q[1], q[2], q[3], q[0]).normalize();
    let f = rotation * -glam::Vec3::Z;
    Some((-f.z as f64).atan2(f.x as f64))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn packed_parse_and_sector() {
        let node = RouteNode {
            node_uid: 123,
            distance_to_end: 3.0,
            time_to_end: 4.0,
        };
        let bytes = bytemuck::bytes_of(&node);
        let parsed = records::<RouteNode>(bytes).next().unwrap();
        let uid = parsed.node_uid;
        assert_eq!(uid, 123);
        assert_eq!(
            sector_world([1.0, 2.0, 3.0], [2, -1]),
            [1025.0, 2.0, -509.0]
        );
        assert!(
            (yaw_scs_quaternion([1.0, 0.0, 0.0, 0.0]).unwrap() - std::f64::consts::FRAC_PI_2).abs()
                < 1e-6
        );
    }
}
