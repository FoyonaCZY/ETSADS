//! ETSADS-owned capture publisher and DX11 hook. Build the actual plugin with
//! `--features native-dx11`; the default build has no native hooking dependency.
#[cfg(all(windows, feature = "native-dx11"))]
mod hud;
#[cfg(all(windows, feature = "native-dx11"))]
mod native;
pub mod publisher;

/// Validate a renderer primitive without touching Direct3D state.
pub fn validated_rect(
    rect: &etsads_msgs::protocol::OverlayRect,
    width: u32,
    height: u32,
) -> Option<([i32; 4], [f32; 4])> {
    if !rect
        .bounds
        .iter()
        .chain(rect.color_rgba.iter())
        .all(|x| x.is_finite())
    {
        return None;
    }
    let [l, t, r, b] = rect.bounds;
    if l >= r || t >= b || width == 0 || height == 0 {
        return None;
    }
    let bounds = [
        l.max(0.0).min(width as f32) as i32,
        t.max(0.0).min(height as f32) as i32,
        r.max(0.0).min(width as f32) as i32,
        b.max(0.0).min(height as f32) as i32,
    ];
    if bounds[0] >= bounds[2] || bounds[1] >= bounds[3] {
        return None;
    }
    Some((bounds, rect.color_rgba.map(|x| x.clamp(0.0, 1.0))))
}
#[cfg(test)]
mod tests {
    use super::*;
    use etsads_msgs::protocol::OverlayRect;
    #[test]
    fn validates_overlay_geometry() {
        let mut r = OverlayRect {
            bounds: [-2.0, 1.0, 12.0, 20.0],
            color_rgba: [0.5, 1.2, 0.0, 1.0],
        };
        assert_eq!(
            validated_rect(&r, 10, 10),
            Some(([0, 1, 10, 10], [0.5, 1.0, 0.0, 1.0]))
        );
        r.bounds[0] = f32::NAN;
        assert!(validated_rect(&r, 10, 10).is_none());
    }
}
