//! Attached trailers: how far the combination reaches behind the tractor's
//! hitch, from the game's trailer configuration (wheel and kingpin
//! positions), for gap acceptance, lane changes and tight turns.
use etsads_msgs::protocol::*;
use etsads_shm::SharedRing;

/// Behind the last trailer axle to the rear end (m).
const REAR_OVERHANG_M: f64 = 1.5;
/// A trailer whose configuration is not reported: a standard semitrailer
/// (13.6 m, kingpin about 1 m from its front).
const DEFAULT_BEHIND_KINGPIN_M: f64 = 12.6;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Combination {
    /// Connected trailers.
    pub trailers: usize,
    /// Distance from the tractor's hitch to the rear end of the last trailer
    /// (m); 0 without a trailer.
    pub behind_hitch_m: f64,
    /// Kingpin to the first trailer's axle group (m): the wheelbase that
    /// sets how far it cuts inside a turn.
    pub first_wheelbase_m: f64,
}

/// Kingpin to rearmost axle and to the rear end, in trailer coordinates
/// (SCS: z points backwards).
fn trailer_extent(config: &TrailerConfiguration) -> Option<(f64, f64)> {
    let count = (config.wheel_count as usize).min(16);
    let hook = f64::from(config.hook_position[2]);
    let axles: Vec<f64> = (0..count)
        .map(|i| f64::from(config.wheel_positions[i][2]) - hook)
        .filter(|z| z.is_finite())
        .collect();
    let rear = axles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let front = axles.iter().copied().fold(f64::INFINITY, f64::min);
    (rear.is_finite() && (2.0..=20.0).contains(&rear))
        .then_some(((rear + front) / 2.0, rear + REAR_OVERHANG_M))
}

pub fn combination(
    raw: &TelemetryFrame,
    configuration: Option<&ConfigurationFrame>,
) -> Combination {
    let mut out = Combination::default();
    for (index, trailer) in raw.trailers.iter().enumerate() {
        if trailer.connected == 0 {
            break;
        }
        let extent = configuration
            .filter(|c| c.present_trailer_mask & (1 << index) != 0)
            .and_then(|c| trailer_extent(&c.trailers[index]));
        let (wheelbase, behind) =
            extent.unwrap_or((DEFAULT_BEHIND_KINGPIN_M - 3.0, DEFAULT_BEHIND_KINGPIN_M));
        if out.trailers == 0 {
            out.first_wheelbase_m = wheelbase;
        }
        out.trailers += 1;
        out.behind_hitch_m += behind;
    }
    out
}

/// Reads the latest trailer configuration the bridge published.
#[derive(Default)]
pub struct TrailerReader {
    ring: Option<SharedRing<ConfigurationFrame, 64>>,
}
impl TrailerReader {
    pub fn read(&mut self, raw: &TelemetryFrame) -> Combination {
        if self.ring.is_none() {
            self.ring = SharedRing::open(CONFIG_FULL_MAPPING, SCHEMA_VERSION).ok();
        }
        let latest = self.ring.as_ref().and_then(|r| r.latest_boxed());
        combination(raw, latest.as_deref())
    }
}

/// How far the trailer's axle runs inside the tractor's path in a steady
/// turn of radius `radius_m` (m): R − √(R² − L²).
pub fn offtracking(first_wheelbase_m: f64, radius_m: f64) -> f64 {
    let (l, r) = (first_wheelbase_m.max(0.0), radius_m.abs());
    if l <= 0.0 || !r.is_finite() {
        return 0.0;
    }
    if r <= l {
        return r;
    }
    r - (r * r - l * l).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn semitrailer() -> TrailerConfiguration {
        let mut c = TrailerConfiguration {
            wheel_count: 6,
            hook_position: [0.0, 1.2, -5.5],
            ..Default::default()
        };
        for (i, z) in [3.6, 3.6, 4.9, 4.9, 6.2, 6.2].into_iter().enumerate() {
            c.wheel_positions[i] = [if i % 2 == 0 { -1.0 } else { 1.0 }, 0.5, z];
        }
        c
    }
    #[test]
    fn trailer_length_comes_from_its_kingpin_and_axles() {
        let (wheelbase, behind) = trailer_extent(&semitrailer()).unwrap();
        assert!((wheelbase - 10.4).abs() < 1e-6, "{wheelbase}");
        assert!((behind - 13.2).abs() < 1e-6, "{behind}");
        let mut raw = TelemetryFrame::default();
        assert_eq!(combination(&raw, None).trailers, 0);
        raw.trailers[0].connected = 1;
        let unknown = combination(&raw, None);
        assert_eq!(unknown.trailers, 1);
        assert!((unknown.behind_hitch_m - DEFAULT_BEHIND_KINGPIN_M).abs() < 1e-9);
        let mut frame: Box<ConfigurationFrame> = Box::default();
        frame.present_trailer_mask = 1;
        frame.trailers[0] = semitrailer();
        assert!((combination(&raw, Some(&frame)).behind_hitch_m - 13.2).abs() < 1e-6);
    }
    #[test]
    fn a_long_trailer_cuts_inside_a_tight_turn() {
        assert!(offtracking(10.4, 1000.0) < 0.06);
        let r20 = offtracking(10.4, 20.0);
        assert!((2.8..3.1).contains(&r20), "{r20}");
        assert_eq!(offtracking(0.0, 20.0), 0.0);
    }
}
