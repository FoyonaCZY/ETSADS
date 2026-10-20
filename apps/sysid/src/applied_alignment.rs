//! Record ordering is not control causality: the recorder can append a later
//! input snapshot beside an earlier telemetry frame. Align by actual QPC time.
use etsads_msgs::protocol::AppliedControl;
#[derive(Default)]
pub struct AppliedTimeline {
    samples: Vec<AppliedControl>,
}
impl AppliedTimeline {
    pub fn push(&mut self, value: AppliedControl) -> anyhow::Result<()> {
        anyhow::ensure!(
            self.samples.len() < 5_000_000,
            "applied-control recording exceeds analysis budget"
        );
        if value.applied_qpc > 0 {
            self.samples.push(value);
        }
        Ok(())
    }
    pub fn finish(&mut self) {
        self.samples
            .sort_by_key(|sample| (sample.applied_qpc, sample.frame_index, sample.applied_seq));
    }
    pub fn at(&self, qpc: u64, frequency: u64) -> Option<&AppliedControl> {
        if qpc == 0 || frequency == 0 {
            return None;
        }
        let index = self
            .samples
            .partition_point(|sample| sample.applied_qpc <= qpc)
            .checked_sub(1)?;
        let sample = &self.samples[index];
        if sample.applied_seq == 0
            || sample.reason != 0
            || (qpc - sample.applied_qpc) as u128 * 1000 > frequency as u128 * 200
            || sample.axes.iter().any(|a| !a.is_finite())
            || sample.axes[0].abs() > 1.0
            || sample.axes[1..].iter().any(|a| !(0.0..=1.0).contains(a))
        {
            return None;
        }
        Some(sample)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn no_future_input_and_gate_release_is_not_bridged() {
        let mut timeline = AppliedTimeline::default();
        timeline
            .push(AppliedControl {
                applied_seq: 1,
                applied_qpc: 1000,
                axes: [0.2, 0.0, 0.0, 0.0],
                ..Default::default()
            })
            .unwrap();
        timeline
            .push(AppliedControl {
                applied_seq: 2,
                applied_qpc: 1100,
                axes: [-0.2, 0.0, 0.0, 0.0],
                ..Default::default()
            })
            .unwrap();
        timeline
            .push(AppliedControl {
                applied_seq: 2,
                applied_qpc: 1150,
                reason: 1,
                ..Default::default()
            })
            .unwrap();
        timeline.finish();
        assert_eq!(timeline.at(1050, 1000).unwrap().axes[0], 0.2);
        assert_eq!(timeline.at(1120, 1000).unwrap().axes[0], -0.2);
        assert!(timeline.at(1160, 1000).is_none());
        assert!(timeline.at(999, 1000).is_none());
    }
}
