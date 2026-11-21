//! Bearing estimation: the direction a device lies in, relative to the sensor's facing.
//!
//! Two estimators share one facing source. `BearingEstimator` takes a power-weighted circular
//! mean over recent facings. `SweepBearing` takes the strongest facing after a full turn.

use crate::algorithm::Algorithm;
use crate::model::ScreenFrame;
use spi::{HeadingSource, RadioSample};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

/// Number of facing bins in a sweep, each 10 degrees wide.
pub const SWEEP_BINS: usize = 36;
/// A sweep is only trusted once this many bins have data (75% of a turn).
pub const SWEEP_MIN_COVERAGE: usize = 27;

/// Circular weighted mean of headings (degrees). Returns `None` if there is no usable weight.
pub fn circular_bearing_deg(observations: impl IntoIterator<Item = (f64, f64)>) -> Option<f64> {
    let (mut x, mut y) = (0.0_f64, 0.0_f64);
    for (heading_deg, weight) in observations {
        let rad = heading_deg.to_radians();
        x += weight * rad.cos();
        y += weight * rad.sin();
    }
    if x.abs() < f64::EPSILON && y.abs() < f64::EPSILON {
        return None;
    }
    Some(y.atan2(x).to_degrees().rem_euclid(360.0))
}

/// Sets `bearing_deg` on each device from the recent facings at which it was heard.
pub struct BearingEstimator {
    heading: Arc<dyn HeadingSource>,
    window: usize,
    /// Per device: (facing in degrees, linear power of the sample).
    observations: HashMap<String, VecDeque<(f64, f64)>>,
}

impl BearingEstimator {
    /// `window` caps how many observations are kept per device.
    pub fn new(heading: Arc<dyn HeadingSource>, window: usize) -> Self {
        Self {
            heading,
            window,
            observations: HashMap::new(),
        }
    }
}

impl Algorithm for BearingEstimator {
    fn name(&self) -> &'static str {
        "bearing"
    }

    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
        if let Some(heading) = self.heading.heading_deg() {
            let power = 10f64.powf(f64::from(sample.rssi_dbm) / 10.0);
            let history = self.observations.entry(sample.device_id.clone()).or_default();
            history.push_back((heading, power));
            while history.len() > self.window {
                history.pop_front();
            }
        }

        let bearing = self
            .observations
            .get(&sample.device_id)
            .and_then(|history| circular_bearing_deg(history.iter().copied()));
        if let (Some(bearing), Some(device)) = (
            bearing,
            frame
                .devices
                .iter_mut()
                .find(|d| d.device_id == sample.device_id),
        ) {
            device.bearing_deg = Some(bearing);
        }
    }
}

/// Strongest facing from a full turn. Each device keeps the best RSSI per facing bin.
pub struct SweepBearing {
    heading: Arc<dyn HeadingSource>,
    /// Per device, per bin: the facing and RSSI (dBm) of the strongest sample in that bin.
    bins: HashMap<String, [Option<(f64, f64)>; SWEEP_BINS]>,
}

impl SweepBearing {
    pub fn new(heading: Arc<dyn HeadingSource>) -> Self {
        Self {
            heading,
            bins: HashMap::new(),
        }
    }
}

impl Algorithm for SweepBearing {
    fn name(&self) -> &'static str {
        "sweep-bearing"
    }

    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
        let bins = self
            .bins
            .entry(sample.device_id.clone())
            .or_insert([None; SWEEP_BINS]);
        if let Some(heading) = self.heading.heading_deg() {
            let bin = bin_index(heading);
            let rssi = f64::from(sample.rssi_dbm);
            if bins[bin].is_none_or(|(_, best)| rssi > best) {
                bins[bin] = Some((heading, rssi));
            }
        }

        if let (Some(bearing), Some(device)) = (
            sweep_peak_deg(bins),
            frame
                .devices
                .iter_mut()
                .find(|d| d.device_id == sample.device_id),
        ) {
            device.bearing_deg = Some(bearing);
        }
    }
}

fn bin_index(facing_deg: f64) -> usize {
    let bin_width = 360.0 / SWEEP_BINS as f64;
    (facing_deg.rem_euclid(360.0) / bin_width) as usize % SWEEP_BINS
}

/// RSSI within this many dB of the strongest bin counts as part of the peak. RSSI is reported
/// in whole dB, so a flat top spans several bins and a single argmax would pick an arbitrary one.
const PEAK_TOLERANCE_DB: f64 = 1.0;

/// Bearing at the centre of the strongest plateau: the power-weighted circular mean of every
/// covered bin within `PEAK_TOLERANCE_DB` of the maximum, using the facing where each was measured.
/// `None` until enough of the turn has been covered.
pub fn sweep_peak_deg(bins: &[Option<(f64, f64)>; SWEEP_BINS]) -> Option<f64> {
    let covered = bins.iter().filter(|b| b.is_some()).count();
    if covered < SWEEP_MIN_COVERAGE {
        return None;
    }
    let strongest = bins
        .iter()
        .flatten()
        .map(|(_, rssi)| *rssi)
        .fold(f64::NEG_INFINITY, f64::max);
    let peak = bins.iter().flatten().filter(|(_, rssi)| *rssi >= strongest - PEAK_TOLERANCE_DB);
    circular_bearing_deg(peak.map(|(facing, rssi)| (*facing, 10f64.powf(rssi / 10.0))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DeviceState;
    use spi::{ManualFacing, Transport};
    use std::time::Duration;

    /// Smallest angular difference, so 359.9 and 0.1 compare as close.
    fn angle_gap(a: f64, b: f64) -> f64 {
        let d = (a - b).rem_euclid(360.0);
        d.min(360.0 - d)
    }

    fn sample(rssi: i16) -> RadioSample {
        RadioSample {
            transport: Transport::Wifi,
            device_id: "ap".into(),
            rssi_dbm: rssi,
            observed_at: Duration::from_secs(1),
        }
    }

    fn frame_for(s: &RadioSample) -> ScreenFrame {
        ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(s)],
        }
    }

    fn fixed_facing(deg: f64) -> Arc<ManualFacing> {
        let facing = Arc::new(ManualFacing::new());
        facing.set(Some(deg));
        facing
    }

    /// Signal from a target at `target_deg`, seen by a sensor facing `facing_deg`.
    fn rssi_towards(target_deg: f64, facing_deg: f64) -> i16 {
        (-50.0 + 6.0 * (facing_deg - target_deg).to_radians().cos()).round() as i16
    }

    #[test]
    fn mean_wraps_around_north() {
        let bearing = circular_bearing_deg([(350.0, 1.0), (10.0, 1.0)]).unwrap();
        assert!(angle_gap(bearing, 0.0) < 1e-6);
    }

    #[test]
    fn stronger_observation_dominates() {
        let bearing = circular_bearing_deg([(90.0, 1.0), (270.0, 0.01)]).unwrap();
        assert!(angle_gap(bearing, 90.0) < 1.0);
    }

    #[test]
    fn no_weight_gives_no_bearing() {
        assert_eq!(circular_bearing_deg(std::iter::empty()), None);
    }

    #[test]
    fn estimator_uses_known_facing() {
        let s = sample(-50);
        let mut frame = frame_for(&s);
        let mut estimator = BearingEstimator::new(fixed_facing(120.0), 8);
        estimator.apply(&s, &mut frame);
        assert!(angle_gap(frame.devices[0].bearing_deg.unwrap(), 120.0) < 1e-6);
    }

    #[test]
    fn estimator_leaves_bearing_unset_without_facing() {
        let s = sample(-50);
        let mut frame = frame_for(&s);
        let mut estimator = BearingEstimator::new(Arc::new(ManualFacing::new()), 8);
        estimator.apply(&s, &mut frame);
        assert_eq!(frame.devices[0].bearing_deg, None);
    }

    #[test]
    fn sweep_finds_target_direction_from_full_turn() {
        let target = 90.0;
        let facing = Arc::new(ManualFacing::new());
        let mut sweep = SweepBearing::new(facing.clone());
        let mut bearing = None;
        for step in 0..72 {
            let f = step as f64 * 5.0;
            facing.set(Some(f));
            let s = sample(rssi_towards(target, f));
            let mut frame = frame_for(&s);
            sweep.apply(&s, &mut frame);
            bearing = frame.devices[0].bearing_deg;
        }
        assert!(angle_gap(bearing.unwrap(), target) < 2.0);
    }

    #[test]
    fn sweep_peak_wraps_around_zero() {
        let mut bins = [None; SWEEP_BINS];
        for (i, bin) in bins.iter_mut().enumerate() {
            let facing = (i as f64 + 0.5) * 10.0;
            *bin = Some((facing, f64::from(rssi_towards(355.0, facing))));
        }
        let bearing = sweep_peak_deg(&bins).unwrap();
        assert!(angle_gap(bearing, 355.0) < 3.0);
    }

    #[test]
    fn sweep_waits_for_enough_coverage() {
        let mut bins = [None; SWEEP_BINS];
        for (i, bin) in bins.iter_mut().enumerate().take(SWEEP_MIN_COVERAGE - 1) {
            *bin = Some((i as f64 * 10.0, -60.0));
        }
        assert_eq!(sweep_peak_deg(&bins), None);
        bins[SWEEP_MIN_COVERAGE - 1] = Some(((SWEEP_MIN_COVERAGE - 1) as f64 * 10.0, -60.0));
        assert!(sweep_peak_deg(&bins).is_some());
    }
}
