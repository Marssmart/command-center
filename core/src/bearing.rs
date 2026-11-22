//! Bearing estimation: the direction a device lies in, relative to the sensor's facing.
//!
//! A bearing is reported only when the data supports it: the device was heard across enough of a
//! turn, and its signal swung with facing by more than shadowing noise. Otherwise it stays unset.

use crate::algorithm::Algorithm;
use crate::model::{DeviceState, PlacementMethod, ScreenFrame};
use spi::{HeadingSource, RadioSample};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

/// Number of facing bins in a sweep, each 10 degrees wide.
pub const SWEEP_BINS: usize = 36;
/// Bins that must be covered before a bearing is reported (75% of a turn).
pub const SWEEP_MIN_COVERAGE: usize = 27;
/// Minimum RSSI swing across facings, in dB. About two standard deviations of typical shadowing noise (3 dB).
pub const MIN_RSSI_SWING_DB: f64 = 6.0;
/// RSSI within this many dB of the strongest sample counts as part of the sweep peak. RSSI is
/// reported in whole dB, so a flat top spans several bins.
const PEAK_TOLERANCE_DB: f64 = 1.0;
/// Resultant lengths below this carry no direction.
const MIN_RESULTANT: f64 = 1e-9;

/// A circular estimate: bearing in degrees [0, 360) and its circular standard deviation in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bearing {
    pub bearing_deg: f64,
    pub spread_deg: f64,
}

/// Weighted circular mean of `(angle_deg, weight)` pairs, with its circular spread.
pub fn circular_estimate(observations: impl IntoIterator<Item = (f64, f64)>) -> Option<Bearing> {
    let (mut x, mut y, mut total) = (0.0_f64, 0.0_f64, 0.0_f64);
    for (angle_deg, weight) in observations {
        let rad = angle_deg.to_radians();
        x += weight * rad.cos();
        y += weight * rad.sin();
        total += weight;
    }
    if total <= 0.0 {
        return None;
    }
    let resultant = (x * x + y * y).sqrt() / total;
    if resultant < MIN_RESULTANT {
        return None;
    }
    Some(Bearing {
        bearing_deg: y.atan2(x).to_degrees().rem_euclid(360.0),
        spread_deg: (-2.0 * resultant.min(1.0).ln()).sqrt().to_degrees(),
    })
}

/// True when the observations cover enough facings and the signal swings with facing.
/// Each observation is `(facing_deg, rssi_dbm)`.
pub fn has_facing_evidence(observations: impl IntoIterator<Item = (f64, f64)>) -> bool {
    let mut covered = [false; SWEEP_BINS];
    let (mut lowest, mut highest) = (f64::INFINITY, f64::NEG_INFINITY);
    for (facing, rssi) in observations {
        covered[bin_index(facing)] = true;
        lowest = lowest.min(rssi);
        highest = highest.max(rssi);
    }
    covered.iter().filter(|c| **c).count() >= SWEEP_MIN_COVERAGE && highest - lowest >= MIN_RSSI_SWING_DB
}

fn bin_index(facing_deg: f64) -> usize {
    let bin_width = 360.0 / SWEEP_BINS as f64;
    (facing_deg.rem_euclid(360.0) / bin_width) as usize % SWEEP_BINS
}

fn power(rssi_dbm: f64) -> f64 {
    10f64.powf(rssi_dbm / 10.0)
}

/// Sets the bearing from an estimate, or clears it. Clearing leaves other methods' results alone.
fn apply_bearing(device: &mut DeviceState, estimate: Option<Bearing>, method: PlacementMethod) {
    match estimate {
        Some(b) => {
            device.bearing_deg = Some(b.bearing_deg);
            device.bearing_spread_deg = Some(b.spread_deg);
            device.placement = method;
        }
        None => {
            device.bearing_deg = None;
            device.bearing_spread_deg = None;
            if device.placement == method {
                device.placement = PlacementMethod::Unplaced;
            }
        }
    }
}

/// Sets `bearing_deg` from the recent facings at which each device was heard, once they show a bearing.
pub struct BearingEstimator {
    heading: Arc<dyn HeadingSource>,
    window: usize,
    /// Per device: (facing in degrees, RSSI in dBm) of recent samples.
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
        if let Some(facing) = self.heading.heading_deg() {
            let history = self.observations.entry(sample.device_id.clone()).or_default();
            history.push_back((facing, f64::from(sample.rssi_dbm)));
            while history.len() > self.window {
                history.pop_front();
            }
        }

        let Some(history) = self.observations.get(&sample.device_id) else {
            return;
        };
        let Some(device) = frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        else {
            return;
        };
        let estimate = if has_facing_evidence(history.iter().copied()) {
            circular_estimate(history.iter().map(|(facing, rssi)| (*facing, power(*rssi))))
        } else {
            None
        };
        apply_bearing(device, estimate, PlacementMethod::Facing);
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
        if let Some(facing) = self.heading.heading_deg() {
            let bin = bin_index(facing);
            let rssi = f64::from(sample.rssi_dbm);
            if bins[bin].is_none_or(|(_, best)| rssi > best) {
                bins[bin] = Some((facing, rssi));
            }
        }

        let Some(device) = frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        else {
            return;
        };
        apply_bearing(device, sweep_peak(bins), PlacementMethod::Sweep);
    }
}

/// Bearing at the centre of the strongest plateau: the power-weighted circular mean of every bin
/// within `PEAK_TOLERANCE_DB` of the maximum. `None` without facing evidence.
pub fn sweep_peak(bins: &[Option<(f64, f64)>; SWEEP_BINS]) -> Option<Bearing> {
    let observed: Vec<(f64, f64)> = bins.iter().flatten().copied().collect();
    if !has_facing_evidence(observed.iter().copied()) {
        return None;
    }
    let strongest = observed
        .iter()
        .map(|(_, rssi)| *rssi)
        .fold(f64::NEG_INFINITY, f64::max);
    circular_estimate(
        observed
            .iter()
            .filter(|(_, rssi)| *rssi >= strongest - PEAK_TOLERANCE_DB)
            .map(|(facing, rssi)| (*facing, power(*rssi))),
    )
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
            details: Default::default(),
        }
    }

    fn frame_for(s: &RadioSample) -> ScreenFrame {
        ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(s)],
        }
    }

    /// Signal from a target at `target_deg`, seen by a sensor facing `facing_deg`.
    fn rssi_towards(target_deg: f64, facing_deg: f64) -> i16 {
        (-50.0 + 6.0 * (facing_deg - target_deg).to_radians().cos()).round() as i16
    }

    /// Feeds one full turn in 5-degree steps. `rssi_at` gives the RSSI for each facing.
    fn full_turn(rssi_at: impl Fn(f64) -> i16) -> impl Iterator<Item = (f64, RadioSample)> {
        (0..72).map(move |step| {
            let f = step as f64 * 5.0;
            (f, sample(rssi_at(f)))
        })
    }

    #[test]
    fn circular_estimate_wraps_around_north() {
        let bearing = circular_estimate([(350.0, 1.0), (10.0, 1.0)]).unwrap();
        assert!(angle_gap(bearing.bearing_deg, 0.0) < 1e-6);
    }

    #[test]
    fn circular_estimate_reports_spread() {
        let tight = circular_estimate([(90.0, 1.0), (92.0, 1.0)]).unwrap();
        let wide = circular_estimate([(0.0, 1.0), (180.0, 1.0)]);
        assert!(tight.spread_deg < 3.0);
        assert!(wide.is_none(), "opposite facings cancel out, so there is no direction");
    }

    #[test]
    fn single_fixed_facing_gives_no_bearing() {
        let facing = Arc::new(ManualFacing::new());
        facing.set(Some(120.0));
        let mut estimator = BearingEstimator::new(facing, 64);
        let mut frame = ScreenFrame::default();
        for rssi in [-50, -44, -58, -40, -52] {
            let s = sample(rssi);
            frame = frame_for(&s);
            estimator.apply(&s, &mut frame);
        }
        assert_eq!(frame.devices[0].bearing_deg, None);
        assert_eq!(frame.devices[0].placement, PlacementMethod::Unplaced);
    }

    #[test]
    fn flat_rssi_over_full_turn_gives_no_bearing() {
        let facing = Arc::new(ManualFacing::new());
        let mut estimator = BearingEstimator::new(facing.clone(), 64);
        let mut sweep = SweepBearing::new(facing.clone());
        let mut last_estimate = None;
        let mut last_sweep = None;
        for (f, s) in full_turn(|_| -50) {
            facing.set(Some(f));
            let mut frame = frame_for(&s);
            estimator.apply(&s, &mut frame);
            last_estimate = frame.devices[0].bearing_deg;
            let mut frame = frame_for(&s);
            sweep.apply(&s, &mut frame);
            last_sweep = frame.devices[0].bearing_deg;
        }
        assert_eq!(last_estimate, None);
        assert_eq!(last_sweep, None);
    }

    #[test]
    fn estimator_finds_direction_from_full_turn() {
        let target = 200.0;
        let facing = Arc::new(ManualFacing::new());
        let mut estimator = BearingEstimator::new(facing.clone(), 64);
        let mut bearing = None;
        for (f, s) in full_turn(|f| rssi_towards(target, f)) {
            facing.set(Some(f));
            let mut frame = frame_for(&s);
            estimator.apply(&s, &mut frame);
            bearing = frame.devices[0].bearing_deg;
        }
        assert!(angle_gap(bearing.unwrap(), target) < 5.0);
    }

    #[test]
    fn sweep_finds_target_direction_from_full_turn() {
        let target = 90.0;
        let facing = Arc::new(ManualFacing::new());
        let mut sweep = SweepBearing::new(facing.clone());
        let mut bearing = None;
        for (f, s) in full_turn(|f| rssi_towards(target, f)) {
            facing.set(Some(f));
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
        let bearing = sweep_peak(&bins).unwrap().bearing_deg;
        assert!(angle_gap(bearing, 355.0) < 3.0);
    }

    #[test]
    fn sweep_waits_for_enough_coverage() {
        let mut bins = [None; SWEEP_BINS];
        // Alternate -60 and -50 dBm so the swing passes; coverage is the only gate here.
        for (i, bin) in bins.iter_mut().enumerate().take(SWEEP_MIN_COVERAGE - 1) {
            let rssi = if i % 2 == 0 { -60.0 } else { -50.0 };
            *bin = Some((i as f64 * 10.0, rssi));
        }
        assert_eq!(sweep_peak(&bins), None);
        bins[SWEEP_MIN_COVERAGE - 1] = Some(((SWEEP_MIN_COVERAGE - 1) as f64 * 10.0, -50.0));
        assert!(sweep_peak(&bins).is_some());
    }
}
