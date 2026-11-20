//! Bearing estimation: the direction a device lies in, from a power-weighted circular mean of
//! the device's signal strength over the user's headings.

use crate::algorithm::Algorithm;
use crate::model::ScreenFrame;
use spi::RadioSample;
use std::collections::{HashMap, VecDeque};

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

/// Sets `bearing_deg` on each device from the recent headings at which it was heard.
pub struct BearingEstimator {
    heading: Box<dyn Fn() -> Option<f64> + Send>,
    window: usize,
    /// Per device: (heading in degrees, linear power of the sample).
    observations: HashMap<String, VecDeque<(f64, f64)>>,
}

impl BearingEstimator {
    /// `heading` supplies the current heading in degrees clockwise from north, if known.
    /// `window` caps how many observations are kept per device.
    pub fn new(heading: impl Fn() -> Option<f64> + Send + 'static, window: usize) -> Self {
        Self {
            heading: Box::new(heading),
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
        if let Some(heading) = (self.heading)() {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DeviceState;
    use spi::Transport;
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
    fn apply_sets_bearing_when_heading_is_known() {
        let s = sample(-50);
        let mut frame = ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(&s)],
        };
        let mut estimator = BearingEstimator::new(|| Some(120.0), 8);
        estimator.apply(&s, &mut frame);
        assert!(angle_gap(frame.devices[0].bearing_deg.unwrap(), 120.0) < 1e-6);
    }

    #[test]
    fn apply_leaves_bearing_unset_without_heading() {
        let s = sample(-50);
        let mut frame = ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(&s)],
        };
        let mut estimator = BearingEstimator::new(|| None, 8);
        estimator.apply(&s, &mut frame);
        assert_eq!(frame.devices[0].bearing_deg, None);
    }
}
