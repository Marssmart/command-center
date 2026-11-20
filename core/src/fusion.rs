//! Fusion: combines distance and bearing into a smoothed (east, north) position per device.

use crate::algorithm::Algorithm;
use crate::model::ScreenFrame;
use spi::RadioSample;
use std::collections::HashMap;

/// Converts a distance and a clockwise-from-north bearing into an (east, north) offset in metres.
pub fn position_from_polar(distance_m: f64, bearing_deg: f64) -> (f64, f64) {
    let rad = bearing_deg.to_radians();
    (distance_m * rad.sin(), distance_m * rad.cos())
}

/// Sets `position_m` on each device that has both a distance and a bearing.
pub struct Fusion {
    /// Weight of each new estimate, in (0, 1]. Lower values are steadier and lag more.
    smoothing: f64,
    positions: HashMap<String, (f64, f64)>,
}

impl Fusion {
    pub fn new(smoothing: f64) -> Self {
        Self {
            smoothing,
            positions: HashMap::new(),
        }
    }
}

impl Default for Fusion {
    fn default() -> Self {
        Self::new(0.5)
    }
}

impl Algorithm for Fusion {
    fn name(&self) -> &'static str {
        "fusion"
    }

    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
        let Some(device) = frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        else {
            return;
        };
        let (Some(distance), Some(bearing)) = (device.distance_m, device.bearing_deg) else {
            return;
        };

        let (raw_east, raw_north) = position_from_polar(distance, bearing);
        let position = match self.positions.get(&sample.device_id) {
            Some(&(east, north)) => (
                east + self.smoothing * (raw_east - east),
                north + self.smoothing * (raw_north - north),
            ),
            None => (raw_east, raw_north),
        };
        self.positions.insert(sample.device_id.clone(), position);
        device.position_m = Some(position);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DeviceState;
    use spi::Transport;
    use std::time::Duration;

    fn assert_close(actual: (f64, f64), expected: (f64, f64)) {
        assert!((actual.0 - expected.0).abs() < 1e-9, "east {actual:?} vs {expected:?}");
        assert!((actual.1 - expected.1).abs() < 1e-9, "north {actual:?} vs {expected:?}");
    }

    fn sample() -> RadioSample {
        RadioSample {
            transport: Transport::BluetoothLe,
            device_id: "beacon".into(),
            rssi_dbm: -60,
            observed_at: Duration::from_secs(1),
        }
    }

    #[test]
    fn polar_maps_north_and_east() {
        assert_close(position_from_polar(10.0, 0.0), (0.0, 10.0));
        assert_close(position_from_polar(10.0, 90.0), (10.0, 0.0));
    }

    #[test]
    fn apply_needs_distance_and_bearing() {
        let s = sample();
        let mut frame = ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(&s)],
        };
        frame.devices[0].distance_m = Some(5.0);
        Fusion::default().apply(&s, &mut frame);
        assert_eq!(frame.devices[0].position_m, None);
    }

    #[test]
    fn apply_smooths_successive_positions() {
        let s = sample();
        let mut fusion = Fusion::new(0.5);
        let mut frame = ScreenFrame {
            at: s.observed_at,
            devices: vec![DeviceState::from_sample(&s)],
        };
        frame.devices[0].distance_m = Some(10.0);
        frame.devices[0].bearing_deg = Some(0.0);
        fusion.apply(&s, &mut frame);
        assert_close(frame.devices[0].position_m.unwrap(), (0.0, 10.0));

        // New estimate is due east at 10 m; smoothing moves halfway from (0, 10) to (10, 0).
        frame.devices[0].bearing_deg = Some(90.0);
        fusion.apply(&s, &mut frame);
        assert_close(frame.devices[0].position_m.unwrap(), (5.0, 5.0));
    }
}
