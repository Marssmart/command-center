//! RSSI ranging: estimates distance from smoothed signal strength with a log-distance path-loss model.

use crate::algorithm::Algorithm;
use crate::model::ScreenFrame;
use spi::RadioSample;
use std::collections::HashMap;

/// Distance in metres for a received power, given the power at 1 m and the path-loss exponent.
pub fn distance_from_rssi(rssi_dbm: f64, tx_power_dbm: f64, path_loss_exponent: f64) -> f64 {
    10f64.powf((tx_power_dbm - rssi_dbm) / (10.0 * path_loss_exponent))
}

/// One-sigma relative error of an RSSI distance, for the default path-loss exponent (2.0) and 3 dB
/// shadowing: sigma_d / d = ln(10) * sigma_rssi / (10 * n).
pub const RELATIVE_DISTANCE_ERROR: f64 = std::f64::consts::LN_10 * 3.0 / (10.0 * 2.0);

/// Sets `distance_m` on each device from its exponentially smoothed RSSI.
pub struct RssiRanging {
    tx_power_dbm: f64,
    path_loss_exponent: f64,
    /// Weight of each new sample, in (0, 1]. Higher reacts faster and is noisier.
    smoothing: f64,
    smoothed_rssi: HashMap<String, f64>,
}

impl RssiRanging {
    pub fn new(tx_power_dbm: f64, path_loss_exponent: f64, smoothing: f64) -> Self {
        Self {
            tx_power_dbm,
            path_loss_exponent,
            smoothing,
            smoothed_rssi: HashMap::new(),
        }
    }
}

impl Default for RssiRanging {
    /// Typical BLE beacon reference power and an indoor path-loss exponent.
    fn default() -> Self {
        Self::new(-59.0, 2.0, 0.3)
    }
}

impl Algorithm for RssiRanging {
    fn name(&self) -> &'static str {
        "rssi-ranging"
    }

    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
        let raw = f64::from(sample.rssi_dbm);
        let rssi = match self.smoothed_rssi.get(&sample.device_id) {
            Some(prev) => prev + self.smoothing * (raw - prev),
            None => raw,
        };
        self.smoothed_rssi.insert(sample.device_id.clone(), rssi);

        if let Some(device) = frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        {
            device.distance_m = Some(distance_from_rssi(rssi, self.tx_power_dbm, self.path_loss_exponent));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DeviceState;
    use spi::Transport;
    use std::time::Duration;

    fn sample(rssi: i16, secs: u64) -> RadioSample {
        RadioSample {
            transport: Transport::BluetoothLe,
            device_id: "beacon".into(),
            rssi_dbm: rssi,
            observed_at: Duration::from_secs(secs),
            details: Default::default(),
        }
    }

    fn frame_with(sample: &RadioSample) -> ScreenFrame {
        ScreenFrame {
            at: sample.observed_at,
            devices: vec![DeviceState::from_sample(sample)],
        }
    }

    #[test]
    fn distance_matches_log_distance_model() {
        assert!((distance_from_rssi(-59.0, -59.0, 2.0) - 1.0).abs() < 1e-9);
        // 20 dB weaker with exponent 2 is 10x the distance.
        assert!((distance_from_rssi(-79.0, -59.0, 2.0) - 10.0).abs() < 1e-9);
    }

    #[test]
    fn first_sample_sets_distance_without_smoothing() {
        let mut ranging = RssiRanging::new(-59.0, 2.0, 0.5);
        let s = sample(-79, 1);
        let mut frame = frame_with(&s);
        ranging.apply(&s, &mut frame);
        assert!((frame.devices[0].distance_m.unwrap() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn later_samples_are_smoothed() {
        let mut ranging = RssiRanging::new(-59.0, 2.0, 0.5);
        let first = sample(-59, 1);
        let mut frame = frame_with(&first);
        ranging.apply(&first, &mut frame);

        // Smoothed RSSI moves halfway from -59 to -79, i.e. -69.
        let second = sample(-79, 2);
        ranging.apply(&second, &mut frame);
        let expected = distance_from_rssi(-69.0, -59.0, 2.0);
        assert!((frame.devices[0].distance_m.unwrap() - expected).abs() < 1e-9);
    }
}
