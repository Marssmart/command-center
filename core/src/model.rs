//! Core data model: per-device state and the frames screens render.

use spi::{RadioSample, Transport};
use std::time::Duration;

/// Latest known state of one observed device.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceState {
    pub device_id: String,
    pub transport: Transport,
    pub last_rssi_dbm: i16,
    /// Time of the most recent sample, on the `Clock` timeline.
    pub last_seen: Duration,
    /// Estimated distance in metres, if an algorithm has produced one.
    pub distance_m: Option<f64>,
    /// Estimated bearing in degrees clockwise from north, if known.
    pub bearing_deg: Option<f64>,
    /// Estimated (east, north) offset from the user in metres, once fused from distance and bearing.
    pub position_m: Option<(f64, f64)>,
}

impl DeviceState {
    /// Creates the state of a device from its first sample.
    pub fn from_sample(sample: &RadioSample) -> Self {
        Self {
            device_id: sample.device_id.clone(),
            transport: sample.transport,
            last_rssi_dbm: sample.rssi_dbm,
            last_seen: sample.observed_at,
            distance_m: None,
            bearing_deg: None,
            position_m: None,
        }
    }

    /// Folds a newer sample into this state. Samples older than `last_seen` are ignored.
    pub fn observe(&mut self, sample: &RadioSample) {
        if sample.observed_at < self.last_seen {
            return;
        }
        self.last_rssi_dbm = sample.rssi_dbm;
        self.last_seen = sample.observed_at;
    }
}

/// Snapshot of everything a screen needs at one point in time. Algorithms publish these.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ScreenFrame {
    /// Time the frame describes, on the `Clock` timeline.
    pub at: Duration,
    pub devices: Vec<DeviceState>,
}

impl ScreenFrame {
    pub fn device(&self, device_id: &str) -> Option<&DeviceState> {
        self.devices.iter().find(|d| d.device_id == device_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(id: &str, rssi: i16, secs: u64) -> RadioSample {
        RadioSample {
            transport: Transport::BluetoothLe,
            device_id: id.into(),
            rssi_dbm: rssi,
            observed_at: Duration::from_secs(secs),
        }
    }

    #[test]
    fn observe_updates_to_newer_sample_and_ignores_older() {
        let mut state = DeviceState::from_sample(&sample("dev", -70, 10));
        state.observe(&sample("dev", -60, 12));
        assert_eq!((state.last_rssi_dbm, state.last_seen), (-60, Duration::from_secs(12)));

        state.observe(&sample("dev", -90, 5));
        assert_eq!((state.last_rssi_dbm, state.last_seen), (-60, Duration::from_secs(12)));
    }

    #[test]
    fn frame_finds_device_by_id() {
        let frame = ScreenFrame {
            at: Duration::from_secs(1),
            devices: vec![DeviceState::from_sample(&sample("a", -50, 1))],
        };
        assert_eq!(frame.device("a").map(|d| d.last_rssi_dbm), Some(-50));
        assert!(frame.device("missing").is_none());
    }
}
