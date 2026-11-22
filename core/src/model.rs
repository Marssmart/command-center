//! Model: per-device state and the frames screens render.

use spi::{RadioSample, SampleDetails, Transport};
use std::collections::VecDeque;
use std::time::Duration;

/// Most recent RSSI values kept per device for the detail plot.
pub const RSSI_HISTORY_LEN: usize = 64;

/// How a device's bearing and position were obtained. Shown to the user so placements can be weighed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlacementMethod {
    /// Not placed: distance only, or nothing.
    #[default]
    Unplaced,
    /// Continuous bearing from the manual facing dial, once the signal showed it varies with facing.
    Facing,
    /// Peak of a full-turn sweep against the facing dial.
    Sweep,
    /// Solved from distances measured at surveyed spots. Bearings are from map north.
    Survey,
}

/// Latest known state of one observed device.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceState {
    pub device_id: String,
    pub transport: Transport,
    pub last_rssi_dbm: i16,
    /// Time of the first accepted sample.
    pub first_seen: Duration,
    /// Time of the most recent accepted sample, on the `Clock` timeline.
    pub last_seen: Duration,
    /// Accepted samples. Out-of-order samples older than `last_seen` are not counted.
    pub sample_count: u64,
    /// Most recent RSSI values, oldest first, at most `RSSI_HISTORY_LEN` long.
    pub rssi_history: VecDeque<i16>,
    /// Transport-specific facts from the latest accepted sample.
    pub details: SampleDetails,
    /// Estimated distance in metres from RSSI. Set to the geometric distance from the current spot once surveyed.
    pub distance_m: Option<f64>,
    /// Estimated bearing in degrees clockwise from the reference: the facing for facing modes, map north for a survey.
    pub bearing_deg: Option<f64>,
    /// One standard deviation of the bearing, in degrees. Not set for surveyed bearings.
    pub bearing_spread_deg: Option<f64>,
    /// Estimated (east, north) offset in metres from the origin.
    pub position_m: Option<(f64, f64)>,
    /// One standard deviation of the position, in metres.
    pub position_uncertainty_m: Option<f64>,
    /// Which method produced the bearing and position.
    pub placement: PlacementMethod,
}

impl DeviceState {
    /// Creates the state of a device from its first sample.
    pub fn from_sample(sample: &RadioSample) -> Self {
        Self {
            device_id: sample.device_id.clone(),
            transport: sample.transport,
            last_rssi_dbm: sample.rssi_dbm,
            first_seen: sample.observed_at,
            last_seen: sample.observed_at,
            sample_count: 1,
            rssi_history: VecDeque::from([sample.rssi_dbm]),
            details: sample.details.clone(),
            distance_m: None,
            bearing_deg: None,
            bearing_spread_deg: None,
            position_m: None,
            position_uncertainty_m: None,
            placement: PlacementMethod::Unplaced,
        }
    }

    /// Folds a newer sample into this state. Samples older than `last_seen` are ignored.
    pub fn observe(&mut self, sample: &RadioSample) {
        if sample.observed_at < self.last_seen {
            return;
        }
        self.last_rssi_dbm = sample.rssi_dbm;
        self.last_seen = sample.observed_at;
        self.sample_count += 1;
        self.rssi_history.push_back(sample.rssi_dbm);
        while self.rssi_history.len() > RSSI_HISTORY_LEN {
            self.rssi_history.pop_front();
        }
        self.details = sample.details.clone();
    }

    /// Drops the bearing and position so a different method can take over. Distance is kept.
    pub fn clear_placement(&mut self) {
        self.bearing_deg = None;
        self.bearing_spread_deg = None;
        self.position_m = None;
        self.position_uncertainty_m = None;
        self.placement = PlacementMethod::Unplaced;
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
            details: Default::default(),
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
    fn first_seen_and_sample_count_track_accepted_samples() {
        let mut state = DeviceState::from_sample(&sample("dev", -70, 10));
        state.observe(&sample("dev", -65, 11));
        state.observe(&sample("dev", -60, 12));
        state.observe(&sample("dev", -90, 5)); // out of order: not counted

        assert_eq!(state.first_seen, Duration::from_secs(10));
        assert_eq!(state.last_seen, Duration::from_secs(12));
        assert_eq!(state.sample_count, 3);
    }

    #[test]
    fn rssi_history_keeps_the_most_recent_values() {
        let mut state = DeviceState::from_sample(&sample("dev", -100, 0));
        for step in 1..=(RSSI_HISTORY_LEN as u64 + 10) {
            state.observe(&sample("dev", -50, step));
        }
        assert_eq!(state.rssi_history.len(), RSSI_HISTORY_LEN);
        assert_eq!(state.rssi_history.front(), Some(&-50));
        assert_eq!(state.sample_count, RSSI_HISTORY_LEN as u64 + 11);
    }

    #[test]
    fn details_follow_the_latest_accepted_sample() {
        let mut first = sample("dev", -70, 1);
        first.details.manufacturer_ids = vec![0x004C];
        let mut state = DeviceState::from_sample(&first);
        let mut later = sample("dev", -60, 2);
        later.details.ssid = Some("office".into());
        state.observe(&later);
        assert_eq!(state.details.ssid.as_deref(), Some("office"));
        assert!(state.details.manufacturer_ids.is_empty());
    }

    #[test]
    fn clear_placement_keeps_distance() {
        let mut state = DeviceState::from_sample(&sample("dev", -60, 1));
        state.distance_m = Some(4.0);
        state.bearing_deg = Some(30.0);
        state.position_m = Some((1.0, 2.0));
        state.placement = PlacementMethod::Sweep;
        state.clear_placement();
        assert_eq!(state.distance_m, Some(4.0));
        assert_eq!(state.bearing_deg, None);
        assert_eq!(state.position_m, None);
        assert_eq!(state.placement, PlacementMethod::Unplaced);
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
