//! Service provider interface: the traits `core` depends on and platform crates implement.
//!
//! Shared sample types live here because `core` depends on `spi`, not the other way round.

use std::error::Error;
use std::fmt;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::time::Duration;

pub mod heading;
pub mod identity;
pub mod status;
pub mod wifi;
pub use heading::{HeadingSource, ManualFacing};
pub use identity::{ble_fingerprint, ble_payload_hash};
pub use status::{ScanProblem, ScanState, ScanStatusBoard, ScanStatusSource};
pub use wifi::{WifiBand, WifiSecurity};

/// Failure reported by an SPI implementation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpiError {
    /// The platform does not support the requested capability.
    Unsupported(String),
    /// The required OS permission is missing.
    PermissionDenied(Permission),
    /// Any other failure, with a human-readable message.
    Other(String),
}

impl fmt::Display for SpiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SpiError::Unsupported(what) => write!(f, "unsupported: {what}"),
            SpiError::PermissionDenied(p) => write!(f, "permission denied: {p:?}"),
            SpiError::Other(msg) => write!(f, "{msg}"),
        }
    }
}

impl Error for SpiError {}

/// Radio technology a sample was captured on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    BluetoothLe,
    Wifi,
}

/// Transport-specific facts about a sample. Shown on the device detail view; never used to match devices.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SampleDetails {
    /// BLE manufacturer company identifiers, sorted.
    pub manufacturer_ids: Vec<u16>,
    /// BLE service UUIDs as 128-bit values.
    pub service_uuids: Vec<u128>,
    /// Hash of the BLE advertisement payload, as used by the fingerprint.
    pub payload_hash: Option<u64>,
    /// Wi-Fi network name. `None` for a hidden network, which broadcasts no name.
    pub ssid: Option<String>,
    /// Wi-Fi channel number.
    pub channel: Option<u16>,
    /// Wi-Fi frequency band.
    pub band: Option<WifiBand>,
    /// Wi-Fi channel width in MHz.
    pub channel_width_mhz: Option<u16>,
    /// Strongest security the Wi-Fi access point offers.
    pub security: Option<WifiSecurity>,
    /// Wi-Fi noise floor in dBm, measured on the access point's channel.
    pub noise_dbm: Option<i16>,
}

/// One received-signal observation from a radio scan.
#[derive(Debug, Clone, PartialEq)]
pub struct RadioSample {
    pub transport: Transport,
    /// Platform identifier: BLE fingerprint or Wi-Fi BSSID.
    pub device_id: String,
    pub rssi_dbm: i16,
    /// Time since the `Clock` epoch.
    pub observed_at: Duration,
    pub details: SampleDetails,
}

/// OS permissions that scanning may require.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Permission {
    Bluetooth,
    Location,
}

/// Current state of a [`Permission`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PermissionStatus {
    Granted,
    Denied,
    NotDetermined,
}

/// Scans the radio environment (Bluetooth LE, Wi-Fi) and pushes samples to a channel.
pub trait RadioScanner {
    /// Starts scanning; samples are sent to `sink` until [`RadioScanner::stop`] is called.
    fn start(&mut self, sink: Sender<RadioSample>) -> Result<(), SpiError>;
    /// Stops scanning. Calling it while stopped is a no-op.
    fn stop(&mut self);
    /// Why each transport is or is not producing samples. `None` if the scanner does not report it.
    ///
    /// Take it before handing the scanner to the sample hub; it stays valid across start and stop.
    fn scan_status(&self) -> Option<Arc<dyn ScanStatusSource>> {
        None
    }
}

/// Measures distance to a device that a platform can range directly (for example UWB).
pub trait RangingProvider {
    /// Returns the distance in metres, or `None` if the device cannot be ranged.
    fn distance_m(&self, device_id: &str) -> Option<f64>;
}

/// Reads and requests OS permissions needed for scanning.
pub trait PermissionProvider {
    fn status(&self, permission: Permission) -> PermissionStatus;
    fn request(&mut self, permission: Permission) -> Result<PermissionStatus, SpiError>;
}

/// Monotonic time source. Timestamps are durations since an implementation-defined epoch.
pub trait Clock {
    fn now(&self) -> Duration;
}

/// Byte-oriented persistence for captured data, keyed by stream name.
pub trait Storage {
    fn append(&mut self, key: &str, bytes: &[u8]) -> Result<(), SpiError>;
    fn read_all(&self, key: &str) -> Result<Vec<u8>, SpiError>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    struct FixedClock(Duration);

    impl Clock for FixedClock {
        fn now(&self) -> Duration {
            self.0
        }
    }

    struct OneShotScanner;

    impl RadioScanner for OneShotScanner {
        fn start(&mut self, sink: Sender<RadioSample>) -> Result<(), SpiError> {
            sink.send(RadioSample {
                transport: Transport::Wifi,
                device_id: "aa:bb:cc:dd:ee:ff".into(),
                rssi_dbm: -60,
                observed_at: Duration::from_secs(1),
                details: Default::default(),
            })
            .map_err(|e| SpiError::Other(e.to_string()))
        }

        fn stop(&mut self) {}
    }

    #[test]
    fn traits_are_object_safe() {
        let clock: Box<dyn Clock> = Box::new(FixedClock(Duration::from_secs(5)));
        assert_eq!(clock.now(), Duration::from_secs(5));

        let mut scanner: Box<dyn RadioScanner> = Box::new(OneShotScanner);
        let (tx, rx) = mpsc::channel();
        scanner.start(tx).unwrap();
        let sample = rx.recv().unwrap();
        assert_eq!(sample.rssi_dbm, -60);
        scanner.stop();
    }

    #[test]
    fn spi_error_displays_permission() {
        let err = SpiError::PermissionDenied(Permission::Location);
        assert_eq!(err.to_string(), "permission denied: Location");
    }
}
