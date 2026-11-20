//! macOS implementations of the `spi` traits. Radio and permission support is still stubbed.

use spi::{
    Clock, Permission, PermissionProvider, PermissionStatus, RadioSample, RadioScanner,
    RangingProvider, SpiError, Storage,
};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

/// Entry point for the macOS platform implementations.
pub struct MacosPlatform {
    epoch: Instant,
}

impl MacosPlatform {
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
        }
    }
}

impl Default for MacosPlatform {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for MacosPlatform {
    fn now(&self) -> Duration {
        self.epoch.elapsed()
    }
}

impl RadioScanner for MacosPlatform {
    fn start(&mut self, _sink: Sender<RadioSample>) -> Result<(), SpiError> {
        Err(SpiError::Unsupported("radio scanning not implemented yet".into()))
    }

    fn stop(&mut self) {}
}

impl RangingProvider for MacosPlatform {
    fn distance_m(&self, _device_id: &str) -> Option<f64> {
        None
    }
}

impl PermissionProvider for MacosPlatform {
    fn status(&self, _permission: Permission) -> PermissionStatus {
        PermissionStatus::NotDetermined
    }

    fn request(&mut self, permission: Permission) -> Result<PermissionStatus, SpiError> {
        Err(SpiError::Unsupported(format!("requesting {permission:?} not implemented yet")))
    }
}

impl Storage for MacosPlatform {
    fn append(&mut self, _key: &str, _bytes: &[u8]) -> Result<(), SpiError> {
        Err(SpiError::Unsupported("storage not implemented yet".into()))
    }

    fn read_all(&self, _key: &str) -> Result<Vec<u8>, SpiError> {
        Err(SpiError::Unsupported("storage not implemented yet".into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_is_monotonic() {
        let platform = MacosPlatform::new();
        let first = platform.now();
        assert!(platform.now() >= first);
    }

    #[test]
    fn radio_scanning_reports_unsupported() {
        let mut platform = MacosPlatform::new();
        let (tx, _rx) = std::sync::mpsc::channel();
        assert!(matches!(platform.start(tx), Err(SpiError::Unsupported(_))));
    }
}
