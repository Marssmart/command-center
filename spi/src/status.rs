//! Scan status: whether each transport is producing samples, and if not, why.
//!
//! A scanner that finds nothing and a scanner that cannot scan look the same in the sample
//! stream. This is the side channel that tells them apart, so the UI never shows an empty list
//! without a reason.

use crate::{Permission, Transport};
use std::sync::Mutex;

/// Why a transport cannot scan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScanProblem {
    /// The user has not answered the OS permission prompt yet, or it has not been shown.
    PermissionNotDetermined(Permission),
    /// The user or a policy refused the permission.
    PermissionDenied(Permission),
    /// The radio is switched off.
    RadioOff,
    /// The machine has no radio for this transport.
    NoRadio,
    /// The scan failed. The message comes from the platform.
    Failed(String),
}

/// What one transport's scanner is doing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum ScanState {
    /// Not scanning, because nothing is subscribed.
    #[default]
    Idle,
    /// Scanning was requested and no scan has finished yet.
    Starting,
    /// Scans complete and their results are sent as samples. A scan may still find nothing.
    Active,
    /// Scanning was requested and cannot work.
    Blocked(ScanProblem),
}

/// Reports the scan state of each transport. Screens read this; they never ask the radio.
pub trait ScanStatusSource: Send + Sync {
    fn scan_state(&self, transport: Transport) -> ScanState;
}

/// Shared scan state that a scanner writes from its threads and the UI reads.
#[derive(Debug, Default)]
pub struct ScanStatusBoard {
    bluetooth: Mutex<ScanState>,
    wifi: Mutex<ScanState>,
}

impl ScanStatusBoard {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&self, transport: Transport, state: ScanState) {
        *self.slot(transport).lock().expect("scan status lock poisoned") = state;
    }

    fn slot(&self, transport: Transport) -> &Mutex<ScanState> {
        match transport {
            Transport::BluetoothLe => &self.bluetooth,
            Transport::Wifi => &self.wifi,
        }
    }
}

impl ScanStatusSource for ScanStatusBoard {
    fn scan_state(&self, transport: Transport) -> ScanState {
        self.slot(transport).lock().expect("scan status lock poisoned").clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_starts_idle_and_keeps_transports_apart() {
        let board = ScanStatusBoard::new();
        assert_eq!(board.scan_state(Transport::Wifi), ScanState::Idle);

        let denied = ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Location));
        board.set(Transport::Wifi, denied.clone());
        board.set(Transport::BluetoothLe, ScanState::Active);
        assert_eq!(board.scan_state(Transport::Wifi), denied);
        assert_eq!(board.scan_state(Transport::BluetoothLe), ScanState::Active);
    }
}
