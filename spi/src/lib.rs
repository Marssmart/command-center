//! Service provider interface: the traits `core` depends on and platform crates implement.

/// Scans the radio environment (Bluetooth LE, Wi-Fi).
pub trait RadioScanner {}

/// Measures distance to nearby devices.
pub trait RangingProvider {}

/// Checks and requests OS permissions required for scanning.
pub trait PermissionProvider {}

/// Provides the current time.
pub trait Clock {}

/// Persists collected data.
pub trait Storage {}
