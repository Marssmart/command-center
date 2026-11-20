//! macOS implementations of the `spi` traits. Placeholder until CoreBluetooth and CoreWLAN are wired in.

use spi::{Clock, PermissionProvider, RadioScanner, RangingProvider, Storage};

/// Entry point for the macOS platform implementations.
pub struct MacosPlatform;

impl RadioScanner for MacosPlatform {}
impl RangingProvider for MacosPlatform {}
impl PermissionProvider for MacosPlatform {}
impl Clock for MacosPlatform {}
impl Storage for MacosPlatform {}
