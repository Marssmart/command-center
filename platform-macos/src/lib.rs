//! macOS implementations of the `spi` traits.
//!
//! - BLE scanning: btleplug (CoreBluetooth) on a dedicated thread with its own tokio runtime.
//! - Wi-Fi scanning: CoreWLAN via objc2, polled on a dedicated thread.
//! - Permissions: CoreLocation and CoreBluetooth authorization via objc2.

use btleplug::api::{Central, CentralEvent, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::{Adapter, Manager, PeripheralId};
use futures::StreamExt;
use objc2::rc::Retained;
use objc2_core_bluetooth::{CBCentralManager, CBManager, CBManagerAuthorization};
use objc2_core_location::{CLAuthorizationStatus, CLLocationManager};
use objc2_core_wlan::CWWiFiClient;
use spi::{
    Clock, Permission, PermissionProvider, PermissionStatus, RadioSample, RadioScanner,
    RangingProvider, SpiError, Storage, Transport,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const WIFI_SCAN_INTERVAL: Duration = Duration::from_secs(5);
const BLE_POLL_INTERVAL: Duration = Duration::from_millis(250);

/// Entry point for the macOS platform implementations. Not `Send`, because it owns Objective-C managers.
pub struct MacosPlatform {
    epoch: Instant,
    location: Option<Retained<CLLocationManager>>,
    central: Option<Retained<CBCentralManager>>,
}

/// Radio scanner that can move to the sample hub. It holds only thread handles.
pub struct MacosScanner {
    epoch: Instant,
    scan: Option<ActiveScan>,
}

struct ActiveScan {
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl MacosPlatform {
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
            location: None,
            central: None,
        }
    }

    /// A scanner that stamps samples on this platform's clock.
    pub fn scanner(&self) -> MacosScanner {
        MacosScanner {
            epoch: self.epoch,
            scan: None,
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

impl RadioScanner for MacosScanner {
    fn start(&mut self, sink: Sender<RadioSample>) -> Result<(), SpiError> {
        if self.scan.is_some() {
            return Ok(());
        }
        let stop = Arc::new(AtomicBool::new(false));
        let ble = spawn_ble_scan(sink.clone(), Arc::clone(&stop), self.epoch)?;
        let wifi = spawn_wifi_scan(sink, Arc::clone(&stop), self.epoch);
        self.scan = Some(ActiveScan {
            stop,
            threads: vec![ble, wifi],
        });
        Ok(())
    }

    fn stop(&mut self) {
        if let Some(scan) = self.scan.take() {
            scan.stop.store(true, Ordering::Relaxed);
            for handle in scan.threads {
                let _ = handle.join();
            }
        }
    }
}

impl RangingProvider for MacosPlatform {
    fn distance_m(&self, _device_id: &str) -> Option<f64> {
        None
    }
}

impl PermissionProvider for MacosPlatform {
    fn status(&self, permission: Permission) -> PermissionStatus {
        match permission {
            Permission::Location => location_status(),
            Permission::Bluetooth => bluetooth_status(),
        }
    }

    fn request(&mut self, permission: Permission) -> Result<PermissionStatus, SpiError> {
        // The OS shows the prompt only while the manager that triggered it is alive,
        // so keep it in the platform struct.
        match permission {
            Permission::Location => {
                let manager = self.location.get_or_insert_with(|| unsafe { CLLocationManager::new() });
                unsafe { manager.requestWhenInUseAuthorization() };
            }
            Permission::Bluetooth => {
                self.central.get_or_insert_with(|| unsafe { CBCentralManager::new() });
            }
        }
        Ok(self.status(permission))
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

/// Starts BLE scanning on its own thread. Returns once the adapter is scanning, or the setup error.
fn spawn_ble_scan(
    sink: Sender<RadioSample>,
    stop: Arc<AtomicBool>,
    epoch: Instant,
) -> Result<JoinHandle<()>, SpiError> {
    let (ready_tx, ready_rx) = mpsc::channel::<Result<(), String>>();
    let handle = thread::spawn(move || {
        let runtime = match tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        {
            Ok(rt) => rt,
            Err(e) => {
                let _ = ready_tx.send(Err(e.to_string()));
                return;
            }
        };
        runtime.block_on(async move {
            let adapter = match first_adapter().await {
                Ok(adapter) => adapter,
                Err(e) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
            };
            let setup = async {
                adapter
                    .start_scan(ScanFilter::default())
                    .await
                    .map_err(|e| e.to_string())?;
                adapter.events().await.map_err(|e| e.to_string())
            }
            .await;
            let mut events = match setup {
                Ok(events) => events,
                Err(e) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
            };
            let _ = ready_tx.send(Ok(()));

            while !stop.load(Ordering::Relaxed) {
                match tokio::time::timeout(BLE_POLL_INTERVAL, events.next()).await {
                    Ok(Some(
                        CentralEvent::DeviceDiscovered(id) | CentralEvent::DeviceUpdated(id),
                    )) => {
                        if let Some(sample) = ble_sample(&adapter, &id, epoch).await {
                            if sink.send(sample).is_err() {
                                break;
                            }
                        }
                    }
                    Ok(None) => break,
                    _ => {}
                }
            }
            let _ = adapter.stop_scan().await;
        });
    });

    match ready_rx.recv() {
        Ok(Ok(())) => Ok(handle),
        Ok(Err(msg)) => Err(SpiError::Other(format!("BLE scan failed: {msg}"))),
        Err(_) => Err(SpiError::Other("BLE scan thread exited during setup".into())),
    }
}

async fn first_adapter() -> Result<Adapter, String> {
    let manager = Manager::new().await.map_err(|e| e.to_string())?;
    manager
        .adapters()
        .await
        .map_err(|e| e.to_string())?
        .into_iter()
        .next()
        .ok_or_else(|| "no Bluetooth adapter found".to_string())
}

async fn ble_sample(adapter: &Adapter, id: &PeripheralId, epoch: Instant) -> Option<RadioSample> {
    let peripheral = adapter.peripheral(id).await.ok()?;
    let rssi = peripheral.properties().await.ok()??.rssi?;
    Some(RadioSample {
        transport: Transport::BluetoothLe,
        device_id: id.to_string(),
        rssi_dbm: rssi,
        observed_at: epoch.elapsed(),
    })
}

/// Polls CoreWLAN on its own thread until `stop` is set.
fn spawn_wifi_scan(sink: Sender<RadioSample>, stop: Arc<AtomicBool>, epoch: Instant) -> JoinHandle<()> {
    thread::spawn(move || {
        while !stop.load(Ordering::Relaxed) {
            for sample in scan_wifi_once(epoch) {
                if sink.send(sample).is_err() {
                    return;
                }
            }
            let mut waited = Duration::ZERO;
            while waited < WIFI_SCAN_INTERVAL && !stop.load(Ordering::Relaxed) {
                thread::sleep(BLE_POLL_INTERVAL);
                waited += BLE_POLL_INTERVAL;
            }
        }
    })
}

fn scan_wifi_once(epoch: Instant) -> Vec<RadioSample> {
    let client = unsafe { CWWiFiClient::sharedWiFiClient() };
    let Some(interface) = (unsafe { client.interface() }) else {
        return Vec::new();
    };
    let Ok(networks) = (unsafe { interface.scanForNetworksWithName_error(None) }) else {
        return Vec::new();
    };
    networks
        .iter()
        .filter_map(|network| {
            let bssid = unsafe { network.bssid() }.map(|b| b.to_string());
            let rssi = unsafe { network.rssiValue() };
            wifi_sample(bssid.as_deref(), rssi, epoch.elapsed())
        })
        .collect()
}

/// Builds a Wi-Fi sample; networks without a BSSID (no location permission) are skipped.
fn wifi_sample(bssid: Option<&str>, rssi: isize, observed_at: Duration) -> Option<RadioSample> {
    let device_id = bssid?.to_string();
    Some(RadioSample {
        transport: Transport::Wifi,
        device_id,
        rssi_dbm: rssi.clamp(i16::MIN as isize, i16::MAX as isize) as i16,
        observed_at,
    })
}

fn location_status() -> PermissionStatus {
    let manager = unsafe { CLLocationManager::new() };
    location_permission(unsafe { manager.authorizationStatus() })
}

fn location_permission(raw: CLAuthorizationStatus) -> PermissionStatus {
    if raw == CLAuthorizationStatus::NotDetermined {
        PermissionStatus::NotDetermined
    } else if raw == CLAuthorizationStatus::AuthorizedAlways
        || raw == CLAuthorizationStatus::AuthorizedWhenInUse
    {
        PermissionStatus::Granted
    } else {
        PermissionStatus::Denied
    }
}

fn bluetooth_status() -> PermissionStatus {
    let raw = unsafe { CBManager::authorization_class() };
    bluetooth_permission(raw)
}

fn bluetooth_permission(raw: CBManagerAuthorization) -> PermissionStatus {
    if raw == CBManagerAuthorization::NotDetermined {
        PermissionStatus::NotDetermined
    } else if raw == CBManagerAuthorization::AllowedAlways {
        PermissionStatus::Granted
    } else {
        PermissionStatus::Denied
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
    fn wifi_sample_skips_networks_without_bssid() {
        assert!(wifi_sample(None, -50, Duration::ZERO).is_none());
    }

    #[test]
    fn wifi_sample_keeps_bssid_and_clamps_rssi() {
        let sample = wifi_sample(Some("aa:bb:cc:dd:ee:ff"), -100_000, Duration::from_secs(2)).unwrap();
        assert_eq!(sample.transport, Transport::Wifi);
        assert_eq!(sample.device_id, "aa:bb:cc:dd:ee:ff");
        assert_eq!(sample.rssi_dbm, i16::MIN);
        assert_eq!(sample.observed_at, Duration::from_secs(2));
    }

    #[test]
    fn location_authorization_maps_to_permission_status() {
        assert_eq!(location_permission(CLAuthorizationStatus::NotDetermined), PermissionStatus::NotDetermined);
        assert_eq!(location_permission(CLAuthorizationStatus::AuthorizedAlways), PermissionStatus::Granted);
        assert_eq!(location_permission(CLAuthorizationStatus::Denied), PermissionStatus::Denied);
    }

    #[test]
    fn bluetooth_authorization_maps_to_permission_status() {
        assert_eq!(bluetooth_permission(CBManagerAuthorization::AllowedAlways), PermissionStatus::Granted);
        assert_eq!(bluetooth_permission(CBManagerAuthorization::Denied), PermissionStatus::Denied);
    }
}
