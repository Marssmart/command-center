//! macOS implementations of the `spi` traits.
//!
//! - BLE scanning: btleplug (CoreBluetooth) on a dedicated thread with its own tokio runtime.
//! - Wi-Fi scanning: CoreWLAN via objc2, on dedicated threads. See [`wifi`].
//! - Permissions: CoreLocation and CoreBluetooth authorization via objc2.

mod wifi;

use btleplug::api::{
    Central, CentralEvent, CentralState, Manager as _, Peripheral as _, ScanFilter,
};
use btleplug::platform::{Adapter, Manager, PeripheralId};
use futures::StreamExt;
use objc2::rc::Retained;
use objc2::MainThreadMarker;
use objc2_core_bluetooth::{CBCentralManager, CBManager, CBManagerAuthorization};
use objc2_core_location::{CLAuthorizationStatus, CLLocationManager};
use spi::{
    Clock, Permission, PermissionProvider, PermissionStatus, RadioSample, RadioScanner,
    RangingProvider, SampleDetails, ScanProblem, ScanState, ScanStatusBoard, ScanStatusSource,
    SpiError, Storage, Transport,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

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
    status: Arc<ScanStatusBoard>,
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
            status: Arc::new(ScanStatusBoard::new()),
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
        // Not awaited: CoreBluetooth delivers its state on the main thread, so blocking here deadlocks the UI.
        let ble = spawn_ble_scan(
            sink.clone(),
            Arc::clone(&stop),
            self.epoch,
            Arc::clone(&self.status),
        );
        let wifi = wifi::spawn_wifi_scan(sink, Arc::clone(&stop), self.epoch, Arc::clone(&self.status));
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
            self.status.set(Transport::BluetoothLe, ScanState::Idle);
            self.status.set(Transport::Wifi, ScanState::Idle);
        }
    }

    fn scan_status(&self) -> Option<Arc<dyn ScanStatusSource>> {
        Some(Arc::clone(&self.status) as Arc<dyn ScanStatusSource>)
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

    /// Asks the OS to show its permission prompt. The answer arrives later: read it with `status`.
    ///
    /// Call it on the main thread while the app is in the foreground, and keep the platform alive.
    /// macOS shows the prompt only for a foreground app, drives it from the run loop of the thread
    /// that created the manager, and drops it if that manager is released.
    fn request(&mut self, permission: Permission) -> Result<PermissionStatus, SpiError> {
        if MainThreadMarker::new().is_none() {
            return Err(SpiError::Other(
                "permissions must be requested on the main thread".into(),
            ));
        }
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

/// Scans BLE on its own thread until `stop` is set, publishing its state to `board`.
fn spawn_ble_scan(
    sink: Sender<RadioSample>,
    stop: Arc<AtomicBool>,
    epoch: Instant,
    board: Arc<ScanStatusBoard>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        // Bluetooth permission is shown here too: creating the adapter is what makes macOS ask.
        board.set(
            Transport::BluetoothLe,
            ble_state(bluetooth_status(), CentralState::Unknown),
        );
        let outcome = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| e.to_string())
            .and_then(|runtime| runtime.block_on(run_ble_scan(sink, &stop, epoch, &board)));
        if let Err(error) = outcome {
            let problem = ble_setup_problem(bluetooth_status(), error);
            board.set(Transport::BluetoothLe, ScanState::Blocked(problem));
        }
    })
}

/// Runs until `stop` is set or the event stream ends. An error means scanning never started.
async fn run_ble_scan(
    sink: Sender<RadioSample>,
    stop: &AtomicBool,
    epoch: Instant,
    board: &ScanStatusBoard,
) -> Result<(), String> {
    let adapter = first_adapter().await?;
    let mut events = adapter.events().await.map_err(|e| e.to_string())?;
    adapter
        .start_scan(ScanFilter::default())
        .await
        .map_err(|e| e.to_string())?;
    let central = adapter
        .adapter_state()
        .await
        .unwrap_or(CentralState::Unknown);
    board.set(Transport::BluetoothLe, ble_state(bluetooth_status(), central));

    let mut identities: HashMap<PeripheralId, String> = HashMap::new();
    while !stop.load(Ordering::Relaxed) {
        match tokio::time::timeout(BLE_POLL_INTERVAL, events.next()).await {
            Ok(Some(CentralEvent::DeviceDiscovered(id) | CentralEvent::DeviceUpdated(id))) => {
                if let Some(sample) = ble_sample(&adapter, &id, epoch, &mut identities).await {
                    if sink.send(sample).is_err() {
                        break;
                    }
                }
            }
            Ok(Some(CentralEvent::StateUpdate(central))) => {
                // CoreBluetooth ignores a scan request made before it is powered on, which is the
                // case while the permission prompt is open or Bluetooth is off. Ask again.
                if central == CentralState::PoweredOn {
                    let _ = adapter.start_scan(ScanFilter::default()).await;
                }
                board.set(Transport::BluetoothLe, ble_state(bluetooth_status(), central));
            }
            Ok(None) => break,
            _ => {}
        }
    }
    let _ = adapter.stop_scan().await;
    Ok(())
}

/// The BLE scan state for a permission and radio state.
fn ble_state(permission: PermissionStatus, central: CentralState) -> ScanState {
    match (permission, central) {
        (PermissionStatus::Denied, _) => {
            ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Bluetooth))
        }
        (PermissionStatus::NotDetermined, _) => {
            ScanState::Blocked(ScanProblem::PermissionNotDetermined(Permission::Bluetooth))
        }
        (PermissionStatus::Granted, CentralState::PoweredOff) => {
            ScanState::Blocked(ScanProblem::RadioOff)
        }
        (PermissionStatus::Granted, CentralState::PoweredOn) => ScanState::Active,
        (PermissionStatus::Granted, CentralState::Unknown) => ScanState::Starting,
    }
}

/// Why BLE scanning could not be set up. A refused permission explains the error better than its text.
fn ble_setup_problem(permission: PermissionStatus, error: String) -> ScanProblem {
    match permission {
        PermissionStatus::Denied => ScanProblem::PermissionDenied(Permission::Bluetooth),
        _ if error == NO_BLUETOOTH_ADAPTER => ScanProblem::NoRadio,
        _ => ScanProblem::Failed(error),
    }
}

const NO_BLUETOOTH_ADAPTER: &str = "no Bluetooth adapter found";

async fn first_adapter() -> Result<Adapter, String> {
    let manager = Manager::new().await.map_err(|e| e.to_string())?;
    manager
        .adapters()
        .await
        .map_err(|e| e.to_string())?
        .into_iter()
        .next()
        .ok_or_else(|| NO_BLUETOOTH_ADAPTER.to_string())
}

async fn ble_sample(
    adapter: &Adapter,
    id: &PeripheralId,
    epoch: Instant,
    identities: &mut HashMap<PeripheralId, String>,
) -> Option<RadioSample> {
    let peripheral = adapter.peripheral(id).await.ok()?;
    let props = peripheral.properties().await.ok()??;
    let manufacturer: Vec<(u16, &[u8])> = props
        .manufacturer_data
        .iter()
        .map(|(company, data)| (*company, data.as_slice()))
        .collect();
    let services: Vec<u128> = props.services.iter().map(|uuid| uuid.as_u128()).collect();
    let mut manufacturer_ids: Vec<u16> = manufacturer.iter().map(|(company, _)| *company).collect();
    manufacturer_ids.sort_unstable();
    let details = SampleDetails {
        manufacturer_ids,
        service_uuids: services.clone(),
        payload_hash: spi::ble_payload_hash(manufacturer.iter().copied(), services.iter().copied()),
        ..Default::default()
    };

    let device_id = match identities.get(id) {
        Some(known) => known.clone(),
        None => {
            // Fixed on first sight: a later advertisement that adds data must not change the id.
            // Peripherals with no advertised data yet are skipped until some arrives.
            let fingerprint = spi::ble_fingerprint(
                manufacturer.iter().copied(),
                services.iter().copied(),
            )?;
            identities.insert(id.clone(), fingerprint.clone());
            fingerprint
        }
    };
    Some(RadioSample {
        transport: Transport::BluetoothLe,
        device_id,
        rssi_dbm: props.rssi?,
        observed_at: epoch.elapsed(),
        details,
    })
}

pub(crate) fn location_status() -> PermissionStatus {
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

/// Compass heading from CoreLocation. No heading delegate is wired yet, so it reports `None`
/// rather than guessing. `heading_available` tells the UI whether the Mac has a compass at all.
pub struct CoreLocationHeading;

impl CoreLocationHeading {
    pub fn heading_available() -> bool {
        unsafe { CLLocationManager::headingAvailable_class() }
    }
}

impl spi::HeadingSource for CoreLocationHeading {
    fn heading_deg(&self) -> Option<f64> {
        None
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
    fn scanner_reports_idle_until_started() {
        let scanner = MacosPlatform::new().scanner();
        let status = scanner.scan_status().expect("the macOS scanner reports its status");
        assert_eq!(status.scan_state(Transport::Wifi), ScanState::Idle);
        assert_eq!(status.scan_state(Transport::BluetoothLe), ScanState::Idle);
    }

    #[test]
    fn permission_request_off_the_main_thread_is_refused() {
        let outcome = thread::spawn(|| MacosPlatform::new().request(Permission::Location))
            .join()
            .unwrap();
        assert!(matches!(outcome, Err(SpiError::Other(_))));
    }

    #[test]
    fn location_authorization_maps_to_permission_status() {
        assert_eq!(location_permission(CLAuthorizationStatus::NotDetermined), PermissionStatus::NotDetermined);
        assert_eq!(location_permission(CLAuthorizationStatus::AuthorizedAlways), PermissionStatus::Granted);
        assert_eq!(location_permission(CLAuthorizationStatus::AuthorizedWhenInUse), PermissionStatus::Granted);
        assert_eq!(location_permission(CLAuthorizationStatus::Denied), PermissionStatus::Denied);
        assert_eq!(location_permission(CLAuthorizationStatus::Restricted), PermissionStatus::Denied);
    }

    #[test]
    fn bluetooth_authorization_maps_to_permission_status() {
        assert_eq!(bluetooth_permission(CBManagerAuthorization::AllowedAlways), PermissionStatus::Granted);
        assert_eq!(bluetooth_permission(CBManagerAuthorization::Denied), PermissionStatus::Denied);
    }

    #[test]
    fn ble_state_names_the_missing_permission_before_the_radio() {
        assert_eq!(
            ble_state(PermissionStatus::Denied, CentralState::PoweredOn),
            ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Bluetooth))
        );
        assert_eq!(
            ble_state(PermissionStatus::NotDetermined, CentralState::Unknown),
            ScanState::Blocked(ScanProblem::PermissionNotDetermined(Permission::Bluetooth))
        );
        assert_eq!(
            ble_state(PermissionStatus::Granted, CentralState::PoweredOff),
            ScanState::Blocked(ScanProblem::RadioOff)
        );
        assert_eq!(ble_state(PermissionStatus::Granted, CentralState::PoweredOn), ScanState::Active);
        assert_eq!(ble_state(PermissionStatus::Granted, CentralState::Unknown), ScanState::Starting);
    }

    #[test]
    fn ble_setup_failure_keeps_its_reason() {
        assert_eq!(
            ble_setup_problem(PermissionStatus::Denied, "anything".into()),
            ScanProblem::PermissionDenied(Permission::Bluetooth)
        );
        assert_eq!(
            ble_setup_problem(PermissionStatus::Granted, NO_BLUETOOTH_ADAPTER.into()),
            ScanProblem::NoRadio
        );
        assert_eq!(
            ble_setup_problem(PermissionStatus::Granted, "adapter failed".into()),
            ScanProblem::Failed("adapter failed".into())
        );
    }
}
