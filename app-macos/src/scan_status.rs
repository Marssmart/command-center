//! Scan status strip and the Location permission request.
//!
//! Self-contained on purpose: it wraps any `eframe::App`, reads scan state only through the
//! `spi` traits, and draws one strip along the bottom of the window. Nothing else in the UI needs
//! to know about it.

use eframe::egui::{self, Color32};
use spi::{
    Permission, PermissionProvider, PermissionStatus, ScanProblem, ScanState, ScanStatusSource,
    Transport,
};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// How often permissions are re-read, so the strip follows the user's answer to a prompt.
const REFRESH_INTERVAL: Duration = Duration::from_secs(1);

/// Wraps the app with the status strip. The inner app draws in the space that is left.
pub struct WithScanStatus<A> {
    inner: A,
    status: ScanStatus,
}

impl<A> WithScanStatus<A> {
    pub fn new(
        inner: A,
        source: Option<Arc<dyn ScanStatusSource>>,
        permissions: Box<dyn PermissionProvider>,
    ) -> Self {
        Self {
            inner,
            status: ScanStatus::new(source, permissions),
        }
    }
}

impl<A: eframe::App> eframe::App for WithScanStatus<A> {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        self.status.refresh(Instant::now());
        ui.ctx().request_repaint_after(REFRESH_INTERVAL);
        egui::Panel::bottom("scan-status").show(ui, |ui| {
            for line in self.status.lines() {
                ui.colored_label(line.level.colour(), line.text);
            }
        });
        self.inner.ui(ui, frame);
    }
}

/// Permission state and scan state, turned into the lines the strip shows.
struct ScanStatus {
    source: Option<Arc<dyn ScanStatusSource>>,
    permissions: Box<dyn PermissionProvider>,
    location: PermissionStatus,
    bluetooth: PermissionStatus,
    refreshed: Option<Instant>,
    request_error: Option<String>,
}

impl ScanStatus {
    fn new(source: Option<Arc<dyn ScanStatusSource>>, permissions: Box<dyn PermissionProvider>) -> Self {
        Self {
            source,
            permissions,
            location: PermissionStatus::NotDetermined,
            bluetooth: PermissionStatus::NotDetermined,
            refreshed: None,
            request_error: None,
        }
    }

    /// Re-reads permissions when due. The first call, made from the first UI frame, also asks for
    /// Location: macOS shows that prompt only to a foreground app, from the main thread.
    /// Bluetooth is not asked for here, because macOS asks by itself when the first scan starts.
    fn refresh(&mut self, now: Instant) {
        let first = self.refreshed.is_none();
        if self.refreshed.is_some_and(|at| now.duration_since(at) < REFRESH_INTERVAL) {
            return;
        }
        self.refreshed = Some(now);
        self.location = self.permissions.status(Permission::Location);
        self.bluetooth = self.permissions.status(Permission::Bluetooth);
        if first && self.location == PermissionStatus::NotDetermined {
            if let Err(error) = self.permissions.request(Permission::Location) {
                self.request_error = Some(error.to_string());
            }
        }
    }

    fn lines(&self) -> Vec<Line> {
        let state = |transport| {
            self.source
                .as_ref()
                .map_or(ScanState::Idle, |source| source.scan_state(transport))
        };
        let mut lines = vec![
            describe(Transport::Wifi, &state(Transport::Wifi), self.location),
            describe(Transport::BluetoothLe, &state(Transport::BluetoothLe), self.bluetooth),
        ];
        if let Some(error) = &self.request_error {
            lines.push(Line::problem(format!("Location permission could not be requested: {error}")));
        }
        lines
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    Working,
    Quiet,
    Problem,
}

impl Level {
    fn colour(self) -> Color32 {
        match self {
            Level::Working => Color32::LIGHT_GREEN,
            Level::Quiet => Color32::GRAY,
            Level::Problem => Color32::from_rgb(255, 170, 80),
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
struct Line {
    level: Level,
    text: String,
}

impl Line {
    fn problem(text: String) -> Self {
        Self { level: Level::Problem, text }
    }
}

/// One line for a transport: what it is doing, or why it cannot scan and what to do about it.
///
/// `permission` is the state of the permission the transport depends on. It decides the line
/// while nothing is scanning, so a missing permission is visible before the first scan.
fn describe(transport: Transport, state: &ScanState, permission: PermissionStatus) -> Line {
    let (name, needs, settings) = match transport {
        Transport::Wifi => ("Wi-Fi", Permission::Location, "Location Services"),
        Transport::BluetoothLe => ("Bluetooth", Permission::Bluetooth, "Bluetooth"),
    };
    let problem = match state {
        ScanState::Blocked(problem) => Some(problem.clone()),
        ScanState::Idle if permission == PermissionStatus::Denied => {
            Some(ScanProblem::PermissionDenied(needs))
        }
        // macOS asks for Bluetooth only once a scan starts, so idle Bluetooth is not waiting yet.
        ScanState::Idle
            if permission == PermissionStatus::NotDetermined && transport == Transport::Wifi =>
        {
            Some(ScanProblem::PermissionNotDetermined(needs))
        }
        _ => None,
    };
    let Some(problem) = problem else {
        let (level, text) = match (state, transport) {
            (ScanState::Starting, _) => (Level::Quiet, "starting the first scan".to_string()),
            (ScanState::Active, Transport::Wifi) => (
                Level::Working,
                "scanning. Shows access points only, not the devices joined to them".to_string(),
            ),
            (ScanState::Active, Transport::BluetoothLe) => (Level::Working, "scanning".to_string()),
            _ => (Level::Quiet, "not scanning".to_string()),
        };
        return Line { level, text: format!("{name}: {text}.") };
    };
    let reason = match problem {
        ScanProblem::PermissionNotDetermined(Permission::Location) => format!(
            "waiting for Location permission. macOS hides the addresses of Wi-Fi networks from apps without it. Answer the macOS prompt, or allow Command Center in System Settings > Privacy & Security > {settings}"
        ),
        ScanProblem::PermissionNotDetermined(Permission::Bluetooth) => {
            "waiting for Bluetooth permission. Answer the macOS prompt".to_string()
        }
        ScanProblem::PermissionDenied(Permission::Location) => format!(
            "cannot scan. Location permission is denied, or Location Services is off. macOS hides the addresses of Wi-Fi networks from apps without it. Allow Command Center in System Settings > Privacy & Security > {settings}"
        ),
        ScanProblem::PermissionDenied(Permission::Bluetooth) => format!(
            "cannot scan. Bluetooth permission is denied. Allow Command Center in System Settings > Privacy & Security > {settings}"
        ),
        ScanProblem::RadioOff => format!("cannot scan. {name} is turned off"),
        ScanProblem::NoRadio => format!("cannot scan. This Mac has no {name} radio"),
        ScanProblem::Failed(message) => format!("the last scan failed and will be retried. {message}"),
    };
    Line::problem(format!("{name}: {reason}."))
}

#[cfg(test)]
mod tests {
    use super::*;
    use spi::SpiError;
    use std::cell::Cell;
    use std::rc::Rc;

    struct FakePermissions {
        location: PermissionStatus,
        requests: Rc<Cell<u32>>,
    }

    impl PermissionProvider for FakePermissions {
        fn status(&self, permission: Permission) -> PermissionStatus {
            match permission {
                Permission::Location => self.location,
                Permission::Bluetooth => PermissionStatus::Granted,
            }
        }

        fn request(&mut self, _permission: Permission) -> Result<PermissionStatus, SpiError> {
            self.requests.set(self.requests.get() + 1);
            Ok(self.location)
        }
    }

    fn status(location: PermissionStatus) -> (ScanStatus, Rc<Cell<u32>>) {
        let requests = Rc::new(Cell::new(0));
        let permissions = FakePermissions { location, requests: Rc::clone(&requests) };
        (ScanStatus::new(None, Box::new(permissions)), requests)
    }

    #[test]
    fn location_is_requested_once_and_only_while_undetermined() {
        let start = Instant::now();
        let (mut undetermined, requests) = status(PermissionStatus::NotDetermined);
        undetermined.refresh(start);
        undetermined.refresh(start + REFRESH_INTERVAL * 3);
        assert_eq!(requests.get(), 1);

        for answered in [PermissionStatus::Granted, PermissionStatus::Denied] {
            let (mut decided, requests) = status(answered);
            decided.refresh(start);
            assert_eq!(requests.get(), 0);
        }
    }

    #[test]
    fn missing_location_permission_is_shown_before_any_scan() {
        let (mut waiting, _) = status(PermissionStatus::NotDetermined);
        waiting.refresh(Instant::now());
        let wifi = &waiting.lines()[0];
        assert_eq!(wifi.level, Level::Problem);
        assert!(wifi.text.contains("waiting for Location permission"));

        let denied = describe(Transport::Wifi, &ScanState::Idle, PermissionStatus::Denied);
        assert_eq!(denied.level, Level::Problem);
        assert!(denied.text.contains("Location permission is denied"));
    }

    #[test]
    fn every_blocked_state_says_why() {
        let cases = [
            (ScanProblem::PermissionNotDetermined(Permission::Location), "waiting for Location"),
            (ScanProblem::PermissionDenied(Permission::Location), "Location permission is denied"),
            (ScanProblem::RadioOff, "Wi-Fi is turned off"),
            (ScanProblem::NoRadio, "no Wi-Fi radio"),
            (ScanProblem::Failed("Resource busy".into()), "Resource busy"),
        ];
        for (problem, expected) in cases {
            let line = describe(Transport::Wifi, &ScanState::Blocked(problem), PermissionStatus::Granted);
            assert_eq!(line.level, Level::Problem);
            assert!(line.text.contains(expected), "{} should contain {expected}", line.text);
        }
        let bluetooth = describe(
            Transport::BluetoothLe,
            &ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Bluetooth)),
            PermissionStatus::Denied,
        );
        assert!(bluetooth.text.contains("Bluetooth permission is denied"));
    }

    #[test]
    fn working_wifi_states_the_access_point_limit() {
        let line = describe(Transport::Wifi, &ScanState::Active, PermissionStatus::Granted);
        assert_eq!(line.level, Level::Working);
        assert!(line.text.contains("access points only"));

        let idle = describe(Transport::BluetoothLe, &ScanState::Idle, PermissionStatus::NotDetermined);
        assert_eq!(idle, Line { level: Level::Quiet, text: "Bluetooth: not scanning.".into() });
    }
}
