use core::{
    run_pipeline, Algorithm, BearingEstimator, DeviceState, Fusion, FramePipe, Playback, Pipeline,
    RssiRanging, SampleHub, ScreenFrame,
};
use crate::view3d::{self, Orbit};
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Stroke};
use std::cmp::Reverse;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::broadcast;
use tokio::sync::broadcast::error::TryRecvError;
use tokio::task::JoinHandle;

const FRAME_CAPACITY: usize = 4096;
const HISTORY_FRAMES: usize = 600;
const PIXELS_PER_METRE: f32 = 20.0;
const RING_RADII_M: [f32; 4] = [5.0, 10.0, 15.0, 20.0];
const REPAINT_INTERVAL: Duration = Duration::from_millis(100);

/// Top-level screens of the app.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    #[default]
    Overview,
    Devices,
    Map,
    View3d,
    Signals,
}

impl Tab {
    pub const ALL: [Tab; 5] = [Tab::Overview, Tab::Devices, Tab::Map, Tab::View3d, Tab::Signals];

    pub fn title(self) -> &'static str {
        match self {
            Tab::Overview => "Overview",
            Tab::Devices => "Devices",
            Tab::Map => "Map",
            Tab::View3d => "3D",
            Tab::Signals => "Signals",
        }
    }
}

/// Tabbed app. Play starts the scanner and the algorithm pipeline. Pause stops both.
pub struct ShellApp {
    tab: Tab,
    runtime: tokio::runtime::Runtime,
    hub: SampleHub,
    pipe: FramePipe,
    frames: broadcast::Receiver<ScreenFrame>,
    /// Shared across pause and resume, so the device table is kept.
    pipeline: Arc<Mutex<Pipeline>>,
    task: Option<JoinHandle<()>>,
    playback: Playback,
    orbit: Orbit,
    status: Option<String>,
}

impl ShellApp {
    pub fn new(hub: SampleHub, runtime: tokio::runtime::Runtime) -> Self {
        let pipe = FramePipe::new(FRAME_CAPACITY);
        let frames = pipe.subscribe();
        Self {
            tab: Tab::default(),
            runtime,
            hub,
            pipe,
            frames,
            pipeline: Arc::new(Mutex::new(default_pipeline())),
            task: None,
            playback: Playback::new(HISTORY_FRAMES),
            orbit: Orbit::default(),
            status: None,
        }
    }

    pub fn tab(&self) -> Tab {
        self.tab
    }

    pub fn is_running(&self) -> bool {
        self.task.is_some()
    }

    pub fn toggle_running(&mut self) {
        if self.is_running() {
            self.pause();
        } else {
            self.start();
        }
    }

    fn start(&mut self) {
        match self.hub.subscribe() {
            Ok(samples) => {
                let task = self.runtime.spawn(run_pipeline(
                    samples,
                    Arc::clone(&self.pipeline),
                    self.pipe.clone(),
                ));
                self.task = Some(task);
                self.status = None;
            }
            Err(err) => self.status = Some(err.to_string()),
        }
    }

    /// Aborting drops the pipeline's subscription, which stops the scanner once no one listens.
    fn pause(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }

    fn drain_frames(&mut self) {
        loop {
            match self.frames.try_recv() {
                Ok(frame) => self.playback.push(frame),
                Err(TryRecvError::Lagged(_)) => continue,
                Err(TryRecvError::Empty | TryRecvError::Closed) => break,
            }
        }
    }
}

fn default_pipeline() -> Pipeline {
    let algorithms: Vec<Box<dyn Algorithm>> = vec![
        Box::new(RssiRanging::default()),
        // macOS has no heading source yet, so bearings stay unset and positions are not produced.
        Box::new(BearingEstimator::new(|| None, 64)),
        Box::new(Fusion::default()),
    ];
    Pipeline::new(algorithms)
}

impl eframe::App for ShellApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_frames();
        if self.is_running() {
            ui.ctx().request_repaint_after(REPAINT_INTERVAL);
        }

        egui::Panel::top("tabs").show(ui, |ui| {
            ui.horizontal(|ui| {
                for tab in Tab::ALL {
                    if ui.selectable_label(self.tab() == tab, tab.title()).clicked() {
                        self.tab = tab;
                    }
                }
                ui.separator();
                let label = if self.is_running() { "⏸ Pause" } else { "▶ Play" };
                if ui.button(label).clicked() {
                    self.toggle_running();
                }
            });
        });

        egui::CentralPanel::default().show(ui, |ui| {
            if let Some(status) = &self.status {
                ui.colored_label(Color32::LIGHT_RED, status);
            }
            let frame = self.playback.current();
            match self.tab() {
                Tab::Overview => overview(ui, self.is_running(), frame),
                Tab::Devices => devices(ui, frame),
                Tab::Map => map(ui, frame),
            Tab::View3d => {
                device_list(ui, frame);
                view3d::show(ui, &mut self.orbit, frame);
            }
                Tab::Signals => signals(ui, frame),
            }
        });
    }
}

fn overview(ui: &mut egui::Ui, running: bool, frame: Option<&ScreenFrame>) {
    ui.heading("Overview");
    ui.label(if running { "Scanning" } else { "Paused" });
    ui.label(format!("Devices seen: {}", frame.map_or(0, |f| f.devices.len())));
}

fn devices(ui: &mut egui::Ui, frame: Option<&ScreenFrame>) {
    ui.heading("Devices");
    let Some(frame) = frame else {
        ui.label("No data yet.");
        return;
    };
    for device in sorted_by_signal(frame) {
        ui.label(device_line(device));
    }
}

fn signals(ui: &mut egui::Ui, frame: Option<&ScreenFrame>) {
    ui.heading("Signals");
    let Some(frame) = frame else {
        ui.label("No data yet.");
        return;
    };
    for device in sorted_by_signal(frame) {
        ui.label(format!("{}  {} dBm", device.device_id, device.last_rssi_dbm));
        // Map -100 dBm to 0 and -30 dBm to 1 for the bar.
        let strength = (f32::from(device.last_rssi_dbm) + 100.0) / 70.0;
        ui.add(egui::ProgressBar::new(strength.clamp(0.0, 1.0)));
    }
}

fn map(ui: &mut egui::Ui, frame: Option<&ScreenFrame>) {
    device_list(ui, frame);
    let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
    let origin = response.rect.center();
    let ring = Stroke::new(1.0, Color32::DARK_GRAY);
    for radius_m in RING_RADII_M {
        painter.circle_stroke(origin, radius_m * PIXELS_PER_METRE, ring);
    }
    painter.circle_filled(origin, 4.0, Color32::WHITE);

    let Some(frame) = frame else {
        return;
    };
    // Distance without bearing: a ring with no angle, so it implies no direction.
    let distance_ring = Stroke::new(1.5, Color32::LIGHT_GREEN);
    for device in unplaced_with_distance(frame) {
        if let Some(distance) = device.distance_m {
            painter.circle_stroke(origin, distance as f32 * PIXELS_PER_METRE, distance_ring);
        }
    }
    for device in &frame.devices {
        let Some((east, north)) = device.position_m else {
            continue;
        };
        let point = to_screen(origin, PIXELS_PER_METRE, east, north);
        painter.circle_filled(point, 5.0, Color32::LIGHT_BLUE);
        painter.text(
            point + egui::vec2(8.0, 0.0),
            Align2::LEFT_CENTER,
            &device.device_id,
            FontId::proportional(12.0),
            Color32::WHITE,
        );
    }
}

/// Every detected device with its placement status. Devices without a bearing are listed here
/// because they cannot be drawn at a direction.
fn device_list(ui: &mut egui::Ui, frame: Option<&ScreenFrame>) {
    let Some(frame) = frame else {
        ui.label("No data yet.");
        return;
    };
    let placed = frame.devices.iter().filter(|d| d.position_m.is_some()).count();
    ui.label(format!(
        "{} devices, {} placed on the map",
        frame.devices.len(),
        placed
    ));
    egui::ScrollArea::vertical()
        .id_salt("device-list")
        .max_height(120.0)
        .show(ui, |ui| {
            for device in sorted_by_signal(frame) {
                ui.label(format!("{}  ·  {}", device_line(device), placement_note(device)));
            }
        });
}

fn placement_note(device: &DeviceState) -> &'static str {
    if device.position_m.is_some() {
        "placed"
    } else if device.distance_m.is_some() {
        "distance only (no bearing)"
    } else {
        "signal only"
    }
}

fn unplaced_with_distance(frame: &ScreenFrame) -> impl Iterator<Item = &DeviceState> {
    frame
        .devices
        .iter()
        .filter(|d| d.position_m.is_none() && d.distance_m.is_some())
}

/// Maps an east/north offset in metres to screen pixels. North is up, so y decreases.
fn to_screen(origin: Pos2, pixels_per_metre: f32, east_m: f64, north_m: f64) -> Pos2 {
    Pos2::new(
        origin.x + east_m as f32 * pixels_per_metre,
        origin.y - north_m as f32 * pixels_per_metre,
    )
}

/// Devices strongest first.
fn sorted_by_signal(frame: &ScreenFrame) -> Vec<&DeviceState> {
    let mut devices: Vec<&DeviceState> = frame.devices.iter().collect();
    devices.sort_by_key(|d| Reverse(d.last_rssi_dbm));
    devices
}

fn device_line(device: &DeviceState) -> String {
    let distance = device
        .distance_m
        .map_or("distance unknown".to_string(), |d| format!("{d:.1} m"));
    format!("{}  {} dBm  {}", device.device_id, device.last_rssi_dbm, distance)
}

#[cfg(test)]
mod tests {
    use super::*;
    use spi::{RadioSample, RadioScanner, SpiError};
    use std::sync::mpsc;

    /// Accepts a start and emits nothing, so the toggle can run without hardware.
    struct IdleScanner;

    impl RadioScanner for IdleScanner {
        fn start(&mut self, _sink: mpsc::Sender<RadioSample>) -> Result<(), SpiError> {
            Ok(())
        }

        fn stop(&mut self) {}
    }

    fn test_app() -> ShellApp {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        ShellApp::new(SampleHub::new(Box::new(IdleScanner), 8), runtime)
    }

    #[test]
    fn starts_on_overview_paused() {
        let app = test_app();
        assert_eq!(app.tab(), Tab::Overview);
        assert!(!app.is_running());
    }

    #[test]
    fn device_table_survives_pause_and_resume() {
        let sample = |id: &str| RadioSample {
            transport: spi::Transport::Wifi,
            device_id: id.into(),
            rssi_dbm: -50,
            observed_at: Duration::from_secs(1),
        };
        let mut app = test_app();
        app.toggle_running();
        app.pipeline.lock().unwrap().process(&sample("first"));
        app.toggle_running();
        app.toggle_running();
        let frame = app.pipeline.lock().unwrap().process(&sample("second"));
        assert_eq!(frame.devices.len(), 2);
    }

    #[test]
    fn toggle_starts_and_pauses_pipeline() {
        let mut app = test_app();
        app.toggle_running();
        assert!(app.is_running());
        app.toggle_running();
        assert!(!app.is_running());
    }

    #[test]
    fn tab_titles_are_unique_and_ordered() {
        let titles: Vec<_> = Tab::ALL.iter().map(|t| t.title()).collect();
        assert_eq!(titles, ["Overview", "Devices", "Map", "3D", "Signals"]);
    }

    #[test]
    fn north_is_up_and_east_is_right() {
        let origin = Pos2::new(100.0, 100.0);
        assert_eq!(to_screen(origin, 10.0, 0.0, 2.0), Pos2::new(100.0, 80.0));
        assert_eq!(to_screen(origin, 10.0, 3.0, 0.0), Pos2::new(130.0, 100.0));
    }

    #[test]
    fn devices_sort_strongest_first() {
        let device = |id: &str, rssi: i16| DeviceState {
            device_id: id.into(),
            transport: spi::Transport::Wifi,
            last_rssi_dbm: rssi,
            last_seen: Duration::ZERO,
            distance_m: None,
            bearing_deg: None,
            position_m: None,
        };
        let frame = ScreenFrame {
            at: Duration::ZERO,
            devices: vec![device("far", -90), device("near", -40)],
        };
        let order: Vec<_> = sorted_by_signal(&frame).iter().map(|d| d.device_id.as_str()).collect();
        assert_eq!(order, ["near", "far"]);
    }
}
