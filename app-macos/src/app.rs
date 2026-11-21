use core::{
    run_pipeline, Algorithm, BearingEstimator, DeviceState, Fusion, FramePipe, Playback, Pipeline,
    RssiRanging, SampleHub, ScreenFrame, SweepBearing,
};
use platform_macos::CoreLocationHeading;
use spi::{HeadingSource, ManualFacing, Transport};
use crate::view3d::{self, Orbit};
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Stroke};
use std::cmp::Reverse;
use std::collections::VecDeque;
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
    facing: Arc<ManualFacing>,
    /// Degrees on the facing dial. Only meaningful once `facing_set` is true.
    facing_deg: f64,
    facing_set: bool,
    /// Whether the Mac has a compass. When it does not, the manual dial is the only facing source.
    compass: bool,
    sweeping: bool,
    /// Device whose detail panel is open.
    selected: Option<String>,
    task: Option<JoinHandle<()>>,
    playback: Playback,
    orbit: Orbit,
    status: Option<String>,
}

impl ShellApp {
    pub fn new(hub: SampleHub, runtime: tokio::runtime::Runtime) -> Self {
        let pipe = FramePipe::new(FRAME_CAPACITY);
        let frames = pipe.subscribe();
        let facing = Arc::new(ManualFacing::new());
        let pipeline = Arc::new(Mutex::new(default_pipeline(facing.clone())));
        Self {
            tab: Tab::default(),
            runtime,
            hub,
            pipe,
            frames,
            pipeline,
            facing,
            facing_deg: 0.0,
            facing_set: false,
            compass: CoreLocationHeading::heading_available(),
            sweeping: false,
            selected: None,
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

    fn toggle_sweep(&mut self) {
        self.sweeping = !self.sweeping;
        let chain = if self.sweeping {
            sweep_chain(self.facing.clone())
        } else {
            default_chain(self.facing.clone())
        };
        self.pipeline
            .lock()
            .expect("pipeline lock poisoned")
            .set_algorithms(chain);
    }

    fn set_facing(&mut self, degrees: f64) {
        self.facing_deg = degrees.rem_euclid(360.0);
        self.facing_set = true;
        self.facing.set(Some(self.facing_deg));
    }

    /// Manual facing dial: drag the rose or type degrees. Bearings are relative to this value.
    fn facing_controls(&mut self, ui: &mut egui::Ui) {
        let (rect, response) = ui.allocate_exact_size(egui::Vec2::splat(44.0), egui::Sense::click_and_drag());
        if response.clicked() || response.dragged() {
            if let Some(pointer) = response.interact_pointer_pos() {
                let centre = rect.center();
                let degrees = (pointer.x - centre.x).atan2(centre.y - pointer.y).to_degrees();
                self.set_facing(f64::from(degrees));
            }
        }
        let centre = rect.center();
        let radius = rect.width() / 2.0;
        let painter = ui.painter_at(rect);
        painter.circle_stroke(centre, radius, Stroke::new(1.0, Color32::GRAY));
        if self.facing_set {
            let angle = (self.facing_deg as f32).to_radians();
            let tip = centre + egui::vec2(angle.sin(), -angle.cos()) * radius;
            painter.line_segment([centre, tip], Stroke::new(2.0, Color32::LIGHT_RED));
        }

        let mut degrees = self.facing_deg;
        if ui
            .add(egui::DragValue::new(&mut degrees).range(0.0..=359.0).suffix("°"))
            .changed()
        {
            self.set_facing(degrees);
        }
        let note = match (self.facing_set, self.compass) {
            (false, _) => "Facing not set: drag the dial",
            (true, false) => "Manual facing (no compass on this Mac)",
            (true, true) => "Facing set",
        };
        ui.label(note);

        let sweep_label = if self.sweeping { "Stop sweep" } else { "Start sweep" };
        if ui
            .add_enabled(self.facing_set || self.sweeping, egui::Button::new(sweep_label))
            .on_hover_text("Turn through a full circle with the dial set to your facing")
            .clicked()
        {
            self.toggle_sweep();
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

fn default_pipeline(facing: Arc<dyn HeadingSource>) -> Pipeline {
    Pipeline::new(default_chain(facing))
}

/// Continuous bearings: a weighted mean over recent facings.
fn default_chain(facing: Arc<dyn HeadingSource>) -> Vec<Box<dyn Algorithm>> {
    vec![
        Box::new(RssiRanging::default()),
        Box::new(BearingEstimator::new(facing, 64)),
        Box::new(Fusion::default()),
    ]
}

/// Sweep bearings: the strongest facing after a full turn.
fn sweep_chain(facing: Arc<dyn HeadingSource>) -> Vec<Box<dyn Algorithm>> {
    vec![
        Box::new(RssiRanging::default()),
        Box::new(SweepBearing::new(facing)),
        Box::new(Fusion::default()),
    ]
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
                ui.separator();
                self.facing_controls(ui);
            });
        });

        let selected = self.selected.clone();
        let mut close_detail = false;
        let detail = selected
            .as_deref()
            .and_then(|id| self.playback.current().and_then(|f| f.device(id)));
        if let Some(device) = detail {
            egui::Panel::right("device-detail")
                .default_size(320.0)
                .show(ui, |ui| {
                    if ui.button("Close").clicked() {
                        close_detail = true;
                    }
                    egui::ScrollArea::vertical().show(ui, |ui| device_detail(ui, device));
                });
        }

        let picked = egui::CentralPanel::default()
            .show(ui, |ui| {
                if let Some(status) = &self.status {
                    ui.colored_label(Color32::LIGHT_RED, status);
                }
                let frame = self.playback.current();
                let selected = selected.as_deref();
                match self.tab() {
                    Tab::Overview => {
                        overview(ui, self.is_running(), frame);
                        None
                    }
                    Tab::Devices => devices(ui, frame, selected),
                    Tab::Map => map(ui, frame, selected),
                    Tab::View3d => {
                        let listed = device_list(ui, frame, selected);
                        view3d::show(ui, &mut self.orbit, frame, selected).or(listed)
                    }
                    Tab::Signals => {
                        signals(ui, frame);
                        None
                    }
                }
            })
            .inner;

        if let Some(id) = picked {
            self.selected = Some(id);
        }
        if close_detail {
            self.selected = None;
        }
    }
}

fn overview(ui: &mut egui::Ui, running: bool, frame: Option<&ScreenFrame>) {
    ui.heading("Overview");
    ui.label(if running { "Scanning" } else { "Paused" });
    ui.label(format!("Devices seen: {}", frame.map_or(0, |f| f.devices.len())));
}

fn devices(ui: &mut egui::Ui, frame: Option<&ScreenFrame>, selected: Option<&str>) -> Option<String> {
    ui.heading("Devices");
    let Some(frame) = frame else {
        ui.label("No data yet.");
        return None;
    };
    let mut picked = None;
    for device in sorted_by_signal(frame) {
        let is_selected = selected == Some(device.device_id.as_str());
        if ui.selectable_label(is_selected, device_line(device)).clicked() {
            picked = Some(device.device_id.clone());
        }
    }
    picked
}

fn transport_name(transport: Transport) -> &'static str {
    match transport {
        Transport::BluetoothLe => "Bluetooth LE",
        Transport::Wifi => "Wi-Fi",
    }
}

fn field(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(label);
    ui.label(value);
    ui.end_row();
}

/// Everything known about one device, read from the frame currently shown.
fn device_detail(ui: &mut egui::Ui, device: &DeviceState) {
    ui.heading(&device.device_id);
    egui::Grid::new("device-fields")
        .num_columns(2)
        .spacing([12.0, 4.0])
        .show(ui, |ui| {
            field(ui, "Transport", transport_name(device.transport).to_string());
            field(ui, "Last RSSI", format!("{} dBm", device.last_rssi_dbm));
            field(ui, "Samples", device.sample_count.to_string());
            field(ui, "First seen", format!("{:.1} s", device.first_seen.as_secs_f64()));
            field(ui, "Last seen", format!("{:.1} s", device.last_seen.as_secs_f64()));
            field(
                ui,
                "Distance",
                device
                    .distance_m
                    .map_or("not estimated".into(), |d| format!("{d:.1} m (RSSI log-distance, smoothed)")),
            );
            field(
                ui,
                "Bearing",
                device
                    .bearing_deg
                    .map_or("not known (set the facing dial)".into(), |b| format!("{b:.0}° relative to facing")),
            );
            field(
                ui,
                "Position",
                device.position_m.map_or("not known".into(), |(east, north)| {
                    format!("east {east:.1} m, north {north:.1} m (relative to facing)")
                }),
            );
        });

    ui.separator();
    ui.strong("Identity");
    let details = &device.details;
    egui::Grid::new("device-identity")
        .num_columns(2)
        .spacing([12.0, 4.0])
        .show(ui, |ui| match device.transport {
            Transport::BluetoothLe => {
                let manufacturers = if details.manufacturer_ids.is_empty() {
                    "none advertised".to_string()
                } else {
                    details
                        .manufacturer_ids
                        .iter()
                        .map(|id| format!("0x{id:04X}"))
                        .collect::<Vec<_>>()
                        .join(", ")
                };
                field(ui, "Manufacturer IDs", manufacturers);
                let services = if details.service_uuids.is_empty() {
                    "none advertised".to_string()
                } else {
                    details
                        .service_uuids
                        .iter()
                        .map(|uuid| format!("{uuid:032x}"))
                        .collect::<Vec<_>>()
                        .join("\n")
                };
                field(ui, "Service UUIDs", services);
                field(
                    ui,
                    "Payload hash",
                    details.payload_hash.map_or("not available".into(), |h| format!("{h:016x}")),
                );
            }
            Transport::Wifi => {
                field(ui, "BSSID", device.device_id.clone());
                field(
                    ui,
                    "SSID",
                    details
                        .ssid
                        .clone()
                        .unwrap_or_else(|| "not reported (needs Location permission)".into()),
                );
            }
        });

    ui.separator();
    ui.strong("RSSI history (dBm)");
    rssi_plot(ui, &device.rssi_history);
}

/// Last RSSI values as a line, scaled from -100 dBm (bottom) to -30 dBm (top).
fn rssi_plot(ui: &mut egui::Ui, history: &VecDeque<i16>) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(280.0, 90.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::DARK_GRAY), egui::StrokeKind::Inside);
    let count = history.len();
    if count < 2 {
        return;
    }
    let points: Vec<Pos2> = history
        .iter()
        .enumerate()
        .map(|(i, rssi)| {
            let x = rect.left() + rect.width() * i as f32 / (count - 1) as f32;
            let level = ((f32::from(*rssi) + 100.0) / 70.0).clamp(0.0, 1.0);
            Pos2::new(x, rect.bottom() - level * rect.height())
        })
        .collect();
    painter.add(egui::Shape::line(points, Stroke::new(1.5, Color32::LIGHT_BLUE)));
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

fn map(ui: &mut egui::Ui, frame: Option<&ScreenFrame>, selected: Option<&str>) -> Option<String> {
    let listed = device_list(ui, frame, selected);
    let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click());
    let origin = response.rect.center();
    let ring = Stroke::new(1.0, Color32::DARK_GRAY);
    for radius_m in RING_RADII_M {
        painter.circle_stroke(origin, radius_m * PIXELS_PER_METRE, ring);
    }
    painter.circle_filled(origin, 4.0, Color32::WHITE);

    let Some(frame) = frame else {
        return listed;
    };
    // Distance without bearing: a ring with no angle, so it implies no direction.
    let distance_ring = Stroke::new(1.5, Color32::LIGHT_GREEN);
    for device in unplaced_with_distance(frame) {
        if let Some(distance) = device.distance_m {
            painter.circle_stroke(origin, distance as f32 * PIXELS_PER_METRE, distance_ring);
        }
    }
    let mut points = Vec::new();
    for device in &frame.devices {
        let Some((east, north)) = device.position_m else {
            continue;
        };
        let point = to_screen(origin, PIXELS_PER_METRE, east, north);
        let is_selected = selected == Some(device.device_id.as_str());
        let (radius, fill) = if is_selected {
            (7.0, Color32::WHITE)
        } else {
            (5.0, Color32::LIGHT_BLUE)
        };
        painter.circle_filled(point, radius, fill);
        painter.text(
            point + egui::vec2(8.0, 0.0),
            Align2::LEFT_CENTER,
            &device.device_id,
            FontId::proportional(12.0),
            Color32::WHITE,
        );
        points.push((device.device_id.clone(), point));
    }
    listed.or_else(|| nearest_click(&response, &points, 12.0))
}

/// The device whose point is nearest a click, if the click landed within `radius` pixels of one.
pub(crate) fn nearest_click(response: &egui::Response, points: &[(String, Pos2)], radius: f32) -> Option<String> {
    if !response.clicked() {
        return None;
    }
    let pointer = response.interact_pointer_pos()?;
    points
        .iter()
        .filter(|(_, point)| point.distance(pointer) <= radius)
        .min_by(|a, b| a.1.distance(pointer).total_cmp(&b.1.distance(pointer)))
        .map(|(id, _)| id.clone())
}

/// Every detected device with its placement status. Devices without a bearing are listed here
/// because they cannot be drawn at a direction.
fn device_list(ui: &mut egui::Ui, frame: Option<&ScreenFrame>, selected: Option<&str>) -> Option<String> {
    let Some(frame) = frame else {
        ui.label("No data yet.");
        return None;
    };
    let placed = frame.devices.iter().filter(|d| d.position_m.is_some()).count();
    ui.label(format!(
        "{} devices, {} placed on the map. Bearings and positions are relative to the facing dial.",
        frame.devices.len(),
        placed
    ));
    let mut picked = None;
    egui::ScrollArea::vertical()
        .id_salt("device-list")
        .max_height(120.0)
        .show(ui, |ui| {
            for device in sorted_by_signal(frame) {
                let label = format!("{}  ·  {}", device_line(device), placement_note(device));
                let is_selected = selected == Some(device.device_id.as_str());
                if ui.selectable_label(is_selected, label).clicked() {
                    picked = Some(device.device_id.clone());
                }
            }
        });
    picked
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
            details: Default::default(),
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
        let device = |id: &str, rssi: i16| {
            DeviceState::from_sample(&RadioSample {
                transport: spi::Transport::Wifi,
                device_id: id.into(),
                rssi_dbm: rssi,
                observed_at: Duration::ZERO,
                details: Default::default(),
            })
        };
        let frame = ScreenFrame {
            at: Duration::ZERO,
            devices: vec![device("far", -90), device("near", -40)],
        };
        let order: Vec<_> = sorted_by_signal(&frame).iter().map(|d| d.device_id.as_str()).collect();
        assert_eq!(order, ["near", "far"]);
    }
}
