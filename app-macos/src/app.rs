use core::survey::MIN_SPOTS;
use core::{
    run_pipeline, Algorithm, BearingEstimator, DeviceState, Fusion, FramePipe, PlacementMethod,
    Playback, Pipeline, RssiRanging, SampleHub, ScreenFrame, Survey, SurveyLocator, SweepBearing,
};
use crate::view3d::{self, Orbit};
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Stroke};
use platform_macos::CoreLocationHeading;
use spi::{HeadingSource, ManualFacing, Transport};
use std::cmp::Reverse;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
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

/// How positions are obtained. Each mode is a different algorithm chain over the same samples.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum ChainMode {
    /// Positions from distances measured at spots the user marks while walking.
    #[default]
    Survey,
    /// Bearing from a full turn against the facing dial, then position from distance and bearing.
    Sweep,
    /// Continuous bearing from the facing dial, once the signal shows it varies with facing.
    Facing,
}

impl ChainMode {
    pub const ALL: [ChainMode; 3] = [ChainMode::Survey, ChainMode::Sweep, ChainMode::Facing];

    pub fn label(self) -> &'static str {
        match self {
            ChainMode::Survey => "Survey",
            ChainMode::Sweep => "Sweep",
            ChainMode::Facing => "Facing dial",
        }
    }

    fn uses_dial(self) -> bool {
        !matches!(self, ChainMode::Survey)
    }
}

/// What a click on the map or 3D view picked.
pub(crate) enum MapPick {
    Device(String),
    /// Metres east and north of the map centre, for dropping a survey spot.
    Ground(f64, f64),
}

/// Tabbed app. Play starts the scanner and the algorithm pipeline. Pause stops both.
pub struct ShellApp {
    tab: Tab,
    runtime: tokio::runtime::Runtime,
    hub: SampleHub,
    pipe: FramePipe,
    frames: broadcast::Receiver<ScreenFrame>,
    /// Shared across pause, resume and mode changes, so the device table is kept.
    pipeline: Arc<Mutex<Pipeline>>,
    facing: Arc<ManualFacing>,
    /// Degrees on the facing dial. Only meaningful once `facing_set` is true.
    facing_deg: f64,
    facing_set: bool,
    /// Whether the Mac has a compass. When it does not, the manual dial is the only facing source.
    compass: bool,
    mode: ChainMode,
    /// Spots marked for the survey. Kept across pause and mode changes.
    survey: Arc<Mutex<Survey>>,
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
        let survey = Arc::new(Mutex::new(Survey::new()));
        let mode = ChainMode::default();
        let pipeline = Arc::new(Mutex::new(Pipeline::new(chain_for(
            mode,
            facing.clone(),
            survey.clone(),
        ))));
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
            mode,
            survey,
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

    /// A new subscription to the frames the screens read.
    pub fn frames(&self) -> broadcast::Receiver<ScreenFrame> {
        self.pipe.subscribe()
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

    fn set_mode(&mut self, mode: ChainMode) {
        self.mode = mode;
        let chain = chain_for(mode, self.facing.clone(), self.survey.clone());
        lock_pipeline(&self.pipeline).set_algorithms(chain);
    }

    fn spots(&self) -> Vec<(f64, f64)> {
        lock_survey(&self.survey)
            .spots()
            .iter()
            .map(|spot| spot.position_m)
            .collect()
    }

    fn set_facing(&mut self, degrees: f64) {
        self.facing_deg = degrees.rem_euclid(360.0);
        self.facing_set = true;
        self.facing.set(Some(self.facing_deg));
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

    /// Manual facing dial: drag the rose or type degrees. Bearings are relative to this value.
    fn facing_controls(&mut self, ui: &mut egui::Ui) {
        let (rect, response) =
            ui.allocate_exact_size(egui::Vec2::splat(44.0), egui::Sense::click_and_drag());
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
    }

    fn survey_controls(&mut self, ui: &mut egui::Ui) {
        let marked = self.spots().len();
        ui.label(format!(
            "Spots marked: {marked}. Click the map where you stand to mark one."
        ));
        if ui
            .add_enabled(marked > 0, egui::Button::new("Clear survey"))
            .clicked()
        {
            lock_survey(&self.survey).clear();
        }
    }
}

fn chain_for(
    mode: ChainMode,
    facing: Arc<dyn HeadingSource>,
    survey: Arc<Mutex<Survey>>,
) -> Vec<Box<dyn Algorithm>> {
    match mode {
        ChainMode::Survey => vec![
            Box::new(RssiRanging::default()),
            Box::new(SurveyLocator::new(survey)),
        ],
        ChainMode::Sweep => vec![
            Box::new(RssiRanging::default()),
            Box::new(SweepBearing::new(facing)),
            Box::new(Fusion::default()),
        ],
        ChainMode::Facing => vec![
            Box::new(RssiRanging::default()),
            Box::new(BearingEstimator::new(facing, 64)),
            Box::new(Fusion::default()),
        ],
    }
}

fn lock_pipeline(pipeline: &Mutex<Pipeline>) -> MutexGuard<'_, Pipeline> {
    pipeline.lock().expect("pipeline lock poisoned")
}

fn lock_survey(survey: &Mutex<Survey>) -> MutexGuard<'_, Survey> {
    survey.lock().expect("survey lock poisoned")
}

impl eframe::App for ShellApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.drain_frames();
        if self.is_running() {
            ui.ctx().request_repaint_after(REPAINT_INTERVAL);
        }
        let spots = self.spots();
        let mut next_mode = None;

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
                ui.label("Method:");
                for mode in ChainMode::ALL {
                    if ui.selectable_label(self.mode == mode, mode.label()).clicked() {
                        next_mode = Some(mode);
                    }
                }
            });
            ui.horizontal(|ui| {
                if self.mode.uses_dial() {
                    self.facing_controls(ui);
                } else {
                    self.survey_controls(ui);
                }
            });
        });
        if let Some(mode) = next_mode {
            self.set_mode(mode);
        }

        let selected = self.selected.clone();
        let mut close_detail = false;
        let detail = selected
            .as_deref()
            .and_then(|id| self.playback.current().and_then(|f| f.device(id)));
        if let Some(device) = detail {
            egui::Panel::right("device-detail")
                .default_size(340.0)
                .show(ui, |ui| {
                    if ui.button("Close").clicked() {
                        close_detail = true;
                    }
                    egui::ScrollArea::vertical().show(ui, |ui| device_detail(ui, device));
                });
        }

        let mode = self.mode;
        let user = user_offset(&spots, mode);
        let picked = egui::CentralPanel::default()
            .show(ui, |ui| {
                if let Some(status) = &self.status {
                    ui.colored_label(Color32::LIGHT_RED, status);
                }
                let frame = self.playback.current();
                let selected = selected.as_deref();
                match self.tab() {
                    Tab::Overview => {
                        overview(ui, self.is_running(), mode, frame);
                        None
                    }
                    Tab::Devices => devices(ui, frame, selected).map(MapPick::Device),
                    Tab::Map => map(ui, frame, selected, &spots, user, mode),
                    Tab::View3d => {
                        let listed = device_list(ui, frame, selected, mode, spots.len());
                        view3d::show(ui, &mut self.orbit, frame, selected, user)
                            .or(listed)
                            .map(MapPick::Device)
                    }
                    Tab::Signals => {
                        signals(ui, frame);
                        None
                    }
                }
            })
            .inner;

        match picked {
            Some(MapPick::Device(id)) => self.selected = Some(id),
            Some(MapPick::Ground(east, north)) if mode == ChainMode::Survey => {
                lock_survey(&self.survey).mark((east, north));
            }
            _ => {}
        }
        if close_detail {
            self.selected = None;
        }
    }
}

fn overview(ui: &mut egui::Ui, running: bool, mode: ChainMode, frame: Option<&ScreenFrame>) {
    ui.heading("Overview");
    ui.label(if running { "Scanning" } else { "Paused" });
    ui.label(format!("Method: {}", mode.label()));
    let devices = frame.map_or(0, |f| f.devices.len());
    let placed = frame.map_or(0, |f| f.devices.iter().filter(|d| d.position_m.is_some()).count());
    ui.label(format!("Devices seen: {devices}"));
    ui.label(format!("Positioned: {placed}, distance only: {}", devices - placed));
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

/// Where the user stands relative to the survey origin: the last spot, or the origin itself.
fn user_offset(spots: &[(f64, f64)], mode: ChainMode) -> (f64, f64) {
    match (mode, spots.first(), spots.last()) {
        (ChainMode::Survey, Some(first), Some(last)) => (last.0 - first.0, last.1 - first.1),
        _ => (0.0, 0.0),
    }
}

/// Guidance shown when no device has a position yet, telling the user how to get one.
fn guidance(ui: &mut egui::Ui, mode: ChainMode, placed: bool, spots: usize) {
    if placed {
        return;
    }
    let text = match mode {
        ChainMode::Survey if spots < MIN_SPOTS => format!(
            "No positions yet. Survey: click the map where you stand to mark a spot ({spots} of {MIN_SPOTS} marked). Stay there about 10 s, then move and mark the next spot."
        ),
        ChainMode::Survey => {
            "Survey: spots marked. Positions appear for devices heard from every spot. Stay longer at each spot if none appear yet.".to_string()
        }
        ChainMode::Sweep => {
            "No positions yet. Sweep: set the facing dial to the direction you face, then turn slowly through a full circle, updating the dial as you turn.".to_string()
        }
        ChainMode::Facing => {
            "No bearings yet. Facing dial: set the dial and turn. Bearings need the signal to vary across a turn; one facing never gives one.".to_string()
        }
    };
    ui.colored_label(Color32::YELLOW, text);
}

/// Every detected device with its placement status, plus guidance while nothing is positioned.
fn device_list(
    ui: &mut egui::Ui,
    frame: Option<&ScreenFrame>,
    selected: Option<&str>,
    mode: ChainMode,
    spots: usize,
) -> Option<String> {
    let Some(frame) = frame else {
        guidance(ui, mode, false, spots);
        ui.label("No data yet.");
        return None;
    };
    let placed = frame.devices.iter().filter(|d| d.position_m.is_some()).count();
    guidance(ui, mode, placed > 0, spots);
    ui.label(format!(
        "{} devices, {} positioned. Positions are relative to {}.",
        frame.devices.len(),
        placed,
        if mode == ChainMode::Survey { "the first spot" } else { "you" }
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

fn map(
    ui: &mut egui::Ui,
    frame: Option<&ScreenFrame>,
    selected: Option<&str>,
    spots: &[(f64, f64)],
    user: (f64, f64),
    mode: ChainMode,
) -> Option<MapPick> {
    let listed = device_list(ui, frame, selected, mode, spots.len());
    let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click());
    let centre = response.rect.center();
    // Positions are relative to the first spot, so the origin is drawn there.
    let (fx, fy) = spots.first().copied().unwrap_or((0.0, 0.0));
    let origin = to_screen(centre, PIXELS_PER_METRE, fx, fy);
    let here = to_screen(origin, PIXELS_PER_METRE, user.0, user.1);

    let ring = Stroke::new(1.0, Color32::DARK_GRAY);
    for radius_m in RING_RADII_M {
        painter.circle_stroke(origin, radius_m * PIXELS_PER_METRE, ring);
    }
    painter.circle_filled(origin, 4.0, Color32::WHITE);

    for pair in spots.windows(2) {
        let a = to_screen(centre, PIXELS_PER_METRE, pair[0].0, pair[0].1);
        let b = to_screen(centre, PIXELS_PER_METRE, pair[1].0, pair[1].1);
        painter.line_segment([a, b], Stroke::new(1.0, Color32::GOLD));
    }
    for (i, (east, north)) in spots.iter().enumerate() {
        let p = to_screen(centre, PIXELS_PER_METRE, *east, *north);
        painter.circle_filled(p, 7.0, Color32::GOLD);
        painter.text(
            p,
            Align2::CENTER_CENTER,
            (i + 1).to_string(),
            FontId::proportional(11.0),
            Color32::BLACK,
        );
    }

    let mut points = Vec::new();
    if let Some(frame) = frame {
        // Distance without a bearing: a ring around the user, with no angle, so no direction is implied.
        for device in frame.devices.iter().filter(|d| d.position_m.is_none()) {
            if let Some(distance) = device.distance_m {
                let radius = distance as f32 * PIXELS_PER_METRE;
                painter.circle_stroke(here, radius, Stroke::new(1.0, Color32::from_rgba_unmultiplied(120, 220, 120, 170)));
                painter.text(
                    here + egui::vec2(0.0, -radius),
                    Align2::CENTER_BOTTOM,
                    format!("{} (distance only)", device.device_id),
                    FontId::proportional(11.0),
                    Color32::LIGHT_GREEN,
                );
            }
        }
        for device in &frame.devices {
            let Some((east, north)) = device.position_m else {
                continue;
            };
            let point = to_screen(origin, PIXELS_PER_METRE, east, north);
            if let Some(uncertainty) = device.position_uncertainty_m {
                let radius = uncertainty as f32 * PIXELS_PER_METRE;
                painter.circle_stroke(point, radius, Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 90)));
            }
            let is_selected = selected == Some(device.device_id.as_str());
            let (radius, fill) = if is_selected {
                (7.0, Color32::WHITE)
            } else {
                (5.0, placement_colour(device.placement))
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
    }

    listed
        .map(MapPick::Device)
        .or_else(|| nearest_click(&response, &points, 12.0).map(MapPick::Device))
        .or_else(|| ground_click(&response, centre))
}

/// Colour of a device marker by how its position was obtained.
pub(crate) fn placement_colour(method: PlacementMethod) -> Color32 {
    match method {
        PlacementMethod::Survey => Color32::LIGHT_GREEN,
        PlacementMethod::Sweep => Color32::from_rgb(255, 190, 80),
        PlacementMethod::Facing => Color32::LIGHT_BLUE,
        PlacementMethod::Unplaced => Color32::GRAY,
    }
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

/// A click on empty map, as metres east and north of the map centre.
fn ground_click(response: &egui::Response, centre: Pos2) -> Option<MapPick> {
    if !response.clicked() {
        return None;
    }
    let pointer = response.interact_pointer_pos()?;
    Some(MapPick::Ground(
        f64::from((pointer.x - centre.x) / PIXELS_PER_METRE),
        f64::from((centre.y - pointer.y) / PIXELS_PER_METRE),
    ))
}

fn placement_text(method: PlacementMethod) -> &'static str {
    match method {
        PlacementMethod::Unplaced => "distance only",
        PlacementMethod::Facing => "facing dial (continuous)",
        PlacementMethod::Sweep => "full-turn sweep",
        PlacementMethod::Survey => "survey, solved from marked spots",
    }
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
            field(ui, "Placement", placement_text(device.placement).to_string());
            field(
                ui,
                "Distance",
                device.distance_m.map_or("not estimated".into(), |d| {
                    if device.placement == PlacementMethod::Survey {
                        format!("{d:.1} m (geometric, from the current spot)")
                    } else {
                        format!("{d:.1} m (RSSI estimate)")
                    }
                }),
            );
            field(
                ui,
                "Bearing",
                device.bearing_deg.map_or("not known".into(), |b| {
                    let spread = device
                        .bearing_spread_deg
                        .map_or(String::new(), |s| format!(" ± {s:.0}°"));
                    if device.placement == PlacementMethod::Survey {
                        format!("{b:.0}° from map north")
                    } else {
                        format!("{b:.0}°{spread} relative to facing")
                    }
                }),
            );
            field(
                ui,
                "Position",
                device.position_m.map_or("not placed".into(), |(east, north)| {
                    format!("east {east:.1} m, north {north:.1} m")
                }),
            );
            field(
                ui,
                "Uncertainty",
                device
                    .position_uncertainty_m
                    .map_or("not known".into(), |u| format!("± {u:.1} m (1 σ)")),
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
                        .unwrap_or_else(|| "hidden network (no name broadcast)".into()),
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

fn placement_note(device: &DeviceState) -> String {
    match (device.position_m, device.position_uncertainty_m) {
        (Some(_), Some(u)) => format!("placed, ± {u:.1} m"),
        (Some(_), None) => "placed".to_string(),
        (None, _) if device.distance_m.is_some() => "distance only".to_string(),
        (None, _) => "signal only".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use spi::{RadioSample, RadioScanner, SpiError};
    use std::sync::mpsc;
    use std::time::Duration;

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
    fn starts_on_overview_paused_in_survey_mode() {
        let app = test_app();
        assert_eq!(app.tab(), Tab::Overview);
        assert!(!app.is_running());
        assert_eq!(app.mode, ChainMode::Survey);
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
    fn device_table_survives_pause_and_resume() {
        let sample = |id: &str| RadioSample {
            transport: Transport::Wifi,
            device_id: id.into(),
            rssi_dbm: -50,
            observed_at: Duration::from_secs(1),
            details: Default::default(),
        };
        let mut app = test_app();
        app.toggle_running();
        lock_pipeline(&app.pipeline).process(&sample("first"));
        app.toggle_running();
        app.toggle_running();
        let frame = lock_pipeline(&app.pipeline).process(&sample("second"));
        assert_eq!(frame.devices.len(), 2);
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
    fn user_stands_at_last_spot_only_in_survey_mode() {
        let spots = [(1.0, 1.0), (4.0, 5.0)];
        assert_eq!(user_offset(&spots, ChainMode::Survey), (3.0, 4.0));
        assert_eq!(user_offset(&spots, ChainMode::Sweep), (0.0, 0.0));
        assert_eq!(user_offset(&[], ChainMode::Survey), (0.0, 0.0));
    }

    #[test]
    fn devices_sort_strongest_first() {
        let device = |id: &str, rssi: i16| {
            DeviceState::from_sample(&RadioSample {
                transport: Transport::Wifi,
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
