use eframe::egui;

/// Top-level screens of the app.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    #[default]
    Overview,
    Devices,
    Map,
    Signals,
}

impl Tab {
    pub const ALL: [Tab; 4] = [Tab::Overview, Tab::Devices, Tab::Map, Tab::Signals];

    pub fn title(self) -> &'static str {
        match self {
            Tab::Overview => "Overview",
            Tab::Devices => "Devices",
            Tab::Map => "Map",
            Tab::Signals => "Signals",
        }
    }
}

/// Tabbed shell with a placeholder capture toggle. Screens are empty until the data pipes exist.
#[derive(Debug, Default)]
pub struct ShellApp {
    tab: Tab,
    running: bool,
}

impl ShellApp {
    pub fn tab(&self) -> Tab {
        self.tab
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn toggle_running(&mut self) {
        self.running = !self.running;
    }
}

impl eframe::App for ShellApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("tabs").show(ui, |ui| {
            ui.horizontal(|ui| {
                for tab in Tab::ALL {
                    if ui.selectable_label(self.tab == tab, tab.title()).clicked() {
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
            ui.heading(self.tab().title());
            ui.label("No data yet.");
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_on_overview_paused() {
        let app = ShellApp::default();
        assert_eq!(app.tab(), Tab::Overview);
        assert!(!app.is_running());
    }

    #[test]
    fn toggle_flips_running_state() {
        let mut app = ShellApp::default();
        app.toggle_running();
        assert!(app.is_running());
        app.toggle_running();
        assert!(!app.is_running());
    }

    #[test]
    fn tab_titles_are_unique_and_ordered() {
        let titles: Vec<_> = Tab::ALL.iter().map(|t| t.title()).collect();
        assert_eq!(titles, ["Overview", "Devices", "Map", "Signals"]);
    }
}
