mod app;

use eframe::egui;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Command Center")
            .with_inner_size([1100.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Command Center",
        options,
        Box::new(|_cc| Ok(Box::new(app::ShellApp::default()))),
    )
}
