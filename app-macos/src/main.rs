mod app;

use eframe::egui;
use platform_macos::MacosPlatform;

fn main() -> eframe::Result {
    let runtime = tokio::runtime::Runtime::new().expect("failed to start async runtime");
    let platform = MacosPlatform::new();
    let hub = core::SampleHub::new(Box::new(platform.scanner()), 1024);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Command Center")
            .with_inner_size([1100.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Command Center",
        options,
        Box::new(move |_cc| Ok(Box::new(app::ShellApp::new(hub, runtime)))),
    )
}
