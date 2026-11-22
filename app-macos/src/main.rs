mod app;
mod frame_log;
mod scan_status;
mod view3d;

use eframe::egui;
use platform_macos::MacosPlatform;
use spi::RadioScanner;

/// When set to a file path, scanning starts at launch and the frame stream is summarised there.
const FRAME_LOG_VAR: &str = "COMMAND_CENTER_FRAME_LOG";

fn main() -> eframe::Result {
    let runtime = tokio::runtime::Runtime::new().expect("failed to start async runtime");
    let platform = MacosPlatform::new();
    let scanner = platform.scanner();
    let scan_status = scanner.scan_status();
    let hub = core::SampleHub::new(Box::new(scanner), 1024);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Command Center")
            .with_inner_size([1100.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Command Center",
        options,
        Box::new(move |_cc| {
            let mut shell = app::ShellApp::new(hub, runtime);
            if let Some(path) = std::env::var_os(FRAME_LOG_VAR) {
                frame_log::spawn(shell.frames(), scan_status.clone(), path.into());
                shell.toggle_running();
            }
            // The platform moves into the UI so the permission prompt's manager lives as long as the window.
            Ok(Box::new(scan_status::WithScanStatus::new(
                shell,
                scan_status,
                Box::new(platform),
            )))
        }),
    )
}
