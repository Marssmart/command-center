//! Runs the simulated calibration harness on a floor-plan file.
//!
//! Usage: `cargo run -p core --example calibration -- [path/to/floorplan.toml]`
//! The default is `calibration/floorplan.example.toml`, relative to the repository root.

use core::calibration::{format_report, run, FloorPlan};

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "calibration/floorplan.example.toml".to_string());
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("cannot read {path}: {e}");
        std::process::exit(1);
    });
    let plan = FloorPlan::from_toml(&text).unwrap_or_else(|e| {
        eprintln!("invalid floor plan {path}: {e}");
        std::process::exit(1);
    });
    print!("{}", format_report(&run(&plan)));
}
