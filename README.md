# command-center

Rust workspace for a macOS counter-surveillance tool.

## Crates

- **`core`**: platform-free logic (data model, algorithms, producers, pipes). It must not depend on platform APIs. It depends on `spi`.
- **`spi`**: service provider interface. Defines the traits (RadioScanner, RangingProvider, PermissionProvider, Clock, Storage) that `core` depends on.
- **`platform-macos`**: implements the `spi` traits for macOS (CoreBluetooth, CoreWLAN). Depends on `spi`.
- **`app-macos`**: macOS application shell (UI). Depends on `core`, `spi` and `platform-macos`.

## Build and test

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

## Calibration harness (simulated)

`core` ships a simulated calibration harness. It generates RSSI and compass observations for the floor plan in `calibration/floorplan.example.toml`, runs the ranging, bearing and fusion chain, and prints error statistics. The example values are **simulated**, and the noise and path-loss model matches what the algorithms assume. The output describes that model only. It is not real-world accuracy.

```sh
cargo run -p core --example calibration                              # example plan
cargo run -p core --example calibration -- path/to/your-floorplan.toml
```

To use real measurements, copy the example file and replace the room size, sensor position, path-loss values, and anchor positions. Real-hardware calibration is still open and needs measured data.

## Package a macOS DMG

The `macos-dmg` GitHub Actions workflow builds `app-macos` and packages a `.app` and `.dmg` with cargo-packager 0.11.8. The bundle is ad-hoc signed (no Developer ID, not notarized). Tags `v*` also attach the DMG to a GitHub release.
