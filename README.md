# command-center

Rust workspace for a macOS counter-surveillance tool.

## Crates

- **`core`**: platform-free logic (data model, algorithms, producers, pipes). It must not depend on platform APIs. It depends on `spi`.
- **`spi`**: service provider interface. Defines the traits (RadioScanner, RangingProvider, PermissionProvider, ScanStatusSource, Clock, Storage) that `core` depends on.
- **`platform-macos`**: implements the `spi` traits for macOS (CoreBluetooth, CoreWLAN, CoreLocation). Depends on `spi`.
- **`app-macos`**: macOS application shell (UI). Depends on `core`, `spi` and `platform-macos`.

## Wi-Fi scanning

Wi-Fi access points appear as devices next to the Bluetooth ones. Each is identified by its BSSID, which stays the same across scans and launches, and carries its signal strength, network name, channel, band, channel width, security and noise floor. A hidden network is listed without a name.

**Limit: access points only.** CoreWLAN reports the access points in range. It does not report devices joined to a network, such as phones, cameras or laptops, so those do not appear as Wi-Fi devices. Seeing them needs monitor-mode capture, which this app does not do.

**Location permission is required.** macOS gives an app the BSSID and name of nearby networks only if it has Location access. Without it every network arrives with no identity and none can be listed. The app asks for Location when its window first opens. It does not read or store your location. If you refuse, allow it later in System Settings > Privacy & Security > Location Services.

The strip at the bottom of the window says what each radio is doing. When a radio cannot scan it says why: Location or Bluetooth permission not answered or denied, the radio turned off, no radio, or a failed scan. An empty device list always comes with a reason.

Two things are needed for the Location prompt to appear at all, and both are part of the packaged app:

- `NSLocationUsageDescription` in `app-macos/Info.plist`.
- The `com.apple.security.personal-information.location` entitlement in `app-macos/entitlements.plist`. The bundle is signed with the hardened runtime, and macOS silently drops the prompt for a hardened app without it.

So run the packaged `.app`, not `cargo run`: a bare binary has neither, and stays at "waiting for Location permission". The bundle is ad-hoc signed, so macOS treats each new build as a new app and asks again.

To check a run without watching the window, set `COMMAND_CENTER_FRAME_LOG` to a file. Scanning then starts at launch and one line per second is appended with the device counts per transport, every Wi-Fi device in the frame, and the scan state of both radios:

```sh
open -n --env COMMAND_CENTER_FRAME_LOG=/tmp/frames.log "target/release/Command Center.app"
```

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

To build the same bundle locally:

```sh
cargo install cargo-packager --version 0.11.8 --locked
cargo build --release -p app-macos
cargo packager --release --packages app-macos --formats app
```
