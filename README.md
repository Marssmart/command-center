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
