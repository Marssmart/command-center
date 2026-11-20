//! Platform-free core logic: data model, algorithms, producers and pipes.

pub mod model;

pub use model::{DeviceState, ScreenFrame};
pub use spi::{RadioSample, Transport};

#[cfg(test)]
mod tests {
    #[test]
    fn test_harness_runs() {
        assert_eq!(1 + 1, 2);
    }
}
