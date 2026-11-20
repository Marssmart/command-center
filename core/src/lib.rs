//! Platform-free core logic: data model, algorithms, producers and pipes.

pub mod algorithm;
pub mod bearing;
pub mod bus;
pub mod calibration;
pub mod fusion;
pub mod model;
pub mod pipes;
pub mod playback;
pub mod ranging;

pub use algorithm::{Algorithm, AlgorithmRegistry};
pub use bearing::BearingEstimator;
pub use bus::{SampleHub, Subscription};
pub use fusion::Fusion;
pub use model::{DeviceState, ScreenFrame};
pub use pipes::{run_pipeline, FramePipe, Pipeline};
pub use playback::Playback;
pub use ranging::RssiRanging;
pub use spi::{RadioSample, Transport};

#[cfg(test)]
mod tests {
    #[test]
    fn test_harness_runs() {
        assert_eq!(1 + 1, 2);
    }
}
