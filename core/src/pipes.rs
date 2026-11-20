//! Algorithm pipelines: samples flow in, the algorithm chain refines a `ScreenFrame`, and screens
//! subscribe to the resulting frame pipe.

use crate::algorithm::Algorithm;
use crate::bus::Subscription;
use crate::model::{DeviceState, ScreenFrame};
use spi::RadioSample;
use tokio::sync::broadcast;

/// Owns a working frame and an ordered algorithm chain.
pub struct Pipeline {
    algorithms: Vec<Box<dyn Algorithm>>,
    frame: ScreenFrame,
}

impl Pipeline {
    pub fn new(algorithms: Vec<Box<dyn Algorithm>>) -> Self {
        Self {
            algorithms,
            frame: ScreenFrame::default(),
        }
    }

    /// Updates the device table from `sample`, runs the chain, and returns the resulting frame.
    pub fn process(&mut self, sample: &RadioSample) -> ScreenFrame {
        self.frame.at = sample.observed_at;
        match self
            .frame
            .devices
            .iter_mut()
            .find(|d| d.device_id == sample.device_id)
        {
            Some(device) => device.observe(sample),
            None => self.frame.devices.push(DeviceState::from_sample(sample)),
        }
        for algorithm in &mut self.algorithms {
            algorithm.apply(sample, &mut self.frame);
        }
        self.frame.clone()
    }
}

/// Broadcast channel of frames. Screens subscribe here and never see samples or algorithms.
#[derive(Clone)]
pub struct FramePipe {
    tx: broadcast::Sender<ScreenFrame>,
}

impl FramePipe {
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self { tx }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<ScreenFrame> {
        self.tx.subscribe()
    }
}

/// Drives a pipeline until its sample subscription closes. Spawn it on the tokio runtime.
pub async fn run_pipeline(mut samples: Subscription, mut pipeline: Pipeline, pipe: FramePipe) {
    loop {
        match samples.recv().await {
            Ok(sample) => {
                // Having no frame subscribers is not an error.
                let _ = pipe.tx.send(pipeline.process(&sample));
            }
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::SampleHub;
    use spi::{RadioScanner, SpiError, Transport};
    use std::sync::mpsc;
    use std::time::Duration;

    /// Sets a fixed bearing on every device, to prove the chain ran.
    struct FixedBearing;

    impl Algorithm for FixedBearing {
        fn name(&self) -> &'static str {
            "fixed-bearing"
        }

        fn apply(&mut self, _sample: &RadioSample, frame: &mut ScreenFrame) {
            for device in &mut frame.devices {
                device.bearing_deg = Some(90.0);
            }
        }
    }

    struct OneSampleScanner {
        sink: Option<mpsc::Sender<RadioSample>>,
    }

    impl RadioScanner for OneSampleScanner {
        fn start(&mut self, sink: mpsc::Sender<RadioSample>) -> Result<(), SpiError> {
            sink.send(sample(-40, 2)).map_err(|e| SpiError::Other(e.to_string()))?;
            self.sink = Some(sink);
            Ok(())
        }

        fn stop(&mut self) {
            self.sink = None;
        }
    }

    fn sample(rssi: i16, secs: u64) -> RadioSample {
        RadioSample {
            transport: Transport::BluetoothLe,
            device_id: "beacon".into(),
            rssi_dbm: rssi,
            observed_at: Duration::from_secs(secs),
        }
    }

    #[test]
    fn process_tracks_devices_and_runs_chain() {
        let mut pipeline = Pipeline::new(vec![Box::new(FixedBearing)]);
        pipeline.process(&sample(-70, 1));
        let frame = pipeline.process(&sample(-60, 2));

        assert_eq!(frame.at, Duration::from_secs(2));
        assert_eq!(frame.devices.len(), 1);
        let device = frame.device("beacon").unwrap();
        assert_eq!(device.last_rssi_dbm, -60);
        assert_eq!(device.bearing_deg, Some(90.0));
    }

    #[tokio::test]
    async fn frame_pipe_delivers_chain_output_to_screens() {
        let hub = SampleHub::new(Box::new(OneSampleScanner { sink: None }), 8);
        let samples = hub.subscribe().unwrap();
        let pipe = FramePipe::new(8);
        let mut frames = pipe.subscribe();
        let task = tokio::spawn(run_pipeline(
            samples,
            Pipeline::new(vec![Box::new(FixedBearing)]),
            pipe.clone(),
        ));

        let frame = tokio::time::timeout(Duration::from_secs(5), frames.recv())
            .await
            .expect("frame should arrive")
            .unwrap();
        task.abort();

        assert_eq!(frame.device("beacon").and_then(|d| d.bearing_deg), Some(90.0));
    }
}
