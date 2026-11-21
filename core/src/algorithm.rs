//! Algorithms transform radio samples into `ScreenFrame`s. The registry maps names to factories.

use crate::model::ScreenFrame;
use spi::RadioSample;
use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

/// Turns each new sample into an updated frame. Algorithms run in a chain; each one sees the
/// frame produced by the previous one.
pub trait Algorithm: Send {
    /// Stable identifier used by the registry and configuration.
    fn name(&self) -> &'static str;

    /// Applies one sample to the working frame, e.g. adding or refining device estimates.
    fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame);
}

/// Creates a fresh algorithm instance with default parameters.
pub type Factory = fn() -> Box<dyn Algorithm>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryError {
    DuplicateName(&'static str),
}

impl fmt::Display for RegistryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RegistryError::DuplicateName(name) => write!(f, "algorithm already registered: {name}"),
        }
    }
}

impl Error for RegistryError {}

/// Name-to-factory map. Names are sorted so listings are deterministic.
#[derive(Default)]
pub struct AlgorithmRegistry {
    factories: BTreeMap<&'static str, Factory>,
}

impl AlgorithmRegistry {
    pub fn register(&mut self, name: &'static str, factory: Factory) -> Result<(), RegistryError> {
        if self.factories.contains_key(name) {
            return Err(RegistryError::DuplicateName(name));
        }
        self.factories.insert(name, factory);
        Ok(())
    }

    pub fn create(&self, name: &str) -> Option<Box<dyn Algorithm>> {
        self.factories.get(name).map(|factory| factory())
    }

    pub fn names(&self) -> Vec<&'static str> {
        self.factories.keys().copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use spi::Transport;
    use std::time::Duration;

    struct Counter {
        seen: usize,
    }

    impl Algorithm for Counter {
        fn name(&self) -> &'static str {
            "counter"
        }

        fn apply(&mut self, sample: &RadioSample, frame: &mut ScreenFrame) {
            self.seen += 1;
            frame.at = sample.observed_at;
        }
    }

    fn make_counter() -> Box<dyn Algorithm> {
        Box::new(Counter { seen: 0 })
    }

    #[test]
    fn registers_creates_and_lists_by_name() {
        let mut registry = AlgorithmRegistry::default();
        registry.register("counter", make_counter).unwrap();
        registry.register("alpha", make_counter).unwrap();

        assert_eq!(registry.names(), ["alpha", "counter"]);
        assert_eq!(registry.create("counter").unwrap().name(), "counter");
        assert!(registry.create("missing").is_none());
    }

    #[test]
    fn rejects_duplicate_names() {
        let mut registry = AlgorithmRegistry::default();
        registry.register("counter", make_counter).unwrap();
        assert_eq!(
            registry.register("counter", make_counter),
            Err(RegistryError::DuplicateName("counter"))
        );
    }

    #[test]
    fn created_algorithm_updates_frame() {
        let mut algorithm = make_counter();
        let sample = RadioSample {
            transport: Transport::Wifi,
            device_id: "ap".into(),
            rssi_dbm: -50,
            observed_at: Duration::from_secs(3),
            details: Default::default(),
        };
        let mut frame = ScreenFrame::default();
        algorithm.apply(&sample, &mut frame);
        assert_eq!(frame.at, Duration::from_secs(3));
    }
}
