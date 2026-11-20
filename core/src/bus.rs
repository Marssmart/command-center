//! Sample hub: fans radio samples out to subscribers and runs the scanner only while someone listens.

use spi::{RadioSample, RadioScanner, SpiError};
use std::sync::{mpsc, Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use tokio::sync::broadcast;

/// Shares one `RadioScanner` between subscribers.
///
/// The scanner starts on the first [`SampleHub::subscribe`] and stops when the last
/// [`Subscription`] is dropped. A stopped scanner must drop the sink it was given,
/// otherwise the forwarding thread cannot exit.
pub struct SampleHub {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    scanner: Box<dyn RadioScanner + Send>,
    tx: broadcast::Sender<RadioSample>,
    subscribers: usize,
    forwarder: Option<JoinHandle<()>>,
}

impl SampleHub {
    /// `capacity` is the per-subscriber buffer; slow subscribers see `RecvError::Lagged`.
    pub fn new(scanner: Box<dyn RadioScanner + Send>, capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self {
            inner: Arc::new(Mutex::new(Inner {
                scanner,
                tx,
                subscribers: 0,
                forwarder: None,
            })),
        }
    }

    /// Attaches a subscriber, starting the scanner if it is the first one.
    pub fn subscribe(&self) -> Result<Subscription, SpiError> {
        let mut inner = lock(&self.inner);
        // Subscribe before starting so the first sample is not missed.
        let rx = inner.tx.subscribe();
        if inner.subscribers == 0 {
            inner.start()?;
        }
        inner.subscribers += 1;
        Ok(Subscription {
            rx,
            hub: Arc::clone(&self.inner),
        })
    }

    pub fn subscriber_count(&self) -> usize {
        lock(&self.inner).subscribers
    }

    pub fn is_running(&self) -> bool {
        lock(&self.inner).forwarder.is_some()
    }
}

impl Inner {
    fn start(&mut self) -> Result<(), SpiError> {
        let (sink, samples) = mpsc::channel();
        self.scanner.start(sink)?;
        let tx = self.tx.clone();
        self.forwarder = Some(thread::spawn(move || {
            for sample in samples {
                // No receivers left is not an error here; the hub stops the scanner in that case.
                let _ = tx.send(sample);
            }
        }));
        Ok(())
    }

    fn stop(&mut self) {
        self.scanner.stop();
        if let Some(handle) = self.forwarder.take() {
            let _ = handle.join();
        }
    }
}

/// A receiving handle. Dropping the last one stops the scanner.
pub struct Subscription {
    rx: broadcast::Receiver<RadioSample>,
    hub: Arc<Mutex<Inner>>,
}

impl Subscription {
    pub async fn recv(&mut self) -> Result<RadioSample, broadcast::error::RecvError> {
        self.rx.recv().await
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        let mut inner = lock(&self.hub);
        inner.subscribers -= 1;
        if inner.subscribers == 0 {
            inner.stop();
        }
    }
}

fn lock(inner: &Mutex<Inner>) -> MutexGuard<'_, Inner> {
    inner.lock().expect("sample hub lock poisoned")
}

#[cfg(test)]
mod tests {
    use super::*;
    use spi::Transport;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    struct FakeScanner {
        started: Arc<AtomicUsize>,
        stopped: Arc<AtomicUsize>,
        sink: Option<mpsc::Sender<RadioSample>>,
    }

    impl RadioScanner for FakeScanner {
        fn start(&mut self, sink: mpsc::Sender<RadioSample>) -> Result<(), SpiError> {
            self.started.fetch_add(1, Ordering::SeqCst);
            sink.send(sample(-42)).map_err(|e| SpiError::Other(e.to_string()))?;
            self.sink = Some(sink);
            Ok(())
        }

        fn stop(&mut self) {
            self.stopped.fetch_add(1, Ordering::SeqCst);
            self.sink = None;
        }
    }

    fn sample(rssi: i16) -> RadioSample {
        RadioSample {
            transport: Transport::Wifi,
            device_id: "ap".into(),
            rssi_dbm: rssi,
            observed_at: Duration::from_secs(1),
        }
    }

    fn hub() -> (SampleHub, Arc<AtomicUsize>, Arc<AtomicUsize>) {
        let started = Arc::new(AtomicUsize::new(0));
        let stopped = Arc::new(AtomicUsize::new(0));
        let scanner = FakeScanner {
            started: Arc::clone(&started),
            stopped: Arc::clone(&stopped),
            sink: None,
        };
        (SampleHub::new(Box::new(scanner), 16), started, stopped)
    }

    #[tokio::test]
    async fn scanner_starts_lazily_and_delivers_samples() {
        let (hub, started, _) = hub();
        assert_eq!(started.load(Ordering::SeqCst), 0);

        let mut sub = hub.subscribe().unwrap();
        assert_eq!(started.load(Ordering::SeqCst), 1);
        let received = tokio::time::timeout(Duration::from_secs(5), sub.recv())
            .await
            .expect("sample should arrive")
            .unwrap();
        assert_eq!(received.rssi_dbm, -42);
    }

    #[tokio::test]
    async fn every_subscriber_receives_samples() {
        let (hub, _, _) = hub();
        let mut first = hub.subscribe().unwrap();
        let mut second = hub.subscribe().unwrap();
        assert_eq!(hub.subscriber_count(), 2);

        for sub in [&mut first, &mut second] {
            let s = tokio::time::timeout(Duration::from_secs(5), sub.recv())
                .await
                .expect("sample should arrive")
                .unwrap();
            assert_eq!(s.rssi_dbm, -42);
        }
    }

    #[tokio::test]
    async fn scanner_stops_with_last_subscriber_and_restarts_on_demand() {
        let (hub, started, stopped) = hub();
        let first = hub.subscribe().unwrap();
        let second = hub.subscribe().unwrap();

        drop(first);
        assert_eq!(stopped.load(Ordering::SeqCst), 0);
        assert!(hub.is_running());

        drop(second);
        assert_eq!(stopped.load(Ordering::SeqCst), 1);
        assert!(!hub.is_running());

        let _again = hub.subscribe().unwrap();
        assert_eq!(started.load(Ordering::SeqCst), 2);
    }
}
