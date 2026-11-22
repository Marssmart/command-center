//! Optional log of the frame stream, for checking a live run without watching the window.
//!
//! Enabled by `COMMAND_CENTER_FRAME_LOG=<file>`. It subscribes to the same frames the screens
//! read and appends one summary line per second.

use core::ScreenFrame;
use spi::{ScanStatusSource, Transport};
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::RecvError;
use tokio::sync::broadcast::Receiver;

const WRITE_INTERVAL: Duration = Duration::from_secs(1);

/// Logs until the frame pipe closes. Failing to open the file is reported once and ends the log.
pub fn spawn(mut frames: Receiver<ScreenFrame>, status: Option<Arc<dyn ScanStatusSource>>, path: PathBuf) {
    thread::spawn(move || {
        let mut file = match OpenOptions::new().create(true).append(true).open(&path) {
            Ok(file) => file,
            Err(error) => {
                eprintln!("frame log {} could not be opened: {error}", path.display());
                return;
            }
        };
        let mut written: Option<Instant> = None;
        loop {
            let frame = match frames.blocking_recv() {
                Ok(frame) => frame,
                Err(RecvError::Lagged(_)) => continue,
                Err(RecvError::Closed) => break,
            };
            if written.is_some_and(|at| at.elapsed() < WRITE_INTERVAL) {
                continue;
            }
            written = Some(Instant::now());
            let states = status.as_ref().map_or(String::new(), |s| {
                format!(
                    " wifi_state={:?} ble_state={:?}",
                    s.scan_state(Transport::Wifi),
                    s.scan_state(Transport::BluetoothLe)
                )
            });
            if writeln!(file, "{}{states}", summary(&frame)).is_err() {
                break;
            }
        }
    });
}

/// Device counts per transport, then every Wi-Fi device in BSSID order.
fn summary(frame: &ScreenFrame) -> String {
    let mut wifi: Vec<_> = frame
        .devices
        .iter()
        .filter(|d| d.transport == Transport::Wifi)
        .collect();
    wifi.sort_by(|a, b| a.device_id.cmp(&b.device_id));
    let listed: String = wifi
        .iter()
        .map(|d| {
            let details = &d.details;
            let ssid = details.ssid.as_deref().map_or("(hidden)".to_string(), |s| format!("{s:?}"));
            let facts: String = [
                details.channel.map(|c| format!("ch {c}")),
                details.band.map(|b| b.label().to_string()),
                details.channel_width_mhz.map(|w| format!("{w} MHz")),
                details.security.map(|s| s.label().to_string()),
                details.noise_dbm.map(|n| format!("noise {n} dBm")),
            ]
            .into_iter()
            .flatten()
            .map(|fact| format!(", {fact}"))
            .collect();
            format!(" | {} {ssid} {} dBm{facts}", d.device_id, d.last_rssi_dbm)
        })
        .collect();
    format!(
        "t={:.1}s devices={} ble={} wifi={}{listed}",
        frame.at.as_secs_f64(),
        frame.devices.len(),
        frame.devices.len() - wifi.len(),
        wifi.len(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::DeviceState;
    use spi::RadioSample;

    fn device(transport: Transport, id: &str, ssid: Option<&str>) -> DeviceState {
        let mut sample = RadioSample {
            transport,
            device_id: id.into(),
            rssi_dbm: -60,
            observed_at: Duration::from_secs(3),
            details: Default::default(),
        };
        sample.details.ssid = ssid.map(Into::into);
        if ssid.is_some() {
            sample.details.channel = Some(44);
            sample.details.band = Some(spi::WifiBand::Ghz5);
            sample.details.security = Some(spi::WifiSecurity::Wpa2Personal);
        }
        DeviceState::from_sample(&sample)
    }

    #[test]
    fn summary_counts_transports_and_lists_wifi_by_bssid() {
        let frame = ScreenFrame {
            at: Duration::from_millis(3500),
            devices: vec![
                device(Transport::Wifi, "bb:00:00:00:00:02", None),
                device(Transport::BluetoothLe, "ble:1", None),
                device(Transport::Wifi, "aa:00:00:00:00:01", Some("office")),
            ],
        };
        assert_eq!(
            summary(&frame),
            "t=3.5s devices=3 ble=1 wifi=2 | aa:00:00:00:00:01 \"office\" -60 dBm, ch 44, 5 GHz, WPA2 Personal | bb:00:00:00:00:02 (hidden) -60 dBm"
        );
    }
}
