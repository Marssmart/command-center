//! Wi-Fi scanning with CoreWLAN.
//!
//! CoreWLAN reports access points only. Devices joined to a network (phones, cameras) do not show
//! up in a scan; seeing them needs monitor-mode capture, which this crate does not do.
//!
//! macOS withholds the BSSID and SSID of every network unless the app has Location permission.
//! The BSSID is the device identity, so without it a scan gives nothing usable. That case is
//! reported as a blocked scan, never as an empty result.

use crate::location_status;
use objc2_core_wlan::{CWChannelBand, CWChannelWidth, CWNetwork, CWSecurity, CWWiFiClient};
use spi::{
    Permission, PermissionStatus, RadioSample, SampleDetails, ScanProblem, ScanState,
    ScanStatusBoard, Transport, WifiBand, WifiSecurity,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const SCAN_INTERVAL: Duration = Duration::from_secs(5);
const POLL_INTERVAL: Duration = Duration::from_millis(250);
/// How often to re-read Location permission while waiting for the user to answer the prompt.
const PERMISSION_RECHECK: Duration = Duration::from_secs(1);

/// One scan result, copied out of CoreWLAN so the mapping can be tested without a radio.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct WifiNetwork {
    pub bssid: Option<String>,
    pub ssid: Option<String>,
    pub rssi_dbm: isize,
    /// 0 when CoreWLAN has no measurement.
    pub noise_dbm: isize,
    pub channel: Option<u16>,
    pub band: Option<WifiBand>,
    pub channel_width_mhz: Option<u16>,
    pub security: Option<WifiSecurity>,
}

type ScanResult = Result<Vec<WifiNetwork>, ScanProblem>;

/// Scans on a schedule until `stop` is set, publishing samples to `sink` and the state to `board`.
///
/// A CoreWLAN scan blocks for seconds, so it runs on a second thread. This thread only waits in
/// short steps, which lets it drop the sink soon after `stop` is set.
pub(crate) fn spawn_wifi_scan(
    sink: Sender<RadioSample>,
    stop: Arc<AtomicBool>,
    epoch: Instant,
    board: Arc<ScanStatusBoard>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        board.set(Transport::Wifi, ScanState::Starting);
        let (requests, results) = spawn_scan_worker();
        while !stop.load(Ordering::Relaxed) {
            // Read before the scan: the results reflect the permission the scan ran with, and the
            // user may answer the prompt while it runs.
            let location = location_status();
            if requests.send(()).is_err() {
                break;
            }
            let Some(scan) = await_scan(&results, &stop) else {
                break;
            };
            let (samples, state) = scan_outcome(scan, location, epoch.elapsed());
            board.set(Transport::Wifi, state);
            for sample in samples {
                if sink.send(sample).is_err() {
                    return;
                }
            }
            wait_for_next_scan(&stop, location);
        }
    })
}

/// Runs one blocking scan per request. Ends when the requesting side is dropped.
fn spawn_scan_worker() -> (Sender<()>, Receiver<ScanResult>) {
    let (requests, pending) = mpsc::channel::<()>();
    let (done, results) = mpsc::channel();
    thread::spawn(move || {
        for () in pending {
            if done.send(scan_once()).is_err() {
                break;
            }
        }
    });
    (requests, results)
}

/// The result of the scan in flight, or `None` once `stop` is set or the worker is gone.
fn await_scan(results: &Receiver<ScanResult>, stop: &AtomicBool) -> Option<ScanResult> {
    loop {
        if stop.load(Ordering::Relaxed) {
            return None;
        }
        match results.recv_timeout(POLL_INTERVAL) {
            Ok(scan) => return Some(scan),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return None,
        }
    }
}

/// Sleeps until the next scan is due. Returns early on `stop`, or when the user answers the
/// Location prompt, so results appear right after Allow rather than a scan interval later.
fn wait_for_next_scan(stop: &AtomicBool, location: PermissionStatus) {
    let mut waited = Duration::ZERO;
    let mut since_recheck = Duration::ZERO;
    while waited < SCAN_INTERVAL && !stop.load(Ordering::Relaxed) {
        thread::sleep(POLL_INTERVAL);
        waited += POLL_INTERVAL;
        since_recheck += POLL_INTERVAL;
        if location != PermissionStatus::Granted && since_recheck >= PERMISSION_RECHECK {
            since_recheck = Duration::ZERO;
            if location_status() != location {
                return;
            }
        }
    }
}

fn scan_once() -> ScanResult {
    let client = unsafe { CWWiFiClient::sharedWiFiClient() };
    let interface = unsafe { client.interface() }.ok_or(ScanProblem::NoRadio)?;
    if !unsafe { interface.powerOn() } {
        return Err(ScanProblem::RadioOff);
    }
    let networks = unsafe { interface.scanForNetworksWithName_includeHidden_error(None, true) }
        .map_err(|error| {
            ScanProblem::Failed(format!(
                "{} ({} {})",
                error.localizedDescription(),
                error.domain(),
                error.code()
            ))
        })?;
    Ok(networks.iter().map(|network| read_network(&network)).collect())
}

fn read_network(network: &CWNetwork) -> WifiNetwork {
    let channel = unsafe { network.wlanChannel() };
    WifiNetwork {
        bssid: unsafe { network.bssid() }.map(|b| b.to_string()),
        ssid: unsafe { network.ssid() }.map(|s| s.to_string()),
        rssi_dbm: unsafe { network.rssiValue() },
        noise_dbm: unsafe { network.noiseMeasurement() },
        channel: channel
            .as_ref()
            .and_then(|c| u16::try_from(unsafe { c.channelNumber() }).ok())
            .filter(|number| *number > 0),
        band: channel.as_ref().and_then(|c| band(unsafe { c.channelBand() })),
        channel_width_mhz: channel
            .as_ref()
            .and_then(|c| channel_width_mhz(unsafe { c.channelWidth() })),
        security: strongest_security(|security| unsafe { network.supportsSecurity(security) }),
    }
}

/// Turns one scan into samples and the state to report for it.
pub(crate) fn scan_outcome(
    scan: ScanResult,
    location: PermissionStatus,
    observed_at: Duration,
) -> (Vec<RadioSample>, ScanState) {
    let networks = match scan {
        Ok(networks) => networks,
        Err(problem) => return (Vec::new(), ScanState::Blocked(problem)),
    };
    let samples: Vec<RadioSample> = networks
        .iter()
        .filter_map(|network| wifi_sample(network, observed_at))
        .collect();
    let state = if !samples.is_empty() {
        ScanState::Active
    } else {
        match location {
            PermissionStatus::NotDetermined => {
                ScanState::Blocked(ScanProblem::PermissionNotDetermined(Permission::Location))
            }
            PermissionStatus::Denied => {
                ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Location))
            }
            PermissionStatus::Granted if networks.is_empty() => ScanState::Active,
            PermissionStatus::Granted => ScanState::Blocked(ScanProblem::Failed(format!(
                "macOS returned {} networks without their addresses",
                networks.len()
            ))),
        }
    };
    (samples, state)
}

/// Builds a sample for an access point. A network without a BSSID has no identity and is skipped;
/// `scan_outcome` reports why when that leaves a scan empty.
pub(crate) fn wifi_sample(network: &WifiNetwork, observed_at: Duration) -> Option<RadioSample> {
    let device_id = normalize_bssid(network.bssid.as_deref()?);
    Some(RadioSample {
        transport: Transport::Wifi,
        device_id,
        rssi_dbm: clamp_dbm(network.rssi_dbm),
        observed_at,
        details: SampleDetails {
            ssid: network.ssid.clone().filter(|name| !name.is_empty()),
            channel: network.channel,
            band: network.band,
            channel_width_mhz: network.channel_width_mhz,
            security: network.security,
            noise_dbm: (network.noise_dbm != 0).then(|| clamp_dbm(network.noise_dbm)),
            ..Default::default()
        },
    })
}

fn clamp_dbm(value: isize) -> i16 {
    value.clamp(i16::MIN as isize, i16::MAX as isize) as i16
}

/// Lower-case, two hex digits per octet. macOS has printed octets without their leading zero, and
/// the same access point must keep one identity however it is spelled.
pub(crate) fn normalize_bssid(raw: &str) -> String {
    let octets: Vec<&str> = raw.trim().split(':').collect();
    let well_formed = octets.len() == 6
        && octets
            .iter()
            .all(|o| (1..=2).contains(&o.len()) && o.bytes().all(|b| b.is_ascii_hexdigit()));
    if !well_formed {
        return raw.trim().to_ascii_lowercase();
    }
    octets
        .iter()
        .map(|octet| format!("{octet:0>2}").to_ascii_lowercase())
        .collect::<Vec<_>>()
        .join(":")
}

pub(crate) fn band(raw: CWChannelBand) -> Option<WifiBand> {
    match raw {
        CWChannelBand::Band2GHz => Some(WifiBand::Ghz2_4),
        CWChannelBand::Band5GHz => Some(WifiBand::Ghz5),
        CWChannelBand::Band6GHz => Some(WifiBand::Ghz6),
        _ => None,
    }
}

pub(crate) fn channel_width_mhz(raw: CWChannelWidth) -> Option<u16> {
    match raw {
        CWChannelWidth::Width20MHz => Some(20),
        CWChannelWidth::Width40MHz => Some(40),
        CWChannelWidth::Width80MHz => Some(80),
        CWChannelWidth::Width160MHz => Some(160),
        _ => None,
    }
}

/// CoreWLAN only answers "does this network support security X", so ask strongest first.
///
/// The mixed and transition constants are left out on purpose. CoreWLAN answers yes to them for
/// any network such a profile could join: a WPA2-only network "supports" WPA3 Transition and an
/// open one "supports" OWE Transition, so they would overstate what the access point offers.
pub(crate) fn strongest_security(supports: impl Fn(CWSecurity) -> bool) -> Option<WifiSecurity> {
    const STRONGEST_FIRST: [(CWSecurity, WifiSecurity); 10] = [
        (CWSecurity::WPA3Enterprise, WifiSecurity::Wpa3Enterprise),
        (CWSecurity::WPA2Enterprise, WifiSecurity::Wpa2Enterprise),
        (CWSecurity::WPAEnterprise, WifiSecurity::WpaEnterprise),
        (CWSecurity::WPA3Personal, WifiSecurity::Wpa3Personal),
        (CWSecurity::WPA2Personal, WifiSecurity::Wpa2Personal),
        (CWSecurity::WPAPersonal, WifiSecurity::WpaPersonal),
        (CWSecurity::DynamicWEP, WifiSecurity::Wep),
        (CWSecurity::WEP, WifiSecurity::Wep),
        (CWSecurity::OWE, WifiSecurity::EnhancedOpen),
        (CWSecurity::None, WifiSecurity::Open),
    ];
    STRONGEST_FIRST
        .iter()
        .find(|(raw, _)| supports(*raw))
        .map(|(_, security)| *security)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn network(bssid: Option<&str>, ssid: Option<&str>) -> WifiNetwork {
        WifiNetwork {
            bssid: bssid.map(Into::into),
            ssid: ssid.map(Into::into),
            rssi_dbm: -60,
            ..Default::default()
        }
    }

    #[test]
    fn sample_carries_identity_signal_and_details() {
        let full = WifiNetwork {
            bssid: Some("AA:BB:CC:DD:EE:FF".into()),
            ssid: Some("office".into()),
            rssi_dbm: -61,
            noise_dbm: -94,
            channel: Some(44),
            band: Some(WifiBand::Ghz5),
            channel_width_mhz: Some(80),
            security: Some(WifiSecurity::Wpa2Personal),
        };
        let sample = wifi_sample(&full, Duration::from_secs(2)).unwrap();
        assert_eq!(sample.transport, Transport::Wifi);
        assert_eq!(sample.device_id, "aa:bb:cc:dd:ee:ff");
        assert_eq!(sample.rssi_dbm, -61);
        assert_eq!(sample.observed_at, Duration::from_secs(2));
        assert_eq!(
            sample.details,
            SampleDetails {
                ssid: Some("office".into()),
                channel: Some(44),
                band: Some(WifiBand::Ghz5),
                channel_width_mhz: Some(80),
                security: Some(WifiSecurity::Wpa2Personal),
                noise_dbm: Some(-94),
                ..Default::default()
            }
        );
    }

    #[test]
    fn hidden_network_is_kept_without_a_name() {
        for hidden in [network(Some("aa:bb:cc:dd:ee:ff"), None), network(Some("aa:bb:cc:dd:ee:ff"), Some(""))] {
            let sample = wifi_sample(&hidden, Duration::ZERO).unwrap();
            assert_eq!(sample.device_id, "aa:bb:cc:dd:ee:ff");
            assert_eq!(sample.details.ssid, None);
        }
    }

    #[test]
    fn network_without_bssid_gives_no_sample() {
        assert!(wifi_sample(&network(None, Some("office")), Duration::ZERO).is_none());
    }

    #[test]
    fn out_of_range_levels_are_clamped_and_missing_noise_is_unknown() {
        let mut odd = network(Some("aa:bb:cc:dd:ee:ff"), None);
        odd.rssi_dbm = -100_000;
        let sample = wifi_sample(&odd, Duration::ZERO).unwrap();
        assert_eq!(sample.rssi_dbm, i16::MIN);
        assert_eq!(sample.details.noise_dbm, None);
    }

    #[test]
    fn bssid_spelling_does_not_change_identity() {
        assert_eq!(normalize_bssid("A4:B:C1:0:E3:F4"), "a4:0b:c1:00:e3:f4");
        assert_eq!(normalize_bssid(" a4:0b:c1:00:e3:f4 "), "a4:0b:c1:00:e3:f4");
        // Not a MAC address: kept as reported rather than guessed at.
        assert_eq!(normalize_bssid("Not-A-Mac"), "not-a-mac");
    }

    #[test]
    fn scan_with_identified_networks_is_active_whatever_the_permission_says() {
        let scan = Ok(vec![network(Some("aa:bb:cc:dd:ee:ff"), Some("office")), network(None, None)]);
        let (samples, state) = scan_outcome(scan, PermissionStatus::NotDetermined, Duration::ZERO);
        assert_eq!(samples.len(), 1);
        assert_eq!(state, ScanState::Active);
    }

    #[test]
    fn networks_stripped_of_bssid_are_reported_as_a_location_problem() {
        let stripped = || Ok(vec![network(None, None), network(None, None)]);

        let (samples, state) = scan_outcome(stripped(), PermissionStatus::NotDetermined, Duration::ZERO);
        assert!(samples.is_empty());
        assert_eq!(
            state,
            ScanState::Blocked(ScanProblem::PermissionNotDetermined(Permission::Location))
        );

        let (_, state) = scan_outcome(stripped(), PermissionStatus::Denied, Duration::ZERO);
        assert_eq!(state, ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Location)));

        let (_, state) = scan_outcome(stripped(), PermissionStatus::Granted, Duration::ZERO);
        assert!(matches!(state, ScanState::Blocked(ScanProblem::Failed(_))));
    }

    #[test]
    fn empty_scan_is_active_only_with_location_granted() {
        let (_, state) = scan_outcome(Ok(Vec::new()), PermissionStatus::Granted, Duration::ZERO);
        assert_eq!(state, ScanState::Active);

        let (_, state) = scan_outcome(Ok(Vec::new()), PermissionStatus::Denied, Duration::ZERO);
        assert_eq!(state, ScanState::Blocked(ScanProblem::PermissionDenied(Permission::Location)));
    }

    #[test]
    fn scan_failure_is_reported_with_its_reason() {
        for problem in [
            ScanProblem::NoRadio,
            ScanProblem::RadioOff,
            ScanProblem::Failed("Resource busy".into()),
        ] {
            let (samples, state) = scan_outcome(Err(problem.clone()), PermissionStatus::Granted, Duration::ZERO);
            assert!(samples.is_empty());
            assert_eq!(state, ScanState::Blocked(problem));
        }
    }

    #[test]
    fn channel_facts_map_from_corewlan_constants() {
        assert_eq!(band(CWChannelBand::Band2GHz), Some(WifiBand::Ghz2_4));
        assert_eq!(band(CWChannelBand::Band6GHz), Some(WifiBand::Ghz6));
        assert_eq!(band(CWChannelBand::BandUnknown), None);
        assert_eq!(channel_width_mhz(CWChannelWidth::Width160MHz), Some(160));
        assert_eq!(channel_width_mhz(CWChannelWidth::WidthUnknown), None);
    }

    /// The answer sets are the ones CoreWLAN gave for real networks whose beacons were checked.
    #[test]
    fn security_matches_what_the_access_point_advertises() {
        let answers = |supported: &'static [CWSecurity]| move |s| supported.contains(&s);
        let cases: [(&[CWSecurity], Option<WifiSecurity>); 6] = [
            // WPA2-only beacon. CoreWLAN also says yes to WPA3 Transition.
            (
                &[CWSecurity::WPAPersonalMixed, CWSecurity::WPA2Personal, CWSecurity::Personal, CWSecurity::WPA3Transition],
                Some(WifiSecurity::Wpa2Personal),
            ),
            // WPA2 and WPA3 beacon.
            (
                &[CWSecurity::WPAPersonalMixed, CWSecurity::WPA2Personal, CWSecurity::Personal, CWSecurity::WPA3Personal, CWSecurity::WPA3Transition],
                Some(WifiSecurity::Wpa3Personal),
            ),
            // WPA3-only beacon.
            (
                &[CWSecurity::Personal, CWSecurity::WPA3Personal, CWSecurity::WPA3Transition],
                Some(WifiSecurity::Wpa3Personal),
            ),
            (
                &[CWSecurity::WPAEnterpriseMixed, CWSecurity::WPA2Enterprise, CWSecurity::Enterprise],
                Some(WifiSecurity::Wpa2Enterprise),
            ),
            // Open network. CoreWLAN also says yes to OWE Transition.
            (&[CWSecurity::None, CWSecurity::OWETransition], Some(WifiSecurity::Open)),
            (&[], None),
        ];
        for (supported, expected) in cases {
            assert_eq!(strongest_security(answers(supported)), expected, "{supported:?}");
        }
    }
}
