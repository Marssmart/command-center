//! Stable pseudo-identifiers for devices that hide their hardware address.
//!
//! macOS never exposes a MAC address for BLE peripherals, and many devices rotate their
//! advertising address. The fingerprint is built only from advertisement content, so it does
//! not change when the address rotates. It is probabilistic: identical advertisements from two
//! devices collide, and payloads that change over time (some Apple devices) change the id.

/// FNV-1a 64-bit. Unlike `DefaultHasher`, its output is stable across Rust releases.
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Fingerprints a BLE advertisement from its manufacturer data and service UUIDs.
///
/// Input order does not matter. Returns `None` when the advertisement has neither, since there
/// is nothing to fingerprint.
pub fn ble_fingerprint<'a>(
    manufacturer_data: impl IntoIterator<Item = (u16, &'a [u8])>,
    service_uuids: impl IntoIterator<Item = u128>,
) -> Option<String> {
    ble_payload_hash(manufacturer_data, service_uuids).map(|hash| format!("ble:{hash:016x}"))
}

/// FNV-1a hash of the advertisement content behind [`ble_fingerprint`]. `None` when there is none.
pub fn ble_payload_hash<'a>(
    manufacturer_data: impl IntoIterator<Item = (u16, &'a [u8])>,
    service_uuids: impl IntoIterator<Item = u128>,
) -> Option<u64> {
    let mut manufacturer: Vec<(u16, &[u8])> = manufacturer_data.into_iter().collect();
    manufacturer.sort_unstable();
    let mut services: Vec<u128> = service_uuids.into_iter().collect();
    services.sort_unstable();
    if manufacturer.is_empty() && services.is_empty() {
        return None;
    }

    let mut hash = FNV_OFFSET;
    let mut feed = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(FNV_PRIME);
        }
    };
    for (company, data) in &manufacturer {
        feed(&company.to_le_bytes());
        feed(&(data.len() as u32).to_le_bytes());
        feed(data);
    }
    feed(b"|services|");
    for uuid in &services {
        feed(&uuid.to_be_bytes());
    }
    Some(hash)
}

#[cfg(test)]
mod tests {
    use super::*;

    const APPLE: u16 = 0x004C;
    const BEACON_UUID: u128 = 0xFEAA_0000_0000_1000_8000_0080_5F9B_34FB;

    #[test]
    fn same_advertisement_gives_same_id() {
        let first = ble_fingerprint([(APPLE, &[1, 2, 3][..])], [BEACON_UUID]);
        let second = ble_fingerprint([(APPLE, &[1, 2, 3][..])], [BEACON_UUID]);
        assert!(first.is_some());
        assert_eq!(first, second);
    }

    #[test]
    fn input_order_does_not_change_id() {
        let a = ble_fingerprint([(1, &[9][..]), (2, &[8][..])], [1, 2]);
        let b = ble_fingerprint([(2, &[8][..]), (1, &[9][..])], [2, 1]);
        assert_eq!(a, b);
    }

    #[test]
    fn different_payloads_do_not_collide() {
        let a = ble_fingerprint([(APPLE, &[1, 2, 3][..])], []);
        let b = ble_fingerprint([(APPLE, &[1, 2, 4][..])], []);
        let c = ble_fingerprint([(0x0059, &[1, 2, 3][..])], []);
        assert_ne!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn different_service_uuids_do_not_collide() {
        let a = ble_fingerprint([], [BEACON_UUID]);
        let b = ble_fingerprint([], [BEACON_UUID + 1]);
        assert_ne!(a, b);
    }

    #[test]
    fn section_boundary_is_unambiguous() {
        // Same bytes, split differently between manufacturer data and services.
        let a = ble_fingerprint([(APPLE, &[0x00][..])], []);
        let b = ble_fingerprint([(APPLE, &[][..])], [0x00]);
        assert_ne!(a, b);
    }

    #[test]
    fn empty_advertisement_has_no_fingerprint() {
        assert_eq!(ble_fingerprint([], []), None);
    }
}
