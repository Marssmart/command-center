//! Wi-Fi facts a platform may report for an access point.

/// Frequency band of a Wi-Fi channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiBand {
    Ghz2_4,
    Ghz5,
    Ghz6,
}

impl WifiBand {
    pub fn label(self) -> &'static str {
        match self {
            WifiBand::Ghz2_4 => "2.4 GHz",
            WifiBand::Ghz5 => "5 GHz",
            WifiBand::Ghz6 => "6 GHz",
        }
    }
}

/// Strongest security an access point offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiSecurity {
    /// No encryption.
    Open,
    /// Opportunistic Wireless Encryption: encrypted, with no password.
    EnhancedOpen,
    Wep,
    WpaPersonal,
    Wpa2Personal,
    Wpa3Personal,
    WpaEnterprise,
    Wpa2Enterprise,
    Wpa3Enterprise,
}

impl WifiSecurity {
    pub fn label(self) -> &'static str {
        match self {
            WifiSecurity::Open => "Open (no encryption)",
            WifiSecurity::EnhancedOpen => "Enhanced Open (OWE)",
            WifiSecurity::Wep => "WEP",
            WifiSecurity::WpaPersonal => "WPA Personal",
            WifiSecurity::Wpa2Personal => "WPA2 Personal",
            WifiSecurity::Wpa3Personal => "WPA3 Personal",
            WifiSecurity::WpaEnterprise => "WPA Enterprise",
            WifiSecurity::Wpa2Enterprise => "WPA2 Enterprise",
            WifiSecurity::Wpa3Enterprise => "WPA3 Enterprise",
        }
    }
}
