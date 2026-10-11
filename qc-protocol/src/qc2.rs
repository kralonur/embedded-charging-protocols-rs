//! Quick Charge 2.0, sink (portable device) side.
//!
//! Qualcomm does not publish the specification. Values come from:
//! - BC1.2: USB Battery Charging Specification Revision 1.2 with errata,
//!   March 15, 2012 (USB-IF).
//! - CHY100: Power Integrations CHY100 datasheet Rev. E, August 2018, a
//!   Quick Charge 2.0 charger interface IC ("HVDCP").
//!
//! A charger starts as a dedicated charging port, with D+ shorted to D−
//! (BC1.2 1.4.7). The device holds D+ at 0.6 V; after the glitch filter the
//! charger opens the short and pulls D− low, then reads requests from the
//! D+/D− levels (CHY100 "Quick Charge 2.0 Interface").

/// Data Detect Voltage VDAT(REF) range, in mV (CHY100 HVDCP Functions): the
/// charger reads a line below it as low and above it as at least 0.6 V.
const VDAT_REF_MIN_MV: u16 = 250;
const VDAT_REF_MAX_MV: u16 = 400;
/// Output Voltage Selection Reference VSEL(REF) range, in mV (CHY100 HVDCP
/// Functions): the charger reads a line below it as 0.6 V and above it as
/// 3.3 V.
const VSEL_REF_MIN_MV: u16 = 1_800;
const VSEL_REF_MAX_MV: u16 = 2_200;

/// D+ High Glitch Filter Time TGLITCH(BC)DONE range, in ms (CHY100): the
/// charger enters Quick Charge 2.0 once D+ stays between VDAT(REF) and
/// VSEL(REF) this long.
const ENTRY_MIN_MS: u64 = 1_000;
const ENTRY_MAX_MS: u64 = 1_500;
/// After entry, D− must stay low this long, in ms, before the charger takes
/// requests (CHY100 "Quick Charge 2.0 Interface").
const DM_LOW_MS: u64 = 1;
/// Output Voltage Glitch Filter Time TGLITCH(V)CHANGE maximum, in ms
/// (CHY100): a new request takes effect within this time. The minimum, 20 ms,
/// also filters transient level pairs while both lines change.
pub const REQUEST_SETTLE_MS: u64 = 60;

/// A data-line level from CHY100 Table 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    /// GND.
    Low,
    /// 0.6 V.
    Mid,
    /// 3.3 V.
    High,
}

impl Level {
    /// The level every charger reads a measured line voltage as, or `None`
    /// inside a comparator's tolerance, where chargers may differ.
    pub(crate) fn from_mv(mv: u16) -> Option<Self> {
        if mv < VDAT_REF_MIN_MV {
            Some(Self::Low)
        } else if mv > VDAT_REF_MAX_MV && mv < VSEL_REF_MIN_MV {
            Some(Self::Mid)
        } else if mv > VSEL_REF_MAX_MV {
            Some(Self::High)
        } else {
            None
        }
    }
}

/// An output voltage the device can request (CHY100 Table 1).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Voltage {
    #[default]
    V5,
    V9,
    V12,
    /// Class B only; a Class A charger ignores it (CHY100).
    V20,
}

/// Nominal outputs of the requests, in mV (CHY100 Table 1).
const V5_MV: u16 = 5_000;
const V9_MV: u16 = 9_000;
const V12_MV: u16 = 12_000;
const V20_MV: u16 = 20_000;

impl Voltage {
    pub const ALL: [Self; 4] = [Self::V5, Self::V9, Self::V12, Self::V20];

    /// Nominal output in mV.
    pub const fn millivolts(self) -> u16 {
        match self {
            Self::V5 => V5_MV,
            Self::V9 => V9_MV,
            Self::V12 => V12_MV,
            Self::V20 => V20_MV,
        }
    }
}

/// Why the charger did not enter Quick Charge 2.0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// D− never followed D+ to 0.6 V before the glitch filter could end, so
    /// D+ and D− are not shorted: not a dedicated charging port.
    NotDcp,
    /// D− never went low after the glitch filter: a plain BC1.2 charger.
    NoEntry,
}

/// What [`Handshake::poll`] found.
#[derive(Debug, PartialEq, Eq)]
pub enum Progress {
    /// Poll this handshake again with the next measurement.
    Waiting(Handshake),
    Ready(Session),
    Failed(Failure),
}

/// The entry handshake. The caller drives D+ to [`Level::Mid`], leaves D−
/// undriven, then polls with D− measurements until it is decided.
#[derive(Debug, PartialEq, Eq)]
pub struct Handshake {
    started_ms: u64,
    /// D− was seen at 0.6 V while D+ was held there: the BC1.2 short.
    shorted: bool,
    /// First time D− was seen low after the glitch filter could end.
    low_since_ms: Option<u64>,
}

impl Handshake {
    /// Start timing once D+ is at [`Level::Mid`].
    pub fn start(now_ms: u64) -> Self {
        Self {
            started_ms: now_ms,
            shorted: false,
            low_since_ms: None,
        }
    }

    /// Feed one D− measurement, in mV.
    pub fn poll(mut self, now_ms: u64, dm_mv: u16) -> Progress {
        let elapsed = now_ms.saturating_sub(self.started_ms);
        let level = Level::from_mv(dm_mv);
        if elapsed < ENTRY_MIN_MS {
            self.shorted |= level == Some(Level::Mid);
            return Progress::Waiting(self);
        }
        if !self.shorted {
            return Progress::Failed(Failure::NotDcp);
        }
        if level == Some(Level::Low) {
            let since = *self.low_since_ms.get_or_insert(now_ms);
            if now_ms.saturating_sub(since) >= DM_LOW_MS {
                return Progress::Ready(Session(()));
            }
        } else {
            self.low_since_ms = None;
            if elapsed > ENTRY_MAX_MS {
                return Progress::Failed(Failure::NoEntry);
            }
        }
        Progress::Waiting(self)
    }
}

/// Proof the charger entered Quick Charge 2.0, so requests can be made.
/// It stays there while D+ is above VDAT(REF); letting D+ fall below it
/// returns the charger to 5 V and BC1.2 mode (CHY100).
#[derive(Debug, PartialEq, Eq)]
pub struct Session(());

impl Session {
    /// `(D+, D−)` levels to drive for `voltage` (CHY100 Table 1); it applies
    /// within [`REQUEST_SETTLE_MS`].
    pub fn request(&self, voltage: Voltage) -> (Level, Level) {
        levels(voltage)
    }
}

/// `(D+, D−)` levels that request `voltage` (CHY100 Table 1).
pub(crate) const fn levels(voltage: Voltage) -> (Level, Level) {
    match voltage {
        Voltage::V5 => (Level::Mid, Level::Low),
        Voltage::V9 => (Level::High, Level::Mid),
        Voltage::V12 => (Level::Mid, Level::Mid),
        Voltage::V20 => (Level::High, Level::High),
    }
}
