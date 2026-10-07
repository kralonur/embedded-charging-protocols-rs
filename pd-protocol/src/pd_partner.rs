//! What the Source says about itself: the Sink's information requests (Table
//! 7.4 Recommended initiators) and their decoded answers. Every value is the
//! Source's claim, not a measurement. Pure functions; no transport or caller state.
//!
//! - Get_Source_Cap_Extended -> Source_Capabilities_Extended (sections 6.3.17,
//!   6.5.2, 7.19, PE_SNK_Get_Source_Cap_Ext 9.2.9.1.1);
//! - Get_Revision -> Revision (sections 6.3.24, 6.4.11, 7.25, PE_Get_Revision 9.2.16.1.1);
//! - Get_Manufacturer_Info -> Manufacturer_Info (sections 6.5.7, 6.5.8, 7.22,
//!   PE_Get_Manufacturer_Info 9.2.14.1.1);
//! - Get_Source_Info -> Source_Info (sections 6.3.23, 6.4.10, 7.26,
//!   PE_SNK_Get_Source_Info 9.2.10.1.1);
//! - Get_Status -> Status (sections 6.3.18, 6.5.3, 7.15, PE_Get_Status 9.2.11.1.1),
//!   once per session and again after an Alert with a non-Battery event (7.14.1).
//!
//! All exist only in PD3. Each answer fits one Chunk (MaxExtendedMsgChunkLen
//! 26 bytes), so no Chunking layer is needed.
use crate::pd_constants::SPR_MAX_W;
pub use usbpd::sink::device_policy_manager::PartnerQuery;

// SCEDB field units and limits from Table 6.50, not caller ratings.
const PEAK_OVERLOAD_MAX: u16 = 25;
const PEAK_OVERLOAD_STEP_PERCENT: u16 = 10;
const PEAK_PERIOD_STEP_MS: u16 = 20;
const PEAK_DUTY_STEP_PERCENT: u8 = 5;
const LOAD_STEP_NORMAL_MA_PER_US: u16 = 150;
const LOAD_STEP_HIGH_MA_PER_US: u16 = 500;
const IOC_NORMAL_PERCENT: u8 = 25;
const IOC_HIGH_PERCENT: u8 = 90;
const MAX_BATTERIES_PER_KIND: u8 = 4;
// EPR maximum PDP in watts (section 3.4.2).
const EPR_MAX_W: u8 = 240;

/// The order in which a session asks, once each. Get_Status is asked again
/// after Alerts.
pub const QUERIES: [PartnerQuery; 5] = [
    PartnerQuery::SourceCapabilitiesExtended,
    PartnerQuery::Revision,
    PartnerQuery::ManufacturerInfo,
    PartnerQuery::SourceInfo,
    PartnerQuery::Status,
];

/// Alert Data Object events that call for Get_Status (section 7.14.1: a
/// non-Battery status change; Table 6.25): Extended Alert (31), OVP (30),
/// Source Input Change (29), Operating Condition Change (28), OTP (27), OCP (26).
/// Battery Status Change (25) calls for Get_Battery_Status instead.
pub const fn alert_wants_status(ado: u32) -> bool {
    ado & 0xfc00_0000 != 0
}

/// What became of one request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Answer<T> {
    /// Not asked yet.
    Pending,
    Value(T),
    /// The Source answered Not_Supported.
    NotSupported,
    /// No answer within SenderResponseTimer (a normal outcome, no reset).
    Timeout,
    /// Never sent: SinkTxOK did not allow it, or a Source Message came first
    /// (sections 7.2, 7.3), in every attempt.
    Deferred,
    /// A Soft Reset ended the request before its answer; it is not repeated.
    Interrupted,
    /// Not asked: the negotiated revision is PD2, where the request does not exist.
    NotSent,
}
impl<T> Answer<T> {
    pub const fn is_pending(&self) -> bool {
        matches!(self, Self::Pending)
    }
}

/// Source Capabilities Extended Data Block (SCEDB, Table 6.50).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceExtended {
    pub vid: u16,
    pub pid: u16,
    pub xid: u32,
    pub fw_version: u8,
    pub hw_version: u8,
    /// Voltage Regulation bits 1..0: 150 or 500 mA/us load step (10b/11b are
    /// Invalid: the default 150 is assumed).
    pub load_step_ma_per_us: u16,
    /// Voltage Regulation bit 2: 25 % or 90 % IoC.
    pub ioc_percent: u8,
    /// Holdup Time in ms; 0 = not supported.
    pub holdup_ms: u8,
    /// Compliance bits 0..2.
    pub lps: bool,
    pub ps1: bool,
    pub ps2: bool,
    /// Touch Current bits 0..2: low touch current EPS, ground pin, protective earth.
    pub low_touch_current: bool,
    pub ground_pin: bool,
    pub protective_earth: bool,
    /// Peak Current 1..3 (section 6.5.2.2); all zero = no overload capability.
    pub peak: [PeakCurrent; 3],
    /// Touch Temp: 0 IEC 60950-1, 1 IEC 62368-1 TS1, 2 TS2 (Invalid -> 0).
    pub touch_temp: u8,
    /// Source Inputs: External Supply present; unconstrained (only when
    /// present, otherwise the bit is Reserved); internal Battery present.
    pub external_supply: bool,
    pub external_unconstrained: bool,
    pub internal_battery: bool,
    /// Fixed Batteries and Hot Swappable Battery Slots (5..15 are Invalid -> 0).
    pub fixed_batteries: u8,
    pub hot_swap_slots: u8,
    /// SPR Source PDP Rating in W; `None` when Invalid (101..255).
    pub spr_pdp_w: Option<u8>,
    /// EPR Source PDP Rating in W; `None` when Invalid (241..255) or absent
    /// (a 24-byte PD R3.0 block). An SPR Source sends 0.
    pub epr_pdp_w: Option<u8>,
}

/// One Peak Current field (Table 6.50, bytes 15..14 and so on).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PeakCurrent {
    /// Percent overload, 10 % steps (26..31 are Invalid and read as 25: 250 %).
    pub overload_percent: u16,
    /// Overload Period, 20 ms steps.
    pub period_ms: u16,
    /// Duty Cycle, 5 % steps.
    pub duty_percent: u8,
    /// VBUS may droop an additional 5 % during overload.
    pub droop: bool,
}
impl PeakCurrent {
    pub fn decode(raw: u16) -> Self {
        Self {
            overload_percent: u16::min(raw & 0x1f, PEAK_OVERLOAD_MAX) * PEAK_OVERLOAD_STEP_PERCENT,
            period_ms: ((raw >> 5) & 0x3f) * PEAK_PERIOD_STEP_MS,
            duty_percent: ((raw >> 11) & 0xf) as u8 * PEAK_DUTY_STEP_PERCENT,
            droop: raw & 0x8000 != 0,
        }
    }
}

impl SourceExtended {
    /// 24 or more bytes; reserved bits are ignored. `None` if shorter.
    pub fn decode(block: &[u8]) -> Option<Self> {
        if block.len() < 24 {
            return None;
        }
        let u16_at = |i: usize| u16::from_le_bytes([block[i], block[i + 1]]);
        let count = |nibble: u8| {
            if nibble <= MAX_BATTERIES_PER_KIND {
                nibble
            } else {
                0
            }
        };
        let external_supply = block[21] & 1 != 0;
        Some(Self {
            vid: u16_at(0),
            pid: u16_at(2),
            xid: u32::from_le_bytes([block[4], block[5], block[6], block[7]]),
            fw_version: block[8],
            hw_version: block[9],
            load_step_ma_per_us: if block[10] & 3 == 1 {
                LOAD_STEP_HIGH_MA_PER_US
            } else {
                LOAD_STEP_NORMAL_MA_PER_US
            },
            ioc_percent: if block[10] & 4 != 0 {
                IOC_HIGH_PERCENT
            } else {
                IOC_NORMAL_PERCENT
            },
            holdup_ms: block[11],
            lps: block[12] & 1 != 0,
            ps1: block[12] & 2 != 0,
            ps2: block[12] & 4 != 0,
            low_touch_current: block[13] & 1 != 0,
            ground_pin: block[13] & 2 != 0,
            protective_earth: block[13] & 4 != 0,
            peak: [
                PeakCurrent::decode(u16_at(14)),
                PeakCurrent::decode(u16_at(16)),
                PeakCurrent::decode(u16_at(18)),
            ],
            touch_temp: if block[20] <= 2 { block[20] } else { 0 },
            external_supply,
            external_unconstrained: external_supply && block[21] & 2 != 0,
            internal_battery: block[21] & 4 != 0,
            fixed_batteries: count(block[22] & 0xf),
            hot_swap_slots: count(block[22] >> 4),
            spr_pdp_w: (u32::from(block[23]) <= SPR_MAX_W).then_some(block[23]),
            epr_pdp_w: block.get(24).copied().filter(|&w| w <= EPR_MAX_W),
        })
    }
}

/// Revision Message Data Object (RMDO, Table 6.31): Revision and Version of the
/// highest PD specification the Source supports, e.g. 3.1 and 1.8.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Revision {
    pub major: u8,
    pub minor: u8,
    pub version_major: u8,
    pub version_minor: u8,
}
impl Revision {
    /// Bits 15..0 are Reserved and ignored.
    pub const fn decode(rmdo: u32) -> Self {
        Self {
            major: (rmdo >> 28) as u8,
            minor: ((rmdo >> 24) & 0xf) as u8,
            version_major: ((rmdo >> 20) & 0xf) as u8,
            version_minor: ((rmdo >> 16) & 0xf) as u8,
        }
    }
}

/// Manufacturer_Info Data Block (Table 6.57).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ManufacturerInfo {
    pub vid: u16,
    pub pid: u16,
    name: [u8; 22],
    name_len: u8,
}
impl ManufacturerInfo {
    /// 4..26 bytes. The string ends at its null terminator, or at the end of
    /// the block if the Source left it out; it is kept as sent.
    pub fn decode(block: &[u8]) -> Option<Self> {
        if !(4..=26).contains(&block.len()) {
            return None;
        }
        let text = &block[4..];
        let len = text.iter().position(|&b| b == 0).unwrap_or(text.len());
        let mut name = [0; 22];
        name[..len].copy_from_slice(&text[..len]);
        Some(Self {
            vid: u16::from_le_bytes([block[0], block[1]]),
            pid: u16::from_le_bytes([block[2], block[3]]),
            name,
            name_len: len as u8,
        })
    }
    /// The Manufacturer String bytes, without the terminator. Not checked to be
    /// ASCII: a renderer must map what it cannot show.
    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }
}

/// Source_Info (Tables 6.29, 6.30). SIDO2 exists from R3.2 V1.2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceInfo {
    /// Port Type: Guaranteed Capability (true) or Managed Capability (false).
    pub guaranteed: bool,
    /// Port Maximum, Present and Reported PDP in W.
    pub maximum_w: u8,
    pub present_w: u8,
    pub reported_w: u8,
    pub second: Option<SourceInfo2>,
}
/// Source_Info Data Object 2 (Table 6.30).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceInfo2 {
    /// Dynamic Power Source.
    pub dps: bool,
    /// Port Maximum and Guaranteed PDP in 0.5 W units.
    pub maximum_half_w: u16,
    pub guaranteed_half_w: u16,
}
impl SourceInfo {
    /// SIDO1, then SIDO2 if present (little-endian, 4 or 8 bytes); reserved
    /// bits are ignored. `None` for any other length.
    pub fn decode(block: &[u8]) -> Option<Self> {
        if block.len() != 4 && block.len() != 8 {
            return None;
        }
        let word =
            |i: usize| u32::from_le_bytes([block[i], block[i + 1], block[i + 2], block[i + 3]]);
        let first = word(0);
        Some(Self {
            guaranteed: first >> 31 != 0,
            maximum_w: (first >> 16) as u8,
            present_w: (first >> 8) as u8,
            reported_w: first as u8,
            second: (block.len() == 8).then(|| {
                let second = word(4);
                SourceInfo2 {
                    dps: second & 0x4000_0000 != 0,
                    maximum_half_w: ((second >> 9) & 0x1ff) as u16,
                    guaranteed_half_w: (second & 0x1ff) as u16,
                }
            }),
        })
    }
}

/// SOP Status Data Block (Table 6.51). Bytes 5 and 6 came with later
/// revisions; a shorter block leaves them `None` (section 6.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Internal Temp: 0 not supported, 1 below 2 degrees C, else degrees C.
    pub internal_temp: u8,
    /// Present Input bits 2..1: 0 internal, 1 DC external, 3 AC external (2 Invalid -> 0).
    pub external_power: u8,
    /// Present Input bits 3, 4: internal power from a Battery, from a non-Battery source.
    pub battery_powered: bool,
    pub non_battery_powered: bool,
    /// Event Flags: OCP, OTP, OVP, and PPS Current Limit mode (CV when false).
    pub ocp: bool,
    pub otp: bool,
    pub ovp: bool,
    pub current_limit: bool,
    /// Temperature Status: 0 not supported, 1 normal, 2 warning, 3 over-temperature.
    pub temperature: u8,
    /// Power Status bits 5..1, shifted down: cable, other ports, external power,
    /// event flags, temperature.
    pub power_limited: Option<u8>,
    /// Power State Change bits 2..0 (7 Invalid -> 0).
    pub power_state: Option<u8>,
}
impl Status {
    /// 5 or more bytes; reserved bits are ignored.
    pub fn decode(block: &[u8]) -> Option<Self> {
        if block.len() < 5 {
            return None;
        }
        let external = (block[1] >> 1) & 3;
        Some(Self {
            internal_temp: block[0],
            external_power: if external == 2 { 0 } else { external },
            battery_powered: block[1] & 0x08 != 0,
            non_battery_powered: block[1] & 0x10 != 0,
            ocp: block[3] & 0x02 != 0,
            otp: block[3] & 0x04 != 0,
            ovp: block[3] & 0x08 != 0,
            current_limit: block[3] & 0x10 != 0,
            temperature: (block[4] >> 1) & 3,
            power_limited: block.get(5).map(|b| (b >> 1) & 0x1f),
            power_state: block.get(6).map(|b| if b & 7 == 7 { 0 } else { b & 7 }),
        })
    }
}

/// Everything learned from the information requests in one session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartnerInfo {
    pub source_extended: Answer<SourceExtended>,
    pub revision: Answer<Revision>,
    pub manufacturer: Answer<ManufacturerInfo>,
    pub source_info: Answer<SourceInfo>,
    /// The latest Get_Status outcome, and how many Status answers arrived.
    pub status: Answer<Status>,
    pub status_reads: u8,
}
impl PartnerInfo {
    pub const PENDING: Self = Self {
        source_extended: Answer::Pending,
        revision: Answer::Pending,
        manufacturer: Answer::Pending,
        source_info: Answer::Pending,
        status: Answer::Pending,
        status_reads: 0,
    };
    /// Record a non-value outcome for `query`.
    pub fn set(&mut self, query: PartnerQuery, outcome: Answer<()>) {
        fn map<T>(outcome: Answer<()>) -> Answer<T> {
            match outcome {
                Answer::Value(()) | Answer::Pending => Answer::Pending,
                Answer::NotSupported => Answer::NotSupported,
                Answer::Timeout => Answer::Timeout,
                Answer::Deferred => Answer::Deferred,
                Answer::Interrupted => Answer::Interrupted,
                Answer::NotSent => Answer::NotSent,
            }
        }
        match query {
            PartnerQuery::SourceCapabilitiesExtended => self.source_extended = map(outcome),
            PartnerQuery::Revision => self.revision = map(outcome),
            PartnerQuery::ManufacturerInfo => self.manufacturer = map(outcome),
            PartnerQuery::SourceInfo => self.source_info = map(outcome),
            PartnerQuery::Status => self.status = map(outcome),
        }
    }
    /// Record the answer block for `query`. `false` if it does not decode.
    pub fn answer(&mut self, query: PartnerQuery, block: &[u8]) -> bool {
        match query {
            PartnerQuery::SourceCapabilitiesExtended => SourceExtended::decode(block)
                .map(|value| self.source_extended = Answer::Value(value))
                .is_some(),
            PartnerQuery::Revision => <[u8; 4]>::try_from(block)
                .ok()
                .map(|rmdo| {
                    self.revision = Answer::Value(Revision::decode(u32::from_le_bytes(rmdo)))
                })
                .is_some(),
            PartnerQuery::ManufacturerInfo => ManufacturerInfo::decode(block)
                .map(|value| self.manufacturer = Answer::Value(value))
                .is_some(),
            PartnerQuery::SourceInfo => SourceInfo::decode(block)
                .map(|value| self.source_info = Answer::Value(value))
                .is_some(),
            PartnerQuery::Status => Status::decode(block)
                .map(|value| {
                    self.status = Answer::Value(value);
                    self.status_reads = self.status_reads.saturating_add(1);
                })
                .is_some(),
        }
    }
}

/// Header bits both gates compare (`& 0xf1ff`: MessageID left out): PD3,
/// Sink/UFP, Control, no Data Objects (sections 6.3.17, 6.3.24).
pub const GET_SOURCE_CAP_EXTENDED_HEADER: u16 = 0x0091;
pub const GET_REVISION_HEADER: u16 = 0x0098;
pub const GET_SOURCE_INFO_HEADER: u16 = 0x0097;
pub const GET_STATUS_HEADER: u16 = 0x0092;
/// Get_Manufacturer_Info: Extended, one Data Object, PD3, Sink/UFP, type
/// 00110b; then Chunked Chunk 0 of 2 bytes, Target 0 (Port), Ref 0 (Table 6.56).
pub const GET_MANUFACTURER_INFO_HEADER: u16 = 0x9086;
pub const GET_MANUFACTURER_INFO_BODY: [u8; 4] = [0x02, 0x80, 0, 0];

/// The request an outgoing SOP Message is, if it is exactly one of them.
pub fn query_request(bytes: &[u8]) -> Option<PartnerQuery> {
    let header = u16::from_le_bytes([*bytes.first()?, *bytes.get(1)?]) & 0xf1ff;
    match (header, &bytes[2..]) {
        (GET_SOURCE_CAP_EXTENDED_HEADER, []) => Some(PartnerQuery::SourceCapabilitiesExtended),
        (GET_REVISION_HEADER, []) => Some(PartnerQuery::Revision),
        (GET_SOURCE_INFO_HEADER, []) => Some(PartnerQuery::SourceInfo),
        (GET_STATUS_HEADER, []) => Some(PartnerQuery::Status),
        (GET_MANUFACTURER_INFO_HEADER, body) if body == GET_MANUFACTURER_INFO_BODY => {
            Some(PartnerQuery::ManufacturerInfo)
        }
        _ => None,
    }
}

/// How a received SOP Message relates to the request awaiting its answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reply {
    /// The answer, framed exactly.
    Answer,
    /// PD3 Not_Supported.
    NotSupported,
    /// The answer's Message type, but not a valid PD3 answer: fail closed.
    Malformed,
    /// Any other Message (for a well-formed one: a Protocol Error, 9.2.5.2.1).
    Other,
}

/// Classify `frame` (Message Header and payload, no CRC) while `query` awaits
/// its answer. The answers are PD3 only and fit one Chunk: Chunked, Chunk 0,
/// no Chunk request, the Data Objects the Data Size needs (Table 6.48;
/// Extended Header bit 9 is Reserved and ignored; the 00h padding is not
/// checked, the Data Size defines the content). SCEDB 24..26 bytes (section
/// 6.5, Table 6.48 Data Size), Manufacturer_Info 4..26 bytes (Table 6.57),
/// Status 5..26 bytes (Table 6.51 and section 6.5), Revision exactly one RMDO
/// (section 6.4.11), Source_Info one or more SIDOs (SIDO2 since R3.2 V1.2;
/// objects past it are Ignored like the bytes of section 6.5).
pub fn query_answer(query: PartnerQuery, frame: &[u8]) -> Reply {
    let [low, high, ..] = *frame else {
        return Reply::Other;
    };
    let header = u16::from_le_bytes([low, high]);
    let pd3 = (header >> 6) & 3 == 2;
    let objects = ((header >> 12) & 7) as usize;
    if header & 0xf01f == 0x0010 {
        return if pd3 && frame.len() == 2 {
            Reply::NotSupported
        } else {
            Reply::Malformed
        };
    }
    let (extended, kind, sizes) = match query {
        PartnerQuery::SourceCapabilitiesExtended => (true, 1, 24..=26),
        PartnerQuery::ManufacturerInfo => (true, 7, 4..=26),
        PartnerQuery::Status => (true, 2, 5..=26),
        // With no Data Object these headers are Wait and VCONN_Swap (Table 6.5).
        PartnerQuery::Revision => (false, 12, 1..=1),
        PartnerQuery::SourceInfo => (false, 11, 1..=7),
    };
    if header & 0x801f != (u16::from(extended) << 15) | kind || (!extended && objects == 0) {
        return Reply::Other;
    }
    let valid = pd3
        && frame.len() == 2 + objects * 4
        && if extended {
            frame.len() >= 4 && {
                let extended_header = u16::from_le_bytes([frame[2], frame[3]]) & 0xfdff;
                let size = extended_header & 0x1ff;
                extended_header & 0xfe00 == 0x8000
                    && sizes.contains(&size)
                    && objects == (2 + size as usize).div_ceil(4)
            }
        } else {
            sizes.contains(&(objects as u16))
        };
    if valid {
        Reply::Answer
    } else {
        Reply::Malformed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(header: u16, block: &[u8]) -> std::vec::Vec<u8> {
        let mut frame = header.to_le_bytes().to_vec();
        frame.extend_from_slice(&[block.len() as u8, 0x80]);
        frame.extend_from_slice(block);
        while frame.len() % 4 != 2 {
            frame.push(0);
        }
        frame
    }

    #[test]
    fn scedb_decodes_every_field_and_ignores_reserved_bits() {
        let mut block = [0u8; 25];
        block[..10].copy_from_slice(&[0xd1, 0x2b, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 7, 2]);
        block[10] = 0xf9; // 500 mA/us, 25 % IoC, reserved bits set
        block[11] = 3;
        block[12] = 0xfb; // LPS, PS1, reserved
        block[13] = 0x06;
        block[14..16].copy_from_slice(&(0x8000u16 | 1 << 11 | 5 << 5 | 31).to_le_bytes());
        block[20] = 7; // Invalid touch temp -> default
        block[21] = 0x02; // unconstrained bit without an External Supply is Reserved
        block[22] = 0x52; // 2 fixed, 5 slots (Invalid -> 0)
        block[23] = 65;
        block[24] = 140;
        let source = SourceExtended::decode(&block).unwrap();
        assert_eq!(
            (
                source.vid,
                source.pid,
                source.xid,
                source.fw_version,
                source.hw_version
            ),
            (0x2bd1, 0x1234, 0x1234_5678, 7, 2)
        );
        assert_eq!(
            (
                source.load_step_ma_per_us,
                source.ioc_percent,
                source.holdup_ms
            ),
            (500, 25, 3)
        );
        assert_eq!((source.lps, source.ps1, source.ps2), (true, true, false));
        assert_eq!(
            (
                source.low_touch_current,
                source.ground_pin,
                source.protective_earth
            ),
            (false, true, true)
        );
        assert_eq!(
            source.peak[0],
            PeakCurrent {
                overload_percent: 250,
                period_ms: 100,
                duty_percent: 5,
                droop: true
            }
        );
        assert_eq!(source.peak[1], PeakCurrent::default());
        assert_eq!(source.touch_temp, 0);
        assert_eq!(
            (
                source.external_supply,
                source.external_unconstrained,
                source.internal_battery
            ),
            (false, false, false)
        );
        assert_eq!((source.fixed_batteries, source.hot_swap_slots), (2, 0));
        assert_eq!((source.spr_pdp_w, source.epr_pdp_w), (Some(65), Some(140)));
        // PD R3.0 block: no EPR byte; Invalid PDP values are ignored.
        block[23] = 101;
        assert_eq!(
            SourceExtended::decode(&block[..24]).map(|s| (s.spr_pdp_w, s.epr_pdp_w)),
            Some((None, None))
        );
        block[24] = 241;
        assert_eq!(SourceExtended::decode(&block).unwrap().epr_pdp_w, None);
        assert!(SourceExtended::decode(&block[..23]).is_none());
        // Load step 10b/11b are Invalid: the default.
        block[10] = 2;
        assert_eq!(
            SourceExtended::decode(&block).unwrap().load_step_ma_per_us,
            150
        );
    }

    #[test]
    fn revision_and_manufacturer_info_decode() {
        assert_eq!(
            Revision::decode(0x3118_ffff),
            Revision {
                major: 3,
                minor: 1,
                version_major: 1,
                version_minor: 8
            }
        );
        let info = ManufacturerInfo::decode(b"\x34\x12\x78\x56ACME 65W\0junk").unwrap();
        assert_eq!(
            (info.vid, info.pid, info.name()),
            (0x1234, 0x5678, &b"ACME 65W"[..])
        );
        // No terminator: the whole string; 22 bytes at most.
        let long = [b'A'; 26];
        assert_eq!(ManufacturerInfo::decode(&long).unwrap().name().len(), 22);
        assert_eq!(ManufacturerInfo::decode(&long[..4]).unwrap().name(), b"");
        assert!(ManufacturerInfo::decode(&long[..3]).is_none());
        assert!(ManufacturerInfo::decode(&[0; 27]).is_none());
    }

    #[test]
    fn partner_info_records_outcomes_per_request() {
        let mut info = PartnerInfo::PENDING;
        assert!(info.answer(PartnerQuery::Revision, &0x3212_0000u32.to_le_bytes()));
        assert!(!info.answer(PartnerQuery::SourceCapabilitiesExtended, &[0; 23]));
        info.set(PartnerQuery::SourceCapabilitiesExtended, Answer::Timeout);
        info.set(PartnerQuery::ManufacturerInfo, Answer::NotSupported);
        assert!(info.answer(PartnerQuery::Status, &[30, 0, 0, 0, 2]));
        assert!(info.answer(PartnerQuery::Status, &[31, 0, 0, 0, 2]));
        assert_eq!(info.status_reads, 2);
        info.set(PartnerQuery::Status, Answer::Timeout);
        assert_eq!((info.status, info.status_reads), (Answer::Timeout, 2));
        assert_eq!(
            info.revision,
            Answer::Value(Revision {
                major: 3,
                minor: 2,
                version_major: 1,
                version_minor: 2
            })
        );
        assert_eq!(info.source_extended, Answer::Timeout);
        assert_eq!(info.manufacturer, Answer::NotSupported);
    }

    #[test]
    fn source_info_and_status_decode() {
        // SIDO1 Managed 140/65/140 W, then SIDO2 DPS 140 W max, 70 W guaranteed.
        let info = SourceInfo::decode(&[0x8c, 0x41, 0x8c, 0x7f, 0x8c, 0x30, 0x02, 0xff]).unwrap();
        assert_eq!(
            (
                info.guaranteed,
                info.maximum_w,
                info.present_w,
                info.reported_w
            ),
            (false, 140, 65, 140)
        );
        assert_eq!(
            info.second,
            Some(SourceInfo2 {
                dps: true,
                maximum_half_w: 280,
                guaranteed_half_w: 140
            })
        );
        assert_eq!(
            SourceInfo::decode(&[0x8c, 0x41, 0x8c, 0x80]).map(|i| (i.guaranteed, i.second)),
            Some((true, None))
        );
        assert!(SourceInfo::decode(&[0; 5]).is_none());
        let status = Status::decode(&[40, 0xff, 0, 0xff, 0xff, 0xff, 0xff]).unwrap();
        assert_eq!(status.internal_temp, 40);
        assert_eq!(
            (
                status.external_power,
                status.battery_powered,
                status.non_battery_powered
            ),
            (3, true, true)
        );
        assert_eq!(
            (status.ocp, status.otp, status.ovp, status.current_limit),
            (true, true, true, true)
        );
        assert_eq!(
            (status.temperature, status.power_limited, status.power_state),
            (3, Some(0x1f), Some(0))
        );
        // PD R3.0 block of 5 bytes; Invalid external input bits read as internal.
        let old = Status::decode(&[1, 0x04, 0, 0, 0x02]).unwrap();
        assert_eq!(
            (old.internal_temp, old.external_power, old.temperature),
            (1, 0, 1)
        );
        assert_eq!((old.power_limited, old.power_state), (None, None));
        assert!(Status::decode(&[0; 4]).is_none());
    }

    #[test]
    fn only_non_battery_alerts_want_status() {
        for bit in 26..=31 {
            assert!(alert_wants_status(1 << bit), "{bit}");
        }
        assert!(!alert_wants_status(1 << 25 | 0xf << 20 | 0xffff));
    }

    #[test]
    fn source_info_and_status_requests_and_answers_are_framed_exactly() {
        use PartnerQuery::*;
        assert_eq!(query_request(&[0x97, 0x02]), Some(SourceInfo));
        assert_eq!(query_request(&[0x92, 0x04]), Some(Status));
        assert_eq!(query_request(&[0x57, 0x02]), None); // PD2
        // Source_Info: one, two or more SIDOs; no Data Object is a VCONN_Swap.
        assert_eq!(
            query_answer(SourceInfo, &[0xab, 0x13, 0, 0, 0, 0]),
            Reply::Answer
        );
        assert_eq!(
            query_answer(SourceInfo, &[0xab, 0x23, 0, 0, 0, 0, 0, 0, 0, 0]),
            Reply::Answer
        );
        assert_eq!(
            query_answer(
                SourceInfo,
                &[0xab, 0x33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            ),
            Reply::Answer
        );
        assert_eq!(
            query_answer(SourceInfo, &[0x6b, 0x13, 0, 0, 0, 0]),
            Reply::Malformed
        ); // PD2
        assert_eq!(
            query_answer(SourceInfo, &[0xab, 0x23, 0, 0, 0, 0]),
            Reply::Malformed
        ); // short
        assert_eq!(query_answer(SourceInfo, &[0xab, 0x03]), Reply::Other);
        // Status: 5..26 bytes in one Chunk.
        let status = |size: usize| {
            chunk(
                0x81a2 | ((2 + size as u16).div_ceil(4) << 12),
                &std::vec![0; size],
            )
        };
        assert_eq!(query_answer(Status, &status(7)), Reply::Answer);
        assert_eq!(query_answer(Status, &status(5)), Reply::Answer);
        assert_eq!(query_answer(Status, &status(4)), Reply::Malformed);
        assert_eq!(query_answer(Status, &chunk(0xf1a1, &[1; 25])), Reply::Other);
        // SCEDB
    }

    #[test]
    fn requests_are_recognized_exactly() {
        assert_eq!(
            query_request(&[0x91, 0x00]),
            Some(PartnerQuery::SourceCapabilitiesExtended)
        );
        assert_eq!(query_request(&[0x98, 0x0e]), Some(PartnerQuery::Revision)); // MessageID 7
        assert_eq!(
            query_request(&[0x86, 0x92, 2, 0x80, 0, 0]),
            Some(PartnerQuery::ManufacturerInfo)
        );
        for bad in [
            &[0x91u8, 0x01][..],
            &[0x51, 0x00],
            &[0x98, 0x00, 0, 0],
            &[0x91],
            &[0x86, 0x92, 2, 0x80, 1, 0],
            &[0x86, 0x92, 2, 0x00, 0, 0],
            &[0xb1, 0x00],
            &[0x94, 0x00],
        ] {
            assert_eq!(query_request(bad), None, "{bad:02x?}");
        }
    }

    #[test]
    fn answers_are_framed_exactly() {
        use PartnerQuery::*;
        let sce = |size: usize| chunk(0xf1a1, &std::vec![1; size]);
        assert_eq!(
            query_answer(SourceCapabilitiesExtended, &sce(25)),
            Reply::Answer
        );
        assert_eq!(
            query_answer(SourceCapabilitiesExtended, &sce(24)),
            Reply::Answer
        );
        assert_eq!(
            query_answer(SourceCapabilitiesExtended, &sce(26)),
            Reply::Answer
        );
        let mut reserved = sce(25);
        reserved[3] |= 0x02; // Extended Header bit 9
        assert_eq!(
            query_answer(SourceCapabilitiesExtended, &reserved),
            Reply::Answer
        );
        let mut malformed = std::vec![chunk(0xe1a1, &[1; 23])];
        let mut unchunked = sce(25);
        unchunked[3] &= !0x80;
        let mut request = sce(25);
        request[3] |= 0x04;
        let mut second = sce(25);
        second[3] |= 0x08;
        let mut pd2 = sce(25);
        pd2[0] = (pd2[0] & !0xc0) | 0x40;
        malformed.extend([unchunked, request, second, pd2, sce(25)[..29].to_vec()]);
        for frame in malformed {
            assert_eq!(
                query_answer(SourceCapabilitiesExtended, &frame),
                Reply::Malformed,
                "{frame:02x?}"
            );
        }
        let info = |text: &[u8]| {
            chunk(
                0x8007 | ((2 + text.len() as u16).div_ceil(4) << 12) | 0x1a0,
                text,
            )
        };
        assert_eq!(
            query_answer(ManufacturerInfo, &info(b"\x34\x12\x78\x56ACME\0")),
            Reply::Answer
        );
        assert_eq!(
            query_answer(ManufacturerInfo, &info(&[0; 4])),
            Reply::Answer
        );
        assert_eq!(
            query_answer(ManufacturerInfo, &info(&[0; 3])),
            Reply::Malformed
        );
        assert_eq!(query_answer(ManufacturerInfo, &sce(25)), Reply::Other);
        assert_eq!(
            query_answer(Revision, &[0xac, 0x13, 0, 0, 0x18, 0x31]),
            Reply::Answer
        );
        assert_eq!(
            query_answer(Revision, &[0xac, 0x23, 0, 0, 0x18, 0x31, 0, 0, 0, 0]),
            Reply::Malformed
        );
        assert_eq!(
            query_answer(Revision, &[0x6c, 0x13, 0, 0, 0x18, 0x31]),
            Reply::Malformed
        );
        assert_eq!(query_answer(Revision, &[0xac, 0x03]), Reply::Other); // Wait
        for query in QUERIES {
            assert_eq!(query_answer(query, &[0xb0, 0x03]), Reply::NotSupported);
            assert_eq!(query_answer(query, &[0x70, 0x03]), Reply::Malformed); // PD2 Not_Supported
            assert_eq!(query_answer(query, &[0xa3, 0x03]), Reply::Other); // Accept
        }
    }
}
