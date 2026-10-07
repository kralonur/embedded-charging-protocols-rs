//! Reviewed fixed/PPS SPR requests using the pinned upstream sink policy engine.
//!
//! The caller's transport must provide CRC-validated ordinary SOP packets,
//! verified automatic GoodCRC and bounded retries, and
//! permanently halt after I/O error/cancellation. The caller retains the PHY for
//! separately checked normal shutdown. Errors never authorize cleanup/recovery.
use crate::pd_constants::*;
use crate::pd_partner::{self, Answer, PartnerInfo, QUERIES, alert_wants_status};
use core::{
    cell::{Cell, RefCell},
    future::{Future, pending, poll_fn},
    marker::PhantomData,
    task::{Poll, Waker},
};
use usbpd::protocol_layer::message::{
    Message, Payload,
    data::{Data, request, sink_capabilities, source_capabilities},
    header::{ControlMessageType, DataMessageType, ExtendedMessageType, MessageType},
};
use usbpd::sink::{
    device_policy_manager::{
        DevicePolicyManager, Event, PartnerQuery, PpsStatusOutcome, QueryOutcome,
    },
    policy_engine::Sink,
};
/// Upstream timer interface implemented by the caller.
pub use usbpd::timers::Timer;
use usbpd_traits::{Driver, DriverRxError, DriverTxError};

/// Caller-supplied port identity (Sink Capabilities Extended, Table 6.61).
/// The caller owns the accuracy of vendor IDs, certification ID and versions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SinkIdentity {
    pub vid: u16,
    pub pid: u16,
    pub xid: u32,
    pub fw_version: u8,
    pub hw_version: u8,
}

impl SinkIdentity {
    /// Explicit choice for a port without assigned identity/version values:
    /// VID FFFFh, PID 0000h (section 6.1.5), XID 0 (section 7.19.4.1).
    pub const UNASSIGNED: Self = Self {
        vid: u16::MAX,
        pid: 0,
        xid: 0,
        fw_version: 0,
        hw_version: 0,
    };
}

/// Caller-selected scheduling and bounded unsent-attempt policy.
/// Durations are milliseconds; none is a measurement or transport safety check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionPolicy {
    pub pps_status_settle_ms: u64,
    pub query_settle_ms: u64,
    pub status_min_interval_ms: u64,
    pub retry_delay_ms: u64,
    pub pps_expiry_grace_ms: u64,
    pub pps_status_max_deferrals: u8,
    pub query_max_deferrals: u8,
}

/// Caller-supplied request policy. No operating limits are chosen by the library.
/// Only fixed SPR and SPR PPS are supported; this does not enable EPR/AVS.
pub trait Limits {
    /// Identity advertised by this port, supplied explicitly by the caller.
    const SINK_IDENTITY: SinkIdentity;
    const SESSION_POLICY: SessionPolicy;
    const MAX_REQUEST_MV: u32;
    const MIN_PPS_MV: u32;
    const MAX_PPS_MV: u32;
    const TARGET_MV: u32;
    /// Smallest Fixed PDO Maximum Current this sink accepts (a lower object is
    /// unusable), the Sink Minimum PDP basis and the fixed-target identity
    /// current. Not the requested current: see [`fixed_request_ma`].
    const REQUEST_MA: u32;
    /// The caller's rated maximum current in mA. Fixed Requests
    /// ask for the object's Maximum Current up to this value (Table 6.19).
    const MAX_CURRENT_MA: u32;
    /// Caller facts the library never assumes. USB Communications Capable:
    /// RDO bit 25 (Tables 6.19, 6.21) and Sink_Capabilities bit 26 (Table 6.9).
    const USB_COMMUNICATIONS_CAPABLE: bool;
    /// No USB Suspend: RDO bit 24 (Tables 6.19, 6.21).
    const NO_USB_SUSPEND: bool;
    /// Higher Capability: vSafe5V Sink PDO bit 28 (Table 6.9), the caller needs
    /// more than vSafe5V for full functionality.
    const HIGHER_CAPABILITY: bool;
    /// Unconstrained Power: Sink PDO bit 27 (Table 6.9).
    const UNCONSTRAINED_POWER: bool;
    /// SKEDB Sink Modes bits 1..4 (Table 6.61, section 7.19.4.3): VBUS, AC
    /// supply, Battery, Battery essentially unlimited. Bits 0 (PPS) and 5 (AVS)
    /// are what the library supports and are set by it.
    const SINK_POWER_MODES: u8;
    const FIXED_TARGETS: &'static [u16];
    const PPS_REFRESH_MS: u64;
    /// Truthful current SOP Status (Table 6.51), or no responder support.
    /// A provider must keep its snapshot stable during a transmission.
    fn status() -> Option<[u8; 7]> {
        None
    }
    /// Clear only acknowledged OCP/OTP/OVP event flags (Table 6.51), preserving
    /// later events and the real-time CV/CL flag.
    fn status_sent(_block: [u8; 7]) {}
}
/// SPR PPS iPpsCLMin; the minimum valid PPS operating current setpoint (1.0 A).
pub const PPS_MIN_MA: u16 = 1_000;
/// Highest Fixed operational current advertised in Sink_Capabilities: the USB
/// Type-C cable ceiling of 5 A (a protocol limit, not a caller rating).
pub const FIXED_SINK_MAX_MA: u32 = 5_000;

/// Fixed RDO Operating and Maximum Operating Current for an object advertising
/// `pdo_ma`: Table 6.19 "the highest current the Sink will draw", i.e. the
/// object's Maximum Current, at most the caller's `MAX_CURRENT_MA`; 10 mA units.
pub fn fixed_request_ma<L: Limits>(pdo_ma: u32) -> u32 {
    pdo_ma.min(L::MAX_CURRENT_MA) / FIXED_CURRENT_UNIT_MA * FIXED_CURRENT_UNIT_MA
}
/// Revision Message Data Object (Table 6.31): USB PD Revision 3.2, Version 1.2,
/// the specification this port implements. It does not claim EPR/AVS support.
pub const REVISION_DATA_OBJECT: u32 = 0x3212_0000;
/// Get_PPS_Status header bits checked by both gates (`& 0xf1ff`): Control,
/// no Data Objects, PD3, Sink/UFP, type 10100b (section 6.3.20).
pub const GET_PPS_STATUS_HEADER: u16 = 0x0094;

/// Non-Battery Sink events supported here (Table 6.25): OVP, input change,
/// thermal operating-condition change and OTP. OCP is reserved for a Sink.
pub const SINK_ALERT_EVENTS: u32 = 0x7800_0000;

// SPR PPS APDO discriminator: PDO kind and APDO type, bits 31..28 (Table 6.13).
const PPS_APDO_TYPE_MASK: u32 = 0xf000_0000;
const PPS_APDO_TYPE: u32 = 0xc000_0000;
// Fixed RDO current, PPS voltage/current and APDO voltage field widths (Tables 6.19, 6.21, 6.13).
const FIXED_RDO_CURRENT_MASK: u32 = 0x3ff;
const PPS_RDO_CURRENT_MASK: u32 = 0x7f;
const PPS_RDO_VOLTAGE_MASK: u32 = 0xfff;
const PPS_APDO_VOLTAGE_MASK: u32 = 0xff;
// RDO flags/reserved bits that must match the caller facts (Tables 6.19, 6.21).
const FIXED_RDO_FLAGS_MASK: u32 = 0x0ff0_0000;
const PPS_RDO_FLAGS_MASK: u32 = 0x0fe0_0180;
// Fixed current field maximum in mA (Table 6.19: 1023 units of 10 mA).
const MAX_FIXED_CURRENT_MA: u32 = FIXED_RDO_CURRENT_MASK * FIXED_CURRENT_UNIT_MA;
// PPS refresh requests must fit this profile's half-timeout interval in ms
// (section 7.31.12: tPPSTimeout is 10 s).
const PPS_REFRESH_MAX_MS: u64 = 5_000;
// Caller-defined power-mode bits in SKEDB; PPS/AVS bits belong to this profile (Table 6.61).
const CALLER_POWER_MODES_MASK: u8 = 0b1_1110;
// Exact response headers, excluding MessageID, for the fixed/PPS sink profile
// (Message Header Tables 6.2, 6.5, 6.47; Data Block Tables 6.51, 6.58, 6.61).
const REVISION_RESPONSE_HEADER: u16 = 0x100c;
const SKEDB_RESPONSE_HEADER: u16 = 0xf08f;
const STATUS_RESPONSE_HEADER: u16 = 0xb082;
const ALERT_RESPONSE_HEADER: u16 = 0x1086;
const PPS_STATUS_RESPONSE_HEADER: u16 = 0xa00c;
// Single Chunk 0 headers for 24-byte SKEDB and 4-byte PPSSDB (Table 6.48).
const SKEDB_CHUNK_HEADER: [u8; 2] = [0x18, 0x80];
const PPS_STATUS_CHUNK_HEADER: u16 = 0x8004;
// Extended Header bit 9 is Reserved and ignored (Table 6.48).
const EXTENDED_DEFINED_FIELDS_MASK: u16 = !(1 << 9);
// MessageID field bits 11..9 (Table 6.2).
const MESSAGE_ID_FIELD_MASK: u16 = 0x0e00;
// SOP SDB defined input/event flags and power status fields (Table 6.51).
const STATUS_INPUT_MASK: u8 = 0x1e;
const STATUS_EVENT_MASK: u8 = 0x1e;
const STATUS_POWER_MASK: u8 = 6;
const STATUS_OVER_TEMPERATURE: u8 = 6;
const STATUS_OTP: u8 = 1 << 2;
const STATUS_OVP: u8 = 1 << 3;
// BIST Data Object mode field encodings (Table 6.23).
const BIST_CARRIER_MODE: u32 = 0b0101;
const BIST_TEST_DATA_MODE: u32 = 0b1000;

/// Transmit-side SDB validation (Table 6.51): no reserved bits or Source-only
/// Power Status; OTP and over-temperature must agree. Providers own the
/// truthfulness of defined Event Flags (CV/CL is ignored outside PPS mode).
pub fn sink_status_valid(block: [u8; 7]) -> bool {
    block[1] & !STATUS_INPUT_MASK == 0
        && block[1] & 6 != 4
        && (block[1] & 8 != 0 || block[2] == 0)
        && block[3] & !STATUS_EVENT_MASK == 0
        && block[4] & !STATUS_POWER_MASK == 0
        && block[5] == 0
        && (block[3] & STATUS_OTP != 0) == (block[4] == STATUS_OVER_TEMPERATURE)
        && block[6] & 0xc0 == 0
        && block[6] & 7 != 7
        && (block[6] >> 3) & 7 <= 3
}

/// Check a reported event against the caller's current Status. Detection of a
/// real change is the provider's responsibility, never inferred from raw meters.
pub fn sink_alert_valid(ado: u32, block: [u8; 7]) -> bool {
    ado != 0
        && ado & !SINK_ALERT_EVENTS == 0
        && sink_status_valid(block)
        && (ado & (1 << 30) == 0 || block[3] & STATUS_OVP != 0)
        && (ado & (1 << 27) == 0 || block[3] & STATUS_OTP != 0)
        && (ado & (1 << 28) == 0 || block[4] != 0)
}

/// Decoded PPS Status Data Block (section 6.5.13, Table 6.58). None of the
/// fields is Static: each is a snapshot taken when the Source answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PpsStatus {
    /// Output Voltage in mV; `None` for FFFFh (not supported).
    pub mv: Option<u32>,
    /// Output Current in mA; `None` for FFh (not supported).
    pub ma: Option<u16>,
    /// Present Temperature Flag: 0 not supported, 1 normal, 2 warning, 3 over-temperature.
    pub ptf: u8,
    /// Operating Mode Flag: true in Current Limit mode, false in Constant Voltage mode.
    pub current_limit: bool,
}
impl PpsStatus {
    /// Real Time Flags bit 0 and bits 7..4 are Reserved and ignored.
    pub fn decode(block: [u8; 4]) -> Self {
        let mv = u16::from_le_bytes([block[0], block[1]]);
        Self {
            mv: (mv != u16::MAX).then_some(u32::from(mv) * PPS_RDO_VOLTAGE_UNIT_MV),
            ma: (block[2] != u8::MAX).then_some(u16::from(block[2]) * PPS_CURRENT_UNIT_MA as u16),
            ptf: (block[3] >> 1) & 3,
            current_limit: block[3] & 0x08 != 0,
        }
    }
}

/// Spec-defined handling of a partner SOP Message received while an explicit
/// contract is idle in PE_SNK_Ready. Shared by the policy wrapper and the
/// native PHY so both independently authorize the same single response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadyResponse {
    /// Existing reviewed handling (capabilities, GoodCRC, Soft_Reset) or,
    /// outside idle Ready, fail-closed.
    Policy,
    /// Accept, Reject, PS_RDY or Wait: Recognized and Supported but Unexpected
    /// in PE_SNK_Ready, a Protocol Error answered with Soft_Reset (Table 7.1).
    SoftReset,
    /// GoodCRC only; no policy response.
    Ignore,
    /// PD3 Alert: inform caller policy, no response (PE_SNK_Source_Alert_Received).
    Alert,
    /// PD3 Not_Supported, or PD2 Reject (Table 7.1; PD2 V1.3 section 6.8.1).
    Unsupported,
    /// Reject at either revision:
    /// - DR_Swap (section 7.12.1 "Shall respond by sending an Accept Message, a
    ///   Wait Message or a Reject Message"; PD2 V1.3 section 6.3.9). This UFP-only
    ///   sink never changes Data Role and never enters SOP Modal Operation, so the
    ///   section 6.3.9 Hard Reset does not apply.
    /// - VCONN_Swap (section 7.13: Reject by a recipient "that is not presently
    ///   the VCONN Source"; PD2 V1.3 section 6.3.11). Valid only while this port
    ///   does not source VCONN; the PHY checks that independently.
    Reject,
    /// A Chunk of a multi-Chunk Message: Not_Supported after
    /// ChunkingNotSupportedTimer (sections 7.31.15.1 and 9.2.7).
    UnsupportedAfterChunk,
    /// PD3 Get_Revision: Revision Message (section 7.25; Table 7.4 Required).
    Revision,
    /// Get_Sink_Cap: Sink_Capabilities (section 7.18), the objects from
    /// [`sink_capabilities()`] for the negotiated revision.
    SinkCap,
    /// PD3 Get_Sink_Cap_Extended: Sink_Capabilities_Extended (section 7.19.3;
    /// Table 7.4 Required), the SKEDB from [`sink_capabilities_extended`].
    SinkCapExtended,
    /// PD3 Get_Status -> truthful Status snapshot (sections 7.15, 9.2.11.2).
    Status,
}

/// Classify a received partner SOP Message for an idle explicit contract.
/// `revision` is the negotiated outgoing revision (1 = PD2, 2 = PD3); `header`
/// and `extended` are the received Message and Extended Message headers.
pub fn ready_response(revision: u8, header: u16, extended: Option<u16>) -> ReadyResponse {
    let pd3 = revision == crate::pd_cable::PD3_REVISION;
    if header & HEADER_EXTENDED != 0 {
        // Extended Messages do not exist in PD2; malformed frames stay fail-closed.
        let Some(extended) = extended.filter(|_| pd3) else {
            return ReadyResponse::Policy;
        };
        let chunk = (extended >> 11) & 0xf; // Chunk Number bits 14..11, Table 6.48.
        let size = extended & EXTENDED_SIZE_MASK;
        return if extended & EXTENDED_CHUNKED == 0 {
            ReadyResponse::Unsupported
        } else if chunk > 9 {
            ReadyResponse::Ignore // Table 6.48: Invalid Chunk Number.
        } else if extended & EXTENDED_REQUEST_CHUNK != 0 {
            ReadyResponse::Unsupported // Chunk request with nothing being sent.
        } else if chunk > 0 || size > MAX_CHUNK_BYTES {
            ReadyResponse::UnsupportedAfterChunk
        } else {
            ReadyResponse::Unsupported
        };
    }
    use ControlMessageType as Control;
    use DataMessageType as Data;
    match usbpd::protocol_layer::message::header::Header(header).message_type() {
        MessageType::Control(Control::GetStatus) if pd3 => ReadyResponse::Status,
        MessageType::Control(Control::GoodCRC | Control::SoftReset) => ReadyResponse::Policy,
        MessageType::Control(Control::GetSinkCap) => ReadyResponse::SinkCap,
        MessageType::Control(Control::DrSwap | Control::VconnSwap) => ReadyResponse::Reject,
        MessageType::Control(
            Control::Accept | Control::Reject | Control::PsRdy | Control::Wait,
        ) => ReadyResponse::SoftReset,
        MessageType::Control(Control::Ping) if !pd3 => ReadyResponse::Ignore,
        MessageType::Control(Control::NotSupported) if pd3 => ReadyResponse::Ignore,
        MessageType::Control(Control::GetSinkCapExtended) if pd3 => ReadyResponse::SinkCapExtended,
        MessageType::Control(Control::GetRevision) if pd3 => ReadyResponse::Revision,
        MessageType::Data(Data::SourceCapabilities) => ReadyResponse::Policy,
        MessageType::Data(Data::Bist) => ReadyResponse::Ignore,
        MessageType::Data(Data::Alert) if pd3 => ReadyResponse::Alert,
        MessageType::Data(Data::VendorDefined) if !pd3 => ReadyResponse::Ignore,
        _ => ReadyResponse::Unsupported,
    }
}

/// BIST request received while an explicit contract is idle in PE_SNK_Ready
/// (section 6.4.3, Table 6.23; PD2 V1.3 section 6.4.3, Tables 5-29 and 6-17).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BistRequest {
    /// GoodCRC only: not at vSafe5V (section 9.2.26.4; PD2 V1.3 section 6.4.3
    /// "Shall be Ignored"), Shared Test Mode Entry/Exit (shared-capacity Source
    /// ports only), a BFSK-only PD2 mode, or an Invalid mode.
    Ignore,
    /// BIST Test Data (1000b): after the GoodCRC send nothing but GoodCRC until
    /// Hard Reset (section 6.4.3.1, PE_BIST_Test_Mode section 9.2.26.4.2.1).
    TestData,
    /// BIST Carrier Mode (0101b; PD2 Carrier Mode 2), section 5.4.1: carrier
    /// for tBISTContMode, then PE_SNK_Transition_to_default without Hard Reset
    /// (section 9.2.26.4.1.1): only fresh Source_Capabilities may follow.
    CarrierMode,
}

/// Classify a received BIST Message. `header` is the Message Header, `bdo`
/// the first Data Object and `contract_object` the Object Position of the
/// idle explicit contract's RDO. The vSafe5V Fixed PDO is always object 1
/// (section 6.4.1), so a contract on object 1 is "VBUS is at vSafe5V".
pub fn bist_request(header: u16, bdo: u32, contract_object: u8) -> BistRequest {
    if header & HEADER_KIND_MASK != DataMessageType::Bist as u16
        || header & HEADER_OBJECT_COUNT_MASK == 0
        || contract_object != 1
    {
        return BistRequest::Ignore;
    }
    match bdo >> 28 {
        BIST_CARRIER_MODE => BistRequest::CarrierMode,
        BIST_TEST_DATA_MODE => BistRequest::TestData,
        _ => BistRequest::Ignore,
    }
}

/// Sink Capabilities Extended Data Block (section 6.5.16, Table 6.61), derived
/// only from the caller's request limits. `None` for invalid limits.
///
/// - VID, PID, XID and FW/HW versions from [`Limits::SINK_IDENTITY`].
/// - Load Step 150 mA/us default, no overload/droop tolerance, no LPS/PS1/PS2
///   compliance, no touch-temperature standard and no batteries are claimed
///   (the library has no battery messages).
/// - Sink Modes: caller `SINK_POWER_MODES` (bits 1..4) plus PPS supported;
///   not AVS (the library does not decode SPR AVS yet).
/// - SPR PDPs from [`spr_pdps`].
/// - EPR PDPs 0: not EPR capable (section 6.5.16.1/6.5.16.2).
pub fn sink_capabilities_extended<L: Limits>() -> Option<[u8; 24]> {
    let (minimum, operational, maximum) = spr_pdps::<L>()?;
    let mut block = [0; 24];
    let identity = L::SINK_IDENTITY;
    block[0..2].copy_from_slice(&identity.vid.to_le_bytes());
    block[2..4].copy_from_slice(&identity.pid.to_le_bytes());
    block[4..8].copy_from_slice(&identity.xid.to_le_bytes());
    block[8] = identity.fw_version;
    block[9] = identity.hw_version;
    block[10] = 1; // SKEDB Version 1.0
    block[17] = L::SINK_POWER_MODES | 0b0000_0001; // caller power modes, PPS charging supported
    block[18] = minimum;
    block[19] = operational;
    block[20] = maximum;
    Some(block)
}

/// Largest valid Maximum Current of an SPR PPS APDO: 100 x 50 mA (Tables 6.13,
/// 6.14); larger values are Invalid.
pub const PPS_APDO_MAX_MA: u16 = 5_000;

/// Highest fixed target this policy requests (at least vSafe5V, at most SPR 20 V).
fn fixed_ceiling_mv<L: Limits>() -> u32 {
    L::FIXED_TARGETS
        .iter()
        .map(|&mv| mv as u32)
        .filter(|&mv| mv <= L::MAX_REQUEST_MV.min(SPR_FIXED_MAX_MV))
        .max()
        .unwrap_or(VSAFE5V_MV)
        .max(VSAFE5V_MV)
}

/// Highest PPS output voltage this policy requests.
fn pps_ceiling_mv<L: Limits>() -> u32 {
    L::MAX_PPS_MV.min(L::MAX_REQUEST_MV).min(SPR_PPS_MAX_MV)
}

/// Fixed operational current in Sink_Capabilities: `MAX_CURRENT_MA`, at most
/// the 5 A cable ceiling ([`FIXED_SINK_MAX_MA`]).
fn fixed_sink_ma<L: Limits>() -> u32 {
    L::MAX_CURRENT_MA.min(FIXED_SINK_MAX_MA)
}

/// SPR Sink Minimum, Operational and Maximum PDP in whole watts, rounded up
/// (sections 6.5.16.1-6.5.16.3). Minimum is vSafe5V at REQUEST_MA; Operational
/// covers every mode at its lowest current (fixed targets at REQUEST_MA, PPS at
/// iPpsCLMin up to the PPS ceiling); Maximum is the larger of PPS at the largest
/// valid APDO current and the highest fixed target at the advertised fixed
/// current, capped at SPR's 100 W. `None` for invalid limits.
pub fn spr_pdps<L: Limits>() -> Option<(u8, u8, u8)> {
    if !limits_valid::<L>() {
        return None;
    }
    let watts = |mv: u32, ma: u32| (mv * ma).div_ceil(MV_MA_PER_WATT).min(SPR_MAX_W) as u8;
    let minimum = watts(VSAFE5V_MV, L::REQUEST_MA);
    let operational = watts(fixed_ceiling_mv::<L>(), L::REQUEST_MA)
        .max(watts(pps_ceiling_mv::<L>(), PPS_MIN_MA as u32));
    let maximum = operational
        .max(watts(pps_ceiling_mv::<L>(), PPS_APDO_MAX_MA as u32))
        .max(watts(fixed_ceiling_mv::<L>(), fixed_sink_ma::<L>()));
    Some((minimum, operational, maximum))
}

/// Highest PPS operating current this sink requests at `mv`: the RDO power
/// stays within the SPR Sink Maximum PDP (section 3.4.2, RDO rule 1), rounded
/// down to 50 mA, and within a valid APDO current. 0 for invalid limits or 0 mV.
pub fn pps_ceiling_ma<L: Limits>(mv: u16) -> u16 {
    let Some((_, _, maximum)) = spr_pdps::<L>() else {
        return 0;
    };
    if mv == 0 {
        return 0;
    }
    let ma =
        maximum as u32 * MV_MA_PER_WATT / mv as u32 / PPS_CURRENT_UNIT_MA * PPS_CURRENT_UNIT_MA;
    ma.min(PPS_APDO_MAX_MA as u32) as u16
}

/// Sink_Capabilities Data Objects (sections 6.4.1.2, 6.4.1.1 ordering), from
/// the same limits the Requests use, so what is advertised is what is accepted:
/// - Fixed vSafe5V at the fixed sink current (`MAX_CURRENT_MA`, at most 5 A),
///   with caller-supplied Higher Capability, Unconstrained Power and USB
///   Communications Capable (matches RDO bit 25) bits; Dual-Role Power,
///   Dual-Role Data and Fast Role Swap clear (the library is a Sink only);
/// - the other fixed targets in ascending voltage at the same current;
/// - PD3 only (APDOs do not exist in PD2): one PPS APDO (Table 6.14) from 5 V to
///   the PPS ceiling at [`pps_ceiling_ma`] there, which keeps it within the SPR
///   Sink Maximum PDP (section 3.4.2, Sink Capabilities rule 4).
///
/// Returns the objects and their count; `None` for invalid limits.
pub fn sink_capabilities<L: Limits>(pd3: bool) -> Option<([u32; MAX_OBJECTS], usize)> {
    spr_pdps::<L>()?;
    let current = fixed_sink_ma::<L>() / FIXED_CURRENT_UNIT_MA;
    let mut objects = [0; MAX_OBJECTS];
    objects[0] = u32::from(L::HIGHER_CAPABILITY) << 28
        | u32::from(L::UNCONSTRAINED_POWER) << 27
        | u32::from(L::USB_COMMUNICATIONS_CAPABLE) << 26
        | ((VSAFE5V_MV / FIXED_VOLTAGE_UNIT_MV) << 10)
        | current;
    let mut count = 1;
    let mut last = VSAFE5V_MV;
    // Ascending, without duplicates (section 6.4.1.2), within the request ceiling.
    while let Some(mv) = L::FIXED_TARGETS
        .iter()
        .map(|&mv| mv as u32)
        .filter(|&mv| {
            mv > last && mv <= fixed_ceiling_mv::<L>() && mv.is_multiple_of(FIXED_VOLTAGE_UNIT_MV)
        })
        .min()
    {
        if count == MAX_OBJECTS - 1 {
            return None;
        }
        objects[count] = ((mv / FIXED_VOLTAGE_UNIT_MV) << 10) | current;
        count += 1;
        last = mv;
    }
    let pps_mv = pps_ceiling_mv::<L>();
    if pd3 && L::MIN_PPS_MV <= VSAFE5V_MV && pps_mv >= VSAFE5V_MV {
        let ma = pps_ceiling_ma::<L>(pps_mv as u16) as u32;
        objects[count] = PPS_APDO_TYPE
            | ((pps_mv / PPS_PDO_VOLTAGE_UNIT_MV) << 17)
            | ((VSAFE5V_MV / PPS_PDO_VOLTAGE_UNIT_MV) << 8)
            | (ma / PPS_CURRENT_UNIT_MA);
        count += 1;
    }
    Some((objects, count))
}

/// Diagnostic failure; no retry/reset/recovery follows any of these. Soft
/// Reset (`docs/PD-SOFT-RESET.md`) is protocol handling, not one of these.
#[derive(Debug)]
pub enum Error {
    /// This one-shot instance has already started (including cancellation).
    Stopped,
    /// The PHY cannot supply the reviewed acknowledgement/retry behavior.
    UnsupportedPhy,
    /// Initial PDO is not a usable fixed 5V supply.
    UnsafeCapabilities,
    /// Malformed, wrong-role or out-of-scope incoming message.
    Peer,
    /// Native PHY discarded a packet; no software receive retry is attempted.
    ReceiveDiscarded,
    /// The native PHY saw Hard Reset Signaling from the Source. The session
    /// ends; the caller performs PE_SNK_Transition_to_default (section 9.2.4.9).
    HardResetReceived,
    /// Our Hard Reset Signaling was sent in a mandatory case (`docs/PD-HARD-RESET.md`).
    /// The session ends; the caller performs PE_SNK_Transition_to_default.
    HardResetSent,
    /// Native transmission failed; do not initiate another logical attempt.
    Transmit(DriverTxError),
    /// Source declined or deferred this request; do not retry it.
    Declined(ControlMessageType),
    /// A reset this profile does not perform: any reset outside a maintained
    /// session, the optional SinkWaitCapTimer Hard Reset, a Hard Reset this
    /// wrapper did not independently classify as mandatory, or traffic while one
    /// is due. Blocked before transmitting.
    RecoveryBlocked,
    /// Unexpected or unconfirmed logical transmission, blocked before native TX.
    TransmissionBlocked,
    /// Unrecoverable upstream sink-engine error.
    Engine(usbpd::sink::policy_engine::Error),
}

/// Observed Accept/PS_RDY only, not measured VBUS or an enduring contract.
#[derive(Debug, Clone)]
pub struct Contract {
    /// Source advertisements used to construct this request.
    pub capabilities: source_capabilities::SourceCapabilities,
    /// Exact upstream request object acknowledged by Accept then PS_RDY.
    pub request: ReviewedRdo,
}
/// Raw reviewed fixed or PPS RDO, interpreted against `Contract::capabilities`.
#[derive(Debug, Clone, Copy)]
pub struct ReviewedRdo(pub u32);
fn request_raw(request: &request::PowerSource) -> Option<u32> {
    match request {
        request::PowerSource::FixedVariableSupply(rdo) => Some(rdo.0),
        request::PowerSource::Pps(rdo) => Some(rdo.0),
        _ => None,
    }
}

/// PPS refresh anchor is actual verified TX, not each re-entry into Ready.
fn limits_valid<L: Limits>() -> bool {
    (VSAFE5V_MV..=u16::MAX as u32).contains(&L::MAX_REQUEST_MV)
        && (SPR_PPS_MIN_MV..=SPR_PPS_MAX_MV).contains(&L::MIN_PPS_MV)
        && L::MIN_PPS_MV <= L::MAX_PPS_MV
        && L::MAX_PPS_MV <= SPR_PPS_MAX_MV
        && (VSAFE5V_MV..=L::MAX_REQUEST_MV).contains(&L::TARGET_MV)
        && L::TARGET_MV.is_multiple_of(FIXED_VOLTAGE_UNIT_MV)
        && L::REQUEST_MA > 0
        && L::REQUEST_MA <= MAX_FIXED_CURRENT_MA
        && L::REQUEST_MA.is_multiple_of(PPS_CURRENT_UNIT_MA)
        && L::MAX_CURRENT_MA >= L::REQUEST_MA
        && L::MAX_CURRENT_MA.is_multiple_of(FIXED_CURRENT_UNIT_MA)
        && L::SINK_POWER_MODES & !CALLER_POWER_MODES_MASK == 0
        && (1..=PPS_REFRESH_MAX_MS).contains(&L::PPS_REFRESH_MS)
}

pub fn request_pps<L: Limits>(
    capabilities: &source_capabilities::SourceCapabilities,
    object: u8,
    mv: u16,
    ma: u16,
) -> Result<request::PowerSource, Error> {
    use source_capabilities::{Augmented, PowerDataObject};
    if !limits_valid::<L>()
        || capabilities.pdos().len() > MAX_OBJECTS
        || object == 0
        || object as usize > MAX_OBJECTS
        || !(L::MIN_PPS_MV..=pps_ceiling_mv::<L>()).contains(&(mv as u32))
        || !mv.is_multiple_of(PPS_RDO_VOLTAGE_UNIT_MV as u16)
        || !ma.is_multiple_of(PPS_CURRENT_UNIT_MA as u16)
        || ma < PPS_MIN_MA
        || !matches!(capabilities.pdos().first(), Some(PowerDataObject::FixedSupply(pdo))
            if u32::from(pdo.raw_voltage()) * FIXED_VOLTAGE_UNIT_MV == VSAFE5V_MV)
    {
        return Err(Error::UnsafeCapabilities);
    }
    let Some(PowerDataObject::Augmented(Augmented::Spr(pdo))) =
        capabilities.pdos().get(object as usize - 1)
    else {
        return Err(Error::UnsafeCapabilities);
    };
    let min = pdo.raw_min_voltage() as u32 * PPS_PDO_VOLTAGE_UNIT_MV;
    let max = pdo.raw_max_voltage() as u32 * PPS_PDO_VOLTAGE_UNIT_MV;
    let max_ma = pdo.raw_max_current() as u32 * PPS_CURRENT_UNIT_MA;
    // Table 6.13: bits 26..25, 16 and 7 are Reserved; receiver Shall ignore them.
    // A Maximum Current above 5 A is Invalid. The request stays within the
    // Sink Maximum PDP (section 3.4.2).
    if pdo.0 & PPS_APDO_TYPE_MASK != PPS_APDO_TYPE
        || min == 0
        || min > max
        || (mv as u32) < min
        || mv as u32 > max
        || max_ma < PPS_MIN_MA as u32
        || max_ma > PPS_APDO_MAX_MA as u32
        || (ma as u32) > max_ma
        || ma > pps_ceiling_ma::<L>(mv)
    {
        return Err(Error::UnsafeCapabilities);
    }
    Ok(request::PowerSource::Pps(
        request::Pps(0)
            .with_object_position(object)
            .with_usb_communications_capable(L::USB_COMMUNICATIONS_CAPABLE)
            .with_no_usb_suspend(L::NO_USB_SUSPEND)
            .with_raw_output_voltage(mv / PPS_RDO_VOLTAGE_UNIT_MV as u16)
            .with_raw_operating_current(ma / PPS_CURRENT_UNIT_MA as u16),
    ))
}

/// Construct a request for the single reviewed fixed target, never the highest
/// available voltage/current and never a non-fixed PDO.
pub fn request_target<L: Limits>(
    capabilities: &source_capabilities::SourceCapabilities,
) -> Result<request::PowerSource, Error> {
    request_fixed::<L>(capabilities, L::TARGET_MV)
}

pub fn request_fixed<L: Limits>(
    capabilities: &source_capabilities::SourceCapabilities,
    target_mv: u32,
) -> Result<request::PowerSource, Error> {
    if !limits_valid::<L>()
        || !(VSAFE5V_MV..=L::MAX_REQUEST_MV).contains(&target_mv)
        || !target_mv.is_multiple_of(FIXED_VOLTAGE_UNIT_MV)
        || capabilities.pdos().len() > MAX_OBJECTS
    {
        return Err(Error::UnsafeCapabilities);
    }
    // A conformant source advertises vSafe5V first; keep that safe default as a
    // precondition instead of trusting an unusual object order.
    let Some(source_capabilities::PowerDataObject::FixedSupply(first)) =
        capabilities.pdos().first()
    else {
        return Err(Error::UnsafeCapabilities);
    };
    if first.raw_voltage() as u32 * FIXED_VOLTAGE_UNIT_MV != VSAFE5V_MV {
        return Err(Error::UnsafeCapabilities);
    }
    let voltage = usbpd::units::ElectricPotential::new::<usbpd::_50millivolts_mod::_50millivolts>(
        target_mv / FIXED_VOLTAGE_UNIT_MV,
    );
    let request::PowerSource::FixedVariableSupply(rdo) = request::PowerSource::new_fixed(
        request::CurrentRequest::Highest,
        request::VoltageRequest::Specific(voltage),
        capabilities,
    )
    .map_err(|_| Error::UnsafeCapabilities)?
    else {
        return Err(Error::UnsafeCapabilities);
    };
    // Operating = Maximum Operating Current = the object's Maximum Current, at
    // most MAX_CURRENT_MA (Table 6.19). An object below REQUEST_MA is unusable.
    let pdo_ma = u32::from(rdo.raw_operating_current()) * FIXED_CURRENT_UNIT_MA;
    let raw = (fixed_request_ma::<L>(pdo_ma) / FIXED_CURRENT_UNIT_MA) as u16;
    if rdo.capability_mismatch() || pdo_ma < L::REQUEST_MA {
        return Err(Error::UnsafeCapabilities);
    }
    let rdo = rdo
        .with_raw_operating_current(raw)
        .with_raw_max_operating_current(raw);
    // Upstream defaults both flags to true; these facts must come from the caller.
    Ok(request::PowerSource::FixedVariableSupply(
        rdo.with_usb_communications_capable(L::USB_COMMUNICATIONS_CAPABLE)
            .with_no_usb_suspend(L::NO_USB_SUSPEND),
    ))
}

/// RDO bits 25 (USB Communications Capable) and 24 (No USB Suspend) from the
/// caller facts; every other flag in bits 27..20 must be clear (Tables 6.19, 6.21).
fn rdo_flags<L: Limits>() -> u32 {
    u32::from(L::USB_COMMUNICATIONS_CAPABLE) << 25 | u32::from(L::NO_USB_SUSPEND) << 24
}

/// Independent raw-RDO check for a transport before committing actual TX.
/// Caller supplies its explicit selected target; no external state is read.
pub fn review_rdo<L: Limits>(
    caps: &crate::pd_rx::Advertisements,
    rdo: u32,
    selected: Option<Target>,
) -> bool {
    if !limits_valid::<L>() || caps.count == 0 || caps.count as usize > MAX_OBJECTS {
        return false;
    }
    let Some(index) = (((rdo >> 28) & 0xf) as usize).checked_sub(1) else {
        return false;
    };
    if index >= caps.count as usize {
        return false;
    }
    let object = caps.objects[index];
    let target = selected.unwrap_or(Target::fixed(L::TARGET_MV as u16, L::REQUEST_MA as u16));
    if target.pps_object != 0 {
        let raw = object.raw;
        let min = ((raw >> 8) & PPS_APDO_VOLTAGE_MASK) * PPS_PDO_VOLTAGE_UNIT_MV;
        let max = ((raw >> 17) & PPS_APDO_VOLTAGE_MASK) * PPS_PDO_VOLTAGE_UNIT_MV;
        let max_ma = (raw & PPS_RDO_CURRENT_MASK) * PPS_CURRENT_UNIT_MA;
        let mv = ((rdo >> 9) & PPS_RDO_VOLTAGE_MASK) * PPS_RDO_VOLTAGE_UNIT_MV;
        let ma = (rdo & PPS_RDO_CURRENT_MASK) * PPS_CURRENT_UNIT_MA;
        return target.pps_object as usize == index + 1
            && caps.objects[0]
                .fixed
                .is_some_and(|(mv, _)| u32::from(mv) == VSAFE5V_MV)
            && raw & PPS_APDO_TYPE_MASK == PPS_APDO_TYPE
            && min > 0
            && min <= max
            && mv >= min
            && mv <= max
            && (L::MIN_PPS_MV..=pps_ceiling_mv::<L>()).contains(&mv)
            && mv == target.mv as u32
            && max_ma >= PPS_MIN_MA as u32
            && max_ma <= PPS_APDO_MAX_MA as u32
            && ma <= pps_ceiling_ma::<L>(mv as u16) as u32
            && ma >= PPS_MIN_MA as u32
            && ma <= max_ma
            && ma == target.ma as u32
            && rdo & PPS_RDO_FLAGS_MASK == rdo_flags::<L>();
    }
    let Some((mv, ma)) = object.fixed else {
        return false;
    };
    let current = fixed_request_ma::<L>(ma as u32) / FIXED_CURRENT_UNIT_MA;
    rdo & FIXED_RDO_FLAGS_MASK == rdo_flags::<L>()
        && mv as u32 == target.mv as u32
        && mv as u32 <= L::MAX_REQUEST_MV
        && ma as u32 >= L::REQUEST_MA
        && (rdo >> 10) & FIXED_RDO_CURRENT_MASK == current
        && rdo & FIXED_RDO_CURRENT_MASK == current
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Once,
    Maintained,
}

/// Statistics and observed state during a maintained contract session.
#[derive(Debug, Clone, Default)]
pub struct MaintainedTrace {
    pub contract: Option<Contract>,
    pub packets_serviced: u32,
    pub not_supported_sent: u32,
    pub sink_caps_sent: u32,
    pub re_requests_sent: u32,
    /// Revision Messages sent in response to Get_Revision.
    pub revisions_sent: u32,
    /// Sink_Capabilities_Extended sent in response to Get_Sink_Cap_Extended.
    pub sink_caps_extended_sent: u32,
    /// PD3 Alerts received, and the most recent Alert Data Object (Table 6.25).
    pub alerts_received: u32,
    pub sink_alerts_sent: u32,
    pub sink_alerts_deferred: u32,
    pub statuses_sent: u32,
    pub last_alert: Option<u32>,
    /// Sink-initiated Requests not sent: SinkTxNG or a SOP Message first (sections 7.2, 7.3).
    pub requests_deferred: u32,
    /// Get_PPS_Status outcomes (section 7.16): PPS_Status received, Not_Supported,
    /// SenderResponseTimer timeout, not sent (Deferred).
    pub pps_status_received: u32,
    pub pps_status_not_supported: u32,
    pub pps_status_timeouts: u32,
    pub pps_status_deferred: u32,
    /// Information request outcomes (`pd_partner`): answered, Not_Supported,
    /// SenderResponseTimer timeout, not sent (Deferred).
    pub queries_answered: u32,
    pub queries_not_supported: u32,
    pub queries_timeouts: u32,
    pub queries_deferred: u32,
    /// Soft_Reset Messages sent after a Protocol Error (section 9.2.5.2.1), and
    /// partner Soft_Resets answered with Accept (section 9.2.5.2.2).
    pub soft_resets_sent: u32,
    pub soft_resets_accepted: u32,
    /// Hard Reset Signaling sent in a mandatory case (`docs/PD-HARD-RESET.md`).
    pub hard_resets_sent: u32,
    /// BIST Test Data Mode entered (section 6.4.3.1): GoodCRC only until Hard Reset.
    pub bist_test_data: bool,
    /// BIST Carrier Mode transmissions completed (section 5.4.1).
    pub bist_carriers_sent: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Target {
    pub mv: u16,
    pub ma: u16,
    /// Zero denotes fixed supply; nonzero is the advertised APDO position.
    pub pps_object: u8,
}
impl Target {
    const fn fixed(mv: u16, ma: u16) -> Self {
        Self {
            mv,
            ma,
            pps_object: 0,
        }
    }
    const fn pps(mv: u16, ma: u16, pps_object: u8) -> Self {
        Self { mv, ma, pps_object }
    }
    fn request<L: Limits>(
        self,
        caps: &source_capabilities::SourceCapabilities,
    ) -> Result<request::PowerSource, Error> {
        if self.pps_object == 0 {
            request_fixed::<L>(caps, self.mv as u32)
        } else {
            request_pps::<L>(caps, self.pps_object, self.mv, self.ma)
        }
    }
}
/// Explicit confirmed targets and absolute PPS refreshes, consumed only while
/// native RX is parked between complete transactions. One owner, no allocation.
pub struct Selection<L: Limits> {
    limits: PhantomData<L>,
    options: Cell<[Target; 7]>,
    pps_max: Cell<[u16; 7]>,
    pps_max_ma: Cell<[u16; 7]>,
    peer_revision: Cell<u8>,
    now_ms: Cell<u64>,
    next_pps_ms: Cell<Option<u64>>,
    previous_pps_ms: Cell<Option<u64>>,

    count: Cell<usize>,
    candidate: Cell<Target>,
    target: Cell<Target>,
    established: Cell<Option<Target>>,
    request_in_flight: Cell<bool>,
    rejected: Cell<bool>,
    wait_until_ms: Cell<Option<u64>>,
    pending: Cell<Option<Target>>,
    rx_idle: Cell<bool>,
    adjust_current: Cell<bool>,
    cable_pending: Cell<bool>,
    cable_mode: Cell<Option<u16>>,
    cable_active: Cell<bool>,
    cable_result: Cell<Option<usbpd_traits::CableReport>>,
    // Get_PPS_Status for the present PPS contract: due time and unsent attempts.
    pps_status_at_ms: Cell<Option<u64>>,
    pps_status_deferrals: Cell<u8>,
    pps_status: Cell<Option<PpsStatus>>,
    // Information requests: due time of the next, its index in QUERIES, its
    // unsent attempts, and what the Source answered so far.
    query_at_ms: Cell<Option<u64>>,
    query_index: Cell<u8>,
    query_deferrals: Cell<u8>,
    // An Alert with a non-Battery event asked for Get_Status (section 7.14.1).
    status_requested: Cell<bool>,
    sink_alert_events: Cell<u32>,
    sink_alert_armed: Cell<Option<u32>>,
    sink_alert_deferred: Cell<bool>,
    partner: Cell<PartnerInfo>,
    waker: RefCell<Option<Waker>>,
}
impl<L: Limits> Selection<L> {
    pub const fn new() -> Self {
        Self {
            limits: PhantomData,
            options: Cell::new([Target::fixed(0, L::REQUEST_MA as u16); 7]),
            pps_max: Cell::new([0; 7]),
            pps_max_ma: Cell::new([0; 7]),
            peer_revision: Cell::new(0),
            now_ms: Cell::new(0),
            next_pps_ms: Cell::new(None),
            previous_pps_ms: Cell::new(None),

            count: Cell::new(0),
            candidate: Cell::new(Target::fixed(VSAFE5V_MV as u16, L::REQUEST_MA as u16)),
            target: Cell::new(Target::fixed(VSAFE5V_MV as u16, L::REQUEST_MA as u16)),
            established: Cell::new(None),
            request_in_flight: Cell::new(false),
            rejected: Cell::new(false),
            wait_until_ms: Cell::new(None),
            pending: Cell::new(None),
            rx_idle: Cell::new(false),
            adjust_current: Cell::new(false),
            cable_pending: Cell::new(false),
            cable_mode: Cell::new(None),
            cable_active: Cell::new(false),
            cable_result: Cell::new(None),
            pps_status_at_ms: Cell::new(None),
            pps_status_deferrals: Cell::new(0),
            pps_status: Cell::new(None),
            query_at_ms: Cell::new(None),
            query_index: Cell::new(0),
            query_deferrals: Cell::new(0),
            status_requested: Cell::new(false),
            sink_alert_events: Cell::new(0),
            sink_alert_armed: Cell::new(None),
            sink_alert_deferred: Cell::new(false),
            partner: Cell::new(PartnerInfo::PENDING),
            waker: RefCell::new(None),
        }
    }
    // Software lifecycle only: never authorizes resetting/reusing a PHY.
    fn end_session(&self) {
        self.count.set(0);
        self.peer_revision.set(0);
        self.next_pps_ms.set(None);
        self.previous_pps_ms.set(None);
        self.wait_until_ms.set(None);
        self.pending.set(None);
        self.rx_idle.set(false);
        self.rejected.set(true);
        self.adjust_current.set(false);
        self.cable_pending.set(false);
        self.cable_mode.set(None);
        self.cable_active.set(false);
        self.pps_status_at_ms.set(None);
        self.query_at_ms.set(None);
        self.sink_alert_events.set(0);
        self.sink_alert_armed.set(None);
        self.sink_alert_deferred.set(false);
        self.waker.borrow_mut().take();
        // Target/in-flight/established and received cable-report evidence
        // remains for diagnostics/checked finish, not permission for new work.
    }
    fn begin_session(&self) {
        self.end_session();
        self.cable_result.set(None);
        self.pps_status.set(None);
        self.partner.set(PartnerInfo::PENDING);
        self.query_index.set(0);
        self.query_deferrals.set(0);
        self.status_requested.set(false);
        self.options
            .set([Target::fixed(0, L::REQUEST_MA as u16); 7]);
        self.pps_max.set([0; 7]);
        self.pps_max_ma.set([0; 7]);
        self.now_ms.set(0);
        self.target
            .set(Target::fixed(VSAFE5V_MV as u16, L::REQUEST_MA as u16));
        self.candidate.set(self.target.get());
        self.established.set(None);
        self.request_in_flight.set(false);
        self.rejected.set(false);
    }
    pub fn target_mv(&self) -> u16 {
        self.target.get().mv
    }
    pub fn target_ma(&self) -> u16 {
        self.target.get().ma
    }
    pub fn candidate_mv(&self) -> u16 {
        self.candidate.get().mv
    }
    pub fn candidate_ma(&self) -> u16 {
        self.candidate.get().ma
    }
    pub fn target_pps_object(&self) -> u8 {
        self.target.get().pps_object
    }
    pub fn candidate_is_pps(&self) -> bool {
        self.candidate.get().pps_object != 0
    }
    pub fn adjusting_current(&self) -> bool {
        self.adjust_current.get()
    }
    pub fn set_now_ms(&self, now: u64) {
        self.now_ms.set(now);
    }
    pub fn established_target(&self) -> Option<Target> {
        self.established.get()
    }
    pub fn request_in_flight(&self) -> bool {
        self.request_in_flight.get()
    }
    fn begin_request(&self) {
        self.previous_pps_ms.set(self.next_pps_ms.get());
        self.request_in_flight.set(true);
    }
    // Back to the established target after an unsent or refused change.
    fn restore_established(&self) {
        self.request_in_flight.set(false);
        if let Some(established) = self.established.get() {
            if self.target.get() != established {
                self.next_pps_ms.set(self.previous_pps_ms.get());
            }
            self.target.set(established);
        } else {
            self.next_pps_ms.set(None);
        }
    }
    fn cool_down(&self) {
        self.wait_until_ms.set(Some(
            self.now_ms
                .get()
                .saturating_add(L::SESSION_POLICY.retry_delay_ms),
        ));
    }
    fn waiting(&self) -> bool {
        self.wait_until_ms
            .get()
            .is_some_and(|until| self.now_ms.get() < until)
    }
    pub fn note_request_sent(&self, now: u64) {
        if self.target_pps_object() != 0 {
            self.next_pps_ms.set(Some(now + L::PPS_REFRESH_MS));
        }
        // A pending fixed request does not end an established PPS contract.
    }
    pub fn refresh_due(&self) -> bool {
        !self.waiting()
            && !self.request_in_flight.get()
            && self
                .next_pps_ms
                .get()
                .is_some_and(|next| self.now_ms.get() >= next)
    }
    pub fn pps_expired(&self) -> bool {
        self.next_pps_ms.get().is_some_and(|next| {
            self.now_ms.get() >= next.saturating_add(L::SESSION_POLICY.pps_expiry_grace_ms)
        })
    }
    /// Get_PPS_Status is due: an idle PD3 PPS contract, after the caller's settle delay
    /// from establishment, and no user change, refresh or cooldown.
    pub fn pps_status_due(&self) -> bool {
        !self.waiting()
            && !self.request_in_flight.get()
            && !self.has_pending()
            && !self.cable_active.get()
            && self.peer_revision.get() == 2
            && self.target.get().pps_object != 0
            && self.established.get() == Some(self.target.get())
            && self
                .pps_status_at_ms
                .get()
                .is_some_and(|at| self.now_ms.get() >= at)
    }
    /// Most recent PPS_Status of this session, if any.
    pub fn pps_status(&self) -> Option<PpsStatus> {
        self.pps_status.get()
    }
    /// The next information request is due: an idle PD3 contract and no user
    /// change, refresh, cooldown or cable inspection.
    pub fn query_due(&self) -> bool {
        !self.waiting()
            && !self.request_in_flight.get()
            && !self.has_pending()
            && !self.cable_active.get()
            && self.peer_revision.get() == 2
            && self.established.get().is_some()
            && self.next_query().is_some()
            && self
                .query_at_ms
                .get()
                .is_some_and(|at| self.now_ms.get() >= at)
    }
    /// The request asked next, until all are done.
    /// The request asked next: the once-per-session list, then Get_Status
    /// whenever an Alert asked for it.
    pub fn next_query(&self) -> Option<PartnerQuery> {
        QUERIES
            .get(self.query_index.get() as usize)
            .copied()
            .or(self.status_requested.get().then_some(PartnerQuery::Status))
    }
    /// The current request is one of the once-per-session list (not an Alert's Get_Status).
    fn in_list(&self) -> bool {
        (self.query_index.get() as usize) < QUERIES.len()
    }
    /// What the Source answered so far in this session.
    pub fn partner_info(&self) -> PartnerInfo {
        self.partner.get()
    }
    /// Record a final outcome of the current request and move to the next.
    fn finish_query(&self, query: PartnerQuery, outcome: Answer<()>) {
        let mut partner = self.partner.get();
        partner.set(query, outcome);
        self.partner.set(partner);
        self.advance_query();
    }
    fn advance_query(&self) {
        if self.in_list() {
            self.query_index.set(self.query_index.get() + 1);
        }
        self.query_deferrals.set(0);
    }
    /// Report an independently detected non-Battery Sink status change.
    /// False means no notification was accepted.
    /// A Deferred Alert is retained but not automatically retried; another
    /// explicit report is needed to arm it again.
    pub fn notify_sink_alert(&self, ado: u32) -> bool {
        if self.peer_revision.get() != 2
            || self.established.get().is_none()
            || self.sink_alert_armed.get().is_some()
            || !L::status().is_some_and(|block| sink_alert_valid(ado, block))
        {
            return false;
        }
        self.sink_alert_events
            .set(self.sink_alert_events.get() | ado);
        self.sink_alert_deferred.set(false);
        if let Some(waker) = self.waker.borrow().as_ref() {
            waker.wake_by_ref();
        }
        true
    }
    /// Exact ADO authorized by policy, for the independent PHY gate.
    pub fn armed_sink_alert(&self) -> Option<u32> {
        self.sink_alert_armed.get()
    }
    pub fn pending_sink_alert(&self) -> u32 {
        self.sink_alert_events.get()
    }
    pub fn sink_alert_due(&self) -> bool {
        self.sink_alert_events.get() != 0
            && !self.sink_alert_deferred.get()
            && self.sink_alert_armed.get().is_none()
            && self.peer_revision.get() == 2
            && self.established.get().is_some()
            && !self.request_in_flight.get()
            && !self.waiting()
            && !self.has_pending()
            && !self.refresh_due()
            && !self.cable_active.get()
    }
    pub fn has_pending(&self) -> bool {
        self.pending.get().is_some() || self.cable_pending.get()
    }
    pub fn cable_active(&self) -> bool {
        self.cable_active.get()
    }
    pub fn cable_result(&self) -> Option<usbpd_traits::CableReport> {
        self.cable_result.get()
    }
    /// Explicit discovery, only from PD3 fixed 5V; never from PPS.
    pub fn request_cable(&self) -> bool {
        if self.target.get() != Target::fixed(VSAFE5V_MV as u16, L::REQUEST_MA as u16)
            || self.candidate.get() != self.target.get()
            || self.peer_revision.get() != 2
            || self.has_pending()
            || self.request_in_flight.get()
            || self.waiting()
            || self.cable_active.get()
        {
            return false;
        }
        self.cable_mode.set(None);
        self.cable_pending.set(true);
        if let Some(waker) = self.waker.borrow().as_ref() {
            waker.wake_by_ref();
        }
        true
    }
    /// Same explicit Ready handoff and fixed5V gates as full discovery.
    pub fn request_cable_mode(&self, svid: u16) -> bool {
        if svid == 0 || !self.request_cable() {
            return false;
        }
        self.cable_mode.set(Some(svid));
        true
    }
    pub fn cable_mode(&self) -> Option<u16> {
        self.cable_mode.get()
    }
    pub fn set_rx_idle(&self, idle: bool) {
        self.rx_idle.set(idle);
        if idle && let Some(waker) = self.waker.borrow().as_ref() {
            waker.wake_by_ref();
        }
    }
    fn advertise(&self, capabilities: &source_capabilities::SourceCapabilities) {
        let mut options = [Target::fixed(0, L::REQUEST_MA as u16); 7];
        let mut pps_max = [0; 7];
        let mut pps_max_ma = [0; 7];
        let mut count = 0;
        for (index, pdo) in capabilities.pdos().iter().enumerate() {
            if let source_capabilities::PowerDataObject::FixedSupply(pdo) = pdo {
                let mv = pdo.raw_voltage() as u32 * FIXED_VOLTAGE_UNIT_MV;
                if L::FIXED_TARGETS.contains(&(mv as u16))
                    && mv <= L::MAX_REQUEST_MV
                    && pdo.raw_max_current() as u32 * FIXED_CURRENT_UNIT_MA >= L::REQUEST_MA
                    && !options[..count].contains(&Target::fixed(mv as u16, L::REQUEST_MA as u16))
                    && count < options.len()
                {
                    options[count] = Target::fixed(mv as u16, L::REQUEST_MA as u16);
                    count += 1;
                }
            } else if let source_capabilities::PowerDataObject::Augmented(
                source_capabilities::Augmented::Spr(pdo),
            ) = pdo
            {
                let min = (pdo.raw_min_voltage() as u16 * PPS_PDO_VOLTAGE_UNIT_MV as u16)
                    .max(L::MIN_PPS_MV as u16);
                let max = (pdo.raw_max_voltage() as u16 * PPS_PDO_VOLTAGE_UNIT_MV as u16)
                    .min(pps_ceiling_mv::<L>() as u16);
                let max_ma = pdo.raw_max_current() as u16 * PPS_CURRENT_UNIT_MA as u16;
                let start_ma = max_ma.min(pps_ceiling_ma::<L>(min));
                if self.peer_revision.get() == 2
                    && min <= max
                    && max_ma >= PPS_MIN_MA
                    && count < 7
                    && request_pps::<L>(capabilities, index as u8 + 1, min, start_ma).is_ok()
                {
                    options[count] = Target::pps(min, start_ma, index as u8 + 1);
                    pps_max[index] = max;
                    pps_max_ma[index] = max_ma;
                    count += 1;
                }
            }
        }
        self.options.set(options);
        self.pps_max.set(pps_max);
        self.pps_max_ma.set(pps_max_ma);
        self.count.set(count);
        let candidate = self.candidate.get();
        if !options[..count].iter().any(|option| {
            *option == candidate
                || (candidate.pps_object != 0
                    && option.pps_object == candidate.pps_object
                    && candidate.mv >= option.mv
                    && candidate.mv <= pps_max[candidate.pps_object as usize - 1]
                    && candidate.ma >= PPS_MIN_MA
                    && candidate.ma <= pps_max_ma[candidate.pps_object as usize - 1]
                    && candidate.ma <= pps_ceiling_ma::<L>(candidate.mv))
        }) {
            self.candidate.set(options[0]);
            self.adjust_current.set(false);
        }
    }
    pub fn now_ms(&self) -> u64 {
        self.now_ms.get()
    }
    pub fn option_count(&self) -> usize {
        self.count.get()
    }
    pub fn option(&self, index: usize) -> Option<Target> {
        (index < self.count.get()).then(|| self.options.get()[index])
    }
    /// Set a preview from semantic values, not buttons; confirmation is separate.
    pub fn preview(&self, target: Target) -> bool {
        if self.has_pending() || self.request_in_flight.get() || self.cable_active() {
            return false;
        }
        let options = self.options.get();
        let valid = options[..self.count.get()].iter().any(|option| {
            if target.pps_object == 0 {
                *option == target
            } else {
                let Some(index) = target
                    .pps_object
                    .checked_sub(1)
                    .map(usize::from)
                    .filter(|i| *i < 7)
                else {
                    return false;
                };
                option.pps_object == target.pps_object
                    && target.mv >= option.mv
                    && target.mv <= self.pps_max.get()[index]
                    && target.mv.is_multiple_of(20)
                    && target.ma >= PPS_MIN_MA
                    && target.ma <= self.pps_max_ma.get()[index]
                    && target.ma <= pps_ceiling_ma::<L>(target.mv)
                    && target.ma.is_multiple_of(PPS_CURRENT_UNIT_MA as u16)
            }
        });
        if valid {
            self.candidate.set(target);
            self.adjust_current.set(false);
        }
        valid
    }
    /// Explicitly confirm a preview; browsing alone never schedules transmission.
    pub fn confirm(&self) -> bool {
        if self.has_pending()
            || self.request_in_flight.get()
            || self.cable_active()
            || self.rejected.get()
            || self.waiting()
            || self.candidate.get() == self.target.get()
        {
            return false;
        }
        self.pending.set(Some(self.candidate.get()));
        if let Some(waker) = self.waker.borrow().as_ref() {
            waker.wake_by_ref();
        }
        true
    }
    pub fn toggle_adjustment(&self) {
        if self.candidate_is_pps()
            && !self.has_pending()
            && !self.request_in_flight.get()
            && !self.cable_active()
        {
            self.adjust_current.set(!self.adjust_current.get());
        }
    }
    /// Advance a preview using protocol-aligned caller-specified increments.
    /// No button IDs, hold durations, screen numbers or rendering are involved.
    pub fn advance(&self, voltage_step: u16, current_step: u16) {
        let count = self.count.get();
        if count == 0
            || self.has_pending()
            || self.request_in_flight.get()
            || self.cable_active()
            || voltage_step == 0
            || !voltage_step.is_multiple_of(20)
            || current_step == 0
            || !current_step.is_multiple_of(PPS_CURRENT_UNIT_MA as u16)
        {
            return;
        }
        let candidate = self.candidate.get();
        let options = self.options.get();
        if candidate.pps_object != 0 {
            let obj_idx = candidate.pps_object as usize - 1;
            let max_mv = self.pps_max.get()[obj_idx];
            let max_ma = self.pps_max_ma.get()[obj_idx];
            if self.adjust_current.get() {
                let next_ma = candidate.ma.saturating_add(current_step);
                let ma = if next_ma > max_ma.min(pps_ceiling_ma::<L>(candidate.mv)) {
                    PPS_MIN_MA
                } else {
                    next_ma
                };
                self.candidate.set(Target { ma, ..candidate });
            } else if candidate.mv < max_mv {
                // A higher voltage lowers the current that stays within the Sink Maximum PDP.
                let mv = candidate.mv.saturating_add(voltage_step).min(max_mv);
                self.candidate.set(Target {
                    mv,
                    ma: candidate.ma.min(pps_ceiling_ma::<L>(mv)),
                    ..candidate
                });
            } else {
                let index = options[..count]
                    .iter()
                    .position(|option| {
                        *option == candidate || option.pps_object == candidate.pps_object
                    })
                    .unwrap_or(0);
                self.candidate.set(options[(index + 1) % count]);
                self.adjust_current.set(false);
            }
        } else {
            let index = options[..count]
                .iter()
                .position(|option| *option == candidate)
                .unwrap_or(0);
            self.candidate.set(options[(index + 1) % count]);
            self.adjust_current.set(false);
        }
    }
}
// Cancellation/return revokes software work without touching the driver's bus.
struct SelectionSession<'a, L: Limits>(&'a Selection<L>);
impl<L: Limits> Drop for SelectionSession<'_, L> {
    fn drop(&mut self) {
        self.0.end_session();
    }
}

impl<L: Limits> Default for Selection<L> {
    fn default() -> Self {
        Self::new()
    }
}

/// SOP Soft Reset progress (sections 7.7, 9.2.5.2). Each phase admits exactly
/// one next step; anything else needs a Hard Reset and is blocked.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Reset {
    Idle,
    /// A Protocol Error was seen: the next transmission is our Soft_Reset.
    Send,
    /// Our Soft_Reset was sent: only Accept (or the partner's Soft_Reset).
    AwaitAccept,
    /// A partner Soft_Reset was received: the next transmission is our Accept.
    Accept,
    /// Either reset completed: only fresh Source_Capabilities may follow.
    Capabilities,
    /// BIST Carrier Mode ended in PE_SNK_Transition_to_default without Hard
    /// Reset (section 9.2.26.4.1.1): only fresh Source_Capabilities may follow.
    /// Nothing else is a Shall Hard Reset case here, so anything else fails closed.
    Default,
}

struct State<'a, L: Limits> {
    reset: Reset,
    // From a completed reset until the renegotiated Request ends (PS_RDY, or
    // Reject/Wait under the kept contract). A further Protocol Error then means
    // the Soft Reset did not correct it: Hard Reset (section 7.1.1).
    recovering: bool,
    // A mandatory Hard Reset is due (`docs/PD-HARD-RESET.md` B1): it is the only
    // permitted next step.
    hard_reset_armed: bool,
    request_in_flight: bool,
    // Accept received for the Request in flight: power is transitioning
    // (PE_SNK_Transition_Sink); only PS_RDY may follow (Table 7.1).
    transitioning: bool,
    partner_revision: u8,
    unsupported_reply: Option<ControlMessageType>,
    /// BIST Test Data Mode (section 6.4.3.1): every received Message is
    /// discarded after its GoodCRC and nothing is transmitted until Hard Reset.
    bist_test_data: bool,
    /// A received BIST Carrier Mode at vSafe5V armed one carrier transmission.
    bist_carrier: bool,
    revision_reply: bool,
    sink_caps_reply: bool,
    skedb_reply: bool,
    status_reply: Option<[u8; 7]>,
    sink_alert: Option<u32>,
    sink_alert_pending: bool,
    // A received Source_Capabilities not yet answered: that Request is a
    // response, never the start of a Sink-initiated AMS (section 7.2).
    caps_reply: bool,
    // Get_PPS_Status armed by policy, and sent and not yet answered.
    pps_status_query: bool,
    pps_status_pending: bool,
    // An information request armed by policy, and sent and not yet answered.
    query: Option<PartnerQuery>,
    query_pending: Option<PartnerQuery>,
    selected: Option<Contract>,
    established: Option<Contract>,
    outcome: Option<Result<Contract, Error>>,
    sent: bool,
    mode: Mode,
    trace: Option<&'a RefCell<MaintainedTrace>>,
    selection: Option<&'a Selection<L>>,
}
impl<L: Limits> State<'_, L> {
    fn fail(&mut self, error: Error) {
        if self.outcome.is_none() {
            self.outcome = Some(Err(error));
        }
    }
    /// Abandon the AMS in progress and queued work for a Soft Reset. Power and
    /// the established contract are unchanged (section 7.1.1); the renegotiation
    /// requests the established target again, or the initial target before one.
    fn begin_reset(&mut self, reset: Reset) {
        self.reset = reset;
        self.request_in_flight = false;
        self.transitioning = false;
        self.caps_reply = false;
        self.pps_status_query = false;
        self.unsupported_reply = None;
        self.revision_reply = false;
        self.sink_caps_reply = false;
        self.skedb_reply = false;
        self.status_reply = None;
        self.sink_alert = None;
        self.sink_alert_pending = false;
        if let Some(selection) = self.selection {
            // Do not blindly resend an interrupted Alert after reset.
            if selection.sink_alert_armed.take().is_some() {
                selection.sink_alert_deferred.set(true);
            }
        }
        self.selected = self.established.clone();
        let interrupted_query = core::mem::take(&mut self.pps_status_pending);
        self.query = None;
        let interrupted_request = self.query_pending.take();
        if let Some(selection) = self.selection {
            if interrupted_query {
                // The interrupted query is not repeated for this contract.
                selection.pps_status_at_ms.set(None);
            }
            if let Some(query) = interrupted_request {
                // Nor an interrupted information request.
                selection.finish_query(query, Answer::Interrupted);
            }
            selection.pending.set(None);
            selection.cable_pending.set(false);
            selection.cable_mode.set(None);
            if selection.request_in_flight.get() {
                selection.restore_established();
            } else if let Some(established) = selection.established.get() {
                selection.target.set(established);
            }
            selection.begin_request();
        }
    }
    /// A Protocol Error (Table 7.1, section 9.2.5.2.1): permit one Soft_Reset in
    /// a maintained session after the first Request fixed the revision. While
    /// recovering it means the Soft Reset did not correct the error: Hard Reset
    /// (section 7.1.1).
    fn protocol_error(&mut self) -> bool {
        if self.mode != Mode::Maintained || !self.sent {
            self.fail(Error::RecoveryBlocked);
            return false;
        }
        if self.recovering || self.reset != Reset::Idle {
            return self.require_hard_reset();
        }
        self.begin_reset(Reset::Send);
        true
    }
    /// A mandatory Hard Reset case (`docs/PD-HARD-RESET.md` B1) in a maintained
    /// session after the first Request or a validated startup reset exchange:
    /// the upstream engine must ask for it next (sections 7.7, 9.2.5.2.2).
    fn require_hard_reset(&mut self) -> bool {
        if self.mode != Mode::Maintained || (!self.sent && self.reset != Reset::Capabilities) {
            self.fail(Error::RecoveryBlocked);
            return false;
        }
        self.hard_reset_armed = true;
        true
    }
    /// The renegotiated Request ended normally: the reset corrected the error.
    fn request_ended(&mut self) {
        self.request_in_flight = false;
        self.transitioning = false;
        self.recovering = false;
    }
}

/// Pure one-shot or maintained guard; creating another instance is not transport recovery.
pub struct Trial<L: Limits> {
    started: bool,
    limits: PhantomData<L>,
}
impl<L: Limits> Trial<L> {
    /// Unstarted one-shot trial. Stops after PS_RDY before entering Ready.
    pub const fn once() -> Self {
        Self {
            started: false,
            limits: PhantomData,
        }
    }
    /// Unstarted maintained session. Enters Ready to service traffic at the selected voltage.
    pub const fn maintained() -> Self {
        Self {
            started: false,
            limits: PhantomData,
        }
    }
    /// Run upstream Source_Capabilities -> Request -> Accept -> PS_RDY states.
    /// Stop at its post-PS_RDY callback before entering normal Ready servicing.
    /// No resets, Get_Source_Cap, EPR, VCONN, swaps or software request retries.
    /// The transport may retry the same Request within its separately reviewed bound.
    pub async fn run<D: Driver, T: Timer>(&mut self, driver: &mut D) -> Result<Contract, Error> {
        if self.started {
            return Err(Error::Stopped);
        }
        self.started = true;
        if !D::HAS_AUTO_GOOD_CRC || !D::HAS_AUTO_RETRY {
            return Err(Error::UnsupportedPhy);
        }
        let shared = RefCell::new(State::<L> {
            reset: Reset::Idle,
            recovering: false,
            hard_reset_armed: false,
            request_in_flight: false,
            transitioning: false,
            partner_revision: 0,
            unsupported_reply: None,
            bist_test_data: false,
            bist_carrier: false,
            revision_reply: false,
            sink_caps_reply: false,
            skedb_reply: false,
            caps_reply: false,
            pps_status_query: false,
            pps_status_pending: false,
            query: None,
            query_pending: None,
            selected: None,
            established: None,
            outcome: None,
            sent: false,
            mode: Mode::Once,
            status_reply: None,
            sink_alert: None,
            sink_alert_pending: false,
            trace: None,
            selection: None,
        });
        let mut sink = Sink::<_, T, _>::new(
            Borrowed {
                driver,
                shared: &shared,
            },
            Policy { shared: &shared },
        );
        let mut engine = core::pin::pin!(sink.run());
        // Callbacks stop by setting an outcome and pending without I/O. Observe
        // it in this same poll; never let upstream enter retry/reset/Ready paths.
        poll_fn(|cx| {
            let result = engine.as_mut().poll(cx);
            if let Some(outcome) = shared.borrow_mut().outcome.take() {
                return Poll::Ready(outcome);
            }
            match result {
                Poll::Ready(Err(error)) => Poll::Ready(Err(Error::Engine(error))),
                Poll::Ready(Ok(())) => Poll::Ready(Err(Error::Stopped)),
                Poll::Pending => Poll::Pending,
            }
        })
        .await
    }

    /// Run a maintained session that enters Ready and handles incoming source traffic.
    /// Services reviewed Soft_Reset exchanges and mandatory Hard Reset cases;
    /// blocks optional recovery, EPR and unreviewed requests. A Hard Reset ends
    /// the session; transition to default remains caller-owned.
    /// Updates `trace` with contract and counters.
    pub async fn run_maintained<D: Driver, T: Timer>(
        &mut self,
        driver: &mut D,
        trace: &RefCell<MaintainedTrace>,
    ) -> Result<(), Error> {
        self.run_controlled::<D, T>(driver, trace, None).await
    }

    /// Start a fresh logical session, clearing old selection/trace on first
    /// poll. The driver must already own a separately reviewed clean boundary.
    /// Return/cancellation revokes queued work; it never restores a PHY.
    pub async fn run_selectable<D: Driver, T: Timer>(
        &mut self,
        driver: &mut D,
        trace: &RefCell<MaintainedTrace>,
        selection: &Selection<L>,
    ) -> Result<(), Error> {
        self.run_controlled::<D, T>(driver, trace, Some(selection))
            .await
    }

    async fn run_controlled<D: Driver, T: Timer>(
        &mut self,
        driver: &mut D,
        trace: &RefCell<MaintainedTrace>,
        selection: Option<&Selection<L>>,
    ) -> Result<(), Error> {
        if self.started {
            return Err(Error::Stopped);
        }
        self.started = true;
        *trace.borrow_mut() = MaintainedTrace::default();
        let _session = selection.map(|selection| {
            selection.begin_session();
            SelectionSession(selection)
        });
        if !D::HAS_AUTO_GOOD_CRC || !D::HAS_AUTO_RETRY {
            return Err(Error::UnsupportedPhy);
        }
        let shared = RefCell::new(State {
            reset: Reset::Idle,
            recovering: false,
            hard_reset_armed: false,
            request_in_flight: false,
            transitioning: false,
            partner_revision: 0,
            unsupported_reply: None,
            bist_test_data: false,
            bist_carrier: false,
            revision_reply: false,
            sink_caps_reply: false,
            skedb_reply: false,
            caps_reply: false,
            pps_status_query: false,
            pps_status_pending: false,
            query: None,
            query_pending: None,
            selected: None,
            established: None,
            outcome: None,
            sent: false,
            mode: Mode::Maintained,
            status_reply: None,
            sink_alert: None,
            sink_alert_pending: false,
            trace: Some(trace),
            selection,
        });
        let mut sink = Sink::<_, T, _>::new(
            Borrowed {
                driver,
                shared: &shared,
            },
            Policy { shared: &shared },
        );
        let mut engine = core::pin::pin!(sink.run());
        poll_fn(|cx| {
            let result = engine.as_mut().poll(cx);
            if let Some(outcome) = shared.borrow_mut().outcome.take() {
                return Poll::Ready(outcome.map(|_| ()));
            }
            match result {
                Poll::Ready(Err(error)) => Poll::Ready(Err(Error::Engine(error))),
                Poll::Ready(Ok(())) => Poll::Ready(Err(Error::Stopped)),
                Poll::Pending => Poll::Pending,
            }
        })
        .await
    }
}

struct Policy<'a, 'b, L: Limits> {
    shared: &'a RefCell<State<'b, L>>,
}
impl<L: Limits> DevicePolicyManager for Policy<'_, '_, L> {
    // No reviewed need to receive any multi-Chunk Message (section 7.31.15).
    const CHUNKING: bool = false;
    // Fixed/PPS SPR only; the profile does not support EPR.
    const EPR: bool = false;

    fn revision(&self) -> u32 {
        REVISION_DATA_OBJECT
    }

    fn sink_capabilities_extended(&self) -> Option<[u8; 24]> {
        sink_capabilities_extended::<L>()
    }

    fn status(&self) -> Option<[u8; 7]> {
        L::status().filter(|&block| sink_status_valid(block))
    }
    async fn status_sent(&mut self, block: [u8; 7]) {
        L::status_sent(block);
        if let Some(trace) = self.shared.borrow().trace {
            trace.borrow_mut().statuses_sent += 1;
        }
    }
    async fn sink_alert_sent(&mut self, ado: u32) {
        let mut state = self.shared.borrow_mut();
        if !state.sink_alert_pending || state.sink_alert != Some(ado) {
            state.fail(Error::TransmissionBlocked);
            return;
        }
        state.sink_alert = None;
        if let Some(selection) = state.selection {
            selection
                .sink_alert_events
                .set(selection.sink_alert_events.get() & !ado);
            selection.sink_alert_armed.set(None);
        }
        if let Some(trace) = state.trace {
            trace.borrow_mut().sink_alerts_sent += 1;
        }
    }
    async fn sink_alert_not_sent(&mut self, ado: u32) {
        let mut state = self.shared.borrow_mut();
        // Only proven Deferred is expected: an engine PD2/no-Status refusal
        // before transmission disagrees with this policy's authorization.
        if !state.sink_alert_pending || state.sink_alert != Some(ado) {
            state.fail(Error::TransmissionBlocked);
            return;
        }
        state.sink_alert = None;
        state.sink_alert_pending = false;
        if let Some(selection) = state.selection {
            selection.sink_alert_armed.set(None);
            selection.sink_alert_deferred.set(true);
        }
        if let Some(trace) = state.trace {
            trace.borrow_mut().sink_alerts_deferred += 1;
        }
    }
    async fn sink_alert_finished(&mut self) {
        self.shared.borrow_mut().sink_alert_pending = false;
    }

    async fn alert(&mut self, alert_data_object: u32) {
        if let Some(trace) = self.shared.borrow().trace {
            let mut trace = trace.borrow_mut();
            trace.alerts_received = trace.alerts_received.saturating_add(1);
            trace.last_alert = Some(alert_data_object);
        }
        // PE_SNK_Source_Alert_Received (section 9.2.8.2.1): the DPM requests
        // status for a non-Battery event (section 7.14.1 Should).
        if alert_wants_status(alert_data_object)
            && let Some(selection) = self.shared.borrow().selection
        {
            selection.status_requested.set(true);
        }
    }

    async fn request(
        &mut self,
        capabilities: &source_capabilities::SourceCapabilities,
    ) -> request::PowerSource {
        let selection = self.shared.borrow().selection;
        if let Some(selection) = selection {
            // A fresh Source_Capabilities starts a new negotiation, unlike the
            // Ready event callback which also refreshes the preview options.
            selection.rejected.set(false);
            selection.wait_until_ms.set(None);
            selection.begin_request();
            selection.advertise(capabilities);
        }
        let target = selection.map_or(
            Target::fixed(L::TARGET_MV as u16, L::REQUEST_MA as u16),
            |selection| selection.target.get(),
        );
        if target.pps_object != 0 && selection.is_none_or(|s| s.peer_revision.get() != 2) {
            self.shared.borrow_mut().fail(Error::UnsafeCapabilities);
            return pending().await;
        }
        match target.request::<L>(capabilities) {
            Ok(request) => {
                // Target::request only constructs fixed or PPS requests.
                self.shared.borrow_mut().selected = Some(Contract {
                    capabilities: capabilities.clone(),
                    request: ReviewedRdo(
                        request_raw(&request).expect("target request must be fixed or PPS"),
                    ),
                });
                request
            }
            _ => {
                self.shared.borrow_mut().fail(Error::UnsafeCapabilities);
                pending().await
            }
        }
    }
    async fn transition_power(&mut self, accepted: &request::PowerSource) {
        let should_wait = {
            let matches = self
                .shared
                .borrow()
                .selected
                .as_ref()
                .is_some_and(|selected| request_raw(accepted) == Some(selected.request.0));
            let mode = self.shared.borrow().mode;
            let mut state = self.shared.borrow_mut();
            if matches && state.sent {
                // `matches` above requires selected to contain this exact request.
                let contract = state
                    .selected
                    .clone()
                    .expect("matched request must have a selected contract");
                state.established = Some(contract.clone());
                state.request_ended();
                if let Some(selection) = state.selection {
                    let previous = selection.established.replace(Some(selection.target.get()));
                    selection.request_in_flight.set(false);
                    if selection.target_pps_object() == 0 {
                        selection.next_pps_ms.set(None);
                        selection.pps_status_at_ms.set(None);
                    } else if previous != Some(selection.target.get()) {
                        // Section 7.16: query once per new PPS contract, never per refresh.
                        selection.pps_status_at_ms.set(Some(
                            selection
                                .now_ms
                                .get()
                                .saturating_add(L::SESSION_POLICY.pps_status_settle_ms),
                        ));
                        selection.pps_status_deferrals.set(0);
                    }
                    // Information requests once per session, from its first contract;
                    // they do not exist in PD2 (Table 6.5).
                    if previous.is_none() {
                        if selection.peer_revision.get() == 2 {
                            selection.query_at_ms.set(Some(
                                selection
                                    .now_ms
                                    .get()
                                    .saturating_add(L::SESSION_POLICY.query_settle_ms),
                            ));
                        } else {
                            for query in QUERIES {
                                selection.finish_query(query, Answer::NotSent);
                            }
                        }
                    }
                }
                if let Some(trace) = state.trace {
                    trace.borrow_mut().contract = Some(contract.clone());
                }
                if mode == Mode::Once {
                    state.outcome = Some(Ok(contract));
                    true
                } else {
                    false
                }
            } else {
                state.fail(Error::TransmissionBlocked);
                true
            }
        };
        if should_wait {
            pending::<()>().await;
        }
    }
    async fn request_declined(
        &mut self,
        requested: &request::PowerSource,
        response: ControlMessageType,
    ) -> bool {
        let mut state = self.shared.borrow_mut();
        state.request_ended();
        // A refused refresh of the existing PPS RDO cannot be repeated after
        // Reject; halt rather than silently continuing an unmaintainable PPS.
        if response == ControlMessageType::Reject
            && matches!(requested, request::PowerSource::Pps(_))
            && state
                .established
                .as_ref()
                .is_some_and(|c| Some(c.request.0) == request_raw(requested))
        {
            state.fail(Error::Declined(response));
            return false;
        }
        if let Some(selection) = state.selection {
            selection.restore_established();
            if response == ControlMessageType::Reject {
                selection.rejected.set(true);
            }
            if response == ControlMessageType::Wait {
                selection.cool_down();
            }
        }
        state.selected = state.established.clone();
        false // Optional Wait retries require a new explicit confirmation.
    }

    // Sections 7.2/7.3: nothing was sent. Keep the established contract; a PPS
    // refresh stays due and is retried after the caller's cooldown, as for Wait;
    // a user change needs a new confirmation (new capabilities may not hold it).
    async fn request_deferred(&mut self, _requested: &request::PowerSource) {
        let mut state = self.shared.borrow_mut();
        state.request_in_flight = false;
        state.selected = state.established.clone();
        if let Some(selection) = state.selection {
            selection.restore_established();
            selection.cool_down();
        }
        if let Some(trace) = state.trace {
            trace.borrow_mut().requests_deferred += 1;
        }
    }

    // PE_SNK_Get_PPS_Status exit (section 9.2.11.3.1). Only a Deferred query is
    // retried according to the caller's cooldown and unsent-attempt limit.
    async fn pps_status(&mut self, outcome: PpsStatusOutcome) {
        let mut state = self.shared.borrow_mut();
        // NotSent: the engine refused a query this policy armed; the gates disagree.
        if !state.pps_status_pending || outcome == PpsStatusOutcome::NotSent {
            state.fail(Error::TransmissionBlocked);
            return;
        }
        state.pps_status_query = false;
        state.pps_status_pending = false;
        if let Some(trace) = state.trace {
            let mut trace = trace.borrow_mut();
            match outcome {
                PpsStatusOutcome::Status(_) => trace.pps_status_received += 1,
                PpsStatusOutcome::NotSupported => trace.pps_status_not_supported += 1,
                PpsStatusOutcome::Timeout => trace.pps_status_timeouts += 1,
                PpsStatusOutcome::Deferred => trace.pps_status_deferred += 1,
                PpsStatusOutcome::NotSent => {}
            }
        }
        if let Some(selection) = state.selection {
            if let PpsStatusOutcome::Status(block) = outcome {
                selection.pps_status.set(Some(PpsStatus::decode(block)));
            }
            let deferrals = selection.pps_status_deferrals.get() + 1;
            if outcome == PpsStatusOutcome::Deferred
                && deferrals < L::SESSION_POLICY.pps_status_max_deferrals
            {
                selection.pps_status_deferrals.set(deferrals);
                selection.cool_down();
            } else {
                selection.pps_status_at_ms.set(None);
            }
        }
    }

    // Exit of PE_SNK_Get_Source_Cap_Ext, PE_Get_Revision or
    // PE_Get_Manufacturer_Info. Only a Deferred request is retried, after the
    // caller's cooldown and unsent-attempt limit; every other outcome is final.
    async fn query(&mut self, query: PartnerQuery, outcome: QueryOutcome) {
        let mut state = self.shared.borrow_mut();
        // NotSent, or an outcome for another request: the gates disagree.
        if state.query_pending != Some(query) || outcome == QueryOutcome::NotSent {
            state.fail(Error::TransmissionBlocked);
            return;
        }
        state.query = None;
        state.query_pending = None;
        if let Some(trace) = state.trace {
            let mut trace = trace.borrow_mut();
            match outcome {
                QueryOutcome::Answer(_) => trace.queries_answered += 1,
                QueryOutcome::NotSupported => trace.queries_not_supported += 1,
                QueryOutcome::Timeout => trace.queries_timeouts += 1,
                QueryOutcome::Deferred => trace.queries_deferred += 1,
                QueryOutcome::NotSent => {}
            }
        }
        let Some(selection) = state.selection else {
            return;
        };
        match outcome {
            QueryOutcome::Answer(block) => {
                let mut partner = selection.partner.get();
                // Both gates checked the framing; a block that does not decode is a fault.
                if !partner.answer(query, &block) {
                    state.fail(Error::Peer);
                    return;
                }
                selection.partner.set(partner);
                selection.advance_query();
            }
            QueryOutcome::Deferred
                if selection.query_deferrals.get() + 1 < L::SESSION_POLICY.query_max_deferrals =>
            {
                selection
                    .query_deferrals
                    .set(selection.query_deferrals.get() + 1);
                // An Alert's Get_Status is asked again (its request was taken when armed).
                if !selection.in_list() {
                    selection.status_requested.set(true);
                }
                selection.cool_down();
            }
            QueryOutcome::Deferred => selection.finish_query(query, Answer::Deferred),
            QueryOutcome::NotSupported => selection.finish_query(query, Answer::NotSupported),
            QueryOutcome::Timeout => selection.finish_query(query, Answer::Timeout),
            QueryOutcome::NotSent => {}
        }
    }

    async fn get_event(&mut self, capabilities: &source_capabilities::SourceCapabilities) -> Event {
        let Some(selection) = self.shared.borrow().selection else {
            return pending().await;
        };
        selection.advertise(capabilities);
        poll_fn(|cx| {
            *selection.waker.borrow_mut() = Some(cx.waker().clone());
            if !selection.rx_idle.get() || selection.waiting() {
                return Poll::Pending;
            }
            if selection.cable_pending.get() {
                let established = self
                    .shared
                    .borrow()
                    .trace
                    .is_some_and(|trace| trace.borrow().contract.is_some());
                if !established {
                    return Poll::Pending;
                }
                selection.cable_pending.set(false);
                selection.cable_active.set(true);
                selection.set_rx_idle(false);
                return Poll::Ready(Event::DiscoverCable);
            }
            let target = match selection.pending.take() {
                Some(target) => target,
                None if selection.refresh_due() => selection.target.get(),
                None if selection.sink_alert_due() => {
                    let ado = selection.sink_alert_events.get();
                    let mut state = self.shared.borrow_mut();
                    if state.established.is_none()
                        || state.query_pending.is_some()
                        || state.pps_status_pending
                    {
                        return Poll::Pending;
                    }
                    // A prior Status may already have cleared its SDB flags;
                    // the separately latched ADO remains due until Alert is sent.
                    if !L::status().is_some_and(sink_status_valid) {
                        state.fail(Error::TransmissionBlocked);
                        return Poll::Pending;
                    }
                    state.sink_alert = Some(ado);
                    selection.sink_alert_armed.set(Some(ado));
                    selection.set_rx_idle(false);
                    return Poll::Ready(Event::SendSinkAlert(ado));
                }
                None if selection.pps_status_due() => {
                    let mut state = self.shared.borrow_mut();
                    if state.established.is_none() || state.pps_status_pending {
                        return Poll::Pending;
                    }
                    state.pps_status_query = true;
                    selection.set_rx_idle(false);
                    return Poll::Ready(Event::GetPpsStatus);
                }
                None if selection.query_due() => {
                    let mut state = self.shared.borrow_mut();
                    let Some(query) = selection.next_query() else {
                        return Poll::Pending;
                    };
                    if state.established.is_none()
                        || state.query_pending.is_some()
                        || state.pps_status_pending
                    {
                        return Poll::Pending;
                    }
                    state.query = Some(query);
                    if !selection.in_list() {
                        // An Alert's Get_Status: Alerts from now on ask again, at
                        // most at the caller's selected status interval.
                        selection.status_requested.set(false);
                        selection.query_at_ms.set(Some(
                            selection
                                .now_ms
                                .get()
                                .saturating_add(L::SESSION_POLICY.status_min_interval_ms),
                        ));
                    }
                    selection.set_rx_idle(false);
                    return Poll::Ready(Event::Query(query));
                }
                None => return Poll::Pending,
            };
            match target.request::<L>(capabilities) {
                Ok(request) => {
                    selection.target.set(target);
                    selection.begin_request();
                    selection.set_rx_idle(false);
                    // Target::request only constructs fixed or PPS requests.
                    self.shared.borrow_mut().selected = Some(Contract {
                        capabilities: capabilities.clone(),
                        request: ReviewedRdo(
                            request_raw(&request).expect("target request must be fixed or PPS"),
                        ),
                    });
                    Poll::Ready(Event::RequestPower(request))
                }
                _ => {
                    self.shared.borrow_mut().fail(Error::UnsafeCapabilities);
                    Poll::Pending
                }
            }
        })
        .await
    }

    async fn cable_report(&mut self, report: usbpd_traits::CableReport) {
        if let Some(selection) = self.shared.borrow().selection {
            selection.cable_result.set(Some(report));
            selection.cable_active.set(false);
            selection.cable_mode.set(None);
        }
    }

    fn sink_capabilities(&self) -> sink_capabilities::SinkCapabilities {
        use sink_capabilities::{FixedSupply, SinkPowerDataObject, SprPps};
        let pd3 = self.shared.borrow().partner_revision == 2;
        let mut caps = sink_capabilities::SinkCapabilities::default();
        // Invalid limits leave the reply empty, which the transmit gate refuses.
        if let Some((objects, count)) = self::sink_capabilities::<L>(pd3) {
            for &raw in &objects[..count] {
                let pdo = if raw >> 30 == 0b11 {
                    SinkPowerDataObject::Pps(SprPps(raw))
                } else {
                    SinkPowerDataObject::FixedSupply(FixedSupply(raw))
                };
                caps.0.push(pdo).ok();
            }
        }
        caps
    }
}

struct Borrowed<'a, 'b, D, L: Limits> {
    driver: &'a mut D,
    shared: &'a RefCell<State<'b, L>>,
}
impl<D: Driver, L: Limits> Driver for Borrowed<'_, '_, D, L> {
    const HAS_AUTO_GOOD_CRC: bool = D::HAS_AUTO_GOOD_CRC;
    const HAS_AUTO_RETRY: bool = D::HAS_AUTO_RETRY;
    fn sender_response_elapsed_ms(&self) -> u64 {
        self.driver.sender_response_elapsed_ms()
    }
    async fn wait_for_vbus(&mut self) {
        self.driver.wait_for_vbus().await;
    }
    async fn transmit_bist_carrier(&mut self) -> Result<(), DriverTxError> {
        // Only once per received BIST Carrier Mode at vSafe5V, in idle Ready.
        let allowed = {
            let mut state = self.shared.borrow_mut();
            let allowed = core::mem::take(&mut state.bist_carrier)
                && state.mode == Mode::Maintained
                && state.outcome.is_none()
                && state.reset == Reset::Idle
                && !state.request_in_flight
                && !state.hard_reset_armed
                && !state.bist_test_data;
            if !allowed {
                state.fail(Error::TransmissionBlocked);
            }
            allowed
        };
        if !allowed {
            return pending().await;
        }
        match self.driver.transmit_bist_carrier().await {
            Ok(()) => {
                let mut state = self.shared.borrow_mut();
                state.begin_reset(Reset::Default);
                if let Some(trace) = state.trace {
                    trace.borrow_mut().bist_carriers_sent += 1;
                }
                Ok(())
            }
            Err(DriverTxError::HardReset) => {
                self.shared.borrow_mut().fail(Error::HardResetReceived);
                pending().await
            }
            // Not supported or not sent: the policy stays in Ready. A PHY that
            // failed stays stopped, so its next operation fails closed.
            Err(DriverTxError::Discarded) => Err(DriverTxError::Discarded),
            Err(error) => {
                self.shared.borrow_mut().fail(Error::Transmit(error));
                pending().await
            }
        }
    }
    async fn discover_cable(
        &mut self,
        next_tx_id: u8,
        last_rx_id: Option<u8>,
    ) -> Result<usbpd_traits::CableReport, DriverRxError> {
        if !self.shared.borrow().selection.is_some_and(|s| {
            s.cable_active() && u32::from(s.target_mv()) == VSAFE5V_MV && s.target_pps_object() == 0
        }) {
            self.shared.borrow_mut().fail(Error::TransmissionBlocked);
            return pending().await;
        }
        match self.driver.discover_cable(next_tx_id, last_rx_id).await {
            Ok(report) => {
                use usbpd_traits::CableStatus;
                let bounds = report.next_tx_id <= 7
                    && report.last_rx_id.is_none_or(|id| id <= 7)
                    && report.count <= 7;
                let valid = bounds
                    && match report.status {
                        CableStatus::Timeout | CableStatus::Deferred => report.count == 0,
                        status => matches!(
                            (
                                status,
                                crate::pd_cable::decode(
                                    &report.objects[..report.count as usize],
                                    report.revision
                                )
                            ),
                            (
                                CableStatus::Identity,
                                Ok(crate::pd_cable::Response::Identity(_))
                            ) | (CableStatus::Nak, Ok(crate::pd_cable::Response::Nak))
                                | (CableStatus::Busy, Ok(crate::pd_cable::Response::Busy))
                        ),
                    };
                if valid {
                    Ok(report)
                } else {
                    self.shared.borrow_mut().fail(Error::Peer);
                    pending().await
                }
            }
            Err(_) => {
                self.shared.borrow_mut().fail(Error::ReceiveDiscarded);
                pending().await
            }
        }
    }
    /// Hard Reset only in a mandatory case (`docs/PD-HARD-RESET.md` B1): one
    /// armed by a Protocol Error, or a response timer that runs only in these
    /// states (SenderResponseTimer for our Request or Soft_Reset, PSTransitionTimer).
    /// Never the optional SinkWaitCapTimer one (waiting for capabilities). The
    /// PHY separately checks the elapsed time. The session ends after it.
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        let allowed = {
            let state = self.shared.borrow();
            state.mode == Mode::Maintained
                && state.outcome.is_none()
                && (state.sent || state.reset == Reset::Capabilities && state.hard_reset_armed)
                && (state.hard_reset_armed
                    || (state.reset == Reset::Idle && state.request_in_flight)
                    || state.reset == Reset::AwaitAccept)
        };
        if !allowed {
            self.shared.borrow_mut().fail(Error::RecoveryBlocked);
            return pending().await;
        }
        let result = self.driver.transmit_hard_reset().await;
        {
            let mut state = self.shared.borrow_mut();
            match result {
                Ok(()) => {
                    if let Some(trace) = state.trace {
                        trace.borrow_mut().hard_resets_sent += 1;
                    }
                    state.fail(Error::HardResetSent);
                }
                Err(DriverTxError::HardReset) => state.fail(Error::HardResetReceived),
                Err(error) => state.fail(Error::Transmit(error)),
            }
        }
        pending().await
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        let is_request = bytes.len() == 6
            && bytes[1] & 0x80 == 0
            && Message::from_bytes(bytes).ok().is_some_and(|m| {
                if !matches!(
                    m.header.message_type(),
                    MessageType::Data(DataMessageType::Request)
                ) || !matches!(m.header.port_power_role(), usbpd::PowerRole::Sink)
                {
                    return false;
                }
                let state = self.shared.borrow();
                let Some(selected) = &state.selected else {
                    return false;
                };
                let parsed = Data::parse_message(
                    Message::new(m.header),
                    DataMessageType::Request,
                    &bytes[2..],
                    &selected.capabilities,
                )
                .ok();
                matches!(parsed.and_then(|m| m.payload),
                    Some(Payload::Data(Data::Request(request)))
                    if request_raw(&request) == Some(selected.request.0))
            });
        let is_unsupported_reply = self.shared.borrow().mode == Mode::Maintained
            && bytes.len() == 2
            && Message::from_bytes(bytes).ok().is_some_and(|m| {
                let state = self.shared.borrow();
                !state.request_in_flight
                    && state.reset == Reset::Idle
                    && state
                        .unsupported_reply
                        .is_some_and(|kind| m.header.message_type() == MessageType::Control(kind))
                    && ((m.header.0 >> 6) & 3) as u8 == state.partner_revision
                    && matches!(m.header.port_power_role(), usbpd::PowerRole::Sink)
            });
        // Sink_Capabilities only once per received Get_Sink_Cap, in idle Ready:
        // Data, the exact objects for the negotiated revision, Sink/UFP.
        let is_sink_caps = self.shared.borrow().mode == Mode::Maintained && bytes.len() >= 6 && {
            let state = self.shared.borrow();
            let header = u16::from_le_bytes([bytes[0], bytes[1]]);
            state.sink_caps_reply
                && !state.request_in_flight
                && state.reset == Reset::Idle
                && sink_capabilities::<L>(state.partner_revision == 2).is_some_and(
                    |(objects, count)| {
                        bytes.len() == 2 + 4 * count
                            && header & HEADER_WITHOUT_ID_MASK
                                == (count as u16) << 12 | u16::from(state.partner_revision) << 6 | 4
                            && bytes[2..]
                                .as_chunks::<4>()
                                .0
                                .iter()
                                .zip(&objects[..count])
                                .all(|(raw, object)| *raw == object.to_le_bytes())
                    },
                )
        };
        let is_revision_reply =
            self.shared.borrow().mode == Mode::Maintained && bytes.len() == 6 && {
                let state = self.shared.borrow();
                let header = u16::from_le_bytes([bytes[0], bytes[1]]);
                // One Data Object, Revision, Sink/UFP, at the negotiated revision.
                state.revision_reply
                    && !state.request_in_flight
                    && state.reset == Reset::Idle
                    && header & (HEADER_WITHOUT_ID_MASK & !HEADER_REVISION_MASK)
                        == REVISION_RESPONSE_HEADER
                    && ((header >> 6) & 3) as u8 == state.partner_revision
                    && u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]])
                        == REVISION_DATA_OBJECT
            };
        let is_skedb_reply =
            self.shared.borrow().mode == Mode::Maintained && bytes.len() == 30 && {
                let state = self.shared.borrow();
                let header = u16::from_le_bytes([bytes[0], bytes[1]]);
                // Extended, 7 Data Objects, Sink_Capabilities_Extended, Sink/UFP,
                // PD3; Chunked Chunk 0 of 24 bytes, exact SKEDB, 00h padding.
                state.skedb_reply
                    && !state.request_in_flight
                    && state.reset == Reset::Idle
                    && state.partner_revision == 2
                    && header & HEADER_WITHOUT_ID_MASK == SKEDB_RESPONSE_HEADER
                    && bytes[2..4] == SKEDB_CHUNK_HEADER
                    && sink_capabilities_extended::<L>().is_some_and(|block| bytes[4..28] == block)
                    && bytes[28..] == [0, 0]
            };
        let is_status_reply = bytes.len() == 14 && {
            let state = self.shared.borrow();
            let header = u16::from_le_bytes([bytes[0], bytes[1]]);
            state.mode == Mode::Maintained
                && !state.request_in_flight
                && state.reset == Reset::Idle
                && state.partner_revision == 2
                && state.established.is_some()
                && header & HEADER_WITHOUT_ID_MASK == STATUS_RESPONSE_HEADER
                && bytes[2..4] == [7, 0x80]
                && bytes[11..] == [0, 0, 0]
                && state
                    .status_reply
                    .is_some_and(|block| bytes[4..11] == block && sink_status_valid(block))
        };
        let is_sink_alert = bytes.len() == 6 && {
            let state = self.shared.borrow();
            let header = u16::from_le_bytes([bytes[0], bytes[1]]);
            let ado = u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
            state.mode == Mode::Maintained
                && !state.request_in_flight
                && state.reset == Reset::Idle
                && state.partner_revision == 2
                && state.established.is_some()
                && !state.caps_reply
                && !state.pps_status_pending
                && state.query_pending.is_none()
                && !state.sink_alert_pending
                && header & HEADER_WITHOUT_ID_MASK == ALERT_RESPONSE_HEADER
                && state.sink_alert == Some(ado)
                && state
                    .selection
                    .is_some_and(|s| s.armed_sink_alert() == Some(ado))
                && ado != 0
                && ado & !SINK_ALERT_EVENTS == 0
                && L::status().is_some_and(sink_status_valid)
        };
        // Get_PPS_Status only when policy armed it for an idle PD3 PPS contract.
        let is_pps_status_query =
            self.shared.borrow().mode == Mode::Maintained && bytes.len() == 2 && {
                let state = self.shared.borrow();
                let header = u16::from_le_bytes([bytes[0], bytes[1]]);
                state.pps_status_query
                    && !state.pps_status_pending
                    && !state.request_in_flight
                    && state.reset == Reset::Idle
                    && !state.caps_reply
                    && state.partner_revision == 2
                    && state.established.is_some()
                    && header & HEADER_WITHOUT_ID_MASK == GET_PPS_STATUS_HEADER
                    && state.selection.is_some_and(|s| {
                        s.target_pps_object() != 0 && s.established_target() == Some(s.target.get())
                    })
            };
        // An information request only as armed by policy, once, from an idle
        // PD3 contract: exactly the request's bytes (`pd_partner::query_request`).
        let query_request = (self.shared.borrow().mode == Mode::Maintained)
            .then(|| pd_partner::query_request(bytes))
            .flatten()
            .filter(|&query| {
                let state = self.shared.borrow();
                state.query == Some(query)
                    && state.query_pending.is_none()
                    && !state.pps_status_pending
                    && !state.request_in_flight
                    && state.reset == Reset::Idle
                    && !state.caps_reply
                    && state.partner_revision == 2
                    && state.established.is_some()
            });
        let is_reset_accept = self.shared.borrow().reset == Reset::Accept
            && bytes.len() == 2
            && u16::from_le_bytes([bytes[0], bytes[1]])
                == (3 | (u16::from(self.shared.borrow().partner_revision) << 6));
        // PE_SNK_Send_Soft_Reset: Control, MessageID 0 after the protocol reset,
        // Sink/UFP, negotiated revision, only once a Protocol Error armed it.
        // Not subject to SinkTxOK (section 7.1.1).
        let is_soft_reset = self.shared.borrow().reset == Reset::Send
            && bytes.len() == 2
            && u16::from_le_bytes([bytes[0], bytes[1]])
                == (13 | (u16::from(self.shared.borrow().partner_revision) << 6));
        let recovery = !is_soft_reset
            && bytes.len() >= 2
            && Message::from_bytes(bytes).ok().is_some_and(|m| {
                matches!(
                    m.header.message_type(),
                    MessageType::Control(ControlMessageType::SoftReset)
                )
            });
        let mode = self.shared.borrow().mode;
        // Only a Ready-originated Request or Get_PPS_Status (the first Message
        // of a Sink-initiated AMS) may come back Deferred (section 7.2).
        let ams_start = is_sink_alert
            || is_pps_status_query
            || query_request.is_some()
            || is_request && mode == Mode::Maintained && {
                let state = self.shared.borrow();
                state.established.is_some() && !state.caps_reply && state.reset == Reset::Idle
            };
        {
            let mut state = self.shared.borrow_mut();
            if state.bist_test_data {
                // Section 6.4.3.1: no Message but GoodCRC until Hard Reset.
                state.fail(Error::TransmissionBlocked);
            } else if recovery || state.hard_reset_armed {
                state.fail(Error::RecoveryBlocked);
            } else if state.sent && (bytes[0] >> 6) & 3 != state.partner_revision {
                state.fail(Error::TransmissionBlocked);
            } else if is_request {
                if mode == Mode::Once && state.sent {
                    state.fail(Error::TransmissionBlocked);
                } else {
                    if let Some(trace) = state.trace
                        && mode == Mode::Maintained
                        && state.sent
                    {
                        trace.borrow_mut().re_requests_sent += 1;
                    }
                    state.sent = true;
                    state.request_in_flight = true;
                    state.caps_reply = false;
                    state.partner_revision = (bytes[0] >> 6) & 3;
                }
            } else if is_sink_alert {
                state.sink_alert_pending = true;
            } else if is_status_reply {
                state.status_reply = None;
            } else if is_pps_status_query {
                state.pps_status_pending = true;
            } else if let Some(query) = query_request {
                state.query_pending = Some(query);
            } else if is_reset_accept {
                state.reset = Reset::Capabilities;
                state.recovering = true;
                if let Some(trace) = state.trace {
                    trace.borrow_mut().soft_resets_accepted += 1;
                }
            } else if is_soft_reset {
                state.reset = Reset::AwaitAccept;
                state.recovering = true;
                if let Some(trace) = state.trace {
                    trace.borrow_mut().soft_resets_sent += 1;
                }
            } else if is_unsupported_reply {
                if state.unsupported_reply == Some(ControlMessageType::NotSupported)
                    && let Some(trace) = state.trace
                {
                    trace.borrow_mut().not_supported_sent += 1;
                }
                state.unsupported_reply = None;
            } else if is_sink_caps {
                state.sink_caps_reply = false;
                if let Some(trace) = state.trace {
                    trace.borrow_mut().sink_caps_sent += 1;
                }
            } else if is_revision_reply {
                state.revision_reply = false;
                if let Some(trace) = state.trace {
                    trace.borrow_mut().revisions_sent += 1;
                }
            } else if is_skedb_reply {
                state.skedb_reply = false;
                if let Some(trace) = state.trace {
                    trace.borrow_mut().sink_caps_extended_sent += 1;
                }
            } else {
                state.fail(Error::TransmissionBlocked);
            }
        }
        if self.shared.borrow().outcome.is_some() {
            return pending().await;
        }
        match self.driver.transmit(bytes).await {
            Ok(()) => Ok(()),
            Err(DriverTxError::Deferred) if ams_start => {
                let state = self.shared.borrow();
                if is_sink_alert || is_pps_status_query || query_request.is_some() {
                    return Err(DriverTxError::Deferred);
                }
                if let Some(trace) = state.trace {
                    let mut trace = trace.borrow_mut();
                    trace.re_requests_sent = trace.re_requests_sent.saturating_sub(1);
                }
                Err(DriverTxError::Deferred)
            }
            // No GoodCRC after the PHY's retries: a Protocol Error, except for the
            // reset Messages themselves, whose failure needs Hard Reset (section 7.1.1).
            Err(DriverTxError::NotAcknowledged) if !is_soft_reset && !is_reset_accept => {
                if self.shared.borrow_mut().protocol_error() {
                    Err(DriverTxError::NotAcknowledged)
                } else {
                    pending().await
                }
            }
            // Sections 7.1.1, 9.2.5.2: our Soft_Reset or reset Accept failed.
            Err(DriverTxError::NotAcknowledged) => {
                if self.shared.borrow_mut().require_hard_reset() {
                    Err(DriverTxError::NotAcknowledged)
                } else {
                    pending().await
                }
            }
            Err(DriverTxError::HardReset) => {
                self.shared.borrow_mut().fail(Error::HardResetReceived);
                pending().await
            }
            Err(error) => {
                self.shared.borrow_mut().fail(Error::Transmit(error));
                pending().await
            }
        }
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        // A due Hard Reset is the only next step; the engine never waits first.
        if self.shared.borrow().hard_reset_armed {
            self.shared.borrow_mut().fail(Error::RecoveryBlocked);
            return pending().await;
        }
        let received = loop {
            let received = self.driver.receive(buffer).await;
            // BIST Test Data Mode: the PHY sent GoodCRC; the Message is not
            // acted on (section 6.4.3.1). Only Hard Reset ends the mode.
            if !(received.is_ok() && self.shared.borrow().bist_test_data) {
                break received;
            }
        };
        let length = match received {
            Ok(length) if length >= 2 && length <= buffer.len() => length,
            Err(DriverRxError::HardReset) => {
                self.shared.borrow_mut().fail(Error::HardResetReceived);
                return pending().await;
            }
            Err(DriverRxError::Discarded) => {
                self.shared.borrow_mut().fail(Error::ReceiveDiscarded);
                return pending().await;
            }
            _ => {
                self.shared.borrow_mut().fail(Error::Peer);
                return pending().await;
            }
        };
        // Avoid upstream's ParseError -> unreachable! and extended chunk-request
        // paths. Native PHY still owns SOP identity, framing and CRC validation.
        // Deprecated wire code0 means PD2, including for portable drivers.
        if buffer[0] & HEADER_REVISION_MASK as u8 == 0 {
            buffer[0] |= HEADER_PD2_REVISION as u8;
        }
        let header = u16::from_le_bytes([buffer[0], buffer[1]]);
        // Table 6.2: incoming SOP power role is not verified. The native PHY
        // separately validates SOP addressing and cable sender classification.
        let objects = ((header >> 12) & 7) as usize;
        let (ready, revision, pps_status_pending, maintained, query_pending) = {
            let mut state = self.shared.borrow_mut();
            state.unsupported_reply = None;
            state.revision_reply = false;
            state.sink_caps_reply = false;
            state.skedb_reply = false;
            state.status_reply = None;
            state.bist_carrier = false;
            (
                state.mode == Mode::Maintained
                    && state.established.is_some()
                    && !state.request_in_flight
                    && state.reset == Reset::Idle,
                state.partner_revision,
                state.pps_status_pending,
                state.mode == Mode::Maintained,
                state.query_pending,
            )
        };
        let extended = (header & HEADER_EXTENDED != 0
            && length >= HEADER_BYTES + EXTENDED_HEADER_BYTES)
            .then(|| u16::from_le_bytes([buffer[2], buffer[3]]));
        // Table 6.48 framing only for Extended Messages: they are answered or
        // classified, never assembled. Ordinary Messages must parse.
        let framed = if header & HEADER_EXTENDED != 0 {
            extended.is_some_and(|extended| {
                if extended & EXTENDED_CHUNKED != 0 {
                    objects > 0 && length == HEADER_BYTES + objects * OBJECT_BYTES
                } else {
                    extended & EXTENDED_INCOMPLETE_MASK == 0
                        && extended & EXTENDED_SIZE_MASK <= MAX_CHUNK_BYTES
                        && length
                            == HEADER_BYTES
                                + EXTENDED_HEADER_BYTES
                                + (extended & EXTENDED_SIZE_MASK) as usize
                }
            })
        } else {
            length == 2 + objects * 4 && Message::from_bytes(&buffer[..length]).is_ok()
        };
        let same_revision = ((header >> 6) & 3) as u8 == revision;
        if maintained && header & HEADER_KIND_COUNT_MASK == ControlMessageType::SoftReset as u16 {
            // PE_SNK_Soft_Reset from any state (section 9.2.5.2.2): Control,
            // MessageID 0 (section 7.7). Before the first Request, use the reset's
            // revision for Accept without locking it (section 6.1.3.1).
            let accepted = {
                let mut state = self.shared.borrow_mut();
                let reset_revision = ((header >> 6) & 3) as u8;
                let valid = length == HEADER_BYTES
                    && header & MESSAGE_ID_FIELD_MASK == 0
                    && matches!(reset_revision, 1 | 2)
                    && (!state.sent || same_revision)
                    && matches!(
                        state.reset,
                        Reset::Idle | Reset::AwaitAccept | Reset::Capabilities | Reset::Default
                    );
                if valid {
                    if !state.sent {
                        state.partner_revision = reset_revision;
                    }
                    state.begin_reset(Reset::Accept);
                    if let Some(trace) = state.trace {
                        trace.borrow_mut().packets_serviced += 1;
                    }
                } else {
                    state.fail(Error::RecoveryBlocked);
                }
                valid
            };
            return if accepted {
                Ok(length)
            } else {
                pending().await
            };
        }
        if self.shared.borrow().sink_alert_pending {
            // PE_SNK_Wait_for_Get_Status: exactly Get_Status at PD3, or a
            // well-formed unexpected Message answered with Soft_Reset.
            let get_status = length == HEADER_BYTES
                && header & HEADER_KIND_COUNT_MASK == ControlMessageType::GetStatus as u16
                && same_revision;
            let serviced = {
                let mut state = self.shared.borrow_mut();
                if get_status {
                    state.status_reply = L::status().filter(|&block| sink_status_valid(block));
                    if state.status_reply.is_none() {
                        state.fail(Error::Peer);
                    }
                    state.status_reply.is_some()
                } else if framed && same_revision {
                    state.protocol_error()
                } else {
                    state.fail(Error::Peer);
                    false
                }
            };
            return if serviced {
                Ok(length)
            } else {
                pending().await
            };
        }
        if pps_status_pending {
            // PE_SNK_Get_PPS_Status: only PD3 PPS_Status as one Chunk (Chunked,
            // Chunk 0, no Chunk request, Data Size 4, two Data Objects; Tables
            // 6.48, 6.58) or Not_Supported answer it. Any other well-formed
            // Message is a Protocol Error (section 9.2.5.2.1). A malformed one,
            // or a PPS_Status without that exact PPSSDB, fails closed.
            // Extended Header bit 9 is Reserved and ignored.
            let pps_status = header & HEADER_KIND_COUNT_MASK == PPS_STATUS_RESPONSE_HEADER
                && length == 10
                && u16::from_le_bytes([buffer[2], buffer[3]]) & EXTENDED_DEFINED_FIELDS_MASK
                    == PPS_STATUS_CHUNK_HEADER;
            let not_supported = header & HEADER_KIND_COUNT_MASK
                == ControlMessageType::NotSupported as u16
                && length == 2;
            let answered = (pps_status || not_supported) && (header >> 6) & 3 == 2;
            let serviced = {
                let mut state = self.shared.borrow_mut();
                let serviced = if answered {
                    true
                } else if framed
                    && same_revision
                    && header & HEADER_KIND_MASK
                        != HEADER_EXTENDED | ExtendedMessageType::PpsStatus as u16
                {
                    state.protocol_error()
                } else {
                    state.fail(Error::Peer);
                    false
                };
                if serviced && let Some(trace) = state.trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                serviced
            };
            return if serviced {
                Ok(length)
            } else {
                pending().await
            };
        }
        if let Some(query) = query_pending {
            // PE_SNK_Get_Source_Cap_Ext / PE_Get_Revision / PE_Get_Manufacturer_Info:
            // only the exact PD3 answer or Not_Supported (`pd_partner::query_answer`).
            // Any other well-formed Message at this revision is a Protocol Error
            // (section 9.2.5.2.1); a malformed answer fails closed.
            let serviced = {
                let mut state = self.shared.borrow_mut();
                let serviced = match pd_partner::query_answer(query, &buffer[..length]) {
                    pd_partner::Reply::Answer | pd_partner::Reply::NotSupported => true,
                    pd_partner::Reply::Other if framed && same_revision => state.protocol_error(),
                    _ => {
                        state.fail(Error::Peer);
                        false
                    }
                };
                if serviced && let Some(trace) = state.trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                serviced
            };
            return if serviced {
                Ok(length)
            } else {
                pending().await
            };
        }
        // A reset in progress admits exactly one next Message (sections 7.7,
        // 9.2.5.2); GoodCRC is the PHY's own and is skipped upstream.
        let good_crc =
            header & HEADER_KIND_COUNT_MASK == ControlMessageType::GoodCRC as u16 && length == 2;
        let phase = self.shared.borrow().reset;
        match phase {
            Reset::Idle => {}
            Reset::AwaitAccept => {
                let accepted = {
                    let mut state = self.shared.borrow_mut();
                    let accept = header & HEADER_KIND_COUNT_MASK
                        == ControlMessageType::Accept as u16
                        && length == HEADER_BYTES
                        && header & MESSAGE_ID_FIELD_MASK == 0
                        && same_revision;
                    // Section 7.7: another well-formed Message at this revision
                    // is a Protocol Error during the Soft Reset: Hard Reset. A
                    // malformed Accept fails closed.
                    let protocol_error = !accept
                        && !good_crc
                        && framed
                        && same_revision
                        && header & HEADER_KIND_COUNT_MASK != ControlMessageType::Accept as u16;
                    if accept {
                        state.reset = Reset::Capabilities;
                        if let Some(trace) = state.trace {
                            trace.borrow_mut().packets_serviced += 1;
                        }
                    } else if protocol_error {
                        state.require_hard_reset();
                    } else if !good_crc {
                        state.fail(Error::Peer);
                    }
                    accept || good_crc || protocol_error
                };
                return if accepted {
                    Ok(length)
                } else {
                    pending().await
                };
            }
            Reset::Capabilities => {
                let capabilities = header & HEADER_KIND_MASK
                    == DataMessageType::SourceCapabilities as u16
                    && header & HEADER_OBJECT_COUNT_MASK != 0
                    && framed
                    && (!self.shared.borrow().sent || same_revision);
                if capabilities {
                    self.shared.borrow_mut().reset = Reset::Idle;
                } else if !good_crc {
                    // Section 7.7: any other well-formed Message at this revision
                    // is a Protocol Error during the Soft Reset: Hard Reset.
                    let protocol_error = framed
                        && same_revision
                        && header & HEADER_KIND_MASK != DataMessageType::SourceCapabilities as u16;
                    let mut state = self.shared.borrow_mut();
                    if protocol_error && state.require_hard_reset() {
                        return Ok(length);
                    }
                    state.fail(Error::Peer);
                }
                if !good_crc && !capabilities {
                    return pending().await;
                }
            }
            Reset::Default => {
                let capabilities = header & HEADER_KIND_MASK
                    == DataMessageType::SourceCapabilities as u16
                    && header & HEADER_OBJECT_COUNT_MASK != 0
                    && framed
                    && same_revision;
                if capabilities {
                    self.shared.borrow_mut().reset = Reset::Idle;
                } else if !good_crc {
                    self.shared.borrow_mut().fail(Error::Peer);
                    return pending().await;
                }
            }
            Reset::Send | Reset::Accept => {
                self.shared.borrow_mut().fail(Error::RecoveryBlocked);
                return pending().await;
            }
        }
        if header & HEADER_EXTENDED != 0 {
            let serviced = {
                let mut state = self.shared.borrow_mut();
                let in_ams = maintained && state.request_in_flight && !state.transitioning;
                let serviced = match ready_response(revision, header, extended) {
                    ReadyResponse::Unsupported | ReadyResponse::UnsupportedAfterChunk
                        if ready && framed =>
                    {
                        state.unsupported_reply = Some(ControlMessageType::NotSupported);
                        true
                    }
                    ReadyResponse::Ignore if ready && framed => true,
                    // Section 9.2.5.2.1: a Protocol Error during the Request AMS.
                    _ if in_ams && framed && same_revision => state.protocol_error(),
                    // Table 7.1: while power transitions, Hard Reset.
                    _ if maintained && state.transitioning && framed && same_revision => {
                        state.require_hard_reset()
                    }
                    _ => false,
                };
                if !serviced {
                    state.fail(Error::Peer);
                } else if let Some(trace) = state.trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                serviced
            };
            return if serviced {
                Ok(length)
            } else {
                pending().await
            };
        }
        let parsed = framed
            .then(|| Message::from_bytes(&buffer[..length]).ok())
            .flatten();
        let Some(message) = parsed else {
            self.shared.borrow_mut().fail(Error::Peer);
            return pending().await;
        };
        let response = ready_response(revision, header, None);
        let unsupported = if revision == 1 {
            ControlMessageType::Reject
        } else {
            ControlMessageType::NotSupported
        };
        if maintained && !good_crc {
            use ControlMessageType::{Accept, PsRdy, Reject, Wait};
            let kind = message.header.message_type();
            let protocol_error = {
                let mut state = self.shared.borrow_mut();
                let outcome = if state.transitioning {
                    // Table 7.1, section 9.2.4.6: power is transitioning; only
                    // PS_RDY may follow, anything else at this revision is a
                    // Protocol Error answered with Hard Reset.
                    (kind != MessageType::Control(PsRdy)).then(|| {
                        if same_revision {
                            state.require_hard_reset()
                        } else {
                            state.fail(Error::Peer);
                            false
                        }
                    })
                } else if state.request_in_flight {
                    // PE_SNK_Select_Capability: anything but Accept/Reject/Wait
                    // is a Protocol Error (section 9.2.5.2.1, Figure 9.18 note 1).
                    if kind == MessageType::Control(Accept) {
                        state.transitioning = true;
                        None
                    } else if matches!(kind, MessageType::Control(Reject | Wait)) {
                        None
                    } else {
                        Some(state.protocol_error())
                    }
                } else if ready && response == ReadyResponse::SoftReset {
                    Some(state.protocol_error()) // Table 7.1, PE_SNK_Ready.
                } else {
                    None
                };
                if outcome == Some(true)
                    && let Some(trace) = state.trace
                {
                    trace.borrow_mut().packets_serviced += 1;
                }
                outcome
            };
            match protocol_error {
                Some(true) => return Ok(length),
                Some(false) => return pending().await,
                None => {}
            }
        }
        if matches!(
            message.header.message_type(),
            MessageType::Data(DataMessageType::SourceCapabilities)
        ) {
            let revision_changed = {
                let state = self.shared.borrow();
                state.sent && ((header >> 6) & 3) as u8 != state.partner_revision
            };
            if revision_changed {
                self.shared.borrow_mut().fail(Error::UnsafeCapabilities);
                return pending().await;
            }
            if let Some(selection) = self.shared.borrow().selection {
                selection.peer_revision.set(((header >> 6) & 3) as u8);
            }
            self.shared.borrow_mut().caps_reply = true;
        }
        let mode = self.shared.borrow().mode;
        match message.header.message_type() {
            MessageType::Data(DataMessageType::SourceCapabilities)
            | MessageType::Control(
                ControlMessageType::GoodCRC
                | ControlMessageType::Accept
                | ControlMessageType::PsRdy,
            ) => {
                if let Some(trace) = self.shared.borrow().trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                Ok(length)
            }
            MessageType::Control(
                other @ (ControlMessageType::Reject | ControlMessageType::Wait),
            ) => {
                if mode == Mode::Once {
                    self.shared.borrow_mut().fail(Error::Declined(other));
                    pending().await
                } else {
                    if let Some(trace) = self.shared.borrow().trace {
                        trace.borrow_mut().packets_serviced += 1;
                    }
                    Ok(length)
                }
            }
            MessageType::Control(_) if mode == Mode::Maintained => {
                if ready && response == ReadyResponse::Status && !same_revision {
                    self.shared.borrow_mut().fail(Error::Peer);
                    return pending().await;
                }
                let mut state = self.shared.borrow_mut();
                if ready {
                    match response {
                        ReadyResponse::Unsupported => state.unsupported_reply = Some(unsupported),
                        ReadyResponse::Reject => {
                            state.unsupported_reply = Some(ControlMessageType::Reject)
                        }
                        ReadyResponse::Revision => state.revision_reply = true,
                        ReadyResponse::SinkCap => state.sink_caps_reply = true,
                        ReadyResponse::SinkCapExtended => state.skedb_reply = true,
                        ReadyResponse::Status => {
                            state.status_reply =
                                L::status().filter(|&block| sink_status_valid(block));
                            if state.status_reply.is_none() {
                                state.unsupported_reply = Some(ControlMessageType::NotSupported);
                            }
                        }
                        _ => {}
                    }
                }
                if let Some(trace) = state.trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                Ok(length)
            }
            // PE_BIST_Test_Mode / PE_BIST_Carrier_Mode (section 9.2.26.4) at vSafe5V only.
            MessageType::Data(DataMessageType::Bist)
                if ready
                    && length >= 6
                    && self
                        .shared
                        .borrow()
                        .established
                        .as_ref()
                        .is_some_and(|contract| {
                            bist_request(
                                header,
                                u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]),
                                (contract.request.0 >> 28) as u8,
                            ) != BistRequest::Ignore
                        }) =>
            {
                let mut state = self.shared.borrow_mut();
                let object = state
                    .established
                    .as_ref()
                    .map_or(0, |contract| (contract.request.0 >> 28) as u8);
                let test_data = bist_request(
                    header,
                    u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]),
                    object,
                ) == BistRequest::TestData;
                if test_data {
                    state.bist_test_data = true;
                } else {
                    state.bist_carrier = true;
                }
                if let Some(trace) = state.trace {
                    let mut trace = trace.borrow_mut();
                    trace.bist_test_data |= test_data;
                    trace.packets_serviced += 1;
                }
                Ok(length)
            }
            // Alert informs policy; BIST/PD2 VDMs are Ignored; others are unsupported.
            MessageType::Data(_)
                if ready
                    && matches!(
                        response,
                        ReadyResponse::Alert | ReadyResponse::Ignore | ReadyResponse::Unsupported
                    ) =>
            {
                let mut state = self.shared.borrow_mut();
                if response == ReadyResponse::Unsupported {
                    state.unsupported_reply = Some(unsupported);
                }
                if let Some(trace) = state.trace {
                    trace.borrow_mut().packets_serviced += 1;
                }
                Ok(length)
            }
            _ => {
                self.shared.borrow_mut().fail(Error::Peer);
                pending().await
            }
        }
    }
}
