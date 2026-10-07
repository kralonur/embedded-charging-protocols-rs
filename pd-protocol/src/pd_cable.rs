//! Offline PD2/PD3 SOP' Discover Identity response decoding.
//! Authority: USB PD R3.2 V1.2 (2026-08-19), Tables 6.33, 6.42–6.44.
//! Legacy PD2 is decoded separately; deprecated PD3/SVDM1 layouts stay raw.
//! Independent implementation: no upstream code is incorporated.
//! Callers must first validate CRC, message type, revision and SOP' addressing.
//! Advertised properties are not electrical qualification or permission to raise power.

pub use usbpd_traits::{CableReport, CableStatus};

// USB-IF's assigned SVID for standard discovery (USB PD Table 6.33).
pub(crate) const USB_IF_SVID: u16 = 0xff00;
// A standard PD Message carries at most seven Data Objects (Table 6.2).
pub(crate) const MAX_DATA_OBJECTS: usize = 7;
// Interpreted Specification Revision codes from the PD Message Header (Table 6.2).
pub(crate) const PD2_REVISION: u8 = 1;
pub(crate) const PD3_REVISION: u8 = 2;
// Structured VDM header bit and command field (Table 6.33).
pub(crate) const STRUCTURED_VDM: u32 = 1 << 15;
pub(crate) const VDM_COMMAND_MASK: u32 = 0x1f;
// Standard Discover Identity command (Table 6.34).
pub(crate) const DISCOVER_IDENTITY: u8 = 1;
// Encoded SVDM Version 2.0 and 2.1 fields in bits 14..11 (Table 6.33).
const SVDM_VERSION_2_0: u32 = 0x2000;
const SVDM_VERSION_2_1: u32 = 0x2800;
// Structured VDM Command Type response encodings (Table 6.33).
const VDM_ACK: u32 = 1;
const VDM_NAK: u32 = 2;
const VDM_BUSY: u32 = 3;
// ID Header Product Type encodings for passive and active cables (Table 6.35).
pub(crate) const PASSIVE_CABLE: u32 = 3;
pub(crate) const ACTIVE_CABLE: u32 = 4;
// Identity ACK: VDM header, ID Header, Cert Stat, Product and Cable VDO (Tables 6.35–6.44).
const MIN_IDENTITY_OBJECTS: usize = 5;
// Active Cable VDO version 1.3 additionally requires Active Cable VDO 2 (Table 6.44).
const MIN_ACTIVE_V13_OBJECTS: usize = 6;
// SOP'' Controller Present bit, defined by the PD2 and active v1.3 layouts (Table 6.43).
const SECOND_CONTROLLER_PRESENT: u32 = 1 << 3;
// VBUS Through Cable bit in active Cable VDOs (Table 6.43).
const VBUS_THROUGH_CABLE: u32 = 1 << 4;
// Cable Current Handling Capability encodings and their currents in mA (Tables 6.42–6.43).
const CURRENT_3A_CODE: u32 = 1;
const CURRENT_5A_CODE: u32 = 2;
const CURRENT_3A_MA: u16 = 3_000;
const CURRENT_5A_MA: u16 = 5_000;
// Modern Cable VDO voltage encodings: code 3 is 50 V; all others decode as 20 V
// under R3.2 V1.2's receiver rules, including deprecated 30/40 V (Tables 6.42–6.43).
const VOLTAGE_50V_CODE: u32 = 3;
const VOLTAGE_50V_MV: u32 = 50_000;
const VOLTAGE_20V_MV: u32 = 20_000;
// EPR Mode Capable bit in supported modern Cable VDOs (Tables 6.42–6.43).
const EPR_CAPABLE: u32 = 1 << 17;
// Supported Cable VDO Version field encodings (Tables 6.42–6.43).
const PASSIVE_V10_VERSION: u8 = 0;
const ACTIVE_V13_VERSION: u8 = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CableKind {
    Passive,
    Active,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Length,
    Header,
    UnsupportedVersion,
    NotCable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Layout {
    Pd2,
    PassiveV10,
    ActiveV13,
    Unknown,
}

/// Normalize SVDM major values as required by v1.2 Table 6.33.
/// Reserved major 2/3 is received as 2.x, but the raw header is retained.
pub fn svdm_version(header: u32) -> Result<u32, Error> {
    if (header >> 13) & 3 == 0 {
        return Ok(0);
    }
    match (header >> 11) & 3 {
        0 => Ok(SVDM_VERSION_2_0),
        1 => Ok(SVDM_VERSION_2_1),
        _ => Err(Error::UnsupportedVersion),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity<'a> {
    /// All objects, including the structured VDM header and unknown fields.
    pub raw: &'a [u32],
    pub kind: CableKind,
    pub layout: Layout,
    /// v1.2 passive reserved-current encodings require a 3A receiver default.
    pub current_assumed: bool,
    pub vid: u16,
    pub pid: u16,
    pub bcd_device: u16,
    pub certification: u32,
    pub current_ma: Option<u16>,
    pub maximum_voltage_mv: Option<u32>,
    /// Actual EPR bit in supported modern layouts; never inferred from voltage.
    pub epr_capable: Option<bool>,
    /// Version-dependent raw speed code; deliberately not a marketing label.
    pub speed_code: u8,
    pub cable_vdo_version: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Response<'a> {
    Identity(Identity<'a>),
    Nak,
    Busy,
}

/// Version-defined SOP'' controller presence, never inferred from reserved bits.
pub fn second_controller(report: &CableReport) -> Option<bool> {
    if report.status != CableStatus::Identity {
        return None;
    }
    let Response::Identity(identity) = decode(
        report.objects.get(..report.count as usize)?,
        report.revision,
    )
    .ok()?
    else {
        return None;
    };
    match identity.layout {
        Layout::Pd2 | Layout::ActiveV13 => Some(report.objects[4] & SECOND_CONTROLLER_PRESENT != 0),
        Layout::PassiveV10 => Some(false),
        Layout::Unknown => None,
    }
}

/// Decode PD3 cable responses, including negotiated SVDM 1.0 fallback.
pub fn decode_pd3(objects: &[u32]) -> Result<Response<'_>, Error> {
    decode(objects, PD3_REVISION)
}

/// Decode PD2 cable responses with SVDM 1.0 headers.
pub fn decode_pd2(objects: &[u32]) -> Result<Response<'_>, Error> {
    decode(objects, PD2_REVISION)
}

/// Revision is the interpreted PD header code (1=PD2, 2=PD3).
/// Callers normalize received code 0 to 1; this is not a marketing version.
pub fn decode(objects: &[u32], revision: u8) -> Result<Response<'_>, Error> {
    if objects.is_empty() || objects.len() > MAX_DATA_OBJECTS {
        return Err(Error::Length);
    }
    let header = objects[0];
    // USB-IF SVID, structured, Discover Identity; ignored fields are not gates.
    if header >> 16 != u32::from(USB_IF_SVID)
        || header & STRUCTURED_VDM == 0
        || header & VDM_COMMAND_MASK != u32::from(DISCOVER_IDENTITY)
    {
        return Err(Error::Header);
    }
    let svdm = svdm_version(header)?;
    let version_ok = match revision {
        PD2_REVISION => svdm == 0,
        PD3_REVISION => true,
        _ => false,
    };
    if !version_ok {
        return Err(Error::UnsupportedVersion);
    }
    match (header >> 6) & 3 {
        VDM_NAK | VDM_BUSY if objects.len() != 1 => return Err(Error::Length),
        VDM_NAK => return Ok(Response::Nak),
        VDM_BUSY => return Ok(Response::Busy),
        VDM_ACK => {}
        _ => return Err(Error::Header),
    }
    if objects.len() < MIN_IDENTITY_OBJECTS {
        return Err(Error::Length);
    }
    let kind = match (objects[1] >> 27) & 7 {
        PASSIVE_CABLE => CableKind::Passive,
        ACTIVE_CABLE => CableKind::Active,
        _ => return Err(Error::NotCable),
    };
    let cable = objects[4];
    let version = ((cable >> 21) & 7) as u8;
    let layout = match (revision, svdm, kind, version) {
        (PD2_REVISION, 0, _, _) => Layout::Pd2,
        (
            PD3_REVISION,
            SVDM_VERSION_2_0 | SVDM_VERSION_2_1,
            CableKind::Passive,
            PASSIVE_V10_VERSION,
        ) => Layout::PassiveV10,
        (
            PD3_REVISION,
            SVDM_VERSION_2_0 | SVDM_VERSION_2_1,
            CableKind::Active,
            ACTIVE_V13_VERSION,
        ) => Layout::ActiveV13,
        _ => Layout::Unknown,
    };
    let modern = matches!(layout, Layout::PassiveV10 | Layout::ActiveV13);
    if layout == Layout::ActiveV13 && objects.len() < MIN_ACTIVE_V13_OBJECTS {
        return Err(Error::Length);
    }
    Ok(Response::Identity(Identity {
        raw: objects,
        kind,
        layout,
        current_assumed: layout == Layout::PassiveV10 && matches!((cable >> 5) & 3, 0 | 3),
        vid: objects[1] as u16,
        pid: (objects[3] >> 16) as u16,
        bcd_device: objects[3] as u16,
        certification: objects[2],
        current_ma: if layout != Layout::Unknown
            && (kind == CableKind::Passive || cable & VBUS_THROUGH_CABLE != 0)
        {
            match (cable >> 5) & 3 {
                CURRENT_3A_CODE => Some(CURRENT_3A_MA),
                CURRENT_5A_CODE => Some(CURRENT_5A_MA),
                _ if layout == Layout::PassiveV10 => Some(CURRENT_3A_MA),
                _ => None,
            }
        } else {
            None
        },
        maximum_voltage_mv: if modern {
            // v1.2: deprecated 30V/40V encodings are received as 20V.
            Some(if (cable >> 9) & 3 == VOLTAGE_50V_CODE {
                VOLTAGE_50V_MV
            } else {
                VOLTAGE_20V_MV
            })
        } else {
            None
        },
        epr_capable: modern.then_some(cable & EPR_CAPABLE != 0),
        speed_code: (cable & 7) as u8,
        cable_vdo_version: version,
    }))
}
