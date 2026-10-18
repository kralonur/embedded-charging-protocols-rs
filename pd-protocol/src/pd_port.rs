//! Bounded SOP partner discovery. Raw objects are retained; absent identity
//! VDOs never acquire default values. No power, data or VCONN role changes.
use crate::pd_constants::MAX_OBJECTS;
use usbpd::protocol_layer::message::data::{
    sink_capabilities::{FixedSupply, SinkPowerDataObject, parse_raw_pdo},
    vendor_defined::UFPTypeVDO,
};

/// PD SID, Structured VDM 2.1, initiator Discover Identity (Table 6.33).
pub(crate) const DISCOVER_IDENTITY: u32 = 0xff00_a801;
/// SVDM 2.1, the highest version this port supports, as VDM Header bits 14..11.
const SVDM_VERSION_MAX: u32 = DISCOVER_IDENTITY & 0x7800;

/// The SVDM version as VDM Header bits 14..11 (Table 6.33): SVDM 1.0 is 0, its
/// minor field not defined; an Invalid major 10b/11b is read as 2.x.
fn svdm_version(vdm: u32) -> u32 {
    if (vdm >> 13) & 3 == 0 {
        0
    } else {
        0x2000 | vdm & 0x1800
    }
}

/// The NAK reply to a received Structured VDM request (Table 7.2), with its SVID
/// (section 8.6), at the received version or SVDM 2.1 when that is higher
/// (section 8.5.1); reserved bit 5 is sent as zero. `None` for responses and
/// Attention (command 6), which get no reply (Table 8.1).
pub fn svdm_nak(vdm: u32) -> Option<u32> {
    // Bit 15 Structured; bits 7..6 Command Type: 00b REQ, 10b NAK.
    if vdm & 0x8000 == 0 || (vdm >> 6) & 3 != 0 || vdm & 0x1f == 6 {
        return None;
    }
    Some(vdm & !(0x7800 | 0x20) | svdm_version(vdm).min(SVDM_VERSION_MAX) | 0x80)
}

/// Exactly the objects received in one message, including a VDM header if present.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Objects {
    raw: [u32; MAX_OBJECTS],
    count: u8,
}
impl Objects {
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.is_empty() || bytes.len() > MAX_OBJECTS * 4 || !bytes.len().is_multiple_of(4) {
            return None;
        }
        let mut result = Self {
            raw: [0; MAX_OBJECTS],
            count: (bytes.len() / 4) as u8,
        };
        for (slot, raw) in result.raw.iter_mut().zip(bytes.as_chunks::<4>().0) {
            *slot = u32::from_le_bytes(*raw);
        }
        Some(result)
    }
    pub fn as_slice(&self) -> &[u32] {
        &self.raw[..usize::from(self.count)]
    }
    /// Only the first, fixed vSafe5V Sink PDO carries the flags (Table 6.9).
    pub fn sink_fixed(&self) -> Option<FixedSupply> {
        use crate::pd_constants::{FIXED_VOLTAGE_UNIT_MV, VSAFE5V_MV};
        let SinkPowerDataObject::FixedSupply(pdo) = parse_raw_pdo(self.raw[0]) else {
            return None;
        };
        (u32::from(pdo.raw_voltage()) * FIXED_VOLTAGE_UNIT_MV == VSAFE5V_MV).then_some(pdo)
    }
}

/// Mandatory identity VDOs are present. Unknown Product Type VDO layouts stay raw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Identity(Objects);
impl Identity {
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let objects = Objects::decode(bytes)?;
        (identity_reply(objects.as_slice()) == Some(IdentityReply::Ack)).then_some(Self(objects))
    }
    pub fn objects(&self) -> &[u32] {
        self.0.as_slice()
    }
    pub fn id_header(&self) -> u32 {
        self.0.raw[1]
    }
    pub fn xid(&self) -> u32 {
        self.0.raw[2]
    }
    pub fn product(&self) -> u32 {
        self.0.raw[3]
    }
    /// PD3 SOP UFP v1.2 or v1.3 (Table 6.40); not a cable or legacy AMA layout.
    /// Table 6.40 still lists 010b as the deprecated v1.2, which differs only in
    /// bits 23..22 (Connector Type, Ignored by the receiver).
    pub fn ufp_vdo(&self) -> Option<UFPTypeVDO> {
        if self.objects()[0] & (3 << 13) == 0 || !matches!((self.id_header() >> 27) & 7, 1 | 2) {
            return None;
        }
        let vdo = UFPTypeVDO(*self.objects().get(4)?);
        matches!(vdo.version(), 2 | 3).then_some(vdo)
    }
    /// PD3 SOP DFP v1.2 only (Table 6.41). DRD replies place it after UFP + pad
    /// (Figure 6.5), even when the UFP VDO version is not understood here.
    pub fn dfp_vdo(&self) -> Option<Dfp> {
        if self.objects()[0] & (3 << 13) == 0 || !matches!((self.id_header() >> 23) & 7, 1..=3) {
            return None;
        }
        let index = match (self.id_header() >> 27) & 7 {
            0 => 4,
            1 | 2 if self.objects().get(5) == Some(&0) => 6,
            _ => return None,
        };
        let raw = *self.objects().get(index)?;
        (raw >> 29 == 2).then_some(Dfp {
            usb_capabilities: ((raw >> 24) & 7) as u8,
            port_number: (raw & 0x1f) as u8,
        })
    }
}

/// Received DFP v1.2 claims, not a measurement of USB operation (Table 6.41).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dfp {
    /// Bits 0, 1, 2: USB 2.0, USB 3.2 and USB4 host capability respectively.
    pub usb_capabilities: u8,
    pub port_number: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum IdentityReply {
    Ack,
    Nak,
    Busy,
}

/// A Discover Identity response, not a request, another SVID or command, or an
/// Unstructured VDM (Table 6.33).
pub(crate) fn is_identity_response(vdm: u32) -> bool {
    vdm >> 16 == 0xff00 && vdm & 0x8000 != 0 && vdm & 0x1f == 1 && (vdm >> 6) & 3 != 0
}

/// Table 6.33: a Discover Identity response at SVDM 1.0/2.0/2.1; Object Position
/// and reserved bit 5 are ignored. ACK includes the three mandatory identity VDOs;
/// NAK/BUSY have no VDOs.
pub(crate) fn identity_reply(objects: &[u32]) -> Option<IdentityReply> {
    let raw = *objects.first()?;
    if !is_identity_response(raw) || svdm_version(raw) > SVDM_VERSION_MAX {
        return None;
    }
    match ((raw >> 6) & 3, objects.len()) {
        (1, 4..=MAX_OBJECTS) => Some(IdentityReply::Ack),
        (2, 1) => Some(IdentityReply::Nak),
        (3, 1) => Some(IdentityReply::Busy),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn identity_never_invents_missing_product_type_vdos() {
        let bytes: std::vec::Vec<u8> = [0xff00_a041u32, 0xd040_04e8, 0, 0x6860_0000]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let identity = Identity::decode(&bytes).unwrap();
        assert_eq!(identity.objects().len(), 4);
        assert_eq!(identity.objects().get(4), None);
        assert_eq!(
            (identity.id_header() as u16, identity.product() >> 16),
            (0x04e8, 0x6860)
        );
        assert!(Identity::decode(&bytes[..12]).is_none());
        assert!(Objects::decode(&[0; 29]).is_none());
        assert!(identity.ufp_vdo().is_none());
        assert!(identity.dfp_vdo().is_none());
    }

    fn bytes(objects: &[u32]) -> std::vec::Vec<u8> {
        objects.iter().flat_map(|raw| raw.to_le_bytes()).collect()
    }

    #[test]
    fn sink_flags_only_come_from_the_first_fixed_five_volt_object() {
        let objects = Objects::decode(&bytes(&[0x3601_912c, 0x0002_d0c8])).unwrap();
        let flags = objects.sink_fixed().unwrap();
        assert!(flags.dual_role_power());
        assert!(flags.higher_capability());
        assert!(flags.usb_communications_capable());
        assert!(flags.dual_role_data());
        assert!(!flags.unconstrained_power());
        assert_eq!(flags.raw_fast_role_swap(), 0);
        for first in [0x0002_d0c8, 0x8b41_90c8, 0xc0dc_212e] {
            assert!(
                Objects::decode(&bytes(&[first, 0x3601_912c]))
                    .unwrap()
                    .sink_fixed()
                    .is_none()
            );
        }
    }

    #[test]
    fn product_type_vdos_require_sop_roles_known_versions_and_received_objects() {
        // PD3 DRD: UFP peripheral + DFP host; UFP v1.3, zero pad, DFP v1.2.
        let mut raw = [0xff00_a041, 0x5100_0000, 0, 0, 0x6d00_0003, 0, 0x4700_0005];
        let identity = Identity::decode(&bytes(&raw)).unwrap();
        let ufp = identity.ufp_vdo().unwrap();
        assert_eq!(ufp.device_capability(), 0b1101);
        assert_eq!(ufp.usb_highest_speed(), 3);
        assert_eq!(
            identity.dfp_vdo(),
            Some(Dfp {
                usb_capabilities: 7,
                port_number: 5
            })
        );
        // The UFP VDO must never be mistaken for a DFP VDO in a truncated DRD ACK.
        assert!(
            Identity::decode(&bytes(&raw[..5]))
                .unwrap()
                .dfp_vdo()
                .is_none()
        );
        raw[5] = 1;
        assert!(Identity::decode(&bytes(&raw)).unwrap().dfp_vdo().is_none());
        raw[5] = 0;
        raw[4] = 0x4d00_0003; // UFP v1.2: the same fields.
        assert_eq!(
            Identity::decode(&bytes(&raw))
                .unwrap()
                .ufp_vdo()
                .unwrap()
                .usb_highest_speed(),
            3
        );
        raw[4] = 0xe100_0000; // Unknown UFP layout is raw; DFP position is unchanged.
        let identity = Identity::decode(&bytes(&raw)).unwrap();
        assert!(identity.ufp_vdo().is_none());
        assert!(identity.dfp_vdo().is_some());
        raw[6] = 0xe700_0005;
        assert!(Identity::decode(&bytes(&raw)).unwrap().dfp_vdo().is_none());
        raw[0] = 0xff00_8041; // SVDM 1.0 does not define these SOP Product Type layouts.
        let identity = Identity::decode(&bytes(&raw)).unwrap();
        assert!(identity.ufp_vdo().is_none());
        assert!(identity.dfp_vdo().is_none());
        // DFP-only SOP reply has no UFP/pad. Cable-type IDs never select SOP layouts.
        raw = [0xff00_a041, 0x4100_0000, 0, 0, 0x4300_0002, 0, 0];
        assert_eq!(
            Identity::decode(&bytes(&raw[..5])).unwrap().dfp_vdo(),
            Some(Dfp {
                usb_capabilities: 3,
                port_number: 2
            })
        );
        raw[1] = 0x1800_0000;
        assert!(
            Identity::decode(&bytes(&raw[..5]))
                .unwrap()
                .ufp_vdo()
                .is_none()
        );
    }
}
