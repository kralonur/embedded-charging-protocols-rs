//! Bounded SOP partner discovery. Raw objects are retained; absent identity
//! VDOs never acquire default values. No power, data or VCONN role changes.
/// PD message payload limit: seven 32-bit objects (Table 6.2).
const MAX_OBJECTS: usize = 7;
/// PD SID, Structured VDM 2.1, initiator Discover Identity (Table 6.33).
pub(crate) const DISCOVER_IDENTITY: u32 = 0xff00_a801;

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
}

/// Mandatory identity VDOs are present; optional Product Type VDOs stay raw.
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
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum IdentityReply {
    Ack,
    Nak,
    Busy,
}

/// Table 6.33: only a matching SVDM 1.0/2.0/2.1 response, never an initiator request.
/// ACK includes the three mandatory identity VDOs; NAK/BUSY have no VDOs.
pub(crate) fn identity_reply(objects: &[u32]) -> Option<IdentityReply> {
    let raw = *objects.first()?;
    if raw >> 16 != 0xff00
        || raw & 0x8000 == 0
        || (raw >> 13) & 3 > 1
        || ((raw >> 13) & 3 == 1 && (raw >> 11) & 3 > 1)
        || (raw >> 8) & 7 != 0
        || raw & 0x1f != 1
    {
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
    }
}
