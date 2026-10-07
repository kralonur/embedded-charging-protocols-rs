//! Receive framing and validation without transmission.
//! Input is one SOP* address plus the wire bytes (header, data, CRC); a PHY
//! adapter (e.g. a FIFO token decoder in its own crate) supplies the address.
//! Ordinary decoding rejects extended frames. Explicit cable-only decoding also
//! accepts complete extended payloads up to 26 bytes. Explicit partner framing
//! accepts single SOP Chunks (any Chunk of a multi-Chunk Message, or a Chunk
//! request) and unchunked payloads up to 26 bytes, so policy can answer them;
//! it never assembles Chunks. CRC covers header + data.

use crate::pd_constants::*;
use usbpd::protocol_layer::message::header::{ControlMessageType, DataMessageType};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sop {
    Partner,
    Cable,
    CableDoublePrime,
    Debug,
    DebugDoublePrime,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Truncated,
    /// A PHY adapter read an address code that is not a defined SOP* address.
    InvalidToken,
    UnsupportedExtended,
    InvalidExtended,
    ReservedRevision,
    Crc,
    NotSourceCapabilities,
}

/// Which SOP* addresses may carry extended frames into this decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ExtendedRx {
    cable: bool,
    partner: bool,
}
const NO_EXTENDED: ExtendedRx = ExtendedRx {
    cable: false,
    partner: false,
};
const CABLE_EXTENDED: ExtendedRx = ExtendedRx {
    cable: true,
    partner: false,
};

/// Borrowed, CRC-checked single frame. Trailing bytes belong to later frames.
#[derive(Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    sop: Sop,
    header: u16,
    data: &'a [u8],
}

impl Frame<'_> {
    pub fn sop(&self) -> Sop {
        self.sop
    }
    pub fn header(&self) -> u16 {
        self.header
    }
    pub fn data(&self) -> &[u8] {
        self.data
    }

    /// Only complete chunk 0 (or unchunked) data up to 26 bytes is supported.
    /// Larger/chunk-request messages fail closed, never silently truncate.
    pub fn extended_data(&self) -> Result<&[u8], Error> {
        if !matches!(self.sop, Sop::Partner | Sop::Cable | Sop::CableDoublePrime)
            || self.header & HEADER_EXTENDED == 0
            || self.data.len() < EXTENDED_HEADER_BYTES
        {
            return Err(Error::InvalidExtended);
        }
        let header = u16::from_le_bytes([self.data[0], self.data[1]]);
        let size = (header & EXTENDED_SIZE_MASK) as usize;
        let expected = if header & EXTENDED_CHUNKED != 0 {
            (size + EXTENDED_HEADER_BYTES).div_ceil(OBJECT_BYTES) * OBJECT_BYTES
        } else {
            size + EXTENDED_HEADER_BYTES
        };
        if header & EXTENDED_INCOMPLETE_MASK != 0
            || size > MAX_CHUNK_BYTES as usize
            || expected != self.data.len()
        {
            return Err(Error::InvalidExtended);
        }
        Ok(&self.data[EXTENDED_HEADER_BYTES..EXTENDED_HEADER_BYTES + size])
    }

    /// Upstream USB-PD message/PDO parsing after our framing/length/CRC boundary.
    /// Pure parsing only: no PHY, GoodCRC, policy engine or voltage request.
    /// SOP/source identity must still be checked separately by the caller.
    pub fn message(
        &self,
    ) -> Result<usbpd::protocol_layer::message::Message, usbpd::protocol_layer::message::ParseError>
    {
        let mut wire = [0; MAX_MESSAGE_BYTES];
        let header = if self.header & HEADER_KIND_COUNT_MASK == ControlMessageType::GoodCRC as u16 {
            (self.header & !HEADER_REVISION_MASK) | HEADER_PD2_REVISION // GoodCRC revision is ignored.
        } else if self.header & HEADER_REVISION_MASK == 0 {
            self.header | HEADER_PD2_REVISION
        } else {
            self.header
        };
        wire[..HEADER_BYTES].copy_from_slice(&header.to_le_bytes());
        wire[HEADER_BYTES..HEADER_BYTES + self.data.len()].copy_from_slice(self.data);
        usbpd::protocol_layer::message::Message::from_bytes(&wire[..HEADER_BYTES + self.data.len()])
    }

    /// Raw advertised PDOs, little-endian. No selection, requests or unit guesses.
    /// Ordinary partner Source_Capabilities only, never cable/control messages.
    pub fn source_capabilities(&self) -> Result<impl ExactSizeIterator<Item = u32> + '_, Error> {
        if self.sop != Sop::Partner
            || self.header & HEADER_TYPE_MASK != DataMessageType::SourceCapabilities as u16
            || self.data.is_empty()
        {
            return Err(Error::NotSourceCapabilities);
        }
        Ok(self
            .data
            .as_chunks::<OBJECT_BYTES>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b)))
    }
}

/// Immutable display-only advertisement, not a selected/requested power level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdvertisedPdo {
    pub raw: u32,
    pub fixed: Option<(u16, u16)>, // upstream fixed PDO voltage mV / current mA
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Advertisements {
    pub objects: [AdvertisedPdo; MAX_OBJECTS],
    pub count: u8,
}
#[derive(Debug, PartialEq, Eq)]
pub enum AdvertisementError {
    NotSourceCapabilities,
    Parse(usbpd::protocol_layer::message::ParseError),
}
impl Frame<'_> {
    pub fn advertisements(&self) -> Result<Advertisements, AdvertisementError> {
        use usbpd::protocol_layer::message::{
            Payload,
            data::{Data, source_capabilities::PowerDataObject},
        };
        let _ = self
            .source_capabilities()
            .map_err(|_| AdvertisementError::NotSourceCapabilities)?;
        let message = self.message().map_err(AdvertisementError::Parse)?;
        let Some(Payload::Data(Data::SourceCapabilities(capabilities))) = message.payload else {
            return Err(AdvertisementError::NotSourceCapabilities);
        };
        let mut result = Advertisements {
            objects: [AdvertisedPdo {
                raw: 0,
                fixed: None,
            }; MAX_OBJECTS],
            count: capabilities.pdos().len() as u8,
        };
        for (slot, pdo) in result.objects.iter_mut().zip(capabilities.pdos()) {
            slot.raw = pdo.to_raw();
            if let PowerDataObject::FixedSupply(fixed) = pdo {
                slot.fixed = Some((
                    fixed.raw_voltage() * FIXED_VOLTAGE_UNIT_MV as u16,
                    fixed.raw_max_current() * FIXED_CURRENT_UNIT_MA as u16,
                ));
            }
        }
        Ok(result)
    }
}

/// Controller-neutral packet: address supplied separately, bytes contain the
/// PD header, data and little-endian CRC, without any controller prefix.
/// Extended reception is opt-in and still bounded to the currently supported
/// cable messages. Prefer this API when the controller exposes CRC bytes.
/// Caller must stop/resynchronize on errors, not scan arbitrary payload for a frame.
/// CRC-valid is not proof of source identity, conformance or electrical safety.
pub fn decode_wire(
    sop: Sop,
    bytes: &[u8],
    cable_extended: bool,
) -> Result<(Frame<'_>, usize), Error> {
    decode_wire_with(sop, bytes, cable_extended, false)
}

/// Explicitly selected extended reception: `cable` keeps the cable bounds
/// above; `partner` admits individual SOP Chunks for policy classification
/// (Table 6.48), never Chunk assembly or a payload beyond one Chunk.
pub fn decode_wire_with(
    sop: Sop,
    bytes: &[u8],
    cable: bool,
    partner: bool,
) -> Result<(Frame<'_>, usize), Error> {
    let (header, consumed) = wire_info(sop, bytes, ExtendedRx { cable, partner })?;
    if bytes.len() < consumed {
        return Err(Error::Truncated);
    }
    let end = consumed - CRC_BYTES;
    let expected = u32::from_le_bytes([bytes[end], bytes[end + 1], bytes[end + 2], bytes[end + 3]]);
    if crc32(&bytes[..end]) != expected {
        return Err(Error::Crc);
    }
    let frame = Frame {
        sop,
        header,
        data: &bytes[HEADER_BYTES..end],
    };
    frame.validate_extended()?;
    Ok((frame, consumed))
}

/// Receive boundary for transports that strip independently validated CRC bytes.
/// Caller MUST have independently observed successful CRC validation
/// for this exact packet/address before calling. This function checks framing,
/// revision and supported extended layout, not electrical packet integrity.
/// A PHY that delivers the CRC bytes should use [`decode_wire`] instead, which
/// checks the CRC in software.
pub fn decode_verified_message(
    sop: Sop,
    bytes: &[u8],
    cable_extended: bool,
) -> Result<(Frame<'_>, usize), Error> {
    let extended = if cable_extended {
        CABLE_EXTENDED
    } else {
        NO_EXTENDED
    };
    let (header, with_crc) = wire_info(sop, bytes, extended)?;
    let consumed = with_crc - CRC_BYTES;
    if bytes.len() < consumed {
        return Err(Error::Truncated);
    }
    let frame = Frame {
        sop,
        header,
        data: &bytes[HEADER_BYTES..consumed],
    };
    frame.validate_extended()?;
    Ok((frame, consumed))
}

/// Validated header length only (header, data and CRC), NOT a CRC-checked or
/// complete packet. Lets a receiver reject unsupported headers before reading a
/// body: `prefix` needs the 2 header bytes, plus the 2 extended-header bytes
/// when the header's Extended bit is set (otherwise `Truncated`).
pub fn wire_frame_len(sop: Sop, prefix: &[u8], cable: bool, partner: bool) -> Result<usize, Error> {
    wire_info(sop, prefix, ExtendedRx { cable, partner }).map(|(_, length)| length)
}

fn wire_info(sop: Sop, bytes: &[u8], extended_rx: ExtendedRx) -> Result<(u16, usize), Error> {
    if bytes.len() < HEADER_BYTES {
        return Err(Error::Truncated);
    }
    let header = u16::from_le_bytes([bytes[0], bytes[1]]);
    let allowed = match sop {
        Sop::Cable | Sop::CableDoublePrime => extended_rx.cable,
        Sop::Partner => extended_rx.partner,
        _ => false,
    };
    if header & HEADER_EXTENDED != 0 && !allowed {
        return Err(Error::UnsupportedExtended);
    }
    // PD R3.2 v1.2 §6.1.3: received GoodCRC revision carries no meaning.
    if header & HEADER_REVISION_MASK == HEADER_REVISION_MASK
        && header & HEADER_KIND_COUNT_MASK != ControlMessageType::GoodCRC as u16
    {
        return Err(Error::ReservedRevision);
    }
    let count = ((header & HEADER_OBJECT_COUNT_MASK) >> HEADER_OBJECT_COUNT_SHIFT) as usize;
    let body_len = if header & HEADER_EXTENDED != 0 {
        if bytes.len() < HEADER_BYTES + EXTENDED_HEADER_BYTES {
            return Err(Error::Truncated);
        }
        let extended = u16::from_le_bytes([bytes[2], bytes[3]]);
        if sop == Sop::Partner && extended & EXTENDED_CHUNKED != 0 {
            // Table 6.48: a Chunk (or Chunk request) occupies its padded Data
            // Objects. Data Size is the whole Data Block, at most MaxExtendedMsgLen.
            if count == 0 || extended & EXTENDED_SIZE_MASK > MAX_EXTENDED_BYTES {
                return Err(Error::InvalidExtended);
            }
            count * OBJECT_BYTES
        } else if extended & EXTENDED_INCOMPLETE_MASK != 0
            || extended & EXTENDED_SIZE_MASK > MAX_CHUNK_BYTES
        {
            return Err(Error::InvalidExtended);
        } else if extended & EXTENDED_CHUNKED != 0 {
            if count == 0 {
                return Err(Error::InvalidExtended);
            }
            count * OBJECT_BYTES
        } else {
            // v1.2 Table 6.3: NDO is reserved for unchunked messages.
            EXTENDED_HEADER_BYTES + (extended & EXTENDED_SIZE_MASK) as usize
        }
    } else {
        count * OBJECT_BYTES
    };
    let consumed = HEADER_BYTES + body_len + CRC_BYTES; // Transport prefixes are excluded.
    Ok((header, consumed))
}

impl Frame<'_> {
    /// Cable frames must be complete single-packet messages. Partner frames were
    /// sized by `wire_info`; their Chunk semantics belong to the policy layer.
    fn validate_extended(&self) -> Result<(), Error> {
        if self.header & HEADER_EXTENDED != 0 && self.sop != Sop::Partner {
            self.extended_data()?;
        }
        Ok(())
    }
}

/// v1.2 Table 6.2: received revision code 0 is interpreted as PD2.
/// GoodCRC ignores revision entirely and must not use this helper.
pub fn revision(header: u16) -> Result<u8, Error> {
    match (header & HEADER_REVISION_MASK) >> HEADER_REVISION_SHIFT {
        0 | 1 => Ok(1),
        2 => Ok(2),
        _ => Err(Error::ReservedRevision),
    }
}

/// USB PD R3.2 V1.2 §5.3.1.1.4: polynomial 04C11DB7, init FFFFFFFF, LSB-first,
/// complemented. Over the header and data bytes; sent little-endian.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ if crc & 1 != 0 { CRC32_POLYNOMIAL } else { 0 };
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_external_known_vector() {
        // Check value from the CRC-32 catalogue, not this implementation.
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn wire_frame_len_needs_the_extended_header_only_for_extended_frames() {
        assert_eq!(
            wire_frame_len(Sop::Partner, &0x11a1u16.to_le_bytes(), false, false),
            Ok(10)
        );
        assert_eq!(
            wire_frame_len(Sop::Partner, &[0xa1], false, false),
            Err(Error::Truncated)
        );
        let extended = 0x918cu16.to_le_bytes();
        assert_eq!(
            wire_frame_len(Sop::Partner, &extended, false, true),
            Err(Error::Truncated)
        );
        assert_eq!(
            wire_frame_len(
                Sop::Partner,
                &[extended[0], extended[1], 4, 0x80],
                false,
                true
            ),
            Ok(10)
        );
        assert_eq!(
            wire_frame_len(
                Sop::Partner,
                &[extended[0], extended[1], 4, 0x80],
                false,
                false
            ),
            Err(Error::UnsupportedExtended)
        );
    }
}
