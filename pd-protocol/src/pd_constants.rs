//! Shared USB PD wire limits and units, not caller policy.

// Message Header and Extended Header sizes in bytes (Tables 6.2 and 6.48).
pub(crate) const HEADER_BYTES: usize = 2;
pub(crate) const EXTENDED_HEADER_BYTES: usize = 2;
// A Data Object and the CRC each occupy four bytes (sections 6.1 and 5.3.1.1.4).
pub(crate) const OBJECT_BYTES: usize = 4;
pub(crate) const CRC_BYTES: usize = 4;
// Standard Message Header NDO field capacity (Table 6.2).
pub(crate) const MAX_OBJECTS: usize = 7;
// MaxExtendedMsgChunkLen and MaxExtendedMsgLen, in bytes (section 6.6).
pub(crate) const MAX_CHUNK_BYTES: u16 = 26;
pub(crate) const MAX_EXTENDED_BYTES: u16 = 260;
// Standard packet ceiling: Message Header followed by seven Data Objects.
pub(crate) const MAX_MESSAGE_BYTES: usize = HEADER_BYTES + MAX_OBJECTS * OBJECT_BYTES;
// Message Header Type field, bit 15 (Extended), and bits 14..12 (NDO), Table 6.2.
pub(crate) const HEADER_TYPE_MASK: u16 = 0x001f;
pub(crate) const HEADER_EXTENDED: u16 = 0x8000;
pub(crate) const HEADER_OBJECT_COUNT_MASK: u16 = 0x7000;
pub(crate) const HEADER_OBJECT_COUNT_SHIFT: u32 = 12;
// Type, Extended and NDO combined, ignoring roles, revision and MessageID (Table 6.2).
pub(crate) const HEADER_KIND_COUNT_MASK: u16 = 0xf01f;
// Type plus Extended, ignoring NDO (Table 6.2).

// Specification Revision bits 7..6 and PD2 encoding (Table 6.2).
pub(crate) const HEADER_REVISION_MASK: u16 = 0x00c0;
pub(crate) const HEADER_REVISION_SHIFT: u32 = 6;
pub(crate) const HEADER_PD2_REVISION: u16 = 0x0040;
// Everything except MessageID; used to match exact Sink/UFP responses (Table 6.2).

// Extended Header Chunked, Request Chunk and Data Size fields (Table 6.48).
pub(crate) const EXTENDED_CHUNKED: u16 = 0x8000;

pub(crate) const EXTENDED_SIZE_MASK: u16 = 0x01ff;
// Chunk Number and Request Chunk must be clear for a complete single Chunk (Table 6.48).
pub(crate) const EXTENDED_INCOMPLETE_MASK: u16 = 0x7c00;
// Power units used by fixed PDO/RDOs and PPS PDO/RDOs (Tables 6.9–6.14, 6.19, 6.21).
pub(crate) const FIXED_VOLTAGE_UNIT_MV: u32 = 50;
pub(crate) const FIXED_CURRENT_UNIT_MA: u32 = 10;
pub(crate) const PPS_PDO_VOLTAGE_UNIT_MV: u32 = 100;
// AVS APDO voltage unit, in mV (Table 6.15).
pub(crate) const AVS_PDO_VOLTAGE_UNIT_MV: u32 = 100;
pub(crate) const PPS_RDO_VOLTAGE_UNIT_MV: u32 = 20;
pub(crate) const PPS_CURRENT_UNIT_MA: u32 = 50;
// Millivolts times milliamperes per watt, from SI unit conversion.

// vSafe5V and SPR fixed/PPS voltage and power ceilings (sections 6.4.1 and 3.4.2).

pub(crate) const SPR_MAX_W: u32 = 100;
// Lowest SPR PPS output voltage encodable for this profile (Table 6.13).

// Reflected IEEE CRC-32 polynomial used by PD packets (section 5.3.1.1.4).
pub(crate) const CRC32_POLYNOMIAL: u32 = 0xedb8_8320;
