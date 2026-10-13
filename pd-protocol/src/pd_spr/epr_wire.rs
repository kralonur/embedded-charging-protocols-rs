//! Bounded EPR wire state, instantiated separately by policy and native PHY.
use super::*;
use usbpd::protocol_layer::message::data::source_capabilities::{
    SourceCapabilities, parse_raw_pdo,
};

/// EPR_Mode is one object, type 10, PD3 Sink/UFP (Tables 6.2, 6.5).
const MODE_HEADER: u16 = EPR_MODE_HEADER;
/// Message Header PD3 revision code, bits 7:6 (Table 6.2).
const PD3_REVISION: u16 = 0x0080;
/// Extended Control EPR_KeepAlive/ACK, Chunk 0, two-byte data (section 6.5.14).
const KEEP_HEADER: u16 = EPR_KEEP_ALIVE_HEADER;
const KEEP_BODY: [u8; 4] = EPR_KEEP_ALIVE_BODY;
/// ECDB type values for Get_Sink_Cap and KeepAlive_Ack (section 6.5.14).
const GET_SINK_CAP: u8 = 2;
const KEEP_ALIVE_ACK: u8 = 4;
/// Mode actions in the most significant byte of the EPR Mode DO (Table 6.49).
const ACTION_ENTER: u8 = EPR_ENTER;
const ACTION_ACK: u8 = EPR_ENTER_ACK;
const ACTION_SUCCEEDED: u8 = EPR_ENTER_SUCCEEDED;
const ACTION_FAILED: u8 = EPR_ENTER_FAILED;
const ACTION_EXIT: u8 = EPR_EXIT;
/// Source Extended Control kind, and Sink EPR_Sink_Capabilities header without
/// object count (Tables 6.2, 6.47). Chunk 1 request fields from Table 6.48.
const EXTENDED_CONTROL_KIND: u16 = 0x9010;
const SINK_CAPABILITIES_HEADER: u16 = 0x8092;
const SINK_CHUNK_REQUEST_KIND: u16 = 0x9012;
const SINK_CHUNK_REQUEST_EXTENDED: u16 = 0x8c00;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Spr,
    Ack,
    Succeeded,
    Capabilities,
    Active,
    KeepAlive,
    Exit,
}

/// Traffic authorization only, not electrical protection or a timeout source.
/// Each transport still owns its response deadlines and native cancellation.
pub struct EprWire {
    phase: Phase,
    chunk: u8,
    size: usize,
    caps: [u8; EPR_OBJECTS * OBJECT_BYTES],
    sink_reply: Option<u8>,
    sink_next: bool,
    last_rx_id: Option<u16>,
}
impl Default for EprWire {
    fn default() -> Self {
        Self::new()
    }
}
impl EprWire {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Spr,
            chunk: 0,
            size: 0,
            caps: [0; EPR_OBJECTS * OBJECT_BYTES],
            sink_reply: None,
            sink_next: false,
            last_rx_id: None,
        }
    }
    pub fn active(&self) -> bool {
        matches!(
            self.phase,
            Phase::Capabilities | Phase::Active | Phase::KeepAlive
        )
    }
    pub fn busy(&self) -> bool {
        !matches!(self.phase, Phase::Spr | Phase::Active)
            || self.sink_reply.is_some()
            || self.sink_next
    }
    pub fn awaiting_sink_chunk(&self) -> bool {
        self.sink_next
    }
    /// Protocol retransmissions are passed to PRL deduplication without advancing
    /// EPR state a second time. Observe all SOP messages, not only EPR messages.
    pub fn duplicate(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() < HEADER_BYTES {
            return false;
        }
        let header = u16::from_le_bytes([bytes[0], bytes[1]]);
        if header & HEADER_KIND_COUNT_MASK == ControlMessageType::SoftReset as u16 {
            self.last_rx_id = None;
            return false;
        }
        if header & HEADER_KIND_COUNT_MASK == ControlMessageType::GoodCRC as u16 {
            return false;
        }
        let id = header & MESSAGE_ID_FIELD_MASK;
        let duplicate = self.last_rx_id == Some(id);
        self.last_rx_id = Some(id);
        duplicate
    }
    pub fn entering(&self) -> bool {
        matches!(self.phase, Phase::Ack | Phase::Succeeded)
    }
    pub fn awaiting_keep_alive(&self) -> bool {
        self.phase == Phase::KeepAlive
    }
    pub fn awaiting_exit(&self) -> bool {
        self.phase == Phase::Exit
    }
    pub fn awaiting_capabilities(&self) -> bool {
        self.phase == Phase::Capabilities
    }
    pub fn ready(&self) -> bool {
        self.phase == Phase::Active && !self.busy()
    }
    /// Our Enter, KeepAlive or Exit was Discarded unsent (section 6.11): return
    /// to the phase it left, so the retry is authorized again.
    pub fn transmit_deferred(&mut self) {
        self.phase = match self.phase {
            Phase::Ack => Phase::Spr,
            Phase::KeepAlive | Phase::Exit => Phase::Active,
            phase => phase,
        };
    }
    pub fn objects(&self) -> [u32; EPR_OBJECTS] {
        core::array::from_fn(|i| {
            let offset = i * OBJECT_BYTES;
            u32::from_le_bytes([
                self.caps[offset],
                self.caps[offset + 1],
                self.caps[offset + 2],
                self.caps[offset + 3],
            ])
        })
    }
    /// Reset the chunk boundary after Soft Reset, preserving EPR mode.
    /// EPR objects must follow the seven SPR positions (section 6.5.15.1).
    pub fn valid_spr_prefix(&self) -> bool {
        let caps = SourceCapabilities::new_with_pdos(
            self.objects().into_iter().map(parse_raw_pdo).collect(),
        );
        !caps.has_epr_pdo_in_spr_positions()
    }
    pub fn soft_reset(&mut self) {
        self.sink_reply = None;
        self.sink_next = false;
        self.last_rx_id = None;
        if self.active() {
            self.phase = Phase::Capabilities;
            self.chunk = 0;
            self.size = 0;
        } else {
            self.phase = Phase::Spr;
        }
    }
    /// True only for an EPR message authorized here; ordinary traffic is checked
    /// by the caller's existing gates. `enter`/`exit` are policy-owned permission.
    pub fn transmit<L: Limits>(&mut self, bytes: &[u8], enter: bool, exit: bool) -> bool {
        if !L::EPR || bytes.len() < HEADER_BYTES {
            return false;
        }
        let header = u16::from_le_bytes([bytes[0], bytes[1]]) & HEADER_WITHOUT_ID_MASK;
        if let Some(chunk) = self.sink_reply
            && self.sink_chunk::<L>(bytes, chunk)
        {
            self.sink_reply = None;
            self.sink_next = chunk == 0;
            return true;
        }
        if header == MODE_HEADER && bytes.len() == 6 {
            let action = bytes[5];
            if self.phase == Phase::Spr
                && !self.busy()
                && enter
                && action == ACTION_ENTER
                && bytes[2..4] == [0, 0]
                && epr_pdps::<L>().is_some_and(|(_, pdp, _)| bytes[4] == pdp)
            {
                self.phase = Phase::Ack;
                return true;
            }
            if self.phase == Phase::Active
                && !self.busy()
                && exit
                && action == ACTION_EXIT
                && bytes[2..5] == [0, 0, 0]
            {
                self.phase = Phase::Exit;
                return true;
            }
        }
        if self.ready() && header == KEEP_HEADER && bytes.len() == 6 && bytes[2..] == KEEP_BODY {
            self.phase = Phase::KeepAlive;
            return true;
        }
        self.phase == Phase::Capabilities
            && self.chunk > 0
            && bytes.len() == 6
            && header == EPR_CHUNK_REQUEST_HEADER
            && bytes[2..] == epr_chunk_request(self.chunk)
    }
    /// EPR messages only. Invalid EPR traffic is rejected by the caller, which
    /// independently authorizes the specified Soft/Hard Reset response.
    fn sink_chunk<L: Limits>(&self, bytes: &[u8], chunk: u8) -> bool {
        let Some((objects, count)) = epr_sink_capabilities::<L>() else {
            return false;
        };
        let size = count * OBJECT_BYTES;
        let offset = usize::from(chunk) * MAX_CHUNK_BYTES as usize;
        if offset >= size {
            return false;
        }
        let len = (size - offset).min(MAX_CHUNK_BYTES as usize);
        let padded = (len + EXTENDED_HEADER_BYTES).div_ceil(OBJECT_BYTES) * OBJECT_BYTES;
        let header = u16::from_le_bytes([bytes[0], bytes[1]]) & HEADER_WITHOUT_ID_MASK;
        if bytes.len() != HEADER_BYTES + padded
            || header != SINK_CAPABILITIES_HEADER | ((padded / OBJECT_BYTES) as u16) << 12
        {
            return false;
        }
        let extended = 0x8000 | u16::from(chunk) << 11 | size as u16;
        let raw: [u8; EPR_OBJECTS * OBJECT_BYTES] =
            core::array::from_fn(|i| objects[i / OBJECT_BYTES].to_le_bytes()[i % OBJECT_BYTES]);
        bytes[2..4] == extended.to_le_bytes()
            && bytes[4..4 + len] == raw[offset..offset + len]
            && bytes[4 + len..].iter().all(|&byte| byte == 0)
    }
    pub fn receive(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() < 2 {
            return false;
        }
        let header = u16::from_le_bytes([bytes[0], bytes[1]]);
        // Ignore source role/data-role, but require PD3 and the exact type/count.
        let kind = header & HEADER_KIND_COUNT_MASK;
        if header & HEADER_REVISION_MASK != PD3_REVISION {
            return false;
        }
        if self.sink_next {
            if kind == SINK_CHUNK_REQUEST_KIND
                && bytes.len() == 6
                && u16::from_le_bytes([bytes[2], bytes[3]]) & EXTENDED_DEFINED_FIELDS_MASK
                    == SINK_CHUNK_REQUEST_EXTENDED
            {
                self.sink_reply = Some(1);
                self.sink_next = false;
                return true;
            }
            return false;
        }
        if self.sink_reply.is_some() {
            return false;
        }
        // Receiver ignores reserved Extended Header bit 9 and unused ECDB Data.
        let control = kind == EXTENDED_CONTROL_KIND
            && bytes.len() == 6
            && u16::from_le_bytes([bytes[2], bytes[3]]) & EXTENDED_DEFINED_FIELDS_MASK
                == EXTENDED_CHUNKED | 2;
        if control && bytes[4] == GET_SINK_CAP && matches!(self.phase, Phase::Spr | Phase::Active) {
            self.sink_reply = Some(0);
            return true;
        }
        if kind == EPR_MODE_KIND && bytes.len() == 6 {
            match (self.phase, bytes[5]) {
                (Phase::Ack, ACTION_ACK) => self.phase = Phase::Succeeded,
                (Phase::Succeeded, ACTION_SUCCEEDED) => {
                    self.phase = Phase::Capabilities;
                    self.chunk = 0;
                    self.size = 0;
                }
                (Phase::Ack | Phase::Succeeded, ACTION_FAILED) => self.phase = Phase::Spr,
                (Phase::Active, ACTION_EXIT) => self.phase = Phase::Exit,
                _ => return false,
            }
            return true;
        }
        if self.phase == Phase::KeepAlive && control && bytes[4] == KEEP_ALIVE_ACK {
            self.phase = Phase::Active;
            return true;
        }
        if header & HEADER_KIND_MASK == EPR_SOURCE_CAPABILITIES_KIND
            && matches!(self.phase, Phase::Capabilities | Phase::Active)
        {
            if self.phase == Phase::Active {
                self.phase = Phase::Capabilities;
                self.chunk = 0;
                self.size = 0;
            }
            let Some(result) = epr_chunk(bytes, self.chunk) else {
                return false;
            };
            let size = usize::from(u16::from_le_bytes([bytes[2], bytes[3]]) & EXTENDED_SIZE_MASK);
            if self.chunk == 0 {
                self.size = size;
                self.caps.fill(0);
            }
            if size != self.size {
                return false;
            }
            let offset = usize::from(self.chunk) * MAX_CHUNK_BYTES as usize;
            let len = (size - offset).min(MAX_CHUNK_BYTES as usize);
            self.caps[offset..offset + len].copy_from_slice(&bytes[4..4 + len]);
            match result {
                EprChunk::Next(next) => self.chunk = next,
                EprChunk::Last => {
                    self.chunk = 0;
                    self.phase = Phase::Active;
                }
            }
            return true;
        }
        if self.phase == Phase::Exit
            && header & HEADER_KIND_MASK == DataMessageType::SourceCapabilities as u16
        {
            self.phase = Phase::Spr;
            return true;
        }
        false
    }
}
