//! Passive decoding of another terminal's transactions from D− edge
//! timestamps alone; nothing here drives the line. Packets are framed as in
//! the active PHY: each starts after a ping and ends with its sender's own
//! ping. Every byte is re-timed from its own nine bit periods, so the
//! terminal's and the charger's clocks may differ within Table B.4.

use crate::UI_US;
use crate::control::{SHORT_OFFSET_MV, SHORT_STEP_MV, SHORT_VOLTAGE, VOLTAGE};
use crate::wire::{
    ACK, ACK_MASK, CRC, MULTI_WRITE, NACK, PING_MAX_US, PING_MIN_US, PING_UI, READ, WRITE,
    parity_bit,
};

/// Multiple-byte register read command (`SC8545_SCP_CMD_MBRRD` in
/// `sc8545_protocol.h`, see the crate docs). Its reply carries the data.
const MULTI_READ: u8 = 0x1c;
/// FCP output voltage register and its step in mV: `FCP_PROTOCOL_VOUT_CONFIG`
/// and `FCP_PROTOCOL_VOUT_STEP` in Huawei's `adapter_protocol_fcp.h` (crate
/// docs' kernel commit).
const FCP_VOLTAGE: u8 = 0x2c;
const FCP_STEP_MV: u16 = 100;
/// Register blocks: `adapter_protocol_fcp.h` defines FCP registers below
/// 0x70 and identification/version registers from 0x70 to 0x8b, which both
/// drivers read; SCP ratings and controls start at 0x90 (Tables B.19–B.27).
const FCP_END: u8 = 0x70;
const SCP_START: u8 = 0x90;
/// Latency of each edge timestamp in µs, from 0 to this, so a run's length
/// is off by at most this much. Implementation choice: one active-PHY
/// sample period, covering the capturing side's interrupt latency.
const JITTER_US: u32 = 8;
/// Pings in µs: Table B.4 `Tp-MR`, widened by the jitter.
const PING_MIN: u32 = PING_MIN_US - JITTER_US;
const PING_MAX: u32 = PING_MAX_US + JITTER_US;
/// A sync pulse lasts at least Table B.4 `Tb/4-MR` (36 µs), less jitter.
/// Runs shorter than half a nominal UI are sync pulses, longer ones data:
/// Table B.4 keeps every sync below 44 µs and every bit above 144 µs.
const SYNC_MIN_US: u32 = 36 - JITTER_US;
const SYNC_MAX_US: u32 = UI_US / 2;
/// Bits per byte on the wire: eight data bits, then odd parity.
const BYTE_BITS: usize = 9;
/// A byte's nine bit periods in µs, at the UI a received ping allows.
const BYTE_MIN_US: u32 = BYTE_BITS as u32 * PING_MIN_US / PING_UI - JITTER_US;
const BYTE_MAX_US: u32 = BYTE_BITS as u32 * PING_MAX_US / PING_UI + JITTER_US;
/// Most data bytes in one transaction: Huawei's `fcp_protocol_check_trans_num`
/// accepts 1 or an even count up to 16 (crate docs' kernel commit).
const MAX_DATA: usize = 16;
/// Longest packet: a multiple write's command, address, length, data and CRC.
const MAX_PACKET: usize = 3 + MAX_DATA + 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    Read,
    Write,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    Fcp,
    Scp,
}

/// One complete command and the charger's checked reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transaction {
    pub access: Access,
    pub register: u8,
    /// The charger acknowledged rather than rejected the command.
    pub acked: bool,
    data: [u8; MAX_DATA],
    len: usize,
}

impl Transaction {
    /// The bytes written, or the bytes an acknowledged read returned.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// The protocol the register belongs to; `None` for the identification
    /// registers FCP and SCP share.
    pub fn protocol(&self) -> Option<Protocol> {
        match self.register {
            ..FCP_END => Some(Protocol::Fcp),
            SCP_START.. => Some(Protocol::Scp),
            _ => None,
        }
    }

    /// The output voltage in mV an acknowledged write requested. A multiple
    /// write starting at a voltage register sets that register first.
    pub fn requested_mv(&self) -> Option<u16> {
        if self.access != Access::Write || !self.acked {
            return None;
        }
        match (self.register, self.data()) {
            (FCP_VOLTAGE, &[step, ..]) => Some(u16::from(step) * FCP_STEP_MV),
            (VOLTAGE, &[high, low, ..]) => Some(u16::from_be_bytes([high, low])),
            (SHORT_VOLTAGE, &[step, ..]) => Some(SHORT_OFFSET_MV + u16::from(step) * SHORT_STEP_MV),
            _ => None,
        }
    }

    /// `None` unless `command` and `reply` form a known transaction.
    fn new(command: &[u8], reply: &[u8]) -> Option<Self> {
        let &[code, register, ref rest @ ..] = command else {
            return None;
        };
        let acked = match reply.first()? & ACK_MASK {
            ACK => true,
            NACK => false,
            _ => return None,
        };
        let (access, data) = match (code, rest) {
            (READ | MULTI_READ, _) if acked => (Access::Read, &reply[1..]),
            (READ | MULTI_READ, _) => (Access::Read, &[][..]),
            (WRITE, &[_]) => (Access::Write, rest),
            (MULTI_WRITE, &[len, ref data @ ..]) if usize::from(len) == data.len() => {
                (Access::Write, data)
            }
            _ => return None,
        };
        let mut transaction = Self {
            access,
            register,
            acked,
            data: [0; MAX_DATA],
            len: data.len(),
        };
        transaction
            .data
            .get_mut(..data.len())?
            .copy_from_slice(data);
        Some(transaction)
    }
}

#[derive(Clone, Copy, Debug, Default)]
enum State {
    /// Out of step: wait for a ping.
    #[default]
    Idle,
    /// After a ping: another ping, or the first sync pulse of a packet.
    Gap,
    /// Counting a sync's quarter-UI pulses.
    Sync(u8),
    /// A byte's data runs so far, the first at level `high`.
    Bits {
        high: bool,
        runs: [u32; BYTE_BITS],
        len: usize,
    },
}

#[derive(Clone, Copy, Debug, Default)]
struct Packet {
    bytes: [u8; MAX_PACKET],
    len: usize,
}

impl Packet {
    fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Rebuilds transactions from the edges another terminal and its charger
/// make on D−. Damaged or partial traffic is dropped until the next ping.
#[derive(Clone, Copy, Debug, Default)]
pub struct Decoder {
    /// Time and level of the latest edge.
    last: Option<(u32, bool)>,
    state: State,
    packet: Packet,
    /// A checked command waiting for its reply.
    command: Option<Packet>,
}

impl Decoder {
    /// Feed one edge: the level D− changed to and when, in µs on a clock
    /// that may wrap. Returns a transaction once its reply is complete.
    pub fn push(&mut self, at_us: u32, high: bool) -> Option<Transaction> {
        let (since, level) = self.last.replace((at_us, high))?;
        if level == high {
            // An edge was missed, so every run length after it is wrong.
            self.lose_sync();
            return None;
        }
        self.run(level, at_us.wrapping_sub(since))
    }

    /// One finished run of the bus at level `high` lasting `us`.
    fn run(&mut self, high: bool, us: u32) -> Option<Transaction> {
        if high && (PING_MIN..=PING_MAX).contains(&us) {
            let closed = matches!(self.state, State::Sync(2 | 3)) && self.packet.len > 0;
            let cut = matches!(self.state, State::Sync(_) | State::Bits { .. });
            if cut && !closed {
                // A packet cut short may have been the lost reply.
                self.command = None;
            }
            self.state = State::Gap;
            return if closed { self.finish() } else { None };
        }
        let sync = (SYNC_MIN_US..SYNC_MAX_US).contains(&us);
        self.state = match self.state {
            State::Gap if !high => State::Gap,
            State::Gap if sync => {
                self.packet.len = 0;
                State::Sync(1)
            }
            State::Sync(pulses @ (1 | 2)) if sync => State::Sync(pulses + 1),
            State::Sync(2 | 3) if !sync && us <= BYTE_MAX_US => {
                let mut runs = [0; BYTE_BITS];
                runs[0] = us;
                State::Bits { high, runs, len: 1 }
            }
            State::Bits { high, runs, len } if sync => {
                if self.byte(high, &runs[..len]) {
                    State::Sync(1)
                } else {
                    State::Idle
                }
            }
            State::Bits {
                high: first,
                mut runs,
                len,
            } if len < BYTE_BITS && us <= BYTE_MAX_US => {
                runs[len] = us;
                State::Bits {
                    high: first,
                    runs,
                    len: len + 1,
                }
            }
            _ => State::Idle,
        };
        if matches!(self.state, State::Idle) {
            self.lose_sync();
        }
        None
    }

    /// Wait for the next ping, forgetting any command: whatever was lost
    /// may have been its reply, or a later command.
    fn lose_sync(&mut self) {
        self.state = State::Idle;
        self.command = None;
    }

    /// Decode one byte from its data runs, the first at level `high`, and
    /// append it. `false` on bad timing, parity or an overlong packet.
    fn byte(&mut self, mut high: bool, runs: &[u32]) -> bool {
        let total: u32 = runs.iter().sum();
        if !(BYTE_MIN_US..=BYTE_MAX_US).contains(&total) {
            return false;
        }
        let (mut bits, mut count, mut elapsed) = (0u16, 0, 0);
        for run in runs {
            elapsed += run;
            // The bit boundary nearest this edge, timed by this byte's own UI.
            let end = (elapsed * BYTE_BITS as u32 + total / 2) / total;
            if end <= count {
                return false;
            }
            for _ in count..end {
                bits = (bits << 1) | u16::from(high);
            }
            count = end;
            high = !high;
        }
        let [byte, _] = (bits >> 1).to_le_bytes();
        if (bits & 1 != 0) != parity_bit(byte) || self.packet.len == MAX_PACKET {
            return false;
        }
        self.packet.bytes[self.packet.len] = byte;
        self.packet.len += 1;
        true
    }

    /// A packet closed by its sender's ping: keep a command, or pair a reply
    /// with the command before it.
    fn finish(&mut self) -> Option<Transaction> {
        let packet = self.packet;
        let checked = match packet.as_slice().split_last() {
            Some((crc, payload)) if !payload.is_empty() && CRC.checksum(payload) == *crc => {
                Some(payload)
            }
            _ => None,
        };
        let command = self.command.take();
        let payload = checked?;
        if matches!(payload[0], READ | MULTI_READ | WRITE | MULTI_WRITE) {
            self.command = Some(packet);
            return None;
        }
        let command = command?;
        let (_, command) = command.as_slice().split_last()?;
        Transaction::new(command, payload)
    }
}
