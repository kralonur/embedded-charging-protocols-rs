//! Appendix B physical and logical layer for one single-byte register read:
//! pings, ¼-UI syncs, MSB-first bytes with odd parity, and CRC-8.

use crate::{Error, Register, Transport, UI_US};
use crc::{Algorithm, Crc, NoTable};

/// Appendix B.3.7.2 polynomial; zero-initialized, MSB-first CRC profile.
/// `check` is the independently calculated remainder for ASCII "123456789".
const ALGORITHM: Algorithm<u8> = Algorithm {
    width: 8,
    poly: 0x39,
    init: 0,
    refin: false,
    refout: false,
    xorout: 0,
    check: 0x8f,
    residue: 0,
};
pub(super) const CRC: Crc<u8, NoTable> = Crc::<u8, NoTable>::new(&ALGORITHM);
/// Single-byte register read command (`SBRRD`, Figure B.18).
pub(super) const READ: u8 = 0x0c;
/// Single/multiple-byte register write commands (Figures B.20–B.23).
pub(super) const WRITE: u8 = 0x0b;
pub(super) const MULTI_WRITE: u8 = 0x1b;
/// Acknowledgement codes in the low nibble, `0bxxxx_1000` and `0bxxxx_0011`.
/// Bit 7 is an adapter status flag, so 0x88 and 0x83 are valid too
/// (B.9.1, Table B.15). The whole byte is part of the CRC.
pub(super) const ACK: u8 = 0x08;
pub(super) const NACK: u8 = 0x03;
pub(super) const ACK_MASK: u8 = 0x0f;
/// Reply lengths in bytes: ACK, value and CRC (Figure B.19, and
/// `SC8545_SCP_ACK_AND_CRC_LEN` plus one data byte in `sc8545_protocol.h`);
/// a NACK is assumed to carry only its CRC.
const ACK_LEN: usize = 3;
const NACK_LEN: usize = 2;
/// Ping length in UI: Table B.4 `Tp-MT/16/Tb-MT`.
pub(super) const PING_UI: u32 = 16;
/// Charger ping response window and the wait before declaring a failure,
/// in UI (Figure B.6 and Table B.4 `Twait_f`).
const RESPONSE_UI: u32 = 5;
/// Idle UI after the closing ping exchange, within Table B.4's 1–5 UI
/// transaction interval `Twait`.
const SETTLE_UI: u32 = 4;
/// Received ping duration in µs (Table B.4 `Tp-MR`).
pub(super) const PING_MIN_US: u32 = 2_304;
pub(super) const PING_MAX_US: u32 = 2_816;
/// Transmit timing may run 10% long: Table B.4 `Tb-MT` 144–176 µs,
/// `Tb/4-MT` 36–44 µs and `Tp-MT` 2304–2816 µs around the nominal value.
/// Overhead may stretch a ping, never a sync by more than 4 µs.
const TX_TOLERANCE_DIVISOR: u32 = 10;
/// Longest single sample in µs. Implementation choice: at least four
/// observations within the shortest received sync (Table B.4 `Tb/4-MR`, 36 µs).
const SAMPLE_MAX_US: u32 = 8;
/// Polling step in µs. Implementation choice for sub-sync edge timestamps.
const POLL_US: u32 = 1;
/// Attempts per register. B.3.8 retries damaged transactions at most three
/// times; Huawei's `SC8545_SCP_RETRY_TIME` makes three attempts in total.
const ATTEMPTS: u32 = 3;
/// Released-bus UI before another attempt. Longer than the most a failed
/// read can leave the charger sending: ping, low UI, three bytes with sync
/// (9¾ UI each), end sync, ping and low UI, about 64 UI.
const RECOVERY_UI: u32 = 100;

/// Read one register, retrying damaged or unanswered transactions.
/// D− is released after every attempt.
pub(super) fn read<T: Transport>(phy: &mut T, register: Register) -> Result<u8, Error<T::Error>> {
    read_address(phy, register as u8)
}

pub(super) fn read_address<T: Transport>(phy: &mut T, register: u8) -> Result<u8, Error<T::Error>> {
    for _ in 1..ATTEMPTS {
        match attempt(phy, &[READ, register], true) {
            Err(error) if error.is_retryable() => phy.delay_us(RECOVERY_UI * UI_US),
            result => return result,
        }
    }
    attempt(phy, &[READ, register], true)
}

/// Control writes are never retried: a missing reply may hide an applied
/// write. The caller must release the session on any ambiguous outcome.
/// Two-byte values use one packet, avoiding partially updated setpoints.
pub(super) fn write<T: Transport>(
    phy: &mut T,
    register: u8,
    value: &[u8],
) -> Result<(), Error<T::Error>> {
    match value {
        [byte] => attempt(phy, &[WRITE, register, *byte], false).map(|_| ()),
        [high, low] => attempt(phy, &[MULTI_WRITE, register, 2, *high, *low], false).map(|_| ()),
        // All callers are internal and use one- or two-byte control registers.
        _ => Err(Error::Framing),
    }
}

fn attempt<T: Transport>(phy: &mut T, command: &[u8], read: bool) -> Result<u8, Error<T::Error>> {
    let result = transaction(phy, command, read);
    phy.drive(None).map_err(Error::Transport)?;
    result
}

/// Figures B.18 and B.19: ping exchange, command, ping exchange, reply,
/// ping exchange. A complete reply is checked only after the closing
/// exchange, so a parity or CRC failure still leaves the bus idle.
fn transaction<T: Transport>(
    phy: &mut T,
    command: &[u8],
    read: bool,
) -> Result<u8, Error<T::Error>> {
    let ui = exchange_ping(phy)?;
    // The charger drives the bus low for one UI after its ping.
    phy.delay_us(ui);
    send(phy, command)?;
    let ui = exchange_ping(phy)?;
    let reply = receive(phy, ui, read)?;
    phy.delay_us(ui);
    let ui = exchange_ping(phy)?;
    // Respect the charger's final active-low interval and idle settle.
    phy.delay_us(SETTLE_UI * ui);
    reply.value()
}

/// Send the command and its CRC, ending with the sync that leads into the
/// terminal ping (Figures B.8–B.10).
fn send<T: Transport>(phy: &mut T, command: &[u8]) -> Result<(), Error<T::Error>> {
    // The bus is low after the charger's active-low interval.
    let mut previous = false;
    for byte in command
        .iter()
        .copied()
        .chain(core::iter::once(CRC.checksum(command)))
    {
        sync(phy, previous, byte & 0x80 != 0)?;
        for bit in (0..8).rev() {
            hold(phy, byte & (1 << bit) != 0, UI_US)?;
        }
        previous = parity_bit(byte);
        hold(phy, previous, UI_US)?;
    }
    sync(phy, previous, true)
}

/// A complete charger reply, checked once the bus is idle again.
struct Reply {
    bytes: [u8; ACK_LEN],
    len: usize,
    parity_ok: bool,
}

impl Reply {
    /// B.3.7.2 puts every data byte of the packet under the CRC, so it is
    /// checked over the acknowledgement as well. Not yet confirmed by a
    /// capture from real hardware.
    fn value<E>(&self) -> Result<u8, Error<E>> {
        let (payload, crc) = self.bytes[..self.len].split_at(self.len - 1);
        if !self.parity_ok {
            Err(Error::Parity)
        } else if CRC.checksum(payload) != crc[0] {
            Err(Error::Crc)
        } else if self.bytes[0] & ACK_MASK == NACK {
            Err(Error::Nack)
        } else {
            // Write acknowledgements have no data byte; their caller only
            // consumes success after parity/CRC and the closing ping pass.
            Ok(payload.get(1).copied().unwrap_or_default())
        }
    }
}

/// Receive the reply bytes and the charger ping that ends the transfer.
/// The acknowledgement byte decides how many bytes follow it.
fn receive<T: Transport>(phy: &mut T, ui: u32, read: bool) -> Result<Reply, Error<T::Error>> {
    let (ack, mut previous) = receive_byte(phy, false, ui)?;
    let len = match ack & ACK_MASK {
        ACK if read => ACK_LEN,
        ACK => NACK_LEN,
        NACK => NACK_LEN,
        _ => return Err(Error::Ack),
    };
    let mut reply = Reply {
        bytes: [ack, 0, 0],
        len,
        parity_ok: previous == parity_bit(ack),
    };
    for slot in &mut reply.bytes[1..len] {
        let (byte, parity) = receive_byte(phy, previous, ui)?;
        reply.parity_ok &= parity == parity_bit(byte);
        *slot = byte;
        previous = parity;
    }
    let (start, high) = receive_sync(phy, previous, ui)?;
    if !high {
        return Err(Error::Framing);
    }
    ping_end(phy, start)?;
    Ok(reply)
}

/// The odd-parity bit that follows `byte` (Appendix B.3.7.1).
pub(super) fn parity_bit(byte: u8) -> bool {
    byte.count_ones().is_multiple_of(2)
}

/// Send the terminal ping and return the UI measured from the charger's.
fn exchange_ping<T: Transport>(phy: &mut T) -> Result<u32, Error<T::Error>> {
    let end = ping(phy)?;
    let start = wait_level(phy, true, end + RESPONSE_UI * UI_US)?;
    ping_end(phy, start)
}

fn ping<T: Transport>(phy: &mut T) -> Result<u32, Error<T::Error>> {
    hold(phy, true, PING_UI * UI_US)?;
    let end = phy.now_us();
    // Appendix B requires one UI actively low before releasing the bus.
    hold(phy, false, UI_US)?;
    phy.drive(None).map_err(Error::Transport)?;
    Ok(end)
}

/// Wait for a charger ping that began at `start` to end, check its width
/// and derive the receive UI from it.
fn ping_end<T: Transport>(phy: &mut T, start: u32) -> Result<u32, Error<T::Error>> {
    let end = wait_level(phy, false, start + PING_MAX_US)?;
    let width = end.saturating_sub(start);
    if !(PING_MIN_US..=PING_MAX_US).contains(&width) {
        return Err(Error::Framing);
    }
    Ok(width / PING_UI)
}

/// Appendix B.3.3: drive at least three edges between `previous` (the bus
/// level, or the last parity bit) and the `next` level.
fn sync<T: Transport>(phy: &mut T, previous: bool, next: bool) -> Result<(), Error<T::Error>> {
    hold(phy, !previous, UI_US / 4)?;
    hold(phy, previous, UI_US / 4)?;
    if previous == next {
        hold(phy, !previous, UI_US / 4)?;
    }
    Ok(())
}

/// Decode a two or three ¼-UI sync and return where the next bit starts
/// and its level. The level half a UI after the third edge tells a long
/// first bit apart from a third sync pulse.
fn receive_sync<T: Transport>(
    phy: &mut T,
    previous: bool,
    ui: u32,
) -> Result<(u32, bool), Error<T::Error>> {
    let quarter = ui / 4;
    let first = wait_level(phy, !previous, phy.now_us() + RESPONSE_UI * ui)?;
    let second = wait_level(phy, previous, first + quarter + SAMPLE_MAX_US)?;
    if second - first + SAMPLE_MAX_US < quarter {
        return Err(Error::Framing);
    }
    let third = wait_level(phy, !previous, second + quarter + SAMPLE_MAX_US)?;
    if third - second + SAMPLE_MAX_US < quarter {
        return Err(Error::Framing);
    }
    wait_until(phy, third + ui / 2)?;
    let level = sample(phy)?;
    let start = if level == previous {
        third + quarter
    } else {
        third
    };
    Ok((start, level))
}

/// Receive a sync, eight MSB-first data bits and the parity bit, each
/// sampled at its centre. Returns the byte and its received parity bit.
fn receive_byte<T: Transport>(
    phy: &mut T,
    previous: bool,
    ui: u32,
) -> Result<(u8, bool), Error<T::Error>> {
    let (start, _) = receive_sync(phy, previous, ui)?;
    let mut byte = 0;
    for index in 0..8 {
        wait_until(phy, start + ui * index + ui / 2)?;
        byte = (byte << 1) | u8::from(sample(phy)?);
    }
    wait_until(phy, start + 8 * ui + ui / 2)?;
    let parity = sample(phy)?;
    wait_until(phy, start + 9 * ui)?;
    Ok((byte, parity))
}

fn sample<T: Transport>(phy: &mut T) -> Result<bool, Error<T::Error>> {
    let before = phy.now_us();
    let high = phy.sample().map_err(Error::Transport)?;
    if phy.now_us().saturating_sub(before) > SAMPLE_MAX_US {
        return Err(Error::Timing);
    }
    Ok(high)
}

/// Wait for `deadline`. Arriving more than one sample late is a timing
/// failure, never a silently shifted sample point.
fn wait_until<T: Transport>(phy: &mut T, deadline: u32) -> Result<(), Error<T::Error>> {
    if phy.now_us() > deadline + SAMPLE_MAX_US {
        return Err(Error::Timing);
    }
    while phy.now_us() < deadline {
        phy.delay_us(POLL_US);
    }
    Ok(())
}

/// Poll until the bus reads `high`, returning when it was seen.
fn wait_level<T: Transport>(
    phy: &mut T,
    high: bool,
    deadline: u32,
) -> Result<u32, Error<T::Error>> {
    loop {
        let level = sample(phy)?;
        let now = phy.now_us();
        if now > deadline {
            return Err(Error::Timeout);
        }
        if level == high {
            return Ok(now);
        }
        phy.delay_us(POLL_US);
    }
}

fn hold<T: Transport>(phy: &mut T, high: bool, us: u32) -> Result<(), Error<T::Error>> {
    let before = phy.now_us();
    phy.drive(Some(high)).map_err(Error::Transport)?;
    phy.delay_us(us);
    let allowed = us + us / TX_TOLERANCE_DIVISOR;
    if phy.now_us().saturating_sub(before) > allowed {
        return Err(Error::Timing);
    }
    Ok(())
}
