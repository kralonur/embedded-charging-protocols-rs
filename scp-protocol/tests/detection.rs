//! Boundary tests exercise the complete wire exchange against a scheduled
//! source, including all sync/parity/CRC checks and legacy type fallback.
use scp_protocol::{Capabilities, Error, Modes, Transport, detect};
use std::collections::VecDeque;

const UI: u64 = 160;
const Q: u64 = 40;

#[derive(Clone, Copy)]
enum Reply {
    Ack(u8),
    Nack,
    BadCrc,
    BadParity,
}

struct Source {
    now: u64,
    level: Option<bool>,
    edges: Vec<(u64, bool)>,
    replies: VecDeque<Reply>,
    pings: usize,
    pending: bool,
    silent: bool,
    slow: bool,
    read_bits: Vec<bool>,
    contention: bool,
    stretch_sync: bool,
}

impl Source {
    fn new(replies: impl IntoIterator<Item = Reply>) -> Self {
        Self {
            now: 0,
            level: None,
            edges: vec![],
            replies: replies.into_iter().collect(),
            pings: 0,
            pending: false,
            silent: false,
            slow: false,
            read_bits: vec![],
            contention: false,
            stretch_sync: false,
        }
    }

    fn response(&mut self) {
        self.edges.clear();
        if self.silent {
            return;
        }
        let mut at = self.now + UI;
        self.edges.push((at, true));
        at += 16 * UI;
        self.edges.push((at, false));
        if self.pings % 3 != 2 {
            return;
        }
        at += 3 * UI;
        let reply = self.replies.pop_front().unwrap_or(Reply::Nack);
        let mut packet = match reply {
            Reply::Ack(data) => vec![0x08, data],
            Reply::Nack => vec![0x03],
            Reply::BadCrc | Reply::BadParity => vec![0x08, 0x98],
        };
        let checksum = checksum(&packet);
        packet.push(if matches!(reply, Reply::BadCrc) {
            checksum ^ 1
        } else {
            checksum
        });
        let mut previous = false;
        for (index, byte) in packet.into_iter().enumerate() {
            let next = byte & 0x80 != 0;
            sync(&mut self.edges, &mut at, previous, next);
            for bit in (0..8).rev() {
                self.edges.push((at, byte & (1 << bit) != 0));
                at += UI;
            }
            previous = byte.count_ones().is_multiple_of(2);
            if matches!(reply, Reply::BadParity) && index == 0 {
                previous = !previous;
            }
            self.edges.push((at, previous));
            at += UI;
        }
        sync(&mut self.edges, &mut at, previous, true);
        self.edges.push((at, true));
        at += 16 * UI;
        self.edges.push((at, false));
    }
}

fn sync(edges: &mut Vec<(u64, bool)>, at: &mut u64, previous: bool, next: bool) {
    for high in [!previous, previous] {
        edges.push((*at, high));
        *at += Q;
    }
    if previous == next {
        edges.push((*at, !previous));
        *at += Q;
    }
}

// Independent bit-by-bit reference, not the library's CRC implementation.
fn checksum(bytes: &[u8]) -> u8 {
    let mut value = 0u8;
    for byte in bytes {
        value ^= byte;
        for _ in 0..8 {
            value = if value & 0x80 != 0 {
                (value << 1) ^ 0x39
            } else {
                value << 1
            };
        }
    }
    value
}

impl Transport for Source {
    type Error = ();
    fn drive(&mut self, high: Option<bool>) -> Result<(), ()> {
        if high.is_some()
            && self
                .edges
                .last()
                .is_some_and(|(at, _)| self.now >= *at && self.now < *at + UI)
        {
            self.contention = true;
        }
        self.level = high;
        if high.is_none() && self.pending {
            self.pending = false;
            self.response();
        }
        Ok(())
    }
    fn sample(&mut self) -> Result<bool, ()> {
        if self.slow {
            self.now += 9;
        }
        Ok(self
            .edges
            .iter()
            .rev()
            .find(|(at, _)| *at <= self.now)
            .is_some_and(|(_, high)| *high))
    }
    fn now_us(&self) -> u32 {
        self.now as u32
    }
    fn delay_us(&mut self, us: u32) {
        if self.level == Some(true) && us == 2_560 {
            self.pings += 1;
            self.pending = true;
        }
        if self.pings % 3 == 1
            && us == 160
            && let Some(high) = self.level
        {
            self.read_bits.push(high);
        }
        self.now += u64::from(us);
        if self.stretch_sync && us == 40 && self.level.is_some() {
            self.now += 5;
        }
    }
}

#[test]
fn modern_adapter_requires_a_checked_read_and_reports_modes() {
    assert_eq!(checksum(b"123456789"), 0x8f);
    assert_eq!(checksum(&[0x0c, 0x7e]), 0x12);
    assert_eq!(checksum(&[0x08, 0xb8]), 0xef);
    for (data, modes) in [
        (0x90, Modes::LowVoltage),
        (0x9c, Modes::HighVoltage),
        (0xb8, Modes::Both),
    ] {
        let mut source = Source::new([Reply::Ack(data)]);
        let adapter = detect(&mut source).unwrap();
        assert_eq!(adapter.modes, modes);
        assert_eq!(adapter.type0, Some(data));
        assert_eq!(adapter.capabilities, None);
        assert_eq!(source.level, None);
        assert_eq!(source.pings, 6);
        assert_eq!(
            &source.read_bits[1..9],
            &[false, false, false, false, true, true, false, false]
        );
        assert!(source.now < 80_000);
    }
    // Verify Table B.19 specification register decoding, including 0x9c current extension.
    let mut source = Source::new([
        Reply::Ack(0xb8),          // 0x7e Type0: Both
        Reply::Ack((2 << 6) | 55), // 0x92 MinVout: 55 * 10^2 = 5500 mV
        Reply::Ack((3 << 6) | 11), // 0x93 MaxVout: 11 * 10^3 = 11000 mV
        Reply::Ack((2 << 6) | 10), // 0x94 MinIout: 10 * 10^2 = 1000 mA
        Reply::Ack((2 << 6) | 60), // 0x95 MaxIout: 60 * 10^2 = 6000 mA (>= 6A threshold)
        Reply::Ack(60),            // 0x9c MaxIoutExtend: +600 mA -> 6600 mA
        Reply::Ack((1 << 7) | 2),  // 0x96 VStep: 2 * 10^1 = 20 mV
        Reply::Ack((1 << 7) | 5),  // 0x97 IStep: 5 * 10^1 = 50 mA
        Reply::Ack((1 << 7) | 66), // 0x90 MaxPower: 66 * 10^1 = 660 (66.0 W)
    ]);
    let adapter = detect(&mut source).unwrap();
    assert_eq!(
        adapter.capabilities,
        Some(Capabilities {
            min_mv: 5_500,
            max_mv: 11_000,
            min_ma: 1_000,
            max_ma: 6_600,
            vstep_mv: Some(20),
            istep_ma: Some(50),
            max_power_dw: Some(660),
        })
    );
}

#[test]
fn legacy_detection_requires_both_type_registers() {
    let mut source = Source::new([Reply::Nack, Reply::Ack(0x90), Reply::Ack(0x10)]);
    let adapter = detect(&mut source).unwrap();
    assert_eq!(adapter.modes, Modes::LowVoltage);
    assert_eq!(adapter.type0, None);
    assert_eq!(adapter.type1, Some(0x90));
    assert_eq!(adapter.capabilities, None);
    assert_eq!(source.pings, 12);
    assert!(!source.contention);
    for (packet, register) in source
        .read_bits
        .as_chunks::<28>()
        .0
        .iter()
        .zip([0x7e, 0x80, 0x81])
    {
        let decode = |bits: &[bool]| {
            bits.iter()
                .fold(0u8, |value, bit| (value << 1) | u8::from(*bit))
        };
        assert_eq!(decode(&packet[1..9]), 0x0c);
        assert_eq!(decode(&packet[10..18]), register);
        assert_eq!(decode(&packet[19..27]), checksum(&[0x0c, register]));
    }
    let mut source = Source::new([Reply::Ack(0xa0), Reply::Ack(0x80)]);
    assert_eq!(detect(&mut source), Err(Error::NotScp));
    assert_eq!(source.level, None);
}

#[test]
fn programmable_type_patterns_preserve_all_data_and_parity_combinations() {
    for value in 0..=u8::MAX {
        if value & 0x10 == 0 || (value & 0x04 != 0 && value & 0x08 == 0) {
            continue;
        }
        let mut source = Source::new([Reply::Ack(value)]);
        assert_eq!(detect(&mut source).unwrap().type0, Some(value));
        assert!(!source.contention);
    }
}

#[test]
fn corrupt_packets_do_not_fall_back_or_claim_scp() {
    for (reply, expected) in [
        (Reply::BadCrc, Error::Crc),
        (Reply::BadParity, Error::Parity),
    ] {
        // Three attempts, each closing its ping exchange.
        let mut source = Source::new([reply; 3]);
        assert_eq!(detect(&mut source), Err(expected));
        assert_eq!(source.level, None);
        assert_eq!(source.pings, 9);
        assert!(!source.contention);
        // A retry after one damaged reply recovers the same register.
        let mut source = Source::new([reply, Reply::Ack(0xb8)]);
        assert_eq!(
            detect(&mut source).map(|adapter| adapter.modes),
            Ok(Modes::Both)
        );
        assert!(!source.contention);
    }
}

#[test]
fn absent_or_slow_transport_fails_bounded_and_releases() {
    let mut source = Source::new([]);
    source.silent = true;
    assert_eq!(detect(&mut source), Err(Error::Timeout));
    assert_eq!(source.level, None);
    // Three unanswered pings (about 3.5 ms each) and two 16 ms recovery gaps.
    assert!(source.now < 48_000);
    let mut source = Source::new([]);
    source.slow = true;
    assert_eq!(detect(&mut source), Err(Error::Timing));
    assert_eq!(source.level, None);
    let mut source = Source::new([]);
    source.stretch_sync = true;
    assert_eq!(detect(&mut source), Err(Error::Timing));
    assert_eq!(source.level, None);
}
