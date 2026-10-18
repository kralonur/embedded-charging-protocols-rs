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
    WriteAck,
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
    registers: Option<[u8; 256]>,
    commands: Vec<Vec<u8>>,
    command_start: usize,
    reject_write: bool,
    corrupt_readback: bool,
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
            registers: None,
            commands: vec![],
            command_start: 0,
            reject_write: false,
            corrupt_readback: false,
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
        let reply = if let Some(registers) = &mut self.registers {
            let packet: Vec<u8> = self.read_bits[self.command_start + 1..]
                .as_chunks::<9>()
                .0
                .iter()
                .map(|bits| {
                    bits[..8]
                        .iter()
                        .fold(0, |value, bit| (value << 1) | u8::from(*bit))
                })
                .collect();
            self.command_start = self.read_bits.len();
            let (crc, command) = packet.split_last().unwrap();
            assert_eq!(*crc, checksum(command));
            self.commands.push(command.to_vec());
            match command {
                [0x0c, address] => Reply::Ack(registers[usize::from(*address)]),
                [0x0b, address, value] if !self.reject_write => {
                    registers[usize::from(*address)] = value ^ u8::from(self.corrupt_readback);
                    Reply::WriteAck
                }
                [0x1b, address, 2, high, low] if !self.reject_write => {
                    registers[usize::from(*address)] = *high;
                    registers[usize::from(*address) + 1] = low ^ u8::from(self.corrupt_readback);
                    Reply::WriteAck
                }
                _ => Reply::Nack,
            }
        } else {
            self.replies.pop_front().unwrap_or(Reply::Nack)
        };
        let mut packet = match reply {
            Reply::Ack(data) => vec![0x08, data],
            Reply::Nack => vec![0x03],
            Reply::WriteAck => vec![0x08],
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
    // A NACK at either legacy register, as from an FCP-only adapter, means
    // no SCP rather than an undetermined result.
    for replies in [
        &[Reply::Ack(0xa0), Reply::Ack(0x80)][..],
        &[Reply::Nack, Reply::Nack],
        &[Reply::Nack, Reply::Ack(0x90), Reply::Nack],
    ] {
        let mut source = Source::new(replies.iter().copied());
        assert_eq!(detect(&mut source), Err(Error::NotScp));
        assert_eq!(source.level, None);
    }
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

fn controlled_adapter() -> scp_protocol::Adapter {
    scp_protocol::Adapter {
        modes: Modes::HighVoltage,
        type0: Some(0x1c),
        type1: None,
        capabilities: Some(Capabilities {
            min_mv: 5_500,
            max_mv: 11_000,
            min_ma: 300,
            max_ma: 3_200,
            vstep_mv: Some(20),
            istep_ma: Some(50),
            max_power_dw: Some(350),
        }),
    }
}

fn controlled_source() -> Source {
    let mut source = Source::new([]);
    let mut registers = [0; 256];
    registers[0xa0] = 0x80; // Ordinary mode, output enabled.
    registers[0xa1] = 2; // Default one-second watchdog.
    source.registers = Some(registers);
    source
}

// Output writes must be atomic, acknowledged, read back and ordered so
// changing setpoints cannot briefly exceed the adapter's power rating.
/// Run every queued control transaction, stopping at the first error.
fn run<T: scp_protocol::Transport>(
    session: &mut scp_protocol::control::Session,
    phy: &mut T,
) -> Result<(), scp_protocol::control::ControlError<T::Error>> {
    while session.busy() {
        session.poll(phy)?;
    }
    Ok(())
}

#[test]
fn output_control_checks_writes_preserves_protections_and_stops() {
    use scp_protocol::control::{Output, RequestError, Session};
    let adapter = controlled_adapter();
    let output = Output::new(adapter, 11_000, 3_150).unwrap();
    let mut source = controlled_source();
    let mut session = Session::start(adapter, output).unwrap();
    // Each poll is exactly one transaction, so callers stay responsive.
    while session.busy() {
        let before = source.commands.len();
        session.poll(&mut source).unwrap();
        assert_eq!(source.commands.len(), before + 1);
        if session.busy() {
            // No second procedure may queue behind a running one.
            assert_eq!(session.set(output), Err(RequestError::Busy));
        }
    }
    assert_eq!(session.output(), output);
    let registers = source.registers.as_ref().unwrap();
    assert_eq!(registers[0xa0], 0xc1); // Mode + output + temperature; D+ protection stays on.
    assert_eq!(registers[0xa1], 4); // Finite two-second watchdog.
    assert_eq!(&registers[0xb8..=0xbb], &[0x2a, 0xf8, 0x0c, 0x4e]);
    let start = source.commands.len();
    session.set(session.request(6_000, 3_200).unwrap()).unwrap();
    run(&mut session, &mut source).unwrap();
    let writes: Vec<_> = source.commands[start..]
        .iter()
        .filter(|command| command[0] != 0x0c)
        .collect();
    assert_eq!(writes[0], &[0x1b, 0xba, 2, 0x0c, 0x4e]); // Keep 3.15 A before voltage falls.
    assert_eq!(writes[writes.len() - 2], &[0x1b, 0xb8, 2, 0x17, 0x70]);
    assert_eq!(writes[writes.len() - 1], &[0x1b, 0xba, 2, 0x0c, 0x80]); // Only then raise to 3.2 A.
    session.refresh();
    run(&mut session, &mut source).unwrap();
    session.stop();
    run(&mut session, &mut source).unwrap();
    assert_eq!(source.registers.as_ref().unwrap()[0xa0], 0x81);
    assert_eq!(source.level, None);
    assert!(!source.contention);
}

#[test]
fn output_control_rejects_bad_requests_and_ambiguous_writes_without_retry() {
    use scp_protocol::control::{ControlError, Output, RequestError, Session};
    let adapter = controlled_adapter();
    for (mv, ma, error) in [
        (12_000, 1_000, RequestError::Range),
        (9_001, 1_000, RequestError::Step),
        (9_000, 0, RequestError::Range),
        (11_000, 3_200, RequestError::Power),
    ] {
        assert_eq!(Output::new(adapter, mv, ma), Err(error));
    }
    for corrupt in [false, true] {
        let mut source = controlled_source();
        source.reject_write = !corrupt;
        source.corrupt_readback = corrupt;
        let mut session =
            Session::start(adapter, Output::new(adapter, 9_000, 1_000).unwrap()).unwrap();
        let result = run(&mut session, &mut source);
        assert!(matches!(
            result,
            Err(ControlError::Wire(Error::Nack)) | Err(ControlError::Readback)
        ));
        assert_eq!(
            source
                .commands
                .iter()
                .filter(|command| command[0] != 0x0c)
                .count(),
            1
        );
        assert_eq!(source.level, None);
    }
    let mut source = controlled_source();
    let mut session = Session::start(adapter, Output::new(adapter, 9_000, 1_000).unwrap()).unwrap();
    run(&mut session, &mut source).unwrap();
    source.registers.as_mut().unwrap()[0xa0] &= !0x40; // Adapter left SCP mode.
    session.refresh();
    assert_eq!(
        run(&mut session, &mut source),
        Err(ControlError::Protection)
    );
}

fn fcp_source() -> Source {
    let mut source = Source::new([]);
    let mut registers = [0; 256];
    registers[0x21] = 2; // Three voltages; the first defaults to 5.2 V (Table B.11).
    registers[0x22] = 36; // 18 W.
    registers[0x30..=0x32].copy_from_slice(&[52, 90, 120]);
    source.registers = Some(registers);
    source
}

fn fcp_detect(source: &mut Source) -> Result<scp_protocol::fcp::Adapter, Error<()>> {
    let mut detection = scp_protocol::fcp::Detection::new();
    loop {
        if let Some(adapter) = detection.poll(source)? {
            return Ok(adapter);
        }
    }
}

// FCP sets a listed voltage, reads it back and only then applies it;
// anything the adapter does not list is refused before any write.
#[test]
fn fcp_detects_listed_voltages_and_applies_only_checked_writes() {
    use scp_protocol::control::{ControlError, RequestError};
    use scp_protocol::fcp::{Output, Session};
    let mut source = fcp_source();
    let adapter = fcp_detect(&mut source).unwrap();
    assert_eq!(adapter.levels(), [5_200, 9_000, 12_000]);
    assert_eq!(adapter.max_power_mw, Some(18_000));
    assert_eq!(Output::new(adapter, 10_000), Err(RequestError::Range));
    // A level the one-byte register cannot hold would be written as another.
    for levels in [[5_000, 9_050], [5_000, 25_600]] {
        assert_eq!(scp_protocol::fcp::Adapter::new(&levels, None), None);
    }
    // A fault latched before the session is read to clear it, not judged.
    source.registers.as_mut().unwrap()[0x28] = 0x08;
    let start = source.commands.len();
    let mut session = Session::start(adapter, Output::new(adapter, 9_000).unwrap()).unwrap();
    while session.busy() {
        session.poll(&mut source).unwrap();
    }
    assert_eq!(
        source.commands[start..],
        [
            vec![0x0c, 0x28],
            vec![0x0b, 0x2c, 90],
            vec![0x0c, 0x2c],
            vec![0x0b, 0x2b, 1],
        ]
    );
    session.stop();
    while session.busy() {
        session.poll(&mut source).unwrap();
    }
    assert_eq!(source.registers.as_ref().unwrap()[0x2c], 52);
    assert_eq!(source.level, None);
    source.registers.as_mut().unwrap()[0x28] = 0x04; // Overvoltage.
    session.refresh();
    assert_eq!(session.poll(&mut source), Err(ControlError::Protection));

    let mut source = fcp_source();
    source.registers.as_mut().unwrap()[0x21] = 3;
    assert_eq!(fcp_detect(&mut source), Err(Error::NotFcp));
}
