//! The passive decoder sees only bus edges built from Appendix B framing,
//! with the terminal and charger on different clocks and jittered edges.
use scp_protocol::sniff::{Access, Decoder, Protocol, Transaction};

/// Terminal and charger UIs in µs, both inside Table B.4's 144–176 µs.
const TERMINAL_UI: u32 = 148;
const CHARGER_UI: u32 = 174;

struct Bus {
    at: u32,
    high: bool,
    edges: Vec<(u32, bool)>,
    /// Edge timestamp latencies in µs, cycled: within the decoder's 0–8 µs.
    jitter: std::iter::Cycle<std::array::IntoIter<i32, 5>>,
}

impl Bus {
    fn new() -> Self {
        Self {
            at: 0,
            high: false,
            edges: vec![],
            jitter: [0, 6, 1, 8, 3].into_iter().cycle(),
        }
    }

    fn hold(&mut self, high: bool, us: u32) {
        if high != self.high {
            let offset = self.jitter.next().unwrap();
            self.edges.push((self.at.wrapping_add_signed(offset), high));
            self.high = high;
        }
        self.at = self.at.wrapping_add(us);
    }

    /// A ping and the sender's active-low UI, then the released bus.
    fn ping(&mut self, ui: u32) {
        self.hold(true, 16 * ui);
        self.hold(false, 3 * ui);
    }

    fn sync(&mut self, ui: u32, previous: bool, next: bool) {
        self.hold(!previous, ui / 4);
        self.hold(previous, ui / 4);
        if previous == next {
            self.hold(!previous, ui / 4);
        }
    }

    /// `bytes` and their CRC (or a damaged one), closed by the sender's ping.
    fn packet(&mut self, ui: u32, bytes: &[u8], crc_error: u8) {
        let mut previous = false;
        for byte in bytes.iter().copied().chain([checksum(bytes) ^ crc_error]) {
            self.sync(ui, previous, byte & 0x80 != 0);
            for bit in (0..8).rev() {
                self.hold(byte & (1 << bit) != 0, ui);
            }
            previous = byte.count_ones().is_multiple_of(2);
            self.hold(previous, ui);
        }
        self.sync(ui, previous, true);
        self.ping(ui);
    }

    /// Figures B.18/B.19: ping exchange, command, ping exchange, reply and
    /// a closing ping exchange.
    fn transaction(&mut self, command: &[u8], reply: &[u8], crc_error: u8) {
        self.ping(TERMINAL_UI);
        self.ping(CHARGER_UI);
        self.packet(TERMINAL_UI, command, 0);
        self.ping(CHARGER_UI);
        self.packet(CHARGER_UI, reply, crc_error);
        self.ping(TERMINAL_UI);
        self.ping(CHARGER_UI);
        self.hold(false, 5_000);
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

fn decode(edges: &[(u32, bool)]) -> Vec<Transaction> {
    let mut decoder = Decoder::default();
    edges
        .iter()
        .filter_map(|&(at, high)| decoder.push(at, high))
        .collect()
}

fn summary(transaction: &Transaction) -> (Access, u8, bool, Vec<u8>) {
    (
        transaction.access,
        transaction.register,
        transaction.acked,
        transaction.data().to_vec(),
    )
}

#[test]
fn reads_and_writes_decode_across_clocks_and_jitter() {
    let mut bus = Bus::new();
    // Start near the clock's wrap so timestamps roll over mid-stream.
    bus.at = u32::MAX - 20_000;
    bus.transaction(&[0x0c, 0x7e], &[0x08, 0xb8], 0);
    bus.transaction(&[0x0c, 0x81], &[0x03], 0);
    bus.transaction(&[0x0b, 0x2c, 90], &[0x88], 0);
    bus.transaction(&[0x1b, 0xb8, 2, 0x2a, 0xf8], &[0x08], 0);
    bus.transaction(&[0x1c, 0x92, 2], &[0x08, 0x37, 0xcb], 0);
    let decoded: Vec<_> = decode(&bus.edges).iter().map(summary).collect();
    assert_eq!(
        decoded,
        [
            (Access::Read, 0x7e, true, vec![0xb8]),
            (Access::Read, 0x81, false, vec![]),
            (Access::Write, 0x2c, true, vec![90]),
            (Access::Write, 0xb8, true, vec![0x2a, 0xf8]),
            (Access::Read, 0x92, true, vec![0x37, 0xcb]),
        ]
    );
}

#[test]
fn every_byte_value_survives_both_sync_forms() {
    let mut bus = Bus::new();
    for value in 0..=u8::MAX {
        bus.transaction(&[0x0b, value, !value], &[0x08], 0);
    }
    let decoded = decode(&bus.edges);
    assert_eq!(decoded.len(), 256);
    for (value, transaction) in (0..=u8::MAX).zip(&decoded) {
        assert_eq!(transaction.register, value);
        assert_eq!(transaction.data(), [!value]);
    }
}

#[test]
fn damaged_or_partial_traffic_is_dropped_until_the_next_ping() {
    let mut bus = Bus::new();
    bus.transaction(&[0x0c, 0x7e], &[0x08, 0xb8], 1);
    bus.transaction(&[0x0c, 0x80], &[0x08, 0x90], 0);
    let edges = bus.edges.clone();
    let decoded = decode(&edges);
    assert_eq!(decoded.len(), 1);
    assert_eq!(decoded[0].register, 0x80);
    // Joining mid-transaction, or missing one edge, loses only that one.
    let second = edges.len() / 2;
    assert_eq!(decode(&edges[second - 40..]).len(), 1);
    let mut missed = edges.clone();
    missed.remove(60);
    assert_eq!(decode(&missed).len(), 1);
    // A command whose reply never comes is not paired with a later reply.
    let mut bus = Bus::new();
    bus.ping(TERMINAL_UI);
    bus.ping(CHARGER_UI);
    bus.packet(TERMINAL_UI, &[0x0c, 0x7e], 0);
    bus.transaction(&[0x0c, 0x80], &[0x08, 0x90], 0);
    let decoded = decode(&bus.edges);
    assert_eq!(decoded.len(), 1);
    assert_eq!(decoded[0].register, 0x80);
}

#[test]
fn registers_name_their_protocol_and_voltage_writes_their_target() {
    let mut bus = Bus::new();
    bus.transaction(&[0x0c, 0x7e], &[0x08, 0xb8], 0);
    bus.transaction(&[0x0b, 0x2c, 90], &[0x08], 0);
    bus.transaction(&[0x0b, 0x2c, 120], &[0x03], 0);
    bus.transaction(&[0x1b, 0xb8, 2, 0x2a, 0xf8], &[0x08], 0);
    bus.transaction(&[0x0b, 0xca, 200], &[0x08], 0);
    // Voltage and current in one write from the voltage register.
    bus.transaction(&[0x1b, 0xb8, 4, 0x23, 0x28, 0x0b, 0xb8], &[0x08], 0);
    let decoded: Vec<_> = decode(&bus.edges)
        .iter()
        .map(|transaction| (transaction.protocol(), transaction.requested_mv()))
        .collect();
    assert_eq!(
        decoded,
        [
            (None, None),
            (Some(Protocol::Fcp), Some(9_000)),
            // A rejected write requested nothing the charger took.
            (Some(Protocol::Fcp), None),
            (Some(Protocol::Scp), Some(11_000)),
            (Some(Protocol::Scp), Some(5_000)),
            (Some(Protocol::Scp), Some(9_000)),
        ]
    );
}

#[test]
fn a_reply_is_never_paired_with_a_command_before_lost_sync() {
    let mut bus = Bus::new();
    bus.ping(TERMINAL_UI);
    bus.ping(CHARGER_UI);
    bus.packet(TERMINAL_UI, &[0x0b, 0x2c, 90], 0);
    bus.ping(CHARGER_UI);
    // A glitch where the reply should start, then a reply whose command
    // was lost: neither may complete the write above.
    bus.hold(true, 10);
    bus.hold(false, 3_000);
    bus.ping(TERMINAL_UI);
    bus.ping(CHARGER_UI);
    bus.packet(CHARGER_UI, &[0x08], 0);
    bus.ping(TERMINAL_UI);
    bus.ping(CHARGER_UI);
    assert!(decode(&bus.edges).is_empty());
}
