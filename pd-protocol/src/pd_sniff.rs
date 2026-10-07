//! Receive-only PD bus log for a passive passthrough sniffer: message labels and
//! fixed-width text. CC sampling belongs to the PHY adapter. Pure decoding; it never builds, sends or
//! acknowledges a message. Codes: USB PD R3.2 V1.2 Tables 6.4 (control), 6.5
//! (data), 6.47 (extended). Fields: Tables 6.9..6.16 (PDOs), 6.19/6.21 (RDOs).

use crate::pd_constants::*;
use usbpd::protocol_layer::message::header::ControlMessageType;

// Milli-units per base unit (SI prefix), used only for decimal display.
const MILLI_PER_UNIT: u32 = 1_000;
// Base of the decimal output representation.
const DECIMAL_RADIX: u32 = 10;

/// Rp level on the active CC with the sniffing port's own Rd removed: the source's Rp
/// against the attached sink's Rd (Type-C default/1.5 A/3 A vRd ranges). Under a
/// PD3 Explicit Contract, 1.5 A is SinkTxNG and 3 A is SinkTxOK (§5.2.2 Table 5.1,
/// §7.2); otherwise it is only the advertised Type-C current. The PHY adapter
/// classifies it from its own comparator readings; this is not a voltage measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rp {
    Default,
    Current1A5,
    Current3A0,
}

/// GoodCRC: control, type 1, no data objects (Table 6.4). Counted, not logged.
pub fn is_goodcrc(header: u16) -> bool {
    header & HEADER_KIND_COUNT_MASK == ControlMessageType::GoodCRC as u16
}

/// Requested supply, decoded when the message is captured, against the
/// Source_Capabilities seen most recently (or the PDO copy in an EPR_Request).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Supply {
    Fixed {
        mv: u32,
        ma: u32,
    },
    Pps {
        mv: u32,
        ma: u32,
    },
    /// No matching PDO known, or a PDO kind this log does not decode.
    Raw,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    pub header: u16,
    /// First data object, or the extended header in the low 16 bits.
    pub word: u32,
    pub supply: Option<Supply>,
}

impl Entry {
    /// `caps`: raw PDOs of the latest Source_Capabilities, in order.
    pub fn classify(header: u16, data: &[u8], caps: &[u32]) -> Self {
        let word = |i: usize| {
            data.get(i * 4..i * 4 + 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        };
        let extended = header & HEADER_EXTENDED != 0;
        let entry_word = if extended {
            data.get(..2)
                .map_or(0, |b| u16::from_le_bytes([b[0], b[1]]) as u32)
        } else {
            word(0).unwrap_or(0)
        };
        let objects = (header >> 12) & 7;
        let kind = header & HEADER_TYPE_MASK;
        let supply = match (extended, objects, kind) {
            (false, 1.., 2) => word(0).map(|rdo| {
                let position = (rdo >> 28) as usize;
                let pdo = position.checked_sub(1).and_then(|i| caps.get(i).copied());
                decode_request(rdo, pdo)
            }),
            (false, 2.., 9) => word(0).map(|rdo| decode_request(rdo, word(1))),
            _ => None,
        };
        Self {
            header,
            word: entry_word,
            supply,
        }
    }
}

fn decode_request(rdo: u32, pdo: Option<u32>) -> Supply {
    match pdo {
        // Fixed PDO (Table 6.9/6.10) with a Fixed RDO (Table 6.19).
        Some(p) if p >> 30 == 0 => Supply::Fixed {
            mv: ((p >> 10) & 0x3ff) * FIXED_VOLTAGE_UNIT_MV,
            ma: ((rdo >> 10) & 0x3ff) * FIXED_CURRENT_UNIT_MA,
        },
        // SPR PPS APDO (Table 6.13) with a PPS RDO (Table 6.21).
        Some(p) if p >> 28 == 0b1100 => Supply::Pps {
            mv: ((rdo >> 9) & 0xfff) * PPS_RDO_VOLTAGE_UNIT_MV,
            ma: (rdo & 0x7f) * PPS_CURRENT_UNIT_MA,
        },
        _ => Supply::Raw,
    }
}

// Allocation-free diagnostic retention bound: seven most recent messages.
pub const LOG_LEN: usize = 7;

/// Latest `LOG_LEN` non-GoodCRC messages plus totals. Totals saturate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Log {
    entries: [Entry; LOG_LEN],
    len: u8,
    next: u8,
    pub messages: u32,
    pub goodcrc: u32,
}

impl Log {
    pub const fn new() -> Self {
        Self {
            entries: [Entry {
                header: 0,
                word: 0,
                supply: None,
            }; LOG_LEN],
            len: 0,
            next: 0,
            messages: 0,
            goodcrc: 0,
        }
    }
    pub fn push(&mut self, entry: Entry) {
        if is_goodcrc(entry.header) {
            self.goodcrc = self.goodcrc.saturating_add(1);
            return;
        }
        self.messages = self.messages.saturating_add(1);
        self.entries[self.next as usize] = entry;
        self.next = (self.next + 1) % LOG_LEN as u8;
        self.len = (self.len + 1).min(LOG_LEN as u8);
    }
    /// Oldest first.
    pub fn entries(&self) -> impl ExactSizeIterator<Item = &Entry> + '_ {
        let start = (self.next as usize + LOG_LEN - self.len as usize) % LOG_LEN;
        (0..self.len as usize).map(move |i| &self.entries[(start + i) % LOG_LEN])
    }
}

impl Default for Log {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-width text row; writes past column 24 are dropped.
pub struct Row {
    pub bytes: [u8; 24],
    at: usize,
}

impl Row {
    pub const fn new() -> Self {
        Self {
            bytes: [b' '; 24],
            at: 0,
        }
    }
    pub fn text(&mut self, text: &[u8]) -> &mut Self {
        for &b in text {
            if self.at < 24 {
                self.bytes[self.at] = b;
            }
            self.at += 1;
        }
        self
    }
    pub fn hex(&mut self, value: u32, digits: u32) -> &mut Self {
        for shift in (0..digits).rev() {
            let nibble = ((value >> (shift * 4)) & 0xf) as usize;
            self.text(&[b"0123456789ABCDEF"[nibble]]);
        }
        self
    }
    pub fn decimal(&mut self, value: u32) -> &mut Self {
        let mut digits = [0u8; 10];
        let mut n = value;
        let mut len = 0;
        loop {
            digits[len] = b'0' + (n % 10) as u8;
            len += 1;
            n /= 10;
            if n == 0 {
                break;
            }
        }
        for i in (0..len).rev() {
            self.text(&[digits[i]]);
        }
        self
    }
    /// `value` thousandths with `places` (1 or 2) decimals, truncated.
    pub fn fixed(&mut self, value: u32, places: u32) -> &mut Self {
        self.decimal(value / MILLI_PER_UNIT).text(b".");
        let fraction = (value % MILLI_PER_UNIT)
            / if places == 1 {
                MILLI_PER_UNIT / DECIMAL_RADIX
            } else {
                DECIMAL_RADIX
            };
        if places == 2 && fraction < 10 {
            self.text(b"0");
        }
        self.decimal(fraction)
    }
    pub fn column(&self) -> usize {
        self.at
    }
}

impl Default for Row {
    fn default() -> Self {
        Self::new()
    }
}

const CONTROL: [&[u8]; 25] = [
    b"CTRL",
    b"GOODCRC",
    b"GOTOMIN",
    b"ACCEPT",
    b"REJECT",
    b"PING",
    b"PS_RDY",
    b"GET_SRC_CAP",
    b"GET_SNK_CAP",
    b"DR_SWAP",
    b"PR_SWAP",
    b"VCONN_SWAP",
    b"WAIT",
    b"SOFT_RESET",
    b"DATA_RESET",
    b"DATA_RST_OK",
    b"NOT_SUPP",
    b"GET_SRC_CEX",
    b"GET_STATUS",
    b"FR_SWAP",
    b"GET_PPS_ST",
    b"GET_CCODES",
    b"GET_SNK_CEX",
    b"GET_SRC_INF",
    b"GET_REV",
];
const DATA: [&[u8]; 16] = [
    b"DATA",
    b"SRC_CAP",
    b"REQUEST",
    b"BIST",
    b"SNK_CAP",
    b"BAT_STATUS",
    b"ALERT",
    b"GET_CINFO",
    b"ENTER_USB",
    b"EPR_REQ",
    b"EPR_MODE",
    b"SRC_INFO",
    b"REVISION",
    b"DATA",
    b"DATA",
    b"VDM",
];
const EXTENDED: [&[u8]; 19] = [
    b"EXT",
    b"SRC_CAP_EXT",
    b"STATUS",
    b"GET_BAT_CAP",
    b"GET_BAT_ST",
    b"BAT_CAP",
    b"GET_MFR",
    b"MFR_INFO",
    b"SEC_REQ",
    b"SEC_RESP",
    b"FWU_REQ",
    b"FWU_RESP",
    b"PPS_STATUS",
    b"COUNTRY_INF",
    b"COUNTRY_COD",
    b"SNK_CAP_EXT",
    b"EXT_CTRL",
    b"EPR_SRC_CAP",
    b"EPR_SNK_CAP",
];

/// Label, or `None` for reserved codes (shown with their numeric type).
pub fn name(header: u16) -> Option<&'static [u8]> {
    let kind = (header & HEADER_TYPE_MASK) as usize;
    let objects = (header >> 12) & 7;
    let name = if header & 0x8000 != 0 {
        if kind == 30 {
            Some(&b"VDM_EXT"[..])
        } else {
            EXTENDED.get(kind).copied()
        }
    } else if objects == 0 {
        CONTROL.get(kind).copied()
    } else {
        DATA.get(kind).copied()
    };
    name.filter(|n| !matches!(*n, b"CTRL" | b"DATA" | b"EXT"))
}

/// One log line. The sender is the header's Port Power Role claim (bit 8);
/// receivers do not verify it (Table 6.2), so it is a label, not proof.
pub fn describe(entry: &Entry) -> [u8; 24] {
    let mut row = Row::new();
    let header = entry.header;
    row.text(if header & 0x100 != 0 {
        b"SRC "
    } else {
        b"SNK "
    });
    let objects = (header >> 12) & 7;
    if let Some(supply) = entry.supply {
        row.text(if header & 0x1f == 9 { b"EQ" } else { b"RQ" })
            .decimal(entry.word >> 28)
            .text(b" ");
        match supply {
            Supply::Fixed { mv, ma } => row
                .text(b"FIX ")
                .fixed(mv, 2)
                .text(b"V ")
                .fixed(ma, 2)
                .text(b"A"),
            Supply::Pps { mv, ma } => row
                .text(b"PPS ")
                .fixed(mv, 2)
                .text(b"V ")
                .fixed(ma, 2)
                .text(b"A"),
            Supply::Raw => row.text(b"RAW ").hex(entry.word, 8),
        };
        return row.bytes;
    }
    match name(header) {
        Some(label) => row.text(label),
        None => {
            let class: &[u8] = if header & 0x8000 != 0 {
                b"EXT"
            } else if objects == 0 {
                b"CTRL"
            } else {
                b"DATA"
            };
            row.text(class).text(b" T").hex((header & 0x1f) as u32, 2)
        }
    };
    row.text(b" ");
    if header & 0x8000 != 0 {
        // Extended header (Table 6.48): Data Size, Chunk Number, Request Chunk.
        let ext = entry.word;
        row.decimal(ext & u32::from(EXTENDED_SIZE_MASK)).text(b"B");
        let chunk = (ext >> 11) & 0xf;
        if ext & 0x400 != 0 {
            row.text(b" RQC").decimal(chunk);
        } else if ext & 0x8000 != 0 && chunk != 0 {
            row.text(b" C").decimal(chunk);
        }
    } else if objects == 0 {
        row.text(b"#").decimal(((header >> 9) & 7) as u32);
    } else {
        match header & 0x1f {
            1 | 4 => {
                row.text(b"N").decimal(objects as u32);
            }
            // VDM header (Table 6.29): SVID and command.
            15 => {
                row.hex(entry.word >> 16, 4)
                    .text(b":")
                    .hex(entry.word & 0x1f, 2);
            }
            // Alert Data Object (Table 6.25): Type of Alert byte.
            6 => {
                row.hex(entry.word >> 24, 2);
            }
            _ => {
                row.hex(entry.word, 8);
            }
        }
    }
    if row.column() <= 21 {
        let revision = (header >> 6) & 3;
        row.bytes[22] = b'R';
        row.bytes[23] = if revision == 0 {
            b'1'
        } else {
            b'1' + revision as u8
        };
    }
    row.bytes
}

/// One advertised PDO, decoded from its bit layout (Tables 6.9..6.16).
pub fn describe_pdo(index: usize, raw: u32) -> [u8; 24] {
    let mut row = Row::new();
    row.decimal(index as u32 + 1).text(b" ");
    match raw >> 28 {
        0..=3 => {
            row.text(b"FIX ")
                .fixed(((raw >> 10) & 0x3ff) * FIXED_VOLTAGE_UNIT_MV, 2)
                .text(b"V ")
                .fixed((raw & 0x3ff) * 10, 2)
                .text(b"A");
        }
        0b1100 => {
            row.text(b"PPS ")
                .fixed(((raw >> 8) & 0xff) * PPS_PDO_VOLTAGE_UNIT_MV, 1)
                .text(b"-")
                .fixed(((raw >> 17) & 0xff) * PPS_PDO_VOLTAGE_UNIT_MV, 1)
                .text(b"V ")
                .fixed((raw & 0x7f) * PPS_CURRENT_UNIT_MA, 2)
                .text(b"A");
        }
        0b1110 => {
            row.text(b"AVS ")
                .fixed(((raw >> 10) & 0x3ff) * 10, 2)
                .text(b"A/")
                .fixed((raw & 0x3ff) * 10, 2)
                .text(b"A");
        }
        0b1101 => {
            row.text(b"EAVS ")
                .fixed(((raw >> 8) & 0xff) * AVS_PDO_VOLTAGE_UNIT_MV, 1)
                .text(b"-")
                .fixed(((raw >> 17) & 0x1ff) * AVS_PDO_VOLTAGE_UNIT_MV, 1)
                .text(b"V ")
                .decimal(raw & 0xff)
                .text(b"W");
        }
        _ => {
            row.text(b"RAW ").hex(raw, 8);
        }
    }
    row.bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text(row: [u8; 24]) -> std::string::String {
        std::string::String::from_utf8(row.to_vec())
            .unwrap()
            .trim_end()
            .into()
    }

    const CAPS: [u32; 3] = [0x0801_912c, 0x0002_d12c, 0xc0dc_2164];

    #[test]
    fn requests_decode_against_the_latest_capabilities() {
        // Fixed 9 V object 2 at 2.22 A; PPS object 3 at 8.40 V/2.70 A.
        let fixed = (2u32 << 28) | (222 << 10) | 222;
        let entry = Entry::classify(0x1042, &fixed.to_le_bytes(), &CAPS);
        assert_eq!(entry.supply, Some(Supply::Fixed { mv: 9000, ma: 2220 }));
        assert_eq!(text(describe(&entry)), "SNK RQ2 FIX 9.00V 2.22A");
        let pps = (3u32 << 28) | (420 << 9) | 54;
        let entry = Entry::classify(0x1242, &pps.to_le_bytes(), &CAPS);
        assert_eq!(entry.supply, Some(Supply::Pps { mv: 8400, ma: 2700 }));
        assert_eq!(text(describe(&entry)), "SNK RQ3 PPS 8.40V 2.70A");
        // Unknown position or no caps yet: raw RDO, never a guessed unit.
        let unknown = (5u32 << 28) | 0x1234;
        let entry = Entry::classify(0x1042, &unknown.to_le_bytes(), &CAPS);
        assert_eq!(text(describe(&entry)), "SNK RQ5 RAW 50001234");
        let entry = Entry::classify(0x1042, &fixed.to_le_bytes(), &[]);
        assert_eq!(entry.supply, Some(Supply::Raw));
    }

    #[test]
    fn epr_request_uses_its_own_pdo_copy() {
        let rdo = (1u32 << 28) | (300 << 10) | 300;
        let mut data = [0; 8];
        data[..4].copy_from_slice(&rdo.to_le_bytes());
        data[4..].copy_from_slice(&0x0001_912cu32.to_le_bytes());
        let entry = Entry::classify(0x2049, &data, &[]);
        assert_eq!(text(describe(&entry)), "SNK EQ1 FIX 5.00V 3.00A");
    }

    #[test]
    fn labels_and_details() {
        let row = |header: u16, data: &[u8]| text(describe(&Entry::classify(header, data, &[])));
        assert_eq!(row(0x03a3, &[]), "SRC ACCEPT #1         R3");
        assert_eq!(row(0x0946, &[]), "SRC PS_RDY #4         R2");
        assert_eq!(&row(0x0094, &[])[..10], "SNK GET_PP");
        assert_eq!(&row(0x61a1, &[0; 24])[..14], "SRC SRC_CAP N6");
        assert_eq!(
            &row(0x1b8f, &0xff00_8001u32.to_le_bytes())[..16],
            "SRC VDM FF00:01 "
        );
        assert_eq!(
            &row(0x1186, &0x0200_0000u32.to_le_bytes())[..13],
            "SRC ALERT 02 "
        );
        // Extended PPS_Status, chunked, 4-byte data block.
        assert_eq!(
            &row(0xa18c, &[4, 0x80, 0, 0, 0, 0, 0, 0])[..17],
            "SRC PPS_STATUS 4B"
        );
        // Request-chunk for chunk 1 of a 25-byte Source_Capabilities_Extended.
        assert_eq!(row(0x9181, &[25, 0x8c, 0, 0]), "SRC SRC_CAP_EXT 25B RQC1");
        assert_eq!(row(0x9181, &[25, 0x88, 0, 0]), "SRC SRC_CAP_EXT 25B C1");
        // Reserved types keep their code.
        assert_eq!(&row(0x0099, &[])[..12], "SNK CTRL T19");
        assert_eq!(&row(0x109d, &[0; 4])[..12], "SNK DATA T1D");
        assert_eq!(&row(0x9194, &[0; 4])[..11], "SRC EXT T14");
        assert!(row(0x0081, &[]).starts_with("SNK GOODCRC"));
    }

    #[test]
    fn log_keeps_the_latest_entries_and_counts_goodcrc_separately() {
        let mut log = Log::new();
        assert_eq!(log.entries().len(), 0);
        for id in 0..9u16 {
            log.push(Entry {
                header: 0x0003 | ((id % 8) << 9),
                word: 0,
                supply: None,
            });
            log.push(Entry {
                header: 0x0001,
                word: 0,
                supply: None,
            });
        }
        assert_eq!(log.messages, 9);
        assert_eq!(log.goodcrc, 9);
        let ids: std::vec::Vec<u16> = log.entries().map(|e| (e.header >> 9) & 7).collect();
        assert_eq!(ids, [2, 3, 4, 5, 6, 7, 0]);
    }

    #[test]
    fn pdos_decode_by_type() {
        assert_eq!(text(describe_pdo(0, CAPS[0])), "1 FIX 5.00V 3.00A");
        assert_eq!(text(describe_pdo(2, CAPS[2])), "3 PPS 3.3-11.0V 5.00A");
        // SPR AVS 3.00 A (15 V) / 2.25 A (20 V); EPR AVS 15-28 V 140 W.
        assert_eq!(
            text(describe_pdo(3, 0xe000_0000 | (300 << 10) | 225)),
            "4 AVS 3.00A/2.25A"
        );
        assert_eq!(
            text(describe_pdo(
                4,
                0xd000_0000 | (280 << 17) | (150 << 8) | 140
            )),
            "5 EAVS 15.0-28.0V 140W"
        );
        assert_eq!(text(describe_pdo(5, 0x4000_0000)), "6 RAW 40000000");
    }
}
