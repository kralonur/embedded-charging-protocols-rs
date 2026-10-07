use pd_protocol::{
    pd_cable::{CableReport, CableStatus},
    pd_cable_details::{field, field_count},
    pd_cable_modes::{Status, response},
    pd_svids::Svids,
};

fn report(active: bool) -> CableReport {
    CableReport {
        status: CableStatus::Identity,
        objects: [
            0xff00a841,
            if active { 0x24001234 } else { 0x1c001234 },
            0x12345678,
            0xabcd0102,
            0x120a4643 | if active { 3 << 21 } else { 0 },
            0x503c0e0b,
            0xdeadbeef,
        ],
        count: if active { 7 } else { 5 },
        revision: 2,
        next_tx_id: 1,
        last_rx_id: Some(2),
    }
}

#[test]
fn passive_preserves_all_words_and_exposes_all_modern_fields() {
    let r = report(false);
    for index in 0..5 {
        assert_eq!(field(&r, index).unwrap().value, r.objects[index]);
    }
    let fields: Vec<_> = (0..field_count(&r))
        .map(|i| field(&r, i).unwrap())
        .collect();
    assert_eq!(
        fields
            .iter()
            .find(|f| f.name == "SVDM MINOR")
            .unwrap()
            .value,
        1
    );
    assert_eq!(
        fields
            .iter()
            .find(|f| f.name == "LATENCY CODE")
            .unwrap()
            .value,
        2
    );
    assert_eq!(
        fields.iter().find(|f| f.name == "EPR BIT").unwrap().value,
        1
    );
    assert_eq!(
        fields
            .iter()
            .find(|f| f.name == "BCD DEVICE")
            .unwrap()
            .value,
        0x102
    );
    assert_eq!(field_count(&r), 29);
}

#[test]
fn active_vdo2_and_unknown_words_are_lossless() {
    let r = report(true);
    assert_eq!(field(&r, 6).unwrap().value, 0xdeadbeef);
    let fields: Vec<_> = (0..field_count(&r))
        .map(|i| field(&r, i).unwrap())
        .collect();
    assert_eq!(
        fields
            .iter()
            .find(|f| f.name == "MAX TEMP C")
            .unwrap()
            .value,
        80
    );
    assert_eq!(
        fields
            .iter()
            .find(|f| f.name == "SHUTDOWN TEMP C")
            .unwrap()
            .value,
        60
    );
    assert_eq!(
        fields.iter().find(|f| f.name == "RETIMER").unwrap().value,
        1
    );
    assert_eq!(field_count(&r), 49);
}

#[test]
fn pd2_directionality_is_not_labelled_voltage_or_epr() {
    let mut r = report(false);
    r.revision = 1;
    r.objects[0] = 0xff008041;
    let names: Vec<_> = (0..field_count(&r))
        .map(|i| field(&r, i).unwrap().name)
        .collect();
    assert!(names.contains(&"SSTX1 RAW"));
    assert!(names.contains(&"PLUG RECEPTACLE"));
    assert!(!names.contains(&"SOP2 PRESENT"));
    assert!(!names.contains(&"EPR BIT"));
    assert!(!names.contains(&"VOLTAGE CODE"));
    r.count = 8;
    assert_eq!(field_count(&r), 0);
}

#[test]
fn extended_packets_reject_chunks_reserved_bits_and_size_mismatches() {
    fn wire(header: u16, data: &[u8]) -> Vec<u8> {
        let mut bytes = header.to_le_bytes().to_vec();
        bytes.extend(data);
        let mut crc = !0u32;
        for &byte in &bytes {
            crc ^= byte as u32;
            for _ in 0..8 {
                crc = (crc >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(crc & 1));
            }
        }
        bytes.extend((!crc).to_le_bytes());
        bytes
    }
    use pd_protocol::pd_rx::{Sop, decode_wire};
    for header in [0x8802u16, 0x8402, 0x8003, 0x801b] {
        let mut data = header.to_le_bytes().to_vec();
        data.extend([50, 1]);
        assert_eq!(
            decode_wire(Sop::Cable, &wire(0x9182, &data), true),
            Err(pd_protocol::pd_rx::Error::InvalidExtended)
        );
    }
    assert!(decode_wire(Sop::Cable, &wire(0x9182, &[2, 0, 50, 1]), true).is_ok());
    assert!(decode_wire(Sop::Cable, &wire(0x9182, &[2, 0x82, 50, 1]), true).is_ok());
    // Unchunked NDO is reserved, and odd payload sizes have no NDO padding.
    for header in [0x8187, 0xf187] {
        let bytes = wire(header, &[5, 0, 0x34, 0x12, 0x78, 0x56, 0]);
        let (frame, length) = decode_wire(Sop::Cable, &bytes, true).unwrap();
        assert_eq!(length, 13);
        assert_eq!(frame.extended_data().unwrap(), &[0x34, 0x12, 0x78, 0x56, 0]);
    }
}

#[test]
fn svids_are_high_half_first_and_modes_remain_raw() {
    let mut d = Svids::default();
    let progress = d.response(&[0xff00a842, 0x8087ff01, 0x12340000]).unwrap();
    assert_eq!(progress.status, Status::Ack);
    assert_eq!(progress.count, 3);
    assert_eq!(
        [d.at(0), d.at(1), d.at(2)],
        [Some(0x1234), Some(0x8087), Some(0xff01)]
    );
    assert_eq!(
        response(&[0x8087a843, 0xdeadbeef], 0x8087, 3),
        Some(Status::Ack)
    );
    assert_eq!(response(&[0x8087a883], 0x8087, 3), Some(Status::Nak));
    assert_eq!(response(&[0x8087a8c3], 0x8087, 3), Some(Status::Busy));
}

#[test]
fn malformed_and_truncated_lists_do_not_claim_completeness() {
    assert_eq!(response(&[0x8087a841], 0x8087, 3), None);
    assert_eq!(response(&[0x8087a883, 1], 0x8087, 3), None);
    assert_eq!(response(&[0x80872843], 0x8087, 3), None);
    let mut d = Svids::default();
    let full = [
        0xff00a842, 0x00010002, 0x00030004, 0x00050006, 0x00070008, 0x0009000a, 0x000b000c,
    ];
    assert_eq!(d.response(&full).unwrap().status, Status::Continuing);
    assert_eq!(d.count(), 12);
    assert!(d.response(&full).unwrap().repeated);
    assert_eq!(d.count(), 12);
    assert_eq!(d.response(&[0xff00a842, 0x000d0001]), None);
    assert_eq!(d.count(), 12); // rejection is atomic
    assert!(!d.contains(13));
    assert_eq!(d.response(&[0xff00a842, 0]).unwrap().status, Status::Ack);
    assert_eq!(d.response(&full), None); // terminal list cannot continue
    assert_eq!(response(&[0xff00a842], 0xff00, 2), None);
    for words in [
        vec![0xff00a842, 0x00000001],
        vec![0xff00a842, 0x80878087],
        vec![0xff00a842, 0],
        vec![0xff00a842, 0x00010002],
        vec![0xff00a842, 0, 0],
    ] {
        let mut d = Svids::default();
        assert_eq!(d.response(&words), None);
        assert_eq!(d.count(), 0);
    }
}

#[test]
fn complete_svid_domain_has_no_24_or_byte_count_limit() {
    let mut set = Svids::default();
    for start in (1..=65521u32).step_by(12) {
        let mut words = [0xff00a842; 7];
        for i in 0..6 {
            words[i + 1] = ((start + i as u32 * 2) << 16) | (start + i as u32 * 2 + 1);
        }
        assert_eq!(set.response(&words).unwrap().status, Status::Continuing);
    }
    let end = set.response(&[0xff00a842, 0xfffdfffe, 0xffff0000]).unwrap();
    assert_eq!((end.status, end.count), (Status::Ack, u16::MAX));
    assert_eq!(set.at(0), Some(1));
    assert_eq!(set.at(65534), Some(65535));
    assert_eq!(set.at(65535), None);
    assert!(!set.contains(0));
    assert!(set.contains(65535));
    assert!(core::mem::size_of::<Svids>() < 8256);
    assert!(core::mem::size_of::<pd_protocol::pd_cable_modes::Discovery>() < 192);
    set.clear();
    assert_eq!(set.count(), 0);
    assert_eq!(set.at(0), None);
    assert_eq!(set.response(&[0xff00a882]).unwrap().status, Status::Nak);
}

#[test]
fn second_controller_presence_requires_version_defined_bit() {
    use pd_protocol::pd_cable::second_controller;
    let mut r = report(true);
    assert_eq!(second_controller(&r), Some(false));
    r.objects[4] |= 8;
    assert_eq!(second_controller(&r), Some(true));
    r.objects[4] &= !(7 << 21); // deprecated PD3 layout must not borrow modern bits
    assert_eq!(second_controller(&r), None);
    r = report(false);
    r.objects[4] |= 8; // modern passive reserved bit
    assert_eq!(second_controller(&r), Some(false));
    r.objects[0] = 0xff008041;
    r.revision = 1;
    assert_eq!(second_controller(&r), Some(true));
    r.count = 8;
    assert_eq!(second_controller(&r), None);
    r = report(true);
    r.status = CableStatus::Busy;
    assert_eq!(second_controller(&r), None);
}

#[test]
fn extended_sop_double_prime_is_opt_in_and_keeps_crc_and_raw_status() {
    use pd_protocol::pd_rx;
    let mut bytes = vec![0x82, 0x93, 3, 0, 50, 1, 0xaa];
    let mut crc = !0u32;
    for &byte in &bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    bytes.extend((!crc).to_le_bytes());
    assert!(pd_rx::decode_wire(pd_rx::Sop::CableDoublePrime, &bytes, false).is_err());
    let (frame, consumed) = pd_rx::decode_wire(pd_rx::Sop::CableDoublePrime, &bytes, true).unwrap();
    assert_eq!(frame.sop(), pd_rx::Sop::CableDoublePrime);
    assert_eq!(consumed, bytes.len());
    assert_eq!(frame.extended_data().unwrap(), &[50, 1, 0xaa]);
    *bytes.last_mut().unwrap() ^= 1;
    assert!(pd_rx::decode_wire(pd_rx::Sop::CableDoublePrime, &bytes, true).is_err());
}
