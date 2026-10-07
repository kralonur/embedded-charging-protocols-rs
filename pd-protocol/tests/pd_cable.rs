use pd_protocol::pd_cable::{CableKind, Error, Layout, Response, decode_pd2, decode_pd3};

const ACK: u32 = 0xff00_a841;
fn passive() -> [u32; 5] {
    [
        ACK,
        (3 << 27) | 0x1234,
        0x11223344,
        0x56780102,
        (2 << 18) | (1 << 17) | (3 << 9) | (2 << 5) | 3,
    ]
}

#[test]
fn passive_identity_preserves_raw_and_decodes_claims() {
    let objects = passive();
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.raw, objects);
    assert_eq!(id.kind, CableKind::Passive);
    assert_eq!(id.layout, Layout::PassiveV10);
    assert_eq!((id.vid, id.pid, id.bcd_device), (0x1234, 0x5678, 0x0102));
    assert_eq!(id.certification, 0x11223344);
    assert_eq!(id.current_ma, Some(5000));
    assert!(!id.current_assumed);
    assert_eq!(id.maximum_voltage_mv, Some(50000));
    assert_eq!(id.speed_code, 3);
    assert_eq!(id.epr_capable, Some(true));
}

#[test]
fn active_v13_requires_second_vdo_and_decodes_epr_bit() {
    let mut objects = passive().to_vec();
    objects[1] = 4 << 27;
    objects[4] |= (3 << 21) | (1 << 4);
    assert_eq!(decode_pd3(&objects), Err(Error::Length));
    objects.push(0xdeadbeef);
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.layout, Layout::ActiveV13);
    assert_eq!(id.raw[5], 0xdeadbeef);
    assert_eq!(id.epr_capable, Some(true));
    assert_eq!(id.current_ma, Some(5000));
    objects[4] &= !(1 << 4);
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.current_ma, None);
}

#[test]
fn deprecated_active_versions_are_not_decoded_as_v13() {
    for version in [0, 1, 2, 4, 7] {
        let mut objects = passive();
        objects[1] = 4 << 27;
        objects[4] |= version << 21;
        let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
            panic!()
        };
        assert_eq!(id.layout, Layout::Unknown);
        assert_eq!(
            (id.current_ma, id.maximum_voltage_mv, id.epr_capable),
            (None, None, None)
        );
        assert_eq!(id.raw, objects);
    }
}

#[test]
fn nak_busy_and_request_are_distinct() {
    assert_eq!(decode_pd3(&[0xff00_a881]), Ok(Response::Nak));
    assert_eq!(decode_pd3(&[0xff00_a8c1]), Ok(Response::Busy));
    assert_eq!(decode_pd3(&[0xff00_a801]), Err(Error::Header));
    assert_eq!(decode_pd3(&[0xff00_a881, 0]), Err(Error::Length));
}

#[test]
fn malformed_headers_lengths_and_non_cables_are_rejected() {
    let objects = passive();
    for len in 0..5 {
        assert!(decode_pd3(&objects[..len]).is_err());
    }
    assert_eq!(decode_pd3(&[ACK; 8]), Err(Error::Length));
    for mask in [1 << 16, 1 << 15, 1] {
        let mut bad = objects;
        bad[0] ^= mask;
        assert_eq!(decode_pd3(&bad), Err(Error::Header));
    }
    let mut bad = objects;
    bad[1] = 2 << 27;
    assert_eq!(decode_pd3(&bad), Err(Error::NotCable));
    bad = objects;
    bad[0] = 0xff00_b041;
    assert_eq!(decode_pd3(&bad), Err(Error::UnsupportedVersion));
}

#[test]
fn receiver_defaults_and_ignored_fields_follow_v12() {
    for major in [1, 2, 3] {
        let mut objects = passive();
        objects[0] = 0xff008041 | (major << 13) | (1 << 11) | 0x720;
        let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
            panic!()
        };
        assert_eq!(id.layout, Layout::PassiveV10);
        assert_eq!(id.raw[0], objects[0]);
    }
}

#[test]
fn passive_reserved_current_uses_explicit_receiver_default() {
    for current in 0..4 {
        let mut objects = passive();
        objects[4] = current << 5;
        let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
            panic!()
        };
        assert_eq!(id.current_ma, Some(if current == 2 { 5000 } else { 3000 }));
        assert_eq!(id.current_assumed, matches!(current, 0 | 3));
    }
    let mut objects = passive();
    objects[4] |= 7 << 21;
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.current_ma, None);
}

#[test]
fn deprecated_voltage_codes_are_20v_and_do_not_imply_epr() {
    for code in 0..4 {
        let mut objects = passive();
        objects[4] = (objects[4] & !((3 << 9) | (1 << 17))) | (code << 9);
        let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
            panic!()
        };
        assert_eq!(
            id.maximum_voltage_mv,
            Some(if code == 3 { 50000 } else { 20000 })
        );
        assert_eq!(id.epr_capable, Some(false));
    }
}

#[test]
fn pd2_directionality_never_becomes_voltage_or_epr() {
    let mut objects = passive();
    objects[0] = 0xff008041;
    objects[4] = (2 << 18) | (0xf << 7) | (2 << 5) | 2;
    let Response::Identity(id) = decode_pd2(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.layout, Layout::Pd2);
    assert_eq!(id.current_ma, Some(5000));
    assert_eq!((id.maximum_voltage_mv, id.epr_capable), (None, None));
    // SVDM1 in PD3 has its own historical schema, not modern PD3 or PD2.
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.layout, Layout::Unknown);
    assert_eq!(
        (id.current_ma, id.maximum_voltage_mv, id.epr_capable),
        (None, None, None)
    );
    objects[0] = ACK;
    assert_eq!(decode_pd2(&objects), Err(Error::UnsupportedVersion));
}

#[test]
fn passive_unknown_version_is_not_confused_with_active_v13() {
    let mut objects = passive();
    objects[4] |= 3 << 21;
    let Response::Identity(id) = decode_pd3(&objects).unwrap() else {
        panic!()
    };
    assert_eq!(id.layout, Layout::Unknown);
    assert_eq!(
        (id.current_ma, id.maximum_voltage_mv, id.epr_capable),
        (None, None, None)
    );
}
