//! A scripted transport and independent caller policy.
#[path = "support/timing.rs"]
mod timing;
use pd_protocol::{
    pd_cable_modes::{Discovery, Status, Sync},
    pd_discovery::{self, Fault, Phase, Transport},
    pd_rx::{self, Frame, Sop},
    pd_spr::{self, Limits, Target},
    pd_svids::{Progress, Store, Svids},
};
use std::{
    cell::RefCell,
    collections::VecDeque,
    future::{Future, pending},
    task::{Context, Poll, Waker},
};
use usbpd::protocol_layer::message::{
    Message, Payload,
    data::{Data, request, source_capabilities},
};
use usbpd_traits::{CableReport, CableStatus, Driver, DriverRxError, DriverTxError};

fn poll<F: Future>(future: std::pin::Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}
fn complete<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    match poll(future.as_mut()) {
        Poll::Ready(result) => result,
        Poll::Pending => panic!("unexpected pending"),
    }
}
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
fn objects(header: u16, data: &[u32]) -> Vec<u8> {
    let mut bytes = Vec::new();
    for raw in data {
        bytes.extend(raw.to_le_bytes());
    }
    wire(header, &bytes)
}
struct Raw {
    sop: Sop,
    bytes: Vec<u8>,
}
impl pd_discovery::Packet for Raw {
    fn frame(&self) -> Frame<'_> {
        pd_rx::decode_wire(self.sop, &self.bytes, true).unwrap().0
    }
}
fn raw(sop: Sop, header: u16, data: &[u32]) -> Option<Raw> {
    Some(Raw {
        sop,
        bytes: objects(header, data),
    })
}
const PASSIVE: [u32; 5] = [0xff00a841, 0x1c001234, 0, 0x56780100, 0x000a4643];
const ACTIVE: [u32; 6] = [
    0xff00a841, 0x20001234, 0, 0x56780100, 0x126a464b, 0x503c0e0b,
];
#[derive(Debug, PartialEq, Eq)]
enum Error {
    Peer,
    Request,
    Io,
    NoGoodCrc,
}
struct Set(RefCell<Svids>);
impl Store for Set {
    fn clear(&self) {
        self.0.borrow_mut().clear();
    }
    fn response(&self, objects: &[u32]) -> Option<Progress> {
        self.0.borrow_mut().response(objects)
    }
    fn contains(&self, svid: u16) -> bool {
        self.0.borrow().contains(svid)
    }
    fn at(&self, index: u16) -> Option<u16> {
        self.0.borrow().at(index)
    }
}
struct Link {
    rx: VecDeque<Option<Raw>>,
    tx: Vec<(Sop, Vec<u8>)>,
    result: Discovery,
    set: Set,
    report: Option<CableReport>,
    id: u8,
    phase: Phase,
    extended: bool,
    second: bool,
    unsafe_power: bool,
    checks: usize,
    selected_second: usize,
    fail_send: bool,
    pause_send: bool,
    no_goodcrc: Vec<usize>,
}
impl Link {
    fn new(rx: Vec<Option<Raw>>) -> Self {
        Self {
            rx: rx.into(),
            tx: vec![],
            result: Discovery::default(),
            set: Set(RefCell::new(Svids::default())),
            report: None,
            id: 0,
            phase: Phase::Identity,
            extended: false,
            second: true,
            unsafe_power: false,
            checks: 0,
            selected_second: 0,
            fail_send: false,
            pause_send: false,
            no_goodcrc: vec![],
        }
    }
}
impl Transport for Link {
    // Synthetic regression deadlines in milliseconds, supplied by the scripted peer.
    const RESPONSE_TIMEOUT_MS: u64 = 51;
    const SVID_NO_PROGRESS_TIMEOUT_MS: u64 = 1_000;
    type Error = Error;
    type Packet = Raw;
    fn fault(f: Fault) -> Error {
        match f {
            Fault::Peer => Error::Peer,
            Fault::Request => Error::Request,
        }
    }
    fn is_tx_timeout(error: &Error) -> bool {
        *error == Error::NoGoodCrc
    }
    fn now_ms(&self) -> u64 {
        0
    }
    fn supports_second(&self) -> bool {
        self.second
    }
    async fn send(&mut self, bytes: &[u8], sop: Sop) -> Result<(), Error> {
        self.tx.push((sop, bytes.to_vec()));
        if self.pause_send {
            pending::<()>().await;
        }
        if self.fail_send {
            return Err(Error::Io);
        }
        if self.no_goodcrc.contains(&(self.tx.len() - 1)) {
            return Err(Error::NoGoodCrc);
        }
        Ok(())
    }
    async fn wait(&mut self, timeout_ms: u64) -> Result<Option<Raw>, Error> {
        assert_eq!(timeout_ms, 51);
        Ok(self.rx.pop_front().expect("unexpected receive/retry"))
    }
    fn discovery(&mut self) -> &mut Discovery {
        &mut self.result
    }
    fn store(&self) -> Option<&dyn Store> {
        Some(&self.set)
    }
    fn receive_id(&self) -> u8 {
        self.id
    }
    fn set_receive_id(&mut self, id: u8) {
        self.id = id;
    }
    fn phase(&self) -> Phase {
        self.phase
    }
    fn set_phase(&mut self, phase: Phase) {
        self.phase = phase;
    }
    fn allow_extended(&mut self, enabled: bool) {
        self.extended = enabled;
    }
    fn set_report(&mut self, report: Option<CableReport>) {
        self.report = report;
    }
    async fn recheck_power(&mut self) -> Result<(), Error> {
        self.checks += 1;
        if self.unsafe_power {
            Err(Error::Io)
        } else {
            Ok(())
        }
    }
    async fn select_second(&mut self) -> Result<(), Error> {
        assert!(self.checks > 0);
        self.selected_second += 1;
        Ok(())
    }
}
/// SOP' Accept to our Soft_Reset: Cable Plug, MessageID 0, PD3.
fn accept() -> Option<Raw> {
    raw(Sop::Cable, 0x0183, &[])
}
// After its Accept (MessageID 0) the plug numbers its replies from 1.
fn passive() -> Link {
    Link::new(vec![
        accept(),
        raw(Sop::Cable, 0x538f, &PASSIVE),
        raw(Sop::Cable, 0x258f, &[0xff00a842, 0x80870000]),
        raw(Sop::Cable, 0x0790, &[]),
    ])
}
fn active() -> Link {
    Link::new(vec![
        accept(),
        raw(Sop::Cable, 0x638f, &ACTIVE),
        raw(Sop::Cable, 0x0590, &[]),
        Some(Raw {
            sop: Sop::Cable,
            bytes: wire(0x8782, &[2, 0, 50, 1]),
        }),
        raw(Sop::CableDoublePrime, 0x0183, &[]),
        Some(Raw {
            sop: Sop::CableDoublePrime,
            bytes: wire(0x8382, &[2, 0, 55, 2]),
        }),
    ])
}

#[test]
fn neutral_wire_decoder_preserves_address_and_crc() {
    let bytes = objects(0x518f, &PASSIVE);
    let (frame, n) = pd_rx::decode_wire(Sop::Cable, &bytes, false).unwrap();
    assert_eq!(n, bytes.len());
    assert_eq!(frame.sop(), Sop::Cable);
    let stripped = &bytes[..bytes.len() - 4];
    let (verified, used) = pd_rx::decode_verified_message(Sop::Cable, stripped, false).unwrap();
    assert_eq!(used, stripped.len());
    assert_eq!(verified.data(), frame.data());
    assert_eq!(
        pd_rx::decode_verified_message(Sop::Cable, &stripped[..3], false),
        Err(pd_rx::Error::Truncated)
    );
    let mut bad = bytes.clone();
    *bad.last_mut().unwrap() ^= 1;
    assert_eq!(
        pd_rx::decode_wire(Sop::Cable, &bad, false),
        Err(pd_rx::Error::Crc)
    );
    assert_eq!(
        pd_rx::decode_wire(Sop::Cable, &bytes[..3], false),
        Err(pd_rx::Error::Truncated)
    );
    assert_eq!(
        pd_rx::decode_wire(Sop::Partner, &wire(0x8382, &[2, 0, 55, 2]), true),
        Err(pd_rx::Error::UnsupportedExtended)
    );
}
#[test]
fn transport_discovers_complete_set_and_preserves_partner_ids() {
    let mut link = passive();
    pd_discovery::begin(&mut link, None, None).unwrap();
    let report = complete(pd_discovery::inspect(&mut link, 6, Some(5), None, None)).unwrap();
    assert_eq!((report.next_tx_id, report.last_rx_id), (6, Some(5)));
    assert_eq!(link.result.status, Status::Ack);
    assert!(link.set.contains(0x8087));
    assert_eq!(link.result.count, 1);
    assert_eq!(link.tx.len(), 4);
    assert_eq!(link.result.sync, Sync::Accepted);
    assert_eq!(link.tx[0], (Sop::Cable, vec![0x8d, 0]));
    assert_eq!(
        link.tx[1],
        (Sop::Cable, vec![0x8f, 0x12, 0x01, 0xa8, 0x00, 0xff])
    );
    assert_eq!(&link.tx[2].1[..2], &[0x8f, 0x14]);
    assert_eq!(&link.tx[3].1[..2], &[0x86, 0x96]);
    assert!(link.rx.is_empty());
    assert_eq!(link.selected_second, 0);
}
#[test]
fn transport_reads_only_explicit_selected_mode() {
    let mut link = passive();
    let report = complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap();
    link.rx.extend([
        accept(),
        raw(Sop::Cable, 0x538f, &PASSIVE),
        raw(Sop::Cable, 0x258f, &[0x8087a843, 0xdeadbeef]),
    ]);
    pd_discovery::begin(&mut link, Some(0x8087), Some(report)).unwrap();
    complete(pd_discovery::inspect(
        &mut link,
        4,
        Some(3),
        Some(0x8087),
        Some(report),
    ))
    .unwrap();
    assert_eq!(link.result.mode.svid, 0x8087);
    assert_eq!(link.result.mode.status, Status::Ack);
    assert_eq!(link.result.mode.objects[1], 0xdeadbeef);
    assert_eq!(link.tx.len(), 7);
    assert_eq!(link.tx[4].1, [0x8d, 0]);
    assert_eq!(&link.tx[6].1[..2], &[0x8f, 0x14]);
    assert!(link.rx.is_empty());
}
#[test]
fn changed_identity_clears_storage_and_halts_without_other_queries() {
    let mut link = passive();
    let report = complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap();
    let mut changed = PASSIVE;
    changed[3] ^= 1;
    link.rx
        .extend([accept(), raw(Sop::Cable, 0x538f, &changed)]);
    assert_eq!(
        complete(pd_discovery::inspect(
            &mut link,
            0,
            None,
            Some(0x8087),
            Some(report)
        ))
        .unwrap_err(),
        Error::Peer
    );
    assert!(!link.set.contains(0x8087));
    assert!(link.report.is_none());
    assert_eq!(link.tx.len(), 6);
}
#[test]
fn second_status_obeys_scoped_reset_status_only() {
    let mut link = active();
    complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap();
    assert_eq!(link.result.second_sync, Status::Ack);
    assert_eq!(link.result.second_status.status, Status::Ack);
    assert_eq!(&link.result.second_status.data[..2], &[55, 2]);
    let second: Vec<_> = link
        .tx
        .iter()
        .filter(|(sop, _)| *sop == Sop::CableDoublePrime)
        .collect();
    assert_eq!(second.len(), 2);
    assert_eq!(second[0].1, [0x8d, 0]);
    assert_eq!(second[1].1, [0x92, 2]);
    assert_eq!(link.checks, 1);
    assert_eq!(link.selected_second, 1);
    assert!(!link.extended);
}
#[test]
fn unavailable_second_transport_does_not_attempt_sync() {
    let mut link = active();
    link.second = false;
    complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap();
    assert_eq!(link.result.second_status.status, Status::NotSupported);
    assert_eq!(link.tx.len(), 4);
    assert_eq!(link.checks, 0);
    assert_eq!(link.selected_second, 0);
}
#[test]
fn failed_fresh_power_gate_halts_before_second_selection_or_reset() {
    let mut link = active();
    link.unsafe_power = true;
    assert_eq!(
        complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap_err(),
        Error::Io
    );
    assert_eq!(link.tx.len(), 4);
    assert_eq!(link.selected_second, 0);
    assert!(link.phase == Phase::Identity);
}
#[test]
fn unadvertised_mode_transport_error_and_cancellation_never_retry() {
    let mut link = passive();
    assert_eq!(
        pd_discovery::begin(&mut link, Some(0xffff), None),
        Err(Error::Request)
    );
    assert!(link.tx.is_empty());
    link.fail_send = true;
    assert_eq!(
        complete(pd_discovery::inspect(&mut link, 0, None, None, None)).unwrap_err(),
        Error::Io
    );
    assert_eq!(link.tx.len(), 1);
    let mut link = passive();
    link.pause_send = true;
    {
        let mut future = std::pin::pin!(pd_discovery::inspect(&mut link, 0, None, None, None));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(link.tx.len(), 1);
    assert_eq!(link.selected_second, 0);
}

/// The SOP' Soft_Reset outcome decides whether identity is asked at all, and
/// at which revision and MessageID (sections 6.1.3.2, 7.32.3).
#[test]
fn cable_sync_reports_why_and_asks_identity_only_after_accept() {
    let run = |rx: Vec<Option<Raw>>, no_goodcrc: &[usize]| {
        let mut link = Link::new(rx);
        link.no_goodcrc = no_goodcrc.to_vec();
        let report = complete(pd_discovery::inspect(&mut link, 3, Some(2), None, None));
        (report, link)
    };
    // Silent plug: one Soft_Reset, no identity request.
    let (report, link) = run(vec![], &[0]);
    assert_eq!(report.unwrap().status, CableStatus::Timeout);
    assert_eq!((link.result.sync, link.tx.len()), (Sync::NoGoodCrc, 1));
    assert!(link.phase == Phase::Identity && link.report.is_some());
    // GoodCRC but no Accept: still no identity request.
    let (report, link) = run(vec![None], &[]);
    assert_eq!(report.unwrap().status, CableStatus::Timeout);
    assert_eq!((link.result.sync, link.tx.len()), (Sync::NoAccept, 1));
    // PD2 Accept: only a PD2 request, MessageID 1, SVDM 1.0.
    let (report, link) = run(vec![raw(Sop::Cable, 0x0143, &[]), None], &[]);
    assert_eq!(report.unwrap().status, CableStatus::Timeout);
    assert_eq!(link.result.sync, Sync::Accepted);
    assert_eq!(link.tx[1].1, [0x4f, 0x12, 0x01, 0x80, 0x00, 0xff]);
    assert_eq!(link.tx.len(), 2);
    // Unanswered PD3 request: one PD2 request with the next MessageID.
    let (report, link) = run(vec![accept(), None, None], &[]);
    assert_eq!(report.unwrap().status, CableStatus::Timeout);
    assert_eq!(link.tx[2].1, [0x4f, 0x14, 0x01, 0x80, 0x00, 0xff]);
    assert_eq!(link.tx.len(), 3);
    // Unacknowledged PD3 request also counts its MessageID.
    let (_, link) = run(vec![accept(), None], &[1]);
    assert_eq!(link.tx.len(), 3);
    // A PD2 answer after the fallback continues from its MessageID.
    let (report, link) = run(
        vec![
            accept(),
            None,
            raw(
                Sop::Cable,
                0x534f,
                &[0xff008041, 0x18001234, 0, 0x56780100, 0x00000642],
            ),
        ],
        &[],
    );
    let report = report.unwrap();
    assert_eq!((report.status, report.revision), (CableStatus::Identity, 1));
    assert_eq!((link.result.next_id, link.id), (3, 2));
    assert_eq!((report.next_tx_id, report.last_rx_id), (3, Some(2)));
    // Anything but a Cable Plug's MessageID-0 Accept on SOP' halts.
    for bad in [
        raw(Sop::Cable, 0x0383, &[]),
        raw(Sop::Cable, 0x0083, &[]),
        raw(Sop::Cable, 0x1183, &[0]),
        raw(Sop::Cable, 0x0184, &[]),
        raw(Sop::CableDoublePrime, 0x0183, &[]),
    ] {
        let (report, link) = run(vec![bad], &[]);
        assert_eq!(report.unwrap_err(), Error::Peer);
        assert_eq!(link.tx.len(), 1);
    }
}

struct Portable;
impl Limits for Portable {
    const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
    const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
    const MAX_REQUEST_MV: u32 = 20_000;
    const MIN_PPS_MV: u32 = 3_300;
    const MAX_PPS_MV: u32 = 11_000;
    const TARGET_MV: u32 = 6_000;
    const REQUEST_MA: u32 = 500;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 0b10;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000, 12_000];
    const PPS_REFRESH_MS: u64 = 3_000;
}
fn fixed(mv: u32) -> u32 {
    ((mv / 50) << 10) | 300
}
fn caps() -> source_capabilities::SourceCapabilities {
    let bytes = objects(
        0x4181,
        &[
            fixed(5000),
            fixed(6000),
            fixed(12000),
            0xc0000000 | (110 << 17) | (33 << 8) | 60,
        ],
    );
    match Message::from_bytes(&bytes[..bytes.len() - 4])
        .unwrap()
        .payload
        .unwrap()
    {
        Payload::Data(Data::SourceCapabilities(caps)) => caps,
        _ => panic!("fixture"),
    }
}
#[test]
fn caller_policy_controls_fixed_targets_pps_range_and_current() {
    let caps = caps();
    let request::PowerSource::FixedVariableSupply(rdo) =
        pd_spr::request_target::<Portable>(&caps).unwrap()
    else {
        panic!()
    };
    // Table 6.19: the object's Maximum Current (3 A), not REQUEST_MA.
    assert_eq!(rdo.object_position(), 2);
    assert_eq!(rdo.raw_operating_current(), 300);
    assert_eq!(rdo.raw_max_operating_current(), 300);
    assert!(!rdo.capability_mismatch());
    assert!(pd_spr::request_pps::<Portable>(&caps, 4, 3300, 1000).is_ok());
    assert!(pd_spr::request_pps::<Portable>(&caps, 4, 11020, 1000).is_err());
}

struct Capped;
impl Limits for Capped {
    const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
    const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
    const MAX_REQUEST_MV: u32 = 20_000;
    const MIN_PPS_MV: u32 = 3_300;
    const MAX_PPS_MV: u32 = 11_000;
    const TARGET_MV: u32 = 6_000;
    const REQUEST_MA: u32 = 500;
    const MAX_CURRENT_MA: u32 = 2_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 0b10;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000, 12_000];
    const PPS_REFRESH_MS: u64 = 3_000;
}
struct Independent;
impl Limits for Independent {
    const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
    // Synthetic caller identity, deliberately distinct from the anonymous fixture.
    const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity {
        vid: 0x1234,
        pid: 0x5678,
        xid: 0x90ab_cdef,
        fw_version: 2,
        hw_version: 3,
    };
    const MAX_REQUEST_MV: u32 = 20_000;
    const MIN_PPS_MV: u32 = 3_300;
    const MAX_PPS_MV: u32 = 11_000;
    const TARGET_MV: u32 = 6_000;
    const REQUEST_MA: u32 = 500;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = true;
    const NO_USB_SUSPEND: bool = false;
    const HIGHER_CAPABILITY: bool = false;
    const UNCONSTRAINED_POWER: bool = true;
    const SINK_POWER_MODES: u8 = 0b1100; // AC supply and Battery
    const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000, 12_000];
    const PPS_REFRESH_MS: u64 = 3_000;
}
#[test]
fn advertised_facts_come_only_from_the_caller_limits() {
    // RDO bits 25/24 (Tables 6.19, 6.21), Sink PDO bits 28/27/26 (Table 6.9) and
    // SKEDB Sink Modes bits 1..4 (Table 6.61) are caller facts, never library defaults.
    let request::PowerSource::FixedVariableSupply(rdo) =
        pd_spr::request_target::<Independent>(&caps()).unwrap()
    else {
        panic!()
    };
    assert!(rdo.usb_communications_capable() && !rdo.no_usb_suspend());
    let request::PowerSource::Pps(pps) =
        pd_spr::request_pps::<Independent>(&caps(), 4, 3300, 1000).unwrap()
    else {
        panic!()
    };
    assert_eq!(pps.0 >> 24 & 3, 0b10);
    let wire = objects(
        0x4181,
        &[
            fixed(5000),
            fixed(6000),
            fixed(12000),
            0xc0000000 | (110 << 17) | (33 << 8) | 60,
        ],
    );
    let advertisements = pd_rx::decode_wire(Sop::Partner, &wire, false)
        .unwrap()
        .0
        .advertisements()
        .unwrap();
    assert!(pd_spr::review_rdo::<Independent>(
        &advertisements,
        rdo.0,
        None
    ));
    assert!(pd_spr::review_rdo::<Independent>(
        &advertisements,
        pps.0,
        Some(Target {
            mv: 3300,
            ma: 1000,
            pps_object: 4
        })
    ));
    // Another caller's flags are refused by this review gate.
    for flipped in [rdo.0 ^ 1 << 25, rdo.0 ^ 1 << 24] {
        assert!(
            !pd_spr::review_rdo::<Independent>(&advertisements, flipped, None),
            "{flipped:08x}"
        );
    }
    let (objects, _) = pd_spr::sink_capabilities::<Independent>(false).unwrap();
    assert_eq!(
        objects[0] >> 26 & 7,
        0b011,
        "no Higher Capability; Unconstrained; USB Communications"
    );
    let extended = pd_spr::sink_capabilities_extended::<Independent>().unwrap();
    // Table 6.61 little-endian encoding of synthetic VID, PID, XID,
    // version fields; independent of the library serializer.
    const CALLER_IDENTITY_WIRE: [u8; 10] = [0x34, 0x12, 0x78, 0x56, 0xef, 0xcd, 0xab, 0x90, 2, 3];
    // Prevent the anonymous port identity from leaking into another caller's SKEDB.
    assert_eq!(&extended[..10], &CALLER_IDENTITY_WIRE);
    assert_eq!(extended[17], 0b1101);
    let (objects, _) = pd_spr::sink_capabilities::<Portable>(false).unwrap();
    assert_eq!(objects[0] >> 26 & 7, 0b100);
    assert_eq!(
        pd_spr::sink_capabilities_extended::<Portable>().unwrap()[17],
        0b0011
    );
    // Undefined Sink Modes bits (0 and 5..7 belong to the library) are an invalid policy.
    struct Undefined;
    impl Limits for Undefined {
        const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
        const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
        const MAX_REQUEST_MV: u32 = 20_000;
        const MIN_PPS_MV: u32 = 3_300;
        const MAX_PPS_MV: u32 = 11_000;
        const TARGET_MV: u32 = 6_000;
        const REQUEST_MA: u32 = 500;
        const MAX_CURRENT_MA: u32 = 5_000;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = true;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b10_0010;
        const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000];
        const PPS_REFRESH_MS: u64 = 3_000;
    }
    assert!(pd_spr::sink_capabilities_extended::<Undefined>().is_none());
    assert!(pd_spr::request_target::<Undefined>(&caps()).is_err());
}

#[test]
fn fixed_request_current_is_the_object_maximum_capped_by_the_caller_maximum() {
    assert_eq!(pd_spr::fixed_request_ma::<Portable>(3_000), 3_000);
    assert_eq!(pd_spr::fixed_request_ma::<Capped>(3_000), 2_000);
    assert_eq!(pd_spr::fixed_request_ma::<Capped>(1_550), 1_550);
    let request::PowerSource::FixedVariableSupply(rdo) =
        pd_spr::request_target::<Capped>(&caps()).unwrap()
    else {
        panic!()
    };
    assert_eq!(
        (rdo.raw_operating_current(), rdo.raw_max_operating_current()),
        (200, 200)
    );
    assert!(
        !rdo.capability_mismatch(),
        "a lower request is not a Capability Mismatch"
    );
    // The review gate recomputes the same current; anything else is refused.
    let wire = objects(
        0x4181,
        &[
            fixed(5000),
            fixed(6000),
            fixed(12000),
            0xc0000000 | (110 << 17) | (33 << 8) | 60,
        ],
    );
    let advertisements = pd_rx::decode_wire(Sop::Partner, &wire, false)
        .unwrap()
        .0
        .advertisements()
        .unwrap();
    assert!(pd_spr::review_rdo::<Capped>(&advertisements, rdo.0, None));
    for other in [
        rdo.0 + 1,
        rdo.0 + (1 << 10),
        rdo.with_raw_operating_current(300)
            .with_raw_max_operating_current(300)
            .0,
    ] {
        assert!(
            !pd_spr::review_rdo::<Capped>(&advertisements, other, None),
            "{other:08x}"
        );
    }
    // Sink_Capabilities advertise the caller's current limit below the cable ceiling.
    let (objects, _) = pd_spr::sink_capabilities::<Capped>(false).unwrap();
    assert_eq!(objects[0] & 0x3ff, 200);
    // A rating below REQUEST_MA, or not in 10 mA units, is an invalid policy.
    struct Below;
    impl Limits for Below {
        const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
        const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
        const MAX_REQUEST_MV: u32 = 20_000;
        const MIN_PPS_MV: u32 = 3_300;
        const MAX_PPS_MV: u32 = 11_000;
        const TARGET_MV: u32 = 6_000;
        const REQUEST_MA: u32 = 500;
        const MAX_CURRENT_MA: u32 = 450;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = true;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b10;
        const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000];
        const PPS_REFRESH_MS: u64 = 3_000;
    }
    struct Uneven;
    impl Limits for Uneven {
        const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
        const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
        const MAX_REQUEST_MV: u32 = 20_000;
        const MIN_PPS_MV: u32 = 3_300;
        const MAX_PPS_MV: u32 = 11_000;
        const TARGET_MV: u32 = 6_000;
        const REQUEST_MA: u32 = 500;
        const MAX_CURRENT_MA: u32 = 2_005;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = true;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b10;
        const FIXED_TARGETS: &'static [u16] = &[5_000, 6_000];
        const PPS_REFRESH_MS: u64 = 3_000;
    }
    assert!(pd_spr::request_target::<Below>(&caps()).is_err());
    assert!(pd_spr::sink_capabilities::<Below>(false).is_none());
    assert!(pd_spr::request_target::<Uneven>(&caps()).is_err());
    assert!(pd_spr::sink_capabilities::<Uneven>(true).is_none());
}
#[test]
fn portable_raw_rdo_review_matches_caller_target_and_limits() {
    let wire = objects(
        0x4181,
        &[
            fixed(5000),
            fixed(6000),
            fixed(12000),
            0xc0000000 | (110 << 17) | (33 << 8) | 60,
        ],
    );
    let advertisements = pd_rx::decode_wire(Sop::Partner, &wire, false)
        .unwrap()
        .0
        .advertisements()
        .unwrap();
    let request::PowerSource::FixedVariableSupply(rdo) =
        pd_spr::request_target::<Portable>(&caps()).unwrap()
    else {
        panic!()
    };
    assert!(pd_spr::review_rdo::<Portable>(&advertisements, rdo.0, None));
    assert!(!pd_spr::review_rdo::<Portable>(
        &advertisements,
        rdo.0 | (1 << 27),
        None
    ));
    let request::PowerSource::Pps(rdo) =
        pd_spr::request_pps::<Portable>(&caps(), 4, 3300, 1000).unwrap()
    else {
        panic!()
    };
    let target = Target {
        mv: 3300,
        ma: 1000,
        pps_object: 4,
    };
    assert!(pd_spr::review_rdo::<Portable>(
        &advertisements,
        rdo.0,
        Some(target)
    ));
    assert!(!pd_spr::review_rdo::<Portable>(
        &advertisements,
        rdo.0,
        Some(Target { mv: 3320, ..target })
    ));
    let mut malformed = advertisements;
    malformed.count = 8;
    assert!(!pd_spr::review_rdo::<Portable>(
        &malformed,
        rdo.0,
        Some(target)
    ));
}

struct Bad;
impl Limits for Bad {
    const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
    const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
    const MAX_REQUEST_MV: u32 = 100_000;
    const MIN_PPS_MV: u32 = 3_300;
    const MAX_PPS_MV: u32 = 11_000;
    const TARGET_MV: u32 = 100_000;
    const REQUEST_MA: u32 = 125;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 0b10;
    const FIXED_TARGETS: &'static [u16] = &[5000];
    const PPS_REFRESH_MS: u64 = 10_001;
}
#[test]
fn malformed_policy_fails_closed_before_rdo_construction() {
    assert!(pd_spr::request_target::<Bad>(&caps()).is_err());
    assert!(pd_spr::request_pps::<Bad>(&caps(), 4, 3300, 1000).is_err());
}
struct Never;
impl pd_spr::Timer for Never {
    async fn after_millis(_: u64) {
        pending::<()>().await;
    }
}
struct DriverModel {
    rx: VecDeque<Vec<u8>>,
    tx: Vec<Vec<u8>>,
}
impl Driver for DriverModel {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        panic!("reset")
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        self.tx.push(bytes.to_vec());
        Ok(())
    }
    async fn receive(&mut self, bytes: &mut [u8]) -> Result<usize, DriverRxError> {
        let Some(packet) = self.rx.pop_front() else {
            return pending().await;
        };
        bytes[..packet.len()].copy_from_slice(&packet);
        Ok(packet.len())
    }
}
#[test]
fn semantic_previews_and_confirmations_work_without_buttons_or_display() {
    let mut advertisements = objects(
        0x4181,
        &[
            fixed(5000),
            fixed(6000),
            fixed(12000),
            0xc0000000 | (110 << 17) | (33 << 8) | 60,
        ],
    );
    advertisements.truncate(advertisements.len() - 4);
    let mut driver = DriverModel {
        rx: [advertisements, vec![0x83, 3], vec![0x86, 5]].into(),
        tx: vec![],
    };
    let selection = pd_spr::Selection::<Portable>::new();
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = pd_spr::Trial::<Portable>::maintained();
    let mut future = Box::pin(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    assert!(poll(future.as_mut()).is_pending());
    assert_eq!(
        trace.borrow().contract.as_ref().unwrap().request.0 & 0x3ff,
        300
    ); // object maximum
    assert_eq!(selection.option(1).unwrap().mv, 6000);
    assert!(!selection.preview(Target {
        mv: 9000,
        ma: 500,
        pps_object: 0
    }));
    assert!(selection.preview(Target {
        mv: 3300,
        ma: 1000,
        pps_object: 4
    }));
    assert_eq!(selection.target_mv(), 5000);
    assert!(!selection.has_pending());
    assert!(selection.confirm());
    assert!(selection.has_pending());
    selection.set_rx_idle(true);
    assert!(poll(future.as_mut()).is_pending());
    assert_eq!(selection.target_mv(), 3300);
    selection.note_request_sent(200);
    selection.set_now_ms(3199);
    assert!(!selection.refresh_due());
    selection.set_now_ms(3200);
    assert!(selection.request_in_flight());
    assert!(
        !selection.refresh_due(),
        "no second request before Accept/PS_RDY"
    );
    assert!(!selection.confirm());
}

#[test]
fn pps_apdo_reserved_bits_are_ignored_but_rdo_reserved_bits_are_not() {
    // Table 6.13: APDO bits 26..25, 16 and 7 are Reserved; the receiver Shall ignore them.
    let pps = 0xc0000000 | (110 << 17) | (33 << 8) | 60;
    let reserved = pps | 0x0601_0080;
    let wire = objects(0x4181, &[fixed(5000), fixed(6000), fixed(12000), reserved]);
    let advertisements = pd_rx::decode_wire(Sop::Partner, &wire, false)
        .unwrap()
        .0
        .advertisements()
        .unwrap();
    let source = match Message::from_bytes(&wire[..wire.len() - 4])
        .unwrap()
        .payload
        .unwrap()
    {
        Payload::Data(Data::SourceCapabilities(caps)) => caps,
        _ => panic!("fixture"),
    };
    let request::PowerSource::Pps(rdo) =
        pd_spr::request_pps::<Portable>(&source, 4, 3300, 1000).unwrap()
    else {
        panic!()
    };
    let request::PowerSource::Pps(clean) =
        pd_spr::request_pps::<Portable>(&caps(), 4, 3300, 1000).unwrap()
    else {
        panic!()
    };
    assert_eq!(rdo.0, clean.0);
    let target = Target {
        mv: 3300,
        ma: 1000,
        pps_object: 4,
    };
    assert!(pd_spr::review_rdo::<Portable>(
        &advertisements,
        rdo.0,
        Some(target)
    ));
    // Our own outgoing RDO reserved bits stay zero (sender rule).
    for bit in [27, 21, 8, 7] {
        assert!(
            !pd_spr::review_rdo::<Portable>(&advertisements, rdo.0 | (1 << bit), Some(target)),
            "{bit}"
        );
    }
}

#[test]
fn sink_capabilities_extended_derives_table_6_61_from_limits_only() {
    let block = pd_spr::sink_capabilities_extended::<Portable>().unwrap();
    let mut expected = [0u8; 24];
    expected[0..2].copy_from_slice(&[0xff, 0xff]); // no USB-IF VID (section 6.1.5)
    expected[10] = 1; // SKEDB Version 1.0
    expected[17] = 0b11; // VBUS powered, PPS
    // 5 V x 500 mA -> 3 W; max(12 V x 500 mA, 11 V x 1 A) -> 11 W;
    // max(11 V x 5 A APDO, 12 V x 5 A advertised fixed) -> 60 W (all rounded up).
    expected[18..21].copy_from_slice(&[3, 11, 60]);
    assert_eq!(block, expected);
    assert_eq!(pd_spr::sink_capabilities_extended::<Bad>(), None);
    assert_eq!(pd_spr::spr_pdps::<Portable>(), Some((3, 11, 60)));
}

#[test]
fn sink_capabilities_advertise_what_the_limits_request() {
    // Section 6.4.1.2 / 6.4.1.1: vSafe5V first (Higher Capability, Table 6.9),
    // other fixed targets ascending, all at the fixed sink current (caller limit,
    // at most the 5 A cable ceiling), then one PPS APDO (Table 6.14) under PD3 only.
    let fixed = [0x1001_91f4, 0x0001_e1f4, 0x0003_c1f4];
    let (objects, count) = pd_spr::sink_capabilities::<Portable>(false).unwrap();
    assert_eq!(objects[..count], fixed);
    let (objects, count) = pd_spr::sink_capabilities::<Portable>(true).unwrap();
    assert_eq!(objects[..3], fixed);
    // 5.0-11.0 V at 5 A: 11 V x 5 A = 55 W, within the 60 W Maximum PDP.
    assert_eq!(objects[3..count], [0xc0dc_3264]);
    assert_eq!(pd_spr::sink_capabilities::<Bad>(true), None);
    // Unordered, duplicated and out-of-range targets are sorted and dropped.
    struct Messy;
    impl Limits for Messy {
        const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
        const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
        const MAX_REQUEST_MV: u32 = 15_000;
        const MIN_PPS_MV: u32 = 5_000;
        const MAX_PPS_MV: u32 = 5_900;
        const TARGET_MV: u32 = 9_000;
        const REQUEST_MA: u32 = 100;
        const MAX_CURRENT_MA: u32 = 5_000;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = true;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b10;
        const FIXED_TARGETS: &'static [u16] = &[15_000, 9_000, 5_000, 9_000, 20_000, 12_000];
        const PPS_REFRESH_MS: u64 = 3_000;
    }
    let (objects, count) = pd_spr::sink_capabilities::<Messy>(true).unwrap();
    assert_eq!(
        objects[..count],
        [
            0x1001_91f4,
            0x0002_d1f4,
            0x0003_c1f4,
            0x0004_b1f4,
            0xc076_3264
        ]
    );
}

#[test]
fn pps_current_stays_within_sink_maximum_pdp() {
    struct Spr;
    impl Limits for Spr {
        const SESSION_POLICY: pd_spr::SessionPolicy = timing::SESSION_POLICY;
        const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
        const MAX_REQUEST_MV: u32 = 30_000;
        const MIN_PPS_MV: u32 = 5_000;
        const MAX_PPS_MV: u32 = 21_000;
        const TARGET_MV: u32 = 9_000;
        const REQUEST_MA: u32 = 100;
        const MAX_CURRENT_MA: u32 = 5_000;
        const USB_COMMUNICATIONS_CAPABLE: bool = false;
        const NO_USB_SUSPEND: bool = true;
        const HIGHER_CAPABILITY: bool = true;
        const UNCONSTRAINED_POWER: bool = false;
        const SINK_POWER_MODES: u8 = 0b10;
        const FIXED_TARGETS: &'static [u16] = &[5_000, 20_000];
        const PPS_REFRESH_MS: u64 = 4_000;
    }
    // Section 3.4.2: RDO power <= Sink Maximum PDP (100 W), rounded down to 50 mA.
    assert_eq!(pd_spr::pps_ceiling_ma::<Spr>(20_000), 5_000);
    assert_eq!(pd_spr::pps_ceiling_ma::<Spr>(20_020), 4_950);
    assert_eq!(pd_spr::pps_ceiling_ma::<Spr>(21_000), 4_750);
    assert_eq!(pd_spr::pps_ceiling_ma::<Spr>(5_000), 5_000);
    assert_eq!(pd_spr::pps_ceiling_ma::<Spr>(0), 0);
    assert_eq!(pd_spr::pps_ceiling_ma::<Bad>(5_000), 0);
    let apdo = |ma: u32| 0xc000_0000 | (210 << 17) | (50 << 8) | (ma / 50);
    let source = |raw: u32| {
        let wire = objects(0x2181, &[fixed(5000), raw]);
        let advertisements = pd_rx::decode_wire(Sop::Partner, &wire, false)
            .unwrap()
            .0
            .advertisements()
            .unwrap();
        let caps = match Message::from_bytes(&wire[..wire.len() - 4])
            .unwrap()
            .payload
            .unwrap()
        {
            Payload::Data(Data::SourceCapabilities(caps)) => caps,
            _ => panic!("fixture"),
        };
        (caps, advertisements)
    };
    let (caps, advertisements) = source(apdo(5_000));
    assert!(pd_spr::request_pps::<Spr>(&caps, 2, 20_000, 5_000).is_ok());
    assert!(pd_spr::request_pps::<Spr>(&caps, 2, 21_000, 4_750).is_ok());
    assert!(pd_spr::request_pps::<Spr>(&caps, 2, 21_000, 4_800).is_err());
    let request::PowerSource::Pps(rdo) =
        pd_spr::request_pps::<Spr>(&caps, 2, 21_000, 4_750).unwrap()
    else {
        panic!()
    };
    let target = Target {
        mv: 21_000,
        ma: 4_750,
        pps_object: 2,
    };
    assert!(pd_spr::review_rdo::<Spr>(
        &advertisements,
        rdo.0,
        Some(target)
    ));
    let over = (rdo.0 & !0x7f) | 96;
    assert!(!pd_spr::review_rdo::<Spr>(
        &advertisements,
        over,
        Some(Target {
            ma: 4_800,
            ..target
        })
    ));
    // The selection never offers a current above the cap: previews are
    // refused and a voltage step lowers the current.
    let mut wire = objects(0x2181, &[fixed(5000), apdo(5_000)]);
    wire.truncate(wire.len() - 4);
    let mut driver = DriverModel {
        rx: [wire, vec![0x83, 3], vec![0x86, 5]].into(),
        tx: vec![],
    };
    let selection = pd_spr::Selection::<Spr>::new();
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = pd_spr::Trial::<Spr>::maintained();
    let mut future = Box::pin(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    assert!(poll(future.as_mut()).is_pending());
    assert!(
        selection.option(1)
            == Some(Target {
                mv: 5_000,
                ma: 5_000,
                pps_object: 2
            })
    );
    assert!(!selection.preview(Target {
        mv: 21_000,
        ma: 5_000,
        pps_object: 2
    }));
    assert!(!selection.preview(Target {
        mv: 21_000,
        ma: 4_800,
        pps_object: 2
    }));
    assert!(selection.preview(Target {
        mv: 21_000,
        ma: 4_750,
        pps_object: 2
    }));
    assert!(selection.preview(Target {
        mv: 20_000,
        ma: 5_000,
        pps_object: 2
    }));
    selection.advance(1_000, 50);
    assert_eq!(
        (selection.candidate_mv(), selection.candidate_ma()),
        (21_000, 4_750)
    );
    selection.toggle_adjustment();
    selection.advance(1_000, 50);
    assert_eq!(
        selection.candidate_ma(),
        pd_spr::PPS_MIN_MA,
        "wraps instead of 4.8 A"
    );
    drop(future);
    // Table 6.13: a Maximum Current above 100 x 50 mA is Invalid.
    let (caps, advertisements) = source(apdo(5_050));
    assert!(pd_spr::request_pps::<Spr>(&caps, 2, 9_000, 1_000).is_err());
    let raw = (2 << 28) | ((9_000 / 20) << 9) | 20;
    assert!(!pd_spr::review_rdo::<Spr>(
        &advertisements,
        raw,
        Some(Target {
            mv: 9_000,
            ma: 1_000,
            pps_object: 2
        })
    ));
}

#[test]
fn bist_request_follows_table_6_23_at_vsafe5v_only() {
    use pd_spr::{BistRequest::*, bist_request};
    let bist = |objects: u16| 3 | objects << 12;
    assert_eq!(bist_request(bist(7), 0x8000_0000, 1), TestData);
    assert_eq!(bist_request(bist(1), 0x8fff_ffff, 1), TestData); // B27..0 Reserved, ignored
    assert_eq!(bist_request(bist(1), 0x5000_0000, 1), CarrierMode);
    // Section 9.2.26.4: only at vSafe5V (the contract on object 1).
    for object in [0, 2, 7] {
        assert_eq!(
            bist_request(bist(7), 0x8000_0000, object),
            Ignore,
            "{object}"
        );
        assert_eq!(
            bist_request(bist(1), 0x5000_0000, object),
            Ignore,
            "{object}"
        );
    }
    // Shared Test Mode Entry/Exit (shared-capacity Sources), PD2 BFSK-only and Invalid modes.
    for mode in [0u32, 1, 2, 3, 4, 6, 7, 9, 10, 11, 15] {
        assert_eq!(bist_request(bist(1), mode << 28, 1), Ignore, "{mode}");
    }
    // Not a BIST Data Message: no objects, another type, or Extended.
    assert_eq!(bist_request(3, 0x8000_0000, 1), Ignore);
    assert_eq!(bist_request(6 | 1 << 12, 0x8000_0000, 1), Ignore);
    assert_eq!(bist_request(0x8003 | 1 << 12, 0x8000_0000, 1), Ignore);
}

#[test]
fn ready_response_follows_tables_7_1_7_2_and_6_48() {
    use pd_spr::{ReadyResponse::*, ready_response as respond};
    let control = |kind: u16| kind;
    let data = |kind: u16| kind | 1 << 12;
    for kind in [1, 13] {
        assert_eq!(respond(1, control(kind), None), Policy);
        assert_eq!(respond(2, control(kind), None), Policy);
    }
    assert_eq!(respond(1, control(8), None), SinkCap); // Get_Sink_Cap
    assert_eq!(respond(2, control(8), None), SinkCap);
    // Table 7.1: Accept, Reject, PS_RDY, Wait are Unexpected in PE_SNK_Ready.
    for kind in [3, 4, 6, 12] {
        assert_eq!(respond(1, control(kind), None), SoftReset, "{kind}");
        assert_eq!(respond(2, control(kind), None), SoftReset, "{kind}");
    }
    assert_eq!(respond(1, control(5), None), Ignore); // PD2 Ping
    assert_eq!(respond(2, control(5), None), Unsupported); // deprecated PD3 Ping
    assert_eq!(respond(2, control(16), None), Ignore); // Not_Supported received
    assert_eq!(respond(1, control(16), None), Unsupported); // reserved in PD2 -> Reject
    assert_eq!(respond(2, control(24), None), Revision);
    assert_eq!(respond(1, control(24), None), Unsupported);
    assert_eq!(respond(2, control(22), None), SinkCapExtended);
    assert_eq!(respond(1, control(22), None), Unsupported); // reserved in PD2 -> Reject
    // Sections 7.12.1/7.13 (PD2 V1.3 6.3.9/6.3.11): DR_Swap and VCONN_Swap get
    // Reject, not Not_Supported.
    for kind in [9, 11] {
        assert_eq!(respond(1, control(kind), None), Reject, "{kind}");
        assert_eq!(respond(2, control(kind), None), Reject, "{kind}");
    }
    assert_eq!(respond(2, control(18), None), Status);
    assert_eq!(respond(1, control(18), None), Unsupported);
    for kind in [2, 7, 10, 14, 15, 17, 19, 20, 21, 23, 25, 31] {
        assert_eq!(respond(2, control(kind), None), Unsupported, "{kind}");
    }
    assert_eq!(respond(2, data(1), None), Policy);
    assert_eq!(respond(2, data(3), None), Ignore); // BIST
    assert_eq!(respond(1, data(3), None), Ignore);
    assert_eq!(respond(2, data(6), None), Alert);
    assert_eq!(respond(1, data(6), None), Unsupported); // reserved in PD2 -> Reject
    assert_eq!(respond(2, data(15), None), Unsupported); // sections 8.4/8.5
    assert_eq!(respond(1, data(15), None), Ignore); // PD2 V1.3 section 6.4.4
    for kind in [2, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 31] {
        assert_eq!(respond(2, data(kind), None), Unsupported, "{kind}");
    }
    let extended = |kind: u16| 0x8000 | kind | 1 << 12;
    assert_eq!(respond(2, extended(3), Some(0x8001)), Unsupported); // single Chunk
    assert_eq!(respond(2, extended(3), Some(0x0004)), Unsupported); // unchunked
    assert_eq!(
        respond(2, extended(30), Some(0x8000 | 27)),
        UnsupportedAfterChunk
    ); // Chunk 0 of 2
    assert_eq!(
        respond(2, extended(30), Some(0x8000 | 1 << 11 | 40)),
        UnsupportedAfterChunk
    );
    assert_eq!(
        respond(2, extended(30), Some(0x8000 | 9 << 11 | 260)),
        UnsupportedAfterChunk
    );
    for chunk in 10..16 {
        assert_eq!(
            respond(2, extended(30), Some(0x8000 | chunk << 11 | 40)),
            Ignore,
            "{chunk}"
        );
    }
    assert_eq!(
        respond(2, extended(30), Some(0x8400 | 1 << 11)),
        Unsupported
    ); // Chunk request
    assert_eq!(respond(2, extended(3), None), Policy); // truncated: fail closed
    assert_eq!(respond(1, extended(3), Some(0x8001)), Policy); // no PD2 Extended Messages
}

#[test]
fn partner_extended_framing_is_opt_in_and_never_assembles() {
    let partner = Sop::Partner;
    // Chunk 0 of a 40-byte Data Block: seven Data Objects, 26 data bytes.
    let mut first = 0x8028u16.to_le_bytes().to_vec();
    first.extend([0x5a; 26]);
    let chunk = wire(0xf19e, &first);
    let request = wire(0x919e, &[0x00, 0x8c, 0, 0]); // Request Chunk 1
    let unchunked = wire(0x9083, &[0x04, 0x00, 1, 2, 3, 4]);
    for bytes in [&chunk, &request, &unchunked] {
        assert_eq!(
            pd_rx::decode_wire(partner, bytes, false),
            Err(pd_rx::Error::UnsupportedExtended)
        );
        assert_eq!(
            pd_rx::decode_wire(partner, bytes, true),
            Err(pd_rx::Error::UnsupportedExtended)
        );
        assert_eq!(
            pd_rx::decode_wire_with(partner, bytes, true, false),
            Err(pd_rx::Error::UnsupportedExtended)
        );
        let (frame, used) = pd_rx::decode_wire_with(partner, bytes, false, true).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(frame.sop(), Sop::Partner);
        assert_eq!(
            pd_rx::wire_frame_len(partner, &bytes[..4], false, true),
            Ok(bytes.len())
        );
    }
    // Only a complete single Chunk exposes data; later Chunks are never assembled.
    assert_eq!(
        pd_rx::decode_wire_with(partner, &chunk, false, true)
            .unwrap()
            .0
            .extended_data(),
        Err(pd_rx::Error::InvalidExtended)
    );
    assert_eq!(
        pd_rx::decode_wire_with(partner, &unchunked, false, true)
            .unwrap()
            .0
            .extended_data(),
        Ok(&[1, 2, 3, 4][..])
    );
    // Size beyond MaxExtendedMsgLen, chunked without objects, or an unchunked
    // payload beyond one Chunk is malformed.
    let mut oversize = 0x8105u16.to_le_bytes().to_vec();
    oversize.extend([0; 26]);
    assert_eq!(
        pd_rx::decode_wire_with(partner, &wire(0xf19e, &oversize), false, true),
        Err(pd_rx::Error::InvalidExtended)
    );
    assert_eq!(
        pd_rx::decode_wire_with(partner, &wire(0x819e, &[]), false, true),
        Err(pd_rx::Error::InvalidExtended)
    );
    let mut long = 0x001bu16.to_le_bytes().to_vec();
    long.extend([0; 27]);
    assert_eq!(
        pd_rx::decode_wire_with(partner, &wire(0x8083, &long), false, true),
        Err(pd_rx::Error::InvalidExtended)
    );
    // Cable bounds are unchanged by partner framing.
    assert_eq!(
        pd_rx::decode_wire_with(Sop::Cable, &wire(0x9182, &[2, 0x88, 0, 0]), true, true),
        Err(pd_rx::Error::InvalidExtended)
    );
}
