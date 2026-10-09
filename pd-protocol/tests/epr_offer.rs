//! EPR offer request through both gates and the chunking layer, against a
//! scripted Source, with a test-local caller policy.
#[path = "support/timing.rs"]
mod timing;
use pd_protocol::pd_spr::{self as shared, EprOffer, MaintainedTrace, Trial};
use std::{
    cell::RefCell,
    collections::VecDeque,
    future::pending,
    rc::Rc,
    task::{Context, Poll, Waker},
};
use usbpd_traits::{Driver, DriverRxError, DriverTxError};

struct EprLimits;
impl shared::Limits for EprLimits {
    const SINK_IDENTITY: shared::SinkIdentity = shared::SinkIdentity::UNASSIGNED;
    const SESSION_POLICY: shared::SessionPolicy = timing::SESSION_POLICY;
    const MAX_REQUEST_MV: u32 = 20_000;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = 5_000;
    const REQUEST_MA: u32 = 100;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 0b10;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 20_000];
    const PPS_REFRESH_MS: u64 = 4_000;
    const EPR_OFFER: bool = true;
}
type Selection = shared::Selection<EprLimits>;

/// Source/DFP Message at PD3 (Table 6.2), without FIFO token or CRC.
fn packet(kind: u16, id: u16, objects: &[u32]) -> Vec<u8> {
    let header = kind | 0x180 | id << 9 | (objects.len() as u16) << 12;
    let mut bytes = header.to_le_bytes().to_vec();
    for raw in objects {
        bytes.extend(raw.to_le_bytes());
    }
    bytes
}
/// Chunk `chunk` of EPR_Source_Capabilities carrying `block` (Table 6.48).
fn epr_chunk(id: u16, chunk: u16, block: &[u8]) -> Vec<u8> {
    let start = usize::from(chunk) * 26;
    let carried = &block[start..block.len().min(start + 26)];
    let mut body = (0x8000 | chunk << 11 | block.len() as u16)
        .to_le_bytes()
        .to_vec();
    body.extend(carried);
    body.resize(body.len().div_ceil(4) * 4, 0);
    let header = 0x8011 | 0x180 | id << 9 | (body.len() as u16 / 4) << 12;
    let mut bytes = header.to_le_bytes().to_vec();
    bytes.extend(body);
    bytes
}
/// Fixed Supply PDO (Table 6.9); bit 23 is EPR Mode Capable.
fn fixed(mv: u32, ma: u32) -> u32 {
    ((mv / 50) << 10) | (ma / 10)
}
const EPR_CAPABLE: u32 = 1 << 23;
/// SPR PPS 3.3-21 V 5 A (Table 6.13) and EPR AVS 15-28 V 140 W (Table 6.15).
const PPS: u32 = 0xc000_0000 | (210 << 17) | (33 << 8) | 100;
const AVS: u32 = 0xd000_0000 | (280 << 17) | (150 << 8) | 140;

struct Source {
    rx: VecDeque<Vec<u8>>,
    tx: Vec<Vec<u8>>,
    /// EPR_Get_Source_Cap attempts still to come back Deferred (SinkTxNG).
    defer: usize,
}
struct Scripted<'a> {
    source: Rc<RefCell<Source>>,
    selection: &'a Selection,
}
impl Driver for Scripted<'_> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        panic!("no Hard Reset in this script")
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        let mut source = self.source.borrow_mut();
        let header = u16::from_le_bytes([bytes[0], bytes[1]]);
        if header & 0xf1ff == shared::EPR_GET_SOURCE_CAP_HEADER && source.defer > 0 {
            source.defer -= 1;
            return Err(DriverTxError::Deferred);
        }
        source.tx.push(bytes.to_vec());
        Ok(())
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        std::future::poll_fn(|_| match self.source.borrow_mut().rx.pop_front() {
            Some(bytes) => {
                self.selection.set_rx_idle(false);
                buffer[..bytes.len()].copy_from_slice(&bytes);
                Poll::Ready(Ok(bytes.len()))
            }
            None => {
                self.selection.set_rx_idle(true);
                Poll::Pending
            }
        })
        .await
    }
}
struct Never;
impl usbpd::timers::Timer for Never {
    async fn after_millis(_ms: u64) {
        pending::<()>().await;
    }
}

#[test]
fn epr_offer_is_asked_once_and_assembled_from_two_chunks_without_epr_request() {
    let first = fixed(5_000, 3_000) | EPR_CAPABLE;
    let spr = [first, fixed(9_000, 3_000), fixed(20_000, 5_000), PPS];
    let selection = Selection::new();
    let source = Rc::new(RefCell::new(Source {
        rx: [packet(1, 0, &spr), packet(3, 1, &[]), packet(6, 2, &[])].into(),
        tx: Vec::new(),
        defer: 0,
    }));
    let mut driver = Scripted {
        source: source.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(MaintainedTrace::default());
    let mut trial = Trial::<EprLimits>::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    let mut poll = || engine.as_mut().poll(&mut cx);

    assert!(poll().is_pending());
    assert!(selection.established_target().is_some());
    assert!(selection.epr_capable());
    assert_eq!(selection.epr_offer(), EprOffer::NotAsked);
    assert_eq!(source.borrow().tx.len(), 1, "only the Request");

    assert!(selection.request_epr_offer());
    assert!(!selection.request_epr_offer(), "already asked");
    assert!(poll().is_pending());
    let request = source.borrow().tx[1].clone();
    // Extended, one Data Object, PD3, Sink/UFP, Extended_Control, MessageID 1;
    // Chunked, Data Size 2; ECDB Type 1 (EPR_Get_Source_Cap), Data 0.
    assert_eq!(request, [0x90, 0x92, 0x02, 0x80, 0x01, 0x00]);

    // Positions 1..7: the SPR objects, zero filled; then EPR 28 V 5 A and AVS.
    let mut objects = [0; shared::EPR_OBJECTS];
    objects[..4].copy_from_slice(&spr);
    objects[7] = fixed(28_000, 5_000);
    objects[8] = AVS;
    let block: Vec<u8> = objects[..9].iter().flat_map(|o| o.to_le_bytes()).collect();
    source.borrow_mut().rx.push_back(epr_chunk(3, 0, &block));
    assert!(poll().is_pending());
    // Request for Chunk 1: Extended, one Data Object, EPR_Source_Capabilities,
    // MessageID 2; Chunked, Chunk 1, Request Chunk, Data Size 0; 00h padding.
    assert_eq!(source.borrow().tx[2], [0x91, 0x94, 0x00, 0x8c, 0x00, 0x00]);
    assert_eq!(selection.epr_offer(), EprOffer::Asked);

    source.borrow_mut().rx.push_back(epr_chunk(4, 1, &block));
    assert!(poll().is_pending());
    assert_eq!(selection.epr_offer(), EprOffer::Received(objects));
    // Informational only: no EPR_Request, no Soft_Reset, nothing more sent.
    assert_eq!(source.borrow().tx.len(), 3);
    assert_eq!(trace.borrow().soft_resets_sent, 0);
}

#[test]
fn epr_offer_is_not_asked_of_a_source_without_epr_mode() {
    let selection = Selection::new();
    let source = Rc::new(RefCell::new(Source {
        rx: [
            packet(1, 0, &[fixed(5_000, 3_000), fixed(9_000, 3_000)]),
            packet(3, 1, &[]),
            packet(6, 2, &[]),
        ]
        .into(),
        tx: Vec::new(),
        defer: 0,
    }));
    let mut driver = Scripted {
        source: source.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(MaintainedTrace::default());
    let mut trial = Trial::<EprLimits>::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(!selection.request_epr_offer());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(source.borrow().tx.len(), 1);
    assert_eq!(selection.epr_offer(), EprOffer::NotAsked);
}

#[test]
fn a_deferred_epr_offer_request_is_asked_again_after_the_cooldown_up_to_the_limit() {
    let retry = timing::SESSION_POLICY.retry_delay_ms;
    for (defer, sent) in [(1, true), (3, false)] {
        let first = fixed(5_000, 3_000) | EPR_CAPABLE;
        let selection = Selection::new();
        let source = Rc::new(RefCell::new(Source {
            rx: [
                packet(1, 0, &[first, fixed(9_000, 3_000)]),
                packet(3, 1, &[]),
                packet(6, 2, &[]),
            ]
            .into(),
            tx: Vec::new(),
            defer,
        }));
        let mut driver = Scripted {
            source: source.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(MaintainedTrace::default());
        let mut trial = Trial::<EprLimits>::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        let mut poll = || engine.as_mut().poll(&mut cx);
        assert!(poll().is_pending());
        assert!(selection.request_epr_offer());
        for attempt in 0..3 {
            selection.set_now_ms(attempt * retry);
            assert!(poll().is_pending());
        }
        if sent {
            // Unsent attempts use no MessageID: the request carries ID 1.
            assert_eq!(source.borrow().tx[1], [0x90, 0x92, 0x02, 0x80, 0x01, 0x00]);
            assert_eq!(selection.epr_offer(), EprOffer::Asked);
        } else {
            assert_eq!(source.borrow().tx.len(), 1, "only the Request");
            assert_eq!(selection.epr_offer(), EprOffer::NotSent);
            assert!(selection.request_epr_offer(), "can be asked again");
        }
    }
}
