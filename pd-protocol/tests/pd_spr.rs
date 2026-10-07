//! Preserved session regressions with a test-local caller policy and scripted PHY.
#[path = "support/session_policy.rs"]
mod pd_spr;
use pd_spr::{Error, Trial};
use std::{
    cell::RefCell,
    collections::VecDeque,
    future::{Future, pending},
    rc::Rc,
    task::{Context, Poll, Waker},
};
use usbpd::protocol_layer::message::{
    Message, Payload,
    data::{Data, request, source_capabilities},
    header::{ControlMessageType, MessageType},
};
use usbpd_traits::{Driver, DriverRxError, DriverTxError};

// Independent wire fixtures (without FIFO token/CRC, supplied by the native PHY).
fn packet(kind: u16, id: u16, pdos: &[u32]) -> Vec<u8> {
    let header = kind | 0x180 | id << 9 | (pdos.len() as u16) << 12;
    let mut bytes = header.to_le_bytes().to_vec();
    for raw in pdos {
        bytes.extend(raw.to_le_bytes());
    }
    bytes
}
fn fixed(mv: u32, ma: u32) -> u32 {
    ((mv / 50) << 10) | (ma / 10)
}
/// Fixed RDO current fields: Operating = Maximum Operating = the object's
/// Maximum Current (Table 6.19; the fixture's caller current ceiling never binds).
fn fixed_current(ma: u32) -> u32 {
    (ma / 10) << 10 | (ma / 10)
}
fn caps(pdos: &[u32]) -> source_capabilities::SourceCapabilities {
    let message = Message::from_bytes(&packet(1, 0, pdos)).unwrap();
    let Some(Payload::Data(Data::SourceCapabilities(caps))) = message.payload else {
        panic!("caps fixture");
    };
    caps
}
fn ready() -> Vec<Vec<u8>> {
    vec![
        packet(
            1,
            0,
            &[
                fixed(5000, 3000),
                fixed(9000, 3000),
                fixed(20000, 5000),
                0xc0dc2164,
            ],
        ),
        packet(3, 1, &[]),
        packet(6, 2, &[]),
    ]
}
struct Ticks;
impl usbpd::timers::Timer for Ticks {
    async fn after_millis(_ms: u64) {
        let mut first = true;
        std::future::poll_fn(|cx| {
            if std::mem::take(&mut first) {
                cx.waker().wake_by_ref();
                Poll::Pending
            } else {
                Poll::Ready(())
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
fn complete<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..32 {
        if let Poll::Ready(result) = future.as_mut().poll(&mut cx) {
            return result;
        }
    }
    panic!("bounded fixture did not complete");
}
struct Fake<const AUTO_CRC: bool = true, const AUTO_RETRY: bool = true> {
    rx: VecDeque<Vec<u8>>,
    tx: Vec<Vec<u8>>,
    reads: usize,
    waits: usize,
    rx_error: Option<DriverRxError>,
    tx_error: Option<DriverTxError>,
    /// Zero-based transmissions that get no GoodCRC after the PHY's retries.
    nack: Vec<usize>,
    /// Result of Hard Reset Signaling, which is recorded in `tx` as `HARD_RESET`.
    hard_reset_error: Option<DriverTxError>,
}
/// Marker for Hard Reset Signaling in a fake's `tx` record (not a Message).
const HARD_RESET: [u8; 2] = *b"HR";
impl<const C: bool, const R: bool> Fake<C, R> {
    fn new(rx: Vec<Vec<u8>>) -> Self {
        Self {
            rx: rx.into(),
            tx: vec![],
            reads: 0,
            waits: 0,
            rx_error: None,
            tx_error: None,
            nack: Vec::new(),
            hard_reset_error: None,
        }
    }
}
impl<const C: bool, const R: bool> Driver for Fake<C, R> {
    const HAS_AUTO_GOOD_CRC: bool = C;
    const HAS_AUTO_RETRY: bool = R;
    async fn wait_for_vbus(&mut self) {
        self.waits += 1;
    }
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        self.tx.push(HARD_RESET.to_vec());
        match self.hard_reset_error.take() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        self.tx.push(bytes.to_vec());
        if self.nack.contains(&(self.tx.len() - 1)) {
            return Err(DriverTxError::NotAcknowledged);
        }
        match self.tx_error.take() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }
    async fn receive(&mut self, bytes: &mut [u8]) -> Result<usize, DriverRxError> {
        self.reads += 1;
        if let Some(error) = self.rx_error.take() {
            return Err(error);
        }
        match self.rx.pop_front() {
            Some(packet) => {
                bytes[..packet.len()].copy_from_slice(&packet);
                Ok(packet.len())
            }
            None => pending().await,
        }
    }
}

// Scripted test peer: auto-GoodCRC/retry are provided by this driver contract;
// wait_for_vbus completes immediately in this scripted policy test.
struct InteractiveState {
    rx: VecDeque<Vec<u8>>,
    tx: Vec<Vec<u8>>,
    idle: bool,
    now: u64,
}
/// Marker for a BIST carrier in an interactive peer's `tx` record (not a Message).
const CARRIER: [u8; 2] = *b"BC";
thread_local! {
    /// The interactive peer answers each information request (`pd_partner`)
    /// at once with Not_Supported and records it in `QUERIES_SEEN`, not `tx`, so
    /// scripted sessions keep their transmit counts. Query tests turn this off.
    static ANSWER_QUERIES: std::cell::Cell<bool> = const { std::cell::Cell::new(true) };
    static QUERIES_SEEN: RefCell<Vec<Vec<u8>>> = const { RefCell::new(Vec::new()) };
    static LAST_RX_ID: std::cell::Cell<u16> = const { std::cell::Cell::new(0) };
}
struct Interactive<'a> {
    state: Rc<RefCell<InteractiveState>>,
    selection: &'a pd_spr::Selection,
}
impl Driver for Interactive<'_> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        panic!("reset")
    }
    async fn transmit_bist_carrier(&mut self) -> Result<(), DriverTxError> {
        self.state.borrow_mut().tx.push(CARRIER.to_vec());
        Ok(())
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        let mut state = self.state.borrow_mut();
        if ANSWER_QUERIES.get() && pd_protocol::pd_partner::query_request(bytes).is_some() {
            QUERIES_SEEN.with_borrow_mut(|seen| seen.push(bytes.to_vec()));
            // MessageID 4 ahead of the last received one: never a retransmission
            // of it, and never the ID the script sends next (last + 1).
            let id = (LAST_RX_ID.get() + 4) % 8;
            state.rx.push_front(packet(16, id, &[]));
            return Ok(());
        }
        state.tx.push(bytes.to_vec());
        if bytes[0] & 0x1f == 2 {
            self.selection.note_request_sent(state.now);
        }
        Ok(())
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        std::future::poll_fn(|_| {
            let mut state = self.state.borrow_mut();
            self.selection.set_now_ms(state.now);
            if let Some(bytes) = state.rx.pop_front() {
                if bytes.is_empty() {
                    return Poll::Ready(Err(DriverRxError::HardReset)); // Hard Reset Signaling
                }
                self.selection.set_rx_idle(false);
                LAST_RX_ID.set((u16::from_le_bytes([bytes[0], bytes[1]]) >> 9) & 7);
                buffer[..bytes.len()].copy_from_slice(&bytes);
                Poll::Ready(Ok(bytes.len()))
            } else {
                self.selection.set_rx_idle(state.idle);
                Poll::Pending
            }
        })
        .await
    }
}

#[test]
fn selectable_decline_preserves_contract_and_gates_confirmation() {
    for response in [4, 12] {
        // Reject, Wait
        let selection = pd_spr::Selection::new();
        let state = Rc::new(RefCell::new(InteractiveState {
            rx: ready().into(),
            tx: Vec::new(),
            idle: true,
            now: 0,
        }));
        let mut driver = Interactive {
            state: state.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        let established = trace.borrow().contract.as_ref().unwrap().request.0;
        assert!(selection.preview(pd_protocol::pd_spr::Target {
            mv: 9000,
            ma: 100,
            pps_object: 0
        }));
        assert!(selection.confirm());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        state.borrow_mut().rx.push_back(packet(response, 3, &[]));
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            established
        );
        assert_eq!(selection.target_mv(), 5000);
        assert!(!selection.confirm());
        state.borrow_mut().now = 99;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert!(!selection.confirm());
        state.borrow_mut().now = 100;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 2, "no automatic retry");
        if response == 12 {
            assert!(selection.confirm());
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(state.borrow().tx.len(), 3);
            state
                .borrow_mut()
                .rx
                .extend([packet(3, 4, &[]), packet(6, 5, &[])]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(selection.target_mv(), 9000);
        } else {
            assert!(!selection.confirm());
            state.borrow_mut().rx.extend([
                packet(1, 4, &[fixed(5000, 3000), fixed(9000, 3000)]),
                packet(3, 5, &[]),
                packet(6, 6, &[]),
            ]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(selection.target_mv(), 5000);
            assert!(selection.confirm(), "fresh capabilities lift Reject gate");
        }
    }
}

#[test]
fn declined_change_keeps_established_pps_refresh_schedule() {
    for response in [4, 12] {
        for attempted_pps in [false, true] {
            let selection = pd_spr::Selection::new();
            let state = Rc::new(RefCell::new(InteractiveState {
                rx: ready().into(),
                tx: Vec::new(),
                idle: true,
                now: 0,
            }));
            let mut driver = Interactive {
                state: state.clone(),
                selection: &selection,
            };
            let trace = RefCell::new(pd_spr::MaintainedTrace::default());
            let mut trial = Trial::maintained();
            let mut engine =
                std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
            let mut cx = Context::from_waker(Waker::noop());
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert!(selection.preview(pd_protocol::pd_spr::Target {
                mv: 5000,
                ma: 1000,
                pps_object: 4
            }));
            assert!(selection.confirm());
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            state
                .borrow_mut()
                .rx
                .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            let established = trace.borrow().contract.as_ref().unwrap().request.0;
            state.borrow_mut().now = 1000;
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            // Section 7.16: the new PPS contract's one Get_PPS_Status (tx[2]).
            assert_eq!(state.borrow().tx.len(), 3);
            state.borrow_mut().rx.push_back(pps_status(0));
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            let candidate = if attempted_pps {
                pd_protocol::pd_spr::Target {
                    mv: 9000,
                    ma: 1000,
                    pps_object: 4,
                }
            } else {
                pd_protocol::pd_spr::Target {
                    mv: 9000,
                    ma: 100,
                    pps_object: 0,
                }
            };
            assert!(selection.preview(candidate));
            assert!(selection.confirm());
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            state.borrow_mut().rx.push_back(packet(response, 5, &[]));
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(selection.target_mv(), 5000);
            assert_eq!(selection.target_pps_object(), 4);
            assert_eq!(
                trace.borrow().contract.as_ref().unwrap().request.0,
                established
            );
            state.borrow_mut().now = 3999;
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(state.borrow().tx.len(), 4);
            state.borrow_mut().now = 4000;
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(state.borrow().tx.len(), 5);
            assert_eq!(
                u32::from_le_bytes(state.borrow().tx[4][2..6].try_into().unwrap()),
                established
            );
            state
                .borrow_mut()
                .rx
                .extend([packet(3, 6, &[]), packet(6, 7, &[])]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert_eq!(
                trace.borrow().contract.as_ref().unwrap().request.0,
                established
            );
        }
    }
}

#[test]
fn rejected_established_pps_refresh_halts_without_repeating_it() {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 5000,
        ma: 1000,
        pps_object: 4
    }));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    state.borrow_mut().now = 4000;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    state.borrow_mut().rx.push_back(packet(4, 5, &[]));
    assert!(matches!(
        engine.as_mut().poll(&mut cx),
        Poll::Ready(Err(Error::Declined(ControlMessageType::Reject)))
    ));
    assert_eq!(state.borrow().tx.len(), 3);
    assert!(
        trace.borrow().contract.is_some(),
        "last established contract remains historical evidence"
    );
}

#[test]
fn maintained_initial_decline_waits_for_fresh_capabilities() {
    for response in [4, 12] {
        let mut replies = ready();
        replies[1] = packet(response, 1, &[]);
        replies.truncate(2);
        replies.extend([ready()[0].clone(), packet(3, 2, &[]), packet(6, 3, &[])]);
        let mut phy = Fake::<true, true>::new(replies);
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        {
            let mut future = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
            assert!(
                future
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending()
            );
        }
        assert!(trace.borrow().contract.is_some());
        assert_eq!(phy.tx.len(), 2);
    }
}

#[test]
fn selectable_session_requires_confirmation_and_an_idle_receiver_for_every_change() {
    let advertisement = packet(
        1,
        0,
        &[
            fixed(5000, 3000),
            fixed(9000, 3000),
            fixed(12000, 3000),
            fixed(15000, 3000),
            fixed(20000, 3250),
            fixed(28000, 3000),
            0xc0dc2164,
        ],
    );
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: VecDeque::from([advertisement, packet(3, 1, &[]), packet(6, 2, &[])]),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(selection.target_mv(), 5000);
    assert!(trace.borrow().contract.is_some());
    for (index, mv) in [9000, 12000, 15000, 20000].into_iter().enumerate() {
        selection.advance(20, 50); // Preview only; no transmission.
        assert_eq!(selection.candidate_mv(), mv);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), index + 1);
        assert!(!selection.has_pending());
        state.borrow_mut().idle = false; // model a receiver still in I/O
        selection.set_rx_idle(false);
        assert!(selection.confirm());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), index + 1);
        state.borrow_mut().idle = true;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        let request = state.borrow().tx.last().unwrap().clone();
        assert_eq!(
            u32::from_le_bytes(request[2..].try_into().unwrap()),
            ((index as u32 + 2) << 28)
                | 0x0100_0000
                | fixed_current(if mv == 20000 { 3250 } else { 3000 })
        );
        state.borrow_mut().rx.extend([
            packet(3, ((index * 2 + 3) % 8) as u16, &[]),
            packet(6, ((index * 2 + 4) % 8) as u16, &[]),
        ]);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(selection.target_mv(), mv);
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            ((index as u32 + 2) << 28)
                | 0x0100_0000
                | fixed_current(if mv == 20000 { 3250 } else { 3000 })
        );
    }
    selection.advance(20, 50);
    assert_eq!(selection.candidate_mv(), 5000); // PPS starts safely at5V; EPR skipped
    assert!(selection.candidate_is_pps());
    assert!(!selection.has_pending());
    assert_eq!(selection.candidate_mv(), 5000);
}

#[test]
fn pps_request_is_exact_and_rejects_unsafe_ranges_objects_and_currents() {
    let source = caps(&[fixed(5000, 3000), 0xc0dc2164]); //3.3..11V,5A
    let request::PowerSource::Pps(rdo) = pd_spr::request_pps(&source, 2, 5020, 1000).unwrap()
    else {
        panic!("PPS");
    };
    assert_eq!(rdo.0, 0x2101_f614);
    assert_eq!((rdo.0 & 0x7f) * 50, 1000); // iPpsCLMin
    let request::PowerSource::Pps(rdo_5a) = pd_spr::request_pps(&source, 2, 5020, 5000).unwrap()
    else {
        panic!("PPS");
    };
    assert_eq!(rdo_5a.0, 0x2101_f664);
    assert_eq!((rdo_5a.0 & 0x7f) * 50, 5000);

    // Current limit boundaries
    assert!(pd_spr::request_pps(&source, 2, 5000, 950).is_err()); // below 1000 mA
    assert!(pd_spr::request_pps(&source, 2, 5000, 5050).is_err()); // exceeds 5000 mA
    assert!(pd_spr::request_pps(&source, 2, 5000, 1025).is_err()); // non-multiple of 50
    assert!(pd_spr::request_pps(&caps(&[fixed(5000, 3000), 0xc0dc2113]), 2, 5000, 1000).is_err()); // APDO only offers 950mA (< 1A)
    assert!(pd_spr::request_pps(&caps(&[fixed(5000, 3000), 0xc0dc2114]), 2, 5000, 1000).is_ok()); // APDO offers 1A

    for (object, mv) in [
        (0, 5000),
        (1, 5000),
        (3, 5000),
        (8, 5000),
        (2, 4980),
        (2, 5001),
        (2, 11020),
        (2, 30020),
    ] {
        assert!(pd_spr::request_pps(&source, object, mv, 1000).is_err());
    }
    for raw in [0xd0dc2164, 0xc0426e64, 0xc0dc2101, 0xc0dc0064] {
        assert!(
            pd_spr::request_pps(&caps(&[fixed(5000, 3000), raw]), 2, 5000, 1000).is_err(),
            "{raw:08x}"
        );
    }
    // Table 6.13: reserved bits 26..25, 16 and 7 are ignored by the receiver;
    // they neither refuse the APDO nor leak into the reviewed RDO.
    for raw in [0xc2dc2164, 0xc4dc2164, 0xc0dd2164, 0xc0dc21e4, 0xc6dd21e4] {
        let request::PowerSource::Pps(rdo) =
            pd_spr::request_pps(&caps(&[fixed(5000, 3000), raw]), 2, 5020, 1000).unwrap()
        else {
            panic!("PPS {raw:08x}");
        };
        assert_eq!(rdo.0, 0x2101_f614, "{raw:08x}");
    }
    assert!(pd_spr::request_pps(&caps(&[fixed(9000, 3000), 0xc0dc2164]), 2, 5000, 1000).is_err());
    for mv in [5000, 5020, 11000] {
        assert!(pd_spr::request_pps(&source, 2, mv, 1000).is_ok());
    }
}

#[test]
fn pps_selection_requires_pd3_and_keepalive_is_absolute_and_idle_safe() {
    for revision in [1, 2] {
        let mut advertisement = packet(1, 0, &[fixed(5000, 3000), 0xc0dc2164]);
        advertisement[0] = (advertisement[0] & !0xc0) | (revision << 6);
        let selection = pd_spr::Selection::new();
        let state = Rc::new(RefCell::new(InteractiveState {
            rx: VecDeque::from([advertisement, packet(3, 1, &[]), packet(6, 2, &[])]),
            tx: Vec::new(),
            idle: true,
            now: 0,
        }));
        let mut driver = Interactive {
            state: state.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        selection.advance(20, 50);
        assert_eq!(selection.candidate_is_pps(), revision == 2);
        if revision == 1 {
            assert_eq!(state.borrow().tx.len(), 1);
            continue;
        }
        selection.advance(20, 50); //5.02V preview; no TX
        assert_eq!(selection.candidate_mv(), 5020);
        assert!(!selection.request_cable_mode(0x8087)); // preview differs from safe fixed5V
        assert_eq!(state.borrow().tx.len(), 1);
        assert!(selection.confirm());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        let rdo = |bytes: &[u8]| u32::from_le_bytes(bytes[2..6].try_into().unwrap());
        assert_eq!(rdo(&state.borrow().tx[1]), 0x2101_f664);
        assert!(!selection.request_cable_mode(0x8087)); // pending PPS negotiation
        state
            .borrow_mut()
            .rx
            .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert!(!selection.request_cable_mode(0x8087)); // established PPS, even near5V
        // Section 7.16: one Get_PPS_Status after the new PPS contract settles.
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 3);
        state.borrow_mut().rx.push_back(pps_status(0));
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        state.borrow_mut().now = 3999;
        state.borrow_mut().rx.push_back(packet(5, 5, &[])); //Ping re-enters Ready
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 4); //Not_Supported, not a refresh
        state.borrow_mut().now = 4000;
        state.borrow_mut().idle = false;
        selection.set_rx_idle(false);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 4); //cannot cancel active I/O
        state.borrow_mut().idle = true;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 5);
        assert_eq!(rdo(&state.borrow().tx[4]), 0x2101_f664);
        // The five information requests at QUERY_SETTLE_MS used MessageIDs 4..0.
        assert_eq!(QUERIES_SEEN.with_borrow(|seen| seen.len()), 5);
        assert_eq!((state.borrow().tx[4][1] >> 1) & 7, 1);
        state
            .borrow_mut()
            .rx
            .extend([packet(3, 6, &[]), packet(6, 7, &[])]);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x2101_f664
        );
        assert_eq!(trace.borrow().re_requests_sent, 2);
        state.borrow_mut().now = 7999;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 5);
        selection.set_now_ms(12999);
        assert!(!selection.pps_expired());
        selection.set_now_ms(13000);
        assert!(selection.pps_expired());
        // Selecting a different preview does not alter the active PPS target.
        selection.set_now_ms(0);
        selection.advance(20, 50);
        assert_eq!(selection.target_mv(), 5020);
        assert_eq!(selection.candidate_mv(), 5040);
        selection.set_now_ms(500);
        selection.advance(100, 250); // Caller chooses the larger increment.
        assert_eq!(selection.candidate_mv(), 5140);
        assert!(!selection.has_pending());
    }
}

#[test]
fn request_is_exactly_the_reviewed_target_pdo_not_highest_or_usb_data() {
    let capabilities = caps(&[
        fixed(5000, 3000),
        fixed(9000, 3000),
        fixed(20000, 5000),
        fixed(28000, 5000),
        fixed(48000, 5000),
        0xc0dc2164,
    ]);
    let request::PowerSource::FixedVariableSupply(rdo) =
        pd_spr::request_target(&capabilities).unwrap()
    else {
        panic!("fixed RDO");
    };
    assert_eq!(rdo.0, 0x2104_b12c); // object2, no suspend, operating/max=10*10mA
    assert_eq!(rdo.object_position(), 2);
    assert!(!rdo.capability_mismatch());
    assert!(!rdo.giveback_flag());
    assert!(!rdo.epr_mode_capable());
    assert!(!rdo.unchunked_extended_messages_supported());
    assert!(!rdo.usb_communications_capable());
    let source_capabilities::PowerDataObject::FixedSupply(selected) =
        capabilities.pdos()[rdo.object_position() as usize - 1]
    else {
        panic!("selected fixed PDO");
    };
    assert_eq!(selected.raw_voltage() as u32 * 50, pd_spr::TARGET_MV);
    assert!(selected.raw_voltage() as u32 * 50 <= pd_spr::MAX_REQUEST_MV);
    // Never the highest advertised voltage.
    let highest = capabilities
        .pdos()
        .iter()
        .filter_map(|pdo| match pdo {
            source_capabilities::PowerDataObject::FixedSupply(fixed) => {
                Some(fixed.raw_voltage() as u32 * 50)
            }
            _ => None,
        })
        .max()
        .unwrap();
    assert!(selected.raw_voltage() as u32 * 50 < highest);
}

#[test]
fn a_source_without_a_usable_target_object_is_rejected() {
    for unusable in [
        vec![fixed(5000, 3000)],                     // no 9 V object
        vec![fixed(5000, 3000), fixed(9000, 90)],    // 9 V below the reviewed current
        vec![fixed(9000, 3000), 0xc0dc2164],         // 9 V first, no 5 V safe default
        vec![fixed(5000, 3000), fixed(20000, 5000)], // 9 V missing, higher present
    ] {
        assert!(
            matches!(
                pd_spr::request_target(&caps(&unusable)),
                Err(Error::UnsafeCapabilities)
            ),
            "{unusable:?}"
        );
    }
    assert!(matches!(
        pd_spr::request_target(&source_capabilities::SourceCapabilities::new_with_pdos(
            Default::default()
        )),
        Err(Error::UnsafeCapabilities)
    ));
    // The target object is usable on its own at exactly the reviewed current.
    assert!(pd_spr::request_target(&caps(&[fixed(5000, 3000), fixed(9000, 100)])).is_ok());
    let too_many = source_capabilities::SourceCapabilities::new_with_pdos(
        (0..8)
            .map(|_| {
                source_capabilities::PowerDataObject::FixedSupply(source_capabilities::FixedSupply(
                    fixed(9000, 3000),
                ))
            })
            .collect(),
    );
    assert!(matches!(
        pd_spr::request_target(&too_many),
        Err(Error::UnsafeCapabilities)
    ));
}

#[test]
fn upstream_engine_completes_only_after_accept_then_ps_rdy() {
    let mut phy = Fake::<true, true>::new(ready());
    let mut trial = Trial::once();
    let contract = complete(trial.run::<_, Ticks>(&mut phy)).unwrap();
    assert_eq!(contract.request.0, 0x2104_b12c);
    assert_eq!(contract.capabilities.pdos().len(), 4);
    assert_eq!(phy.tx.len(), 1);
    assert_eq!(&phy.tx[0][2..], &0x2104_b12cu32.to_le_bytes());
    let message = Message::from_bytes(&phy.tx[0]).unwrap();
    assert!(matches!(
        message.header.message_type(),
        MessageType::Data(usbpd::protocol_layer::message::header::DataMessageType::Request)
    ));
    assert!(matches!(
        message.header.port_power_role(),
        usbpd::PowerRole::Sink
    ));
    assert!(matches!(
        message.header.port_data_role(),
        usbpd::DataRole::Ufp
    ));
    assert_eq!(message.header.message_id(), 0);
    assert_eq!(phy.reads, 3);
    assert_eq!(phy.waits, 1);
    assert!(matches!(
        complete(trial.run::<_, Ticks>(&mut phy)),
        Err(Error::Stopped)
    ));
    assert_eq!(phy.tx.len(), 1);
    assert_eq!(phy.reads, 3);
}

#[test]
fn protocol_deduplicates_retransmission_and_ignores_good_crc() {
    let mut replies = ready();
    replies.insert(1, replies[0].clone());
    replies.insert(2, packet(1, 0, &[])); // GoodCRC is control1, not another PDO list
    let mut phy = Fake::<true, true>::new(replies);
    assert!(complete(Trial::once().run::<_, Ticks>(&mut phy)).is_ok());
    assert_eq!(phy.tx.len(), 1);
    assert_eq!(phy.reads, 5);
}

#[test]
fn source_reject_and_wait_stop_without_request_retry_or_reset() {
    for (kind, expected) in [
        (4, ControlMessageType::Reject),
        (12, ControlMessageType::Wait),
    ] {
        let mut replies = ready();
        replies[1] = packet(kind, 1, &[]);
        let mut phy = Fake::<true, true>::new(replies);
        let result = complete(Trial::once().run::<_, Ticks>(&mut phy));
        let Err(Error::Declined(observed)) = result else {
            panic!("wrong result {result:?}");
        };
        assert_eq!(observed as u8, expected as u8);
        assert_eq!(phy.tx.len(), 1);
        assert_eq!(phy.reads, 2);
        assert_eq!(phy.waits, 1);
    }
}

#[test]
fn invalid_peer_never_reaches_upstream_panic_or_extended_tx_paths() {
    let mut extended = ready()[0].clone();
    extended[1] |= 0x80;
    let mut bad_revision = ready()[0].clone();
    bad_revision[0] |= 0xc0;
    for invalid in [
        vec![0],
        vec![],
        extended,
        bad_revision,
        packet(13, 0, &[]),
        packet(31, 0, &[]),
    ] {
        let mut phy = Fake::<true, true>::new(vec![invalid]);
        let result = complete(Trial::once().run::<_, Ticks>(&mut phy));
        assert!(matches!(result, Err(Error::Peer)), "{result:?}");
        assert!(phy.tx.is_empty());
        assert_eq!(phy.reads, 1);
        assert_eq!(phy.waits, 1);
    }
}

#[test]
fn unsafe_capabilities_stop_before_native_transmit() {
    let mut phy =
        Fake::<true, true>::new(vec![packet(1, 0, &[fixed(20000, 5000), fixed(5000, 3000)])]);
    assert!(matches!(
        complete(Trial::once().run::<_, Ticks>(&mut phy)),
        Err(Error::UnsafeCapabilities)
    ));
    assert!(phy.tx.is_empty());
    assert_eq!(phy.reads, 1);
}

#[test]
fn one_shot_timeouts_and_out_of_order_messages_never_reset() {
    let frames = ready();
    for (replies, expected_tx) in [
        (vec![], 0),
        (vec![frames[0].clone()], 1),
        (frames[..2].to_vec(), 1),
        (vec![frames[0].clone(), frames[2].clone()], 1),
    ] {
        let mut phy = Fake::<true, true>::new(replies);
        let result = complete(Trial::once().run::<_, Ticks>(&mut phy));
        assert!(matches!(result, Err(Error::RecoveryBlocked)), "{result:?}");
        assert_eq!(phy.tx.len(), expected_tx);
        assert_eq!(phy.waits, 1);
    }
}

#[test]
fn native_receive_errors_do_not_hot_retry_or_recover() {
    for error in [DriverRxError::Discarded, DriverRxError::HardReset] {
        let mut phy = Fake::<true, true>::new(ready());
        phy.rx_error = Some(error);
        let result = complete(Trial::once().run::<_, Ticks>(&mut phy));
        assert!(matches!(
            (error, result),
            (DriverRxError::Discarded, Err(Error::ReceiveDiscarded))
                | (DriverRxError::HardReset, Err(Error::HardResetReceived))
        ));
        assert!(phy.tx.is_empty());
        assert_eq!(phy.reads, 1);
        assert_eq!(phy.waits, 1);
    }
}

#[test]
fn native_transmit_failure_is_not_retried_by_software() {
    for error in [DriverTxError::Discarded, DriverTxError::HardReset] {
        let mut phy = Fake::<true, true>::new(ready());
        phy.tx_error = Some(error);
        let result = complete(Trial::once().run::<_, Ticks>(&mut phy));
        assert!(matches!(
            (error, result),
            (DriverTxError::Discarded, Err(Error::Transmit(_)))
                | (DriverTxError::HardReset, Err(Error::HardResetReceived))
        ));
        assert_eq!(phy.tx.len(), 1);
        assert_eq!(phy.reads, 1);
        assert_eq!(phy.waits, 1);
    }
}

#[test]
fn cancellation_is_one_shot_at_each_handshake_wait() {
    for count in 0..3 {
        let mut phy = Fake::<true, true>::new(ready()[..count].to_vec());
        let mut trial = Trial::once();
        {
            let mut future = std::pin::pin!(trial.run::<_, Never>(&mut phy));
            assert!(
                future
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending()
            );
        }
        let reads = phy.reads;
        let tx = phy.tx.len();
        assert!(matches!(
            complete(trial.run::<_, Ticks>(&mut phy)),
            Err(Error::Stopped)
        ));
        assert_eq!(phy.reads, reads);
        assert_eq!(phy.tx.len(), tx);
    }
}

#[test]
fn unreviewed_acknowledgement_or_retry_phy_is_refused_before_any_io() {
    let mut crc = Fake::<false, true>::new(ready());
    assert!(matches!(
        complete(Trial::once().run::<_, Ticks>(&mut crc)),
        Err(Error::UnsupportedPhy)
    ));
    assert_eq!((crc.reads, crc.waits, crc.tx.len()), (0, 0, 0));
    let mut retries = Fake::<true, false>::new(ready());
    assert!(matches!(
        complete(Trial::once().run::<_, Ticks>(&mut retries)),
        Err(Error::UnsupportedPhy)
    ));
    assert_eq!((retries.reads, retries.waits, retries.tx.len()), (0, 0, 0));
}

#[test]
fn maintained_handshake_enters_ready_and_services_get_sink_cap() {
    let mut replies = ready();
    replies.push(packet(8, 3, &[])); // GetSinkCap = 8
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = std::cell::RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut future = std::pin::pin!(trial.run_maintained::<_, Ticks>(&mut phy, &trace));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..64 {
            let _ = future.as_mut().poll(&mut cx);
        }
    }
    let t = trace.borrow();
    assert!(t.contract.is_some());
    assert_eq!(t.contract.as_ref().unwrap().request.0, 0x2104_b12c);
    assert_eq!(t.sink_caps_sent, 1);
    assert_eq!(phy.tx.len(), 2); // 1 Request + 1 Sink_Capabilities
    let msg = Message::from_bytes(&phy.tx[1]).unwrap();
    assert!(matches!(
        msg.header.message_type(),
        MessageType::Data(
            usbpd::protocol_layer::message::header::DataMessageType::SinkCapabilities
        )
    ));
    assert_eq!(
        tx_header(&phy.tx[1]),
        0x6284,
        "6 Data Objects, PD3, Sink/UFP, ID1"
    );
    assert_eq!(phy.tx[1][2..], SINK_CAPS);
}

#[test]
fn maintained_ignores_partner_power_role_in_handshake_and_ready() {
    // Table 6.2: SOP power role must not be verified, even when incorrect.
    for cleared_roles in 0..8 {
        let mut replies = ready();
        for (index, reply) in replies.iter_mut().enumerate() {
            if cleared_roles & (1 << index) != 0 {
                reply[1] &= !1;
            }
        }
        let mut get_sink_cap = packet(8, 3, &[]);
        get_sink_cap[1] &= !1;
        replies.push(get_sink_cap);
        let mut phy = Fake::<true, true>::new(replies);
        let mut trial = Trial::maintained();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        {
            let mut future = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
            let mut cx = Context::from_waker(Waker::noop());
            for _ in 0..8 {
                assert!(future.as_mut().poll(&mut cx).is_pending());
            }
        }
        let trace = trace.borrow();
        assert_eq!(trace.contract.as_ref().unwrap().request.0, 0x2104_b12c);
        assert_eq!(trace.packets_serviced, 4);
        assert_eq!(trace.sink_caps_sent, 1);
        assert_eq!(phy.tx.len(), 2);
        for bytes in &phy.tx {
            assert_eq!(bytes[1] & 1, 0, "outgoing role remains Sink");
        }
        assert_eq!(phy.tx[1][2..], SINK_CAPS);
    }
}

#[test]
fn cancelled_session_revokes_work_and_fresh_session_forgets_old_state() {
    for pps in [false, true] {
        let selection = pd_spr::Selection::new();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let state = Rc::new(RefCell::new(InteractiveState {
            rx: VecDeque::from(ready()),
            tx: Vec::new(),
            idle: true,
            now: 0,
        }));
        let mut driver = Interactive {
            state: state.clone(),
            selection: &selection,
        };
        let mut trial = Trial::maintained();
        let mut engine =
            Box::pin(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        if pps {
            assert!(selection.preview(pd_protocol::pd_spr::Target {
                mv: 5000,
                ma: 1000,
                pps_object: 4
            }));
            assert!(selection.confirm());
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            state
                .borrow_mut()
                .rx
                .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            assert!(selection.preview(pd_protocol::pd_spr::Target {
                mv: 9000,
                ma: 100,
                pps_object: 0
            }));
            assert!(selection.confirm());
            selection.set_now_ms(100_000);
            assert!(selection.refresh_due());
        } else {
            assert!(selection.request_cable_mode(0x8087));
        }
        let transmitted = state.borrow().tx.len();
        drop(engine);
        assert_eq!(
            state.borrow().tx.len(),
            transmitted,
            "cancellation performs no I/O"
        );
        assert!(!selection.has_pending());
        assert!(!selection.refresh_due());
        assert!(!selection.pps_expired());
        assert_eq!(selection.option_count(), 0);
        assert_eq!(selection.cable_mode(), None);
        assert!(!selection.confirm());
        assert!(!selection.request_cable());
        assert!(
            trace.borrow().contract.is_some(),
            "retain historical wire evidence"
        );

        let state = Rc::new(RefCell::new(InteractiveState {
            rx: VecDeque::new(),
            tx: Vec::new(),
            idle: true,
            now: 0,
        }));
        let mut driver = Interactive {
            state: state.clone(),
            selection: &selection,
        };
        let mut trial = Trial::maintained(); // fresh logical owner, not transport recovery
        let mut engine =
            Box::pin(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert!(trace.borrow().contract.is_none());
        assert!(selection.established_target().is_none());
        assert_eq!(selection.target_mv(), 5000);
        assert_eq!(selection.target_pps_object(), 0);
        assert_eq!(selection.option_count(), 0);
        let mut replies = vec![
            packet(1, 0, &[fixed(5000, 3000), fixed(9000, 3000)]),
            packet(3, 1, &[]),
            packet(6, 2, &[]),
        ];
        for reply in &mut replies {
            reply[0] = (reply[0] & !0xc0) | 0x40;
        }
        state.borrow_mut().rx.extend(replies);
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 1);
        assert_eq!(
            u16::from_le_bytes(state.borrow().tx[0][..2].try_into().unwrap()),
            0x1042
        );
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x1104_b12c
        );
        assert!(
            !selection.request_cable(),
            "PD3 eligibility must not survive into the fresh PD2 session"
        );
    }
}

#[test]
fn initial_request_revision_survives_later_control_headers() {
    for wire_revision in [0u8, 1, 2] {
        let revision = wire_revision.max(1);
        let other = 3 - revision;
        let mut replies = vec![
            packet(1, 0, &[fixed(5000, 3000), fixed(9000, 3000)]),
            packet(3, 1, &[]),
            packet(6, 2, &[]),
            packet(10, 3, &[]),
            packet(5, 4, &[]),
            packet(8, 5, &[]),
        ];
        replies[0][0] = (replies[0][0] & !0xc0) | (wire_revision << 6);
        for reply in &mut replies[1..] {
            reply[0] = (reply[0] & !0xc0) | (other << 6);
        }
        let mut phy = Fake::<true, true>::new(replies);
        let mut trial = Trial::maintained();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        {
            let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
            assert!(
                engine
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending()
            );
        }
        assert_eq!(phy.tx.len(), if revision == 1 { 3 } else { 4 });
        for tx in &phy.tx {
            assert_eq!((tx[0] >> 6) & 3, revision);
        }
        assert_eq!(phy.tx[1][0] & 0x1f, if revision == 1 { 4 } else { 16 });
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x2104_b12c
        );
        assert_eq!(trace.borrow().sink_caps_sent, 1);
    }
}

#[test]
fn changed_source_capabilities_revision_cannot_renegotiate_a_session() {
    for revision in [1u8, 2] {
        let selection = pd_spr::Selection::new();
        let mut replies = ready();
        for reply in &mut replies {
            reply[0] = (reply[0] & !0xc0) | (revision << 6);
        }
        let mut changed = packet(1, 3, &[fixed(5000, 3000), 0xc0dc2164]);
        changed[0] = (changed[0] & !0xc0) | ((3 - revision) << 6);
        replies.push(changed);
        let mut phy = Fake::<true, true>::new(replies);
        let mut trial = Trial::maintained();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        assert!(matches!(
            complete(trial.run_selectable::<_, Never>(&mut phy, &trace, &selection)),
            Err(Error::UnsafeCapabilities)
        ));
        assert_eq!(phy.tx.len(), 1);
        assert_eq!((phy.tx[0][0] >> 6) & 3, revision);
        assert!(trace.borrow().contract.is_some());
    }
}

#[test]
fn maintained_unsupported_controls_use_revision_appropriate_response() {
    for revision in [1u8, 2] {
        for kind in [7, 10, 17, 18, 25, 31] {
            if revision == 2 && kind == 18 {
                continue;
            } // now a Status responder
            let mut replies = vec![
                packet(1, 0, &[fixed(5000, 3000), fixed(9000, 3000)]),
                packet(3, 1, &[]),
                packet(6, 2, &[]),
                packet(kind, 3, &[]),
                packet(kind, 3, &[]), // retransmission, not a second request
                packet(8, 4, &[]),
            ];
            for reply in &mut replies {
                reply[0] = (reply[0] & !0xc0) | (revision << 6);
            }
            let mut phy = Fake::<true, true>::new(replies);
            let mut trial = Trial::maintained();
            let trace = RefCell::new(pd_spr::MaintainedTrace::default());
            {
                let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
                assert!(
                    engine
                        .as_mut()
                        .poll(&mut Context::from_waker(Waker::noop()))
                        .is_pending()
                );
            }
            assert_eq!(phy.tx.len(), 3);
            let response = u16::from_le_bytes(phy.tx[1][..2].try_into().unwrap());
            assert_eq!(
                response,
                0x200 | (u16::from(revision) << 6) | if revision == 1 { 4 } else { 16 }
            );
            assert_eq!(
                trace.borrow().contract.as_ref().unwrap().request.0,
                0x2104_b12c
            );
            assert_eq!(trace.borrow().not_supported_sent, u32::from(revision == 2));
            assert_eq!(trace.borrow().sink_caps_sent, 1);
        }
    }
}

#[test]
fn dr_swap_and_vconn_swap_get_reject_at_either_revision_and_ready_continues() {
    // Section 7.12.1, PD2 V1.3 section 6.3.9: Accept, Wait or Reject; this UFP
    // keeps its Data Role. Section 7.13: not the VCONN Source in Ready, so Reject.
    // One Reject per swap request, none for a retransmission.
    for revision in [1u8, 2] {
        let (tx, trace, outcome) = after_contract::<Never>(
            revision,
            vec![
                packet(9, 3, &[]),
                packet(9, 3, &[]), // retransmission, not a second request
                packet(11, 4, &[]),
            ],
        );
        assert!(outcome.is_none(), "PD{}", revision + 1);
        assert_eq!(tx.len(), 4, "Request, Reject, Reject, Sink_Capabilities");
        for (reply, id) in [(&tx[1], 1u16), (&tx[2], 2)] {
            assert_eq!(
                tx_header(reply),
                id << 9 | u16::from(revision) << 6 | 4,
                "Reject ID{id}"
            );
        }
        assert_eq!(
            &tx[3][2..],
            if revision == 2 {
                &SINK_CAPS[..]
            } else {
                &SINK_CAPS_PD2[..]
            }
        );
        assert_eq!(trace.not_supported_sent, 0);
        assert_eq!(trace.soft_resets_sent, 0);
        assert_eq!(trace.contract.unwrap().request.0, 0x2104_b12c);
    }
}

#[test]
fn bist_test_data_at_vsafe5v_sends_nothing_until_hard_reset() {
    // Sections 6.4.3.1, 9.2.26.4.2: on the vSafe5V contract (object 1) BIST Test
    // Data stops every response, Soft_Reset handling and user Request.
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(trace.borrow().contract.as_ref().unwrap().request.0 >> 28, 1);
    let mut test_data = vec![0x8000_0000u32];
    test_data.extend([0x5555_5555; 6]);
    state.borrow_mut().rx.extend([
        packet(3, 3, &test_data),
        packet(3, 3, &test_data), // test frames keep the same MessageID
        packet(8, 4, &[]),        // Get_Sink_Cap
        packet(13, 0, &[]),       // Soft_Reset
        packet(6, 5, &[]),        // PS_RDY, Unexpected in Ready
    ]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(trace.borrow().bist_test_data);
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 9000,
        ma: 100,
        pps_object: 0
    }));
    assert!(selection.confirm());
    state.borrow_mut().now = 10_000;
    for _ in 0..4 {
        assert!(engine.as_mut().poll(&mut cx).is_pending());
    }
    assert_eq!(state.borrow().tx.len(), 1, "only the first Request");
    assert!(state.borrow().rx.is_empty());
    assert_eq!(trace.borrow().soft_resets_accepted, 0);
    state.borrow_mut().rx.push_back(Vec::new()); // Hard Reset Signaling ends the mode
    assert!(matches!(
        engine.as_mut().poll(&mut cx),
        Poll::Ready(Err(Error::HardResetReceived))
    ));
    assert_eq!(state.borrow().tx.len(), 1);
}

#[test]
fn bist_carrier_mode_at_vsafe5v_then_only_fresh_capabilities() {
    // Section 9.2.26.4.1.1: one carrier, then PE_SNK_Transition_to_default without
    // Hard Reset. Fresh Source_Capabilities renegotiate; any other Message fails
    // closed (no Shall Hard Reset applies).
    for fresh in [true, false] {
        let selection = pd_spr::Selection::new();
        let state = Rc::new(RefCell::new(InteractiveState {
            rx: ready().into(),
            tx: Vec::new(),
            idle: true,
            now: 0,
        }));
        let mut driver = Interactive {
            state: state.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        state
            .borrow_mut()
            .rx
            .push_back(packet(3, 3, &[0x5000_0000]));
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(state.borrow().tx.len(), 2);
        assert_eq!(state.borrow().tx[1], CARRIER);
        assert_eq!(trace.borrow().bist_carriers_sent, 1);
        assert!(!trace.borrow().bist_test_data);
        if fresh {
            // Fresh Source_Capabilities (MessageID 0 after the protocol reset).
            state.borrow_mut().rx.extend([
                ready()[0].clone(),
                packet(3, 1, &[]),
                packet(6, 2, &[]),
            ]);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            let tx = state.borrow().tx.clone();
            assert_eq!(tx.len(), 3, "Request, carrier, renegotiated Request");
            assert_eq!(u16::from_le_bytes([tx[2][0], tx[2][1]]) & 0xf01f, 0x1002);
            assert_eq!(
                (tx[2][1] >> 1) & 7,
                0,
                "MessageID 0 after the protocol reset"
            );
            assert_eq!(trace.borrow().contract.as_ref().unwrap().request.0 >> 28, 1);
        } else {
            state.borrow_mut().rx.push_back(packet(8, 4, &[])); // Get_Sink_Cap
            assert!(matches!(
                engine.as_mut().poll(&mut cx),
                Poll::Ready(Err(Error::Peer))
            ));
            assert_eq!(state.borrow().tx.len(), 2);
        }
        assert_eq!(
            trace.borrow().soft_resets_sent + trace.borrow().hard_resets_sent,
            0
        );
    }
}

#[test]
fn bist_carrier_mode_without_driver_support_is_acknowledged_only() {
    // A driver without carrier support (the trait default) sends nothing; Ready continues.
    let mut replies = ready();
    replies.extend([packet(3, 3, &[0x5000_0000]), packet(8, 4, &[])]);
    let mut phy = Fake::<true, true>::new(replies);
    let selection = pd_spr::Selection::new();
    let mut trial = Trial::maintained();
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut phy, &trace, &selection));
        assert!(
            engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending()
        );
    }
    assert_eq!(phy.tx.len(), 2, "Request, then Sink_Capabilities");
    assert_eq!(trace.borrow().bist_carriers_sent, 0);
    assert_eq!(trace.borrow().sink_caps_sent, 1);
}

// Extended Message with an explicit Extended Message Header and padded payload.
fn extended(kind: u16, id: u16, ext: u16, data: &[u8], chunked_objects: bool) -> Vec<u8> {
    let mut body = ext.to_le_bytes().to_vec();
    body.extend(data);
    if chunked_objects {
        body.resize(body.len().div_ceil(4) * 4, 0);
    }
    let objects = if chunked_objects {
        body.len() as u16 / 4
    } else {
        0
    };
    let header = 0x8000 | kind | 0x180 | id << 9 | objects << 12;
    let mut bytes = header.to_le_bytes().to_vec();
    bytes.extend(body);
    bytes
}
/// Contract, then `traffic`, then Get_Sink_Cap proving Ready still services.
fn after_contract<T: usbpd::timers::Timer>(
    revision: u8,
    traffic: Vec<Vec<u8>>,
) -> (
    Vec<Vec<u8>>,
    pd_spr::MaintainedTrace,
    Option<Result<(), Error>>,
) {
    let next = 3 + traffic.len() as u16;
    let mut replies = ready();
    replies.extend(traffic);
    replies.push(packet(8, next % 8, &[]));
    for reply in &mut replies {
        reply[0] = (reply[0] & !0xc0) | (revision << 6);
    }
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut outcome = None;
    {
        let mut engine = std::pin::pin!(trial.run_maintained::<_, T>(&mut phy, &trace));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..64 {
            if let Poll::Ready(result) = engine.as_mut().poll(&mut cx) {
                outcome = Some(result);
                break;
            }
        }
    }
    (phy.tx, trace.into_inner(), outcome)
}
fn tx_header(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}
/// Sink_Capabilities for the fixture's caller policy: fixed
/// 5 V (Higher Capability), 9, 12, 15, 20 V at the 5 A cable ceiling; under PD3 the
/// PPS APDO 5.0-21.0 V at 4.75 A (21 V x 4.75 A <= 100 W).
const SINK_CAPS: [u8; 24] = [
    0xf4, 0x91, 0x01, 0x10, 0xf4, 0xd1, 0x02, 0x00, 0xf4, 0xc1, 0x03, 0x00, 0xf4, 0xb1, 0x04, 0x00,
    0xf4, 0x41, 0x06, 0x00, 0x5f, 0x32, 0xa4, 0xc1,
];
const SINK_CAPS_PD2: [u8; 20] = [
    0xf4, 0x91, 0x01, 0x10, 0xf4, 0xd1, 0x02, 0x00, 0xf4, 0xc1, 0x03, 0x00, 0xf4, 0xb1, 0x04, 0x00,
    0xf4, 0x41, 0x06, 0x00,
];

#[test]
fn pd3_vdms_and_unsupported_data_get_not_supported_and_ready_continues() {
    for message in [
        packet(15, 3, &[0xff00_8001]), // Structured Discover Identity (DFP source)
        packet(15, 3, &[0x05ac_0000]), // Unstructured VDM
        packet(10, 3, &[0x0300_0000]), // EPR_Mode outside EPR Mode
        packet(7, 3, &[0x5553_0000]),  // Get_Country_Info
        packet(8, 3, &[0x2000_0000]),  // Enter_USB on a non-USB4 port
        packet(13, 3, &[0]),           // Reserved data type
    ] {
        let (tx, trace, outcome) = after_contract::<Never>(2, vec![message.clone()]);
        assert!(outcome.is_none(), "{message:02x?}");
        assert_eq!(tx.len(), 3, "{message:02x?}");
        assert_eq!(
            tx_header(&tx[1]),
            0x0290,
            "Not_Supported ID1 {message:02x?}"
        );
        assert_eq!(tx[2][2..], SINK_CAPS);
        assert_eq!(trace.not_supported_sent, 1);
        assert_eq!(trace.contract.unwrap().request.0, 0x2104_b12c);
    }
}

#[test]
fn pd3_alert_informs_policy_without_a_response() {
    // Table 6.25: PPS sources Shall Alert on CV<->CL operating-condition changes.
    let (tx, trace, outcome) = after_contract::<Never>(
        2,
        vec![
            packet(6, 3, &[0x1000_0000]),
            packet(6, 4, &[0x0400_0000, 0xdead_beef]),
            packet(6, 4, &[0x0400_0000, 0xdead_beef]), // retransmission
        ],
    );
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 2, "Request then Sink_Capabilities only");
    assert_eq!(tx[1][2..], SINK_CAPS);
    assert_eq!(trace.alerts_received, 2);
    assert_eq!(trace.last_alert, Some(0x0400_0000));
    assert_eq!(trace.not_supported_sent, 0);
}

#[test]
fn bist_and_received_not_supported_get_goodcrc_only() {
    let (tx, trace, outcome) = after_contract::<Never>(
        2,
        vec![
            packet(3, 3, &[0x5000_0000]), // BIST Carrier Mode
            packet(3, 4, &[0x8000_0000]), // BIST Test Data
            packet(16, 5, &[]),           // Not_Supported
        ],
    );
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 2);
    assert_eq!(trace.not_supported_sent, 0);
}

#[test]
fn pd3_get_revision_returns_revision_3_2_version_1_2() {
    let (tx, trace, outcome) =
        after_contract::<Never>(2, vec![packet(24, 3, &[]), packet(24, 3, &[])]);
    assert!(outcome.is_none());
    assert_eq!(
        tx.len(),
        3,
        "a retransmitted Get_Revision is not answered twice"
    );
    assert_eq!(tx_header(&tx[1]), 0x128c);
    assert_eq!(tx[1][2..], 0x3212_0000u32.to_le_bytes());
    assert_eq!(trace.revisions_sent, 1);
    assert_eq!(trace.not_supported_sent, 0);
}

#[test]
fn pd3_get_sink_cap_extended_returns_one_padded_skedb_chunk() {
    let (tx, trace, outcome) =
        after_contract::<Never>(2, vec![packet(22, 3, &[]), packet(22, 3, &[])]);
    assert!(outcome.is_none());
    assert_eq!(
        tx.len(),
        3,
        "a retransmitted Get_Sink_Cap_Extended is not answered twice"
    );
    assert_eq!(tx[1].len(), 30);
    assert_eq!(
        tx_header(&tx[1]),
        0xf28f,
        "Extended, 7 objects, ID1, PD3, Sink/UFP"
    );
    assert_eq!(tx[1][2..4], [0x18, 0x80], "Chunked, Chunk 0, Data Size 24");
    let mut skedb = [0u8; 24];
    skedb[0..2].copy_from_slice(&[0xff, 0xff]);
    skedb[10] = 1;
    skedb[17] = 0b11;
    skedb[18..21].copy_from_slice(&[1, 21, 100]); // Fixture: 5 V x 100 mA; 21 V PPS at 1 A / 5 A (capped)
    assert_eq!(tx[1][4..28], skedb);
    assert_eq!(tx[1][28..], [0, 0]);
    assert_eq!(trace.sink_caps_extended_sent, 1);
    assert_eq!(trace.not_supported_sent, 0);
}

#[test]
fn pd2_ignores_vdms_and_rejects_pd3_only_messages() {
    let (tx, trace, outcome) = after_contract::<Never>(
        1,
        vec![packet(15, 3, &[0xff00_8001]), packet(3, 4, &[0x5000_0000])],
    );
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 2, "PD2 V1.3: unsupported VDMs are Ignored");
    assert_eq!(trace.not_supported_sent, 0);
    for message in [
        packet(6, 3, &[0x1000_0000]),
        packet(22, 3, &[]),
        packet(24, 3, &[]),
    ] {
        let (tx, _, outcome) = after_contract::<Never>(1, vec![message]);
        assert!(outcome.is_none());
        assert_eq!(tx.len(), 3);
        assert_eq!(tx_header(&tx[1]), 0x0244, "PD2 Reject ID1");
    }
    // Extended Messages do not exist in PD2: still fail closed.
    let (tx, _, outcome) = after_contract::<Never>(1, vec![extended(3, 3, 0x8001, &[0], true)]);
    assert!(matches!(outcome, Some(Err(Error::Peer))));
    assert_eq!(tx.len(), 1);
}

#[test]
fn single_chunk_unchunked_and_stray_extended_messages_get_not_supported() {
    for message in [
        extended(3, 3, 0x8001, &[0], true), // Get_Battery_Cap, one Chunk
        extended(16, 3, 0x8002, &[2, 0], true), // EPR_Get_Sink_Cap: not EPR capable
        extended(17, 3, 0x8004, &[0; 4], true), // EPR_Source_Capabilities, unsolicited
        extended(16, 3, 0x0000, &[], false), // truncated ECDB: never a parser panic
        extended(3, 3, 0x0001, &[0], false), // unchunked (not negotiated)
        extended(30, 3, 0x8c00, &[0, 0], true), // stray Chunk request
    ] {
        let (tx, trace, outcome) = after_contract::<Never>(2, vec![message.clone()]);
        assert!(outcome.is_none(), "{message:02x?}");
        assert_eq!(tx.len(), 3, "{message:02x?}");
        assert_eq!(tx_header(&tx[1]), 0x0290, "{message:02x?}");
        assert_eq!(tx[2][2..], SINK_CAPS);
        assert_eq!(trace.not_supported_sent, 1);
    }
}

#[test]
fn multi_chunk_message_waits_for_chunking_not_supported_timer() {
    let first = extended(30, 3, 0x8000 | 40, &[0x5a; 26], true); // Chunk 0 of 2
    // The timer has not expired: no Chunk request and no early Not_Supported.
    let (tx, trace, outcome) = after_contract::<Never>(2, vec![first.clone()]);
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 1);
    assert_eq!(trace.not_supported_sent, 0);
    // After tChunkingNotSupported: exactly one Not_Supported, then Ready continues.
    let (tx, trace, outcome) = after_contract::<Ticks>(2, vec![first]);
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 3);
    assert_eq!(tx_header(&tx[1]), 0x0290);
    assert_eq!(tx[2][2..], SINK_CAPS);
    assert_eq!(trace.not_supported_sent, 1);
    // Table 6.48: an Invalid Chunk Number is Ignored.
    let (tx, trace, outcome) = after_contract::<Ticks>(
        2,
        vec![extended(30, 3, 0x8000 | 12 << 11 | 40, &[0; 2], true)],
    );
    assert!(outcome.is_none());
    assert_eq!(tx.len(), 2);
    assert_eq!(trace.not_supported_sent, 0);
}

#[test]
fn malformed_extended_framing_still_fails_closed() {
    for message in [
        extended(3, 3, 0x8001, &[], false), // chunked without Data Objects
        extended(3, 3, 0x0005, &[0], false), // unchunked size beyond payload
    ] {
        let (tx, _, outcome) = after_contract::<Never>(2, vec![message.clone()]);
        assert!(matches!(outcome, Some(Err(Error::Peer))), "{message:02x?}");
        assert_eq!(tx.len(), 1);
    }
}

#[test]
fn maintained_pd2_ping_has_no_policy_response() {
    let mut replies = ready();
    replies.push(packet(5, 3, &[]));
    replies.push(packet(8, 4, &[]));
    for reply in &mut replies {
        reply[0] = (reply[0] & !0xc0) | 0x40;
    }
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
        assert!(
            engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending()
        );
    }
    assert_eq!(phy.tx.len(), 2); // Request, then Sink_Capabilities; PHY handles GoodCRC.
    assert_eq!(trace.borrow().not_supported_sent, 0);
    assert_eq!(trace.borrow().sink_caps_sent, 1);
}

#[test]
fn maintained_services_ping_with_not_supported() {
    let mut replies = ready();
    replies.push(packet(5, 3, &[])); // Ping = 5
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = std::cell::RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut future = std::pin::pin!(trial.run_maintained::<_, Ticks>(&mut phy, &trace));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..64 {
            let _ = future.as_mut().poll(&mut cx);
        }
    }
    let t = trace.borrow();
    assert!(t.contract.is_some());
    assert_eq!(t.not_supported_sent, 1);
    assert_eq!(phy.tx.len(), 2); // 1 Request + 1 NotSupported
    let msg = Message::from_bytes(&phy.tx[1]).unwrap();
    assert!(matches!(
        msg.header.message_type(),
        MessageType::Control(ControlMessageType::NotSupported)
    ));
}

#[test]
fn maintained_services_source_capabilities_readvertisement() {
    let mut replies = ready();
    // Second Source_Capabilities announcement, still containing the 9 V object.
    replies.push(packet(1, 3, &[fixed(5000, 3000), fixed(9000, 3000)]));
    replies.push(packet(3, 4, &[])); // Accept
    replies.push(packet(6, 5, &[])); // PsRdy
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = std::cell::RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut future = std::pin::pin!(trial.run_maintained::<_, Ticks>(&mut phy, &trace));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..64 {
            let _ = future.as_mut().poll(&mut cx);
        }
    }
    let t = trace.borrow();
    assert!(t.contract.is_some());
    assert_eq!(t.re_requests_sent, 1);
    assert_eq!(phy.tx.len(), 2); // First Request + Second Request
    assert_eq!(&phy.tx[1][2..], &0x2104_b12cu32.to_le_bytes()); // Still the reviewed 9 V target!
}

#[test]
fn maintained_partner_soft_reset_reacquires_same_target_with_reset_ids() {
    for revision in [1, 2] {
        let mut replies = ready();
        replies.extend([
            packet(13, 0, &[]),
            packet(1, 1, &[fixed(5000, 3000), fixed(9000, 3000)]),
            packet(3, 2, &[]),
            packet(6, 3, &[]),
            packet(8, 4, &[]),
        ]);
        for reply in &mut replies {
            reply[0] = (reply[0] & !0xc0) | (revision << 6);
        }
        let mut phy = Fake::<true, true>::new(replies);
        let mut trial = Trial::maintained();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        {
            let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
            assert!(
                engine
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending()
            );
        }
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x2104_b12c
        );
        assert_eq!(phy.tx.len(), 4);
        assert_eq!(
            u16::from_le_bytes(phy.tx[1][..2].try_into().unwrap()),
            3 | (u16::from(revision) << 6)
        );
        let request_header = u16::from_le_bytes(phy.tx[2][..2].try_into().unwrap());
        assert_eq!((request_header >> 9) & 7, 1, "Accept consumes TX ID0");
        assert_eq!(&phy.tx[2][2..], &0x2104_b12cu32.to_le_bytes());
        assert_eq!(trace.borrow().sink_caps_sent, 1);
    }
}

#[test]
fn partner_soft_reset_preserves_pps_and_discards_queued_user_change() {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 5000,
        ma: 1000,
        pps_object: 4
    }));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    let established = trace.borrow().contract.as_ref().unwrap().request.0;
    state.borrow_mut().now = 2000;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    // Section 7.16: the new PPS contract's one Get_PPS_Status (tx[2]).
    state.borrow_mut().rx.push_back(pps_status(5));
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 9000,
        ma: 100,
        pps_object: 0
    }));
    assert!(selection.confirm());
    state.borrow_mut().rx.push_back(packet(13, 0, &[]));
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(state.borrow().tx.len(), 4);
    assert_eq!(&state.borrow().tx[3], &[0x83, 0]);
    assert!(selection.request_in_flight());
    assert!(!selection.has_pending());
    assert!(!selection.confirm());
    assert!(!selection.request_cable());
    assert_eq!(
        trace.borrow().contract.as_ref().unwrap().request.0,
        established
    );
    let mut capabilities = ready()[0].clone();
    capabilities[1] = (capabilities[1] & !0x0e) | 2; // source TX ID1 after Soft_Reset
    state
        .borrow_mut()
        .rx
        .extend([capabilities, packet(3, 2, &[]), packet(6, 3, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(state.borrow().tx.len(), 5);
    assert_eq!(
        u32::from_le_bytes(state.borrow().tx[4][2..6].try_into().unwrap()),
        established
    );
    assert_eq!(selection.target_pps_object(), 4);
    assert!(!selection.request_in_flight());
    state.borrow_mut().now = 5999;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(
        state.borrow().tx.len(),
        5,
        "same target after the reset: no new query"
    );
    state.borrow_mut().now = 6000;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(
        state.borrow().tx.len(),
        6,
        "PPS refresh anchored to the renegotiated request"
    );
}

#[test]
fn partner_soft_reset_failures_hard_reset_but_never_for_sink_wait_cap() {
    let mut wrong_revision = packet(13, 0, &[]);
    wrong_revision[0] = (wrong_revision[0] & !0xc0) | 0x40;
    for (after_contract, expected, expected_tx) in [
        (vec![packet(13, 1, &[])], 0, 1), // section 7.7: MessageID 0
        (vec![wrong_revision], 0, 1),
        // SinkWaitCapTimer: the optional Hard Reset is never sent (owner rule).
        (vec![packet(13, 0, &[])], 0, 2),
        // Section 9.2.5.2.2 "from any State": a repeated reset is answered again.
        (vec![packet(13, 0, &[]), packet(13, 0, &[])], 0, 3),
        // Section 7.7: not Source_Capabilities -> Hard Reset.
        (vec![packet(13, 0, &[]), packet(8, 1, &[])], 2, 3),
        (
            vec![packet(13, 0, &[]), packet(1, 1, &[fixed(5000, 3000)])],
            1,
            2,
        ),
    ] {
        let mut replies = ready();
        replies.extend(after_contract);
        let mut driver = Fake::<true, true>::new(replies);
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let result = complete(trial.run_maintained::<_, Ticks>(&mut driver, &trace));
        match expected {
            0 => assert!(matches!(result, Err(Error::RecoveryBlocked)), "{result:?}"),
            1 => assert!(matches!(result, Err(Error::UnsafeCapabilities))),
            _ => {
                assert!(matches!(result, Err(Error::HardResetSent)), "{result:?}");
                assert_eq!(driver.tx.last().unwrap(), &HARD_RESET);
                assert_eq!(trace.borrow().hard_resets_sent, 1);
            }
        }
        if expected != 2 {
            assert!(!driver.tx.contains(&HARD_RESET.to_vec()));
        }
        assert_eq!(driver.tx.len(), expected_tx);
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x2104_b12c
        );
        assert!(
            driver.tx.iter().all(|bytes| bytes[0] & 0x1f != 13),
            "nothing to correct: no Soft_Reset sent"
        );
    }
    // Section 9.2.5.2.2: an Accept that gets no GoodCRC needs Hard Reset.
    let mut replies = ready();
    replies.push(packet(13, 0, &[]));
    let mut driver = Fake::<true, true>::new(replies);
    driver.nack = vec![1];
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Never>(&mut driver, &trace));
    assert!(matches!(result, Err(Error::HardResetSent)), "{result:?}");
    assert_eq!(driver.tx.len(), 3);
    assert_eq!(driver.tx[2], HARD_RESET);
}

/// A selectable fixed 5 V session with a 9 V change confirmed (Request at tx[1]).
fn changing_to_9v(
    check: impl FnOnce(
        &pd_spr::Selection,
        &Rc<RefCell<InteractiveState>>,
        &RefCell<pd_spr::MaintainedTrace>,
        &mut dyn FnMut() -> Poll<Result<(), Error>>,
    ),
) {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    let mut poll = || engine.as_mut().poll(&mut cx);
    assert!(poll().is_pending());
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 9000,
        ma: 100,
        pps_object: 0
    }));
    assert!(selection.confirm());
    assert!(poll().is_pending());
    assert_eq!(state.borrow().tx.len(), 2);
    check(&selection, &state, &trace, &mut poll);
}
/// Source_Capabilities, Accept, PS_RDY with MessageIDs 1..3 after a Soft Reset.
fn reacquired() -> [Vec<u8>; 3] {
    let mut capabilities = ready()[0].clone();
    capabilities[1] = (capabilities[1] & !0x0e) | 2;
    [capabilities, packet(3, 2, &[]), packet(6, 3, &[])]
}

#[test]
fn partner_soft_reset_during_a_power_request_or_transition_is_accepted() {
    // Section 9.2.5.2.2 (D1): before Accept, and after Accept before PS_RDY.
    for accepted in [false, true] {
        changing_to_9v(|selection, state, trace, poll| {
            if accepted {
                state.borrow_mut().rx.push_back(packet(3, 3, &[]));
                assert!(poll().is_pending());
            }
            state.borrow_mut().rx.push_back(packet(13, 0, &[]));
            assert!(poll().is_pending());
            assert_eq!(state.borrow().tx.len(), 3);
            assert_eq!(&state.borrow().tx[2], &[0x83, 0], "Accept, MessageID 0");
            assert!(selection.request_in_flight(), "confirmations stay blocked");
            assert!(!selection.confirm());
            // D3: the established 5 V target is requested again; 9 V needs a new confirmation.
            state.borrow_mut().rx.extend(reacquired());
            assert!(poll().is_pending());
            assert_eq!(state.borrow().tx.len(), 4);
            assert_eq!(message_id(&state.borrow().tx[3]), 1);
            assert_eq!(
                u32::from_le_bytes(state.borrow().tx[3][2..6].try_into().unwrap()),
                0x1104_b12c
            );
            assert!(!selection.request_in_flight());
            assert_eq!(selection.target_mv(), 5000);
            assert_eq!(trace.borrow().soft_resets_accepted, 1);
            assert_eq!(trace.borrow().soft_resets_sent, 0);
        });
    }
}

#[test]
fn startup_soft_reset_accepts_before_capabilities_without_locking_revision() {
    for reset_revision in [0u16, 1, 2] {
        for caps_revision in [1u16, 2] {
            let wire = |kind, id, objects: &[u32], revision| {
                let mut bytes = packet(kind, id, objects);
                bytes[0] = (bytes[0] & !0xc0) | (revision << 6) as u8;
                bytes
            };
            let replies = vec![
                wire(13, 0, &[], reset_revision),
                wire(1, 1, &[fixed(5000, 3000)], caps_revision),
                wire(3, 2, &[], caps_revision),
                wire(6, 3, &[], caps_revision),
            ];
            let mut phy = Fake::<true, true>::new(replies);
            let trace = RefCell::new(pd_spr::MaintainedTrace::default());
            let selection = pd_spr::Selection::new();
            let mut trial = Trial::maintained();
            {
                let mut engine =
                    std::pin::pin!(trial.run_selectable::<_, Never>(&mut phy, &trace, &selection));
                assert!(
                    engine
                        .as_mut()
                        .poll(&mut Context::from_waker(Waker::noop()))
                        .is_pending()
                );
            }
            assert_eq!(
                phy.tx.len(),
                2,
                "reset {reset_revision}, caps {caps_revision}"
            );
            assert_eq!(phy.tx[0], [3 | (reset_revision.max(1) as u8) << 6, 0]);
            assert_eq!(message_id(&phy.tx[1]), 1);
            assert_eq!((phy.tx[1][0] >> 6) & 3, caps_revision as u8);
            assert!(trace.borrow().contract.is_some());
            assert_eq!(trace.borrow().soft_resets_accepted, 1);
        }
    }
}

#[test]
fn malformed_startup_soft_reset_and_once_mode_do_not_transmit() {
    for reset in [
        packet(13, 1, &[]),
        vec![0xcd, 1],
        packet(13, 0, &[0]),
        vec![0x8d],
    ] {
        let mut phy = Fake::<true, true>::new(vec![reset]);
        let selection = pd_spr::Selection::new();
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        assert!(complete(trial.run_selectable::<_, Never>(&mut phy, &trace, &selection)).is_err());
        assert!(phy.tx.is_empty());
    }
    let mut phy = Fake::<true, true>::new(vec![packet(13, 0, &[])]);
    assert!(complete(Trial::once().run::<_, Never>(&mut phy)).is_err());
    assert!(phy.tx.is_empty());
}

#[test]
fn startup_reset_accept_failure_hard_resets_but_capability_timeout_does_not() {
    for (failed_accept, unexpected) in [(true, false), (false, true), (false, false)] {
        let mut phy = Fake::<true, true>::new(vec![packet(13, 0, &[])]);
        if failed_accept {
            phy.nack.push(0);
        }
        if unexpected {
            phy.rx.push_back(packet(5, 1, &[]));
        }
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let mut trial = Trial::maintained();
        let result = complete(trial.run_maintained::<_, Ticks>(&mut phy, &trace));
        assert_eq!(phy.tx[0], [0x83, 0]);
        if failed_accept || unexpected {
            assert!(matches!(result, Err(Error::HardResetSent)));
            assert_eq!(phy.tx[1], HARD_RESET);
            assert_eq!(trace.borrow().hard_resets_sent, 1);
        } else {
            assert!(result.is_err());
            assert_eq!(phy.tx.len(), 1, "no optional SinkWaitCapTimer Hard Reset");
            assert_eq!(trace.borrow().hard_resets_sent, 0);
        }
    }
}

#[test]
fn partner_soft_reset_during_the_initial_request_is_accepted() {
    let mut replies = ready();
    replies.truncate(1);
    replies.push(packet(13, 0, &[]));
    replies.extend(reacquired());
    replies.push(packet(8, 4, &[]));
    let mut phy = Fake::<true, true>::new(replies);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut trial = Trial::maintained();
        let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
        assert!(
            engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending()
        );
    }
    assert_eq!(phy.tx.len(), 4);
    assert_eq!(&phy.tx[1], &[0x83, 0]);
    assert_eq!(message_id(&phy.tx[2]), 1);
    assert_eq!(phy.tx[3][2..], SINK_CAPS);
    assert_eq!(
        trace.borrow().contract.as_ref().unwrap().request.0,
        0x2104_b12c
    );
}

#[test]
fn maintained_blocks_unsafe_readvertisement_without_transmit() {
    let mut replies = ready();
    // Re-advertisement without the reviewed target object (5 V and 20 V only)
    replies.push(packet(1, 3, &[fixed(5000, 3000), fixed(20000, 3000)]));
    let mut phy = Fake::<true, true>::new(replies);
    let mut trial = Trial::maintained();
    let trace = std::cell::RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(trial.run_maintained::<_, Ticks>(&mut phy, &trace));
    assert!(matches!(result, Err(Error::UnsafeCapabilities)));
    assert_eq!(phy.tx.len(), 1); // Only initial 5V Request, no second request or reset
}

#[test]
fn pps_current_adjustment_and_voltage_stepping_require_confirmation() {
    let advertisement = packet(1, 0, &[fixed(5000, 3000), 0xc0dc2164]); // 3.3..11V, 5.0A
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: VecDeque::from([advertisement, packet(3, 1, &[]), packet(6, 2, &[])]),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());

    // Cycle to PPS
    selection.advance(20, 50);
    assert!(selection.candidate_is_pps());
    assert_eq!(selection.candidate_mv(), 5000);
    assert_eq!(selection.candidate_ma(), 5000);
    assert!(!selection.adjusting_current());

    // Explicitly toggle adjustment; no confirmation is implied.
    selection.toggle_adjustment();
    assert!(selection.adjusting_current());
    assert!(!selection.has_pending());

    // Wrap from 5000 to 1000 mA.
    selection.advance(20, 50);
    assert_eq!(selection.candidate_ma(), 1000);

    // Step +50 mA to 1050 mA.
    selection.advance(20, 50);
    assert_eq!(selection.candidate_ma(), 1050);

    // Toggle back to voltage adjustment and step to 5020 mV.
    selection.toggle_adjustment();
    assert!(!selection.adjusting_current());
    selection.advance(20, 50);
    assert_eq!(selection.candidate_mv(), 5020);
    assert_eq!(selection.candidate_ma(), 1050);

    // Explicitly confirm 5020 mV @ 1050 mA.
    assert!(selection.confirm());
    assert!(selection.has_pending());
    assert!(engine.as_mut().poll(&mut cx).is_pending());

    let rdo = |bytes: &[u8]| u32::from_le_bytes(bytes[2..6].try_into().unwrap());
    // object 2, 5020 mV (251 units = 0xfb), 1050 mA (21 units = 0x15)
    // RDO: (2 << 28) | (1 << 24) | (251 << 9) | 21 = 0x2101_f615
    assert_eq!(rdo(&state.borrow().tx[1]), 0x2101_f615);
}

// ---- SinkTxOK collision avoidance: Deferred AMS starts (sections 7.2, 7.3) ----

// The PHY reports a Request it did not send as Deferred (SinkTxNG, or a SOP
// Message arrived first). Nothing reaches the wire or the peer script.
struct Deferring<'a> {
    inner: Interactive<'a>,
    defer: Rc<std::cell::Cell<usize>>,
    attempts: Rc<RefCell<Vec<Vec<u8>>>>,
    /// Control/Data Message type that is deferred (2 = Request).
    kind: u8,
}
impl Driver for Deferring<'_> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        panic!("reset")
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        if bytes[0] & 0x1f == self.kind && bytes[1] & 0x80 == 0 && self.defer.get() > 0 {
            self.defer.set(self.defer.get() - 1);
            self.attempts.borrow_mut().push(bytes.to_vec());
            return Err(DriverTxError::Deferred);
        }
        self.inner.transmit(bytes).await
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        self.inner.receive(buffer).await
    }
}
fn message_id(bytes: &[u8]) -> u8 {
    (bytes[1] >> 1) & 7
}

#[test]
fn deferred_response_request_is_never_accepted_as_collision_avoidance() {
    // The Request answering Source_Capabilities belongs to the Source's AMS.
    let mut phy = Fake::<true, true>::new(ready());
    phy.tx_error = Some(DriverTxError::Deferred);
    assert!(matches!(
        complete(Trial::once().run::<_, Ticks>(&mut phy)),
        Err(Error::Transmit(DriverTxError::Deferred))
    ));
    let mut phy = Fake::<true, true>::new(ready());
    phy.tx_error = Some(DriverTxError::Deferred);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    assert!(matches!(
        complete(Trial::maintained().run_maintained::<_, Never>(&mut phy, &trace)),
        Err(Error::Transmit(DriverTxError::Deferred))
    ));
    assert!(trace.borrow().contract.is_none());
    assert_eq!(phy.tx.len(), 1);
}

#[test]
fn deferred_user_change_keeps_contract_and_needs_new_confirmation() {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let defer = Rc::new(std::cell::Cell::new(0));
    let attempts = Rc::new(RefCell::new(Vec::new()));
    let mut driver = Deferring {
        inner: Interactive {
            state: state.clone(),
            selection: &selection,
        },
        defer: defer.clone(),
        attempts: attempts.clone(),
        kind: 2,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    let established = trace.borrow().contract.as_ref().unwrap().request.0;
    defer.set(1); // the next Request starts a Sink-initiated AMS
    let nine = pd_protocol::pd_spr::Target {
        mv: 9000,
        ma: 100,
        pps_object: 0,
    };
    assert!(selection.preview(nine));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(attempts.borrow().len(), 1);
    assert_eq!(trace.borrow().requests_deferred, 1);
    assert_eq!(trace.borrow().re_requests_sent, 0);
    assert_eq!(
        state.borrow().tx.len(),
        1,
        "only the initial Request reached the wire"
    );
    assert_eq!(
        trace.borrow().contract.as_ref().unwrap().request.0,
        established
    );
    assert_eq!(selection.target_mv(), 5000);
    assert!(!selection.request_in_flight());
    // Same 100 ms cooldown as Wait; the change is not resent by itself.
    state.borrow_mut().now = 99;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(!selection.confirm());
    state.borrow_mut().now = 100;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(state.borrow().tx.len(), 1);
    assert!(selection.preview(nine));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(state.borrow().tx.len(), 2);
    // Nothing went on the wire, so the retry reuses the deferred MessageID.
    assert_eq!(
        message_id(&state.borrow().tx[1]),
        message_id(&attempts.borrow()[0])
    );
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(selection.target_mv(), 9000);
    assert_eq!(trace.borrow().re_requests_sent, 1);
}

#[test]
fn deferred_pps_refresh_stays_due_and_is_retried_after_the_cooldown() {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let defer = Rc::new(std::cell::Cell::new(0));
    let attempts = Rc::new(RefCell::new(Vec::new()));
    let mut driver = Deferring {
        inner: Interactive {
            state: state.clone(),
            selection: &selection,
        },
        defer: defer.clone(),
        attempts: attempts.clone(),
        kind: 2,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(selection.preview(pd_protocol::pd_spr::Target {
        mv: 5000,
        ma: 1000,
        pps_object: 4
    }));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    let established = trace.borrow().contract.as_ref().unwrap().request.0;
    assert_eq!(state.borrow().tx.len(), 2);
    // The refresh comes due; the PHY defers it once.
    defer.set(1);
    let due = pd_spr::PPS_REFRESH_MS;
    state.borrow_mut().now = due;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(attempts.borrow().len(), 1);
    assert_eq!(
        u32::from_le_bytes(attempts.borrow()[0][2..6].try_into().unwrap()),
        established
    );
    assert_eq!(state.borrow().tx.len(), 2);
    assert_eq!(trace.borrow().requests_deferred, 1);
    assert_eq!(
        trace.borrow().contract.as_ref().unwrap().request.0,
        established
    );
    assert_eq!(selection.target_pps_object(), 4, "the PPS contract is kept");
    state.borrow_mut().now = due + 99;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(state.borrow().tx.len(), 2, "cooldown");
    state.borrow_mut().now = due + 100;
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(
        state.borrow().tx.len(),
        3,
        "still due: retried without user action"
    );
    let sent = state.borrow().tx[2].clone();
    assert_eq!(
        u32::from_le_bytes(sent[2..6].try_into().unwrap()),
        established
    );
    assert_eq!(message_id(&sent), message_id(&attempts.borrow()[0]));
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 5, &[]), packet(6, 6, &[])]);
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(trace.borrow().re_requests_sent, 2);
}

// ---- Get_PPS_Status (sections 6.3.20, 6.5.13, 7.16, 9.2.11.3) ----

thread_local! {
    static SENDER_RESPONSE_EXPIRES: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}
/// Once armed, only SenderResponseTimer (30 ms upstream) expires; others pend.
struct SenderResponseOnly;
impl usbpd::timers::Timer for SenderResponseOnly {
    async fn after_millis(ms: u64) {
        if ms == 30 && SENDER_RESPONSE_EXPIRES.get() {
            Ticks::after_millis(ms).await
        } else {
            pending::<()>().await
        }
    }
}
const PPS_5V: pd_protocol::pd_spr::Target = pd_protocol::pd_spr::Target {
    mv: 5000,
    ma: 1000,
    pps_object: 4,
};
/// PPS_Status: Chunked, Chunk 0, Data Size 4; 9.00 V, 2.00 A, PTF normal, OMF CL.
fn pps_status(id: u16) -> Vec<u8> {
    extended(12, id, 0x8004, &[0xc2, 0x01, 0x28, 0x0a], true)
}
fn is_get_pps_status(bytes: &[u8]) -> bool {
    bytes.len() == 2 && tx_header(bytes) & 0xf1ff == pd_protocol::pd_spr::GET_PPS_STATUS_HEADER
}

/// Establish the PPS contract (Request at tx[1], Accept/PS_RDY IDs 3/4) at time 0.
fn pps_session<T: usbpd::timers::Timer>(
    defer_kind: u8,
    check: impl FnOnce(
        &pd_spr::Selection,
        &Rc<RefCell<InteractiveState>>,
        &RefCell<pd_spr::MaintainedTrace>,
        &Rc<std::cell::Cell<usize>>,
        &mut dyn FnMut() -> Poll<Result<(), Error>>,
    ),
) {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let defer = Rc::new(std::cell::Cell::new(0));
    let mut driver = Deferring {
        inner: Interactive {
            state: state.clone(),
            selection: &selection,
        },
        defer: defer.clone(),
        attempts: Rc::new(RefCell::new(Vec::new())),
        kind: defer_kind,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine = std::pin::pin!(trial.run_selectable::<_, T>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    let mut poll = || engine.as_mut().poll(&mut cx);
    assert!(poll().is_pending());
    assert!(selection.preview(PPS_5V));
    assert!(selection.confirm());
    assert!(poll().is_pending());
    state
        .borrow_mut()
        .rx
        .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
    assert!(poll().is_pending());
    assert!(selection.established_target() == Some(PPS_5V));
    assert_eq!(state.borrow().tx.len(), 2);
    check(&selection, &state, &trace, &defer, &mut poll);
}

/// Advance the scripted clock to `now`, poll once, return the transmit count.
fn at(
    state: &Rc<RefCell<InteractiveState>>,
    poll: &mut dyn FnMut() -> Poll<Result<(), Error>>,
    now: u64,
) -> usize {
    state.borrow_mut().now = now;
    assert!(poll().is_pending());
    state.borrow().tx.len()
}

#[test]
fn pps_status_is_queried_once_per_new_pps_contract_after_it_settles() {
    pps_session::<Never>(2, |selection, state, trace, _, poll| {
        assert_eq!(
            at(state, poll, pd_spr::PPS_STATUS_SETTLE_MS - 1),
            2,
            "not before the output settles"
        );
        assert_eq!(at(state, poll, pd_spr::PPS_STATUS_SETTLE_MS), 3);
        assert!(is_get_pps_status(&state.borrow().tx[2]));
        assert_eq!((tx_header(&state.borrow().tx[2]) >> 6) & 3, 2, "PD3 only");
        state.borrow_mut().rx.push_back(pps_status(5));
        assert!(poll().is_pending());
        assert_eq!(
            selection.pps_status(),
            Some(pd_spr::PpsStatus {
                mv: Some(9000),
                ma: Some(2000),
                ptf: 1,
                current_limit: true,
            })
        );
        assert_eq!(trace.borrow().pps_status_received, 1);
        // Keep-alive refreshes of the same target do not query again.
        assert_eq!(at(state, poll, pd_spr::PPS_REFRESH_MS), 4);
        state
            .borrow_mut()
            .rx
            .extend([packet(3, 6, &[]), packet(6, 7, &[])]);
        assert!(poll().is_pending());
        assert_eq!(at(state, poll, pd_spr::PPS_REFRESH_MS + 1000), 4);
        // A new PPS target is a new contract: one more query after it settles.
        let changed = pd_protocol::pd_spr::Target { mv: 5100, ..PPS_5V };
        assert!(selection.preview(changed));
        assert!(selection.confirm());
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 5);
        state
            .borrow_mut()
            .rx
            .extend([packet(3, 0, &[]), packet(6, 1, &[])]);
        let changed_at = pd_spr::PPS_REFRESH_MS + 1000;
        assert_eq!(at(state, poll, changed_at), 5, "PS_RDY");
        assert_eq!(
            at(state, poll, changed_at + pd_spr::PPS_STATUS_SETTLE_MS - 1),
            5
        );
        assert_eq!(
            at(state, poll, changed_at + pd_spr::PPS_STATUS_SETTLE_MS),
            6
        );
        assert!(is_get_pps_status(&state.borrow().tx[5]));
    });
}

#[test]
fn pps_status_not_supported_is_recorded_and_not_asked_again() {
    pps_session::<Never>(2, |selection, state, trace, _, poll| {
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
        assert!(poll().is_pending());
        state.borrow_mut().rx.push_back(packet(16, 5, &[]));
        assert!(poll().is_pending());
        assert_eq!(trace.borrow().pps_status_not_supported, 1);
        assert_eq!(selection.pps_status(), None);
        state.borrow_mut().now = 3000;
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 3);
        assert!(selection.established_target() == Some(PPS_5V));
    });
}

#[test]
fn pps_status_timeout_returns_to_ready_and_a_late_reply_gets_not_supported() {
    pps_session::<SenderResponseOnly>(2, |selection, state, trace, _, poll| {
        SENDER_RESPONSE_EXPIRES.set(true);
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
        assert!(poll().is_pending());
        assert!(poll().is_pending());
        SENDER_RESPONSE_EXPIRES.set(false);
        assert_eq!(
            trace.borrow().pps_status_timeouts,
            1,
            "Figure 9.37: timeout -> Ready"
        );
        assert!(selection.established_target() == Some(PPS_5V));
        // A PPS_Status after the timeout is an unexpected Message in Ready.
        state.borrow_mut().rx.push_back(pps_status(5));
        assert!(poll().is_pending());
        let tx = state.borrow().tx.clone();
        assert_eq!(tx.len(), 4);
        assert_eq!(tx_header(&tx[3]) & 0xf01f, 16, "Not_Supported");
        state.borrow_mut().now = 3000;
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 4, "not asked again");
    });
}

#[test]
fn pps_status_malformed_reply_fails_closed() {
    // D2: a PPS_Status without the exact one-Chunk PPSSDB, or the wrong revision.
    let mut pd2 = pps_status(5);
    pd2[0] = (pd2[0] & !0xc0) | 0x40;
    for reply in [
        extended(12, 5, 0x0004, &[0xc2, 0x01, 0x28, 0x0a], false), // unchunked
        extended(12, 5, 0x8804, &[0xc2, 0x01, 0x28, 0x0a], true),  // Chunk 1
        extended(12, 5, 0x8404, &[0xc2, 0x01, 0x28, 0x0a], true),  // Chunk request
        extended(12, 5, 0x8003, &[0xc2, 0x01, 0x28], true),        // Data Size 3
        extended(12, 5, 0x8008, &[0; 8], true),                    // Data Size 8
        pd2,
    ] {
        pps_session::<Never>(2, |selection, state, trace, _, poll| {
            state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
            assert!(poll().is_pending());
            state.borrow_mut().rx.push_back(reply.clone());
            assert!(
                matches!(poll(), Poll::Ready(Err(Error::Peer))),
                "{reply:02x?}"
            );
            assert_eq!(state.borrow().tx.len(), 3, "no reset, no response");
            assert_eq!(selection.pps_status(), None);
            assert_eq!(trace.borrow().pps_status_received, 0);
        });
    }
    // Extended Header bit 9 is Reserved and ignored.
    pps_session::<Never>(2, |selection, state, _, _, poll| {
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
        assert!(poll().is_pending());
        state
            .borrow_mut()
            .rx
            .push_back(extended(12, 5, 0x8204, &[0xc2, 0x01, 0x28, 0x0a], true));
        assert!(poll().is_pending());
        assert!(selection.pps_status().is_some());
    });
}

#[test]
fn pps_status_unexpected_reply_soft_resets_and_keeps_the_pps_contract() {
    // Section 9.2.5.2.1: a well-formed but Unexpected Message during the AMS.
    for reply in [
        packet(3, 5, &[]),                  // Accept
        packet(1, 5, &[fixed(5000, 3000)]), // Source_Capabilities
        packet(6, 5, &[0x1000_0000]),       // Alert
        extended(2, 5, 0x8001, &[0], true), // Status
    ] {
        pps_session::<Never>(2, |selection, state, trace, _, poll| {
            assert_eq!(at(state, poll, pd_spr::PPS_STATUS_SETTLE_MS), 3);
            state.borrow_mut().rx.push_back(reply.clone());
            assert!(poll().is_pending(), "{reply:02x?}");
            assert_eq!(state.borrow().tx.len(), 4);
            assert_eq!(
                &state.borrow().tx[3],
                &[0x8d, 0],
                "Soft_Reset, MessageID 0, PD3"
            );
            assert_eq!(trace.borrow().soft_resets_sent, 1);
            let established = trace.borrow().contract.as_ref().unwrap().request.0;
            state.borrow_mut().rx.push_back(packet(3, 0, &[]));
            state.borrow_mut().rx.extend(reacquired());
            assert!(poll().is_pending());
            assert_eq!(state.borrow().tx.len(), 5);
            assert_eq!(message_id(&state.borrow().tx[4]), 1);
            assert_eq!(
                u32::from_le_bytes(state.borrow().tx[4][2..6].try_into().unwrap()),
                established
            );
            assert!(selection.established_target() == Some(PPS_5V));
            assert!(!selection.request_in_flight());
            // The interrupted query is not repeated for this contract.
            assert_eq!(at(state, poll, 2 * pd_spr::PPS_STATUS_SETTLE_MS), 5);
            assert_eq!(
                trace.borrow().pps_status_received + trace.borrow().pps_status_timeouts,
                0
            );
        });
    }
    // The partner's own Soft_Reset during the query is answered with Accept.
    pps_session::<Never>(2, |_, state, trace, _, poll| {
        assert_eq!(at(state, poll, pd_spr::PPS_STATUS_SETTLE_MS), 3);
        state.borrow_mut().rx.push_back(packet(13, 0, &[]));
        assert!(poll().is_pending());
        assert_eq!(&state.borrow().tx[3], &[0x83, 0]);
        assert_eq!(trace.borrow().soft_resets_accepted, 1);
    });
}

#[test]
fn deferred_pps_status_is_retried_after_cooldown_at_most_three_times() {
    pps_session::<Never>(0x14, |selection, state, trace, defer, poll| {
        defer.set(usize::MAX);
        let settle = pd_spr::PPS_STATUS_SETTLE_MS;
        for (now, deferred) in [
            (settle, 1),
            (settle + 99, 1),
            (settle + 100, 2),
            (settle + 200, 3),
            (settle + 1000, 3),
        ] {
            state.borrow_mut().now = now;
            assert!(poll().is_pending());
            assert_eq!(trace.borrow().pps_status_deferred, deferred, "at {now} ms");
        }
        assert_eq!(state.borrow().tx.len(), 2, "nothing reached the wire");
        assert!(selection.established_target() == Some(PPS_5V));
    });
    // One deferral, then the retry is sent and answered.
    pps_session::<Never>(0x14, |selection, state, trace, defer, poll| {
        defer.set(1);
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS;
        assert!(poll().is_pending());
        state.borrow_mut().now = pd_spr::PPS_STATUS_SETTLE_MS + 100;
        assert!(poll().is_pending());
        assert!(is_get_pps_status(&state.borrow().tx[2]));
        assert_eq!(
            message_id(&state.borrow().tx[2]),
            2,
            "the deferred attempt used no MessageID"
        );
        state.borrow_mut().rx.push_back(pps_status(5));
        assert!(poll().is_pending());
        assert_eq!(trace.borrow().pps_status_deferred, 1);
        assert!(selection.pps_status().is_some());
    });
}

#[test]
fn fixed_contracts_never_query_pps_status() {
    let selection = pd_spr::Selection::new();
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: ready().into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let mut driver = Interactive {
        state: state.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Never>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    for now in [0, 300, 5000] {
        state.borrow_mut().now = now;
        assert!(engine.as_mut().poll(&mut cx).is_pending());
    }
    assert_eq!(state.borrow().tx.len(), 1);
    assert!(!selection.pps_status_due());
}

#[test]
fn pps_status_block_decodes_unsupported_fields_and_ignores_reserved_bits() {
    let status = pd_spr::PpsStatus::decode([0xff, 0xff, 0xff, 0xf1]);
    assert_eq!(
        status,
        pd_spr::PpsStatus {
            mv: None,
            ma: None,
            ptf: 0,
            current_limit: false
        }
    );
    let status = pd_spr::PpsStatus::decode([0xfe, 0xff, 0xfe, 0x0e]);
    assert_eq!(
        status,
        pd_spr::PpsStatus {
            mv: Some(0xfffe * 20),
            ma: Some(0xfe * 50),
            ptf: 3,
            current_limit: true
        }
    );
}

// ---- Initiated Soft Reset (docs/PD-SOFT-RESET.md, sections 7.1.1, 9.2.5.2.1) ----

/// Contract, `traffic`, then whatever follows; returns transmissions, trace, outcome.
fn maintained<T: usbpd::timers::Timer>(
    revision: u8,
    traffic: Vec<Vec<u8>>,
    nack: Vec<usize>,
) -> (
    Vec<Vec<u8>>,
    pd_spr::MaintainedTrace,
    Option<Result<(), Error>>,
) {
    let mut replies = ready();
    replies.extend(traffic);
    for reply in &mut replies {
        reply[0] = (reply[0] & !0xc0) | (revision << 6);
    }
    let mut phy = Fake::<true, true>::new(replies);
    phy.nack = nack;
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut outcome = None;
    {
        let mut trial = Trial::maintained();
        let mut engine = std::pin::pin!(trial.run_maintained::<_, T>(&mut phy, &trace));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..64 {
            if let Poll::Ready(result) = engine.as_mut().poll(&mut cx) {
                outcome = Some(result);
                break;
            }
        }
    }
    (phy.tx, trace.into_inner(), outcome)
}
fn soft_reset_header(revision: u8) -> [u8; 2] {
    [13 | revision << 6, 0]
}

#[test]
fn unexpected_accept_reject_ps_rdy_or_wait_in_ready_soft_resets_and_renegotiates() {
    for revision in [1, 2] {
        for kind in [3, 4, 6, 12] {
            let mut traffic = vec![packet(kind, 3, &[]), packet(3, 0, &[])];
            traffic.extend(reacquired());
            traffic.push(packet(8, 4, &[]));
            let (tx, trace, outcome) = maintained::<Never>(revision, traffic, vec![]);
            assert!(outcome.is_none(), "{kind} {outcome:?}");
            assert_eq!(tx.len(), 4, "kind {kind}");
            assert_eq!(tx[1], soft_reset_header(revision), "Table 7.1, kind {kind}");
            assert_eq!(message_id(&tx[2]), 1, "Soft_Reset consumed TX ID0");
            assert_eq!(&tx[2][2..], &0x2104_b12cu32.to_le_bytes(), "same target");
            // APDOs do not exist in PD2: fixed objects only.
            let expected: &[u8] = if revision == 1 {
                &SINK_CAPS_PD2
            } else {
                &SINK_CAPS
            };
            assert_eq!(tx[3][2..], *expected, "Ready serviced again");
            assert_eq!(
                tx_header(&tx[3]) & 0xf1ff,
                ((expected.len() as u16 / 4) << 12) | (u16::from(revision) << 6) | 4
            );
            assert_eq!(trace.soft_resets_sent, 1);
            assert_eq!(trace.contract.unwrap().request.0, 0x2104_b12c);
        }
    }
}

#[test]
fn unexpected_message_during_a_request_soft_resets() {
    // PE_SNK_Select_Capability: anything but Accept/Reject/Wait (Figure 9.18 note 1).
    for unexpected in [
        packet(8, 1, &[]),                  // Get_Sink_Cap
        packet(1, 1, &[fixed(5000, 3000)]), // Source_Capabilities
        packet(6, 1, &[0x1000_0000]),       // Alert
        packet(31, 1, &[]),                 // Reserved Control type
        extended(2, 1, 0x8001, &[0], true), // Status
    ] {
        let mut replies = ready();
        replies.truncate(1);
        replies.extend([unexpected.clone(), packet(3, 0, &[])]);
        replies.extend(reacquired());
        replies.push(packet(8, 4, &[]));
        let mut phy = Fake::<true, true>::new(replies);
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        {
            let mut trial = Trial::maintained();
            let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
            assert!(
                engine
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending(),
                "{unexpected:02x?}"
            );
        }
        assert_eq!(phy.tx.len(), 4, "{unexpected:02x?}");
        assert_eq!(phy.tx[1], soft_reset_header(2));
        assert_eq!(&phy.tx[2][2..], &0x2104_b12cu32.to_le_bytes());
        assert_eq!(phy.tx[3][2..], SINK_CAPS);
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x2104_b12c
        );
    }
}

#[test]
fn protocol_error_while_power_transitions_hard_resets() {
    // Table 7.1 / section 9.2.4.6: after Accept only PS_RDY; never a Soft_Reset.
    for unexpected in [
        packet(8, 2, &[]),
        packet(3, 2, &[]),
        packet(6, 2, &[0x1000_0000]),
        extended(2, 2, 0x8001, &[0], true),
    ] {
        let mut replies = ready();
        replies.truncate(2);
        replies.push(unexpected.clone());
        let mut phy = Fake::<true, true>::new(replies);
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let result = complete(Trial::maintained().run_maintained::<_, Never>(&mut phy, &trace));
        assert!(
            matches!(result, Err(Error::HardResetSent)),
            "{unexpected:02x?} {result:?}"
        );
        assert_eq!(
            phy.tx,
            [phy.tx[0].clone(), HARD_RESET.to_vec()],
            "{unexpected:02x?}"
        );
    }
    // A Message at another revision is not a Protocol Error at this one: fail closed.
    let mut replies = ready();
    replies.truncate(2);
    let mut other = packet(8, 2, &[]);
    other[0] = (other[0] & !0xc0) | 0x40;
    replies.push(other);
    let mut phy = Fake::<true, true>::new(replies);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Never>(&mut phy, &trace));
    assert!(matches!(result, Err(Error::Peer)), "{result:?}");
    assert_eq!(phy.tx.len(), 1);
}

#[test]
fn response_timeouts_after_a_request_hard_reset_in_a_maintained_session() {
    // Section 9.2.4.5: SenderResponseTimer, even for the first Request.
    // Section 9.2.4.6: PSTransitionTimer after Accept.
    for count in [1, 2] {
        let mut phy = Fake::<true, true>::new(ready()[..count].to_vec());
        let trace = RefCell::new(pd_spr::MaintainedTrace::default());
        let result = complete(Trial::maintained().run_maintained::<_, Ticks>(&mut phy, &trace));
        assert!(
            matches!(result, Err(Error::HardResetSent)),
            "{count} {result:?}"
        );
        assert_eq!(phy.tx.len(), 2);
        assert_eq!(phy.tx[1], HARD_RESET);
        assert_eq!(trace.borrow().hard_resets_sent, 1);
    }
    // SinkWaitCapTimer before any capabilities: optional, never sent.
    let mut phy = Fake::<true, true>::new(vec![]);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Ticks>(&mut phy, &trace));
    assert!(matches!(result, Err(Error::RecoveryBlocked)), "{result:?}");
    assert!(phy.tx.is_empty());
    // A Hard Reset the PHY could not send ends the session as a fault.
    let mut phy = Fake::<true, true>::new(ready()[..1].to_vec());
    phy.hard_reset_error = Some(DriverTxError::Discarded);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Ticks>(&mut phy, &trace));
    assert!(
        matches!(result, Err(Error::Transmit(DriverTxError::Discarded))),
        "{result:?}"
    );
    assert_eq!(trace.borrow().hard_resets_sent, 0);
    // The Source's Hard Reset crossing ours.
    let mut phy = Fake::<true, true>::new(ready()[..1].to_vec());
    phy.hard_reset_error = Some(DriverTxError::HardReset);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Ticks>(&mut phy, &trace));
    assert!(
        matches!(result, Err(Error::HardResetReceived)),
        "{result:?}"
    );
}

#[test]
fn a_soft_reset_that_fails_or_does_not_correct_the_error_hard_resets() {
    for (traffic, nack, timer_fires, expected_tx) in [
        (vec![packet(3, 3, &[])], vec![], true, 3), // SenderResponseTimer
        (vec![packet(3, 3, &[]), packet(6, 0, &[])], vec![], false, 3), // wrong reply
        (vec![packet(3, 3, &[])], vec![1], false, 3), // Soft_Reset not acknowledged
        // Section 7.1.1: a further Protocol Error before the new contract.
        (
            vec![
                packet(3, 3, &[]),
                packet(3, 0, &[]),
                reacquired()[0].clone(),
                packet(8, 2, &[]),
            ],
            vec![],
            false,
            4,
        ),
        (
            vec![
                packet(3, 3, &[]),
                packet(3, 0, &[]),
                reacquired()[0].clone(),
            ],
            vec![2],
            false,
            4,
        ),
    ] {
        let (tx, trace, outcome) = if timer_fires {
            maintained::<Ticks>(2, traffic.clone(), nack)
        } else {
            maintained::<Never>(2, traffic.clone(), nack)
        };
        assert!(
            matches!(outcome, Some(Err(Error::HardResetSent))),
            "{traffic:02x?} {outcome:?}"
        );
        assert_eq!(tx.len(), expected_tx, "{traffic:02x?}");
        assert_eq!(tx.last().unwrap(), &HARD_RESET, "{traffic:02x?}");
        assert_eq!(trace.hard_resets_sent, 1);
        assert_eq!(tx[1], soft_reset_header(2));
        assert_eq!(
            tx.iter()
                .filter(|bytes| bytes[0] & 0x1f == 13 && bytes[1] & 0xf0 == 0)
                .count(),
            1,
            "never a second Soft_Reset"
        );
        assert_eq!(trace.soft_resets_sent, 1);
    } // An Accept that is not MessageID 0 is malformed for the reset: fail closed.
    let (tx, _, outcome) =
        maintained::<Never>(2, vec![packet(3, 3, &[]), packet(3, 1, &[])], vec![]);
    assert!(matches!(outcome, Some(Err(Error::Peer))), "{outcome:?}");
    assert_eq!(tx.len(), 2);
}

#[test]
fn the_partner_soft_reset_wins_over_ours_and_a_corrected_error_allows_another_reset() {
    // Section 9.2.5.2.2: a Soft_Reset instead of the Accept to ours.
    let mut traffic = vec![packet(3, 3, &[]), packet(13, 0, &[])];
    traffic.extend(reacquired());
    // The error was corrected (PS_RDY): a later Protocol Error is handled again.
    traffic.extend([packet(6, 4, &[]), packet(3, 0, &[])]);
    traffic.extend(reacquired());
    traffic.push(packet(8, 4, &[]));
    let (tx, trace, outcome) = maintained::<Never>(2, traffic, vec![]);
    assert!(outcome.is_none(), "{outcome:?}");
    assert_eq!(tx.len(), 7);
    assert_eq!(tx[1], soft_reset_header(2));
    assert_eq!(&tx[2], &[0x83, 0]);
    assert_eq!(tx[4], soft_reset_header(2));
    assert_eq!(tx[6][2..], SINK_CAPS);
    assert_eq!((trace.soft_resets_sent, trace.soft_resets_accepted), (2, 1));
}

#[test]
fn a_message_without_goodcrc_soft_resets_but_a_local_fault_does_not() {
    // Section 7.1.1: no GoodCRC after nRetryCount retries; Sink_Capabilities reply.
    let mut traffic = vec![packet(8, 3, &[]), packet(3, 0, &[])];
    traffic.extend(reacquired());
    traffic.push(packet(8, 4, &[]));
    let (tx, trace, outcome) = maintained::<Never>(2, traffic, vec![1]);
    assert!(outcome.is_none(), "{outcome:?}");
    assert_eq!(tx.len(), 5);
    assert_eq!(tx[1][2..], SINK_CAPS);
    assert_eq!(tx[2], soft_reset_header(2));
    assert_eq!(tx[4][2..], SINK_CAPS);
    assert_eq!(trace.soft_resets_sent, 1);
    // The initial Request, before any Explicit Contract.
    let mut phy = Fake::<true, true>::new({
        let mut replies = ready();
        replies.truncate(1);
        replies.push(packet(3, 0, &[]));
        replies.extend(reacquired());
        replies
    });
    phy.nack = vec![0];
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    {
        let mut trial = Trial::maintained();
        let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut phy, &trace));
        assert!(
            engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending()
        );
    }
    assert_eq!(phy.tx.len(), 3);
    assert_eq!(phy.tx[1], soft_reset_header(2));
    assert!(trace.borrow().contract.is_some());
    // A Discarded transmission may be a local fault: halt, never reset.
    let mut phy = Fake::<true, true>::new(ready());
    phy.tx_error = Some(DriverTxError::Discarded);
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let result = complete(Trial::maintained().run_maintained::<_, Never>(&mut phy, &trace));
    assert!(
        matches!(result, Err(Error::Transmit(DriverTxError::Discarded))),
        "{result:?}"
    );
    assert_eq!(phy.tx.len(), 1);
    // A one-shot trial never resets.
    let mut phy = Fake::<true, true>::new(ready());
    phy.nack = vec![0];
    assert!(matches!(
        complete(Trial::once().run::<_, Ticks>(&mut phy)),
        Err(Error::RecoveryBlocked)
    ));
    assert_eq!(phy.tx.len(), 1);
}

#[test]
fn a_soft_reset_discards_a_queued_change_and_requests_the_established_target() {
    changing_to_9v(|selection, state, trace, poll| {
        // A Get_Sink_Cap instead of the Accept for the 9 V Request.
        state.borrow_mut().rx.push_back(packet(8, 3, &[]));
        assert!(poll().is_pending());
        assert_eq!(&state.borrow().tx[2], &[0x8d, 0]);
        assert!(selection.request_in_flight() && !selection.confirm());
        state.borrow_mut().rx.push_back(packet(3, 0, &[]));
        state.borrow_mut().rx.extend(reacquired());
        assert!(poll().is_pending());
        assert_eq!(
            u32::from_le_bytes(state.borrow().tx[3][2..6].try_into().unwrap()),
            0x1104_b12c
        );
        assert_eq!(selection.target_mv(), 5000);
        assert!(!selection.request_in_flight());
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            0x1104_b12c
        );
    });
}

/// Source_Capabilities_Extended block (Table 6.50): VID 291Ah, PID 1234h, FW 1,
/// HW 2, holdup 3 ms, LPS, IEC 62368-1 TS1, External Supply, 65 W SPR, 0 W EPR.
const SCEDB: [u8; 25] = [
    0x1a, 0x29, 0x34, 0x12, 0, 0, 0, 0, 1, 2, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 65, 0,
];
fn scedb_reply(id: u16) -> Vec<u8> {
    extended(1, id, 0x8019, &SCEDB, true)
}
fn info_reply(id: u16, block: &[u8]) -> Vec<u8> {
    extended(7, id, 0x8000 | block.len() as u16, block, true)
}
fn query_kind(bytes: &[u8]) -> Option<pd_protocol::pd_partner::PartnerQuery> {
    pd_protocol::pd_partner::query_request(bytes)
}

/// A fixed 5 V contract at time 0 (Request tx[0], source IDs 0..2) with the
/// interactive peer's automatic Not_Supported off; information requests reach `tx`.
fn query_session<T: usbpd::timers::Timer>(
    revision: u8,
    defer_kind: u8,
    check: impl FnOnce(
        &pd_spr::Selection,
        &Rc<RefCell<InteractiveState>>,
        &RefCell<pd_spr::MaintainedTrace>,
        &Rc<std::cell::Cell<usize>>,
        &mut dyn FnMut() -> Poll<Result<(), Error>>,
    ),
) {
    ANSWER_QUERIES.set(false);
    let selection = pd_spr::Selection::new();
    let mut rx = ready();
    for reply in &mut rx {
        reply[0] = (reply[0] & !0xc0) | (revision << 6);
    }
    let state = Rc::new(RefCell::new(InteractiveState {
        rx: rx.into(),
        tx: Vec::new(),
        idle: true,
        now: 0,
    }));
    let defer = Rc::new(std::cell::Cell::new(0));
    let mut driver = Deferring {
        inner: Interactive {
            state: state.clone(),
            selection: &selection,
        },
        defer: defer.clone(),
        attempts: Rc::new(RefCell::new(Vec::new())),
        kind: defer_kind,
    };
    let trace = RefCell::new(pd_spr::MaintainedTrace::default());
    let mut trial = Trial::maintained();
    let mut engine = std::pin::pin!(trial.run_selectable::<_, T>(&mut driver, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    let mut poll = || engine.as_mut().poll(&mut cx);
    assert!(poll().is_pending());
    assert!(selection.established_target().is_some());
    assert_eq!(state.borrow().tx.len(), 1);
    check(&selection, &state, &trace, &defer, &mut poll);
    ANSWER_QUERIES.set(true);
}

#[test]
fn information_requests_run_once_in_order_after_the_first_contract() {
    use pd_protocol::pd_partner::Answer;
    query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
        assert_eq!(
            at(state, poll, pd_spr::QUERY_SETTLE_MS - 1),
            1,
            "not before QUERY_SETTLE_MS"
        );
        assert_eq!(selection.partner_info().source_extended, Answer::Pending);
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
        // Get_Source_Cap_Extended: Control, PD3, Sink/UFP, MessageID 1.
        assert_eq!(state.borrow().tx[1], [0x91, 0x02]);
        state.borrow_mut().rx.push_back(scedb_reply(3));
        assert!(poll().is_pending());
        // The next request follows at once: Get_Revision, MessageID 2.
        assert_eq!(state.borrow().tx[2], [0x98, 0x04]);
        state
            .borrow_mut()
            .rx
            .push_back(packet(12, 4, &[0x3118_0000]));
        assert!(poll().is_pending());
        // Get_Manufacturer_Info for the Port: Chunked, Data Size 2, Target 0, Ref 0.
        assert_eq!(state.borrow().tx[3], [0x86, 0x96, 0x02, 0x80, 0, 0]);
        state
            .borrow_mut()
            .rx
            .push_back(info_reply(5, b"\x1a\x29\x34\x12ACME 65W\0"));
        assert!(poll().is_pending());
        // Get_Source_Info (ID 4): SIDO1 Guaranteed, 140 W max, 65 W now, 140 W reported;
        // SIDO2 DPS, 140 W max, 70 W guaranteed.
        assert_eq!(state.borrow().tx[4], [0x97, 0x08]);
        state
            .borrow_mut()
            .rx
            .push_back(packet(11, 6, &[0x8c8c_418c, 0x4002_308c]));
        assert!(poll().is_pending());
        // Get_Status (ID 5): 40 C, AC input, normal temperature.
        assert_eq!(state.borrow().tx[5], [0x92, 0x0a]);
        state
            .borrow_mut()
            .rx
            .push_back(extended(2, 7, 0x8007, &[40, 6, 0, 0, 2, 0, 1], true));
        assert!(poll().is_pending());
        // Once per session: nothing more, also not after a later contract change.
        assert_eq!(at(state, poll, 5000), 6);
        let info = selection.partner_info();
        let Answer::Value(source) = info.source_extended else {
            panic!("{info:?}")
        };
        assert_eq!(
            (
                source.vid,
                source.pid,
                source.fw_version,
                source.hw_version,
                source.holdup_ms
            ),
            (0x291a, 0x1234, 1, 2, 3)
        );
        assert_eq!(
            (source.lps, source.touch_temp, source.external_supply),
            (true, 1, true)
        );
        assert_eq!((source.spr_pdp_w, source.epr_pdp_w), (Some(65), Some(0)));
        let Answer::Value(revision) = info.revision else {
            panic!("{info:?}")
        };
        assert_eq!(
            (
                revision.major,
                revision.minor,
                revision.version_major,
                revision.version_minor
            ),
            (3, 1, 1, 8)
        );
        let Answer::Value(manufacturer) = info.manufacturer else {
            panic!("{info:?}")
        };
        assert_eq!(
            (manufacturer.vid, manufacturer.pid, manufacturer.name()),
            (0x291a, 0x1234, &b"ACME 65W"[..])
        );
        let Answer::Value(source_info) = info.source_info else {
            panic!("{info:?}")
        };
        assert_eq!((source_info.maximum_w, source_info.present_w), (140, 65));
        assert_eq!(
            source_info
                .second
                .map(|second| (second.maximum_half_w, second.guaranteed_half_w)),
            Some((280, 140))
        );
        let Answer::Value(status) = info.status else {
            panic!("{info:?}")
        };
        assert_eq!(
            (
                status.internal_temp,
                status.external_power,
                info.status_reads
            ),
            (40, 3, 1)
        );
        assert_eq!(trace.borrow().queries_answered, 5);
        assert!(selection.next_query().is_none());
    });
}

#[test]
fn information_requests_record_not_supported_and_timeout_and_a_late_answer_gets_not_supported() {
    use pd_protocol::pd_partner::Answer;
    query_session::<SenderResponseOnly>(2, 0, |selection, state, trace, _, poll| {
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
        for id in [3, 4, 5] {
            state.borrow_mut().rx.push_back(packet(16, id, &[]));
            assert!(poll().is_pending());
        }
        // The Get_Status sent last runs a SenderResponseTimer that expires: a
        // normal exit to Ready, no reset.
        SENDER_RESPONSE_EXPIRES.set(true);
        state.borrow_mut().rx.push_back(packet(16, 6, &[]));
        assert!(poll().is_pending());
        assert_eq!(
            query_kind(&state.borrow().tx[5]),
            Some(pd_protocol::pd_partner::PartnerQuery::Status)
        );
        assert!(poll().is_pending());
        SENDER_RESPONSE_EXPIRES.set(false);
        let info = selection.partner_info();
        assert_eq!(
            (info.source_extended, info.revision),
            (Answer::NotSupported, Answer::NotSupported)
        );
        assert_eq!(
            (info.manufacturer, info.source_info),
            (Answer::NotSupported, Answer::NotSupported)
        );
        assert_eq!(info.status, Answer::Timeout);
        assert_eq!(
            (
                trace.borrow().queries_not_supported,
                trace.borrow().queries_timeouts
            ),
            (4, 1)
        );
        // A late Status is an unexpected Message in Ready.
        state
            .borrow_mut()
            .rx
            .push_back(extended(2, 7, 0x8007, &[40, 6, 0, 0, 2, 0, 1], true));
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 7);
        assert_eq!(
            tx_header(&state.borrow().tx[6]) & 0xf01f,
            16,
            "Not_Supported"
        );
        assert_eq!(at(state, poll, 5000), 7, "not asked again");
    });
}

#[test]
fn pd2_sessions_never_send_information_requests() {
    use pd_protocol::pd_partner::Answer;
    query_session::<Never>(1, 0, |selection, state, _, _, poll| {
        assert_eq!(at(state, poll, 5000), 1);
        let info = selection.partner_info();
        assert_eq!(
            (info.source_extended, info.revision),
            (Answer::NotSent, Answer::NotSent)
        );
        assert_eq!(info.manufacturer, Answer::NotSent);
        assert!(!selection.query_due());
    });
}

#[test]
fn malformed_information_answers_fail_closed() {
    let pd2 = |mut reply: Vec<u8>| {
        reply[0] = (reply[0] & !0xc0) | 0x40;
        reply
    };
    // In the Source_Capabilities_Extended window (source ID 3).
    for reply in [
        extended(1, 3, 0x8017, &SCEDB[..23], true), // Data Size 23
        extended(1, 3, 0x0019, &SCEDB, false),      // unchunked
        extended(1, 3, 0x8819, &SCEDB, true),       // Chunk 1
        extended(1, 3, 0x8419, &SCEDB, true),       // Chunk request
        pd2(scedb_reply(3)),
        pd2(packet(16, 3, &[])), // PD2 Not_Supported
    ] {
        query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
            assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
            state.borrow_mut().rx.push_back(reply.clone());
            assert!(
                matches!(poll(), Poll::Ready(Err(Error::Peer))),
                "{reply:02x?}"
            );
            assert_eq!(state.borrow().tx.len(), 2, "no reset, no response");
            assert_eq!(trace.borrow().queries_answered, 0);
            assert!(selection.partner_info().source_extended.is_pending());
        });
    }
    // Revision with two Data Objects; Manufacturer_Info of 3 bytes.
    for (answers, reply) in [
        (1, packet(12, 4, &[0x3118_0000, 0])),
        (2, info_reply(5, &[0x1a, 0x29, 0x34])),
    ] {
        query_session::<Never>(2, 0, |_, state, _, _, poll| {
            assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
            state.borrow_mut().rx.push_back(scedb_reply(3));
            assert!(poll().is_pending());
            if answers == 2 {
                state
                    .borrow_mut()
                    .rx
                    .push_back(packet(12, 4, &[0x3118_0000]));
                assert!(poll().is_pending());
            }
            let sent = state.borrow().tx.len();
            state.borrow_mut().rx.push_back(reply.clone());
            assert!(
                matches!(poll(), Poll::Ready(Err(Error::Peer))),
                "{reply:02x?}"
            );
            assert_eq!(state.borrow().tx.len(), sent);
        });
    }
    // Extended Header bit 9 is Reserved and ignored.
    query_session::<Never>(2, 0, |selection, state, _, _, poll| {
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
        state
            .borrow_mut()
            .rx
            .push_back(extended(1, 3, 0x8219, &SCEDB, true));
        assert!(poll().is_pending());
        assert!(matches!(
            selection.partner_info().source_extended,
            pd_protocol::pd_partner::Answer::Value(_)
        ));
    });
}

#[test]
fn unexpected_message_during_an_information_request_soft_resets_and_it_is_not_repeated() {
    use pd_protocol::pd_partner::{Answer, PartnerQuery};
    query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
        // Section 9.2.5.2.1: an Accept is a Protocol Error during the AMS.
        state.borrow_mut().rx.push_back(packet(3, 3, &[]));
        assert!(poll().is_pending());
        assert_eq!(
            &state.borrow().tx[2],
            &[0x8d, 0],
            "Soft_Reset, MessageID 0, PD3"
        );
        assert_eq!(trace.borrow().soft_resets_sent, 1);
        state.borrow_mut().rx.push_back(packet(3, 0, &[]));
        state.borrow_mut().rx.extend(reacquired());
        assert!(poll().is_pending());
        // The same contract again (ID 1), then the next request (ID 2).
        assert_eq!(message_id(&state.borrow().tx[3]), 1);
        assert_eq!(
            query_kind(&state.borrow().tx[4]),
            Some(PartnerQuery::Revision)
        );
        assert_eq!(message_id(&state.borrow().tx[4]), 2);
        assert_eq!(
            selection.partner_info().source_extended,
            Answer::Interrupted
        );
    });
}

#[test]
fn deferred_information_request_is_retried_after_cooldown_at_most_three_times() {
    use pd_protocol::pd_partner::{Answer, PartnerQuery};
    // Get_Source_Cap_Extended (type 10001b) is never let through.
    query_session::<Never>(2, 0x11, |selection, state, trace, defer, poll| {
        defer.set(usize::MAX);
        let settle = pd_spr::QUERY_SETTLE_MS;
        for (now, deferred) in [
            (settle, 1),
            (settle + 99, 1),
            (settle + 100, 2),
            (settle + 200, 3),
        ] {
            state.borrow_mut().now = now;
            assert!(poll().is_pending());
            assert_eq!(trace.borrow().queries_deferred, deferred, "at {now} ms");
        }
        assert_eq!(selection.partner_info().source_extended, Answer::Deferred);
        // Given up after three unsent attempts; the next request goes out at once.
        assert_eq!(state.borrow().tx.len(), 2);
        assert_eq!(
            query_kind(&state.borrow().tx[1]),
            Some(PartnerQuery::Revision)
        );
        assert_eq!(
            message_id(&state.borrow().tx[1]),
            1,
            "deferred attempts used no MessageID"
        );
    });
    // One deferral, then the retry is sent and answered.
    query_session::<Never>(2, 0x11, |selection, state, _, defer, poll| {
        defer.set(1);
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 1);
        assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS + 100), 2);
        assert_eq!(state.borrow().tx[1], [0x91, 0x02]);
        state.borrow_mut().rx.push_back(scedb_reply(3));
        assert!(poll().is_pending());
        assert!(matches!(
            selection.partner_info().source_extended,
            Answer::Value(_)
        ));
    });
}

/// Answer the five session requests in order (source IDs from 3), the last
/// one, Get_Status, with `status`.
fn answer_session_queries(
    state: &Rc<RefCell<InteractiveState>>,
    poll: &mut dyn FnMut() -> Poll<Result<(), Error>>,
    status: &[u8],
) {
    assert_eq!(at(state, poll, pd_spr::QUERY_SETTLE_MS), 2);
    for id in [3, 4, 5, 6] {
        state.borrow_mut().rx.push_back(packet(16, id, &[]));
        assert!(poll().is_pending());
    }
    state
        .borrow_mut()
        .rx
        .push_back(extended(2, 7, 0x8000 | status.len() as u16, status, true));
    assert!(poll().is_pending());
    assert_eq!(state.borrow().tx.len(), 6);
}

#[test]
fn a_non_battery_alert_asks_for_status_again_at_most_every_200_ms() {
    use pd_protocol::pd_partner::Answer;
    query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
        answer_session_queries(state, poll, &[40, 6, 0, 0, 2, 0, 1]);
        assert_eq!(selection.partner_info().status_reads, 1);
        // Battery Status Change only (bit 25): Get_Battery_Status territory, no Get_Status.
        let now = pd_spr::QUERY_SETTLE_MS + 10;
        state.borrow_mut().now = now;
        state
            .borrow_mut()
            .rx
            .push_back(packet(6, 0, &[1 << 25 | 1 << 20]));
        assert!(poll().is_pending());
        assert_eq!(at(state, poll, now + 1000), 6);
        // OCP (bit 26): Get_Status at once (MessageID 6).
        state.borrow_mut().rx.push_back(packet(6, 1, &[1 << 26]));
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx[6], [0x92, 0x0c]);
        state.borrow_mut().rx.push_back(extended(
            2,
            2,
            0x8007,
            &[90, 6, 0, 0x02, 6, 0x20, 1],
            true,
        ));
        assert!(poll().is_pending());
        let Answer::Value(status) = selection.partner_info().status else {
            panic!()
        };
        assert!(status.ocp && status.temperature == 3);
        assert_eq!(selection.partner_info().status_reads, 2);
        // Two more Alerts at once: one Get_Status, the next 200 ms after the last.
        state
            .borrow_mut()
            .rx
            .extend([packet(6, 3, &[1 << 28]), packet(6, 4, &[1 << 30])]);
        assert!(poll().is_pending());
        let asked = state.borrow().tx.len();
        assert_eq!(asked, 7, "within 200 ms of the previous Get_Status");
        assert_eq!(
            at(state, poll, now + 1000 + pd_spr::STATUS_MIN_INTERVAL_MS),
            8
        );
        assert_eq!(
            query_kind(&state.borrow().tx[7]),
            Some(pd_protocol::pd_partner::PartnerQuery::Status)
        );
        state.borrow_mut().rx.push_back(packet(16, 5, &[]));
        assert!(poll().is_pending());
        assert_eq!(selection.partner_info().status, Answer::NotSupported);
        assert_eq!(
            at(state, poll, now + 5000),
            8,
            "both Alerts answered by one read"
        );
        assert_eq!(trace.borrow().alerts_received, 4);
    });
}

#[test]
fn get_status_returns_caller_snapshot_once_per_message_and_pd2_rejects() {
    for revision in [1, 2] {
        let (tx, trace, outcome) =
            after_contract::<Never>(revision, vec![packet(18, 3, &[]), packet(18, 3, &[])]);
        assert!(outcome.is_none());
        assert_eq!(tx.len(), 3, "Request, one reply, Sink_Capabilities");
        if revision == 2 {
            assert_eq!(tx[1], [0x82, 0xb2, 7, 0x80, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0]);
            assert_eq!(trace.statuses_sent, 1);
        } else {
            assert_eq!(tx[1], [0x44, 2]);
            assert_eq!(trace.statuses_sent, 0);
        }
    }
}

#[test]
fn sink_alert_is_explicit_only_clears_after_ack_and_answers_get_status() {
    query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
        assert_eq!(selection.pending_sink_alert(), 0);
        assert!(!selection.sink_alert_due(), "no fabricated boot event");
        assert!(
            selection.notify_sink_alert(1 << 29),
            "host-simulated input change"
        );
        assert_eq!(selection.pending_sink_alert(), 1 << 29);
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx[1], [0x86, 0x12, 0, 0, 0, 0x20]);
        assert_eq!(selection.pending_sink_alert(), 0, "clear only on success");
        assert_eq!(trace.borrow().sink_alerts_sent, 1);
        // No periodic or duplicate spontaneous Alert.
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 2);
        state.borrow_mut().rx.push_back(packet(18, 3, &[]));
        assert!(poll().is_pending());
        assert_eq!(
            state.borrow().tx[2],
            [0x82, 0xb4, 7, 0x80, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(trace.borrow().statuses_sent, 1);
        assert_eq!(trace.borrow().soft_resets_sent, 0);
    });
}

#[test]
fn sink_alert_timeout_returns_ready_and_late_get_status_still_works() {
    query_session::<Ticks>(2, 0, |selection, state, trace, _, poll| {
        assert!(selection.notify_sink_alert(1 << 29));
        for _ in 0..4 {
            assert!(poll().is_pending());
        }
        state.borrow_mut().rx.push_back(packet(18, 3, &[]));
        assert!(poll().is_pending());
        assert_eq!(trace.borrow().statuses_sent, 1);
        assert_eq!(trace.borrow().hard_resets_sent, 0);
        assert_eq!(trace.borrow().soft_resets_sent, 0);
        assert_eq!(state.borrow().tx.len(), 3);
    });
}

#[test]
fn sink_alert_deferral_retains_events_without_blind_retry() {
    query_session::<Never>(2, 6, |selection, state, trace, defer, poll| {
        defer.set(1);
        assert!(selection.notify_sink_alert(1 << 29));
        assert!(poll().is_pending());
        assert_eq!(selection.pending_sink_alert(), 1 << 29);
        assert!(!selection.sink_alert_due());
        assert!(selection.armed_sink_alert().is_none());
        assert_eq!(trace.borrow().sink_alerts_sent, 0);
        assert_eq!(trace.borrow().sink_alerts_deferred, 1);
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 1);
        assert!(
            selection.notify_sink_alert(1 << 29),
            "explicit producer report re-arms"
        );
        assert!(poll().is_pending());
        assert_eq!(trace.borrow().sink_alerts_sent, 1);
    });
}

#[test]
fn sink_alert_rejects_unsupported_events_pd2_and_unestablished_sessions() {
    let selection = pd_spr::Selection::new();
    assert!(!selection.notify_sink_alert(1 << 29));
    query_session::<Never>(1, 0, |selection, _, _, _, _| {
        assert!(!selection.notify_sink_alert(1 << 29));
    });
    query_session::<Never>(2, 0, |selection, state, _, _, poll| {
        for ado in [
            0,
            1,
            1 << 26,
            1 << 25,
            1 << 30,
            1 << 28,
            1 << 27,
            0xffff_ffff,
        ] {
            assert!(!selection.notify_sink_alert(ado), "{ado:08x}");
        }
        assert!(poll().is_pending());
        assert_eq!(state.borrow().tx.len(), 1);
    });
}

#[test]
fn get_status_at_a_different_locked_revision_halts_without_reply() {
    query_session::<Never>(2, 0, |_, state, trace, _, poll| {
        let mut reply = packet(18, 3, &[]);
        reply[0] = (reply[0] & !0xc0) | 0x40;
        state.borrow_mut().rx.push_back(reply);
        assert!(matches!(poll(), Poll::Ready(Err(Error::Peer))));
        assert_eq!(state.borrow().tx.len(), 1);
        assert_eq!(trace.borrow().statuses_sent, 0);
    });
}

#[test]
fn sink_alert_unexpected_reply_soft_resets_and_malformed_reply_halts() {
    for reply in [packet(7, 3, &[]), vec![0x92, 0x17]] {
        query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
            assert!(selection.notify_sink_alert(1 << 29));
            assert!(poll().is_pending());
            state.borrow_mut().rx.push_back(reply.clone());
            if reply.len() == 2 && reply[1] == 0x17 {
                assert!(matches!(poll(), Poll::Ready(Err(Error::Peer))));
            } else {
                assert!(poll().is_pending());
                assert_eq!(trace.borrow().soft_resets_sent, 1);
                assert_eq!(state.borrow().tx[2], [0x8d, 0]);
            }
        });
    }
}
