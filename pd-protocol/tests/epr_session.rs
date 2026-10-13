//! Black-box entry, request, keep-alive and commanded exit through the session gate.
#[path = "support/timing.rs"]
mod timing;
use pd_protocol::pd_spr::{self as pd, EprState, Supply};
use std::{
    cell::{Cell, RefCell},
    collections::VecDeque,
    future::{Future, pending},
    rc::Rc,
    task::{Context, Poll, Waker},
};
use usbpd_traits::{Driver, DriverRxError, DriverTxError};
struct Limits;
impl pd::Limits for Limits {
    const SINK_IDENTITY: pd::SinkIdentity = pd::SinkIdentity::UNASSIGNED;
    const SESSION_POLICY: pd::SessionPolicy = timing::SESSION_POLICY;
    const MAX_REQUEST_MV: u32 = 30_000;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = 5_000;
    const REQUEST_MA: u32 = 100;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 2;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 20_000, 28_000];
    const PPS_REFRESH_MS: u64 = 4_000;
    const EPR_OFFER: bool = true;
    const EPR: bool = true;
}
thread_local! { static NOW: Cell<u64> = const { Cell::new(0) }; }
struct Timer;
impl usbpd::timers::Timer for Timer {
    async fn after_millis(ms: u64) {
        if !matches!(ms, 375 | 500) && ms != usbpd_traits::SENDER_RESPONSE_MS {
            pending::<()>().await;
        }
        let deadline = NOW.get() + ms;
        std::future::poll_fn(|_| {
            if NOW.get() >= deadline {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}
fn fixed(mv: u32) -> u32 {
    (mv / 50) << 10 | 500
}
fn spr() -> [u32; 3] {
    [fixed(5_000) | 1 << 23, fixed(9_000), fixed(20_000)]
}
fn epr() -> Vec<u8> {
    [
        spr()[0],
        spr()[1],
        spr()[2],
        0,
        0,
        0,
        0,
        fixed(28_000),
        0xd000_0000 | 360 << 17 | 150 << 8 | 180,
    ]
    .into_iter()
    .flat_map(u32::to_le_bytes)
    .collect()
}
#[derive(Default)]
struct Source {
    rx: VecDeque<Vec<u8>>,
    tx: Vec<Vec<u8>>,
    id: u16,
    resets: usize,
    fail_entry: Option<u8>,
    suppress_keep_ack: bool,
    hard_reset: bool,
    sink_block: Vec<u8>,
}
impl Source {
    fn send(&mut self, kind: u16, body: &[u8]) {
        let mut bytes = (kind | 0x180 | self.id << 9).to_le_bytes().to_vec();
        self.id = (self.id + 1) % 8;
        bytes.extend(body);
        self.rx.push_back(bytes);
    }
    fn spr(&mut self) {
        self.send(
            0x3001,
            &spr()
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect::<Vec<_>>(),
        );
    }
    fn chunk(&mut self, chunk: usize) {
        let block = epr();
        let carried = &block[chunk * 26..block.len().min((chunk + 1) * 26)];
        let mut body = (0x8000 | (chunk as u16) << 11 | block.len() as u16)
            .to_le_bytes()
            .to_vec();
        body.extend(carried);
        body.resize(body.len().div_ceil(4) * 4, 0);
        self.send(0x8011 | (body.len() as u16 / 4) << 12, &body);
    }
}
struct Port<'a> {
    source: Rc<RefCell<Source>>,
    selection: &'a pd::Selection<Limits>,
}
impl Driver for Port<'_> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        self.source.borrow_mut().resets += 1;
        Ok(())
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        let mut source = self.source.borrow_mut();
        source.tx.push(bytes.to_vec());
        let kind = u16::from_le_bytes([bytes[0], bytes[1]]) & 0xf1ff;
        match kind {
            0x1082 | 0x2089 => {
                source.send(3, &[]);
                source.send(6, &[]);
            }
            0x108a if bytes[5] == 1 => {
                if let Some(reason) = source.fail_entry {
                    source.send(
                        0x100a,
                        &(0x0400_0000 | u32::from(reason) << 16).to_le_bytes(),
                    );
                } else {
                    source.send(0x100a, &0x0200_0000_u32.to_le_bytes());
                    source.send(0x100a, &0x0300_0000_u32.to_le_bytes());
                    source.chunk(0);
                }
            }
            0x108a if bytes[5] == 5 => source.spr(),
            0x9090 if bytes[4] == 1 => source.chunk(0),
            0x9090 if bytes[4] == 3 => {
                if !source.suppress_keep_ack {
                    source.send(0x9010, &[2, 0x80, 4, 0]);
                }
            }
            0x9091 => source.chunk(1),
            0x008d => {
                source.id = 0;
                source.send(3, &[]);
                source.spr();
            }
            _ if kind & 0x801f == 0x8012 => {
                let extended = u16::from_le_bytes([bytes[2], bytes[3]]);
                let size = usize::from(extended & 0x1ff);
                let chunk = usize::from(extended >> 11 & 0xf);
                let len = (size - chunk * 26).min(26);
                source.sink_block.extend(&bytes[4..4 + len]);
                if chunk == 0 {
                    source.send(0x9012, &[0, 0x8c, 0, 0]);
                }
            }
            _ => panic!("unexpected outgoing header {kind:04x}"),
        }
        Ok(())
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        std::future::poll_fn(|_| {
            if std::mem::take(&mut self.source.borrow_mut().hard_reset) {
                return Poll::Ready(Err(DriverRxError::HardReset));
            }
            if let Some(bytes) = self.source.borrow_mut().rx.pop_front() {
                self.selection.set_rx_idle(false);
                buffer[..bytes.len()].copy_from_slice(&bytes);
                Poll::Ready(Ok(bytes.len()))
            } else {
                self.selection.set_rx_idle(true);
                Poll::Pending
            }
        })
        .await
    }
}
#[test]
fn confirmed_epr_profiles_enter_keep_alive_and_exit_through_an_spr_contract() {
    for avs in [false, true] {
        NOW.set(0);
        let selection = pd::Selection::<Limits>::new();
        let source = Rc::new(RefCell::new(Source::default()));
        source.borrow_mut().spr();
        let mut port = Port {
            source: source.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(pd::MaintainedTrace::default());
        let mut trial = pd::Trial::<Limits>::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Timer>(&mut port, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        let mut poll = || {
            let result = engine.as_mut().poll(&mut cx);
            assert!(result.is_pending(), "session ended: {result:?}");
        };
        poll();
        assert_eq!(selection.epr_state(), EprState::Spr);
        assert!(selection.request_epr_offer());
        poll();
        let target = if avs {
            selection.avs_target(9, 30_000).unwrap()
        } else {
            (0..selection.option_count())
                .filter_map(|i| selection.option(i))
                .find(|target| target.mv == 28_000 && target.supply == Supply::Fixed)
                .unwrap()
        };
        assert!(selection.preview(target));
        assert!(selection.confirm());
        poll();
        assert_eq!(selection.epr_state(), EprState::Active);
        assert_eq!(selection.established_target(), Some(target));
        source.borrow_mut().send(0x9010, &[2, 0x80, 2, 0]);
        poll();
        let (sink, count) = pd::epr_sink_capabilities::<Limits>().unwrap();
        assert_eq!(
            source.borrow().sink_block,
            sink[..count]
                .iter()
                .flat_map(|pdo| pdo.to_le_bytes())
                .collect::<Vec<_>>()
        );
        NOW.set(375);
        selection.set_now_ms(375);
        poll();
        assert!(
            source
                .borrow()
                .tx
                .iter()
                .any(|bytes| bytes.len() == 6 && bytes[2..] == [2, 0x80, 3, 0])
        );
        assert!(selection.request_epr_exit());
        poll();
        assert_eq!(selection.epr_state(), EprState::Spr);
        assert_eq!(selection.established_target().unwrap().mv, 5_000);
        assert_eq!(source.borrow().resets, 0);
        let tx = &source.borrow().tx;
        let exit = tx
            .iter()
            .position(|bytes| bytes.len() == 6 && bytes[5] == 5 && bytes[0] & 0x1f == 10)
            .unwrap();
        assert_eq!(
            tx[exit - 1].len(),
            10,
            "SPR PDO was negotiated with EPR_Request before exit"
        );
        assert_eq!(
            u32::from_le_bytes(tx[exit - 1][2..6].try_into().unwrap()) >> 28,
            1
        );
    }
}

#[test]
fn failed_entry_reports_reason_and_preserves_the_spr_contract() {
    NOW.set(0);
    let selection = pd::Selection::<Limits>::new();
    let source = Rc::new(RefCell::new(Source {
        fail_entry: Some(1),
        ..Source::default()
    }));
    source.borrow_mut().spr();
    let mut port = Port {
        source: source.clone(),
        selection: &selection,
    };
    let trace = RefCell::new(pd::MaintainedTrace::default());
    let mut trial = pd::Trial::<Limits>::maintained();
    let mut engine =
        std::pin::pin!(trial.run_selectable::<_, Timer>(&mut port, &trace, &selection));
    let mut cx = Context::from_waker(Waker::noop());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert!(selection.request_epr_offer());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    let target = (0..selection.option_count())
        .filter_map(|i| selection.option(i))
        .find(|target| target.mv == 28_000)
        .unwrap();
    assert!(selection.preview(target));
    assert!(selection.confirm());
    assert!(engine.as_mut().poll(&mut cx).is_pending());
    assert_eq!(selection.epr_state(), EprState::Failed(1));
    assert_eq!(selection.established_target().unwrap().mv, 5_000);
    assert_eq!(trace.borrow().soft_resets_sent, 1);
    assert_eq!(source.borrow().resets, 0);
}

#[test]
fn hard_reset_and_keep_alive_timeout_clear_epr_mode() {
    for received in [false, true] {
        NOW.set(0);
        let selection = pd::Selection::<Limits>::new();
        let source = Rc::new(RefCell::new(Source {
            suppress_keep_ack: true,
            ..Source::default()
        }));
        source.borrow_mut().spr();
        let mut port = Port {
            source: source.clone(),
            selection: &selection,
        };
        let trace = RefCell::new(pd::MaintainedTrace::default());
        let mut trial = pd::Trial::<Limits>::maintained();
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Timer>(&mut port, &trace, &selection));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert!(selection.request_epr_offer());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        let target = selection.avs_target(9, 30_000).unwrap();
        assert!(selection.preview(target));
        assert!(selection.confirm());
        assert!(engine.as_mut().poll(&mut cx).is_pending());
        assert_eq!(selection.epr_state(), EprState::Active);
        if received {
            source.borrow_mut().hard_reset = true;
        } else {
            NOW.set(375);
            assert!(engine.as_mut().poll(&mut cx).is_pending());
            NOW.set(375 + usbpd_traits::SENDER_RESPONSE_MS);
        }
        let result = engine.as_mut().poll(&mut cx);
        assert!(
            matches!(result, Poll::Ready(Err(pd::Error::HardResetReceived))) && received
                || matches!(result, Poll::Ready(Err(pd::Error::HardResetSent))) && !received
        );
        assert_eq!(selection.epr_state(), EprState::Spr);
        assert_eq!(source.borrow().resets, usize::from(!received));
    }
}
