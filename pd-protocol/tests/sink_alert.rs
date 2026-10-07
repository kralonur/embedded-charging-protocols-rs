use pd_protocol::pd_spr::{SINK_ALERT_EVENTS, sink_alert_valid, sink_status_valid};
#[path = "support/timing.rs"]
mod timing;

#[test]
fn sink_status_transmit_fields_are_truthful_and_reserved_bits_are_zero() {
    let dc = [0, 2, 0, 0, 0, 0, 0];
    assert!(sink_status_valid(dc));
    for (index, mask) in [
        (1, 1),
        (1, 0x20),
        (2, 1),
        (3, 1),
        (3, 0x20),
        (4, 1),
        (5, 2),
        (6, 0x40),
        (6, 7),
        (6, 0x20),
    ] {
        let mut bad = dc;
        bad[index] |= mask;
        assert!(!sink_status_valid(bad), "{bad:02x?}");
    }
    assert!(
        !sink_status_valid([0, 4, 0, 0, 0, 0, 0]),
        "invalid input encoding"
    );
    assert!(
        !sink_status_valid([0, 2, 0, 4, 2, 0, 0]),
        "OTP without overtemperature"
    );
    assert!(
        !sink_status_valid([0, 2, 0, 0, 6, 0, 0]),
        "overtemperature without OTP"
    );
    assert!(sink_status_valid([60, 2, 0, 4, 6, 0, 0]));
    assert!(
        sink_status_valid([0, 2, 0, 0x12, 0, 0, 0]),
        "defined SDB flags are distinct from Sink ADO bit 26"
    );
    assert!(
        sink_status_valid([0, 8, 1, 0, 2, 0, 1]),
        "another caller's battery and power state"
    );
}

#[test]
fn sink_alert_only_reports_supported_detected_events_matching_status() {
    let dc = [0, 2, 0, 0, 0, 0, 0];
    assert!(sink_alert_valid(1 << 29, dc));
    for bit in 0..32 {
        if SINK_ALERT_EVENTS & (1 << bit) == 0 {
            assert!(!sink_alert_valid(1 << bit, dc), "bit {bit}");
        }
    }
    assert!(!sink_alert_valid(0, dc));
    for bit in [30, 28, 27] {
        assert!(!sink_alert_valid(1 << bit, dc));
    }
    assert!(sink_alert_valid(1 << 30, [0, 2, 0, 8, 0, 0, 0]));
    assert!(sink_alert_valid(1 << 28, [0, 2, 0, 0, 2, 0, 0]));
    assert!(sink_alert_valid(SINK_ALERT_EVENTS, [80, 2, 0, 12, 6, 0, 0]));
}

#[test]
fn alert_and_status_fit_existing_crc_checked_partner_framing() {
    use pd_protocol::pd_rx::{self, Sop};
    for packet in [
        &[0x86, 0x10, 0, 0, 0, 0x20][..],
        &[0x82, 0xb0, 7, 0x80, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0][..],
    ] {
        let mut wire = packet.to_vec();
        wire.extend(pd_rx::crc32(packet).to_le_bytes());
        assert!(
            wire.len() < 35,
            "fits the FIFO address/header/data/CRC frame"
        );
        let (frame, consumed) = pd_rx::decode_wire_with(Sop::Partner, &wire, false, true).unwrap();
        assert_eq!(consumed, wire.len());
        if packet.len() == 14 {
            assert_eq!(frame.extended_data().unwrap(), &[0, 2, 0, 0, 0, 0, 0]);
        } else {
            assert_eq!(frame.data(), &[0, 0, 0, 0x20]);
        }
        assert!(
            pd_rx::decode_wire_with(Sop::Partner, &wire[..wire.len() - 1], false, true).is_err()
        );
        wire[4] ^= 1;
        assert!(pd_rx::decode_wire_with(Sop::Partner, &wire, false, true).is_err());
    }
}

// Simulated caller facts and a latched OVP event.
thread_local! {
    static OVP: std::cell::Cell<u8> = const { std::cell::Cell::new(8) };
    static STATUS_ACKS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}
struct Simulated;
impl pd_protocol::pd_spr::Limits for Simulated {
    const SESSION_POLICY: pd_protocol::pd_spr::SessionPolicy = timing::SESSION_POLICY;
    const SINK_IDENTITY: pd_protocol::pd_spr::SinkIdentity =
        pd_protocol::pd_spr::SinkIdentity::UNASSIGNED;
    const MAX_REQUEST_MV: u32 = 20_000;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = 5_000;
    const REQUEST_MA: u32 = 100;
    const MAX_CURRENT_MA: u32 = 5_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = false;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 2;
    const FIXED_TARGETS: &'static [u16] = &[5_000];
    const PPS_REFRESH_MS: u64 = 4_000;
    fn status() -> Option<[u8; 7]> {
        Some([0, 2, 0, OVP.get(), 0, 0, 0])
    }
    fn status_sent(block: [u8; 7]) {
        OVP.set(OVP.get() & !block[3]);
        STATUS_ACKS.set(STATUS_ACKS.get() + 1);
    }
}
struct Never;
impl usbpd::timers::Timer for Never {
    async fn after_millis(_: u64) {
        std::future::pending::<()>().await;
    }
}
struct Peer<'a> {
    selection: Option<&'a pd_protocol::pd_spr::Selection<Simulated>>,
    report_ovp: bool,
    rx: std::collections::VecDeque<std::vec::Vec<u8>>,
    tx: std::vec::Vec<std::vec::Vec<u8>>,
    reject_status: bool,
}
impl usbpd_traits::Driver for Peer<'_> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    async fn wait_for_vbus(&mut self) {}
    async fn transmit_hard_reset(&mut self) -> Result<(), usbpd_traits::DriverTxError> {
        panic!("unexpected Hard Reset in the Alert/Status host fixture")
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), usbpd_traits::DriverTxError> {
        if bytes.len() == 14 && self.reject_status {
            return Err(usbpd_traits::DriverTxError::Discarded);
        }
        self.tx.push(bytes.to_vec());
        Ok(())
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, usbpd_traits::DriverRxError> {
        if let Some(bytes) = self.rx.pop_front() {
            if let Some(selection) = self.selection {
                selection.set_rx_idle(false);
                if self.report_ovp && bytes[0] & 0x1f == 18 {
                    assert!(selection.notify_sink_alert(1 << 30));
                    self.report_ovp = false;
                }
            }
            buffer[..bytes.len()].copy_from_slice(&bytes);
            Ok(bytes.len())
        } else {
            if let Some(selection) = self.selection {
                selection.set_rx_idle(true);
            }
            std::future::pending().await
        }
    }
}

#[test]
fn caller_status_event_flags_clear_only_after_acknowledged_status() {
    use std::{
        future::Future,
        task::{Context, Poll, Waker},
    };
    for reject_status in [false, true] {
        OVP.set(8);
        STATUS_ACKS.set(0);
        let mut peer = Peer {
            selection: None,
            report_ovp: false,
            rx: [
                vec![0xa1, 0x11, 0x2c, 0x91, 1, 0],
                vec![0xa3, 3],
                vec![0xa6, 5],
                vec![0xb2, 7],
            ]
            .into(),
            tx: vec![],
            reject_status,
        };
        let trace = std::cell::RefCell::new(pd_protocol::pd_spr::MaintainedTrace::default());
        let mut trial = pd_protocol::pd_spr::Trial::<Simulated>::maintained();
        {
            let mut engine = std::pin::pin!(trial.run_maintained::<_, Never>(&mut peer, &trace));
            let result = engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()));
            if reject_status {
                assert!(matches!(result, Poll::Ready(Err(_))));
            } else {
                assert!(result.is_pending());
            }
        }
        assert_eq!(STATUS_ACKS.get(), usize::from(!reject_status));
        assert_eq!(OVP.get(), if reject_status { 8 } else { 0 });
        assert_eq!(trace.borrow().statuses_sent, u32::from(!reject_status));
        if !reject_status {
            assert_eq!(
                peer.tx[1],
                [0x82, 0xb2, 7, 0x80, 0, 2, 0, 8, 0, 0, 0, 0, 0, 0]
            );
        }
    }
}

#[test]
fn sending_status_clears_sdb_flags_but_does_not_erase_a_pending_ado_event() {
    use std::{
        future::Future,
        task::{Context, Waker},
    };
    OVP.set(8);
    STATUS_ACKS.set(0);
    let selection = pd_protocol::pd_spr::Selection::<Simulated>::new();
    let mut peer = Peer {
        selection: Some(&selection),
        report_ovp: true,
        rx: [
            vec![0xa1, 0x11, 0x2c, 0x91, 1, 0],
            vec![0xa3, 3],
            vec![0xa6, 5],
            vec![0xb2, 7],
        ]
        .into(),
        tx: vec![],
        reject_status: false,
    };
    let trace = std::cell::RefCell::new(pd_protocol::pd_spr::MaintainedTrace::default());
    let mut trial = pd_protocol::pd_spr::Trial::<Simulated>::maintained();
    {
        let mut engine =
            std::pin::pin!(trial.run_selectable::<_, Never>(&mut peer, &trace, &selection));
        assert!(
            engine
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending()
        );
        assert_eq!(
            selection.pending_sink_alert(),
            0,
            "ADO clears after Alert ACK, not Status ACK"
        );
    }
    assert_eq!(OVP.get(), 0);
    assert_eq!(trace.borrow().statuses_sent, 1);
    assert_eq!(trace.borrow().sink_alerts_sent, 1);
    assert_eq!(peer.tx.len(), 3);
    assert_eq!(peer.tx[2], [0x86, 0x14, 0, 0, 0, 0x40]);
}
