//! Real sink-policy/allowlist exchange: no source operation or role swaps.
use super::*;
use pd_protocol::pd_partner::{Answer, PartnerQuery};

#[test]
fn explicit_port_scan_preserves_five_volts_and_the_automatic_query_order() {
    query_session::<Never>(2, 0, |selection, state, trace, _, poll| {
        // Update the source's advertised DRP flag, preserving the fixed 5 V contract.
        state.borrow_mut().rx.extend([
            packet(1, 3, &[fixed(5000, 3000) | (1 << 29), fixed(9000, 3000)]),
            packet(3, 4, &[]),
            packet(6, 5, &[]),
        ]);
        assert!(poll().is_pending());
        let contract = trace.borrow().contract.as_ref().unwrap().request.0;
        assert!(selection.partner_info().port.is_none());
        assert!(selection.request_port_scan());
        assert!(!selection.request_port_scan(), "no overlapping scans");
        assert!(
            !selection.request_cable(),
            "no VCONN inspection during the scan"
        );
        assert!(selection.preview(pd_protocol::pd_spr::Target {
            mv: 9000,
            ma: 100,
            supply: pd_spr::Supply::Fixed
        }));
        assert!(!selection.confirm(), "no power change during the scan");
        at(state, poll, pd_spr::QUERY_SETTLE_MS);
        assert_eq!(
            query_kind(state.borrow().tx.last().unwrap()),
            Some(PartnerQuery::SinkCapabilities)
        );
        state.borrow_mut().rx.push_back(packet(
            4,
            6,
            &[0x3601_912c, 0x0002_d0c8, 0x8b41_90c8, 0xc0dc_212c],
        ));
        assert!(poll().is_pending());
        assert_eq!(
            query_kind(state.borrow().tx.last().unwrap()),
            Some(PartnerQuery::DiscoverIdentity)
        );
        state
            .borrow_mut()
            .rx
            .push_back(packet(15, 7, &[0xff00_a041, 0xd040_04e8, 0, 0x6860_0000]));
        assert!(poll().is_pending());
        assert_eq!(
            query_kind(state.borrow().tx.last().unwrap()),
            Some(PartnerQuery::SourceCapabilitiesExtended)
        );
        let port = selection.partner_info().port.unwrap();
        assert!(
            matches!(port.sink, Answer::Value(objects) if objects.as_slice() == [0x3601_912c, 0x0002_d0c8, 0x8b41_90c8, 0xc0dc_212c])
        );
        assert!(matches!(port.identity, Answer::Value(identity) if identity.objects().len() == 4));
        assert_eq!(
            trace.borrow().contract.as_ref().unwrap().request.0,
            contract
        );
        assert_eq!(selection.target_mv(), 5000);
    });
}

#[test]
fn source_only_scan_skips_sink_query_and_preserves_busy_as_a_final_result() {
    query_session::<Never>(2, 0, |selection, state, _, _, poll| {
        assert!(selection.request_port_scan());
        at(state, poll, pd_spr::QUERY_SETTLE_MS);
        assert_eq!(
            query_kind(state.borrow().tx.last().unwrap()),
            Some(PartnerQuery::DiscoverIdentity)
        );
        state
            .borrow_mut()
            .rx
            .push_back(packet(15, 3, &[0xff00_a0c1]));
        assert!(poll().is_pending());
        let port = selection.partner_info().port.unwrap();
        assert_eq!(port.sink, Answer::NotSent);
        assert_eq!(port.identity, Answer::Busy);
        assert_eq!(
            query_kind(state.borrow().tx.last().unwrap()),
            Some(PartnerQuery::SourceCapabilitiesExtended)
        );
        assert!(
            !state
                .borrow()
                .tx
                .iter()
                .any(|bytes| query_kind(bytes) == Some(PartnerQuery::SinkCapabilities))
        );
    });
}

#[test]
fn port_scan_refuses_pd2_and_other_voltages_and_reset_interrupts_it() {
    query_session::<Never>(1, 0, |selection, _, _, _, _| {
        assert!(!selection.request_port_scan());
        assert!(selection.partner_info().port.is_none());
    });
    query_session::<Never>(2, 0, |selection, state, _, _, poll| {
        assert!(selection.preview(pd_protocol::pd_spr::Target {
            mv: 9000,
            ma: 100,
            supply: pd_spr::Supply::Fixed
        }));
        assert!(selection.confirm());
        assert!(poll().is_pending());
        state
            .borrow_mut()
            .rx
            .extend([packet(3, 3, &[]), packet(6, 4, &[])]);
        assert!(poll().is_pending());
        assert!(!selection.request_port_scan());
    });
    query_session::<Never>(2, 0, |selection, state, _, _, poll| {
        assert!(selection.request_port_scan());
        at(state, poll, pd_spr::QUERY_SETTLE_MS);
        state.borrow_mut().rx.push_back(packet(13, 0, &[]));
        assert!(poll().is_pending());
        assert_eq!(
            selection.partner_info().port.unwrap().identity,
            Answer::Interrupted
        );
        assert!(!selection.port_query_armed(PartnerQuery::DiscoverIdentity));
    });
}
