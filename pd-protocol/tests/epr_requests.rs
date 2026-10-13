//! Prevent overvoltage and stale PDO-copy regressions at the wire boundary.
use pd_protocol::pd_spr::{self, EPR_OBJECTS, Limits, Supply, Target};
use usbpd::protocol_layer::message::data::{request, source_capabilities};

struct Bounded;
impl Limits for Bounded {
    const SINK_IDENTITY: pd_spr::SinkIdentity = pd_spr::SinkIdentity::UNASSIGNED;
    const SESSION_POLICY: pd_spr::SessionPolicy = pd_spr::SessionPolicy {
        pps_status_settle_ms: 300,
        query_settle_ms: 500,
        status_min_interval_ms: 200,
        retry_delay_ms: 100,
        pps_expiry_grace_ms: 5_000,
        pps_status_max_deferrals: 3,
        query_max_deferrals: 3,
    };
    const MAX_REQUEST_MV: u32 = 30_000;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = 9_000;
    const REQUEST_MA: u32 = 100;
    const MAX_CURRENT_MA: u32 = 12_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 2;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 12_000, 15_000, 20_000, 28_000];
    const PPS_REFRESH_MS: u64 = 4_000;
    const EPR: bool = true;
}
fn fixed(mv: u32) -> u32 {
    (mv / 50) << 10 | 500
}
fn objects() -> [u32; EPR_OBJECTS] {
    // Seven SPR positions (zero padded), three EPR fixed objects, 15-48 V/240 W AVS.
    [
        fixed(5_000) | 1 << 23,
        fixed(9_000),
        fixed(20_000),
        0,
        0,
        0,
        0,
        fixed(28_000),
        fixed(36_000),
        fixed(48_000),
        0xd000_0000 | 480 << 17 | 150 << 8 | 240,
    ]
}
fn caps(objects: [u32; EPR_OBJECTS]) -> source_capabilities::SourceCapabilities {
    source_capabilities::SourceCapabilities::new_with_pdos(
        objects
            .into_iter()
            .map(source_capabilities::parse_raw_pdo)
            .collect(),
    )
}
fn target(mv: u16, ma: u16, supply: Supply) -> Target {
    Target { mv, ma, supply }
}
fn wire(request: request::PowerSource) -> Vec<u8> {
    let request::PowerSource::EprRequest(request) = request else {
        panic!("EPR request fixture");
    };
    let mut wire = 0x2089_u16.to_le_bytes().to_vec();
    wire.extend(request.rdo.to_le_bytes());
    wire.extend(request.pdo.to_raw().to_le_bytes());
    wire
}
#[test]
fn fixed_28v_requires_exact_pdo_copy_and_never_selects_36_or_48v() {
    let objects = objects();
    let target = target(28_000, 100, Supply::Fixed);
    let request = pd_spr::request_epr::<Bounded>(&caps(objects), target).unwrap();
    let mut bytes = wire(request);
    assert!(pd_spr::review_epr_request::<Bounded>(
        objects, &bytes, target
    ));
    assert_eq!(u32::from_le_bytes(bytes[2..6].try_into().unwrap()) >> 28, 8);
    bytes[6] ^= 1;
    assert!(!pd_spr::review_epr_request::<Bounded>(
        objects, &bytes, target
    ));
    for mv in [36_000, 48_000] {
        assert!(
            pd_spr::request_epr::<Bounded>(&caps(objects), self::target(mv, 100, Supply::Fixed))
                .is_err()
        );
    }
}
#[test]
fn avs_is_bounded_by_voltage_power_current_and_step() {
    let objects = objects();
    let ma = pd_spr::epr_avs_target_ma::<Bounded>(objects[10], 30_000).unwrap();
    assert_eq!(ma, 5_000);
    let target = target(30_000, ma, Supply::Avs(11));
    let bytes = wire(pd_spr::request_epr::<Bounded>(&caps(objects), target).unwrap());
    assert!(pd_spr::review_epr_request::<Bounded>(
        objects, &bytes, target
    ));
    for mv in [30_100, 29_999, 14_900, 48_000] {
        assert!(
            pd_spr::request_epr::<Bounded>(&caps(objects), self::target(mv, ma, Supply::Avs(11)))
                .is_err()
        );
    }
    let low_pdp = objects[10] & !0xff | 140;
    assert_eq!(
        pd_spr::epr_avs_target_ma::<Bounded>(low_pdp, 30_000),
        Some(4_650)
    );
}
#[test]
fn advertisement_is_zero_padded_and_does_not_claim_overvoltage() {
    let (objects, count) = pd_spr::epr_sink_capabilities::<Bounded>().unwrap();
    assert_eq!(count, 9);
    assert_eq!(objects[7], fixed(28_000));
    assert_eq!((objects[8] >> 17 & 0x1ff) * 100, 30_000);
    // Sink Fixed PDO bits 22..20 are reserved, not EPR capability.
    assert_eq!(objects[0] & 0x0070_0000, 0);
    let block = pd_spr::sink_capabilities_extended::<Bounded>().unwrap();
    assert_eq!(&block[21..], &[1, 3, 150]);
}
