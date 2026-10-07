//! Synthetic caller policy for session wire fixtures, not library defaults.
use pd_protocol::pd_spr as shared;
use usbpd::protocol_layer::message::data::{request, source_capabilities};

pub use shared::{Error, MaintainedTrace, PpsStatus};
#[path = "timing.rs"]
mod timing;
// Named clock inputs used by the session fixtures.
pub const PPS_STATUS_SETTLE_MS: u64 = timing::SESSION_POLICY.pps_status_settle_ms;
pub const QUERY_SETTLE_MS: u64 = timing::SESSION_POLICY.query_settle_ms;
pub const STATUS_MIN_INTERVAL_MS: u64 = timing::SESSION_POLICY.status_min_interval_ms;

// Synthetic 30 V caller ceiling (above SPR to exercise protocol clamping),
// and a 9 V fixture target, both in millivolts.
pub const MAX_REQUEST_MV: u32 = 30_000;
pub const TARGET_MV: u32 = 9_000;
// Synthetic PPS refresh period in milliseconds.
pub const PPS_REFRESH_MS: u64 = 4_000;

pub struct FixtureLimits;
impl shared::Limits for FixtureLimits {
    const SINK_IDENTITY: shared::SinkIdentity = shared::SinkIdentity::UNASSIGNED;
    const SESSION_POLICY: shared::SessionPolicy = timing::SESSION_POLICY;
    const MAX_REQUEST_MV: u32 = MAX_REQUEST_MV;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = 21_000;
    const TARGET_MV: u32 = TARGET_MV;
    const REQUEST_MA: u32 = 100;
    // Synthetic 12 A caller ceiling, in mA, intentionally above protocol limits.
    const MAX_CURRENT_MA: u32 = 12_000;
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    const SINK_POWER_MODES: u8 = 0b10;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 12_000, 15_000, 20_000];
    const PPS_REFRESH_MS: u64 = PPS_REFRESH_MS;
    fn status() -> Option<[u8; 7]> {
        Some([0, 2, 0, 0, 0, 0, 0])
    }
}

pub type Trial = shared::Trial<FixtureLimits>;
pub type Selection = shared::Selection<FixtureLimits>;

pub fn request_pps(
    caps: &source_capabilities::SourceCapabilities,
    object: u8,
    mv: u16,
    ma: u16,
) -> Result<request::PowerSource, Error> {
    shared::request_pps::<FixtureLimits>(caps, object, mv, ma)
}
pub fn request_target(
    caps: &source_capabilities::SourceCapabilities,
) -> Result<request::PowerSource, Error> {
    shared::request_target::<FixtureLimits>(caps)
}
