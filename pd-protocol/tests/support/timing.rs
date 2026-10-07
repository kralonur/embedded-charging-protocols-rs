//! Scheduling inputs for scripted regression peers, not library defaults.
use pd_protocol::pd_spr::SessionPolicy;

// The regression clock uses a 300 ms PPS query delay, a 500 ms initial query
// delay, 200 ms between status reads, 100 ms cooldown, 5 s PPS expiry grace,
// and at most three unsent attempts. These are synthetic test inputs.
pub const SESSION_POLICY: SessionPolicy = SessionPolicy {
    pps_status_settle_ms: 300,
    query_settle_ms: 500,
    status_min_interval_ms: 200,
    retry_delay_ms: 100,
    pps_expiry_grace_ms: 5_000,
    pps_status_max_deferrals: 3,
    query_max_deferrals: 3,
};
