//! Handshake outcomes against synthetic D− traces: the timing decides
//! whether a real charger's output changes, so a regression here is costly.

mod common;

use common::{LOW_MV, RELEASE_MS, SCAN_MS, SHORTED_MV, quick_charge_dm_mv};
use qc_protocol::qc2::{Failure, Handshake, Progress};

/// The D+ High Glitch Filter Time range, in ms (CHY100 TGLITCH(BC)DONE).
const FILTER_MIN_MS: u64 = 1_000;
const FILTER_MAX_MS: u64 = 1_500;

/// Feed `dm_mv(t)` every [`SCAN_MS`] from 0 until the handshake decides.
fn run(dm_mv: impl Fn(u64) -> u16) -> (u64, Progress) {
    let mut handshake = Handshake::start(0);
    let mut now = 0;
    loop {
        match handshake.poll(now, dm_mv(now)) {
            Progress::Waiting(next) => {
                handshake = next;
                now += SCAN_MS;
            }
            done => return (now, done),
        }
    }
}

/// The first scan at or after `ms`.
fn scan_from(ms: u64) -> u64 {
    ms.div_ceil(SCAN_MS) * SCAN_MS
}

/// The first scan after `ms`.
fn scan_past(ms: u64) -> u64 {
    (ms / SCAN_MS).saturating_add(1) * SCAN_MS
}

#[test]
fn quick_charge_charger_enters_after_glitch_filter() {
    let (at, progress) = run(quick_charge_dm_mv);
    assert!(matches!(progress, Progress::Ready(_)));
    // One scan sees D− low, the next sees it stayed low.
    assert_eq!(at, scan_past(scan_from(RELEASE_MS)));
}

#[test]
fn plain_dcp_never_enters() {
    let (at, progress) = run(|_| SHORTED_MV);
    assert_eq!(progress, Progress::Failed(Failure::NoEntry));
    // The first scan past the filter's maximum.
    assert_eq!(at, scan_past(FILTER_MAX_MS));
}

#[test]
fn undriven_d_minus_is_not_a_dcp() {
    let (at, progress) = run(|_| LOW_MV);
    assert_eq!(progress, Progress::Failed(Failure::NotDcp));
    // Decided once the filter's minimum passed without the short.
    assert_eq!(at, scan_from(FILTER_MIN_MS));
}
