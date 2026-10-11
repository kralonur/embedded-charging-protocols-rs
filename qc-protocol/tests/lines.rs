//! Requests read from measured line pairs: a misread level pair would name
//! the wrong voltage for another device's Quick Charge.

use qc_protocol::lines::{Request, request};
use qc_protocol::qc2::Voltage;

/// Nominal levels in mV (CHY100 Table 1): GND, 0.6 V and 3.3 V.
const LOW_MV: u16 = 0;
const MID_MV: u16 = 600;
const HIGH_MV: u16 = 3_300;
/// Inside the VSEL(REF) comparator tolerance, 1.8-2.2 V (CHY100).
const AMBIGUOUS_MV: u16 = 2_000;

#[test]
fn reads_each_request() {
    assert_eq!(request(MID_MV, LOW_MV), Some(Request::Fixed(Voltage::V5)));
    assert_eq!(request(HIGH_MV, MID_MV), Some(Request::Fixed(Voltage::V9)));
    assert_eq!(request(MID_MV, MID_MV), Some(Request::Fixed(Voltage::V12)));
    assert_eq!(
        request(HIGH_MV, HIGH_MV),
        Some(Request::Fixed(Voltage::V20))
    );
    assert_eq!(request(MID_MV, HIGH_MV), Some(Request::Continuous));
}

#[test]
fn rejects_unused_and_ambiguous_pairs() {
    assert_eq!(request(LOW_MV, LOW_MV), None);
    assert_eq!(request(HIGH_MV, LOW_MV), None);
    assert_eq!(request(AMBIGUOUS_MV, MID_MV), None);
}
