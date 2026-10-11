//! The charger's side of D+/D−: what request a pair of measured line
//! voltages makes, for watching a device that drives Quick Charge itself.
//!
//! Qualcomm does not publish the specification. Values come from:
//! - BC1.2: USB Battery Charging Specification Revision 1.2 with errata,
//!   March 15, 2012 (USB-IF): a portable device's primary detection.
//! - CHY100: Power Integrations CHY100 datasheet Rev. E, August 2018: the
//!   VDAT(REF) and VSEL(REF) comparator limits (HVDCP Functions) and the
//!   Quick Charge 2.0 requests (Table 1).
//! - NCP4371: onsemi NCP4371 datasheet Rev. 0, March 2017: the Quick Charge
//!   3.0 continuous mode levels (Table 4).

use crate::qc2::{Level, Voltage, levels};
use crate::qc3::HOLD;

/// A request read from the lines.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// A Quick Charge 2.0 voltage (CHY100 Table 1).
    Fixed(Voltage),
    /// Quick Charge 3.0 continuous mode, between steps (NCP4371 Table 4).
    Continuous,
}

/// What a charger in Quick Charge reads D+ at `dp_mv` and D− at `dm_mv` as;
/// `None` for a pair no request uses, or a line inside a comparator's
/// tolerance (CHY100). A 5 V request (D+ 0.6 V, D− low) looks the same as a
/// portable device's primary detection (BC1.2), so only an entered session
/// makes it one.
pub fn request(dp_mv: u16, dm_mv: u16) -> Option<Request> {
    let pair = (Level::from_mv(dp_mv)?, Level::from_mv(dm_mv)?);
    if pair == HOLD {
        return Some(Request::Continuous);
    }
    Voltage::ALL
        .into_iter()
        .find(|voltage| levels(*voltage) == pair)
        .map(Request::Fixed)
}
