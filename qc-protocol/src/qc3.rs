//! Quick Charge 3.0 continuous mode, sink (portable device) side.
//!
//! Qualcomm does not publish the specification. Values come from:
//! - NCP4371: onsemi NCP4371 datasheet Rev. 0, March 2017, a Quick Charge 3.0
//!   charger controller ("HVDCP"): Table 4 and "HVDCP Mode – Continuous
//!   Mode".
//! - CHY103: Power Integrations CHY103 datasheet Rev. G, June 2016, a Quick
//!   Charge 3.0 charger interface IC: Table 1 and the electrical
//!   characteristics.
//!
//! From an entered Quick Charge session the device holds D+ at 0.6 V and D−
//! at 3.3 V. The charger then raises its output one step on each rising edge
//! of D+ (a pulse to 3.3 V) and lowers it one step on each falling edge of D−
//! (a pulse to 0.6 V), staying between [`MIN_MV`] and its class maximum
//! (NCP4371). A Quick Charge 2.0 charger does not take steps.

use crate::qc2::{Level, Session, Voltage};

/// Output change per pulse, in mV (NCP4371; CHY103 Table 1).
pub const STEP_MV: u16 = 200;
/// Lowest continuous mode output, in mV (NCP4371 Figure 17; CHY103).
pub const MIN_MV: u16 = 3_600;
/// Width of a pulse and of the hold after it, in µs. The datasheets name the
/// device's times (NCP4371 TACTIVE and TINACTIVE) without values, so this is
/// five times the 200 µs maximum Continuous Mode Glitch Filter Time
/// TGLITCH(CONT)CHANGE, which the charger must see through, and a twentieth of
/// the 20 ms minimum TGLITCH(V)CHANGE (CHY103): a pulse's levels also code a
/// fixed request (3.3 V/3.3 V is 20 V, 0.6 V/0.6 V is 12 V), which must not
/// be read as one.
pub const PULSE_US: u32 = 1_000;

/// One step of the output voltage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Up,
    Down,
}

/// Levels between pulses (NCP4371 Table 4; CHY103 Table 1).
pub(crate) const HOLD: (Level, Level) = (Level::Mid, Level::High);

/// Proof continuous mode was requested in an entered session, so steps can
/// be sent, and the output they have set.
#[derive(Debug, PartialEq, Eq)]
pub struct Continuous {
    set_mv: u16,
}

impl Continuous {
    /// `(D+, D−)` levels requesting continuous mode in `session`, whose
    /// output was set to `from`: the charger steps from there. It filters
    /// the request like a fixed one, so steps may follow once
    /// [`crate::qc2::REQUEST_SETTLE_MS`] has passed (NCP4371).
    pub fn request(_session: &Session, from: Voltage) -> (Self, (Level, Level)) {
        let continuous = Self {
            set_mv: from.millivolts(),
        };
        (continuous, HOLD)
    }

    /// The output the steps sent so far set, in mV. A charger stops at its
    /// class maximum, which the caller keeps targets under.
    pub fn set_mv(&self) -> u16 {
        self.set_mv
    }

    /// `(D+, D−)` levels for the [`Self::pulse`] moving the output one step
    /// toward `target_mv` (no lower than [`MIN_MV`]), counted as sent;
    /// `None` within a step of it.
    pub fn step_toward(&mut self, target_mv: u16) -> Option<(Level, Level)> {
        let target_mv = target_mv.max(MIN_MV);
        let (step, set_mv) = if target_mv >= self.set_mv.saturating_add(STEP_MV) {
            (Step::Up, self.set_mv + STEP_MV)
        } else if target_mv.saturating_add(STEP_MV) <= self.set_mv {
            (Step::Down, self.set_mv - STEP_MV)
        } else {
            return None;
        };
        self.set_mv = set_mv;
        Some(self.pulse(step))
    }

    /// `(D+, D−)` levels to hold between pulses.
    pub fn hold(&self) -> (Level, Level) {
        HOLD
    }

    /// `(D+, D−)` levels to drive for [`PULSE_US`] for one `step`, then
    /// return to [`Self::hold`] for at least as long.
    pub fn pulse(&self, step: Step) -> (Level, Level) {
        match step {
            Step::Up => (Level::High, Level::High),
            Step::Down => (Level::Mid, Level::Mid),
        }
    }
}
