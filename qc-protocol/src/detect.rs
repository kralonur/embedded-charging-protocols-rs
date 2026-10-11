//! Whether the charger followed a request, judged from its measured output,
//! and a probe of the requests it takes. Quick Charge never acknowledges a
//! request, so the output voltage is the only evidence.

use crate::qc2::{Level, REQUEST_SETTLE_MS, Session, Voltage};
use crate::qc3::{Continuous, STEP_MV, Step};

/// Longest time a power supply may take to reach a new output, in ms: USB PD
/// tPSTransition, SPR maximum (USB Power Delivery Specification Revision
/// 3.2, Version 1.2, Time Values). Quick Charge publishes no such bound; the
/// same supplies meet this one for PD.
const PS_TRANSITION_MS: u64 = 550;
/// A request whose output has not moved past halfway by then was ignored.
const FOLLOW_TIMEOUT_MS: u64 = REQUEST_SETTLE_MS + PS_TRANSITION_MS;

/// What [`Follow::poll`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Waiting,
    /// The output moved past halfway to the requested voltage.
    Followed,
    Ignored,
}

/// Watches the output after one request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Follow {
    from_mv: u16,
    to_mv: u16,
    since_ms: u64,
}

impl Follow {
    /// Start when the request toward `to_mv` is driven, with the output
    /// measured just before at `from_mv`.
    pub fn start(now_ms: u64, from_mv: u16, to_mv: u16) -> Self {
        Self {
            from_mv,
            to_mv,
            since_ms: now_ms,
        }
    }

    /// Feed one output measurement, in mV, taken at `now_ms`.
    pub fn poll(&self, now_ms: u64, mv: u16) -> Outcome {
        let halfway = self.from_mv.midpoint(self.to_mv);
        let moved = if self.to_mv >= self.from_mv {
            mv >= halfway
        } else {
            mv <= halfway
        };
        if moved {
            Outcome::Followed
        } else if now_ms.saturating_sub(self.since_ms) > FOLLOW_TIMEOUT_MS {
            Outcome::Ignored
        } else {
            Outcome::Waiting
        }
    }
}

/// What a probe found. A Quick Charge 2.0 charger takes 9 V; Class A adds
/// 12 V, Class B also 20 V; a Quick Charge 3.0 charger also takes steps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Report {
    pub v9: bool,
    pub v12: bool,
    pub v20: bool,
    pub steps: bool,
}

/// Up steps a probe sends. Three steps (600 mV) put the check's halfway mark
/// 300 mV out, so a first pulse the charger misses (it may wait an unpublished
/// TV_NEW_REQUEST_CONT after the mode change, NCP4371) or slow drift of the
/// output still leaves the result clear.
const STEPS: u8 = 3;
/// The output counts as settled when two measurements differ by less than a
/// quarter step: drift that slow stays well inside the steps' halfway mark.
const SETTLED_MV: u16 = STEP_MV / 4;

/// The request a probe is checking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Continuous mode requested; waiting for the charger to take it and the
    /// output to settle.
    Continuous,
    /// This many [`Step::Up`] pulses sent.
    StepUp(u8),
    /// 5 V requested to leave continuous mode before the fixed requests.
    Back,
    Fixed(Voltage),
}

/// What the caller drives next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Hold these `(D+, D−)` levels.
    Drive((Level, Level)),
    /// Drive `pulse` for [`crate::qc3::PULSE_US`], then hold `hold`.
    Pulse {
        pulse: (Level, Level),
        hold: (Level, Level),
    },
    /// Done: the lines still hold the last request until the caller drives
    /// one in the session from [`Probe::into_session`].
    Finish(Report),
}

/// Requests continuous mode and [`STEPS`] steps up, then 5 V, 9, 12 and
/// 20 V, checking the output after each but 5 V.
///
/// The steps come first, from the settled 5 V of the entered session: every
/// request then raises the output, which the charger drives, while a fall
/// toward a lower request may take seconds without load and would hide the
/// steps. 5 V, the request the session starts from, leaves continuous mode:
/// a charger was seen to ignore a fixed request sent straight from it. 20 V
/// is last, so a Class A charger (which keeps its previous output for it,
/// NCP4371 Table 4) is not left there between checks. The caller measures
/// and polls at a steady pace.
#[derive(Debug, PartialEq, Eq)]
pub struct Probe {
    session: Session,
    stage: Stage,
    follow: Follow,
    continuous: Continuous,
    /// The previous measurement, and the settled output the steps start from.
    last_mv: u16,
    base_mv: u16,
    report: Report,
}

impl Probe {
    /// Start at `now_ms` with the output measured at `mv`.
    pub fn start(session: Session, now_ms: u64, mv: u16) -> (Self, Action) {
        let (continuous, levels) = Continuous::request(&session, Voltage::V5);
        let probe = Self {
            session,
            stage: Stage::Continuous,
            follow: Follow::start(now_ms, mv, mv),
            continuous,
            last_mv: mv,
            base_mv: mv,
            report: Report::default(),
        };
        (probe, Action::Drive(levels))
    }

    pub fn stage(&self) -> Stage {
        self.stage
    }

    /// Feed one output measurement, in mV, taken at `now_ms`; `None` keeps
    /// the lines as they are.
    pub fn poll(&mut self, now_ms: u64, mv: u16) -> Option<Action> {
        let previous_mv = core::mem::replace(&mut self.last_mv, mv);
        match self.stage {
            Stage::Continuous => {
                // The charger takes the mode within the fixed request's
                // filter; a still moving output would blur the steps.
                let waited = now_ms.saturating_sub(self.follow.since_ms);
                if waited < REQUEST_SETTLE_MS || mv.abs_diff(previous_mv) >= SETTLED_MV {
                    return None;
                }
                self.base_mv = mv;
                Some(self.step_up(0, now_ms))
            }
            Stage::StepUp(sent) if sent < STEPS => Some(self.step_up(sent, now_ms)),
            Stage::StepUp(_) => {
                self.report.steps = self.followed(now_ms, mv)?;
                self.stage = Stage::Back;
                self.follow = Follow::start(now_ms, mv, Voltage::V5.millivolts());
                Some(Action::Drive(self.session.request(Voltage::V5)))
            }
            Stage::Back => {
                // Only time: the output may take seconds to fall to 5 V.
                if now_ms.saturating_sub(self.follow.since_ms) < REQUEST_SETTLE_MS {
                    return None;
                }
                Some(self.fixed(Voltage::V9, now_ms, mv))
            }
            Stage::Fixed(voltage) => {
                let followed = self.followed(now_ms, mv)?;
                let next = match voltage {
                    Voltage::V9 => {
                        self.report.v9 = followed;
                        Voltage::V12
                    }
                    Voltage::V12 => {
                        self.report.v12 = followed;
                        Voltage::V20
                    }
                    // 20 V, the last request; the probe never requests a
                    // fixed 5 V, which leaves continuous mode in `Back`.
                    Voltage::V20 | Voltage::V5 => {
                        self.report.v20 = followed;
                        return Some(Action::Finish(self.report));
                    }
                };
                Some(self.fixed(next, now_ms, mv))
            }
        }
    }

    /// Whether the output followed the watched request; `None` while that is
    /// still open.
    fn followed(&self, now_ms: u64, mv: u16) -> Option<bool> {
        match self.follow.poll(now_ms, mv) {
            Outcome::Waiting => None,
            outcome => Some(outcome == Outcome::Followed),
        }
    }

    /// Request `voltage` and watch the output from `mv`.
    fn fixed(&mut self, voltage: Voltage, now_ms: u64, mv: u16) -> Action {
        self.stage = Stage::Fixed(voltage);
        self.follow = Follow::start(now_ms, mv, voltage.millivolts());
        Action::Drive(self.session.request(voltage))
    }

    /// Send the step after `sent`, then watch for all [`STEPS`] from the
    /// settled output; the watch restarts with each pulse.
    fn step_up(&mut self, sent: u8, now_ms: u64) -> Action {
        let rise_mv = u16::from(STEPS) * STEP_MV;
        self.stage = Stage::StepUp(sent + 1);
        self.follow = Follow::start(now_ms, self.base_mv, self.base_mv.saturating_add(rise_mv));
        Action::Pulse {
            pulse: self.continuous.pulse(Step::Up),
            hold: self.continuous.hold(),
        }
    }

    /// End the probe, keeping the entered session for new requests.
    pub fn into_session(self) -> Session {
        self.session
    }
}
