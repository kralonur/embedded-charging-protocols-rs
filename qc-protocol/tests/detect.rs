//! Probe reports against simulated chargers: the report says which voltages
//! a charger takes, so a wrong one misleads whoever loads it next.

mod common;

use common::{SCAN_MS, quick_charge_dm_mv};
use qc_protocol::detect::{Action, Probe, Report};
use qc_protocol::qc2::{Handshake, Level, Progress, Session, Voltage};
use qc_protocol::qc3::STEP_MV;

/// Without load a simulated output falls this fraction of the way to its set
/// level per scan, so a fall takes seconds, as observed.
const FALL_DIVISOR: u16 = 50;
/// A probe still running after this long, in ms, never finishes.
const PROBE_BOUND_MS: u64 = 30_000;

/// An entered session, from a Quick Charge charger answering the handshake.
fn session() -> Session {
    let mut handshake = Handshake::start(0);
    let mut now = 0;
    loop {
        match handshake.poll(now, quick_charge_dm_mv(now)) {
            Progress::Waiting(next) => handshake = next,
            Progress::Ready(session) => return session,
            Progress::Failed(failure) => panic!("handshake failed: {failure:?}"),
        }
        now += SCAN_MS;
    }
}

/// A charger's output in mV for held levels (CHY100 Table 1): `class_b`
/// takes 20 V, `steps` takes continuous mode pulses and then leaves that mode
/// only for 5 V. A Class A charger keeps its output for 3.3 V/3.3 V (NCP4371
/// Table 4). Without load the output falls slowly ([`FALL_DIVISOR`]).
struct Charger {
    class_b: bool,
    steps: bool,
    continuous: bool,
    set_mv: u16,
    mv: u16,
}

impl Charger {
    fn new(class_b: bool, steps: bool) -> Self {
        let mv = Voltage::V5.millivolts();
        Self {
            class_b,
            steps,
            continuous: false,
            set_mv: mv,
            mv,
        }
    }

    /// Quick Charge 2.0, Class A.
    fn qc2_class_a() -> Self {
        Self::new(false, false)
    }

    /// Quick Charge 3.0, Class A.
    fn qc3_class_a() -> Self {
        Self::new(false, true)
    }

    /// Quick Charge 3.0, Class B.
    fn qc3_class_b() -> Self {
        Self::new(true, true)
    }

    fn drive(&mut self, levels: (Level, Level)) {
        if self.continuous && levels != (Level::Mid, Level::Low) {
            return;
        }
        self.continuous = false;
        self.set_mv = match levels {
            (Level::Mid, Level::Low) => Voltage::V5.millivolts(),
            (Level::High, Level::Mid) => Voltage::V9.millivolts(),
            (Level::Mid, Level::Mid) => Voltage::V12.millivolts(),
            (Level::High, Level::High) if self.class_b => Voltage::V20.millivolts(),
            (Level::Mid, Level::High) => {
                self.continuous = true;
                self.set_mv
            }
            _ => self.set_mv,
        };
    }

    fn step_up(&mut self) {
        if self.steps && self.continuous {
            self.set_mv += STEP_MV;
        }
    }

    /// One scan passes.
    fn settle(&mut self) {
        self.mv = if self.set_mv >= self.mv {
            self.set_mv
        } else {
            self.mv - (self.mv - self.set_mv) / FALL_DIVISOR
        };
    }
}

/// Run a probe every scan against `charger` until it reports.
fn probe(mut charger: Charger) -> Report {
    let mut now = 0;
    let (mut probe, action) = Probe::start(session(), now, charger.mv);
    let mut action = Some(action);
    loop {
        match action {
            Some(Action::Drive(levels)) => charger.drive(levels),
            Some(Action::Pulse { .. }) => charger.step_up(),
            Some(Action::Finish(report)) => return report,
            None => {}
        }
        now += SCAN_MS;
        charger.settle();
        assert!(now < PROBE_BOUND_MS, "probe never finished");
        action = probe.poll(now, charger.mv);
    }
}

#[test]
fn class_a_quick_charge_2_takes_9_and_12_volts() {
    let report = probe(Charger::qc2_class_a());
    assert_eq!(
        report,
        Report {
            v9: true,
            v12: true,
            v20: false,
            steps: false,
        }
    );
}

#[test]
fn class_a_quick_charge_3_steps_with_a_slow_falling_output() {
    let report = probe(Charger::qc3_class_a());
    assert_eq!(
        report,
        Report {
            v9: true,
            v12: true,
            v20: false,
            steps: true,
        }
    );
}

#[test]
fn class_b_quick_charge_3_takes_everything() {
    let report = probe(Charger::qc3_class_b());
    assert_eq!(
        report,
        Report {
            v9: true,
            v12: true,
            v20: true,
            steps: true,
        }
    );
}
