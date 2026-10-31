//! Regression: AFC must not accidentally use SCP framing. This scheduled
//! source uses Samsung's ping-only receive path, not the SCP source simulator.
//! Golden transmit edges include the parity extension and immediate release.
use single_wire_protocols::afc::{Adapter, Detection, Session, Voltage, request, reset};
use single_wire_protocols::{Error, Transport, control::RequestError};
use std::ops::Range;

// Independent reference timing from afc_charger_intf.c/.h, in microseconds.
const UI: u32 = 160;
const QUARTER: u32 = 40;
const MPING: u32 = 2_560;
const SKIP: u32 = 2_000;
const FINAL_GAP: u32 = 200;
const RESET: u32 = 16_000;
// Conservative implementation guard: 100 UI released before reset.
const RECOVERY: u32 = 16_000;

type Edge = (u32, Option<bool>);

#[derive(Default)]
struct Source {
    now: u32,
    level: Option<bool>,
    pulses: Vec<Range<u32>>,
    driven: Vec<Edge>,
    fail_drive: Option<usize>,
    sample_cost: u32,
    stretch_sync: bool,
    stall_tx_once: bool,
    stall_rx_once: bool,
}

impl Source {
    fn scheduled(width: u32, response_delay: u32) -> Self {
        let first = MPING + response_delay;
        // First slave ping, UI idle, 3 quarter-UI start pulses, 8 bits,
        // parity + quarter-UI extension, 2 quarter-UI end pulses, MPING.
        let second = first + width + UI + 3 * QUARTER + 9 * UI + 3 * QUARTER + MPING + UI;
        let third = second + width + SKIP + UI;
        let fourth = third + width + FINAL_GAP + MPING + UI;
        Self {
            pulses: [first, second, third, fourth]
                .map(|start| start..start + width)
                .into(),
            ..Self::default()
        }
    }

    fn assert_released(&self) {
        assert_eq!(self.level, None);
        assert_eq!(self.driven.last().map(|(_, level)| *level), Some(None));
    }
}

impl Transport for Source {
    type Error = ();

    fn drive(&mut self, level: Option<bool>) -> Result<(), ()> {
        let index = self.driven.len();
        self.driven.push((self.now, level));
        // A failed drive may have changed the output: cleanup must release it.
        self.level = level;
        if self.fail_drive == Some(index) {
            return Err(());
        }
        assert!(level.is_none() || !self.pulses.iter().any(|pulse| pulse.contains(&self.now)));
        Ok(())
    }

    fn sample(&mut self) -> Result<bool, ()> {
        assert_eq!(self.level, None, "sample only with the master released");
        self.now += self.sample_cost;
        Ok(self.pulses.iter().any(|pulse| pulse.contains(&self.now)))
    }

    fn now_us(&self) -> u32 {
        self.now
    }

    fn delay_us(&mut self, us: u32) {
        // Inject one scheduler pause, either while transmitting the initial
        // ping or before the first receive sample. Later retries are silent.
        if (self.stall_tx_once && self.level == Some(true))
            || (self.stall_rx_once && self.level.is_none() && self.now >= MPING)
        {
            self.stall_tx_once = false;
            self.stall_rx_once = false;
            // A 1 ms pause exceeds both the transmit and sample budgets.
            self.now += 1_000;
        }
        self.now += us;
        if self.stretch_sync && us == QUARTER && self.level.is_some() {
            // Exceeds the shared physical timing guard's 4 µs sync slack.
            self.now += 5;
        }
    }
}

fn golden(source: &Source, bits: [bool; 8]) -> Vec<Edge> {
    let mut edges = vec![(0, Some(true)), (MPING, Some(false)), (MPING, None)];
    let first_end = source.pulses[0].end;
    let mut at = first_end + UI;
    for high in [true, false, true] {
        edges.push((at, Some(high)));
        at += QUARTER;
    }
    for high in bits {
        edges.push((at, Some(high)));
        at += UI;
    }
    // Both reference requests have an odd number of ones: parity is low.
    edges.push((at, Some(false)));
    at += UI;
    for high in [false, true, false] {
        edges.push((at, Some(high)));
        at += QUARTER;
    }
    edges.extend([
        (at, Some(true)),
        (at + MPING, Some(false)),
        (at + MPING, None),
    ]);
    at = source.pulses[2].end + FINAL_GAP;
    edges.extend([
        (at, Some(true)),
        (at + MPING, Some(false)),
        (at + MPING, None),
    ]);
    edges.push((source.pulses[3].end, None));
    edges
}

#[test]
fn requests_match_reference_edges_without_requiring_a_reply_byte() {
    for (voltage, bits) in [
        (
            Voltage::V5,
            [false, false, false, false, true, false, false, false],
        ),
        (
            Voltage::V9,
            [false, true, false, false, false, true, true, false],
        ),
    ] {
        let mut source = Source::scheduled(16 * UI, UI);
        let expected = golden(&source, bits);
        assert_eq!(request(&mut source, voltage), Ok(()));
        assert_eq!(source.driven, expected);
        source.assert_released();
    }
}

#[test]
fn accepts_afc_ping_limits_and_the_seventh_response_sample() {
    // These widths are valid AFC but rejected by SCP's 2304–2816 µs gate.
    // The reference checks high before its timeout counter: sample seven
    // must still be accepted, not mistaken for an absent charger.
    for width in [5 * UI, 20 * UI] {
        for response_delay in [UI, 6 * UI, 7 * UI] {
            let mut source = Source::scheduled(width, response_delay);
            assert_eq!(request(&mut source, Voltage::V5), Ok(()));
            source.assert_released();
        }
    }
}

#[test]
fn reference_ui_polling_ignores_a_short_turnaround_high() {
    let mut source = Source::scheduled(16 * UI, UI);
    // The reference waits one UI before sampling after each master ping.
    // A short high during bus turnaround is not a slave response.
    source.pulses.push(MPING + 1..MPING + QUARTER);
    assert_eq!(request(&mut source, Voltage::V5), Ok(()));
    source.assert_released();

    let mut silent = Source {
        pulses: std::iter::once(MPING + 1..MPING + QUARTER).collect(),
        ..Source::default()
    };
    assert_eq!(request(&mut silent, Voltage::V5), Err(Error::Timeout));
    silent.assert_released();
}

#[test]
fn reference_width_counter_accepts_a_fractional_five_count_ping() {
    let mut source = Source::scheduled(16 * UI, UI);
    // afc_recv_Sping counts delayed samples, not exact microseconds: a
    // pulse ending just after count four is accepted at count five.
    source.pulses[0].end = source.pulses[0].start + 4 * UI + 1;
    // Later replies follow the command sent after observing that fifth
    // sample, eleven UI earlier than the nominal sixteen-count fixture.
    for pulse in &mut source.pulses[1..] {
        pulse.start -= 11 * UI;
        pulse.end -= 11 * UI;
    }
    assert_eq!(request(&mut source, Voltage::V5), Ok(()));
    source.assert_released();
}

#[test]
fn reference_postincrement_accepts_the_twenty_first_width_count() {
    let mut source = Source::scheduled(21 * UI, UI);
    assert_eq!(request(&mut source, Voltage::V5), Ok(()));
    source.assert_released();
}

#[test]
fn each_slave_ping_must_still_pass_the_reference_count_limits() {
    for ping in 0..4 {
        for width in [4 * UI, 22 * UI] {
            let mut source = Source::scheduled(16 * UI, UI);
            source.pulses.truncate(ping + 1);
            source.pulses[ping].end = source.pulses[ping].start + width;
            assert_eq!(request(&mut source, Voltage::V5), Err(Error::Framing));
            source.assert_released();
        }
    }
}

#[test]
fn response_data_is_skipped_and_the_third_ping_may_already_be_high() {
    let mut source = Source::scheduled(16 * UI, UI);
    let second_end = source.pulses[1].end;
    // Arbitrary data pulses during the unread interval, not a framed byte.
    source.pulses.extend([
        second_end + 40..second_end + 80,
        second_end + 120..second_end + 200,
    ]);
    // After the 2 ms skip, part of the third ping has already elapsed.
    source.pulses[2].start = second_end + 1_600;
    assert_eq!(request(&mut source, Voltage::V9), Ok(()));
    source.assert_released();
}

#[test]
fn silent_sources_retry_but_partial_or_invalid_replies_never_become_silence() {
    let mut silent = Source::default();
    assert_eq!(request(&mut silent, Voltage::V5), Err(Error::Timeout));
    assert_eq!(
        silent
            .driven
            .iter()
            .filter(|(_, level)| *level == Some(true))
            .count(),
        3
    );
    silent.assert_released();

    for first_width in [4 * UI, 16 * UI, 21 * UI] {
        let mut partial = Source::scheduled(16 * UI, UI);
        partial.pulses.truncate(1);
        partial.pulses[0].end = partial.pulses[0].start + first_width;
        // The following attempts are entirely silent. Earlier activity must
        // still leave support unverified, not turn it into "NO AFC".
        assert_eq!(request(&mut partial, Voltage::V5), Err(Error::Framing));
        partial.assert_released();
    }
}

#[test]
fn local_timing_failures_survive_later_silent_retries() {
    let mut source = Source {
        stall_tx_once: true,
        ..Source::default()
    };
    assert_eq!(request(&mut source, Voltage::V5), Err(Error::Timing));
    source.assert_released();
}

#[test]
fn receive_preemption_cannot_silently_shift_ping_sample_counts() {
    let mut source = Source::scheduled(16 * UI, UI);
    source.pulses.truncate(1);
    source.stall_rx_once = true;
    assert_eq!(request(&mut source, Voltage::V5), Err(Error::Timing));
    // Only the three retrying master pings; never send command data based
    // on the first response's shortened, shifted sample count.
    assert_eq!(
        source
            .driven
            .iter()
            .filter(|(_, level)| *level == Some(true))
            .count(),
        3
    );
    source.assert_released();
}

#[test]
fn transport_and_timing_failures_release_the_line() {
    for fail_drive in [0, 3] {
        let mut source = Source::scheduled(16 * UI, UI);
        source.fail_drive = Some(fail_drive);
        assert_eq!(request(&mut source, Voltage::V9), Err(Error::Transport(())));
        source.assert_released();
    }
    let mut slow = Source {
        sample_cost: 9,
        ..Source::default()
    };
    assert_eq!(request(&mut slow, Voltage::V5), Err(Error::Timing));
    slow.assert_released();

    let mut stretched = Source::scheduled(16 * UI, UI);
    // No command or closing master ping follows the damaged first sync.
    stretched.pulses.truncate(1);
    stretched.stretch_sync = true;
    assert_eq!(request(&mut stretched, Voltage::V5), Err(Error::Timing));
    stretched.assert_released();
}

#[test]
fn reset_uses_a_100_ui_pulse_and_releases_even_after_a_drive_error() {
    let mut source = Source {
        // A previous aborted response is still driving the bus. Reset must
        // leave it alone until the recovery interval has elapsed.
        pulses: std::iter::once(0..8_000).collect(),
        ..Source::default()
    };
    assert_eq!(reset(&mut source), Ok(()));
    assert_eq!(
        source.driven,
        [
            (0, None),
            (RECOVERY, Some(true)),
            (RECOVERY + RESET, Some(false)),
            (RECOVERY + RESET, None),
        ]
    );
    source.assert_released();
    for fail_drive in [0, 1, 2] {
        let mut source = Source {
            fail_drive: Some(fail_drive),
            ..Source::default()
        };
        assert_eq!(reset(&mut source), Err(Error::Transport(())));
        source.assert_released();
    }
}

#[test]
fn detection_and_session_only_send_explicitly_queued_requests() {
    let mut detection = Detection::new();
    assert!(!detection.answered());
    let mut source = Source::scheduled(16 * UI, UI);
    assert_eq!(detection.poll(&mut source), Ok(Some(Adapter)));
    assert!(detection.answered());
    assert_eq!(
        source.driven,
        golden(
            &source,
            [false, false, false, false, true, false, false, false]
        )
    );

    let mut session = Session::start(Adapter, Voltage::V9);
    assert_eq!(session.set(Voltage::V5), Err(RequestError::Busy));
    let mut source = Source::scheduled(16 * UI, UI);
    session.poll(&mut source).unwrap();
    assert!(!session.busy());
    let count = source.driven.len();
    session.poll(&mut source).unwrap();
    assert_eq!(source.driven.len(), count);
    session.stop();
    assert_eq!(session.output(), Voltage::V5);
    assert!(session.busy());
    let mut source = Source::scheduled(16 * UI, UI);
    session.poll(&mut source).unwrap();
    assert_eq!(
        source.driven,
        golden(
            &source,
            [false, false, false, false, true, false, false, false]
        )
    );
}
