//! Samsung AFC: fixed-voltage requests over D−, with D+ held at 0.6 V.
//!
//! Nominal intervals and request bytes are based on `afc_charger_intf.c` / `.h`:
//! <https://github.com/gta7lite/android_kernel_samsung_mt6765/blob/de3c7f5140460fd449a970613f94f7c16c7f579c/drivers/power/supply/afc_charger_intf.c>.
//! This is an implementation reference, not a normative AFC specification.
//! Its delay macro is platform-tuned; this PHY uses the nominal UI, not
//! the reference's shortened `ndelay` calls or its platform overhead.
//!
//! AFC shares the electrical transport and timing guards with SCP/FCP, not
//! their transaction framing. The reference driver checks four slave pings
//! and skips response data for 2 ms rather than decoding it. Success does
//! not establish voltage/current capabilities: the caller must verify VBUS.

use crate::control::{ControlError, RequestError};
use crate::{Error, Transport, UI_US, wire};

/// Master ping length in UI (`afc_send_Mping`).
const MPING_UI: u32 = 16;
/// `afc_recv_Sping` checks the input before `WAIT_SPING_COUNT < i++`.
/// With that count set to 5 and i initially 0, a high seventh sample is
/// accepted. Allow that observed 7-UI window, not an inferred 5-UI limit.
const RESPONSE_UI: u32 = 7;
/// Slave ping sample counts (`SPING_MIN_UI` / `SPING_MAX_UI`).
/// The reference increments before delaying and tests `MAX < count++`,
/// so a falling level at count 21 is still accepted.
const SPING_MIN_UI: u32 = 5;
const SPING_MAX_UI: u32 = 20;
/// Unread response-data interval in µs (`afc_recv_data`).
const RESPONSE_SKIP_US: u32 = 2_000;
/// Gap in µs before the last master ping (`afc_communication`).
const FINAL_GAP_US: u32 = 200;
/// Reset high interval in UI (`afc_reset`), independent of setpoint replies.
const RESET_UI: u32 = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Voltage {
    V5,
    V9,
}

impl Voltage {
    pub const ALL: [Self; 2] = [Self::V5, Self::V9];

    /// Nominal output in mV.
    pub const fn millivolts(self) -> u16 {
        match self {
            Self::V5 => 5_000,
            Self::V9 => 9_000,
        }
    }

    pub fn from_millivolts(mv: u16) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|voltage| voltage.millivolts() == mv)
    }

    /// Established requests from `afc_communication`. The low nibble is
    /// not a measured current or permission to draw that current.
    const fn byte(self) -> u8 {
        match self {
            Self::V5 => 0x08,
            Self::V9 => 0x46,
        }
    }
}

/// Ask for `voltage`, retrying damaged or unanswered traffic. D+ must
/// already be at 0.6 V after BC1.2 entry. D− is released on every exit.
/// Only measured VBUS can confirm that the output followed.
pub fn request<T: Transport>(phy: &mut T, voltage: Voltage) -> Result<(), Error<T::Error>> {
    // Preserve evidence across retries: a later timeout must not turn a
    // damaged/partial exchange into a claim that the charger was silent.
    let mut answered = false;
    let mut timing_failed = false;
    wire::retried(phy, |phy| {
        let result = exchange(phy, voltage.byte(), &mut answered);
        phy.drive(None).map_err(Error::Transport)?;
        match result {
            Err(Error::Timing) => {
                // A local timing failure is not evidence of an adapter
                // response; later silence must not disguise its cause.
                timing_failed = true;
                Err(Error::Timing)
            }
            Err(Error::Timeout) if timing_failed => Err(Error::Timing),
            Err(Error::Timeout) if answered => Err(Error::Framing),
            result => result,
        }
    })
}

/// Return to the default output using the reference driver's 100-UI reset
/// pulse. First leave a recovery interval released so an aborted response
/// can finish without contention. No reply is required. D− is released on
/// every exit; the caller still must verify VBUS and release D+.
pub fn reset<T: Transport>(phy: &mut T) -> Result<(), Error<T::Error>> {
    phy.drive(None).map_err(Error::Transport)?;
    phy.delay_us(wire::RECOVERY_UI * UI_US);
    let result = wire::hold(phy, true, RESET_UI * UI_US)
        .and_then(|()| phy.drive(Some(false)).map_err(Error::Transport));
    phy.drive(None).map_err(Error::Transport)?;
    result
}

fn exchange<T: Transport>(
    phy: &mut T,
    byte: u8,
    answered: &mut bool,
) -> Result<(), Error<T::Error>> {
    ping(phy)?;
    receive_ping(phy, answered)?;
    send(phy, byte)?;
    ping(phy)?;
    receive_ping(phy, answered)?;
    // The reference driver does not require a decodable reply byte.
    phy.delay_us(RESPONSE_SKIP_US);
    receive_ping(phy, answered)?;
    phy.delay_us(FINAL_GAP_US);
    ping(phy)?;
    receive_ping(phy, answered)
}

fn ping<T: Transport>(phy: &mut T) -> Result<(), Error<T::Error>> {
    wire::hold(phy, true, MPING_UI * UI_US)?;
    // Unlike SCP, AFC does not actively hold D− low for another UI:
    // release immediately so an early slave response cannot contend.
    phy.drive(Some(false)).map_err(Error::Transport)?;
    phy.drive(None).map_err(Error::Transport)
}

fn receive_ping<T: Transport>(phy: &mut T, answered: &mut bool) -> Result<(), Error<T::Error>> {
    // afc_recv_Sping delays before every sample, including the first.
    // Immediate edge polling can mistake a turnaround high or response-data
    // tail for a ping. Its width gate counts UI samples, not microseconds.
    for _ in 0..RESPONSE_UI {
        if sample_ping(phy)? {
            *answered = true;
            // Also follows the reference when the third ping is already
            // high after the response-data skip: count only what remains.
            for count in 1..=SPING_MAX_UI + 1 {
                if !sample_ping(phy)? {
                    return if count < SPING_MIN_UI {
                        Err(Error::Framing)
                    } else {
                        Ok(())
                    };
                }
            }
            return Err(Error::Framing);
        }
    }
    Err(Error::Timeout)
}

/// Keep UI-spaced polling, but reject preemption rather than silently
/// counting a stretched interval as one UI. The lateness budget is the
/// shared PHY's implementation guard, not an AFC specification limit.
fn sample_ping<T: Transport>(phy: &mut T) -> Result<bool, Error<T::Error>> {
    wire::wait_until(phy, phy.now_us() + UI_US)?;
    wire::sample(phy)
}

fn send<T: Transport>(phy: &mut T, byte: u8) -> Result<(), Error<T::Error>> {
    // `afc_send_data` starts one UI after the first slave ping falls.
    phy.delay_us(UI_US);
    wire::hold(phy, true, UI_US / 4)?;
    wire::hold(phy, false, UI_US / 4)?;
    if byte & 0x80 == 0 {
        wire::hold(phy, true, UI_US / 4)?;
    }
    for bit in (0..8).rev() {
        wire::hold(phy, byte & (1 << bit) != 0, UI_US)?;
    }
    let parity = wire::parity_bit(byte);
    wire::hold(phy, parity, UI_US)?;
    // AFC extends parity by ¼ UI, then sends high/low ¼-UI pulses.
    // This is not SCP's level-dependent end sync.
    wire::hold(phy, parity, UI_US / 4)?;
    wire::hold(phy, true, UI_US / 4)?;
    wire::hold(phy, false, UI_US / 4)
}

/// Evidence of a complete 5 V request exchange, not a capability rating.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Adapter;

/// Non-raising detection by a complete 5 V request exchange.
#[derive(Clone, Copy, Debug, Default)]
pub struct Detection {
    adapter: Option<Adapter>,
}

impl Detection {
    pub const fn new() -> Self {
        Self { adapter: None }
    }

    /// Whether a complete exchange has identified the adapter. Partial or
    /// damaged responses return framing/timing errors, never silent timeout.
    pub fn answered(&self) -> bool {
        self.adapter.is_some()
    }

    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<Option<Adapter>, Error<T::Error>> {
        request(phy, Voltage::V5)?;
        self.adapter = Some(Adapter);
        Ok(self.adapter)
    }
}

/// An explicitly requested output. Changes only queue a request; the caller
/// polls it, verifies VBUS, and uses [`reset`] on shutdown or output failure.
/// Drop performs no I/O.
pub struct Session {
    output: Voltage,
    pending: bool,
}

impl Session {
    pub fn start(_adapter: Adapter, output: Voltage) -> Self {
        Self {
            output,
            pending: true,
        }
    }

    pub fn output(&self) -> Voltage {
        self.output
    }

    pub fn busy(&self) -> bool {
        self.pending
    }

    /// Queue a new output once the previous request has been sent.
    pub fn set(&mut self, output: Voltage) -> Result<(), RequestError> {
        if self.busy() {
            return Err(RequestError::Busy);
        }
        self.output = output;
        self.pending = true;
        Ok(())
    }

    /// Replace queued work with a 5 V request. This does not replace the
    /// caller's reset/voltage verification on shutdown.
    pub fn stop(&mut self) {
        self.output = Voltage::V5;
        self.pending = true;
    }

    /// Send the queued request, if any. Any error ends the session.
    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<(), ControlError<T::Error>> {
        if self.pending {
            request(phy, self.output).map_err(ControlError::Wire)?;
            self.pending = false;
        }
        Ok(())
    }
}
