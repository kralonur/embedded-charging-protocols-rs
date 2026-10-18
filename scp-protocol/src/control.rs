//! Explicit output control, separate from read-only detection.
//! Register definitions and sequencing: Appendix B Tables B.21/B.27 and
//! Huawei's `scp_protocol_set_init_data` / `set_default_state` (crate docs).

use crate::{Adapter, Capabilities, Error, Modes, Transport, wire};

/// Control/status and setpoint addresses (Appendix B Tables B.21–B.27).
const CONTROL: u8 = 0xa0;
const TIMER: u8 = 0xa1;
const FAULTS: u8 = 0xa3;
const VOLTAGE_BOUND: u8 = 0xb0;
const CURRENT_BOUND: u8 = 0xb2;
pub(super) const VOLTAGE: u8 = 0xb8;
const CURRENT: u8 = 0xba;
pub(super) const SHORT_VOLTAGE: u8 = 0xca;
const SHORT_CURRENT: u8 = 0xcb;
/// Control byte 0: output on, protocol mode, D+ protection disabled,
/// reset, ACK reporting and port-overtemperature protection, respectively.
const ENABLE: u8 = 1 << 7;
const MODE: u8 = 1 << 6;
const NO_DP_PROTECTION: u8 = 1 << 4;
const RESET: u8 = 1 << 5;
const ACK_REPORT: u8 = 1 << 3;
const TEMPERATURE_PROTECTION: u8 = 1;
/// Control byte 1: preserve only D+ deglitch bits; use a 2 s watchdog
/// (four 0.5 s units). The caller must refresh well before this expires.
const DEGLITCH: u8 = 0x18;
const WATCHDOG: u8 = 4;
/// Status byte 1: temperature, port temperature, overcurrent, overvoltage
/// and undervoltage faults. Communication CRC/parity flags are separate.
const PROTECTION_FAULTS: u8 = 0x1f;
/// Legacy one-byte voltage uses (mV - 3000) / 10, through 5500 mV;
/// one-byte current uses mA / 50 (Huawei VSSET/ISSET definitions).
pub(super) const SHORT_OFFSET_MV: u16 = 3_000;
const SHORT_MAX_MV: u16 = 5_500;
pub(super) const SHORT_STEP_MV: u16 = 10;
const SHORT_STEP_MA: u16 = 50;

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RequestError {
    #[error("complete SCP ratings are unavailable")]
    MissingRatings,
    #[error("SCP request is outside the advertised range")]
    Range,
    #[error("SCP request is not representable at the advertised step")]
    Step,
    #[error("SCP request exceeds the advertised power")]
    Power,
    #[error("a previous SCP procedure is still running")]
    Busy,
}

/// Validated setpoint, not evidence that the output followed it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    mv: u16,
    ma: u16,
}

impl Output {
    pub fn new(adapter: Adapter, mv: u16, ma: u16) -> Result<Self, RequestError> {
        let cap = adapter.capabilities.ok_or(RequestError::MissingRatings)?;
        let vstep = cap
            .vstep_mv
            .filter(|v| *v > 0)
            .ok_or(RequestError::MissingRatings)?;
        let istep = cap
            .istep_ma
            .filter(|v| *v > 0)
            .ok_or(RequestError::MissingRatings)?;
        let power = cap.max_power_dw.ok_or(RequestError::MissingRatings)?;
        if mv == 0
            || ma == 0
            || !(cap.min_mv..=cap.max_mv).contains(&mv)
            || !(cap.min_ma..=cap.max_ma).contains(&ma)
            || (adapter.modes == Modes::LowVoltage && mv > SHORT_MAX_MV)
        {
            return Err(RequestError::Range);
        }
        if !(mv - cap.min_mv).is_multiple_of(vstep)
            || !(ma - cap.min_ma).is_multiple_of(istep)
            || (mv <= SHORT_MAX_MV
                && (mv < SHORT_OFFSET_MV || !(mv - SHORT_OFFSET_MV).is_multiple_of(SHORT_STEP_MV)))
            || (adapter.modes == Modes::LowVoltage
                && (!ma.is_multiple_of(SHORT_STEP_MA) || ma / SHORT_STEP_MA > u16::from(u8::MAX)))
        {
            return Err(RequestError::Step);
        }
        // mV * mA is µW; each advertised deciwatt is 100,000 µW.
        if u32::from(mv) * u32::from(ma) > u32::from(power) * 100_000 {
            return Err(RequestError::Power);
        }
        Ok(Self { mv, ma })
    }

    pub const fn millivolts(self) -> u16 {
        self.mv
    }
    pub const fn milliamps(self) -> u16 {
        self.ma
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ControlError<E> {
    #[error("SCP control transaction failed: {0}")]
    Wire(Error<E>),
    #[error("invalid SCP output request: {0}")]
    Request(RequestError),
    #[error("SCP control readback mismatch")]
    Readback,
    #[error("SCP output protection or watchdog reset")]
    Protection,
}

/// Most single-transaction steps one procedure queues: a start's control
/// and timer writes (2 each, with readback) and starting current (3), then
/// its set: three checks and five writes of up to two bytes, each read
/// back (3 + 5 * 3). `set` refuses while busy, so nothing queues on top.
const QUEUE_LEN: usize = 2 + 2 + 3 + 3 + 5 * 3;

/// One SCP transaction of a control procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    /// Start: check the control byte allows protocol mode, then keep it.
    CaptureControl,
    /// Start: keep the deglitch bits of the timer, then queue the writes.
    CaptureTimer,
    /// Refresh: the control byte, the watchdog and the fault flags.
    CheckControl,
    CheckTimer,
    CheckFaults,
    /// Write one or two bytes; a `Verify` per byte follows.
    Write {
        register: u8,
        bytes: [u8; 2],
        len: u8,
    },
    Verify {
        register: u8,
        expected: u8,
    },
}

/// An explicitly enabled output session. Each procedure only queues
/// transactions; [`Session::poll`] runs one per call, so a caller can keep
/// other work going between them. Drop does not perform I/O: the caller
/// must stop it, or release D+ on errors. D+ protection and the finite
/// watchdog must remain enabled; never retain the drive after an error.
/// Physical VBUS verification and platform limits are caller duties.
pub struct Session {
    adapter: Adapter,
    output: Output,
    control: u8,
    timer: u8,
    queue: [Step; QUEUE_LEN],
    head: usize,
    len: usize,
}

impl Session {
    /// Queue the start: read the control and timer bytes, enable protocol
    /// mode with D+ and temperature protection kept, set the finite
    /// watchdog, start from the smallest current, then set `output`.
    pub fn start(adapter: Adapter, output: Output) -> Result<Self, RequestError> {
        Output::new(adapter, output.mv, output.ma)?;
        let mut session = Self {
            adapter,
            output,
            control: 0,
            timer: 0,
            queue: [Step::CheckControl; QUEUE_LEN],
            head: 0,
            len: 0,
        };
        session.push(Step::CaptureControl);
        session.push(Step::CaptureTimer);
        Ok(session)
    }

    /// The requested setpoint, written once the queue is empty.
    pub fn output(&self) -> Output {
        self.output
    }

    /// Transactions are still queued.
    pub fn busy(&self) -> bool {
        self.len > 0
    }

    pub fn request(&self, mv: u16, ma: u16) -> Result<Output, RequestError> {
        Output::new(self.adapter, mv, ma)
    }

    /// Queue a new setpoint once the previous procedure has finished.
    pub fn set(&mut self, output: Output) -> Result<(), RequestError> {
        if self.busy() {
            return Err(RequestError::Busy);
        }
        self.queue_set(output)
    }

    fn queue_set(&mut self, output: Output) -> Result<(), RequestError> {
        Output::new(self.adapter, output.mv, output.ma)?;
        self.queue_checks();
        // Reduce current before changing voltage; increase it only afterward.
        // This prevents an old high voltage / new high current combination
        // from exceeding the power of either validated endpoint.
        self.current(self.output.ma.min(output.ma));
        self.write(CURRENT_BOUND, &output.ma.to_be_bytes());
        self.write(VOLTAGE_BOUND, &output.mv.to_be_bytes());
        if output.mv <= SHORT_MAX_MV {
            let value = ((output.mv - SHORT_OFFSET_MV) / SHORT_STEP_MV) as u8;
            self.write(SHORT_VOLTAGE, &[value]);
        } else {
            self.write(VOLTAGE, &output.mv.to_be_bytes());
        }
        self.current(output.ma);
        self.output = output;
        Ok(())
    }

    /// Queue a communication refresh that verifies output mode and
    /// protection. Queue one well within the two-second watchdog, even
    /// when the setpoint is unchanged. While busy the queued traffic
    /// already refreshes, so nothing is added.
    pub fn refresh(&mut self) {
        if !self.busy() {
            self.queue_checks();
        }
    }

    fn queue_checks(&mut self) {
        self.push(Step::CheckControl);
        self.push(Step::CheckTimer);
        self.push(Step::CheckFaults);
    }

    /// Replace any queued work with restoring ordinary output mode, without
    /// cutting output enable. Once the queue is empty the caller releases
    /// D+ and verifies actual default VBUS.
    pub fn stop(&mut self) {
        self.len = 0;
        self.queue_checks();
        self.write(
            CONTROL,
            &[self.control & !(MODE | NO_DP_PROTECTION | RESET)],
        );
    }

    /// Run the next queued transaction, if any. Any error ends the session.
    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<(), ControlError<T::Error>> {
        let Some(step) = self.pop() else {
            return Ok(());
        };
        match step {
            Step::CaptureControl => {
                let control = read(phy, CONTROL)?;
                if control & ENABLE == 0 || control & (MODE | RESET) != 0 {
                    return Err(ControlError::Protection);
                }
                // Preserve ACK reporting, but explicitly keep D+ and
                // temperature protection enabled. No reset or reserved bits.
                self.control = (control & (ENABLE | ACK_REPORT)) | MODE | TEMPERATURE_PROTECTION;
            }
            Step::CaptureTimer => {
                self.timer = (read(phy, TIMER)? & DEGLITCH) | WATCHDOG;
                self.queue_start()?;
            }
            Step::CheckControl if read(phy, CONTROL)? != self.control => {
                return Err(ControlError::Protection);
            }
            Step::CheckTimer if read(phy, TIMER)? & !DEGLITCH != WATCHDOG => {
                return Err(ControlError::Protection);
            }
            Step::CheckFaults if read(phy, FAULTS)? & PROTECTION_FAULTS != 0 => {
                return Err(ControlError::Protection);
            }
            Step::CheckControl | Step::CheckTimer | Step::CheckFaults => {}
            Step::Write {
                register,
                bytes,
                len,
            } => wire::write(phy, register, &bytes[..usize::from(len)])
                .map_err(ControlError::Wire)?,
            Step::Verify { register, expected } => {
                if read(phy, register)? != expected {
                    return Err(ControlError::Readback);
                }
            }
        }
        Ok(())
    }

    /// After both captures: enable protocol mode and the watchdog, start
    /// from the smallest representable advertised current before raising
    /// a boundary or voltage, then set the requested output.
    fn queue_start<E>(&mut self) -> Result<(), ControlError<E>> {
        let cap = self
            .adapter
            .capabilities
            .ok_or(ControlError::Request(RequestError::MissingRatings))?;
        let minimum =
            minimum_current(self.adapter, cap).ok_or(ControlError::Request(RequestError::Step))?;
        self.write(CONTROL, &[self.control]);
        self.write(TIMER, &[self.timer]);
        self.current(minimum);
        let output = self.output;
        self.output = Output {
            mv: output.mv,
            ma: minimum,
        };
        self.queue_set(output).map_err(ControlError::Request)
    }

    fn current(&mut self, ma: u16) {
        if self.adapter.modes == Modes::LowVoltage {
            // `Output::new` keeps low-voltage currents within one byte.
            self.write(SHORT_CURRENT, &[(ma / SHORT_STEP_MA) as u8]);
        } else {
            self.write(CURRENT, &ma.to_be_bytes());
        }
    }

    /// Queue a write and a readback per byte. Writes are never retried:
    /// a missing reply may hide an applied write.
    fn write(&mut self, register: u8, bytes: &[u8]) {
        let mut buffer = [0; 2];
        buffer[..bytes.len()].copy_from_slice(bytes);
        self.push(Step::Write {
            register,
            bytes: buffer,
            len: bytes.len() as u8,
        });
        for (offset, &expected) in bytes.iter().enumerate() {
            self.push(Step::Verify {
                register: register + offset as u8,
                expected,
            });
        }
    }

    fn push(&mut self, step: Step) {
        // `set` refuses while busy, so the longest queue is one start
        // (see `QUEUE_LEN`); overflow is a bug in this module.
        assert!(self.len < QUEUE_LEN, "SCP control queue overflow");
        self.queue[(self.head + self.len) % QUEUE_LEN] = step;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<Step> {
        if self.len == 0 {
            return None;
        }
        let step = self.queue[self.head];
        self.head = (self.head + 1) % QUEUE_LEN;
        self.len -= 1;
        Some(step)
    }
}

fn minimum_current(adapter: Adapter, cap: Capabilities) -> Option<u16> {
    let step = cap.istep_ma?;
    (cap.min_ma..=cap.max_ma).find(|ma| {
        *ma > 0
            && (*ma - cap.min_ma).is_multiple_of(step)
            && (adapter.modes != Modes::LowVoltage || ma.is_multiple_of(SHORT_STEP_MA))
    })
}

fn read<T: Transport>(phy: &mut T, register: u8) -> Result<u8, ControlError<T::Error>> {
    wire::read_address(phy, register).map_err(ControlError::Wire)
}
