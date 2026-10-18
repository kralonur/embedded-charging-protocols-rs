//! FCP: the fixed output voltages an adapter offers over the same PHY and
//! entry as SCP. Registers and sequencing come from Huawei's
//! `adapter_protocol_fcp.c` / `.h` at the crate docs' kernel commit:
//! <https://github.com/Huawei-Dev/android_kernel_huawei_kirin970/blob/f99bc558da4d80203fe5a50143f244b27faab1d2/drivers/hwpower/cc_protocol/adapter_protocol/adapter_protocol_fcp/adapter_protocol_fcp.c>.

use crate::control::{ControlError, RequestError};
use crate::{Error, Transport, wire};

/// `FCP_PROTOCOL_DISCRETE_CAPABILOTIES0`: the index of the highest output
/// voltage register; the driver accepts 1 (5 V, 9 V) or 2 (5 V, 9 V, 12 V).
const CAPABILITIES: u8 = 0x21;
const MAX_CAPABILITY: u8 = 2;
/// `FCP_PROTOCOL_MAX_PWR`, in 500 mW steps.
const MAX_POWER: u8 = 0x22;
const MAX_POWER_STEP_MW: u32 = 500;
/// `FCP_PROTOCOL_ADAPTER_STATUS`: undervoltage, overvoltage, overcurrent
/// and overtemperature flags in bits 3..0, each set by a fault since the
/// last read, which clears them (draft standard, Table B.10).
const STATUS: u8 = 0x28;
const FAULTS: u8 = 0x0f;
/// `FCP_PROTOCOL_OUTPUT_CONTROL`: written after the voltage to apply it.
const OUTPUT_CONTROL: u8 = 0x2b;
const ENABLE: u8 = 1;
/// `FCP_PROTOCOL_VOUT_CONFIG` and `FCP_PROTOCOL_OUTPUT_V0`, both in 100 mV
/// steps; output voltage `n` is at `OUTPUT_V0 + n`.
pub(super) const VOLTAGE: u8 = 0x2c;
pub(super) const STEP_MV: u16 = 100;
const OUTPUT_V0: u8 = 0x30;
/// Output voltages one adapter can offer: indices 0 to [`MAX_CAPABILITY`].
const MAX_LEVELS: usize = MAX_CAPABILITY as usize + 1;

/// An adapter's output voltages, lowest first, and its rated power.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Adapter {
    levels: [u16; MAX_LEVELS],
    len: usize,
    pub max_power_mw: Option<u32>,
}

impl Adapter {
    /// An adapter known to list `levels` in mV: one to three, nonzero and
    /// rising, as detection accepts them.
    pub fn new(levels: &[u16], max_power_mw: Option<u32>) -> Option<Self> {
        if levels.is_empty()
            || levels.len() > MAX_LEVELS
            || levels[0] == 0
            || levels.windows(2).any(|pair| pair[1] <= pair[0])
        {
            return None;
        }
        let mut adapter = Self {
            levels: [0; MAX_LEVELS],
            len: levels.len(),
            max_power_mw,
        };
        adapter.levels[..levels.len()].copy_from_slice(levels);
        Some(adapter)
    }

    /// The output voltages in mV, as the adapter lists them.
    pub fn levels(&self) -> &[u16] {
        &self.levels[..self.len]
    }
}

/// [`Adapter`] detection, one register read per [`Detection::poll`], so a
/// caller can keep other work going between reads. D− is released after
/// every read. A NACK of a required register or an output voltage the
/// driver does not accept means the adapter is not FCP.
#[derive(Clone, Copy, Debug, Default)]
pub struct Detection {
    capability: Option<u8>,
    levels: [u16; MAX_LEVELS],
    len: usize,
}

impl Detection {
    pub const fn new() -> Self {
        Self {
            capability: None,
            levels: [0; MAX_LEVELS],
            len: 0,
        }
    }

    /// Whether the adapter has answered a read, even with a NACK: a timeout
    /// before then means no adapter is listening.
    pub fn answered(&self) -> bool {
        self.capability.is_some()
    }

    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<Option<Adapter>, Error<T::Error>> {
        let Some(capability) = self.capability else {
            let capability = match wire::read_address(phy, CAPABILITIES) {
                Err(Error::Nack) => {
                    // A NACK is still an answer.
                    self.capability = Some(0);
                    return Err(Error::NotFcp);
                }
                result => result?,
            };
            self.capability = Some(capability);
            if capability > MAX_CAPABILITY {
                return Err(Error::NotFcp);
            }
            return Ok(None);
        };
        if self.len <= usize::from(capability) {
            self.levels[self.len] = match wire::read_address(phy, OUTPUT_V0 + self.len as u8) {
                Err(Error::Nack) => return Err(Error::NotFcp),
                result => u16::from(result?) * STEP_MV,
            };
            self.len += 1;
            return Ok(None);
        }
        let max_power_mw = match wire::read_address(phy, MAX_POWER) {
            Err(Error::Nack) => None,
            result => Some(u32::from(result?) * MAX_POWER_STEP_MW).filter(|mw| *mw > 0),
        };
        Adapter::new(&self.levels[..self.len], max_power_mw)
            .map(Some)
            .ok_or(Error::NotFcp)
    }
}

/// A validated output voltage, not evidence that the output followed it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    mv: u16,
}

impl Output {
    /// One of the adapter's output voltages.
    pub fn new(adapter: Adapter, mv: u16) -> Result<Self, RequestError> {
        if !adapter.levels().contains(&mv) {
            return Err(RequestError::Range);
        }
        Ok(Self { mv })
    }

    pub const fn millivolts(self) -> u16 {
        self.mv
    }

    /// The `VOLTAGE` register value; listed voltages come from one byte.
    fn code(self) -> u8 {
        (self.mv / STEP_MV) as u8
    }
}

/// One FCP transaction of a procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    /// Read the status flags only to clear what they hold from before.
    ClearStatus,
    CheckStatus,
    Write,
    Verify,
    Enable,
}

/// An explicitly requested output. Each procedure only queues transactions;
/// [`Session::poll`] runs one per call. Drop does not perform I/O: the
/// caller stops it, or releases D+ on errors, and verifies VBUS itself.
pub struct Session {
    adapter: Adapter,
    output: Output,
    next: Option<Step>,
    /// The queued procedure writes `output` after its status check.
    writing: bool,
}

impl Session {
    /// Queue clearing the status flags, then setting `output`. Faults from
    /// before the session, such as entry's line changes, end nothing; later
    /// checks judge only what happens after this.
    pub fn start(adapter: Adapter, output: Output) -> Result<Self, RequestError> {
        let mut session = Self {
            adapter,
            output,
            next: None,
            writing: false,
        };
        session.queue_set(output)?;
        session.next = Some(Step::ClearStatus);
        Ok(session)
    }

    /// The requested output, written once the queue is empty.
    pub fn output(&self) -> Output {
        self.output
    }

    /// Transactions are still queued.
    pub fn busy(&self) -> bool {
        self.next.is_some()
    }

    pub fn request(&self, mv: u16) -> Result<Output, RequestError> {
        Output::new(self.adapter, mv)
    }

    /// Queue a new output once the previous procedure has finished.
    pub fn set(&mut self, output: Output) -> Result<(), RequestError> {
        if self.busy() {
            return Err(RequestError::Busy);
        }
        self.queue_set(output)
    }

    fn queue_set(&mut self, output: Output) -> Result<(), RequestError> {
        self.output = Output::new(self.adapter, output.mv)?;
        self.next = Some(Step::CheckStatus);
        self.writing = true;
        Ok(())
    }

    /// Queue a status check, which also shows the adapter still listens.
    /// While busy the queued procedure checks it already.
    pub fn refresh(&mut self) {
        if !self.busy() {
            self.next = Some(Step::CheckStatus);
        }
    }

    /// Replace any queued work with setting the lowest output voltage. Once
    /// the queue is empty the caller releases D+ and verifies default VBUS.
    pub fn stop(&mut self) {
        self.output = Output {
            mv: self.adapter.levels[0],
        };
        self.next = Some(Step::Write);
        self.writing = true;
    }

    /// Run the next queued transaction, if any. Any error ends the session.
    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<(), ControlError<T::Error>> {
        let Some(step) = self.next.take() else {
            return Ok(());
        };
        let code = self.output.code();
        self.next = match step {
            Step::ClearStatus => {
                read(phy, STATUS)?;
                Some(Step::Write)
            }
            Step::CheckStatus => {
                if read(phy, STATUS)? & FAULTS != 0 {
                    return Err(ControlError::Protection);
                }
                self.writing.then_some(Step::Write)
            }
            // Writes are never retried: a missing reply may hide an
            // applied write.
            Step::Write => {
                wire::write(phy, VOLTAGE, &[code]).map_err(ControlError::Wire)?;
                Some(Step::Verify)
            }
            Step::Verify => {
                if read(phy, VOLTAGE)? != code {
                    return Err(ControlError::Readback);
                }
                Some(Step::Enable)
            }
            Step::Enable => {
                wire::write(phy, OUTPUT_CONTROL, &[ENABLE]).map_err(ControlError::Wire)?;
                self.writing = false;
                None
            }
        };
        Ok(())
    }
}

fn read<T: Transport>(phy: &mut T, register: u8) -> Result<u8, ControlError<T::Error>> {
    wire::read_address(phy, register).map_err(ControlError::Wire)
}
