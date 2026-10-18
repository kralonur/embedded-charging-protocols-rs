//! SCP adapter detection and explicit voltage/current output control.
//! Detection is read-only; output control lives in [`control`], FCP's fixed
//! voltages over the same PHY in [`fcp`] and passive decoding of another
//! terminal's traffic in [`sniff`]. No authentication writes.
//!
//! Wire reference: public draft “移动通信终端快速充电技术要求和测试方法”,
//! Appendix B: B.2–B.5 (160 µs UI, sync, odd parity and CRC-8/0x39) and the
//! B.9 type-B command tables:
//! <https://qxb-img-osscache.qixin.com/standards/cffee4628ca228767edbc6482c25ca32.pdf>.
//! This draft is a reference, not a claim of certification.
//!
//! Cross-checked against Huawei's public kernel sources at commit
//! `f99bc558da4d80203fe5a50143f244b27faab1d2`:
//! - register detection, `adapter_protocol_scp.c`:
//!   <https://github.com/Huawei-Dev/android_kernel_huawei_kirin970/blob/f99bc558da4d80203fe5a50143f244b27faab1d2/drivers/hwpower/cc_protocol/adapter_protocol/adapter_protocol_scp/adapter_protocol_scp.c>
//! - register masks, `adapter_protocol_scp.h`:
//!   <https://github.com/Huawei-Dev/android_kernel_huawei_kirin970/blob/f99bc558da4d80203fe5a50143f244b27faab1d2/include/chipset_common/hwpower/protocol/adapter_protocol_scp.h>
//! - reply framing and retries, `sc8545_protocol.c` / `.h`:
//!   <https://github.com/Huawei-Dev/android_kernel_huawei_kirin970/tree/f99bc558da4d80203fe5a50143f244b27faab1d2/drivers/hwpower/cc_hardware_ic/switch_capacitor/sc8545>
//!
//! The caller establishes BC1.2 entry and validates its electrical interface.
//! A ping alone is not SCP identification. Only a valid register response
//! identifying a programmable adapter produces [`Adapter`].
#![no_std]

pub mod control;
pub mod fcp;
pub mod sniff;
mod wire;

/// Nominal bit duration in µs (Appendix B, Table B.4).
pub const UI_US: u32 = 160;

/// Exclusive bidirectional digital data line. The PHY holds D+ at the SCP
/// detection level and maps logical data onto D−. Clocks must be monotonic,
/// drive/sample operations bounded and fast enough to resolve 40 µs sync.
pub trait Transport {
    type Error;
    fn drive(&mut self, high: Option<bool>) -> Result<(), Self::Error>;
    fn sample(&mut self) -> Result<bool, Self::Error>;
    /// Elapsed microseconds from this probe's start. Must not wrap during
    /// detection; a relative 32-bit clock keeps the small MCU PHY efficient.
    fn now_us(&self) -> u32;
    /// Wait at least `us`, without yielding during a transaction.
    fn delay_us(&mut self, us: u32);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error<E> {
    #[error("SCP physical transport failed")]
    Transport(E),
    #[error("SCP timing budget exceeded")]
    Timing,
    #[error("SCP response timeout")]
    Timeout,
    #[error("SCP response ping or sync is invalid")]
    Framing,
    #[error("SCP response parity is invalid")]
    Parity,
    #[error("SCP response CRC is invalid")]
    Crc,
    #[error("SCP register read was rejected")]
    Nack,
    #[error("SCP response acknowledgement is invalid")]
    Ack,
    #[error("adapter did not identify as an SCP programmable source")]
    NotScp,
    #[error("adapter did not report FCP output voltages")]
    NotFcp,
}

impl<E> Error<E> {
    /// Damaged or unanswered traffic, which the terminal retries (B.3.7.1
    /// and B.3.7.2). A NACK is a valid answer and a transport failure is the
    /// caller's to handle.
    fn is_retryable(&self) -> bool {
        matches!(
            self,
            Self::Timing | Self::Timeout | Self::Framing | Self::Parity | Self::Crc | Self::Ack
        )
    }
}

/// Read-only detection and capability registers (Tables B.17–B.19;
/// `MaxIoutExtend` only in `adapter_protocol_scp.h`). No arbitrary addresses
/// or write commands are exposed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    Type0 = 0x7e,
    Type1 = 0x80,
    LegacyType = 0x81,
    MaxPower = 0x90,
    MinVout = 0x92,
    MaxVout = 0x93,
    MinIout = 0x94,
    MaxIout = 0x95,
    VStep = 0x96,
    IStep = 0x97,
    MaxIoutExtend = 0x9c,
}

/// Programmable modes reported by the adapter, not measured power capability.
/// Huawei's driver calls them LVC (low voltage) and SC (high voltage).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modes {
    LowVoltage,
    HighVoltage,
    Both,
}

impl Modes {
    /// Decode Type0 (`0x7e`): `None` unless it is a type-B adapter with at
    /// least one programmable mode.
    fn from_type0(type0: u8) -> Option<Self> {
        if type0 & TYPE_B == 0 {
            return None;
        }
        match (type0 & NO_LOW_VOLTAGE == 0, type0 & HIGH_VOLTAGE != 0) {
            (true, true) => Some(Self::Both),
            (true, false) => Some(Self::LowVoltage),
            (false, true) => Some(Self::HighVoltage),
            (false, false) => None,
        }
    }
}

/// Read-only output voltage, current, step and power ratings (Table B.19).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capabilities {
    pub min_mv: u16,
    pub max_mv: u16,
    pub min_ma: u16,
    pub max_ma: u16,
    pub vstep_mv: Option<u16>,
    pub istep_ma: Option<u16>,
    /// Rated maximum output power in 0.1 W (deciwatts), register `0x90`.
    pub max_power_dw: Option<u16>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Adapter {
    pub modes: Modes,
    pub type0: Option<u8>,
    pub type1: Option<u8>,
    pub capabilities: Option<Capabilities>,
}

/// Type-B flag, bit 4 of both `Type0` and `Type1` (Table B.17).
const TYPE_B: u8 = 1 << 4;
/// `Type0` mode bits. Table B.17 reserves bits 3:2; Huawei's header defines
/// bit 3 as high-voltage support and bit 2 as no low-voltage support
/// (`SCP_PROTOCOL_ADP_TYPE0_B_SC_MASK` and `_B_LVC_MASK`).
const HIGH_VOLTAGE: u8 = 1 << 3;
const NO_LOW_VOLTAGE: u8 = 1 << 2;
/// The only `LegacyType` (`0x81`) value Huawei's driver accepts
/// (`SCP_PROTOCOL_B_ADP_TYPE_B_MASK`), reported as low-voltage mode. Table
/// B.18 also lists 0x80–0x82 as type-B, but those values state no mode, so
/// they are not identified.
const LEGACY_TYPE_B: u8 = 0x10;
/// From this maximum current in mA, `MaxIoutExtend` (`0x9c`) adds
/// `10 * byte` mA (`SCP_PROTOCOL_MAX_IOUT_EXTEND_TH` in
/// `adapter_protocol_scp.h`; `scp_protocol_get_extend_current` scales by 10).
const EXTEND_CURRENT_THRESHOLD_MA: u16 = 6_000;
const EXTEND_CURRENT_STEP_MA: u16 = 10;

/// Detect programmable SCP and read its specification registers when present.
/// Each read is retried on damaged traffic. Type0 falls back to the legacy
/// registers only on a NACK or a valid non-programmable value, never on
/// corrupt traffic or a timeout. D− is released on every exit.
pub fn detect<T: Transport>(phy: &mut T) -> Result<Adapter, Error<T::Error>> {
    let mut detection = Detection::new();
    let result = loop {
        match detection.poll(phy) {
            Ok(None) => {}
            Ok(Some(adapter)) => break Ok(adapter),
            Err(error) => break Err(error),
        }
    };
    phy.drive(None).map_err(Error::Transport)?;
    result
}

/// [`detect`] one register read per [`Detection::poll`], so a caller can
/// keep other work going between reads. D− is released after every read.
/// Required ranges that are NACKed or inconsistent leave the capabilities
/// unknown; optional registers may be NACKed; damaged traffic fails
/// detection rather than being reported as missing data.
#[derive(Clone, Copy, Debug)]
pub struct Detection {
    /// The register the next poll reads.
    next: Register,
    modes: Modes,
    type0: Option<u8>,
    type1: Option<u8>,
    cap: Capabilities,
}

impl Default for Detection {
    fn default() -> Self {
        Self::new()
    }
}

impl Detection {
    pub const fn new() -> Self {
        Self {
            next: Register::Type0,
            modes: Modes::LowVoltage,
            type0: None,
            type1: None,
            cap: Capabilities {
                min_mv: 0,
                max_mv: 0,
                min_ma: 0,
                max_ma: 0,
                vstep_mv: None,
                istep_ma: None,
                max_power_dw: None,
            },
        }
    }

    /// Whether the adapter has answered a read, even with a NACK: a timeout
    /// before then means no SCP adapter is listening.
    pub fn answered(&self) -> bool {
        self.next != Register::Type0
    }

    /// Read the next register: `Some` once the adapter is identified and
    /// its ratings are read or known to be unavailable.
    pub fn poll<T: Transport>(&mut self, phy: &mut T) -> Result<Option<Adapter>, Error<T::Error>> {
        let register = self.next;
        let cap = &mut self.cap;
        self.next = match register {
            Register::Type0 => {
                self.type0 = read_optional(phy, register)?;
                match self.type0.and_then(Modes::from_type0) {
                    Some(modes) => {
                        self.modes = modes;
                        Register::MinVout
                    }
                    None => Register::Type1,
                }
            }
            // Legacy identification: Type1 (`0x80`) flags type-B and
            // `LegacyType` (`0x81`) confirms it.
            Register::Type1 => {
                let type1 = wire::read(phy, register)?;
                if type1 & TYPE_B == 0 {
                    return Err(Error::NotScp);
                }
                self.type1 = Some(type1);
                Register::LegacyType
            }
            Register::LegacyType => {
                if wire::read(phy, register)? != LEGACY_TYPE_B {
                    return Err(Error::NotScp);
                }
                self.modes = Modes::LowVoltage;
                Register::MinVout
            }
            Register::MinVout | Register::MaxVout | Register::MaxIout => {
                let Some(byte) = read_optional(phy, register)? else {
                    return Ok(Some(self.adapter(None)));
                };
                let value = decode_range(byte);
                match register {
                    Register::MinVout => {
                        cap.min_mv = value;
                        Register::MaxVout
                    }
                    Register::MaxVout => {
                        cap.max_mv = value;
                        Register::MinIout
                    }
                    _ => {
                        cap.max_ma = value;
                        if cap.max_mv == 0
                            || cap.max_ma == 0
                            || cap.min_mv > cap.max_mv
                            || cap.min_ma > cap.max_ma
                        {
                            return Ok(Some(self.adapter(None)));
                        }
                        if cap.max_ma >= EXTEND_CURRENT_THRESHOLD_MA {
                            Register::MaxIoutExtend
                        } else {
                            Register::VStep
                        }
                    }
                }
            }
            Register::MinIout => {
                cap.min_ma = read_optional(phy, register)?.map_or(0, decode_range);
                Register::MaxIout
            }
            Register::MaxIoutExtend => {
                if let Some(extra) = read_optional(phy, register)? {
                    cap.max_ma = cap
                        .max_ma
                        .saturating_add(u16::from(extra) * EXTEND_CURRENT_STEP_MA);
                }
                Register::VStep
            }
            Register::VStep => {
                cap.vstep_mv = read_optional(phy, register)?.and_then(decode_step);
                Register::IStep
            }
            Register::IStep => {
                cap.istep_ma = read_optional(phy, register)?.and_then(decode_step);
                Register::MaxPower
            }
            Register::MaxPower => {
                cap.max_power_dw = read_optional(phy, register)?.and_then(decode_step);
                return Ok(Some(self.adapter(Some(self.cap))));
            }
        };
        Ok(None)
    }

    fn adapter(&self, capabilities: Option<Capabilities>) -> Adapter {
        Adapter {
            modes: self.modes,
            type0: self.type0,
            type1: self.type1,
            capabilities,
        }
    }
}

/// `None` when the adapter NACKs the register. Damaged traffic is an error.
fn read_optional<T: Transport>(
    phy: &mut T,
    register: Register,
) -> Result<Option<u8>, Error<T::Error>> {
    match wire::read(phy, register) {
        Ok(value) => Ok(Some(value)),
        Err(Error::Nack) => Ok(None),
        Err(error) => Err(error),
    }
}

/// Table B.19 two-bit decimal exponent: `bits[7:6] = A`, `bits[5:0] = B`,
/// value = `B * 10^A` (in mV or mA).
fn decode_range(byte: u8) -> u16 {
    const TEN_POW: [u16; 4] = [1, 10, 100, 1_000];
    u16::from(byte & 0x3f) * TEN_POW[usize::from(byte >> 6)]
}

/// Table B.19 one-bit decimal exponent: `bit[7] = A`, `bits[6:0] = B`,
/// value = `B * 10^A` (in 0.1 W, mV or mA). Zero means not reported.
fn decode_step(byte: u8) -> Option<u16> {
    let scale = if byte & 0x80 != 0 { 10 } else { 1 };
    let value = u16::from(byte & 0x7f) * scale;
    (value > 0).then_some(value)
}
