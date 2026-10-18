//! Read-only SCP adapter detection. No output control or authentication writes.
//!
//! Wire reference: public draft “移动通信终端快速充电技术要求和测试方法”,
//! Appendix B, B.2–B.5 (160 µs UI, sync, odd parity and CRC-8/0x39):
//! <https://qxb-img-osscache.qixin.com/standards/cffee4628ca228767edbc6482c25ca32.pdf>.
//! This draft is a reference, not a claim of certification.
//! Register detection is cross-checked against Huawei's public
//! `adapter_protocol_scp.c` and `adapter_protocol_scp.h`:
//! <https://github.com/Huawei-Dev/android_kernel_huawei_kirin970/tree/main/drivers/hwpower/cc_protocol/adapter_protocol/adapter_protocol_scp>.
//!
//! The caller establishes BC1.2 entry and validates its electrical interface.
//! A ping alone is not SCP identification. Only a valid register response
//! identifying a programmable adapter produces [`Adapter`].
#![no_std]

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

/// Read-only detection and capability registers from the public adapter
/// protocol headers (Table B.18 and Table B.19). No arbitrary addresses or
/// write commands are exposed.
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modes {
    LowVoltage,
    HighVoltage,
    Both,
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

/// Type-B flag in the type registers (public protocol headers).
const TYPE_B: u8 = 1 << 4;
/// Type0 high-voltage mode is active-high; low-voltage exclusion is active-high.
const HIGH_VOLTAGE: u8 = 1 << 3;
const NO_LOW_VOLTAGE: u8 = 1 << 2;
/// Threshold in mA where `MaxIoutExtend` (`0x9c`) adds `10 * byte` mA
/// (`SCP_PROTOCOL_MAX_IOUT_EXTEND_TH` in `adapter_protocol_scp.h`).
const EXTEND_CURRENT_THRESHOLD_MA: u16 = 6_000;
const EXTEND_CURRENT_STEP_MA: u16 = 10;

/// Detect programmable SCP and read its specification registers when present.
/// Each read is retried on damaged traffic. Type0 falls back to the legacy
/// registers only on a NACK or a valid non-programmable value, never on
/// corrupt traffic or a timeout. D− is released on every exit.
pub fn detect<T: Transport>(phy: &mut T) -> Result<Adapter, Error<T::Error>> {
    let result = identify(phy);
    phy.drive(None).map_err(Error::Transport)?;
    result
}

fn identify<T: Transport>(phy: &mut T) -> Result<Adapter, Error<T::Error>> {
    let type0 = match wire::read(phy, Register::Type0) {
        Ok(value) => Some(value),
        Err(Error::Nack) => None,
        Err(error) => return Err(error),
    };
    if let Some(value) = type0.filter(|value| value & TYPE_B != 0) {
        let high = value & HIGH_VOLTAGE != 0;
        let low = value & NO_LOW_VOLTAGE == 0;
        let modes = match (low, high) {
            (true, true) => Some(Modes::Both),
            (true, false) => Some(Modes::LowVoltage),
            (false, true) => Some(Modes::HighVoltage),
            (false, false) => None,
        };
        if let Some(modes) = modes {
            return Ok(Adapter {
                modes,
                type0,
                type1: None,
                capabilities: read_capabilities(phy),
            });
        }
    }
    let type1 = wire::read(phy, Register::Type1)?;
    if type1 & TYPE_B == 0 || wire::read(phy, Register::LegacyType)? != TYPE_B {
        return Err(Error::NotScp);
    }
    Ok(Adapter {
        modes: Modes::LowVoltage,
        type0,
        type1: Some(type1),
        capabilities: read_capabilities(phy),
    })
}

fn read_capabilities<T: Transport>(phy: &mut T) -> Option<Capabilities> {
    let min_mv = decode_range_byte(wire::read(phy, Register::MinVout).ok()?)?;
    let max_mv = decode_range_byte(wire::read(phy, Register::MaxVout).ok()?)?;
    let min_ma = wire::read(phy, Register::MinIout)
        .ok()
        .and_then(decode_range_byte)
        .unwrap_or(0);
    let mut max_ma = decode_range_byte(wire::read(phy, Register::MaxIout).ok()?)?;
    if min_mv > max_mv || min_ma > max_ma {
        return None;
    }
    if max_ma >= EXTEND_CURRENT_THRESHOLD_MA
        && let Ok(extra) = wire::read(phy, Register::MaxIoutExtend)
    {
        max_ma = max_ma.saturating_add(u16::from(extra).saturating_mul(EXTEND_CURRENT_STEP_MA));
    }
    let vstep_mv = wire::read(phy, Register::VStep)
        .ok()
        .and_then(decode_step_byte);
    let istep_ma = wire::read(phy, Register::IStep)
        .ok()
        .and_then(decode_step_byte);
    let max_power_dw = wire::read(phy, Register::MaxPower)
        .ok()
        .and_then(decode_step_byte);
    Some(Capabilities {
        min_mv,
        max_mv,
        min_ma,
        max_ma,
        vstep_mv,
        istep_ma,
        max_power_dw,
    })
}

/// Table B.19 two-bit decimal exponent: `bits[7:6] = A`, `bits[5:0] = B`,
/// value = `B * 10^A` (in mV or mA).
fn decode_range_byte(byte: u8) -> Option<u16> {
    const TEN_POW: [u16; 4] = [1, 10, 100, 1_000];
    let exp = usize::from(byte >> 6);
    let mantissa = u16::from(byte & 0x3f);
    (mantissa > 0).then(|| mantissa * TEN_POW[exp])
}

/// Table B.19 one-bit decimal exponent: `bit[7] = A`, `bits[6:0] = B`,
/// value = `B * 10^A` (in 0.1 W, mV or mA).
fn decode_step_byte(byte: u8) -> Option<u16> {
    let scale = if byte & 0x80 != 0 { 10 } else { 1 };
    let mantissa = u16::from(byte & 0x7f);
    (mantissa > 0).then(|| mantissa * scale)
}
