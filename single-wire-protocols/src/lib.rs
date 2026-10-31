//! SCP, FCP and AFC over a caller-owned single-wire electrical transport.
//!
//! Each protocol owns its detection and output procedures: [`scp`] provides
//! programmable voltage/current control, [`fcp`] selects advertised fixed
//! voltages, and [`afc`] sends 5 V or 9 V requests. They share [`Transport`],
//! timing guards and error types, not a common transaction format.
//! [`sniff`] passively decodes another terminal's SCP/FCP register traffic.
//!
//! Protocol references are cited in the corresponding modules. Electrical
//! entry, exclusive line ownership, measured VBUS verification and operating
//! limits remain caller duties; a successful exchange is not output proof.
#![no_std]

pub mod afc;
pub mod control;
pub mod fcp;
pub mod scp;
pub mod sniff;
mod wire;

/// Nominal bit duration in µs (Appendix B, Table B.4; see [`scp`]).
pub const UI_US: u32 = 160;

/// Exclusive bidirectional digital data line. The PHY holds D+ at the SCP/FCP
/// detection level, or at 0.6 V for AFC, and maps logical data onto D−.
/// Clocks must be monotonic; drive/sample operations must be bounded and
/// fast enough to resolve 40 µs sync.
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
    #[error("single-wire physical transport failed")]
    Transport(E),
    #[error("single-wire timing budget exceeded")]
    Timing,
    #[error("single-wire response timeout")]
    Timeout,
    #[error("single-wire response ping or sync is invalid")]
    Framing,
    #[error("single-wire response parity is invalid")]
    Parity,
    #[error("single-wire response CRC is invalid")]
    Crc,
    #[error("single-wire register read was rejected")]
    Nack,
    #[error("single-wire response acknowledgement is invalid")]
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
