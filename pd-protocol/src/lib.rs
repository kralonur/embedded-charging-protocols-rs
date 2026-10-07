//! Allocation-free USB PD negotiation, decoding and cable inspection.
//!
//! Callers supply transport, request policy, advertised facts and storage.
//! [`pd_spr::Limits`] and [`pd_discovery::Transport`] define those boundaries.
//! This implements the fixed/PPS and cable-inspection subset, not a complete
//! PD stack.
#![no_std]

#[cfg(test)]
extern crate std;

pub mod pd_cable;
pub mod pd_cable_details;
pub mod pd_cable_modes;
mod pd_constants;
pub mod pd_discovery;
pub mod pd_partner;
pub mod pd_rx;
pub mod pd_sniff;
pub mod pd_spr;
pub mod pd_svids;
