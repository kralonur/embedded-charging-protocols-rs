//! Allocation-free USB PD decoding and cable inspection.
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
pub mod pd_svids;
