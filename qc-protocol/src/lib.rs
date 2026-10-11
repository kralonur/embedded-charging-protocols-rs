//! Allocation-free Quick Charge sink side: the entry handshake, request
//! levels, Quick Charge 3.0 steps, and checks of what the charger followed.
//!
//! The caller drives and measures D+, D− and the output; this crate only says
//! which level each line needs and what the measurements mean. Times are
//! caller-supplied milliseconds from any monotonic clock.
#![no_std]

pub mod detect;
pub mod lines;
pub mod qc2;
pub mod qc3;
