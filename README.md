# Embedded Charging Protocols (`embedded-charging-protocols-rs`)

Allocation-free, `no_std`, hardware-independent embedded charging protocol
implementations in Rust.

This repository is designed as a modular Cargo workspace for reusable charging
protocol state machines and decoders. The library strictly owns protocol logic,
message framing, capability validation, and guarded state transitions. It never
directly touches hardware registers, GPIOs, ADC measurements, or storage; physical
transports (PHY adapters), clocks/timers, operating limits, and safety policies
remain caller-owned.

## Current status

Currently, **only USB Power Delivery (USB PD)** is implemented, provided by the
`pd-protocol` workspace member. Additional charging protocols (such as proprietary
or legacy fast-charging standards) are planned as separate workspace members in
future work.

## `pd-protocol` capabilities

The `pd-protocol` crate (`./pd-protocol`) provides a lightweight, allocation-free
USB PD sink engine and packet inspector:

- **Guarded SPR Sink Negotiation**: Fixed and PPS (Programmable Power Supply)
  Standard Power Range negotiation (up to 20 V fixed, 21 V PPS, 100 W).
- **Session Safety Guards**: Independent validation of incoming framing and
  outgoing RDO requests against Source Capabilities. Handles SOP `Soft_Reset`
  recovery and escalates mandatory `Hard_Reset` protocol boundaries without
  uncontrolled retry loops.
- **Cable Discovery & Inspection**: Decodes SOP' and SOP'' Structured VDMs,
  passive/active cable identities, E-Marker ratings (current limits, voltage
  ratings, USB speed), and SVID/mode discoveries.
- **Passive Bus Sniffing**: Non-intrusive capture decoding and classification of
  PD bus traffic for diagnostic displays and logging.
- **Caller-Supplied Policies**: Settle delays, status query intervals, retry
  limits, device identity facts, and electrical limits are configured via the
  `Limits` and `Transport` traits; no hidden timing or hardware ratings are
  assumed by the library.

## Toolchain and dependencies

- Current stable Rust toolchain (edition 2024), Cargo workspace resolver 3.
- `no_std` compatible across all members (zero heap allocations).
- USB PD dependencies (`usbpd` and `usbpd-traits`) are pinned directly to a reviewed
  Git revision in `Cargo.toml`; no local checkouts or crates.io patches are needed.

## Validation

Run the workspace check, linter, tests, and embedded target verification:

```sh
cargo fmt --all -- --check
cargo check --workspace --locked
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo check --workspace --target thumbv7em-none-eabihf --locked
```

## License

Dual-licensed under either of:

- MIT License (`LICENSE-MIT`)
- Apache License, Version 2.0 (`LICENSE-APACHE`)

at your option.
