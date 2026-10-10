# embedded-charging-protocols-rs

Allocation-free, `no_std`, hardware-independent charging protocol
implementations in Rust. The library owns protocol logic, message framing,
capability validation and guarded state transitions. It never touches
hardware registers, GPIOs, ADC measurements or storage: the physical transport
(PHY adapter), clocks and timers, operating limits and safety policy belong to
the caller.

> **Note:** only USB Power Delivery is implemented so far, as the
> `pd-protocol` workspace member. Review the implementation and validate it on
> your hardware before relying on it.

## Features

The `pd-protocol` crate (`pd-protocol/`) provides a USB PD sink engine and
packet inspector:

- **Guarded SPR sink negotiation:** fixed and PPS (Programmable Power Supply)
  Standard Power Range negotiation (up to 20 V fixed, 21 V PPS, 100 W).
- **EPR offer on request:** asks an EPR-capable source for its EPR offer and
  collects the answer, for display only.
- **Session safety guards:** independent validation of incoming framing and
  of outgoing requests against the Source Capabilities. Handles SOP
  `Soft_Reset` recovery and the mandatory `Hard_Reset` boundaries without
  uncontrolled retry loops.
- **Cable discovery and inspection:** decodes SOP' and SOP'' structured VDMs,
  passive and active cable identities, e-marker ratings (current, voltage,
  USB speed), and SVID and mode discovery.
- **Passive bus sniffing:** non-intrusive decoding and classification of PD
  traffic for diagnostic displays and logging.
- **Caller-supplied policy:** settle delays, status query intervals, retry
  limits, device identity facts and electrical limits come from the `Limits`
  and `Transport` traits; the library assumes no hidden timing or hardware
  ratings.

## Limitations

- **USB PD only.** Proprietary and legacy fast-charging protocols are planned
  as separate workspace members, not implemented.
- **Sink only:** no source role, and no power role, data role or VCONN swap.
- **No EPR mode:** the EPR offer can be read, but EPR mode is never entered and
  no EPR object is requested.
- **No AVS requests:** AVS objects are decoded for display only.
- **No PHY included:** the caller implements `usbpd-traits` for its PD
  controller and supplies clocks, limits and policy.

## Documentation

- [Caller policy](docs/policy.md): what `Limits` and `SESSION_POLICY` supply.
- [SOP soft reset contract](docs/PD-SOFT-RESET.md).
- [Hard reset boundary](docs/PD-HARD-RESET.md).
- [USB PD pin review](docs/usbpd-review.md): the reviewed `usbpd` revision.

## Build and validation

- Current stable Rust toolchain, Rust 2024 edition, Cargo resolver 3.
- `no_std` across all members, with no heap allocation.
- `usbpd` and `usbpd-traits` are pinned to a reviewed Git revision in
  `Cargo.toml`; no local checkouts or crates.io patches are needed.

```sh
cargo fmt --all -- --check
cargo check --workspace --locked
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo check --workspace --target thumbv7em-none-eabihf --locked
```

## License

This project's original code is dual-licensed under either
[MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option
(`MIT OR Apache-2.0`). Third-party material is subject to its own terms.
