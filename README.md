# embedded-charging-protocols-rs

Allocation-free, `no_std`, hardware-independent charging protocol
implementations in Rust. The library owns protocol logic, message framing,
capability validation and guarded state transitions. It never touches
hardware registers, GPIOs, ADC measurements or storage: the physical transport
(PHY adapter), clocks and timers, operating limits and safety policy belong to
the caller.

> **Note:** USB Power Delivery (`pd-protocol`) and Quick Charge 2.0 and 3.0
> (`qc-protocol`) are implemented, with SCP and FCP adapter identification
> and output control (`scp-protocol`). Review the implementation and validate
> it on your hardware before relying on it.

## Features

The `pd-protocol` crate (`pd-protocol/`) provides a USB PD sink engine and
packet inspector:

- **Guarded SPR sink negotiation:** fixed, PPS (Programmable Power Supply)
  and, when the caller turns it on, SPR AVS (Adjustable Voltage Supply)
  Standard Power Range negotiation (up to 20 V fixed, 21 V PPS, 9-20 V AVS,
  100 W).
- **Source information:** extended capabilities, revision, manufacturer,
  source info and status, and, for a battery-powered source, its battery's
  capacity and charge.
- **EPR offer on request:** asks an EPR-capable source for its EPR offer and
  collects the answer without changing the contract.
- **Opt-in EPR sink sessions:** caller-bounded fixed and AVS requests, explicit
  confirmation, entry failure reporting, keep-alive and exit through an SPR
  contract. Cable qualification and electrical protection remain caller duties.
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

The `qc-protocol` crate (`qc-protocol/`) provides the Quick Charge 2.0 and
3.0 sink side, and reads requests another device drives:

- **Entry handshake:** times the D+ hold and checks the D− response, telling a
  Quick Charge charger from a plain dedicated charging port or no charger.
  Measured voltages are read against the charger's own comparator limits.
- **Requests:** the D+/D− levels for 5, 9, 12 and 20 V, available only after a
  completed handshake.
- **Quick Charge 3.0 steps:** continuous mode and the 200 mV up/down pulses.
- **Output checks:** whether the measured output followed a request, and a
  probe that reports which voltages, class and steps a charger takes.
- **Request reading:** the Quick Charge 2.0 voltage or 3.0 continuous mode a
  pair of measured D+/D− voltages asks for, as a charger reads it, for
  watching a device that drives Quick Charge itself.

The `scp-protocol` crate provides SCP and FCP identification and output
control:

- Read-only detection of whitelisted adapter-type and capability registers.
- Explicit voltage/current output control (`control`): requests are checked
  against the adapter's ranges, steps and rated power; every write is
  acknowledged and read back, never retried; protections stay enabled and a
  finite adapter watchdog drops the output if the caller stops refreshing.
  One transaction per poll, so a caller stays responsive. No authentication.
- FCP over the same PHY (`fcp`): the adapter's listed output voltages and
  rated power, and setting one of them, written, read back, then applied.
  The adapter's latched fault flags are cleared when a session starts, then
  checked before each change and on every refresh.
- Checked master/slave ping sequencing, source-clock synchronization,
  quarter-UI framing, odd parity, CRC-8 and ACK/NACK validation.
- Modern programmable-mode identification and the two-register legacy
  fallback. A ping alone does not establish SCP support.
- Caller-owned electrical entry, GPIO drive/receive and microsecond clock.
  Damaged or unanswered reads get up to three attempts, each from an idle
  bus; all exits release the data line.
- Passive decoding (`sniff`) of another terminal's traffic from D− edge
  timestamps alone: each byte is re-timed from its own bit periods, parity
  and CRC are checked, and commands are paired with their ACK/NACK replies.
  A transaction names its protocol (FCP or SCP register) and the output
  voltage an accepted write requested. Damaged traffic is dropped until the
  next ping; nothing drives the line.
- Public draft physical-layer and public driver references are cited in
  `scp-protocol/src/lib.rs`. They are references, not certification claims.

## Limitations

- **SCP and FCP are not hardware-validated.** Simulated wire tests establish encoding
  and failure behavior, not compatibility with a particular adapter or, for
  passive decoding, phone.
  Electrical entry, timing and default-output checks remain caller duties.
- Other proprietary and legacy fast-charging protocols are planned as
  separate workspace members.
- **Quick Charge from public sources:** Qualcomm does not publish the
  specification. Levels and timings come from the USB Battery Charging
  Specification 1.2 and Quick Charge charger IC datasheets, cited in
  `qc-protocol/src/qc2.rs`, `qc-protocol/src/qc3.rs` and
  `qc-protocol/src/lines.rs`. They give no
  device-side pulse widths, so the crate picks one inside the charger's
  filter limits.
- **Sink only:** no source role, and no power role, data role or VCONN swap.
- **EPR is disabled by default:** callers must enable it, supply truthful
  limits and implement native message authorization and safe reset handling.
- **No PHY included:** the caller implements `usbpd-traits` for its PD
  controller and supplies clocks, limits and policy.

## Documentation

- [Caller policy](docs/policy.md): what `Limits` and `SESSION_POLICY` supply.
- [SOP soft reset contract](docs/PD-SOFT-RESET.md).
- [Hard reset boundary](docs/PD-HARD-RESET.md).
- [USB PD pin review](docs/usbpd-review.md): the reviewed `usbpd` revision.
- [SCP and FCP caller contract](docs/scp.md): entry, transport timing, output
  sessions and passive decoding.

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
