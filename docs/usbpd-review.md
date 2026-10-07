# USB PD pin review

## Selected revision

- Repository: <https://github.com/kralonur/usbpd>
- Branch at selection: `feat/sink-protocol-improvements`
- Full revision: `326123330125a74cfc75bc58a6d81e34fba97f4b`
- Fork baseline examined: `72008c424eeebd0ea271eb5794ff000b3832a134`
- Packages: `usbpd` and `usbpd-traits`, both version `2.0.0`.
- Toolchain: current stable Rust.

The branch tip was resolved with `git ls-remote`, cloned, and reviewed locally.
Selection is immutable: direct workspace Git dependencies pin both crates to this revision,
and the lockfile records their Git source. `cargo tree -i usbpd-traits` confirms
one shared trait identity for the library and engine. No local fork checkout,
or floating branch dependency is used by the workspace.

## Review scope and findings

The integration review examined the fork's Driver/DPM extensions and protocol,
message and sink-policy changes against the baseline:

- Driver distinguishes proven unsent `Deferred` from `NotAcknowledged`, preserves
  SOP IDs across exclusive cable handoffs, reports elapsed time since GoodCRC,
  and offers bounded BIST carrier support.
- Deferred AMS starts keep the established contract and do not consume a transmit
  ID. Transport retry exhaustion advances the ID and enters protocol recovery.
- Negotiated revision survives Soft_Reset; Hard Reset restarts detection.
- SenderResponseTimer subtracts driver confirmation delay from its GoodCRC anchor.
- Extended information blocks preserve bounded raw data and consistent parsing;
  reserved extended-control values and truncated blocks are fallible, not panics.
- PPS current clamping uses its seven-bit, 50 mA field.
- Sink policy supports the fixed/PPS status, alerts, information and capability
  responses used by this wrapper, and escalates failed soft-reset recovery.

The extracted guard independently checks outgoing RDOs/responses and incoming
framing, disables EPR/chunk assembly, requires automatic GoodCRC and retries,
blocks optional recovery, and ends after mandatory Hard Reset signaling.

## Constraints, not general approval

This is an integration review, not a USB-IF certification or exhaustive audit of
all fork code. The fork includes source/EPR/software-retry paths not enabled by
this wrapper. In particular, direct fork users must not infer the wrapper's
fail-closed guarantees: underlying reset/retry paths can loop and the bounded
pending receive queue can drop an old message when full.

The sink profile remains UFP and must not be VCONN Source in Ready: its
VCONN_Swap response is Reject. Drivers must independently enforce addressing,
CRC, bounded retries, collision avoidance, timing and safe cancellation. Cable
transport restore/power checks and physical transition to default are caller
responsibilities, never permissions inferred from protocol evidence.

Both packages use the fork directly, rather than crates.io packages with root-scoped
patches. Downstream path-dependency consumers inherit the exact Git sources and
need no patches in their own workspace.

## Checks performed

The selected checkout passed these targeted fork commands (53 tests total):

```sh
cargo test -p usbpd --lib backport_tests
cargo test -p usbpd --lib sink::policy_engine::tests
cargo test -p usbpd --lib pps_current_is_clamped
cargo test -p usbpd --lib reserved_types_are_fallible
cargo test -p usbpd --lib information_blocks_parse_alike
```

The `pd-protocol` workspace member passed all 145 retained library/integration tests, including 84
session regressions using semantic caller interfaces, on the stable compiler. Host and `thumbv7em-none-eabihf` checks, warning-free
Clippy/rustdoc, and an independent downstream `no_std` path-dependency smoke
build without downstream patches also passed. Transport compliance and other
target architectures were not tested.
