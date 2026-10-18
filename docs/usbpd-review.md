# USB PD pin review

## Selected revision

- Repository: <https://github.com/kralonur/usbpd>
- Branch at selection: `feat/partner-port-scan`
- Full revision: `1abe32e7761dbc1e1b7a5d8829abda93b5e0ca17`
- Partner-discovery review baseline: `99bf0229450fa807e225c9b2f6b85f655c2842f4`
- EPR review baseline: `9c1325e1ecf6be5f5f53be656ae3658dd7131101`
- Fork baseline examined: `72008c424eeebd0ea271eb5794ff000b3832a134`
- Packages: `usbpd` and `usbpd-traits`, both version `2.0.0`.
- Toolchain: current stable Rust.

The EPR revision was committed and reviewed locally against USB PD 3.2 v1.1.
It must be published before another machine can fetch this pin from GitHub.
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
framing, permits explicitly confirmed EPR entry when `Limits::EPR` is enabled,
requires automatic GoodCRC and retries, blocks
optional recovery, and ends after mandatory Hard Reset signaling. Chunk assembly
stays off unless the caller sets `Limits::EPR_OFFER`: then an explicit request
sends EPR_Get_Source_Cap from Ready (not standard for a Sink without EPR Mode,
section 6.5.14.1) and the answer is informational only. With it, another
multi-Chunk Message fails closed instead of getting Not_Supported. A Deferred
EPR_Get_Source_Cap returns to Ready and is asked again after the caller's
cooldown, up to its unsent-attempt limit.

## Constraints, not general approval

This is an integration review, not a USB-IF certification or exhaustive audit of
all fork code. The fork includes source/software-retry paths not enabled by
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

## Partner-discovery changes and checks

The selected revision adds PD3 sink-initiated SPR Sink_Capabilities and SOP
Discover Identity queries. Responses retain all seven possible message objects;
identity ACK requires the three mandatory VDOs after the VDM header, while NAK
and BUSY must contain no further VDOs. Unknown sink APDO subtypes remain raw.
The engine preserves the negotiated SVDM version across later scans and soft
resets, resetting it on reattach. Identity's VDMResponseTimer subtracts the
transport's GoodCRC confirmation delay; unsupported queries and timeouts return
to Ready with the established contract.

The wrapper separately authorizes each explicitly armed query at fixed 5 V;
Sink_Capabilities is queried only when the partner advertises dual-role power.
No local source capability is advertised to unlock this path (section 6.3.8
permits a Port to request its partner's sink capabilities). This adds no source
operation, role swap or VCONN permission and is not a compliance claim.

The fork passed all 50 sink-policy regressions, strict all-target Clippy and a
`serde,defmt` feature check. Portable sessions cover the supplied wire shapes,
explicit arming, unsupported roles/revisions/voltages, reset cancellation and
preservation of automatic information-query ordering. Hardware interoperability
and response timing under load remain unverified.

## EPR changes and checks

Entry requires Ack then Succeeded under one continuous entry timer. Requests in
EPR mode include the exact advertised PDO, including SPR-object requests before
exit. Reserved actions/failure reasons are fallible. Eleven-position sink
capabilities use bounded 26-byte chunk transmission and Chunk Request deadlines.
Keep-alive failure causes Hard Reset; mode callbacks clear state on exit/reset.

The EPR revision passed workspace check, strict all-target Clippy and nine
`cargo test -p usbpd epr` regressions. Shared black-box sessions additionally
exercise fixed/AVS entry, chunked sink capabilities, keep-alive, SPR-before-exit,
entry failure and sent/received Hard Reset. Loaded hardware timing remains
unverified; this review does not establish electrical protection.

## Earlier baseline checks

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
