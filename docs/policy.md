# Caller policy

`pd_spr::Limits` supplies operating limits, advertised identity and capability
facts. Its required `SESSION_POLICY` supplies query settle delays, status rate
limits, retry cooldown, PPS expiry grace and maximum unsent attempts. Durations
are milliseconds. There is no default scheduling policy.

`pd_discovery::Transport` requires `RESPONSE_TIMEOUT_MS` and
`SVID_NO_PROGRESS_TIMEOUT_MS`, both in milliseconds. The caller chooses receive
and no-progress deadlines for its transport.

Specification-defined packet sizes, units, protocol timers and SPR limits are
not caller settings. Shared wire constants live in `pd_constants`; specialized
encodings stay beside their owning parser or validator, with specification
references.

`Selection::request_port_scan` explicitly starts PD3 SOP partner discovery
only with an idle SPR fixed 5 V contract. A partner advertising dual-role power
is asked for its SPR Sink_Capabilities; others skip that query. Discover Identity
follows without role swaps or VCONN changes. The scan blocks power confirmation
and cable discovery, retains bounded raw objects and distinct terminal outcomes,
and is revoked by reset or session cancellation. Physical voltage checks and
exclusive receive/transmit authorization remain the transport caller's duty.
Identity uses VDMResponseTimer at the 50 ms maximum of tVDMSenderResponse (USB
PD R3.2 V1.2 Table 7.9), for partners that answer VDMs from software; other
queries use SenderResponseTimer.
Another well-formed Message during that window is a Protocol Error (Soft_Reset,
Table 7.1); a malformed identity response fails closed. Optional
Product Type VDOs are decoded only for known SOP UFP v1.2/v1.3 / DFP v1.2 layouts
at the received positions defined by the ID Header; unknown layouts stay raw,
and absent VDOs are never synthesized. Calculated Sink PDO power and flags are
informational only. A Discover Identity timeout follows successful request
acknowledgement, not proof that identity is unsupported; no automatic retry or
role change is used to work around it. A response after the timeout is Ignored
like any Structured VDM response in Ready.

Because this sink initiates Discover Identity, it supports Structured VDMs: in
PD3 Ready a received Structured VDM request is answered NAK at the received
version, or SVDM 2.1 when that is higher (USB PD R3.2 V1.2 sections 8.5.1 and
8.6, Table 7.2). Attention requests and responses are Ignored; Unstructured VDMs
are answered Not_Supported. A received Discover Identity also sets the common
version used by the next request, until Detach or Hard Reset.

Regression peers provide synthetic policy in `tests/support`. Expected wire
bytes remain explicit, independent test vectors rather than being generated
from the implementation's constants.
