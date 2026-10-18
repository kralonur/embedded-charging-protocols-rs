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
Identity uses VDMResponseTimer; other queries use SenderResponseTimer. Optional
Product Type VDOs remain raw, and absent VDOs are never synthesized.

Regression peers provide synthetic policy in `tests/support`. Expected wire
bytes remain explicit, independent test vectors rather than being generated
from the implementation's constants.
