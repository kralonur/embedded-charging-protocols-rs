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

Regression peers provide synthetic policy in `tests/support`. Expected wire
bytes remain explicit, independent test vectors rather than being generated
from the implementation's constants.
