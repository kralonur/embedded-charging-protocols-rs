# SCP, FCP and AFC caller contract

`single-wire-protocols` implements SCP, FCP and Samsung AFC in separate
`scp`, `fcp` and `afc` modules over a caller-owned single-wire PHY on D−.
Shared transport and wire errors live at the crate root; `control` holds
shared request/control errors. SCP output procedures live in `scp::control`.
The caller owns the electrical entry, the line drive and sampling, the clock,
the output measurement and every decision to keep or end an output.

## Entry

Before the first read the caller completes BC1.2 entry (D+ held at 0.6 V
until the charger opens its D+/D− short and D− falls), then raises D+ to the SCP
detection level, 2.6–3.0 V (Appendix B, Table B.2), and waits the 40–80 ms
`Tvdp_src_on` (Table B.4). It checks both line levels and that VBUS is still
at 5 V before reading. FCP uses the same entry. AFC keeps D+ at 0.6 V after
BC1.2 entry instead of raising it. AFC uses its explicit reset pulse on
shutdown; releasing D+ alone is not assumed to restore its default output.

## Transport

- `drive(Some(level))` drives D−; `drive(None)` releases it. Every read and
  write ends released.
- `sample` must return within 8 µs, and a drive may overrun its interval by at
  most 10%, so a 40 µs sync stays inside Table B.4. AFC reuses these physical
  timing guards as an implementation budget, not its transaction framing.
  Missed deadlines end the transaction with `Timing`, never a shifted sample.
- `now_us` is monotonic and must not wrap during a transaction; `delay_us`
  waits at least the given time.
- A read takes about 30 ms and must not be preempted for more than a few µs.
  Damaged or unanswered reads get up to three attempts, 100 UI apart.

## Detection

`scp::Detection::poll` and `fcp::Detection::poll` read one register per call;
`afc::Detection::poll` sends a 5 V request and requires all four slave
pings. The receiver follows the community-hosted `afc_charger_intf.c`
loop: wait one UI before each sample and validate the high-sample count,
including the post-increment upper bound, rather than exact edge widths.
Its 2 ms response-data skip does not require a decodable reply byte.
It does not discover voltage/current ratings. Nominal intervals come from
that driver's constants, not its platform-specific `ndelay` compensation.
Late receive samples fail with `Timing`; later silence must not relabel a
local timing failure as an invalid adapter ping.
`NotScp` and `NotFcp` mean the charger answered without the protocol. A
`Timeout` before `answered()` means nothing listened. Any other error leaves
support undetermined.

## Output sessions

- A session queues work; each `poll` runs one transaction. `set` refuses with
  `Busy` until the queue is empty. Writes are never retried, because a lost
  reply can hide an applied write; every setpoint write is read back.
- SCP: `start` enables protocol mode with D+ and temperature protection kept
  on and a 2 s watchdog. Call `refresh` well inside that watchdog. `stop`
  restores ordinary output mode.
- FCP: `start` first reads the latched fault flags (0x28) to clear them;
  `set` and `refresh` fail on any flag set after that. FCP has no watchdog.
  `stop` writes the lowest listed voltage.
- AFC: each `poll` sends the queued request, with up to three attempts.
  Response data is skipped; this path reads no status registers and
  configures no watchdog, so measured VBUS must confirm the output. `stop` queues 5 V,
  but shutdown or an ambiguous output failure should use `afc::reset`, the
  reference driver's 100-UI (16 ms) D− high pulse after a released recovery
  interval, then release D+ and verify default VBUS. The pulse requires no response; it is not proof of 5 V.
- SCP/FCP: after `stop`, poll until not busy, then release D+. On any error,
  write nothing more: release D+ and let the charger's D+ protection (and
  SCP's watchdog) return it to 5 V. Dropping a session performs no I/O.
- SCP and FCP requests are checked against the charger's own ranges, steps,
  rated power and listed voltages; AFC only exposes 5 V and 9 V request
  choices, not discovered ratings. The caller adds its own envelope and
  confirms from measured VBUS that the output followed and later returned
  to 5 V. Never treat a cleanup timeout as confirmation of default VBUS.

## Passive decoding

`sniff::Decoder::push` takes every D− edge: its time in µs, on a clock that
may wrap, and the new level. Timestamps may lag the edge by up to 8 µs. A
missed edge drops traffic until the next ping. The decoder never drives the
line.
