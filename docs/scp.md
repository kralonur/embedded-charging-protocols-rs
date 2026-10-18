# SCP and FCP caller contract

`scp-protocol` runs SCP and FCP over one single-wire PHY on D−. The caller
owns the electrical entry, the line drive and sampling, the clock, the output
measurement and every decision to keep or end an output.

## Entry

Before the first read the caller completes BC1.2 entry (D+ held at 0.6 V
until the charger opens its D+/D− short and D− falls), then raises D+ to the SCP
detection level, 2.6–3.0 V (Appendix B, Table B.2), and waits the 40–80 ms
`Tvdp_src_on` (Table B.4). It checks both line levels and that VBUS is still
at 5 V before reading. FCP uses the same entry. Releasing D+ returns the
charger to 5 V.

## Transport

- `drive(Some(level))` drives D−; `drive(None)` releases it. Every read and
  write ends released.
- `sample` must return within 8 µs, and a drive may overrun its interval by at
  most 10%, so a 40 µs sync stays inside Table B.4. Missed deadlines end the
  transaction with `Timing`, never a shifted sample.
- `now_us` is monotonic and must not wrap during a transaction; `delay_us`
  waits at least the given time.
- A read takes about 30 ms and must not be preempted for more than a few µs.
  Damaged or unanswered reads get up to three attempts, 100 UI apart.

## Detection

`Detection::poll` and `fcp::Detection::poll` read one register per call.
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
- After `stop`, poll until not busy, then release D+. On any error, write
  nothing more: release D+ and let the charger's D+ protection (and SCP's
  watchdog) return it to 5 V. Dropping a session performs no I/O.
- Requests are checked against the charger's own ranges, steps, rated power
  and listed voltages. The caller adds its own envelope and confirms from
  measured VBUS that the output followed and later returned to 5 V.

## Passive decoding

`sniff::Decoder::push` takes every D− edge: its time in µs, on a clock that
may wrap, and the new level. Timestamps may lag the edge by up to 8 µs. A
missed edge drops traffic until the next ping. The decoder never drives the
line.
