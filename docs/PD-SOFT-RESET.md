# SOP soft reset contract

The maintained sink profile handles protocol-mandated SOP Soft_Reset exchanges.
One-shot trials do not enter reset/recovery paths.

- An unexpected supported message outside a power transition, or a transmission
  without GoodCRC after bounded transport retries, can arm one Soft_Reset.
- The reset uses MessageID 0 and the negotiated revision. A partner's valid
  Soft_Reset is answered with Accept, including during negotiation.
- Reset clears protocol counters and pending work, not the physical supply or
  established target. Accept consumes transmit ID 0; renegotiation starts at ID 1.
- Only fresh Source_Capabilities may follow the reset exchange. The established
  target is requested again; an interrupted user change requires new confirmation.
- A further protocol error before renegotiation completes, an unacknowledged
  reset, or failure to receive its Accept requires a mandatory Hard Reset.

References: USB PD R3.2 V1.2 sections 7.1.1, 7.7 and 9.2.5.2, Table 7.1.
The session guards independently authorize engine transmissions. They do not
initialize, recover or reset the transport. A malformed frame or local I/O
fault ends the session instead of authorizing cleanup or a hot retry.
