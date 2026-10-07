# Hard reset boundary

## B1: mandatory signaling cases

In a maintained session, the guard admits one Hard Reset after:

- a Request's SenderResponseTimer timeout;
- a power transition's PSTransitionTimer timeout or unexpected message;
- failure of a Soft_Reset exchange (no Accept or no GoodCRC);
- a further protocol error while recovering from Soft_Reset.

These correspond to USB PD R3.2 V1.2 sections 7.1.1, 7.7, 9.2.4.5,
9.2.4.6 and 9.2.5.2. A validated startup reset exchange can also arm mandatory
signaling before the first Request. The optional SinkWaitCapTimer reset is blocked.
One-shot sessions block initiated resets entirely.

After sent or received Hard Reset signaling, the session ends. The caller owns
PE_SNK_Transition_to_default, discharge/power sequencing, measurement, controller
restoration and the decision whether a fresh transport boundary is safe.
Neither constructing another `Trial` nor retained `MaintainedTrace` evidence
permits physical recovery. Drivers must enforce their own timing and safety gates
and must not silently retry a failed or cancelled transport operation.
