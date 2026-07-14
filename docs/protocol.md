# EVEE Link v1

A wireless throttle link for VESC-based boards, over ESP-NOW.

`include/evee_link.h` is the normative contract. This document explains the
reasoning, because most of the rules in that header are safety rules, and a rule
you don't understand is a rule you'll "optimise" away.

## Why ESP-NOW

A throttle needs a 50 Hz control loop with sub-20 ms latency and no connection
state to renegotiate. BLE cannot reliably give you that — it is built around
connection intervals and it will not hold a hard deadline. ESP-NOW is
connectionless, sends in single-digit milliseconds, and needs no association.

The cost is that it is a raw layer-2 link with no session, no retries beyond the
MAC ACK, and no built-in notion of liveness. Everything that makes it *safe* has
to be built on top, which is what the rest of this document is.

## Topology

Two nodes:

- **Remote** — reads a throttle, sends `EveeControl` at 50 Hz.
- **Receiver** — validates, decides, drives the VESC. Sends `EveeStatus` back at
  10 Hz for the remote's screen.

The receiver can be a standalone node (this repo, `[env:receiver]`) or a mode
inside a bigger firmware such as ESK8OS. Both implement the same contract. The
protocol is the product; the boards are an implementation detail.

## Packets

Fixed-size, packed, little-endian. Both ends are Xtensa, so no byte-swapping.
Every frame carries:

| Field | Purpose |
|---|---|
| `magic` | Cheap rejection of unrelated ESP-NOW traffic. Not a security check. |
| `version` | Receiver rejects a mismatch outright. No partial compatibility. |
| `type` | `EVEE_PKT_CONTROL` or `EVEE_PKT_STATUS`. |
| `boot` | Random per power-cycle. See below. |
| `seq` | Monotonic within a boot. |

### Why `boot` exists

Without it: the remote reboots, its `seq` restarts at 1, the receiver still holds
a high `lastSeq`, and every subsequent packet is rejected as a replay — forever.
The link never comes back and the failure mode is baffling.

With it: a changed `boot` means "the sender restarted". The receiver adopts the
new counter *and disarms*. That second half matters — a remote that browns out
and recovers mid-ride must not resume into a throttle the rider is still holding.

### Why strictly-increasing `seq`

It drops replays and out-of-order arrivals. **A stale throttle value is more
dangerous than a missing one**: the failsafe timer catches "missing", and nothing
catches "old".

## Arming

The receiver will not send a non-zero command until all of the following have
held *continuously* for `EVEE_ARM_NEUTRAL_MS` (500 ms):

1. The link is up (a valid packet within `EVEE_FAILSAFE_MS`).
2. The link is **encrypted**. A broadcast link can never arm — see Security.
3. The remote is setting `EVEE_FLAG_ARM_REQUEST`.
4. The throttle is inside the neutral band (`EVEE_NEUTRAL_BAND`, 3%).

Any transition resets the run from zero. This one rule is what makes a
stuck-open throttle at power-on impossible, and what stops a link recovery from
resuming into a held throttle.

There is no way to arm faster, and there should not be one.

## Failsafe

Two independent layers. The second one is the one that matters.

**Layer 1 — this firmware.** No valid control packet for `EVEE_FAILSAFE_MS`
(150 ms, seven missed packets at 50 Hz) and the receiver enters `FAILSAFE` and
commands zero current. Recovery requires a full re-arm.

**Layer 2 — the VESC itself.** Configure *App Settings → General → Timeout* to
`EVEE_VESC_TIMEOUT_MS` (250 ms) with **timeout brake current 0**. If no
set-command arrives, the VESC zeroes the motor on its own.

Layer 2 exists because layer 1 is code I wrote, and it can hang. If the receiver
crashes, deadlocks, or gets stuck in a loop, it stops sending — and the VESC
notices and coasts without any of my code running. **Do not skip configuring
layer 2.** It is the only part of this system that survives a bug in the rest.

### Coast, never brake

On link loss the receiver commands *zero current*, not brake current. An
automatic hard brake at speed throws the rider off the board. The same reasoning
sets the VESC's timeout brake current to 0.

## Throttle sensing

The remote owns calibration. The receiver only ever sees a clean, deadzoned,
clamped value on the `-1000..+1000` scale — and clamps it again anyway, because
a receiver must not trust a sender's range.

**Use a linear hall sensor, not a potentiometer.** A pot is a wiper on a
resistive track; it wears, and a worn track can fail *open*, which an ADC reads
as **full throttle**. A hall sensor is non-contact and fails to a voltage outside
the plausible band, which the remote detects (`EVEE_HALL_VALID_LO/HI`) and
reports as `EVEE_FLAG_THROTTLE_FAULT` — whereupon it sends zero and the receiver
refuses to stay armed.

Note also: **ADC1 only** (GPIO1-10 on the S3). ADC2 is unusable while WiFi is
active, and ESP-NOW is WiFi.

## Security

ESP-NOW gives per-peer CCMP encryption and authentication, keyed by a 16-byte
network PMK and a 16-byte per-peer LMK, unicast to a known MAC. With it, nobody
can inject a control packet without the key.

Broadcast cannot be encrypted — ESP-NOW has no group key. Broadcast is therefore
allowed only as a bench mode, and **a receiver must refuse to arm on it.** An
open throttle channel is not a convenience worth having.

**Change the keys.** The defaults in `evee_link.h` are in a public repo.

## Radio coexistence

ESP-NOW runs on the WiFi radio, pinned to a channel. Every node must agree on
`EVEE_CHANNEL`.

If a node also runs a SoftAP (as ESK8OS does for its export/console), the AP will
drag the radio to *its* channel and the link will die. A receiver that shares a
chip with an AP must either force the AP onto `EVEE_CHANNEL`, or — better —
**refuse to start the AP while armed**. Do not get clever with channel juggling
on a safety path.

The standalone receiver in this repo runs no AP, no BLE, and no display, for
exactly this reason.

## Long-range mode

`WIFI_PROTOCOL_LR` is enabled alongside b/g/n. Not for range — the rider is a
metre from the board — but for **link margin in a park full of 2.4 GHz noise**.
Keeping b/g/n in the bitmask means a node without LR can still talk to one with
it, so it is safe to leave on.
