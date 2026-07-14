# evee_link

A wireless throttle link for VESC-based boards, over ESP-NOW. Part of the
[EVEE](https://github.com/joenilan/Longboard-Display) platform.

Two firmwares and one contract:

- **`[env:remote]`** — the handheld. Hall throttle in, ESP-NOW out at 50 Hz.
- **`[env:receiver]`** — lives on the board. ESP-NOW in, VESC UART out.
- **`include/evee_link.h`** — the wire contract. It is the only file a third
  implementation needs (an ESK8OS receiver mode, a test harness, a different
  remote). The protocol is the product; the boards are an implementation detail.

Read [`docs/protocol.md`](docs/protocol.md) before changing anything in the
header. Most of what looks like a magic number in there is a safety rule.

## Safety, up front

This firmware controls a motor that moves a person. Two things are not optional:

1. **Configure the VESC's own timeout.** *App Settings → General → Timeout* =
   250 ms, **timeout brake current = 0**. This is the failsafe that still works
   when my code hangs. Everything else is the failsafe that works when it doesn't.
2. **The receiver will not arm on an unencrypted link**, and will not arm until
   it has seen the throttle held at neutral for 500 ms. Both are deliberate.
   Don't route around them.

The default build (`[env:receiver]`) has `EVEE_VESC_ENABLED=0` — it logs the
command it *would* send instead of sending it. That is how every stage before a
motor is attached gets tested. `[env:receiver_live]` is the one that moves.

## Hardware

| Node | Board | Notes |
|---|---|---|
| Receiver | ESP32-S3-WROOM-1 (Hosyond) | Bare devkit is exactly right — it lives in the enclosure on board power. |
| Remote (bring-up) | ESP32-S3-WROOM-1 | USB-powered on the bench. |
| Remote (printed) | Seeed XIAO ESP32-S3 | `[env:remote_xiao]`. Onboard LiPo charging, thumbnail-sized. A bare WROOM devkit is a poor handheld: no battery management, and you cannot feed a 4.2 V cell to a 3V3 pin (the S3's max is 3.6 V). |
| Remote display | 0.91" SSD1306, 128×32, I²C @ 0x3C | |
| Throttle | Linear hall sensor (SS49E/A1324) + magnet | On **ADC1** (GPIO1-10). ADC2 does not work while WiFi is active. |

## Pairing

ESP-NOW encrypts per-peer, keyed to a MAC. Each node prints its own at boot:

```
[evee] this node: 34:85:18:AB:CD:EF
```

Put each node's MAC into the *other* node's `include/evee_peers.h` and reflash.
Until you do, both fall back to unencrypted broadcast — which works, and is how
you get far enough to read the MACs, but **cannot arm**.

## Build

```
pio run -e receiver -t upload     # dry run: logs VESC commands, sends nothing
pio run -e remote   -t upload
pio device monitor
```

On the remote's console: `arm`, `t 250`, `z`, `kill`, `mac`. Until a trigger
exists, the throttle is synthetic (`EVEE_THROTTLE_SYNTHETIC=1`) — which is enough
to build and prove the link, the arming rules and the failsafe. Everything except
the feel of the trigger.

## Stages

- [x] **0 — the link.** Packet format, encrypted peering, sequence validation.
- [x] **1 — the receiver.** Arming state machine, failsafe, VESC command path.
- [x] **2 — the remote.** TX loop, OLED, synthetic throttle.
- [ ] **3 — telemetry.** `COMM_GET_VALUES` on the receiver, populating `EveeStatus`.
- [ ] **4 — the trigger.** Hall sensor, calibration, the printed shell.
- [ ] **5 — ESK8OS receiver mode.** The same contract, hosted in the display firmware.

Stage 4 is the only one that needs parts that don't exist yet. Buy a Flipsky or
Maytech replacement trigger module (hall sensor already mounted, known-good
spring and magnet geometry) and print a shell around it. Designing your own
trigger from scratch is a *mechanical* project and shouldn't block the firmware.
