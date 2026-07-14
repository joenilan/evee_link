// ============================================================================
// EVEE Link v1 — the wire contract between an EVEE Remote and an EVEE Receiver.
//
// This header IS the protocol. It is the only file that a third implementation
// (an ESK8OS receiver mode, a different remote, a test harness) needs to copy.
// Nothing in here may depend on Arduino, ESP-IDF, or any board.
//
// Transport is ESP-NOW. Safety-relevant rules that a receiver MUST enforce are
// stated here rather than left to the implementation — see docs/protocol.md for
// the reasoning behind each one.
// ============================================================================
#pragma once
#include <stdint.h>

#define EVEE_LINK_VERSION 1

// 'EV' — cheap early-out for a stray ESP-NOW frame from some other project.
// It is not a security check; encryption is (see EveeSecurity below).
#define EVEE_MAGIC 0x5645

// ---------------------------------------------------------------------------
// Radio. Part of the contract, not an implementation detail: ESP-NOW is pinned
// to a WiFi channel and every node in the link must agree on it or they never
// hear each other.
//
// A node that also runs a SoftAP (as ESK8OS does for its export/console) will
// have the AP drag the shared radio onto ITS channel — so such a node must
// either force the AP onto EVEE_CHANNEL, or refuse to start the AP while armed.
// ESK8OS does the latter. Do not get clever with channel juggling on a safety
// path.
// ---------------------------------------------------------------------------
#ifndef EVEE_CHANNEL
#define EVEE_CHANNEL 1
#endif

// Enable the 802.11 LR rate alongside b/g/n. Not for range — the rider is a
// metre from the board — but for link margin in a park full of 2.4 GHz noise.
// Keeping b/g/n in the bitmask means a node without LR can still talk to one
// with it, so this is safe to leave on.
#ifndef EVEE_LONG_RANGE
#define EVEE_LONG_RANGE 1
#endif

// ---------------------------------------------------------------------------
// Timing. Every one of these is a safety parameter; read docs/protocol.md
// before changing any of them.
// ---------------------------------------------------------------------------

// Remote -> Receiver control rate.
//
// 100 Hz, because the dominant term in end-to-end throttle latency is not the
// radio (ESP-NOW is 1-3 ms) — it is the tick quantization, and there are two of
// them, one at each end. At 50 Hz that was up to 40 ms of pure waiting; at
// 100 Hz it is up to 20 ms, which takes the whole chain to ~15 ms typical.
//
// The radio has ample headroom for this and the receiver skips its telemetry
// poll while armed, so nothing is competing for the UART. Going faster still
// gives diminishing returns against the VESC's own current ramp.
#define EVEE_CONTROL_HZ      100
#define EVEE_CONTROL_MS      (1000 / EVEE_CONTROL_HZ)

// Receiver -> Remote status rate. Telemetry for the remote's screen; not on the
// safety path, so it runs slow to stay out of the control loop's way.
#define EVEE_STATUS_HZ       10
#define EVEE_STATUS_MS       (1000 / EVEE_STATUS_HZ)

// No valid control packet for this long => the receiver drops to FAILSAFE and
// commands zero. Seven missed packets at 50 Hz. Deliberately longer than one or
// two drops (2.4 GHz is a hostile band) and far shorter than a rider notices.
#define EVEE_FAILSAFE_MS     150

// The VESC's own UART timeout is the second, independent layer: if OUR firmware
// hangs, no set-command is sent, and the VESC zeroes the motor by itself. Set
// App Settings -> General -> Timeout to this, with timeout brake current 0.
#define EVEE_VESC_TIMEOUT_MS 250

// To arm, the receiver must see a continuous run of valid packets with the
// throttle at neutral for this long. This is what stops a stuck-open throttle
// at power-on, and what stops a mid-ride link recovery from resuming into a
// throttle the rider is still holding down.
#define EVEE_ARM_NEUTRAL_MS  500

// |throttle| at or below this counts as neutral. 3% of full scale.
#define EVEE_NEUTRAL_BAND    30

// ---------------------------------------------------------------------------
// Throttle scale. Fixed-point so the wire format has no float endianness or
// NaN questions. The REMOTE owns calibration: the receiver only ever sees a
// clean, already-deadzoned, already-clamped value on this scale.
// ---------------------------------------------------------------------------
#define EVEE_THROTTLE_MIN   (-1000)   // full brake
#define EVEE_THROTTLE_ZERO  0
#define EVEE_THROTTLE_MAX   1000      // full acceleration

enum EveePacketType : uint8_t {
    EVEE_PKT_CONTROL = 1,   // remote -> receiver, EVEE_CONTROL_HZ
    EVEE_PKT_STATUS  = 2,   // receiver -> remote, EVEE_STATUS_HZ
};

// Bits in EveeControl.flags.
enum EveeControlFlags : uint8_t {
    // The rider is asking to be armed. Necessary but NOT sufficient — the
    // receiver still requires the neutral-throttle run (EVEE_ARM_NEUTRAL_MS).
    EVEE_FLAG_ARM_REQUEST   = 1 << 0,

    // The remote's own throttle sensor reads outside its plausible band, i.e.
    // the hall sensor or its wiring has failed. The remote sets this AND sends
    // throttle = 0. A receiver seeing it MUST disarm; a faulty sensor is not
    // something to ride through.
    EVEE_FLAG_THROTTLE_FAULT = 1 << 1,

    // Rider pulled the kill switch / power button. Immediate disarm.
    EVEE_FLAG_KILL           = 1 << 2,
};

// Receiver state, reported back so the remote's screen can show it.
enum EveeLinkState : uint8_t {
    EVEE_STATE_BOOT     = 0,  // powered, has never armed
    EVEE_STATE_DISARMED = 1,  // link up, throttle ignored, waiting to arm
    EVEE_STATE_ARMED    = 2,  // throttle live
    EVEE_STATE_FAILSAFE = 3,  // lost the link while armed; commanding zero
    EVEE_STATE_FAULT    = 4,  // throttle-fault or VESC fault; needs a re-arm
};

// ---------------------------------------------------------------------------
// Packets. Packed, little-endian (both ends are Xtensa; no byte-swapping).
// ESP-NOW's payload limit is 250 bytes and both of these are tiny — the budget
// is not a constraint, so fields are sized for clarity, not for squeezing.
// ---------------------------------------------------------------------------
#pragma pack(push, 1)

struct EveeHeader {
    uint16_t magic;    // EVEE_MAGIC
    uint8_t  version;  // EVEE_LINK_VERSION — a receiver MUST reject a mismatch
    uint8_t  type;     // EveePacketType

    // Random, drawn once per power-cycle. Without this, a remote that reboots
    // restarts its seq at 1, the receiver still holds a high lastSeq, and every
    // packet is rejected as a replay — forever. A changed boot id means "the
    // sender restarted": accept it, resync the counter, and DISARM (a reboot
    // mid-ride must go through the neutral-throttle arming run again).
    uint32_t boot;

    // Monotonic within a boot. Receiver accepts only seq > last, which drops
    // replays and out-of-order arrivals. A stale throttle value is worse than a
    // missing one: the failsafe timer catches "missing", nothing catches "old".
    uint32_t seq;
};

// Remote -> Receiver. The only packet on the safety path.
struct EveeControl {
    EveeHeader hdr;
    int16_t  throttle;      // EVEE_THROTTLE_MIN .. EVEE_THROTTLE_MAX
    uint8_t  buttons;       // free bitmask, not safety-relevant
    uint8_t  flags;         // EveeControlFlags
    uint8_t  remoteBattPct; // 0..100, for the receiver's logs / rider warning
    uint8_t  _pad[3];
};

// Receiver -> Remote. Drives the remote's screen. Never safety-relevant: a
// remote that receives nothing still sends throttle, it just shows no telemetry.
struct EveeStatus {
    EveeHeader hdr;
    uint8_t  state;          // EveeLinkState
    uint8_t  boardBattPct;   // 0..100
    uint8_t  vescFault;      // VESC mc_fault_code, 0 = none
    uint8_t  _pad;
    int16_t  speed_kmh_x10;  // 25.4 km/h -> 254
    uint16_t voltage_x10;    // 50.4 V    -> 504
    int16_t  motorCurrent_x10;
    int16_t  motorTemp_c_x10;
    int16_t  escTemp_c_x10;
    uint16_t tripMeters;
};

#pragma pack(pop)

// A receiver MUST reject any frame whose length does not match its type exactly.
// A short frame is a corrupt or hostile frame, never a "partial" one.
static inline int eveePacketSize(uint8_t type) {
    switch (type) {
        case EVEE_PKT_CONTROL: return (int)sizeof(EveeControl);
        case EVEE_PKT_STATUS:  return (int)sizeof(EveeStatus);
        default:               return -1;
    }
}

static inline bool eveeIsNeutral(int16_t throttle) {
    return throttle <= EVEE_NEUTRAL_BAND && throttle >= -EVEE_NEUTRAL_BAND;
}

static inline int16_t eveeClampThrottle(int32_t v) {
    if (v > EVEE_THROTTLE_MAX) return EVEE_THROTTLE_MAX;
    if (v < EVEE_THROTTLE_MIN) return EVEE_THROTTLE_MIN;
    return (int16_t)v;
}

// ---------------------------------------------------------------------------
// Throttle shaping.
//
// The WIRE CARRIES RAW LINEAR POSITION. The remote calibrates its sensor and
// deadzones it; it does not shape the curve. Shaping is the RECEIVER's job,
// because that is where the settings live — you can retune the feel of the board
// without reflashing a sealed handheld with a battery soldered into it.
//
// Expo: out = k*x^3 + (1-k)*x, on x normalized to -1..1. The classic RC curve.
// Monotonic for k in [0,1], and it pins 0 -> 0 and +/-1 -> +/-1, so it can
// never invent throttle the rider did not ask for, and can never cap what they
// did. A linear map feels twitchy just off neutral and dull at the top; expo
// gives fine control where you actually ride and full power when you demand it.
//
// expoPct is 0 (linear) .. 100 (maximum curve). 30-40 is a good starting point.
// ---------------------------------------------------------------------------
#ifndef EVEE_EXPO_PCT_DEFAULT
#define EVEE_EXPO_PCT_DEFAULT 35
#endif

static inline int16_t eveeApplyExpo(int16_t throttle, uint8_t expoPct) {
    if (expoPct == 0) return throttle;
    if (expoPct > 100) expoPct = 100;

    const float x = (float)throttle / (float)EVEE_THROTTLE_MAX;   // -1..1
    const float k = (float)expoPct / 100.0f;
    const float y = k * x * x * x + (1.0f - k) * x;
    return eveeClampThrottle((int32_t)(y * (float)EVEE_THROTTLE_MAX));
}

// ---------------------------------------------------------------------------
// Acceleration slew limit, in amps per second, applied by the receiver.
//
// ACCELERATION ONLY. Braking is never slew-limited: a mushy, delayed brake is a
// safety problem, not a comfort feature. And a failsafe coast bypasses this
// entirely — dropping to zero on a lost link must be immediate, not a ramp.
// ---------------------------------------------------------------------------
#ifndef EVEE_ACCEL_SLEW_A_PER_S
#define EVEE_ACCEL_SLEW_A_PER_S 100.0f
#endif

// ---------------------------------------------------------------------------
// Security.
//
// ESP-NOW gives us CCMP encryption + authentication on a per-peer basis, keyed
// by a 16-byte PMK (network-wide) and a 16-byte LMK (per-peer). Both must be
// set for a link to be encrypted, and an encrypted peer is unicast to a known
// MAC — which means an attacker cannot inject a control packet without the key.
//
// A broadcast link cannot be encrypted. That is fine for bench bring-up, so the
// implementation allows it — but a receiver MUST refuse to ARM on an unencrypted
// link. Convenience is not worth an open throttle channel.
//
// CHANGE THESE KEYS. They are in a public repo; anything using the defaults is
// running with a published key. See docs/protocol.md.
// ---------------------------------------------------------------------------
#define EVEE_PMK "EveeLinkDefault!"   // exactly 16 bytes
#define EVEE_LMK "EveeChangeMeNow!"   // exactly 16 bytes
