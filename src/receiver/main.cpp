// ============================================================================
// EVEE Receiver — lives on the board. ESP-NOW in, VESC UART out.
//
// This node has exactly one job and no others. No display, no BLE, no WiFi AP,
// no web server. Everything that could stall the control loop has been left out
// on purpose, because the only thing standing between a software hiccup and a
// rider on the ground is how boring this firmware is.
//
// Two independent layers of failsafe:
//   1. This firmware: no valid packet for EVEE_FAILSAFE_MS => command zero.
//   2. The VESC itself: no set-command for its configured UART timeout => it
//      zeroes the motor on its own. This one keeps working even if we hang.
// Set the VESC's App Settings -> General -> Timeout to EVEE_VESC_TIMEOUT_MS and
// its timeout brake current to 0. Do not skip this; layer 2 is the one that
// survives a bug in layer 1.
// ============================================================================
#include <Arduino.h>
#include "evee_link.h"
#include "evee_peers.h"
#include "EveeRadio.h"
#include "VescOut.h"

static EveeRadio    radio;
static VescOut      vesc;
static EveePeerState remoteState;

// Written by the ESP-NOW receive callback (WiFi task), read by the control task.
// Both are 32-bit-or-smaller scalars on a 32-bit core, and the control task only
// ever acts on a coherent pair {throttle, flags} guarded by rxSeqStamp — a torn
// read would at worst use one tick's stale value, which the failsafe covers.
static volatile int16_t  rxThrottle  = 0;
static volatile uint8_t  rxFlags     = 0;
static volatile uint32_t rxLastMs    = 0;
static volatile bool     rxRestarted = false;
static volatile uint8_t  rxBattPct   = 0;

static EveeLinkState state       = EVEE_STATE_BOOT;
static uint32_t      neutralSince = 0;   // 0 = not currently in a neutral run

static const char* stateName(EveeLinkState s) {
    switch (s) {
        case EVEE_STATE_BOOT:     return "BOOT";
        case EVEE_STATE_DISARMED: return "DISARMED";
        case EVEE_STATE_ARMED:    return "ARMED";
        case EVEE_STATE_FAILSAFE: return "FAILSAFE";
        case EVEE_STATE_FAULT:    return "FAULT";
    }
    return "?";
}

static void setState(EveeLinkState s, const char* why) {
    if (s == state) return;
    Serial.printf("[evee] %s -> %s (%s)\n", stateName(state), stateName(s), why);
    state = s;
    neutralSince = 0;   // any transition restarts the arming run from scratch
}

static void onRx(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (!eveeValidate(data, len, EVEE_PKT_CONTROL, remoteState)) return;

    EveeControl c;
    memcpy(&c, data, sizeof(c));

    rxThrottle = eveeClampThrottle(c.throttle);   // never trust the sender's range
    rxFlags    = c.flags;
    rxBattPct  = c.remoteBattPct;
    rxLastMs   = millis();
    if (remoteState.restarted) rxRestarted = true;
}

// ---------------------------------------------------------------------------
// The control loop. Pinned to core 1, which on this node runs nothing else —
// the WiFi/ESP-NOW stack lives on core 0. Nothing may be added to core 1.
// ---------------------------------------------------------------------------
static void controlTask(void*) {
    TickType_t last = xTaskGetTickCount();
    uint32_t statusAt = 0;

    for (;;) {
        const uint32_t now = millis();
        const uint32_t age = now - rxLastMs;
        const bool linkUp  = rxLastMs != 0 && age < EVEE_FAILSAFE_MS;
        const int16_t thr  = rxThrottle;
        const uint8_t flags = rxFlags;

        // A remote reboot always drops us out of armed, no matter what the
        // throttle says — the rider may still be holding it down.
        if (rxRestarted) {
            rxRestarted = false;
            if (state == EVEE_STATE_ARMED) setState(EVEE_STATE_DISARMED, "remote rebooted");
        }

        if (flags & EVEE_FLAG_KILL) {
            setState(EVEE_STATE_DISARMED, "kill switch");
        } else if (flags & EVEE_FLAG_THROTTLE_FAULT) {
            // The remote's own sensor is reading out of band. Not something to
            // ride through, and not something a rider should be able to clear
            // by wiggling the trigger — it needs the full arming run again.
            setState(EVEE_STATE_FAULT, "remote reports throttle fault");
        }

        switch (state) {
            case EVEE_STATE_ARMED:
                if (!linkUp) {
                    setState(EVEE_STATE_FAILSAFE, "link lost");
                    vesc.coast();
                } else {
                    vesc.writeThrottle(thr);
                }
                break;

            case EVEE_STATE_FAILSAFE:
                // Keep actively commanding zero rather than just going quiet. Going
                // quiet also works (the VESC timeout catches it) but it takes up to
                // EVEE_VESC_TIMEOUT_MS, and we can be at zero now.
                vesc.coast();
                if (linkUp) setState(EVEE_STATE_DISARMED, "link back — re-arm required");
                break;

            case EVEE_STATE_FAULT:
                vesc.coast();
                // Only a clean link with the fault cleared gets us out of here.
                if (linkUp && !(flags & EVEE_FLAG_THROTTLE_FAULT))
                    setState(EVEE_STATE_DISARMED, "throttle fault cleared");
                break;

            case EVEE_STATE_BOOT:
            case EVEE_STATE_DISARMED:
            default:
                vesc.coast();

                // Arming. All of these must hold, continuously, for
                // EVEE_ARM_NEUTRAL_MS:
                //   - the link is up
                //   - it is ENCRYPTED (never arm on a broadcast bench link)
                //   - the rider is asking to arm
                //   - the throttle is at neutral
                // The neutral run is what makes a stuck-open throttle at power-on
                // impossible, and what stops a mid-ride recovery from resuming
                // into a throttle that is still held down.
                if (linkUp && radio.secure() &&
                    (flags & EVEE_FLAG_ARM_REQUEST) && eveeIsNeutral(thr)) {
                    if (neutralSince == 0) neutralSince = now;
                    if (now - neutralSince >= EVEE_ARM_NEUTRAL_MS)
                        setState(EVEE_STATE_ARMED, "neutral run complete");
                } else {
                    neutralSince = 0;
                }
                break;
        }

        // Telemetry back to the remote's screen. Off the safety path entirely.
        if (now - statusAt >= EVEE_STATUS_MS) {
            statusAt = now;
            EveeStatus s = {};
            radio.fillHeader(s.hdr, EVEE_PKT_STATUS);
            s.state = (uint8_t)state;
            // TODO(stage 4): populate from the VESC. Reading telemetry means
            // COMM_GET_VALUES on this same UART; it fits (10 Hz of ~80-byte
            // replies against 50 Hz of 10-byte writes) but it is not needed to
            // make the link work, so it is not here yet.
            radio.send(&s, sizeof(s));
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(EVEE_CONTROL_MS));
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("[evee] EVEE Receiver");
    Serial.printf("[evee] this node: %s\n", EveeRadio::selfMac().c_str());
#if EVEE_VESC_ENABLED
    Serial.println("[evee] VESC OUTPUT IS LIVE — wheels off the ground.");
#else
    Serial.println("[evee] dry run: VESC output disabled, commands are logged only.");
#endif

    vesc.begin();

    // An all-zero peer MAC means broadcast+unencrypted, which is how you get far
    // enough to read the MACs and pair. It cannot arm. See include/evee_peers.h.
    if (!radio.begin(EVEE_REMOTE_MAC, onRx)) {
        Serial.println("[evee] FATAL: ESP-NOW init failed");
        while (true) delay(1000);
    }
    Serial.printf("[evee] link: %s\n",
        radio.secure() ? "encrypted unicast" : "BROADCAST (unencrypted — cannot arm)");

    setState(EVEE_STATE_DISARMED, "boot");

    xTaskCreatePinnedToCore(controlTask, "evee_ctrl", 4096, nullptr,
                            configMAX_PRIORITIES - 2, nullptr, 1);
}

void loop() {
    // Deliberately empty. Everything happens in controlTask. If you are tempted
    // to put something here, put it on core 0 or in another firmware.
    static uint32_t at = 0;
    if (millis() - at > 1000) {
        at = millis();
        const uint32_t age = millis() - rxLastMs;
        Serial.printf("[evee] %-8s  thr=%+5d  link=%s  rbatt=%u%%\n",
            stateName(state), (int)rxThrottle,
            (rxLastMs && age < EVEE_FAILSAFE_MS) ? "up" : "DOWN",
            (unsigned)rxBattPct);
    }
    delay(50);
}
