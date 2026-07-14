// The remote's tactile buttons.
//
// This class is deliberately dumb: it debounces, and it reports which buttons are
// held. It does NOT decide what a press means. Edge detection and long-press
// timing happen on the receiver, because the wire carries button LEVELS — see
// EveeButtons in evee_link.h for why that survives packet loss and an edge
// doesn't.
//
// There is no arm button. The spring-return trigger is the deadman.
#pragma once
#include <Arduino.h>
#include "evee_link.h"

// Active-low with internal pullups — a button is a short to ground. Avoid the
// S3's strapping pins (0, 3, 45, 46) and the USB pins (19, 20).
#ifndef EVEE_BTN_KILL_PIN
#define EVEE_BTN_KILL_PIN 4
#endif
#ifndef EVEE_BTN_PAGE_PIN
#define EVEE_BTN_PAGE_PIN 5
#endif
#ifndef EVEE_BTN_TRIP_PIN
#define EVEE_BTN_TRIP_PIN 6
#endif

#ifndef EVEE_BTN_DEBOUNCE_MS
#define EVEE_BTN_DEBOUNCE_MS 20
#endif

class Buttons {
public:
    void begin();
    void update();

    // Bitmask of EveeButtons currently held. Goes straight onto the wire.
    uint8_t mask() const { return _mask; }

    bool killHeld() const { return _mask & EVEE_BTN_KILL; }

private:
    struct Btn {
        uint8_t  pin;
        uint8_t  bit;
        bool     stable;      // debounced state, true = pressed
        bool     lastRaw;
        uint32_t changedAt;
    };
    Btn _btns[3];
    uint8_t _mask = 0;
};
