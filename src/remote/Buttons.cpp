#include "Buttons.h"

void Buttons::begin() {
    _btns[0] = { EVEE_BTN_KILL_PIN, EVEE_BTN_KILL, false, false, 0 };
    _btns[1] = { EVEE_BTN_PAGE_PIN, EVEE_BTN_PAGE, false, false, 0 };
    _btns[2] = { EVEE_BTN_TRIP_PIN, EVEE_BTN_TRIP, false, false, 0 };

    for (auto& b : _btns) pinMode(b.pin, INPUT_PULLUP);
    _mask = 0;
}

void Buttons::update() {
    const uint32_t now = millis();
    uint8_t m = 0;

    for (auto& b : _btns) {
        const bool raw = (digitalRead(b.pin) == LOW);   // active low

        // Standard debounce: a reading only becomes the truth once it has held
        // still for EVEE_BTN_DEBOUNCE_MS. Tactile switches bounce for a few ms on
        // both make and break, and at a 100 Hz control tick we would otherwise
        // send a burst of phantom presses on every real one.
        if (raw != b.lastRaw) {
            b.lastRaw = raw;
            b.changedAt = now;
        } else if (b.stable != raw && (now - b.changedAt) >= EVEE_BTN_DEBOUNCE_MS) {
            b.stable = raw;
        }

        if (b.stable) m |= b.bit;
    }

    _mask = m;
}
