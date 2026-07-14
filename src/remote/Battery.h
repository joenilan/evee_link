// 18650 cell monitoring for the remote.
//
// Tapped off the CELL, before the boost converter — not off the 5 V rail. The
// boost holds its output steady at 5 V right up until the cell dies, so the rail
// tells you nothing about how much charge is left. The cell voltage is the only
// thing that does.
//
// Why this matters beyond a nice battery icon: when the TP4056's protection
// circuit trips, it trips INSTANTLY. The remote does not fade out, it vanishes.
// The receiver's failsafe catches that (150 ms to coast), but you would much
// rather read "20%" on the screen than discover it by coasting.
#pragma once
#include <Arduino.h>

// ADC1 only. ADC2 does not work while WiFi is active, and ESP-NOW is WiFi.
#ifndef EVEE_BATT_PIN
#define EVEE_BATT_PIN 2
#endif

// Divider ratio: a 100k/100k pair halves the cell's 3.0-4.2 V into the ADC's
// range. Measure your actual resistors and put the real ratio here — 1% parts
// are not 1% accurate at this job, and a wrong ratio shows a wrong percentage.
#ifndef EVEE_BATT_DIVIDER
#define EVEE_BATT_DIVIDER 2.0f
#endif

class Battery {
public:
    void begin();
    void update();

    float volts() const { return _volts; }
    uint8_t percent() const { return _pct; }

    // Below this the rider should land and charge: the protection cutoff is not
    // far away, and when it comes there is no warning.
    bool low() const { return _pct <= 15; }

private:
    float   _volts = 0.0f;
    uint8_t _pct = 0;
    float   _filtered = 0.0f;
};
