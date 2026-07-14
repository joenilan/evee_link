// Throttle input for the remote.
//
// Two implementations behind one interface, chosen at build time:
//
//   EVEE_THROTTLE_SYNTHETIC=1  the value comes from the serial console. This is
//                              how stages 0-2 get built and tested with no
//                              trigger hardware in existence.
//   EVEE_THROTTLE_SYNTHETIC=0  a linear hall sensor on an ADC1 pin.
//
// Why hall and not a potentiometer: a pot is a wiper scraping a resistive track.
// It wears, and a worn track can fail OPEN — which an ADC reads as full throttle.
// A hall sensor is non-contact, never wears, and when it or its wiring fails it
// fails to a voltage OUTSIDE the plausible band, which is detectable. That
// detection is `fault()` below, and it is the whole reason for the choice.
#pragma once
#include <Arduino.h>
#include "evee_link.h"

// ADC1 only. ADC2 is unusable while WiFi is active on ESP32, and ESP-NOW is WiFi.
#ifndef EVEE_HALL_PIN
#define EVEE_HALL_PIN 1
#endif

// Calibration, in raw 12-bit ADC counts. Set these by running `cal` on the
// serial console with the real trigger attached (stage 3).
#ifndef EVEE_HALL_MIN
#define EVEE_HALL_MIN 900     // trigger fully pulled back (full brake)
#endif
#ifndef EVEE_HALL_CENTER
#define EVEE_HALL_CENTER 2048 // trigger at rest
#endif
#ifndef EVEE_HALL_MAX
#define EVEE_HALL_MAX 3200    // trigger fully pushed (full accel)
#endif

// The plausible band. A reading outside this is not a throttle position, it is a
// broken sensor or a broken wire: report fault, command zero, refuse to arm.
// Keep a healthy margin outside the calibrated travel so that normal end-stops
// and temperature drift never trip it.
#ifndef EVEE_HALL_VALID_LO
#define EVEE_HALL_VALID_LO 300
#endif
#ifndef EVEE_HALL_VALID_HI
#define EVEE_HALL_VALID_HI 3800
#endif

class Throttle {
public:
    void begin();

    // Call every control tick. Filters, calibrates, deadzones, clamps.
    void update();

    // -1000..+1000, already deadzoned and clamped. Zero whenever fault() is set.
    int16_t value() const { return _value; }

    // The sensor is reading outside its plausible band, i.e. it or its wiring has
    // failed. The receiver disarms on this.
    bool fault() const { return _fault; }

#if EVEE_THROTTLE_SYNTHETIC
    void setSynthetic(int16_t v) { _synthetic = eveeClampThrottle(v); }
#endif

private:
    int16_t _value = 0;
    bool    _fault = false;
#if EVEE_THROTTLE_SYNTHETIC
    int16_t _synthetic = 0;
#else
    int32_t _filtered = EVEE_HALL_CENTER;
#endif
};
