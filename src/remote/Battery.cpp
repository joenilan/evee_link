#include "Battery.h"

// Li-ion discharge curve, 1S. Piecewise linear against a handful of points off
// the real curve — a straight 3.0-4.2 V map would read ~50% for most of the ride
// and then fall off a cliff, because the chemistry's voltage plateau is nothing
// like linear in charge.
//
// These are RESTING voltages. The cell sags under load, so a reading taken while
// the radio is transmitting shows low. That is why update() filters hard: we
// want the trend, not the instantaneous dip.
static const struct { float v; uint8_t pct; } kCurve[] = {
    { 4.20f, 100 },
    { 4.00f,  80 },
    { 3.85f,  60 },
    { 3.75f,  45 },
    { 3.65f,  30 },
    { 3.50f,  15 },
    { 3.30f,   5 },
    { 3.00f,   0 },
};

static uint8_t voltsToPct(float v) {
    if (v >= kCurve[0].v) return 100;
    const int n = sizeof(kCurve) / sizeof(kCurve[0]);
    if (v <= kCurve[n - 1].v) return 0;

    for (int i = 0; i < n - 1; i++) {
        const float hi = kCurve[i].v, lo = kCurve[i + 1].v;
        if (v <= hi && v > lo) {
            const float f = (v - lo) / (hi - lo);
            return (uint8_t)(kCurve[i + 1].pct + f * (kCurve[i].pct - kCurve[i + 1].pct));
        }
    }
    return 0;
}

void Battery::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(EVEE_BATT_PIN, ADC_11db);   // full 0-3.3 V swing

    // Seed the filter so the first reading is not a slow crawl up from zero.
    const float v = (analogReadMilliVolts(EVEE_BATT_PIN) / 1000.0f) * EVEE_BATT_DIVIDER;
    _filtered = v;
    _volts = v;
    _pct = voltsToPct(v);
}

void Battery::update() {
    // analogReadMilliVolts applies the chip's factory ADC calibration, which is
    // worth a lot here — the raw ADC is nonlinear enough that a hand-rolled
    // counts-to-volts conversion is off by tenths, and tenths are the whole
    // dynamic range of a li-ion discharge curve.
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(EVEE_BATT_PIN);
    const float v = ((sum / 8.0f) / 1000.0f) * EVEE_BATT_DIVIDER;

    // Heavy filter, on purpose. The cell sags every time the radio transmits, and
    // a battery readout that flickers with TX bursts is worse than useless. We
    // want the trend.
    _filtered += (v - _filtered) * 0.02f;

    _volts = _filtered;
    _pct = voltsToPct(_filtered);
}
