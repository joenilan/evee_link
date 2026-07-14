#include "Throttle.h"

void Throttle::begin() {
#if !EVEE_THROTTLE_SYNTHETIC
    analogReadResolution(12);
    analogSetPinAttenuation(EVEE_HALL_PIN, ADC_11db);   // full 0-3.3V swing
    _filtered = analogRead(EVEE_HALL_PIN);
#endif
}

#if EVEE_THROTTLE_SYNTHETIC

void Throttle::update() {
    _fault = false;
    _value = _synthetic;
}

#else

// Map a raw reading onto the EVEE scale using the two calibrated halves
// separately — the trigger's travel is not symmetric about its rest position, so
// a single linear map would make brake and accel feel different for the same
// thumb movement.
static int16_t mapCalibrated(int32_t raw) {
    if (raw >= EVEE_HALL_CENTER) {
        const int32_t span = EVEE_HALL_MAX - EVEE_HALL_CENTER;
        if (span <= 0) return 0;
        return eveeClampThrottle(((raw - EVEE_HALL_CENTER) * EVEE_THROTTLE_MAX) / span);
    }
    const int32_t span = EVEE_HALL_CENTER - EVEE_HALL_MIN;
    if (span <= 0) return 0;
    return eveeClampThrottle(-((EVEE_HALL_CENTER - raw) * EVEE_THROTTLE_MAX) / span);
}

void Throttle::update() {
    // Oversample: the S3's ADC is noisy enough that a single read visibly
    // jitters the throttle.
    int32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(EVEE_HALL_PIN);
    const int32_t raw = sum / 8;

    if (raw < EVEE_HALL_VALID_LO || raw > EVEE_HALL_VALID_HI) {
        _fault = true;
        _value = 0;
        return;
    }
    _fault = false;

    // Light IIR on top of the oversampling. Deliberately mild — a laggy throttle
    // is its own hazard.
    _filtered += (raw - _filtered) / 4;

    const int16_t mapped = mapCalibrated(_filtered);
    _value = eveeIsNeutral(mapped) ? 0 : mapped;
}

#endif
