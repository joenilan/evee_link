#include "Throttle.h"
#include <Wire.h>

#if !EVEE_THROTTLE_SYNTHETIC
#include <Preferences.h>
static Preferences sPrefs;
#endif

// ---------------------------------------------------------------------------
#if EVEE_THROTTLE_SYNTHETIC

void Throttle::begin() {
    _fault = false;
    _faultReason = "";
}

void Throttle::update() {
    _fault = false;
    _value = _synthetic;
}

// ---------------------------------------------------------------------------
#else

// The AS5600 shares the I2C bus with the OLED (0x36 vs 0x3C — no collision).
void Throttle::begin() {
    if (!_enc.begin(Wire)) {
        _fault = true;
        _faultReason = "AS5600 not responding";
        Serial.println("[thr] AS5600 not found on I2C");
        return;
    }
    calLoad();
    if (!_cal.valid) {
        Serial.println("[thr] NOT CALIBRATED — run `cal center`, `cal brake`, `cal accel`, `cal save`");
    }
}

// Map a wrapped delta onto the EVEE scale, using the two calibrated halves
// separately. The trigger's travel is not symmetric about its rest position —
// brake pull and accel push are different distances — so a single linear map
// would make the same thumb movement mean different things in each direction.
static int16_t mapCalibrated(int32_t delta, const ThrottleCal& c) {
    if (delta >= 0) {
        if (c.accelSpan <= 0) return 0;
        return eveeClampThrottle((delta * EVEE_THROTTLE_MAX) / c.accelSpan);
    }
    if (c.brakeSpan >= 0) return 0;
    return eveeClampThrottle(-((delta * EVEE_THROTTLE_MAX) / c.brakeSpan));
}

void Throttle::update() {
    if (!_cal.valid) {
        _fault = true;
        _faultReason = "not calibrated";
        _value = 0;
        return;
    }

    // The magnet check is the real fault signal. A hall sensor can only tell you
    // "this voltage looks implausible"; the AS5600 tells you the magnet is gone.
    // Require a few consecutive misses before faulting, so a single dropped I2C
    // transaction on a shared bus does not disarm a rider mid-corner.
    if (!_enc.magnetOk()) {
        if (++_statusMissCount >= 3) {
            _fault = true;
            _faultReason = "magnet missing or too weak";
            _value = 0;
            return;
        }
    } else {
        _statusMissCount = 0;
    }

    uint16_t raw;
    if (!_enc.readRaw(raw)) {
        if (++_statusMissCount >= 3) {
            _fault = true;
            _faultReason = "AS5600 read failed";
            _value = 0;
            return;
        }
        return;   // keep the last value for one or two ticks
    }
    _raw = raw;
    _fault = false;
    _faultReason = "";

    // Work in wrapped deltas from the rest position, never in absolute angle.
    // RAW_ANGLE wraps 4095 -> 0 somewhere on the circle, and you do not get to
    // choose where that seam lands when you glue the magnet on. If the trigger's
    // travel straddles it, absolute-angle math produces a violent jump from full
    // brake to full accel. Wrapped deltas make the seam irrelevant.
    const int16_t delta = as5600WrapDelta((int32_t)raw - (int32_t)_cal.center);

    // Mild IIR. Deliberately light: the encoder is already clean (12-bit, no ADC
    // noise), and a heavy filter buys smoothness with lag. A laggy throttle is
    // its own hazard. Smoothness comes from the sensor and the receiver's expo
    // curve, not from smearing the input.
    _filtered += ((int32_t)delta - _filtered) / 3;

    const int16_t mapped = mapCalibrated(_filtered, _cal);
    _value = eveeIsNeutral(mapped) ? 0 : mapped;
}

// ---- calibration ----------------------------------------------------------

bool Throttle::calCenter() {
    uint16_t raw;
    if (!_enc.readRaw(raw)) { Serial.println("[cal] read failed"); return false; }
    _cal.center = raw;
    _cal.valid = false;   // spans must be recaptured relative to the new centre
    Serial.printf("[cal] center = %u\n", raw);
    return true;
}

bool Throttle::calBrake() {
    uint16_t raw;
    if (!_enc.readRaw(raw)) { Serial.println("[cal] read failed"); return false; }
    const int16_t d = as5600WrapDelta((int32_t)raw - (int32_t)_cal.center);
    if (d >= 0) {
        Serial.printf("[cal] brake delta is %+d — expected negative. "
                      "Hold FULL BRAKE, or the magnet is mounted backwards.\n", d);
        return false;
    }
    _cal.brakeSpan = d;
    Serial.printf("[cal] brake span = %+d\n", d);
    return true;
}

bool Throttle::calAccel() {
    uint16_t raw;
    if (!_enc.readRaw(raw)) { Serial.println("[cal] read failed"); return false; }
    const int16_t d = as5600WrapDelta((int32_t)raw - (int32_t)_cal.center);
    if (d <= 0) {
        Serial.printf("[cal] accel delta is %+d — expected positive. "
                      "Hold FULL ACCEL, or the magnet is mounted backwards.\n", d);
        return false;
    }
    _cal.accelSpan = d;
    Serial.printf("[cal] accel span = %+d\n", d);
    return true;
}

void Throttle::calSave() {
    if (_cal.brakeSpan >= 0 || _cal.accelSpan <= 0) {
        Serial.println("[cal] refusing to save: capture center, brake and accel first");
        return;
    }
    _cal.valid = true;

    sPrefs.begin("evee", false);
    sPrefs.putUShort("cal_center", _cal.center);
    sPrefs.putShort("cal_brake",  _cal.brakeSpan);
    sPrefs.putShort("cal_accel",  _cal.accelSpan);
    sPrefs.putBool("cal_valid",   true);
    sPrefs.end();

    Serial.println("[cal] saved");
    calPrint();
}

void Throttle::calLoad() {
    sPrefs.begin("evee", true);
    _cal.center    = sPrefs.getUShort("cal_center", 0);
    _cal.brakeSpan = sPrefs.getShort("cal_brake", 0);
    _cal.accelSpan = sPrefs.getShort("cal_accel", 0);
    _cal.valid     = sPrefs.getBool("cal_valid", false);
    sPrefs.end();

    // Never trust persisted numbers that cannot describe a real trigger.
    if (_cal.brakeSpan >= 0 || _cal.accelSpan <= 0) _cal.valid = false;

    if (_cal.valid) calPrint();
}

void Throttle::calPrint() const {
    Serial.printf("[cal] center=%u brake=%+d accel=%+d %s\n",
                  _cal.center, _cal.brakeSpan, _cal.accelSpan,
                  _cal.valid ? "(valid)" : "(INVALID)");
}

#endif
