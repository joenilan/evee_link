// Throttle input for the remote.
//
// Two implementations behind one interface, chosen at build time:
//
//   EVEE_THROTTLE_SYNTHETIC=1  the value comes from the serial console. This is
//                              how the link, the arming and the failsafe get
//                              built and proven with no trigger in existence.
//   EVEE_THROTTLE_SYNTHETIC=0  an AS5600 magnetic encoder on the trigger pivot.
//
// The remote CALIBRATES and DEADZONES. It does not shape the curve — expo and
// slew limiting live on the receiver, so the feel of the board can be retuned
// without reflashing a sealed handheld. The wire carries raw linear position.
#pragma once
#include <Arduino.h>
#include "evee_link.h"
#include "As5600.h"

// Calibration, in raw AS5600 counts (0..4095 per turn). Captured by the `cal`
// console command and persisted to NVS, so it survives a reflash. The defaults
// are placeholders — an uncalibrated remote reports a fault and cannot arm.
struct ThrottleCal {
    uint16_t center = 0;      // trigger at rest
    int16_t  brakeSpan = 0;   // wrapped delta at full brake (negative)
    int16_t  accelSpan = 0;   // wrapped delta at full accel (positive)
    bool     valid = false;
};

class Throttle {
public:
    void begin();

    // Call every control tick.
    void update();

    // -1000..+1000, deadzoned and clamped. Always zero while fault() is set.
    int16_t value() const { return _value; }

    // The sensor is not giving a trustworthy reading — no magnet, a field too
    // weak to use, the AS5600 not answering, or no calibration. The receiver
    // disarms on this. It is a positive signal from the sensor, not a guess:
    // that is the point of using an encoder rather than a bare hall.
    bool fault() const { return _fault; }

    const char* faultReason() const { return _faultReason; }

#if EVEE_THROTTLE_SYNTHETIC
    void setSynthetic(int16_t v) { _synthetic = eveeClampThrottle(v); }
#else
    // Calibration, driven from the serial console. Sequence:
    //   cal center   trigger at rest
    //   cal brake    trigger held at full brake
    //   cal accel    trigger held at full accel
    //   cal save     write to NVS
    bool calCenter();
    bool calBrake();
    bool calAccel();
    void calSave();
    void calLoad();
    void calPrint() const;
    const ThrottleCal& cal() const { return _cal; }

    uint16_t rawAngle() const { return _raw; }
#endif

private:
    int16_t     _value = 0;
    bool        _fault = true;              // faulted until proven otherwise
    const char* _faultReason = "boot";

#if EVEE_THROTTLE_SYNTHETIC
    int16_t _synthetic = 0;
#else
    As5600      _enc;
    ThrottleCal _cal;
    uint16_t    _raw = 0;
    int32_t     _filtered = 0;   // filtered wrapped delta, not raw angle
    uint8_t     _statusMissCount = 0;
#endif
};
