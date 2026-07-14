// AS5600 12-bit magnetic rotary encoder.
//
// Reads the ANGLE of a diametrically magnetized magnet glued to the trigger's
// pivot — not the field strength, which is what a plain hall sensor reads. That
// distinction is the whole reason this part is here: field strength varies with
// gap, temperature, and the magnet settling in its glue, so a hall throttle
// drifts and needs recalibrating. An angle does not.
//
// It also gives a REAL fault signal rather than an inferred one. The STATUS
// register reports whether a magnet is present and whether its field is usable.
// A hall sensor can only tell you "this voltage looks implausible"; the AS5600
// tells you "the magnet is gone".
//
// NOTE: this driver never touches the AS5600's OTP burn commands (BURN_ANGLE /
// BURN_SETTING). Those are one-shot and permanent. Calibration is done in our
// own software against RAW_ANGLE, which is reversible and costs nothing.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36

class As5600 {
public:
    bool begin(TwoWire& wire = Wire);

    // 0..4095 across a full turn. False if the I2C read failed.
    bool readRaw(uint16_t& out);

    // True when a magnet is present AND its field is in range. This is the
    // throttle's fault signal: no magnet, or a magnet too weak to trust, means
    // command zero and refuse to arm.
    bool magnetOk();

    // Detail for diagnostics: MD (detected), ML (too weak), MH (too strong).
    uint8_t status();

private:
    TwoWire* _wire = nullptr;
    bool readReg8(uint8_t reg, uint8_t& out);
};

// Wrap a raw-angle difference into -2048..+2047.
//
// RAW_ANGLE is 0..4095 around a circle, so a trigger whose travel happens to
// straddle the 0/4095 seam would otherwise produce a violent jump from one end
// of the throttle to the other. Working in wrapped deltas from the rest position
// makes where the seam falls irrelevant — which matters, because you do not get
// to choose how the magnet ends up oriented when you glue it on.
static inline int16_t as5600WrapDelta(int32_t d) {
    while (d >  2047) d -= 4096;
    while (d < -2048) d += 4096;
    return (int16_t)d;
}
