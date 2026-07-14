// Minimal VESC UART output. This is deliberately NOT a full VESC client — the
// receiver only ever needs to say "this much current" or "this much brake", and
// a small, auditable command path is worth more here than feature coverage.
// (ESK8OS has the full protocol client if this ever needs to read telemetry.)
#pragma once
#include <Arduino.h>

// Matches the ESK8OS wiring convention so a board can be moved between them.
#ifndef EVEE_VESC_RX_PIN
#define EVEE_VESC_RX_PIN 18
#endif
#ifndef EVEE_VESC_TX_PIN
#define EVEE_VESC_TX_PIN 17
#endif
#ifndef EVEE_VESC_BAUD
#define EVEE_VESC_BAUD 115200
#endif

// Full-scale limits. A throttle of +1000 means EVEE_MAX_MOTOR_CURRENT_A.
//
// These start LOW on purpose. Raise them only once the link, the arming, and the
// failsafe have all been watched working with the wheels off the ground.
#ifndef EVEE_MAX_MOTOR_CURRENT_A
#define EVEE_MAX_MOTOR_CURRENT_A 20.0f
#endif
#ifndef EVEE_MAX_BRAKE_CURRENT_A
#define EVEE_MAX_BRAKE_CURRENT_A 20.0f
#endif

class VescOut {
public:
    void begin();

    // rawThrottle is the raw linear position off the wire: -1000 (full brake) ..
    // +1000 (full accel). The expo curve and the acceleration slew limit are
    // applied HERE — the receiver owns the feel, so it can be retuned without
    // reflashing a sealed handheld. Anything in the neutral band coasts.
    void writeThrottle(int16_t rawThrottle);

    // Command zero immediately. This is what FAILSAFE calls. Coast, never brake:
    // an automatic hard brake at speed throws the rider off the board.
    void coast();

private:
    void sendCurrent(float amps);
    void sendBrakeCurrent(float amps);
    void sendPacket(const uint8_t* payload, uint8_t len);
};
