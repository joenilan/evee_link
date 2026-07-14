#include "VescOut.h"
#include "evee_link.h"

// VESC comm ids (bldc/datatypes.h). Only the two we actually send.
static const uint8_t COMM_SET_CURRENT       = 6;
static const uint8_t COMM_SET_CURRENT_BRAKE = 7;

// CRC16/CCITT, poly 0x1021, init 0 — the same one bldc/crc.c uses.
static uint16_t crc16(const uint8_t* buf, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void bAppendI32(uint8_t* b, int& i, int32_t v) {
    b[i++] = (uint8_t)(v >> 24);
    b[i++] = (uint8_t)(v >> 16);
    b[i++] = (uint8_t)(v >> 8);
    b[i++] = (uint8_t)v;
}

void VescOut::begin() {
#if EVEE_VESC_ENABLED
    Serial1.begin(EVEE_VESC_BAUD, SERIAL_8N1, EVEE_VESC_RX_PIN, EVEE_VESC_TX_PIN);
#endif
}

void VescOut::sendPacket(const uint8_t* payload, uint8_t len) {
    // Short-frame form: 0x02, len, payload, crc_hi, crc_lo, 0x03.
    uint8_t frame[16];
    int n = 0;
    frame[n++] = 0x02;
    frame[n++] = len;
    memcpy(&frame[n], payload, len);
    n += len;
    const uint16_t crc = crc16(payload, len);
    frame[n++] = (uint8_t)(crc >> 8);
    frame[n++] = (uint8_t)(crc & 0xFF);
    frame[n++] = 0x03;

#if EVEE_VESC_ENABLED
    Serial1.write(frame, n);
#else
    // Dry run. Build the real frame anyway and log it on change, so the whole
    // command path — framing, CRC, scaling — is exercised and inspectable with
    // no motor attached. Logging only on change keeps 50 Hz off the console.
    static uint8_t lastFrame[16];
    static int lastN = 0;
    if (n != lastN || memcmp(frame, lastFrame, n) != 0) {
        memcpy(lastFrame, frame, n);
        lastN = n;
        Serial.print("[vesc] would send:");
        for (int i = 0; i < n; i++) Serial.printf(" %02X", frame[i]);
        Serial.println();
    }
#endif
}

void VescOut::sendCurrent(float amps) {
    uint8_t p[5];
    int i = 0;
    p[i++] = COMM_SET_CURRENT;
    bAppendI32(p, i, (int32_t)(amps * 1000.0f));   // milliamps
    sendPacket(p, i);
}

void VescOut::sendBrakeCurrent(float amps) {
    uint8_t p[5];
    int i = 0;
    p[i++] = COMM_SET_CURRENT_BRAKE;
    bAppendI32(p, i, (int32_t)(amps * 1000.0f));
    sendPacket(p, i);
}

// Only acceleration is slew-limited, so this tracks the last ACCEL current.
static float sLastAccelAmps = 0.0f;

void VescOut::coast() {
    // Immediate zero — bypasses the slew limiter on purpose. A failsafe coast
    // must drop to zero now, not ramp down over a tenth of a second.
    sLastAccelAmps = 0.0f;
    sendCurrent(0.0f);
}

void VescOut::writeThrottle(int16_t rawThrottle) {
    // The wire carries raw linear position; the receiver owns the curve.
    const int16_t throttle = eveeApplyExpo(rawThrottle, EVEE_EXPO_PCT_DEFAULT);

    if (eveeIsNeutral(throttle)) {
        sLastAccelAmps = 0.0f;
        sendCurrent(0.0f);
        return;
    }

    if (throttle > 0) {
        float amps = ((float)throttle / (float)EVEE_THROTTLE_MAX) * EVEE_MAX_MOTOR_CURRENT_A;

        // Limit the rise only. Backing OFF the throttle is always instant — the
        // rider asking for less power must never be made to wait.
        const float step = EVEE_ACCEL_SLEW_A_PER_S * (EVEE_CONTROL_MS / 1000.0f);
        if (amps > sLastAccelAmps + step) amps = sLastAccelAmps + step;

        sLastAccelAmps = amps;
        sendCurrent(amps);
    } else {
        // Brake is never slew-limited. A mushy, delayed brake is a safety problem,
        // not a comfort feature.
        sLastAccelAmps = 0.0f;
        const float frac = (float)(-throttle) / (float)(-EVEE_THROTTLE_MIN);
        sendBrakeCurrent(frac * EVEE_MAX_BRAKE_CURRENT_A);
    }
}
