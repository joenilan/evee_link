#include "As5600.h"
#include <Wire.h>

// Register map (AS5600 datasheet).
static const uint8_t REG_STATUS    = 0x0B;
static const uint8_t REG_RAW_ANGLE = 0x0C;   // 0x0C hi (4 bits), 0x0D lo

// STATUS bits.
static const uint8_t ST_MH = 1 << 3;   // magnet too strong
static const uint8_t ST_ML = 1 << 4;   // magnet too weak
static const uint8_t ST_MD = 1 << 5;   // magnet detected

bool As5600::begin(TwoWire& wire) {
    _wire = &wire;
    uint8_t s;
    return readReg8(REG_STATUS, s);
}

bool As5600::readReg8(uint8_t reg, uint8_t& out) {
    if (!_wire) return false;
    _wire->beginTransmission(AS5600_ADDR);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return false;
    out = _wire->read();
    return true;
}

bool As5600::readRaw(uint16_t& out) {
    if (!_wire) return false;
    _wire->beginTransmission(AS5600_ADDR);
    _wire->write(REG_RAW_ANGLE);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;

    const uint8_t hi = _wire->read();
    const uint8_t lo = _wire->read();
    out = (uint16_t)(((hi & 0x0F) << 8) | lo);   // 12-bit
    return true;
}

uint8_t As5600::status() {
    uint8_t s = 0;
    readReg8(REG_STATUS, s);
    return s;
}

bool As5600::magnetOk() {
    uint8_t s;
    if (!readReg8(REG_STATUS, s)) return false;   // sensor not answering at all
    if (!(s & ST_MD)) return false;               // no magnet
    if (s & ST_ML)    return false;               // too weak to trust
    // MH (too strong) is left as a warning rather than a fault: the angle is
    // still valid, the magnet is just closer than ideal.
    return true;
}
