// ESP-NOW transport for EVEE Link. Shared verbatim by the remote and the
// receiver — the only difference between them is what they send and what they
// do with what they get.
#pragma once
#include <Arduino.h>
#include "evee_link.h"

// EVEE_CHANNEL and EVEE_LONG_RANGE are contract-level: they live in evee_link.h,
// because a channel the two nodes disagree on is a link that silently never
// forms, which makes it a protocol concern rather than a radio-helper one.

typedef void (*EveeRxHandler)(const uint8_t* mac, const uint8_t* data, int len);

class EveeRadio {
public:
    // peerMac == nullptr (or all-zero) => broadcast, UNENCRYPTED. Usable for
    // bench bring-up and for discovering the other node's MAC, but a receiver
    // must never arm on it. Pass a real MAC to get an encrypted unicast peer.
    bool begin(const uint8_t* peerMac, EveeRxHandler onRx);

    bool send(const void* payload, size_t len);

    // True only when the peer is a known MAC with encryption negotiated.
    bool secure() const { return _secure; }

    // This node's own MAC, formatted. Print it at boot — it is how you pair.
    static String selfMac();

    // Fill a header for an outgoing packet of the given type.
    void fillHeader(EveeHeader& h, EveePacketType type) {
        h.magic   = EVEE_MAGIC;
        h.version = EVEE_LINK_VERSION;
        h.type    = (uint8_t)type;
        h.boot    = _bootId;
        h.seq     = ++_txSeq;
    }

private:
    uint8_t  _peer[6] = {0};
    bool     _secure  = false;
    uint32_t _txSeq   = 0;
    uint32_t _bootId  = 0;   // drawn in begin()
};

// What a receiver remembers about one sender, across packets.
struct EveePeerState {
    uint32_t boot      = 0;
    uint32_t lastSeq   = 0;
    bool     seen      = false;

    // Set by eveeValidate when the sender's boot id changes, i.e. it rebooted.
    // The caller MUST treat this as a disarm — see the note on EveeHeader::boot.
    bool     restarted = false;
};

// Validate an inbound frame against the contract in evee_link.h: magic, version,
// type, exact length, and a strictly-increasing sequence number within the
// sender's current boot. Returns false for anything that fails — a rejected
// frame is dropped, never partially used.
bool eveeValidate(const uint8_t* data, int len, uint8_t expectType, EveePeerState& st);
