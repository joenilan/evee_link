// ---------------------------------------------------------------------------
// Pairing. ESP-NOW encrypts per-peer, keyed to a MAC, so each node needs to know
// the other's address.
//
// HOW TO PAIR:
//   1. Flash both nodes. Each prints its own MAC at boot, e.g.
//        [evee] this node: 34:85:18:AB:CD:EF
//   2. Put the remote's MAC in EVEE_REMOTE_MAC and the receiver's in
//      EVEE_RECEIVER_MAC below.
//   3. Reflash both.
//
// Leaving a MAC as all-zero means "broadcast, unencrypted". That works, and it
// is how you get far enough to read the MACs in step 1 — but the receiver will
// REFUSE TO ARM on an unencrypted link. That is deliberate. An open throttle
// channel is not a convenience worth having.
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>

// The handheld.
static const uint8_t EVEE_REMOTE_MAC[6]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// The node on the board.
static const uint8_t EVEE_RECEIVER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
