#include "EveeRadio.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static EveeRxHandler g_handler = nullptr;

// The Arduino-ESP32 core changed this callback's signature in 3.x (it now hands
// you an info struct instead of a bare MAC). Support both so the repo builds on
// whatever core PlatformIO resolves.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (g_handler) g_handler(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (g_handler) g_handler(mac, data, len);
}
#endif

static bool isZeroMac(const uint8_t* m) {
    if (!m) return true;
    for (int i = 0; i < 6; i++) if (m[i]) return false;
    return true;
}

String EveeRadio::selfMac() {
    return WiFi.macAddress();
}

bool EveeRadio::begin(const uint8_t* peerMac, EveeRxHandler onRx) {
    g_handler = onRx;

    // esp_random() is a true HRNG once the radio is up; at this point it is not,
    // but a boot id only needs to differ from the LAST boot's, not be secret.
    _bootId = esp_random();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();   // we never associate; ESP-NOW does not need to

#if EVEE_LONG_RANGE
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
#endif

    // Pin the channel. Without this the radio sits on whatever channel the last
    // association left it on, and the two nodes silently never hear each other.
    esp_wifi_set_channel(EVEE_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer = {};
    peer.channel = EVEE_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;

    if (isZeroMac(peerMac)) {
        // Broadcast. Cannot be encrypted — ESP-NOW has no group key.
        memset(_peer, 0xFF, 6);
        peer.encrypt = false;
        _secure = false;
    } else {
        memcpy(_peer, peerMac, 6);
        // The PMK is network-wide; the LMK is this peer's. Both are needed for
        // ESP-NOW to actually encrypt rather than silently fall back to plain.
        esp_now_set_pmk((const uint8_t*)EVEE_PMK);
        memcpy(peer.lmk, EVEE_LMK, 16);
        peer.encrypt = true;
        _secure = true;
    }
    memcpy(peer.peer_addr, _peer, 6);

    return esp_now_add_peer(&peer) == ESP_OK;
}

bool EveeRadio::send(const void* payload, size_t len) {
    return esp_now_send(_peer, (const uint8_t*)payload, len) == ESP_OK;
}

bool eveeValidate(const uint8_t* data, int len, uint8_t expectType, EveePeerState& st) {
    st.restarted = false;

    if (len < (int)sizeof(EveeHeader)) return false;

    EveeHeader h;
    memcpy(&h, data, sizeof(h));   // the frame is not guaranteed aligned

    if (h.magic   != EVEE_MAGIC)         return false;
    if (h.version != EVEE_LINK_VERSION)  return false;
    if (h.type    != expectType)         return false;
    if (len != eveePacketSize(h.type))   return false;

    const bool firstContact = !st.seen;
    if (firstContact || h.boot != st.boot) {
        // First contact, or the sender power-cycled. Adopt its counter. The
        // caller disarms on `restarted`, so a remote reboot cannot resume a
        // live throttle — it has to walk back through the arming run.
        st.restarted = !firstContact;
        st.seen      = true;
        st.boot      = h.boot;
        st.lastSeq   = h.seq;
        return true;
    }

    if (h.seq <= st.lastSeq) return false;   // replay or out-of-order
    st.lastSeq = h.seq;
    return true;
}
