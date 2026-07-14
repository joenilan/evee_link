// ============================================================================
// EVEE Remote — the handheld. Throttle in, ESP-NOW out, 50 Hz.
//
// Until a trigger exists, the throttle comes from the serial console
// (EVEE_THROTTLE_SYNTHETIC=1). That is enough to build and prove the entire link,
// the arming rules and the failsafe — everything except the feel of the trigger.
//
// Serial console:
//   arm        request arm (the receiver still needs the neutral run)
//   disarm     drop the arm request
//   kill       kill switch — receiver disarms immediately
//   t <n>      set throttle, -1000..1000
//   z          throttle to zero
//   mac        print this node's MAC (for pairing)
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "evee_link.h"
#include "evee_peers.h"
#include "EveeRadio.h"
#include "Throttle.h"

// The 0.91" SSD1306 is 128x32 at address 0x3C.
#define OLED_W 128
#define OLED_H 32
#define OLED_ADDR 0x3C

static Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
static bool oledOk = false;

static EveeRadio     radio;
static Throttle      throttle;
static EveePeerState receiverState;

static volatile bool armRequest = false;
static volatile bool killed     = false;

// Latest status from the board. Display only — never on the safety path. A remote
// that hears nothing back still sends throttle; it just shows no telemetry.
static volatile uint32_t statusLastMs = 0;
static EveeStatus        lastStatus   = {};

static void onRx(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (!eveeValidate(data, len, EVEE_PKT_STATUS, receiverState)) return;
    memcpy(&lastStatus, data, sizeof(lastStatus));
    statusLastMs = millis();
}

static const char* stateName(uint8_t s) {
    switch (s) {
        case EVEE_STATE_BOOT:     return "BOOT";
        case EVEE_STATE_DISARMED: return "DISARM";
        case EVEE_STATE_ARMED:    return "ARMED";
        case EVEE_STATE_FAILSAFE: return "FAILSF";
        case EVEE_STATE_FAULT:    return "FAULT";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// The send loop. Pinned to core 1 at high priority so that a slow I2C screen
// redraw (which runs in loop(), same core, lower priority) cannot stretch a
// 20 ms control tick into something the receiver reads as a dropped link.
// ---------------------------------------------------------------------------
static void txTask(void*) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        throttle.update();

        EveeControl c = {};
        radio.fillHeader(c.hdr, EVEE_PKT_CONTROL);

        const bool fault = throttle.fault();

        // A faulted sensor sends zero, always. The flag tells the receiver why,
        // but the value is what protects the rider if the flag is ever ignored.
        c.throttle = fault ? 0 : throttle.value();
        c.flags    = 0;
        if (armRequest && !fault && !killed) c.flags |= EVEE_FLAG_ARM_REQUEST;
        if (fault)                           c.flags |= EVEE_FLAG_THROTTLE_FAULT;
        if (killed)                          c.flags |= EVEE_FLAG_KILL;

        c.remoteBattPct = 100;   // TODO(stage 2): read the LiPo divider

        radio.send(&c, sizeof(c));

        vTaskDelayUntil(&last, pdMS_TO_TICKS(EVEE_CONTROL_MS));
    }
}

static void handleSerial() {
    static char buf[32];
    static uint8_t n = 0;

    while (Serial.available()) {
        const char ch = (char)Serial.read();
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (n < sizeof(buf) - 1) buf[n++] = ch;
            continue;
        }
        buf[n] = 0;
        n = 0;

        if (!strcmp(buf, "arm")) {
            armRequest = true;
            killed = false;
            Serial.println("[evee] arm requested");
        } else if (!strcmp(buf, "disarm")) {
            armRequest = false;
            Serial.println("[evee] arm request dropped");
        } else if (!strcmp(buf, "kill")) {
            killed = true;
            armRequest = false;
            Serial.println("[evee] KILL");
        } else if (!strcmp(buf, "mac")) {
            Serial.printf("[evee] this node: %s\n", EveeRadio::selfMac().c_str());
        } else if (!strcmp(buf, "z")) {
#if EVEE_THROTTLE_SYNTHETIC
            throttle.setSynthetic(0);
#endif
            Serial.println("[evee] throttle 0");
        } else if (!strncmp(buf, "t ", 2)) {
#if EVEE_THROTTLE_SYNTHETIC
            const int v = atoi(buf + 2);
            throttle.setSynthetic((int16_t)v);
            Serial.printf("[evee] throttle %d\n", (int)throttle.value());
#else
            Serial.println("[evee] built with a real throttle; `t` does nothing");
#endif
        } else if (buf[0]) {
            Serial.println("[evee] arm | disarm | kill | t <n> | z | mac");
        }
    }
}

static void drawOled() {
    if (!oledOk) return;

    const bool linkUp = statusLastMs && (millis() - statusLastMs) < 500;
    const int16_t thr = throttle.value();

    oled.clearDisplay();

    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.print(linkUp ? stateName(lastStatus.state) : "NO LINK");

    if (linkUp) {
        oled.setCursor(64, 0);
        oled.printf("%.1fkm/h", lastStatus.speed_kmh_x10 / 10.0f);
        oled.setCursor(64, 10);
        oled.printf("%.1fV %u%%", lastStatus.voltage_x10 / 10.0f,
                    (unsigned)lastStatus.boardBattPct);
    }

    if (throttle.fault()) {
        oled.setCursor(0, 10);
        oled.print("THROTTLE FAULT");
    }

    // Throttle bar, centred: left of centre is brake, right is accel.
    const int barY = 24, barH = 7, mid = OLED_W / 2;
    oled.drawRect(0, barY, OLED_W, barH, SSD1306_WHITE);
    oled.drawFastVLine(mid, barY, barH, SSD1306_WHITE);
    const int w = (abs((int)thr) * (OLED_W / 2 - 2)) / EVEE_THROTTLE_MAX;
    if (w > 0) {
        if (thr > 0) oled.fillRect(mid + 1, barY + 1, w, barH - 2, SSD1306_WHITE);
        else         oled.fillRect(mid - w, barY + 1, w, barH - 2, SSD1306_WHITE);
    }

    oled.display();
}

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("[evee] EVEE Remote");
    Serial.printf("[evee] this node: %s\n", EveeRadio::selfMac().c_str());
#if EVEE_THROTTLE_SYNTHETIC
    Serial.println("[evee] synthetic throttle — use `t <n>`, `arm`, `kill`");
#endif

    Wire.begin();
    oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (!oledOk) Serial.println("[evee] no SSD1306 at 0x3C — running headless");

    throttle.begin();

    if (!radio.begin(EVEE_RECEIVER_MAC, onRx)) {
        Serial.println("[evee] FATAL: ESP-NOW init failed");
        while (true) delay(1000);
    }
    Serial.printf("[evee] link: %s\n",
        radio.secure() ? "encrypted unicast" : "BROADCAST (unencrypted — cannot arm)");

    xTaskCreatePinnedToCore(txTask, "evee_tx", 4096, nullptr,
                            configMAX_PRIORITIES - 2, nullptr, 1);
}

void loop() {
    handleSerial();

    static uint32_t at = 0;
    if (millis() - at >= EVEE_STATUS_MS) {
        at = millis();
        drawOled();
    }
    delay(5);
}
