#include "uwb_tag.h"

#include <DW1000Ng.hpp>
#include <DW1000NgRTLS.hpp>

#include "uwb_profile.h"

/* DW1000 wiring, same as the anchors */
static const uint8_t PIN_SS = 5;
static const uint8_t PIN_RST = 14;

/* Antenna delay matters far less on the tag than on the anchors: the anchor is
   the one computing the distance. Keep it consistent with them anyway. */
static const uint16_t ANTENNA_DELAY = 16436;

/* A tag is addressed directly, so unlike an anchor it does not accept blinks. */
static frame_filtering_configuration_t TAG_FRAME_FILTER = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

static char eui_hex[UWB_EUI_HEX_LEN] = "";
static byte eui_bytes[8];
static volatile uint32_t cycles = 0;

/* Set by the last anchor of the chain, in ms. */
static volatile uint32_t blink_rate = 200;

static void uwbTagTask(void *arg) {
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    DW1000Ng::applyConfiguration(UWB_RADIO_CONFIG);
    DW1000Ng::enableFrameFiltering(TAG_FRAME_FILTER);

    DW1000Ng::setEUI(eui_bytes);
    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setAntennaDelay(ANTENNA_DELAY);

    /* Without a timeout the tag blocks forever when no anchor answers, and the
       ranging task would never yield again. */
    DW1000Ng::setPreambleDetectionTimeout(UWB_PREAMBLE_DETECTION_TIMEOUT);
    DW1000Ng::setSfdDetectionTimeout(UWB_SFD_DETECTION_TIMEOUT);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(UWB_RECEIVE_FRAME_TIMEOUT_US);

    Serial.print("UWB radio profile: "); Serial.println(UWB_PROFILE_NAME);

    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("UWB device id: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("UWB unique id: "); Serial.println(msg);

    for (;;) {
        RangeInfrastructureResult result = DW1000NgRTLS::tagTwrLocalize(UWB_FINAL_MESSAGE_DELAY_US);
        if (result.success) {
            cycles++;
            if (result.new_blink_rate > 0) {
                blink_rate = result.new_blink_rate;
            }
        }
        /* Yields the core between cycles; also paces how often the anchors get
           a fresh distance. */
        vTaskDelay(pdMS_TO_TICKS(blink_rate));
    }
}

void uwbTagBegin(const uint8_t mac[6]) {
    /* EUI is the wifi MAC padded to 8 bytes, so the anchors can report an id
       the server can tie back to this drone. */
    eui_bytes[0] = mac[0];
    eui_bytes[1] = mac[1];
    eui_bytes[2] = mac[2];
    eui_bytes[3] = mac[3];
    eui_bytes[4] = mac[4];
    eui_bytes[5] = mac[5];
    eui_bytes[6] = 0x00;
    eui_bytes[7] = 0x01;

    for (int i = 0; i < 8; i++) {
        snprintf(eui_hex + i * 2, 3, "%02x", eui_bytes[i]);
    }

    /* Core 0: the Arduino loop with the PID runs on core 1 and must not be
       slowed down by the busy waits inside the ranging exchange. */
    xTaskCreatePinnedToCore(uwbTagTask, "uwb_tag", 4096, nullptr, 1, nullptr, 0);
}

const char *uwbTagEui() {
    return eui_hex;
}

uint32_t uwbTagCycles() {
    return cycles;
}
