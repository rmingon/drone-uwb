#include "ranging.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include <DW1000Ng.hpp>
#include <DW1000NgRTLS.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgUtils.hpp>

#include "config.h"
#include "uwb_profile.h"

static const char *TAG = "RANGING";

/* Anchors must let blink frames through, they carry no destination address. */
static frame_filtering_configuration_t ANCHOR_FRAME_FILTER = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    true /* allow blink frames */
};

static uint16_t s_antenna_delay;

/* --------------------------------------------------------- tag registry -- */

typedef struct {
    byte eui[8]; /* as carried in the blink frame, least significant byte first */
    uint16_t address;
    bool used;
    bool reported;
} tag_slot_t;

static tag_slot_t s_tags[RANGING_MAX_TAGS];

static void eui_to_hex(const byte eui[8], char out[RANGING_EUI_HEX_LEN])
{
    /* print most significant byte first, the order EUIs are usually written in */
    for (int i = 0; i < 8; i++) {
        snprintf(out + i * 2, 3, "%02x", eui[7 - i]);
    }
}

/**
 * Short addresses are handed out sequentially and kept for as long as the anchor
 * runs, so a tag keeps the same address across blinks and every anchor in the
 * chain reports measurements under the same identity.
 */
static uint16_t assign_short_address(const byte eui[8])
{
    for (int i = 0; i < RANGING_MAX_TAGS; i++) {
        if (s_tags[i].used && memcmp(s_tags[i].eui, eui, 8) == 0) {
            return s_tags[i].address;
        }
    }
    for (int i = 0; i < RANGING_MAX_TAGS; i++) {
        if (!s_tags[i].used) {
            memcpy(s_tags[i].eui, eui, 8);
            s_tags[i].address = RANGING_FIRST_TAG_ADDRESS + i;
            s_tags[i].used = true;
            s_tags[i].reported = false;
            ESP_LOGI(TAG, "new tag, short address %u", s_tags[i].address);
            return s_tags[i].address;
        }
    }
    ESP_LOGW(TAG, "tag table full, reusing slot 0");
    memcpy(s_tags[0].eui, eui, 8);
    s_tags[0].reported = false;
    return s_tags[0].address;
}

bool ranging_take_new_tag(ranging_tag_t *out)
{
    for (int i = 0; i < RANGING_MAX_TAGS; i++) {
        if (s_tags[i].used && !s_tags[i].reported) {
            s_tags[i].reported = true;
            eui_to_hex(s_tags[i].eui, out->eui);
            out->address = s_tags[i].address;
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------- filtering -- */

/* A single multipath reflection can add meters to one exchange, so report the
   median of the last few rather than the raw value. */
#define RANGING_WINDOW 5

typedef struct {
    uint16_t address;
    double samples[RANGING_WINDOW];
    uint8_t count;
    uint8_t next;
} filter_t;

static filter_t s_filters[RANGING_MAX_TAGS];

static filter_t *filter_for(uint16_t address)
{
    for (int i = 0; i < RANGING_MAX_TAGS; i++) {
        if (s_filters[i].count > 0 && s_filters[i].address == address) {
            return &s_filters[i];
        }
    }
    for (int i = 0; i < RANGING_MAX_TAGS; i++) {
        if (s_filters[i].count == 0) {
            s_filters[i].address = address;
            return &s_filters[i];
        }
    }
    s_filters[0].address = address;
    s_filters[0].count = 0;
    s_filters[0].next = 0;
    return &s_filters[0];
}

static double median_of(double *values, uint8_t n)
{
    double sorted[RANGING_WINDOW];
    memcpy(sorted, values, n * sizeof(double));
    for (uint8_t i = 1; i < n; i++) {
        const double key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[n / 2];
}

static double filter_push(uint16_t address, double range)
{
    filter_t *filter = filter_for(address);
    filter->samples[filter->next] = range;
    filter->next = (filter->next + 1) % RANGING_WINDOW;
    if (filter->count < RANGING_WINDOW) {
        filter->count++;
    }
    return median_of(filter->samples, filter->count);
}

/* ------------------------------------------------------------------ init -- */

void ranging_set_antenna_delay(uint16_t antenna_delay)
{
    s_antenna_delay = antenna_delay;
    DW1000Ng::setAntennaDelay(antenna_delay);
    ESP_LOGI(TAG, "antenna delay %u", antenna_delay);
}

uint16_t ranging_get_antenna_delay(void)
{
    return s_antenna_delay;
}

void ranging_init(uint16_t antenna_delay)
{
    DW1000Ng::initializeNoInterrupt(DW1000_PIN_SS, DW1000_PIN_RST);
    DW1000Ng::applyConfiguration(UWB_RADIO_CONFIG);
    DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER);

    /* EUI from the wifi MAC so every anchor is unique without hand editing */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    byte eui[8] = {mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], 0x00, 0x01};
    DW1000Ng::setEUI(eui);

    /* Without these an anchor blocks forever waiting for a frame that a tag
       walking out of range will never send. They scale with the airtime of a
       frame, so they come from the profile. */
    DW1000Ng::setPreambleDetectionTimeout(UWB_PREAMBLE_DETECTION_TIMEOUT);
    DW1000Ng::setSfdDetectionTimeout(UWB_SFD_DETECTION_TIMEOUT);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(UWB_RECEIVE_FRAME_TIMEOUT_US);

    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(ANCHOR_ADDRESS);

    ranging_set_antenna_delay(antenna_delay);

    ESP_LOGI(TAG, "radio profile: %s", UWB_PROFILE_NAME);

    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    ESP_LOGI(TAG, "Device ID: %s", msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    ESP_LOGI(TAG, "Unique ID: %s", msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    ESP_LOGI(TAG, "Network ID & Device Address: %s", msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    ESP_LOGI(TAG, "Device mode: %s", msg);

#if ANCHOR_IS_MAIN
    ESP_LOGI(TAG, "main anchor %d, hands the tag over to anchor %d", ANCHOR_ADDRESS, ANCHOR_NEXT_ADDRESS);
#elif ANCHOR_NEXT_ADDRESS
    ESP_LOGI(TAG, "anchor %d, hands the tag over to anchor %d", ANCHOR_ADDRESS, ANCHOR_NEXT_ADDRESS);
#else
    ESP_LOGI(TAG, "anchor %d, last of the chain", ANCHOR_ADDRESS);
#endif
}

/* ---------------------------------------------------------- self survey -- */

static const uint16_t s_peers[] = ANCHOR_PEERS;
static const size_t s_peer_count = sizeof(s_peers) / sizeof(s_peers[0]);
static size_t s_next_peer = 0;
static int64_t s_next_survey_us = 0;

/**
 * Plays tag towards one peer. The peer is the one that computes and reports the
 * distance, so there is nothing to return: this side only pays the air time.
 */
static void survey_next_peer(void)
{
    const uint16_t peer = s_peers[s_next_peer];
    s_next_peer = (s_next_peer + 1) % s_peer_count;

    const RangeResult result = DW1000NgRTLS::tagRangeSingle(peer, UWB_FINAL_MESSAGE_DELAY_US);
    if (!result.success) {
        ESP_LOGD(TAG, "anchor %u did not answer the survey", peer);
    }
    /* result.next points further down the chain; ignore it, the peer list says
       who to talk to. */
}

static bool survey_due(void)
{
    if (s_peer_count == 0) {
        return false;
    }
    const int64_t now = esp_timer_get_time();
    if (now < s_next_survey_us) {
        return false;
    }
    /* spread the peers over the period so a full round takes one period */
    s_next_survey_us = now + (int64_t)ANCHOR_SURVEY_PERIOD_MS * 1000 / (int64_t)s_peer_count;
    return true;
}

/* ------------------------------------------------------------------ poll -- */

/**
 * The last anchor of the chain tells the tag to go back to blinking, every other
 * one points it at its successor.
 */
static RangeAcceptResult accept_range(void)
{
#if ANCHOR_NEXT_ADDRESS
    return DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, ANCHOR_NEXT_ADDRESS);
#else
    return DW1000NgRTLS::anchorRangeAccept(NextActivity::ACTIVITY_FINISHED, TAG_BLINK_RATE_MS);
#endif
}

static bool publish(const RangeAcceptResult &result, ranging_measure_t *out)
{
    if (!result.success) {
        return false;
    }

    out->tag_address = result.tag_address;
    /* Short addresses below RANGING_FIRST_TAG_ADDRESS belong to anchors, so a
       measurement against one of those is a self survey leg. */
    out->is_anchor = result.tag_address < RANGING_FIRST_TAG_ADDRESS;
    out->raw_range = result.range;
    out->range = filter_push(result.tag_address, result.range);
    out->rx_power = DW1000Ng::getReceivePower();
    out->fp_power = DW1000Ng::getFirstPathPower();
    /* Decawave APS006: a gap above ~6 dB between total and first path power
       means the direct path is attenuated, so the range reads long. */
    out->line_of_sight = (out->rx_power - out->fp_power) < 6.0f;
    return true;
}

bool ranging_poll(ranging_measure_t *out)
{
    if (survey_due()) {
        survey_next_peer();
        return false;
    }

#if ANCHOR_IS_MAIN
    /* The main anchor is the one tags blink at. It answers with the short
       address the tag will use for the rest of the chain. */
    if (!DW1000NgRTLS::receiveFrame()) {
        return false;
    }

    const size_t len = DW1000Ng::getReceivedDataLength();
    if (len < 10) {
        return false;
    }
    byte frame[len];
    DW1000Ng::getReceivedData(frame, len);

    if (frame[0] != BLINK) {
        /* A poll outside the blink flow comes from a peer anchor running its
           own survey. Answer it, nobody else will. */
        if (frame[9] == RANGING_TAG_POLL) {
            return publish(DW1000NgRTLS::anchorCompletePoll(frame, len, NextActivity::ACTIVITY_FINISHED,
                                                            TAG_BLINK_RATE_MS),
                           out);
        }
        return false;
    }

    const uint16_t address = assign_short_address(&frame[2]);
    byte short_address[2];
    DW1000NgUtils::writeValueToBytes(short_address, address, 2);

    DW1000NgRTLS::transmitRangingInitiation(&frame[2], short_address);
    DW1000NgRTLS::waitForTransmission();

    return publish(accept_range(), out);
#else
    /* Secondary anchors are handed the tag by the previous one and go straight
       into the poll exchange. */
    return publish(accept_range(), out);
#endif
}
