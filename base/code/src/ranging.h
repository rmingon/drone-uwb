/*
 * Two way ranging (TWR) anchor, ISO/IEC 24730-62 flavour as implemented by the
 * DW1000Ng library. Replaces the RSSI proxy the anchor used to report.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RANGING_EUI_HEX_LEN 17 /* 16 hex digits plus terminator */

typedef struct {
    uint16_t tag_address; /* short address the main anchor handed to the tag */
    double range;         /* filtered distance in meters */
    double raw_range;     /* distance of this exchange alone, in meters */
    float rx_power;       /* dBm */
    float fp_power;       /* first path power, dBm */
    bool line_of_sight;   /* false when rx and first path power disagree */
    bool is_anchor;       /* the peer is another anchor, not a tag */
} ranging_measure_t;

typedef struct {
    char eui[RANGING_EUI_HEX_LEN]; /* tag EUI, most significant byte first */
    uint16_t address;              /* short address assigned to it */
} ranging_tag_t;

/**
 * Configures the DW1000 for TWR. `antenna_delay` is the calibrated value for
 * this board, see ranging_set_antenna_delay().
 */
void ranging_init(uint16_t antenna_delay);

/**
 * Runs one step of the ranging state machine. Returns true and fills `out` when
 * an exchange completed. Blocks for a few milliseconds at most.
 */
bool ranging_poll(ranging_measure_t *out);

/**
 * Main anchor only: returns true once for every tag that has just been given a
 * short address, so the anchor can tell the server which EUI it maps to.
 */
bool ranging_take_new_tag(ranging_tag_t *out);

/**
 * Antenna delay is the dominant source of constant range error: one unit is
 * roughly 4.7 mm, so an uncalibrated board is easily half a meter off.
 */
void ranging_set_antenna_delay(uint16_t antenna_delay);
uint16_t ranging_get_antenna_delay(void);

#ifdef __cplusplus
}
#endif
