/*
 * UWB tag side of the two way ranging exchange.
 *
 * The anchors in base/code measure the distance; the drone only has to take
 * part in the exchange. It runs on its own task so the flight loop keeps its
 * timing.
 */

#pragma once

#include <Arduino.h>

#define UWB_EUI_HEX_LEN 17 /* 16 hex digits plus terminator */

/**
 * Derives the tag EUI from `mac`, brings the DW1000 up and starts ranging in
 * the background.
 */
void uwbTagBegin(const uint8_t mac[6]);

/** EUI as hex, most significant byte first. Matches what the anchors report. */
const char *uwbTagEui();

/** Number of ranging cycles completed, for telemetry. */
uint32_t uwbTagCycles();
