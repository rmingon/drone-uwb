#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Brings up NVS, the default event loop, netif and the Wi-Fi station, then
 * connects to `ssid`. Retries indefinitely in the background on disconnect.
 */
esp_err_t wifi_start(const char *ssid, const char *password);

/**
 * Blocks until the station has an IP or `timeout_ms` elapses.
 * Returns true if connected.
 */
bool wifi_wait_connected(uint32_t timeout_ms);

/**
 * Station MAC address, available before the connection is up.
 */
esp_err_t wifi_get_mac(uint8_t mac[6]);

#ifdef __cplusplus
}
#endif
