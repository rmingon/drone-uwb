#ifndef CONFIG_H
#define CONFIG_H

/*
 * Board and network settings for the drone.
 */

#define SSID     "ESP-TEST"
#define PASSWORD "123456789"

/* Websocket endpoint of code/server, which listens on 1227. */
#define SERVER_HOST_NAME "ws://esp_server:1227"

/* Motor takes a uint8_t, so the throttle range is 0..255. */
#define THROTTLE_MINIMUM 0
#define THROTTLE_MAXIMUM 255

#endif // CONFIG_H
