#ifndef CONFIG_H
#define CONFIG_H

/*
 * Board and network settings for this anchor.
 */

#define SSID     "ESP-TEST"
#define PASSWORD "123456789"

/* Host running code/server. A name is resolved through DNS, an IPv4 literal
   such as "192.168.1.20" works too. */
#define SERVER_HOST_NAME "esp_server"

/* The anchor both sends to and listens on this port; code/server/src/index.ts
   binds 7051 and replies to the anchor on the same port. */
#define UDP_PORT 7051

/* DW1000 wiring. SPI bus pins live in platformio.ini build_flags. */
#define DW1000_PIN_SS  5
#define DW1000_PIN_RST 14

/* WS2812 status LEDs */
#define LED_GPIO       17
#define LED_COUNT      2
#define LED_BRIGHTNESS 30

#endif // CONFIG_H
