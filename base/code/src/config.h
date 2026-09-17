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

/* ---------------------------------------------------------------- ranging --
 * Every anchor needs its own ANCHOR_ADDRESS. Exactly one anchor is the main
 * one: it is what tags blink at, and it hands them down the chain. Each anchor
 * points at the next one, and the last of the chain sets ANCHOR_NEXT_ADDRESS
 * to 0 so the tag goes back to blinking.
 *
 *   anchor 1 (main) -> anchor 2 -> anchor 3 -> back to blink
 */
#define ANCHOR_ADDRESS      1
#define ANCHOR_IS_MAIN      1
#define ANCHOR_NEXT_ADDRESS 2

/* How often a tag restarts the cycle, in ms. Sent by the last anchor. */
#define TAG_BLINK_RATE_MS 200

/* Self survey: the other anchors this one measures its distance to. Each entry
 * is a peer ANCHOR_ADDRESS. The measurement is computed and reported by the
 * peer, not by this board, so list every peer on every board to get the full
 * matrix. Leave empty with {} to turn the survey off.
 *
 * It is what gives the anchors their coordinates: with the distances between
 * them the server can lay the network out without anyone measuring a room. */
#define ANCHOR_PEERS {2, 3}

/* One full round over ANCHOR_PEERS takes this long. */
#define ANCHOR_SURVEY_PERIOD_MS 5000

/* Tags tracked at once, and the first short address handed out. */
#define RANGING_MAX_TAGS          4
#define RANGING_FIRST_TAG_ADDRESS 5

/* Antenna delay in DW1000 units, roughly 4.7 mm each. 16436 is the generic
 * value: calibrate per board against a known distance and either set it here or
 * push it at runtime with {"antenna_delay": N}, which stores it in NVS. */
#define DW1000_ANTENNA_DELAY 16436

/* WS2812 status LEDs */
#define LED_GPIO       17
#define LED_COUNT      2
#define LED_BRIGHTNESS 30

#endif // CONFIG_H
