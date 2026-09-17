# Base station firmware

ESP32 anchor: ranges against the drones over UWB (DW1000) and reports to
`code/server` over UDP.

Built with **ESP-IDF** through PlatformIO (`framework = espidf`).

## Build

```sh
pio run              # build
pio run -t upload    # flash
pio device monitor   # 115200 baud
```

Wifi credentials, the server host and the pin assignment live in `src/config.h`.

## Layout

| Path | |
| --- | --- |
| `src/main.cpp` | setup, main loop, UDP reporting and commands |
| `src/ranging.cpp` | two way ranging, median filter, line of sight check |
| `src/uwb_profile.h` | radio settings, shared with the drone |
| `src/wifi.c` | station bring-up, reconnects on its own |
| `src/led.c` | WS2812 status LEDs over RMT |
| `components/DW1000Ng/` | DW1000 driver, see its README |
| `sdkconfig.defaults` | project-wide ESP-IDF settings, `sdkconfig` is generated |

## Ranging

Distance comes from two way ranging (time of flight), not from signal strength.
The anchors form a chain that a tag walks through on every cycle:

    tag blinks -> main anchor answers with a short address -> anchor 1 ranges
    -> anchor 2 ranges -> ... -> last anchor sends the tag back to blinking

Set `ANCHOR_ADDRESS`, `ANCHOR_IS_MAIN` and `ANCHOR_NEXT_ADDRESS` per board in
`src/config.h`. The last anchor of the chain sets `ANCHOR_NEXT_ADDRESS` to 0.

Each anchor reports its own distance, so the server sees one measurement per
anchor per cycle. Reported distances are the median of the last 5 exchanges,
which removes the occasional multipath outlier, and carry a `los` flag that is
false when the first path power sits more than 6 dB below the total receive
power, the usual sign of an obstructed direct path.

### Radio profile

Every radio setting lives in [`src/uwb_profile.h`](src/uwb_profile.h), which is
kept byte for byte identical to `code/drone/src/uwb_profile.h` because both ends
of a link have to agree on all of it. The default is the long range profile
(110 kbps, preamble 2048, PRF 64 MHz, Decawave SFD); flip the two defines at the
top of the file to go back to the faster one. The repository README has the
trade off in full.

### Calibration

Antenna delay is the dominant constant error, about 4.7 mm per unit, so an
uncalibrated board can be half a meter off. Put two boards at a known distance
in clear line of sight, read what the anchor reports, and push a corrected
value. A longer delay reports a shorter range, so increase it when the anchor
reads long, by about 213 units per metre of error:

```sh
echo -n '{"antenna_delay": 16450}' | nc -u -w1 <anchor ip> 7051
```

The value is stored in NVS under a per profile key, so switching radio profile
never reuses a calibration made for the other one, and survives reboots.
`DW1000_ANTENNA_DELAY` in `src/config.h` is only the fallback for a board that
has never been calibrated.

## Protocol

The anchor binds UDP `7051` and sends to `SERVER_HOST_NAME:7051`, matching
`code/server/src/index.ts`.

Outgoing, all wrapped in `{"uniq": "<anchor mac>", "type": ..., "data": {...}}`:

| type | data |
| --- | --- |
| `anchor` | `{"address": 1, "main": true}` at boot |
| `tag` | `{"eui": "aabbccddeeff0001", "address": 5}`, main anchor only, when it hands a short address to a tag |
| `range` | `{"address": 5, "range": 3.42, "raw_range": 3.51, "rx_power": -82.3, "fp_power": -85.1, "los": true}` |

Incoming: `{"reboot": true}` restarts the board, `{"antenna_delay": N}` stores a
calibration.

## Moving from Arduino

The Arduino libraries were replaced rather than shimmed:

| Was | Now |
| --- | --- |
| `f-army/DW1000-ng` | `components/DW1000Ng`, same driver ported to ESP-IDF |
| receive quality as a distance proxy | real two way ranging, see above |
| `xylopyrographer/LiteLED` | `src/led.c`, RMT WS2812 driver |
| `bblanchon/ArduinoJson` | `espressif/cjson` managed component |
| `WiFi` / `WiFiUDP` | `esp_wifi` + lwip BSD sockets |
| `Serial.print` | `ESP_LOG*` |

`uniq` is still built with `%x` per MAC byte, leading zeros dropped, so anchors
keep the identity the server already knows.
