# DW1000Ng (ESP-IDF port)

Vendored copy of [F-Army/arduino-dw1000-ng](https://github.com/F-Army/arduino-dw1000-ng)
(MIT, see `LICENSE.md`), ported from the Arduino core to ESP-IDF. It replaces the
`f-army/DW1000-ng@^0.1.0` PlatformIO dependency the project used while it was on
the Arduino framework.

The DW1000 register logic is untouched. Only the platform layer changed:

| Upstream | Here |
| --- | --- |
| `SPIporting.cpp` on `SPIClass` | `src/SPIporting.cpp` on `spi_master`, manual chip select, DMA bounce buffer |
| `<Arduino.h>` | `include/DW1000NgPort.hpp` — `byte`, `boolean`, `pinMode`, `digitalWrite`, `delay`, `micros`, `attachInterrupt` |
| `initialize(ss, irq, rst, SPIClass&)` | `initialize(ss, irq, rst)` — the bus comes from the `DW1000NG_*` build flags |
| `setTransmitData(const String&)` | `setTransmitData(const char*)` |
| `getReceivedData(String&)` | `getReceivedData(char* buf, uint16_t size)` |

Two fixes were also made to the driver itself:

- `RangeAcceptResult` carries the ranged tag's short address, so an anchor can
  tell which tag a measurement belongs to. Upstream returns the range alone.
- `anchorRangeAccept()` left its result uninitialized, so a frame that was
  neither a poll nor a final response returned garbage in `.success`.

Two behavioural notes:

- `delay()` busy-waits for sub-tick durations. `vTaskDelay()` rounds `delay(1)`
  down to zero ticks at the default 100 Hz tick rate, and the DW1000 start-up
  sequence depends on those short waits actually happening.
- `attachInterrupt()` dispatches the handler from a task rather than from the
  ISR, because the DW1000Ng handler talks SPI. The base station calls
  `initializeNoInterrupt()` and does not use this path.

## Wiring

Defaults match the Arduino core's `SPI` object on ESP32 (VSPI). Override in
`platformio.ini` via `build_flags`:

    -D DW1000NG_SPI_HOST=SPI3_HOST
    -D DW1000NG_PIN_SCK=18
    -D DW1000NG_PIN_MISO=19
    -D DW1000NG_PIN_MOSI=23

Chip select and reset stay arguments of `DW1000Ng::initializeNoInterrupt(ss, rst)`.
