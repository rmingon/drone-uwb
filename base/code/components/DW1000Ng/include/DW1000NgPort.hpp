/*
 * ESP-IDF porting layer for the DW1000Ng library.
 *
 * The upstream library (https://github.com/F-Army/arduino-dw1000-ng) targets the
 * Arduino core. This header provides the small subset of the Arduino API the
 * driver actually uses, implemented on top of ESP-IDF, so the driver sources can
 * be vendored with only mechanical changes.
 *
 * Board wiring is compile-time configurable through build flags, see
 * platformio.ini.
 */

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------------------------------------------------------- wiring -- */

#ifndef DW1000NG_SPI_HOST
#define DW1000NG_SPI_HOST SPI3_HOST /* VSPI, the Arduino core default */
#endif

#ifndef DW1000NG_PIN_SCK
#define DW1000NG_PIN_SCK 18
#endif

#ifndef DW1000NG_PIN_MISO
#define DW1000NG_PIN_MISO 19
#endif

#ifndef DW1000NG_PIN_MOSI
#define DW1000NG_PIN_MOSI 23
#endif

/* ------------------------------------------------------- Arduino aliases -- */

typedef uint8_t byte;
typedef bool    boolean;

#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef RISING
#define RISING 1
#endif

#ifndef bitRead
#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

/**
 * Arduino-style pin direction. INPUT leaves the pin floating, which is what the
 * DW1000 reset line requires (datasheet v2.08 5.6.1: RSTn must never be driven
 * high).
 */
void pinMode(uint8_t pin, uint8_t mode);

void digitalWrite(uint8_t pin, uint8_t value);

int digitalRead(uint8_t pin);

/**
 * Millisecond delay that stays accurate below one FreeRTOS tick: vTaskDelay()
 * would round delay(1) down to zero ticks at the default 100 Hz tick rate, and
 * the DW1000 start-up sequence depends on those short waits really happening.
 */
void delay(uint32_t ms);

inline void delayMicroseconds(uint32_t us) { esp_rom_delay_us(us); }

inline uint32_t micros() { return (uint32_t)esp_timer_get_time(); }

inline uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000); }

inline int digitalPinToInterrupt(uint8_t pin) { return (int)pin; }

/**
 * Registers `handler` for rising edges on `pin`. Unlike the Arduino core the
 * handler does not run in interrupt context: it is dispatched from a dedicated
 * task, because the DW1000Ng interrupt handler talks SPI and therefore cannot
 * run from an ISR. `mode` is accepted for source compatibility and ignored.
 */
void attachInterrupt(int pin, void (*handler)(void), int mode);

void detachInterrupt(int pin);
