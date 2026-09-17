#include "led.h"

#include <string.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "LED";

/* 1 tick = 0.1 us */
#define LED_RMT_RESOLUTION_HZ (10 * 1000 * 1000)
#define LED_MAX_PIXELS        8
/* WS2812 latch, datasheet asks for >50 us of idle low */
#define LED_RESET_US 80

static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;
static uint8_t s_grb[LED_MAX_PIXELS * 3];
static uint16_t s_count;
static uint8_t s_brightness = 255;

esp_err_t led_init(int gpio, uint16_t count)
{
    ESP_RETURN_ON_FALSE(count > 0 && count <= LED_MAX_PIXELS, ESP_ERR_INVALID_ARG, TAG,
                        "count must be 1..%d", LED_MAX_PIXELS);

    s_count = count;
    memset(s_grb, 0, sizeof(s_grb));

    rmt_tx_channel_config_t channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel_config, &s_channel), TAG, "rmt channel");

    /* WS2812: a 0 bit is 0.3 us high then 0.9 us low, a 1 bit the other way round */
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&encoder_config, &s_encoder), TAG, "rmt encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(s_channel), TAG, "rmt enable");

    ESP_LOGI(TAG, "%u WS2812 pixel(s) on GPIO %d", count, gpio);
    return ESP_OK;
}

void led_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
}

esp_err_t led_set_pixel(uint16_t index, uint32_t rgb)
{
    ESP_RETURN_ON_FALSE(s_channel != NULL, ESP_ERR_INVALID_STATE, TAG, "led_init() first");
    ESP_RETURN_ON_FALSE(index < s_count, ESP_ERR_INVALID_ARG, TAG, "pixel %u out of range", index);

    const uint8_t r = ((rgb >> 16) & 0xff) * s_brightness / 255;
    const uint8_t g = ((rgb >> 8) & 0xff) * s_brightness / 255;
    const uint8_t b = (rgb & 0xff) * s_brightness / 255;

    s_grb[index * 3 + 0] = g;
    s_grb[index * 3 + 1] = r;
    s_grb[index * 3 + 2] = b;

    const rmt_transmit_config_t tx_config = {.loop_count = 0};
    ESP_RETURN_ON_ERROR(rmt_transmit(s_channel, s_encoder, s_grb, s_count * 3, &tx_config), TAG, "transmit");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_channel, portMAX_DELAY), TAG, "wait");
    esp_rom_delay_us(LED_RESET_US);
    return ESP_OK;
}
