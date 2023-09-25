#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/queue.h"
#include "driver/ledc.h"

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

typedef struct {
    uint8_t l;
    uint8_t r;
    uint8_t b;
    uint8_t t;
} motor_t;

TaskHandle_t TaskMotorControl;

QueueHandle_t MotorSpeedQueue;

#define LEFT_MOTOR_PIN 32
#define RIGHT_MOTOR_PIN 27
#define TOP_MOTOR_PIN 26
#define BOTTOM_MOTOR_PIN 25

// GPIO assignment
#define LED_STRIP_BLINK_GPIO  14
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 1
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define MPU6050_ADDR     0x68
#define MPU6050_AX_ADDR  0x3B
#define MPU6050_READ_NUM 14

static const char *TAG = "example";

QueueHandle_t queue1;

ledc_timer_config_t motor_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t motor_channel[4];

static void pwm_init(void)
{
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    motor_channel[0].channel = LEDC_CHANNEL_0;
    motor_channel[0].gpio_num = LEFT_MOTOR_PIN;

    motor_channel[1].channel = LEDC_CHANNEL_1;
    motor_channel[1].gpio_num = RIGHT_MOTOR_PIN;

    motor_channel[2].channel = LEDC_CHANNEL_2;
    motor_channel[2].gpio_num = TOP_MOTOR_PIN;

    motor_channel[3].channel = LEDC_CHANNEL_3;
    motor_channel[3].gpio_num = BOTTOM_MOTOR_PIN;

    for (int i = 0; i < 4; i++)
    {
    	motor_channel[i].speed_mode = LEDC_LOW_SPEED_MODE;
    	motor_channel[i].timer_sel = LEDC_TIMER_0;
    	motor_channel[i].intr_type = LEDC_INTR_DISABLE;
    	motor_channel[i].duty = 0;
    	motor_channel[i].hpoint = 0;

        ESP_ERROR_CHECK(ledc_channel_config(&motor_channel[i]));
    }
}

void motor_control(void)
{
	while(1) {
	    uint32_t red_duty = 8191 * color.r / 255;
	    uint32_t green_duty = 8191 * color.g / 255;
	    uint32_t blue_duty = 8191 * color.b / 255;

	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, red_duty));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, green_duty));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, blue_duty));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));

	}

}

led_strip_handle_t led_init(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

void app_main(void)
{
    led_strip_handle_t led_strip = led_init();
    pwm_init();
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 50, 5, 5));
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
