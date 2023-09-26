/*
 * motor.c
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "freertos/queue.h"
#include "motor.h"
#include "esp_log.h"
#include "esp_err.h"

extern QueueHandle_t MotorSpeedQueue;

#define LEFT_MOTOR_PIN 32
#define RIGHT_MOTOR_PIN 27
#define TOP_MOTOR_PIN 26
#define BOTTOM_MOTOR_PIN 25

ledc_timer_config_t motor_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t motor_channel[4];

void motorInit(void)
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

    MotorsSpeed motor_speed;
    motor_speed.l = 0;
    motor_speed.r = 0;
    motor_speed.b = 0;
    motor_speed.t = 0;

    xQueueSend(MotorSpeedQueue, &motor_speed, 10);

}

void motorControl(void *pvParameters)
{
	MotorsSpeed motor_speed;

	while(1) {
		if ( MotorSpeedQueue ) {
			if ( uxQueueMessagesWaiting ( MotorSpeedQueue ) )
			{
				if (xQueueReceive ( MotorSpeedQueue , &motor_speed , 10))
				{
					int left = 8191 * motor_speed.l / 255;
					int right = 8191 * motor_speed.r / 255;
					int bottom = 8191 * motor_speed.b / 255;
					int top = 8191 * motor_speed.t / 255;
				    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left));
				    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

				    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right));
				    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

				    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, top));
				    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));

				    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, bottom));
				    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}

}
