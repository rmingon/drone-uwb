/*
 * i2c.c
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_CHANNEL 0

void i2cInit()
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 400000,
	};
	i2c_param_config(I2C_CHANNEL, &conf);

	ESP_ERROR_CHECK(i2c_driver_install(I2C_CHANNEL, I2C_MODE_MASTER, 0, 0, 0));
}

esp_err_t i2cWrite(uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t write_buf[2] = {reg_addr, reg_data};
    return i2c_master_write_to_device(I2C_CHANNEL, i2c_addr, write_buf, sizeof(write_buf), 1000);
}

esp_err_t i2cRead(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	return i2c_master_write_read_device(I2C_CHANNEL, i2c_addr, &reg_addr, 1, reg_data, length, 1000);
}


void i2cScan() {
    printf("i2c scan: \n");
	for (uint8_t i = 1; i < 127; i++)
	{
		int ret;
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
		i2c_cmd_link_delete(cmd);

		if (ret == ESP_OK)
		{
			printf("Found device at: 0x%2x\n", i);
		}
	}
}
