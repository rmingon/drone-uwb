/*
 * mpu_6050.c
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c.h"
#include "mpu_6050.h"

extern QueueHandle_t GyroQueue;

extern QueueHandle_t AccQueue;

GYRO gyro_base_level;

bool gyro_initialized = false;

SemaphoreHandle_t xSemaphori2cRead = NULL;

static const char *TAG = "MPU6050";

#define MPU_ADD 	     0x68
#define MPU6050_PWR      0x6B
#define MPU6050_TEMP     0x41
#define MPU6050_RATE	 0x19
#define MPU6050_Raw_ACC  0x3B
#define MPU6050_Raw_GYRO 0x43

#define ACK_VAL    0x0
#define NACK_VAL   0x1
#define I2C_MASTER_FREQ_HZ 100000

void initMPU6050() {
	// create and execute the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_PWR,true);
	i2c_master_write_byte(cmd,0x0,true);
	i2c_master_start(cmd);
	// Set filter rate to 1khz
	i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_RATE,true);
	i2c_master_write_byte(cmd,0x07,true);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000) == ESP_OK) {
		ESP_LOGI(TAG, "-> MPU6050 - OK\n");
	}else{
		ESP_LOGE(TAG, "MPU6050 ERROR  \r\n");
	}
	i2c_cmd_link_delete(cmd);

	xSemaphori2cRead = xSemaphoreCreateBinary();
}

int16_t getTemp() {
	uint8_t TempH, TempL;
	int16_t  TempRaw, TempDeg;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_TEMP,true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MPU_ADD << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &TempH, ACK_VAL);
	i2c_master_read_byte(cmd, &TempL, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	i2c_cmd_link_delete(cmd);
	TempRaw = (TempH << 8 | TempL);
	TempDeg = TempRaw/340+36.53;
	//printf("\n Temp: %d\n", TempDeg);
	return TempDeg;
}

void getRawAcc(ACC *acc) {
	uint8_t AccXH, AccXL, AccYH, AccYL,AccZH, AccZL;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_Raw_ACC ,true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MPU_ADD << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &AccXH, ACK_VAL);
	i2c_master_read_byte(cmd, &AccXL, ACK_VAL);
	i2c_master_read_byte(cmd, &AccYH, ACK_VAL);
	i2c_master_read_byte(cmd, &AccYL, ACK_VAL);
	i2c_master_read_byte(cmd, &AccZH, ACK_VAL);
	i2c_master_read_byte(cmd, &AccZL, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
	i2c_cmd_link_delete(cmd);
	acc->x = (int16_t)(AccXH << 8 | AccXL) / 16384.0;
	acc->y = (int16_t)(AccYH << 8 | AccYL) / 16384.0;
	acc->z = (int16_t)(AccZH << 8 | AccZL) / 16384.0;
}

void getRawGyro(GYRO *gyro) {
    uint8_t GyroXH, GyroXL, GyroYH, GyroYL,GyroZH, GyroZL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
    i2c_master_write_byte(cmd,MPU6050_Raw_GYRO,true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADD << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &GyroXH, ACK_VAL);
    i2c_master_read_byte(cmd, &GyroXL, ACK_VAL);
    i2c_master_read_byte(cmd, &GyroYH, ACK_VAL);
    i2c_master_read_byte(cmd, &GyroYL, ACK_VAL);
    i2c_master_read_byte(cmd, &GyroZH, ACK_VAL);
    i2c_master_read_byte(cmd, &GyroZL, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000);
    i2c_cmd_link_delete(cmd);
    gyro->x = (int16_t)(GyroXH << 8 | GyroXL) / 131.0;
    gyro->y = (int16_t)(GyroYH << 8 | GyroYL) / 131.0;
    gyro->z = (int16_t)(GyroZH << 8 | GyroZL) / 131.0;
}

void gyroTask(void *pvParameters) {
	while (1) {
    	GYRO gyro;
    	xSemaphoreTake(xSemaphori2cRead, 10);
    	getRawGyro(&gyro);
    	xSemaphoreGive(xSemaphori2cRead);
    	if (gyro_initialized == false) {
    		gyro_base_level.x = gyro.x;
    		gyro_base_level.y = gyro.y;
    		gyro_base_level.z = gyro.z;
    		gyro_initialized = true;
    	}
    	ESP_LOGI(TAG, "GYRO X %.2f", gyro.x - gyro_base_level.x);
    	ESP_LOGI(TAG, "GYRO Y %.2f", gyro.y - gyro_base_level.y);
    	ESP_LOGI(TAG, "GYRO Z %.2f", gyro.z - gyro_base_level.z);

        xQueueSend(GyroQueue, &gyro, 10);

        vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void accTask(void *pvParameters) {
	while (1) {
    	ACC acc;
    	xSemaphoreTake(xSemaphori2cRead, 10);
    	getRawAcc(&acc);
    	xSemaphoreGive(xSemaphori2cRead);
    	ESP_LOGI(TAG, "ACC X %.2f", acc.x);
    	ESP_LOGI(TAG, "ACC Y %.2f", acc.y);
    	ESP_LOGI(TAG, "ACC Z %.2f", acc.z);

        xQueueSend(AccQueue, &acc, 10);

        vTaskDelay(pdMS_TO_TICKS(20));
	}
}
