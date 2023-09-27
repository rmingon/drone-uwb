/*
 * mpu_6050.c
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "i2c.h"
#include "mpu_6050.h"

#define MPU_ADD 	     0x68
#define MPU6050_PWR      0x6B
#define MPU6050_TEMP     0x41
#define MPU6050_Raw_ACC  0x3B
#define MPU6050_Raw_GYRO 0x43
#define who_Am_I	     0x75

uint8_t mpuName;

#define ACK_VAL    0x0
#define NACK_VAL   0x1
#define I2C_MASTER_FREQ_HZ 100000

void initMPU6050(int mpuAdd,int channelADD) {
	// create and execute the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(mpuAdd << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_PWR,true);
	i2c_master_write_byte(cmd,0x0,true);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000) == ESP_OK) {
		printf("-> MPU6050 - Canal 0x%02x Inicializado\n", channelADD);
	}else{
		printf("MPU6050 it´s not connected  \r\n");
	}
	i2c_cmd_link_delete(cmd);
}


int16_t getTemp(int mpuAdd){
	uint8_t TempH, TempL;
	int16_t  TempRaw, TempDeg;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(mpuAdd << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,MPU6050_TEMP,true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (mpuAdd << 1) | I2C_MASTER_READ, true);
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
	uint8_t AccXH, AccXL, AccYH, AccYL,AccZH, AccZL ;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(MPU_ADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,0x3B,true);
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

	acc->x = (AccXH << 8 | AccXL);
	acc->y = (AccYH << 8 | AccYL);
	acc->z = (AccZH << 8 | AccZL);

	ESP_LOGI("R_ACC", "AccX : %d", acc->x);
	ESP_LOGI("R_ACC", "AccY : %d", acc->y);
	ESP_LOGI("R_ACC", "AccZ : %d", acc->z);
}

void getRawGyro(GIRO *giro) {
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
    giro->x = (GyroXH << 8 | GyroXL);
    giro->y = (GyroYH << 8 | GyroYL);
    giro->z = (GyroZH << 8 | GyroZL);
    ESP_LOGI("G_ACC", "GyroX : %d", giro->x);
    ESP_LOGI("G_ACC", "GyroY : %d", giro->y);
    ESP_LOGI("G_ACC", "GyroZ : %d", giro->z);
}
