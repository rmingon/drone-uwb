/*
 * mpu_6050.h
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#ifndef MAIN_MPU_6050_H_
#define MAIN_MPU_6050_H_

typedef struct {
	float x;
	float y;
	float z;
} ACC;

typedef struct {
	float x;
	float y;
	float z;
} GYRO;

void initMPU6050();

void getRawAcc(ACC *acc);

void getRawGyro(GYRO *giro);

int16_t getTemp();

void accTask(void *pvParameters);

void gyroTask(void *pvParameters);


#endif /* MAIN_MPU_6050_H_ */
