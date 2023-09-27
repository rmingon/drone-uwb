/*
 * mpu_6050.h
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#ifndef MAIN_MPU_6050_H_
#define MAIN_MPU_6050_H_

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} ACC;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} GIRO;


void getRawAcc(ACC *acc);

void getRawGyro(GIRO *giro);


#endif /* MAIN_MPU_6050_H_ */
