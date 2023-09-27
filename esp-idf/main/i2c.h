/*
 * i2c.h
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#ifndef MAIN_I2C_H_
#define MAIN_I2C_H_

#include "esp_err.h"
#include "esp_log.h"

void i2cInit();

esp_err_t i2cRead(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

esp_err_t i2cWrite(uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_data);

void i2cScan();


#endif /* MAIN_I2C_H_ */
