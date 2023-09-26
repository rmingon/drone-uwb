/*
 * i2c.h
 *
 *  Created on: Sep 26, 2023
 *      Author: roro
 */

#ifndef MAIN_I2C_H_
#define MAIN_I2C_H_

void i2cInit();

int i2cRead(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

int i2cWrite(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);


#endif /* MAIN_I2C_H_ */
