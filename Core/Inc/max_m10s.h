/*
 * max_m10s.h
 *
 *  Created on: Apr 30, 2024
 *      Author: m1cha1s
 */

#ifndef INC_MAX_M10S_H_
#define INC_MAX_M10S_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX_M10S_I2C_ADDR 0x84
#define MAX_M10S_RING_BUFFER_SIZE 1024

int MAX_M10s_init(I2C_HandleTypeDef* i2c);
int MAX_M10s_deinit(I2C_HandleTypeDef* i2c);
int MAX_M10s_reset(I2C_HandleTypeDef* i2c);
void MAX_M10S_parse();

bool MAX_M10s_check_if_exists(I2C_HandleTypeDef* i2c);
void MAX_M10s_poll(I2C_HandleTypeDef* i2c);
void MAX_M10s_irq_handler(I2C_HandleTypeDef* i2c);
int MAX_M10s_bytesToRead(I2C_HandleTypeDef* i2c);

#endif /* INC_MAX_M10S_H_ */
