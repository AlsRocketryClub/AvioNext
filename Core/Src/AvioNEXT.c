#include "main.h"

extern SPI_HandleTypeDef hspi2;



uint8_t HG2_Read_Register(uint8_t addr){
	uint8_t reg_value;
	addr |= (1<<7);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);

	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, &reg_value, 1, 100);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);

	return reg_value;
}

void HG2_Write_Register(uint8_t addr, uint8_t data){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Transmit(&hspi2, &data, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);

}


void HG2_Get_Acc(int16_t* data){
	uint8_t addr = 0x08 | (1<<7);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, data[0], 1, 100);
	HAL_SPI_Receive(&hspi2, data[1], 1, 100);
	HAL_SPI_Receive(&hspi2, data[2], 1, 100);

	HAL_SPI_Receive(&hspi2, data[3], 1, 100);
	HAL_SPI_Receive(&hspi2, data[4], 1, 100);
	HAL_SPI_Receive(&hspi2, data[5], 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
}



uint8_t LG2_Read_Register(uint8_t addr){
	uint8_t reg_value;
	addr |= (1<<7);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);

	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, &reg_value, 1, 100);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);

	return reg_value;
}

void LG2_Write_Register(uint8_t addr, uint8_t data){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Transmit(&hspi2, &data, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);

}

float LG2_Get_Gyro_X(){
	uint8_t Gyro_L = LG2_Read_Register(0x22);
	uint8_t Gyro_H = LG2_Read_Register(0x23);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;
	return omega;
}

float LG2_Get_Gyro_Y(){
	uint8_t Gyro_L = LG2_Read_Register(0x24);
	uint8_t Gyro_H = LG2_Read_Register(0x25);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;

	return omega;
}

float LG2_Get_Gyro_Z(){
	uint8_t Gyro_L = LG2_Read_Register(0x26);
	uint8_t Gyro_H = LG2_Read_Register(0x27);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;

	return omega;
}

float LG2_Get_Acc_X(){
	uint8_t Acc_L = LG2_Read_Register(0x28);
	uint8_t Acc_H = LG2_Read_Register(0x29);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}

float LG2_Get_Acc_Y(){
	uint8_t Acc_L = LG2_Read_Register(0x2A);
	uint8_t Acc_H = LG2_Read_Register(0x2B);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}

float LG2_Get_Acc_Z(){
	uint8_t Acc_L = LG2_Read_Register(0x2C);
	uint8_t Acc_H = LG2_Read_Register(0x2D);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}
