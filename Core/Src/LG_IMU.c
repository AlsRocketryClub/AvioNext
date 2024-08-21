#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

int current_sensor = 2;


uint8_t LG_Read_Register(uint8_t addr){
	uint8_t reg_value;
	addr |= (1<<7);

	if(current_sensor == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
		HAL_SPI_Receive(&hspi1, &reg_value, 1, 100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
	}else if(current_sensor == 2){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
		HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
		HAL_SPI_Receive(&hspi2, &reg_value, 1, 100);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);
	}


	return reg_value;
}

void LG_Write_Register(uint8_t addr, uint8_t data){
	if(current_sensor == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
		HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
		HAL_SPI_Transmit(&hspi1, &data, 1, 100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
	} else if(current_sensor == 2){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
		HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
		HAL_SPI_Transmit(&hspi2, &data, 1, 100);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);
	}

}

uint8_t LG_Check(){
	current_sensor = 1;
	uint8_t sensors_state = 0;
	if(LG_Read_Register(0x0F) == 0x6B){
		sensors_state |= 0b1;
	}

	current_sensor = 2;
	if(LG_Read_Register(0x0F) == 0x6B){
		sensors_state |= 0b10;
	}

	if(sensors_state & 0b1){
		current_sensor = 1;
	}else{
		current_sensor = 2;
	}
	return sensors_state;

}

void LG_Configure(){
	LG_Write_Register(0x10, 0b00111100); //Accelerometer setup - CTRL1_XL
	LG_Write_Register(0x11, 0b01101000); //Gyroscope setup - CTRL2_G
	LG_Write_Register(0x13, 0b00001100); //disables I2C - CTRL4_C
}

float LG_Get_Gyro_X(){
	uint8_t Gyro_L = LG_Read_Register(0x22);
	uint8_t Gyro_H = LG_Read_Register(0x23);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;
	return omega;
}

float LG_Get_Gyro_Y(){
	uint8_t Gyro_L = LG_Read_Register(0x24);
	uint8_t Gyro_H = LG_Read_Register(0x25);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;

	return omega;
}

float LG_Get_Gyro_Z(){
	uint8_t Gyro_L = LG_Read_Register(0x26);
	uint8_t Gyro_H = LG_Read_Register(0x27);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*35.0/1000;

	return omega;
}

float LG_Get_Acc_X(){
	uint8_t Acc_L = LG_Read_Register(0x28);
	uint8_t Acc_H = LG_Read_Register(0x29);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}

float LG_Get_Acc_Y(){
	uint8_t Acc_L = LG_Read_Register(0x2A);
	uint8_t Acc_H = LG_Read_Register(0x2B);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}

float LG_Get_Acc_Z(){
	uint8_t Acc_L = LG_Read_Register(0x2C);
	uint8_t Acc_H = LG_Read_Register(0x2D);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc / 32767) * 9.8 * 8;
	return AccSI;
}
