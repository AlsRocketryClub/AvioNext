#include "main.h"
#include "LoRA"
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;


#define NUM_LEDS_0 5
#define NUM_LEDS_1 5
#define NUM_LEDS_2 2
#define NUM_LEDS_3 2

//first coordinate defines on which string the LED is positioned, second determines the position
const int LEDS_lookup[NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3][2] = {
		{ 2, 1 }, //LED0: CAN
		{ 2, 0 }, //LED1: GPS
		{ 3, 0 }, //LED2: LoRA
		{ 3, 1 }, //LED3: SD Card
		{ 0, 0 }, //LED4: HG1
		{ 0, 1 }, //LED5: LG1
		{ 0, 2 }, //LED6: BAR1
		{ 1, 0 }, //LED7: ARM
		{ 1, 1 }, //LED8: HG2
		{ 1, 2 }, //LED9: LG2
		{ 1, 3 }, //LED10: BAR2
		{ 0, 3 }, //LED11: REG1
		{ 1, 4 }, //LED12: REG2
		{ 0, 4 }  //LED13: BATT

};
const int LED_num_max = 6;
const int LED_order[NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3] = { 0, //LED0: CAN
		0, //LED1: GPS
		0, //LED2: LoRA
		1, //LED3: SD Card
		2, //LED4: HG1
		3, //LED5: LG1
		4, //LED6: BAR1
		3, //LED7: ARM
		4, //LED8: HG2
		3, //LED9: LG2
		2, //LED10: BAR2
		5, //LED11: REG1
		5, //LED12: REG2
		6  //LED13: BATT
		};


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









uint8_t LoRA_Read_Register(uint8_t addr){
	uint8_t reg_value;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

	HAL_SPI_Transmit(&hspi3, &addr, 1, 100);
	HAL_SPI_Receive(&hspi3, &reg_value, 1, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

	return reg_value;
}

void LoRA_Write_Register(uint8_t addr, uint8_t data){
	addr |= (1<<7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	HAL_SPI_Transmit(&hspi3, &addr, 1, 100);
	HAL_SPI_Transmit(&hspi3, &data, 1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

}

float LG2_Get_Gyro_X(){
	uint8_t Gyro_L = LG2_Read_Register(0x22);
	uint8_t Gyro_H = LG2_Read_Register(0x23);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*(1*8.75)/1000;
	return omega;
}

float LG2_Get_Gyro_Y(){
	uint8_t Gyro_L = LG2_Read_Register(0x24);
	uint8_t Gyro_H = LG2_Read_Register(0x25);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*(1*8.75)/1000;

	return omega;
}

float LG2_Get_Gyro_Z(){
	uint8_t Gyro_L = LG2_Read_Register(0x26);
	uint8_t Gyro_H = LG2_Read_Register(0x27);
	int16_t Gyro = ((int16_t) Gyro_H << 8) | Gyro_L;
	float omega = (float)Gyro*(1*8.75)/1000;

	return omega;
}

float LG2_Get_Acc_X(){
	uint8_t Acc_L = LG2_Read_Register(0x28);
	uint8_t Acc_H = LG2_Read_Register(0x29);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc *  (0.061*9.81) /1000)- 0.134732 - 0.104937;
	return AccSI;
}

float LG2_Get_Acc_Y(){
	uint8_t Acc_L = LG2_Read_Register(0x2A);
	uint8_t Acc_H = LG2_Read_Register(0x2B);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc *  (0.061*9.81) /1000) + 0.384580;
	return AccSI;
}

float LG2_Get_Acc_Z(){
	uint8_t Acc_L = LG2_Read_Register(0x2C);
	uint8_t Acc_H = LG2_Read_Register(0x2D);
	int16_t Acc = ((int16_t) Acc_H << 8) | Acc_L;

	float AccSI = ((float)Acc *  (0.061*9.81) /1000) + 0.005841;
	return AccSI;
}

void setServo(int servoNum, float angle) {

	uint16_t timerVal = (int) (3000 + (4000 * (angle / 180)));
	switch (servoNum) {
	case 1:
		TIM4->CCR4 = timerVal;
		break;
	case 2:
		TIM4->CCR3 = timerVal;
		break;
	case 3:
		TIM4->CCR2 = timerVal;
		break;
	case 4:
		TIM4->CCR1 = timerVal;
		break;

	default:
		break;
	}
}



//this function looks like this: /\_/\_/\_/\_
//so it's triangles with spaces between them
double triangle_space(double x) {
	const double LENGTH = 3;
	double normalized = fmod(fabs(x), LENGTH);
	if (normalized <= LENGTH / 3) {
		return LENGTH / 3 - normalized;
	} else if (normalized <= LENGTH * 2 / 3) {
		return 0;
	} else {
		return normalized - LENGTH * 2 / 3;
	}
}

int write_EEPROM(uint32_t address, uint8_t data) {
	if (address > 0x1FFFF) {
		return -1;
	}

	uint8_t writeAddress = (uint8_t) (0b10100000
			| ((address >> 16) & 0b11111110));

	uint16_t memAddr = (uint16_t) address;

	HAL_I2C_Mem_Write(&hi2c2, writeAddress, memAddr, I2C_MEMADD_SIZE_16BIT,
			&data, 1, 100);
	return 0;
}

uint8_t read_EEPROM(uint32_t address) {
	if (address > 0x1FFFF) {
		return -1;
	}
	uint8_t writeAddress = (uint8_t) (0b10100001 | ((address >> 16)));

	uint16_t memAddr = (uint16_t) address;

	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c2, writeAddress, memAddr, 2, &data, 1, 100);
	return data;
}

void setLEDs(uint8_t LED_Color_Data[14][3]) {

	static uint32_t LED_PWM_Data_0[(NUM_LEDS_0 * 24) + 58];
	static uint32_t LED_PWM_Data_1[(NUM_LEDS_1 * 24) + 58];
	static uint32_t LED_PWM_Data_2[(NUM_LEDS_2 * 24) + 58];
	static uint32_t LED_PWM_Data_3[(NUM_LEDS_3 * 24) + 58];

	for (int i = 0; i < NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3;
			i++) {
		switch (LEDS_lookup[i][0]) { //checks in which string the LED is
		case 0:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_0[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 60;
					} else {
						LED_PWM_Data_0[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_0 * 24) + 8; i < (NUM_LEDS_0 * 24) + 58;
					i++) {
				LED_PWM_Data_0[i] = 0;
			}
			break;
		case 1:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_1[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 60;
					} else {
						LED_PWM_Data_1[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_1 * 24) + 8; i < (NUM_LEDS_1 * 24) + 58;
					i++) {
				LED_PWM_Data_1[i] = 0;
			}
			break;
		case 2:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_2[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 60;
					} else {
						LED_PWM_Data_2[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_2 * 24) + 8; i < (NUM_LEDS_2 * 24) + 58;
					i++) {
				LED_PWM_Data_2[i] = 0;
			}
			break;
		case 3:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_3[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 60;
					} else {
						LED_PWM_Data_3[n + (8 * j) + (24 * LEDS_lookup[i][1])
								+ 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_3 * 24) + 8; i < (NUM_LEDS_3 * 24) + 58;
					i++) {
				LED_PWM_Data_3[i] = 0;
			}
			break;
		default:
			break;
		}
	}

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, LED_PWM_Data_0,
			(NUM_LEDS_0 * 24) + 58); //DMA for LEDS 0
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, LED_PWM_Data_1,
			(NUM_LEDS_1 * 24) + 58); //DMA for LEDS 1
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, LED_PWM_Data_2,
			(NUM_LEDS_2 * 24) + 58); //DMA for LEDS 2
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, LED_PWM_Data_3,
			(NUM_LEDS_3 * 24) + 58); //DMA for LEDS 3

}