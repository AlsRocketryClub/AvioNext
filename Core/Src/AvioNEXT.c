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


void LoRA_sleep(void){
	LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRA_set_frequency(long frequency){
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	LoRA_Write_Register(REG_FRF_MSB, (uint8_t)(frf >> 16));
	LoRA_Write_Register(REG_FRF_MID, (uint8_t)(frf >> 8));
	LoRA_Write_Register(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRA_idle(){
	LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRA_setOCP(uint8_t mA){
	  uint8_t ocpTrim = 27;

	  if (mA <= 120) {
	    ocpTrim = (mA - 45) / 5;
	  } else if (mA <=240) {
	    ocpTrim = (mA + 30) / 10;
	  }

	  LoRA_Write_Register(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRA_setTxPower(int level){
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      LoRA_Write_Register(REG_PA_DAC, 0x87);
      LoRA_setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      LoRA_Write_Register(REG_PA_DAC, 0x84);
      LoRA_setOCP(100);
    }

    LoRA_Write_Register(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void LoRA_explicit_header_mode(){
	LoRA_Write_Register(REG_MODEM_CONFIG_1, LoRA_Read_Register(REG_MODEM_CONFIG_1) & 0xFE);
}

void LoRA_begin(long frequency){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

	uint8_t version = LoRA_Read_Register(REG_VERSION);
    char data_debug[100];
	sprintf( data_debug,  "%x\n", version);
	CDC_Transmit_HS(data_debug, strlen(data_debug));

	LoRA_sleep();
	LoRA_set_frequency(frequency);

	LoRA_Write_Register(REG_FIFO_RX_BASE_ADDR, 0);
	LoRA_Write_Register(REG_FIFO_TX_BASE_ADDR, 0);

	LoRA_Write_Register(REG_LNA, LoRA_Read_Register(REG_LNA) | 0x03); //LNA settings

	LoRA_Write_Register(REG_MODEM_CONFIG_3, 0x04);

	LoRA_setTxPower(17);

}


void LoRA_beginPacket(){
	LoRA_explicit_header_mode();

	LoRA_Write_Register(REG_FIFO_ADDR_PTR, 0);
	LoRA_Write_Register(REG_PAYLOAD_LENGTH, 0);
}

void LoRA_endPacket(){
	LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	while((LoRA_Read_Register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){

	}
	LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

	LoRA_Write_Register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

}


int LoRA_parsePacket(){
	int packetLenght = 0;
	int irqFlags = LoRA_Read_Register(REG_IRQ_FLAGS);

	LoRA_explicit_header_mode();

	LoRA_Write_Register(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		packetLenght = LoRA_Read_Register(REG_RX_NB_BYTES);
		LoRA_Write_Register(REG_FIFO_ADDR_PTR, LoRA_Read_Register(REG_FIFO_RX_CURRENT_ADDR));
		LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

		//LoRA_idle();
	} else if (LoRA_Read_Register(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)){
		LoRA_Write_Register(REG_FIFO_ADDR_PTR, 0);

		LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	}
	return packetLenght;

}

void LoRA_sendPacket(char * data){
    LoRA_beginPacket();
    for(int i = 0; i < strlen(data); i++){
    	LoRA_Write_Register(REG_FIFO, data[i]);
    }
    LoRA_Write_Register(REG_PAYLOAD_LENGTH, strlen(data));
    LoRA_endPacket();
}


int recv_packet(char *LoRA_data, int max_length) {
	int packet_length = LoRA_parsePacket();
	if (max_length - 1 < packet_length) //-1 for the null terminator
			{
		return 0;
	}
	if (packet_length) {
		for (int i = 0; i < packet_length; i++) {
			LoRA_data[i] = LoRA_Read_Register(0x00);
		}
		LoRA_data[packet_length] = '\0';
		return packet_length;
	} else {
		return 0;
	}
}

void reliable_send_packet(char *LoRA_data) {
	uint16_t length = strlen(LoRA_data) + 1; //+1 for the \0
	char acknowledge[length];
	LoRA_sendPacket(LoRA_data);
	while (1) {

		if (recv_packet(acknowledge, length)) {
			//cehck crc
			if (strcmp(acknowledge, LoRA_data) != 0) {
				LoRA_sendPacket(LoRA_data);
			} else {
				break;
			}
		}

		//delay
	}
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

