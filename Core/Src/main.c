/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRA"
#include "AvioNEXT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LEDS_0 5
#define NUM_LEDS_1 5
#define NUM_LEDS_2 2
#define NUM_LEDS_3 2
#define usbBufferLen 256

#define MAX_PAYLOAD_LENGHT 250

uint8_t usbDataBuffer[usbBufferLen];
uint32_t usbBytesReady = 0;

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
		{ 1, 3 }, //LED10: BAR1
		{ 0, 3 }, //LED11: REG1
		{ 1, 4 }, //LED12: REG2
		{ 0, 4 }  //LED13: BATT

};
const int LED_num_max=6;
const int LED_order[NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3] = {
		0, //LED0: CAN
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

uint32_t LED_PWM_Data_0[(NUM_LEDS_0 * 24) + 58];
uint32_t LED_PWM_Data_1[(NUM_LEDS_1 * 24) + 58];
uint32_t LED_PWM_Data_2[(NUM_LEDS_2 * 24) + 58];
uint32_t LED_PWM_Data_3[(NUM_LEDS_3 * 24) + 58];

uint32_t LED_PWM_Data_Combined[(5 * 24) + 50][4];

uint32_t LED_Color_Data[NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3][3];


uint16_t DMA_data;

uint16_t read_Data;
;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

FDCAN_HandleTypeDef hfdcan3;

I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim13;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim3_ch2;
DMA_HandleTypeDef hdma_tim3_ch1;
DMA_HandleTypeDef hdma_tim4_ch3;
DMA_HandleTypeDef hdma_tim5_ch4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int datasentflag = 0;


void setServo(int servoNum, float angle){

	uint16_t timerVal =(int)( 3000 + (4000 * (angle/180)));
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

void setLEDs(void) {
	for (int i = 0; i < NUM_LEDS_0 + NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3; i++) {
		switch (LEDS_lookup[i][0]) { //checks in which string the LED is
		case 0:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_0[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 60;
					} else {
						LED_PWM_Data_0[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_0 * 24) + 8; i < (NUM_LEDS_0 * 24) + 58; i++) {
				LED_PWM_Data_0[i] = 0;
			}
			break;
		case 1:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_1[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 60;
					} else {
						LED_PWM_Data_1[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_1 * 24) + 8; i < (NUM_LEDS_1 * 24) + 58; i++) {
				LED_PWM_Data_1[i] = 0;
			}
			break;
		case 2:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_2[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 60;
					} else {
						LED_PWM_Data_2[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_2 * 24) + 8; i < (NUM_LEDS_2 * 24) + 58; i++) {
				LED_PWM_Data_2[i] = 0;
			}
			break;
		case 3:
			for (int j = 0; j < 3; j++) {
				for (int n = 0; n < 8; n++) {
					if (LED_Color_Data[i][j] & (128 >> n)) {
						LED_PWM_Data_3[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 60;
					} else {
						LED_PWM_Data_3[n + (8 * j) + (24 * LEDS_lookup[i][1]) + 8] = 30;
					}
				}
			}
			for (int i = (NUM_LEDS_3 * 24) + 8; i < (NUM_LEDS_3 * 24) + 58; i++) {
				LED_PWM_Data_3[i] = 0;
			}
			break;
		default:
			break;
		}
	}

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, LED_PWM_Data_0, (NUM_LEDS_0 * 24) + 58); //DMA for LEDS 0
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, LED_PWM_Data_1, (NUM_LEDS_1 * 24) + 58); //DMA for LEDS 1
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, LED_PWM_Data_2, (NUM_LEDS_2 * 24) + 58); //DMA for LEDS 2
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, LED_PWM_Data_3, (NUM_LEDS_3 * 24) + 58); //DMA for LEDS 3

}


//this function looks like this: /\_/\_/\_/\_
//so it's triangles with spaces between them
double triangle_space(double x)
{
	const double LENGTH = 3;
	double normalized = fmod(fabs(x),LENGTH);
	if(normalized <= LENGTH/3)
	{
		return LENGTH/3 - normalized;
	}
	else if(normalized <= LENGTH*2/3)
	{
		return 0;
	}
	else
	{
		return normalized - LENGTH*2/3;
	}
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
	LoRA_set_frequency(868000000);

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
	char sent[300];
	sprintf(sent, "sent: %s\n", data);
	HAL_Delay(100);
	CDC_Transmit_HS(sent, strlen(sent));
    LoRA_beginPacket();
    for(int i = 0; i < strlen(data); i++){
    	LoRA_Write_Register(REG_FIFO, data[i]);
    }
    LoRA_Write_Register(REG_PAYLOAD_LENGTH, strlen(data));
    LoRA_endPacket();
}

int write_EEPROM(uint32_t address, uint8_t data){
	if(address > 0x1FFFF){
		return -1;
	}

	uint8_t writeAddress = (uint8_t)(0b10100000 | ((address >> 16) & 0b11111110));

	uint16_t memAddr = (uint16_t)address;

	HAL_I2C_Mem_Write(&hi2c2, writeAddress, memAddr, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
	return 0;
}

uint8_t read_EEPROM(uint32_t address){
	if(address > 0x1FFFF){
		return -1;
	}
	uint8_t writeAddress = (uint8_t)(0b10100001 | ((address >> 16)));

	uint16_t memAddr = (uint16_t)address;


	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c2, writeAddress, memAddr, 2, &data, 1, 100);
	return data;
}

int mount_SD(){
	int status = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	return status;
}

int disarm(char* state)
{
  HAL_GPIO_WritePin(ARM1_GPIO_Port, ARM1_Pin, 0);
  HAL_GPIO_WritePin(ARM2_GPIO_Port, ARM2_Pin, 0);

  HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, 0);
  HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, 0);
  HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, 0);
  HAL_GPIO_WritePin(PYRO4_GPIO_Port, PYRO4_Pin, 0);

  HAL_GPIO_WritePin(PYRO5_GPIO_Port, PYRO5_Pin, 0);
  HAL_GPIO_WritePin(PYRO6_GPIO_Port, PYRO6_Pin, 0);
  HAL_GPIO_WritePin(PYRO7_GPIO_Port, PYRO7_Pin, 0);
  HAL_GPIO_WritePin(PYRO8_GPIO_Port, PYRO8_Pin, 0);

  LED_Color_Data[7][0] = 255;
  LED_Color_Data[7][1] = 0;
  LED_Color_Data[7][2] = 0;
  setLEDs();

  strcpy(state,"DISARMED");
  return 0;
}

int arm(char* state)
{
  HAL_GPIO_WritePin(ARM1_GPIO_Port, ARM1_Pin, 1);
  HAL_GPIO_WritePin(ARM2_GPIO_Port, ARM2_Pin, 1);


  strcpy(state,"ARMED");
  LED_Color_Data[7][0] = 0;
  LED_Color_Data[7][1] = 255;
  LED_Color_Data[7][2] = 0;
  setLEDs();
  return 0;
}

int recv_packet(char* LoRA_data, int max_length)
{
  int packet_length = LoRA_parsePacket();
  if(max_length-1 < packet_length) //-1 for the null terminator
  {
    return 0;
  }
  if(packet_length){
    for(int i = 0; i < packet_length; i++){
      LoRA_data[i] = LoRA_Read_Register(0x00);
    }
    LoRA_data[packet_length] = '\0';

    char rec[300];
    sprintf(rec, "received: %s\n", LoRA_data);
    CDC_Transmit_HS(rec, strlen(rec));
    return packet_length;
  }
  else{
    return 0;
  }
}

void reliable_send_packet(char *LoRA_data) {
	uint16_t length = strlen(LoRA_data) + 1; //+1 for the \0
	char acknowledge[length];
	uint32_t lastTime = HAL_GetTick();
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

		if (HAL_GetTick() - lastTime > 1000) {
			LoRA_sendPacket(LoRA_data);
			lastTime = HAL_GetTick();
		}
	}
}

void pyro_continuity_check()
{
	uint8_t CONTS[8];
	CONTS[0] = HAL_GPIO_ReadPin(CONT1_GPIO_Port, CONT1_Pin);
	CONTS[1] = HAL_GPIO_ReadPin(CONT2_GPIO_Port, CONT2_Pin);
	CONTS[2] = HAL_GPIO_ReadPin(CONT3_GPIO_Port, CONT3_Pin);
	CONTS[3] = HAL_GPIO_ReadPin(CONT4_GPIO_Port, CONT4_Pin);
	CONTS[4] = HAL_GPIO_ReadPin(CONT5_GPIO_Port, CONT5_Pin);
	CONTS[5] = HAL_GPIO_ReadPin(CONT6_GPIO_Port, CONT6_Pin);
	CONTS[6] = HAL_GPIO_ReadPin(CONT7_GPIO_Port, CONT7_Pin);
	CONTS[7] = HAL_GPIO_ReadPin(CONT8_GPIO_Port, CONT8_Pin);

	char message[100];
	for(int i=0; i<8; i++)
	{
		if(CONTS[i])
		{
			sprintf( message,  "PYRO %d DOESN'T HAVE CONTINUITY", i+1);
		}
		else
		{
			sprintf( message,  "PYRO %d HAS CONTINUITY", i+1);
		}

		reliable_send_packet(message);
	}
}

int usbReceiveHandle(char* output){
	uint32_t temp = usbBytesReady;
	if(usbBytesReady > 0){
		if(usbBytesReady > 256){
			//crash(2);
		}
		memcpy(output, usbDataBuffer, usbBytesReady);
		output[usbBytesReady] = '\0';
		usbBytesReady = 0;
	}
	return temp;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten, bytesread; /* File write/read counts */
	uint8_t wtext[] = "STM32 FATFS works great!"; /* File write buffer */
	uint8_t rtext[_MAX_SS];/* File read buffer */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_FDCAN3_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_SDMMC2_SD_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

	const int MAX = 50;
	const double SPEED = 2.0/2000;
	const double r_offset = 0;
	const double g_offset = 1;
	const double b_offset = 2;

	LG2_Write_Register(0x10, 0b00111100); //Accelerometer setup - CTRL1_XL
	LG2_Write_Register(0x11, 0b00110000); //Gyroscope setup - CTRL2_G
	LG2_Write_Register(0x13, 0b00000100); //disables I2C - CTRL4_C

	HAL_Delay(3000);
	HG2_Write_Register(0x1C, 0b10111111);
	HAL_Delay(2);

	HG2_Write_Register(0x1B, 0b01011000);
	HG2_Write_Register(0x1B, 0b11011000);

	float rotZ = 0;
	uint32_t lastTime = 0;

	float calOmegaX = 0;
	float calOmegaY = 0;
	float calOmegaZ = 0;
	//HAL_Delay(2000);
	for(int i = 0; i < 500; i++){
		calOmegaX += LG2_Get_Gyro_X();
		calOmegaY += LG2_Get_Gyro_Y();
		calOmegaZ += LG2_Get_Gyro_Z();

		//HAL_Delay(20);
	}
	calOmegaX /= 500;
	calOmegaY /= 500;
	calOmegaZ /= 500;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_Delay(200);


	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);


    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    setServo(1, 90);
    setServo(2, 180);
    setServo(3, 0);
    setServo(4, 45);


    //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	char data_gyro[100];
//    sprintf( data_gyro,  "Start\n");
//    CDC_Transmit_HS(data_gyro, strlen(data_gyro));
//
//    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
//    	{
//    	Error_Handler();
//
//    	}
//    	else
//    	{
//    		if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
//
//    	    {
//    			Error_Handler();
//    	    }
//    		else
//    		{
//    			//Open file for writing (Create)
//    			if(f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
//    			{
//    				Error_Handler();
//    			}
//    			else
//    			{
//
//    				//Write to the text file
//    				res = f_write(&SDFile, wtext, strlen((char *)wtext), (void *)&byteswritten);
//    				if((byteswritten == 0) || (res != FR_OK))
//    				{
//
//    					Error_Handler();
//    				}
//    				else
//    				{
//
//    					f_close(&SDFile);
//    				}
//    			}
//    		}
//    	}
//    	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
	  LoRA_begin(868000000);
/*
	while(1) {
		LoRA_sendPacket("whatever");
		HAL_Delay(1000);
	}
*/

	int connected = 0;
	long last_packet = 0;
	int ARMED = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//HAL_ADC_Start_DMA(&hadc3, &read_Data, 1);

  int max_packet_count = 0;
  int stream_counter = 0;
  char state[MAX_PAYLOAD_LENGHT] = "";
  char command[MAX_PAYLOAD_LENGHT];
  char acknowledge[MAX_PAYLOAD_LENGHT];
  char previous_packet[MAX_PAYLOAD_LENGHT];
  char recieved_packet[MAX_PAYLOAD_LENGHT];
  char response_packet[MAX_PAYLOAD_LENGHT];
  char sendMessage[MAX_PAYLOAD_LENGHT];
  int last = 0;
  int packets_streamed = 50;
  int packetId;
  int have_recieved_anything = 0;
  char communication_state[50] = "SENDING RELIABLE";
  uint32_t previousTime = HAL_GetTick();
  disarm(state);
  LoRA_begin(868000000);

  /*while (1) {
	  if(recv_packet(recieved_packet, MAX_PAYLOAD_LENGHT))
	  {
		  CDC_Transmit_HS(recieved_packet, strlen(recieved_packet));
	  }
  }*/

while (1) {
    if(strcmp(communication_state,"RECEIVING RELIABLE") == 0)
    {
      if(recv_packet(recieved_packet, MAX_PAYLOAD_LENGHT))
      {
    	  have_recieved_anything = 1;
        previousTime = HAL_GetTick();
        //HAL_Delay(100);
        //CDC_Transmit_HS("is arm 0succ\n", strlen("is arm 0succ\n"));
        //HAL_Delay(100);
        if(sscanf(recieved_packet, "$ %s", state) == 1)
        {
          strcpy(communication_state,"SENDING RELIABLE");
        }
        else if(sscanf(recieved_packet, "! %d", &max_packet_count) == 1)
        {
          strcpy(communication_state,"SENDING STREAM");
        }
        else if(strcmp(recieved_packet, previous_packet)==0)
        {
          //send acknowledge again
          LoRA_sendPacket(recieved_packet);
        }
        else
        {
          //CDC_Transmit_HS("is arm 1succ\n", strlen("is arm 1succ\n"));
          //HAL_Delay(100);
          strcpy(previous_packet, recieved_packet);
          LoRA_sendPacket(recieved_packet);
          //HAL_Delay(100);
          CDC_Transmit_HS(recieved_packet, strlen(recieved_packet));
        }
      } else if (HAL_GetTick()-previousTime > 1000)
      /*else if((!have_recieved_anything && HAL_GetTick()-previousTime > 1000) ||
    		  (have_recieved_anything && HAL_GetTick()-previousTime > 5000))*/
      {
        previousTime = HAL_GetTick();
        //give up SENDING
        LoRA_sendPacket("$");
      }
    }
    else if(strcmp(communication_state,"RECEIVING STREAM") == 0)
    {
      if(recv_packet(recieved_packet, MAX_PAYLOAD_LENGHT))
      {
        previousTime = HAL_GetTick();
        if(sscanf(recieved_packet, "$ %s", state) == 1)
        {
          strcpy(communication_state,"SENDING RELIABLE");
        }
        else
        {
          CDC_Transmit_HS(recieved_packet, strlen(recieved_packet));
        }
      }
      else if(HAL_GetTick()-previousTime > 1000)
      {
        previousTime = HAL_GetTick();
        //give up SENDING
        sprintf(sendMessage, "! %d", packets_streamed);
        LoRA_sendPacket(sendMessage);
      }
    }
    else if(strcmp(communication_state,"SENDING STREAM") == 0)
    {
      if(max_packet_count == 0)
      {
        strcpy(communication_state,"RECEIVING RELIABLE");
        have_recieved_anything = 0;
        LoRA_sendPacket("$");
      }
      else
      {
        //send whatever
        max_packet_count--;
      }

    }
    else if(strcmp(communication_state,"SENDING RELIABLE") == 0)
    {
	  	 CDC_Transmit_HS(state, strlen(state));

    	//get input
    	char input[usbBufferLen];
    	usbReceiveHandle(input);

    	while(!usbReceiveHandle(input))
    	{}

      reliable_send_packet(input);

	  	char debug[usbBufferLen+10];
	  	sprintf(debug, "Debug: %s\n", input);
	  	CDC_Transmit_HS(debug, strlen(debug));

      if(strcmp(input,"FIRE")==0)
      {
        strcpy(communication_state,"RECEIVING STREAM");
        sprintf(sendMessage, "! %d", packets_streamed);
        LoRA_sendPacket(sendMessage);
      }
      else
      {
        strcpy(communication_state,"RECEIVING RELIABLE");
        have_recieved_anything = 0;
        HAL_Delay(100);
        LoRA_sendPacket("$");
      }
    }


  //HAL_Delay(3000);
		//WS2812_Send();
		//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		//TIM4->CCR3 = *ptr;
//		for(int i = 0; i < 14; i++){
//
//			int time = HAL_GetTick();
//			double height_offset = LED_order[i]*1.0/LED_num_max;
//			double color_offset = time*SPEED + height_offset;
//
//			LED_Color_Data[i][0] = (uint32_t)MAX*triangle_space(color_offset+r_offset);
//			LED_Color_Data[i][1] = (uint32_t)MAX*triangle_space(color_offset+g_offset);
//			LED_Color_Data[i][2] = (uint32_t)MAX*triangle_space(color_offset+b_offset);
//		}

		float timeElapsed = ((float)(HAL_GetTick() - lastTime)) / 1000;

		//float omegaZ = LG2_Get_Gyro_Z() - calOmegaZ;
		//rotZ += omegaZ * timeElapsed;

		//int16_t accZ = LG2_Get_Acc_X();

		//HG2_Get_Acc();
		//int16_t AccX = (int16_t)(HG2_Acc[1] << 8) | HG2_Acc[0];
		//float AccX = LG2_Get_Acc_X();
		//float AccY = LG2_Get_Acc_Y();
		//float AccZ = LG2_Get_Acc_Z();

		//float GyroX = LG2_Get_Gyro_X() - calOmegaX;
		//float GyroY = LG2_Get_Gyro_Y() - calOmegaY;
		//float GyroZ = LG2_Get_Gyro_Z() - calOmegaZ;

		lastTime = HAL_GetTick();

//		int packetLenght = LoRA_parsePacket();
//		if(packetLenght > 0){
//			for(int i = 0; i < packetLenght; i++){
//				data_gyro[i] = LoRA_Read_Register(0x00);
//			}
//		    CDC_Transmit_HS(data_gyro, strlen(packetLenght));
//
//		}

		//write_EEPROM(1, 1);

	     // Start ADC Conversion
		//HAL_Delay(100);
    /*
		if(HAL_GetTick() - last_packet > 1000){
			connected = 0;
		}

		if(connected){
			LED_Color_Data[2][0] = 255;
			LED_Color_Data[2][1] = 0;
			LED_Color_Data[2][2] = 0;
		}else{
			LED_Color_Data[2][0] = 120;
			LED_Color_Data[2][1] = 255;
			LED_Color_Data[2][2] = 0;
		}

		int packet_lenght = LoRA_parsePacket();
		char LoRA_data[50];
		if(packet_lenght){


			connected = 1;
			last_packet = HAL_GetTick();
			for(int i = 0; i < packet_lenght; i++){
				LoRA_data[i] = LoRA_Read_Register(0x00);
			}
			LoRA_data[packet_lenght] = '\0';
			//LoRA_data[packet_lenght+1] = '';
			char data_gyro[50];
		    //sprintf( data_gyro,  "%d   %d\n", strlen(LoRA_data), packet_lenght);
		    //CDC_Transmit_HS(data_gyro, strlen(data_gyro));

			//CDC_Transmit_HS(LoRA_data, packet_lenght);

		    if(strcmp(LoRA_data, "ARM") == 0){
		    	ARMED = 1;
		    	setLEDs();
		    	LoRA_sendPacket("ARM SUCCESS");
		    }
		    if(strcmp(LoRA_data, "DISARM") == 0){
		    	ARMED = 0;
		    	LoRA_sendPacket("DISARM SUCCESS");
		    }
		    if(strcmp(LoRA_data, "CONT") == 0){

		    	uint8_t CONTS[8];
		    	CONTS[0] = HAL_GPIO_ReadPin(CONT1_GPIO_Port, CONT1_Pin);
		    	CONTS[1] = HAL_GPIO_ReadPin(CONT2_GPIO_Port, CONT2_Pin);
		    	CONTS[2] = HAL_GPIO_ReadPin(CONT3_GPIO_Port, CONT3_Pin);
		    	CONTS[3] = HAL_GPIO_ReadPin(CONT4_GPIO_Port, CONT4_Pin);
		    	CONTS[4] = HAL_GPIO_ReadPin(CONT5_GPIO_Port, CONT5_Pin);
		    	CONTS[5] = HAL_GPIO_ReadPin(CONT6_GPIO_Port, CONT6_Pin);
		    	CONTS[6] = HAL_GPIO_ReadPin(CONT7_GPIO_Port, CONT7_Pin);
		    	CONTS[7] = HAL_GPIO_ReadPin(CONT8_GPIO_Port, CONT8_Pin);

	    		char message[100];
		    	for(int i=0; i<8; i++)
		    	{
		    		if(CONTS[i])
		    		{
		    			sprintf( message,  "PYRO %d DOESN'T HAVE CONTINUITY", i+1);
		    		}
		    		else
		    		{
		    			sprintf( message,  "PYRO %d HAS CONTINUITY", i+1);
		    		}

		    		LoRA_sendPacket(message);
		    		HAL_Delay(100);
		    	}

		    }
        

        if(strcmp(LoRA_data, "STATIC_FIRE") == 0)
        {
          if(ARMED)
          {
            LoRA_sendPacket("PYRO 1 FIRED");
            //HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, 1);
            char LoRA_data[50];
            int logging = 1;
            while(logging)
            {
            	int packet_lenght;
            	long startTime = HAL_GetTick();
            	while( HAL_GetTick() - startTime < 10){
            		packet_lenght = LoRA_parsePacket();
                	HAL_Delay(0.1);
            }
              if(packet_lenght)
              {
                //flush data from buffer
                //last_packet = HAL_GetTick();
                for(int i = 0; i < packet_lenght; i++){
                  LoRA_data[i] = LoRA_Read_Register(0x00);
                }
                LoRA_data[packet_lenght] = '\0';
                if(strcmp(LoRA_data, "STOP") == 0)
                {
                  logging=0;
                }
              }
              LoRA_sendPacket("Fake data: 21231, 99999");
            }

          }

        }


		    int channel_num;
		    char fire_data[50];
		    sscanf(LoRA_data, "%s %d", fire_data, &channel_num);
		    if(strcmp(fire_data, "FIRE") == 0){
		    	if(ARMED){
					switch (channel_num) {
						case 1:
				    		LoRA_sendPacket("PYRO 1 FIRED");

							HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, 1);
							break;
						case 2:
							HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, 1);
							break;
						case 3:
							HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, 1);
							break;
						case 4:
							HAL_GPIO_WritePin(PYRO4_GPIO_Port, PYRO4_Pin, 1);
							break;

						case 5:
							HAL_GPIO_WritePin(PYRO5_GPIO_Port, PYRO5_Pin, 1);
							break;
						case 6:
							HAL_GPIO_WritePin(PYRO6_GPIO_Port, PYRO6_Pin, 1);
							break;
						case 7:
							HAL_GPIO_WritePin(PYRO7_GPIO_Port, PYRO7_Pin, 1);
							break;
						case 8:
							HAL_GPIO_WritePin(PYRO8_GPIO_Port, PYRO8_Pin, 1);
							break;
						default:
							break;
					}
		    	}else{
		    		LoRA_sendPacket("CANNOT FIRE, BOARD NOT ARMED");
		    	}
		    }
		}
    */

		//uint8_t data = read_EEPROM(1);
	    //sprintf( data_gyro,  "%d\n", DMA_data);

		//HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 16;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 2;
  hfdcan3.Init.NominalTimeSeg2 = 2;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 0;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 0;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 28;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 90;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 95;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|PYRO6_Pin|PYRO7_Pin|PYRO8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARM1_Pin|ARM2_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|PYRO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PYRO2_Pin|PYRO3_Pin|PYRO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, PYRO5_Pin|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PYRO6_Pin PYRO7_Pin PYRO8_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|PYRO6_Pin|PYRO7_Pin|PYRO8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARM1_Pin ARM2_Pin PA15 */
  GPIO_InitStruct.Pin = ARM1_Pin|ARM2_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PYRO1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|PYRO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CONT1_Pin */
  GPIO_InitStruct.Pin = CONT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PYRO2_Pin PYRO3_Pin PYRO4_Pin */
  GPIO_InitStruct.Pin = PYRO2_Pin|PYRO3_Pin|PYRO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT2_Pin CONT3_Pin */
  GPIO_InitStruct.Pin = CONT2_Pin|CONT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CONT4_Pin */
  GPIO_InitStruct.Pin = CONT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PYRO5_Pin PG2 PG3 */
  GPIO_InitStruct.Pin = PYRO5_Pin|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT5_Pin CONT6_Pin CONT7_Pin CONT8_Pin */
  GPIO_InitStruct.Pin = CONT5_Pin|CONT6_Pin|CONT7_Pin|CONT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Servo_ARM_CHECK_Pin */
  GPIO_InitStruct.Pin = Servo_ARM_CHECK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Servo_ARM_CHECK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	DMA_data = read_Data;
	char data_gyro[100];
    sprintf( data_gyro,  "a\n");
    CDC_Transmit_HS(data_gyro, strlen(data_gyro));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
