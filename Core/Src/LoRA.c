//probably would be cleaner to not include everything everywhere
#include "main.h"
#include "LoRA.h"
#include <stdio.h>
#include <string.h>
#include "random.h"

extern SPI_HandleTypeDef hspi3;



// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_CAD_DETECTED_MASK      0x01

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

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
	LoRA_idle();

	char debug[MAX_PKT_LENGTH];
	if(recv_packet(debug, MAX_PKT_LENGTH)) {
		HAL_Delay(100);
		strcat(debug, " was thrown away");
		CDC_Transmit_HS(debug, strlen(debug));
	}



	int irqFlags = LoRA_Read_Register(REG_IRQ_FLAGS);
	/*char debug[250];
	sprintf(debug, "here: %d\n", (irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK));
	CDC_Transmit_HS(debug, strlen(debug));
	HAL_Delay(100);*/
	if(!((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0))
	{
		//CDC_Transmit_HS("here1\n", strlen("here1\n"));
		LoRA_beginPacket();
    	for(int i = 0; i < strlen(data); i++){
    		LoRA_Write_Register(REG_FIFO, data[i]);
    	}
    	LoRA_Write_Register(REG_PAYLOAD_LENGTH, strlen(data));
    	LoRA_endPacket();
    	/*char sent[300];
    	sprintf(sent, "\nsent: %s\n", data);
    	HAL_Delay(100);
    	CDC_Transmit_HS(sent, strlen(sent));*/
	}
	else {
		//CDC_Transmit_HS("here2\n", strlen("here2\n"));
		LoRA_Write_Register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	}
	/*char sent[300];
	sprintf(sent, "\nsent: %s\n", data);
	HAL_Delay(100);
	CDC_Transmit_HS(sent, strlen(sent));*/
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

    /*char rec[300];
    sprintf(rec, "received: %s\n", LoRA_data);
    CDC_Transmit_HS(rec, strlen(rec));*/
    return packet_length;
  }
  else{
    return 0;
  }
}

void reliable_send_packet(char *LoRA_data) {
	//CDC_Transmit_HS("debug\n", strlen("debug\n"));
	uint16_t length = strlen(LoRA_data) + 1; //+1 for the \0
	char acknowledge[length];
	uint32_t lastTime = HAL_GetTick();
	uint32_t wait_time = rand_range(3, 13)*100;
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

		if (HAL_GetTick() - lastTime > wait_time) {
	    	//CDC_Transmit_HS("debug\n", strlen("debug\n"));
			wait_time = rand_range(3, 13)*100;
			LoRA_sendPacket(LoRA_data);
			lastTime = HAL_GetTick();
		}
	}
}
