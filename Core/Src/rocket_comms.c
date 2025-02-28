#include "communication_protocol.h"
#include "LoRa.h"
#include "StatusDisplay.h"

//to do: arm disarm and the state machine in sending reliable should be separated into new file
#include "main.h"
#include "fatfs.h"
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern FATFS FatFs;
extern FIL Fil;
extern FRESULT FR_Status;

int disarm(char *state) {
	HAL_GPIO_WritePin(ARM1_GPIO_Port, ARM1_Pin, 0);
	//HAL_GPIO_WritePin(ARM2_GPIO_Port, ARM2_Pin, 0);

	HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, 0);
	HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, 0);
	HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, 0);
	HAL_GPIO_WritePin(PYRO4_GPIO_Port, PYRO4_Pin, 0);

	HAL_GPIO_WritePin(PYRO5_GPIO_Port, PYRO5_Pin, 0);
	HAL_GPIO_WritePin(PYRO6_GPIO_Port, PYRO6_Pin, 0);
	HAL_GPIO_WritePin(PYRO7_GPIO_Port, PYRO7_Pin, 0);
	HAL_GPIO_WritePin(PYRO8_GPIO_Port, PYRO8_Pin, 0);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	setStatus("ARM", 2);
	strcpy(state, "DISARMED");
	return 0;
}

int arm(char *state) {
	HAL_GPIO_WritePin(ARM1_GPIO_Port, ARM1_Pin, 1);
//HAL_GPIO_WritePin(ARM2_GPIO_Port, ARM2_Pin, 1);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	setServo(1, 100);

	strcpy(state, "ARMED");

	setStatus("ARM", 0);
	return 0;
}

void pyro_check_continuity(uint8_t CONTS[]) {
	CONTS[0] = HAL_GPIO_ReadPin(CONT1_GPIO_Port, CONT1_Pin);
	CONTS[1] = HAL_GPIO_ReadPin(CONT2_GPIO_Port, CONT2_Pin);
	CONTS[2] = HAL_GPIO_ReadPin(CONT3_GPIO_Port, CONT3_Pin);
	CONTS[3] = HAL_GPIO_ReadPin(CONT4_GPIO_Port, CONT4_Pin);
	CONTS[4] = HAL_GPIO_ReadPin(CONT5_GPIO_Port, CONT5_Pin);
	CONTS[5] = HAL_GPIO_ReadPin(CONT6_GPIO_Port, CONT6_Pin);
	CONTS[6] = HAL_GPIO_ReadPin(CONT7_GPIO_Port, CONT7_Pin);
	CONTS[7] = HAL_GPIO_ReadPin(CONT8_GPIO_Port, CONT8_Pin);
}

char state[MAX_PKT_LENGTH] = "DISARMED";
char continuities[8][50];
char status[50];
char command[MAX_PKT_LENGTH];
char streamed_data[100];
char state_message[100];
//length is an arbitrary number, but it's unlikely to have more than 200
char* rocket_messages[200];

void rocketReliableReceiveHandle(char* received_packet) {
	strcpy(command, received_packet);
}

void rocketStreamReceiveHandle(char* received_packet) {
	CDC_Transmit_HS(received_packet, strlen(received_packet));
}

char* rocketStreamSendHandle(int remainingPacketCount) {
	if (strcmp(state, "ARMED") == 0) {
		if (strcmp(command, "FIRE") == 0) {
			HAL_ADC_Start(&hadc1); // start the adc
			HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion

			uint16_t adc_val = HAL_ADC_GetValue(&hadc1); // get the adc value
			sprintf(streamed_data, "%d, %d\n", HAL_GetTick(), adc_val);
			FR_Status = f_open(&Fil, "MyTextFile.txt",
					FA_OPEN_APPEND | FA_WRITE);
			f_puts(streamed_data, &Fil);
			f_close(&Fil);
			HAL_ADC_Stop(&hadc1); // stop adc
			return streamed_data;
		}
	}
	return "";
}

struct ReliableSendConfig rocketReliableSendHandle() {
	struct ReliableSendConfig config;
	int message_count=0;
	//reserve first place for the state message
	message_count++;
	if (strcmp(state, "DISARMED") == 0) {
		if (strcmp(command, "ARM") == 0) {

			if (!arm(state)) {
				strcpy(status, "ARM SUCCESS");
 				rocket_messages[message_count] = status;
				message_count++;
			} else {
				strcpy(status, "ARM UNSUCCESSFUL");
 				rocket_messages[message_count] = status;
				message_count++;
			}
		} else if (strcmp(command, "DISARM") == 0) {
			strcpy(status, "ALREADY DISARMED");
			rocket_messages[message_count] = status;
			message_count++;
		} else if (strcmp(command, "CONT") == 0) {
			uint8_t CONTS[8];
			pyro_check_continuity(CONTS);


			for (int i = 0; i < 8; i++) {
				if (CONTS[i]) {
					sprintf(continuities[i], "PYRO %d DOESN'T HAVE CONTINUITY",
							i + 1);
				} else {
					sprintf(continuities[i], "PYRO %d HAS CONTINUITY", i + 1);
				}
				rocket_messages[message_count] = continuities[i];
				message_count++;
			}
		}
	} else if (strcmp(state, "ARMED") == 0) {
		if (strcmp(command, "DISARM") == 0) {
			if (!disarm(state)) {
				strcpy(status, "DISARM SUCCESS");
				rocket_messages[message_count] = status;
				message_count++;
			} else {
				strcpy(status, "DISARM UNSUCCESS");
				rocket_messages[message_count] = status;
				message_count++;
			}
		} else if (strcmp(command, "ARM") == 0) {
			strcpy(status, "ALREADY ARMED");
			rocket_messages[message_count] = status;
			message_count++;
		} else if (strcmp(command, "FIRE") == 0) {

		}

	} else {
		strcpy(status, "state wrong!");
		rocket_messages[message_count] = status;
		message_count++;
	}

	//first message is state
	sprintf(state_message, "Current state: %s\n", state);
	rocket_messages[0] = state_message;

	config.messages_count = message_count;
	config.messages = rocket_messages;
	config.mode = RECEIVING_RELIABLE;
    return config;
}
