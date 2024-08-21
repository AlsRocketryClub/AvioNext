#include <AvioNEXT.h>
#include <main.h>
uint8_t LED_Color_Data_Staus[14][3];

const float brightness = 0.4;

void setStatus(char *ModuleName, uint8_t status) {

	if (strcmp(ModuleName, "CAN") == 0) {
		statusArray[0] = status;

	} else if (strcmp(ModuleName, "GPS") == 0) {
		statusArray[1] = status;

	} else if (strcmp(ModuleName, "LoRA") == 0) {
		statusArray[2] = status;

	} else if (strcmp(ModuleName, "SD CARD") == 0) {
		statusArray[3] = status;

	} else if (strcmp(ModuleName, "HG 1") == 0) {
		statusArray[4] = status;

	} else if (strcmp(ModuleName, "LG 1") == 0) {
		statusArray[5] = status;

	} else if (strcmp(ModuleName, "BAR 1") == 0) {
		statusArray[6] = status;

	} else if (strcmp(ModuleName, "ARM") == 0) {
		statusArray[7] = status;

	} else if (strcmp(ModuleName, "HG2") == 0) {
		statusArray[8] = status;

	} else if (strcmp(ModuleName, "LG2") == 0) {
		statusArray[9] = status;

	} else if (strcmp(ModuleName, "BAR 2") == 0) {
		statusArray[10] = status;

	} else if (strcmp(ModuleName, "REG 1") == 0) {
		statusArray[11] = status;

	} else if (strcmp(ModuleName, "REG 2") == 0) {
		statusArray[12] = status;

	} else if (strcmp(ModuleName, "BATT") == 0) {
		statusArray[13] = status;

	}

}

void updateStatus() {
	uint8_t statusArray[14];
	for (int i = 0; i < 14; i++) {
		switch (statusArray[i]) {
		case 0: //RED LED, critical module malfunction
			LED_Color_Data_Staus[i][0] = 0;
			LED_Color_Data_Staus[i][1] = (int)(255 * brightness);
			LED_Color_Data_Staus[i][2] = 0;
			break;
		case 1: //orange blinking LED, module awaiting connection
			if ((HAL_GetTick() / 1000) % 2) {
				LED_Color_Data_Staus[i][0] = (int)(50 * brightness);
				LED_Color_Data_Staus[i][1] = (int)(255 * brightness);
				LED_Color_Data_Staus[i][2] = 0;
			} else {
				LED_Color_Data_Staus[i][0] = 0;
				LED_Color_Data_Staus[i][1] = 0;
				LED_Color_Data_Staus[i][2] = 0;
			}
			break;
		case 2: //GREEN LED, module nominal
			LED_Color_Data_Staus[i][0] = (int)(255 * brightness);
			LED_Color_Data_Staus[i][1] = 0;
			LED_Color_Data_Staus[i][2] = 0;
		default:
			break;
		}
	}
	setLEDs(LED_Color_Data_Staus);
}
