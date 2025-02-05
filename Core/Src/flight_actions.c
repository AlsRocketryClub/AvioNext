#include <AvioNEXT.h>
#include <main.h>
#include <usbd_cdc_if.h>;

#define WAITING 0
#define ASCENT 1
#define DESCENT 2
#define MAIN_PARACHUTE 3
#define CANSAT_DEPLOYMENT 4

double last_altitudes[50];
int arr_index = 0;
int array_size = sizeof(last_altitudes) / sizeof(last_altitudes[0]);
double ascent_preset = 9;
double parachute_preset = 12;


double min(double altitudes[]) {
	double min = altitudes[0];
	for (int i = 0; i < array_size; i++) {
		if (altitudes[i] < min) {
			min = altitudes[i];
		}
	}
	return min;
}

void fillAltitude(double altitude) {
	last_altitudes[arr_index] = altitude;
	arr_index = (arr_index + 1) % array_size;
}

void flightActions(double altitude, int* state) {

	fillAltitude(altitude);
	double min_altitude = min(last_altitudes);
	//double min_altitude = 12;

	if (*state == WAITING) {
    	if (altitude > ascent_preset) {
        	*state = ASCENT;

    		char debug[50];
    		sprintf(debug, "ASCENT: %f, ASCENT_PRESET: %f\n", altitude, ascent_preset);
    		CDC_Transmit_HS(debug, strlen(debug));
    		HAL_Delay(20);

        	return;
    	}
	}
	else if (*state == ASCENT) {
    	if (altitude < min_altitude) {
    		*state = DESCENT;

    		char debug[50];
    		sprintf(debug, "DESCENT: %f, Min: %f\n", altitude, min_altitude);
    		CDC_Transmit_HS(debug, strlen(debug));
    		HAL_Delay(20);
    		return;
    	}
	}
	else if (*state == DESCENT) {
    	if (altitude < parachute_preset) {
    		*state = MAIN_PARACHUTE;

    		char debug[50];
    		sprintf(debug, "MAIN_PARACHUTE: %f, PARACHUTE_PRESET: %f\n", altitude, parachute_preset);
    		CDC_Transmit_HS(debug, strlen(debug));
    		HAL_Delay(20);
    		return;
    	}
    }
	else if (*state == MAIN_PARACHUTE) {
		// LOGIC FOR CANSAT DEPLOYMENT
	}
}

const float action_brightness = 0.4;

void flightActionsLedTest(int* state) {
	uint8_t LED_Color_Data_Staus[14][3];
	for (int i = 0; i < 14; i++) {
		switch (*state) {
		case WAITING: //RED
			LED_Color_Data_Staus[i][0] = 0;
			LED_Color_Data_Staus[i][1] = (int)(255 * action_brightness);
			LED_Color_Data_Staus[i][2] = 0;
			break;
		case ASCENT: //BLUE
			LED_Color_Data_Staus[i][0] = 0;
			LED_Color_Data_Staus[i][1] = 0;
			LED_Color_Data_Staus[i][2] = (int)(255 * action_brightness);
			break;
		case DESCENT: //GREEN
			LED_Color_Data_Staus[i][0] = (int)(255 * action_brightness);
			LED_Color_Data_Staus[i][1] = 0;
			LED_Color_Data_Staus[i][2] = 0;
			break;
		case MAIN_PARACHUTE: //ORANGE
			LED_Color_Data_Staus[i][0] = (int)(50 * action_brightness);
			LED_Color_Data_Staus[i][1] = (int)(255 * action_brightness);
			LED_Color_Data_Staus[i][2] = 0;
			break;
		default:
			break;
		}
	}
	setLEDs(LED_Color_Data_Staus);
}

