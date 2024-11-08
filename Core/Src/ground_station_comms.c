#include "communication_protocol.h"
#include "usb_device.h"

//to do: move usb
#define usbBufferLen 256
uint8_t usbDataBuffer[usbBufferLen];
uint32_t usbBytesReady = 0;
int usbReceiveHandle(char* output){
	uint32_t temp = usbBytesReady;

	if(temp > 0){
		if(temp > 256){
			//crash(2);
		}
		memcpy(output, usbDataBuffer, temp);
		output[temp] = '\0';
		usbBytesReady = 0;
	}
	return temp;
}


void groundstationReliableReceiveHandle(char* received_packet) {
	CDC_Transmit_HS(received_packet, strlen(received_packet));
}

void groundstationStreamReceiveHandle(char* received_packet) {
	CDC_Transmit_HS(received_packet, strlen(received_packet));
}

char* groundstationStreamSendHandle(int remainingPacketCount) {
	return "Ground station shouldn't be streaming!\n";
}

char input[usbBufferLen];
char* groundstation_messages[1];

struct ReliableSendConfig groundstationReliableSendHandle() {
	struct ReliableSendConfig config;

    //get input
    while(!usbReceiveHandle(input))
    {}

    groundstation_messages[0]=input;
    config.messages = groundstation_messages;
    config.messages_count = 1;
    config.streamable_packets = 0;
    //reliable_send_packet(input);

    char debug[usbBufferLen+10];
    sprintf(debug, "%s\n", input);
    CDC_Transmit_HS(debug, strlen(debug));

    if(strcmp(input,"FIRE")==0)
    {
      config.mode = RECEIVING_STREAM;
      config.streamable_packets = 50;
    }
    else
    {
      config.mode = RECEIVING_RELIABLE;
    }
    return config;
}
