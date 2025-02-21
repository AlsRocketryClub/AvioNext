#pragma once

#define RECEIVING_RELIABLE 0
#define RECEIVING_STREAM 1
#define SENDING_RELIABLE 2
#define SENDING_STREAM 3
#define TRANSITIONING 4


struct ReliableSendConfig {
	int mode;
	int streamable_packets;
	char** messages;
	int messages_count;
};

void communicationHandler(
		void reliableReceiveHandle(char*),
		void streamReceiveHandle(char*),
		char* streamSendHandle(int),
		struct ReliableSendConfig reliableSendHandle(),
		int initial_communication_state
);
