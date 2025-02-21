#include "communication_protocol.h"

void rocketReliableReceiveHandle(char* received_packet);
void rocketStreamReceiveHandle(char* received_packet);
char* rocketStreamSendHandle(int remainingPacketCount);

struct ReliableSendConfig rocketReliableSendHandle();
