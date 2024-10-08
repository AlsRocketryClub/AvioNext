#include "communication_protocol.h"

void groundstationReliableReceiveHandle(char* received_packet);
void groundstationStreamReceiveHandle(char* received_packet);
char* groundstationStreamSendHandle(int remainingPacketCount);

struct ReliableSendConfig groundstationReliableSendHandle();
