#include <stdint.h>

#define MAX_PKT_LENGTH 255

uint8_t LoRA_Read_Register(uint8_t addr);

void LoRA_Write_Register(uint8_t addr, uint8_t data);

void LoRA_sleep(void);

void LoRA_set_frequency(long frequency);

void LoRA_idle();

void LoRA_setOCP(uint8_t mA);
void LoRA_setTxPower(int level);

void LoRA_explicit_header_mode();

void LoRA_begin(long frequency);


void LoRA_beginPacket();

void LoRA_endPacket();


int LoRA_parsePacket();

void LoRA_sendPacket(char * data);

int recv_packet(char* LoRA_data, int max_length);

void reliable_send_packet(char *LoRA_data);
