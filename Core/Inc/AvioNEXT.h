#include <main.h>

uint8_t HG2_Read_Register(uint8_t addr);

void HG2_Write_Register(uint8_t addr, uint8_t data);

void HG2_Get_Acc(int16_t* data);


uint8_t LG2_Read_Register(uint8_t addr);

void LG2_Write_Register(uint8_t addr, uint8_t data);

float LG2_Get_Gyro_X();
float LG2_Get_Gyro_Y();
float LG2_Get_Gyro_Z();

float LG2_Get_Acc_X();
float LG2_Get_Acc_Y();
float LG2_Get_Acc_Z();

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

int recv_packet(char *LoRA_data, int max_length);

void reliable_send_packet(char *LoRA_data);

void setServo(int servoNum, float angle);

void setLEDs(uint8_t LED_Color_Data[14][3]);

double triangle_space(double x);

int write_EEPROM(uint32_t address, uint8_t data);

uint8_t read_EEPROM(uint32_t address);

int disarm(char *state);

int arm(char *state);

void multiplyQuat(double r[4], double s[4]);
