#include <main.h>

uint8_t HG2_Read_Register(uint8_t addr);

void HG2_Write_Register(uint8_t addr, uint8_t data);

void HG2_Get_Acc(int16_t* data);

void setServo(int servoNum, float angle);

void setLEDs(uint8_t LED_Color_Data[14][3]);

double triangle_space(double x);

int write_EEPROM(uint32_t address, uint8_t data);

uint8_t read_EEPROM(uint32_t address);

int disarm(char *state);

int arm(char *state);
