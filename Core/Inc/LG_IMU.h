#include <main.h>

uint8_t LG_Read_Register(uint8_t addr);

void LG_Write_Register(uint8_t addr, uint8_t data);

uint8_t LG_Check();
void LG_Configure();

float LG_Get_Gyro_X();
float LG_Get_Gyro_Y();
float LG_Get_Gyro_Z();

float LG_Get_Acc_X();
float LG_Get_Acc_Y();
float LG_Get_Acc_Z();
