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


