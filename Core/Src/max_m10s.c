#include <string.h>

#include "max_m10s.h"

// Documentation: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
const char* initMsg = "$PSRF103,04,00,01,00\r\n";

int MAX_M10S_init(I2C_HandleTypeDef* i2c) {
    HAL_I2C_Master_Transmit(i2c, MAX_M10S_I2C_ADDR, initMsg, strlen(initMsg), 1000); // TODO(m1cha1s): This timeout is arbitrary fix if needed!
    return 0;
}

int MAX_M10S_deinit(I2C_HandleTypeDef* i2c) {
    // FIXME(m1cha1s): Implement it
    return -1;
}

int MAX_M10S_reset(I2C_HandleTypeDef* i2c) {
    // FIXME(m1cha1s): Implement it
    return -1;
}

char bankSel = 0;
char rb[2][MAX_M10S_RING_BUFFER_SIZE/2] = {0};

GPRMC_t MAX_M10S_read(I2C_HandleTypeDef* i2c) {
    HAL_I2C_Master_Receive(i2c, MAX_M10S_I2C_ADDR, rb[bankSel], 65, 10000); // TODO(m1cha1s): Change to a DMA or Interrupt
    return (GPRMC_t){0};
}
