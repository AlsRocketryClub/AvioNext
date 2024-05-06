/*

@WARNING(m1cha1s): This code is in heavy development, it is currently in a *NON* functional state.
                   When it will work a pull request will be created :).
                   Also it looks horrible I know, but it's not done yet.

*/

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "max_m10s.h"

// Documentation: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
const char initMsg[] = "$PSRF103,00,01,00,01*25\r\n";

uint8_t* payloadCfg = NULL;

int MAX_M10s_init(I2C_HandleTypeDef* i2c) {
    if (!MAX_M10s_check_if_exists(i2c)) return -1; // @INFO(m1cha1s): We don't see the GPS.
    HAL_I2C_Master_Transmit(i2c, MAX_M10S_I2C_ADDR, (void*)initMsg, sizeof(initMsg), 100000000);
    return 0;
}

int MAX_M10s_deinit(I2C_HandleTypeDef* i2c) {}

int MAX_M10s_reset(I2C_HandleTypeDef* i2c) {}

bool bankSel = 0;
char rb[2][MAX_M10S_RING_BUFFER_SIZE/2] = {0};
uint16_t checksum[2] = {0};

bool packetInProgress = false;
bool packetComplete = false;
bool checksumInProgress = false;
bool invalidPacket[2] = {false};
int writeHead = 0;

uint8_t byte = 0;

GPRMC_t MAX_M10S_read(I2C_HandleTypeDef* i2c) {
    HAL_I2C_Master_Receive(i2c, MAX_M10S_I2C_ADDR, rb[bankSel], 65, 10000); // @TODO(m1cha1s): Change to a DMA or Interrupt.
    return (GPRMC_t){0};
}

// This checks if MAX M10s is on the I2C bus, and returs true if so.
// It works on my(m1cha1s) dev board so...
bool MAX_M10s_check_if_exists(I2C_HandleTypeDef* i2c) {
    return HAL_I2C_IsDeviceReady (i2c, MAX_M10S_I2C_ADDR, 10, 10000) == HAL_OK;
}

int MAX_M10s_bytesToRead(I2C_HandleTypeDef* i2c) {
    uint16_t btr = 0;
    
    uint16_t high = 0;
    uint16_t low = 0;
    if (HAL_I2C_Mem_Read(i2c, MAX_M10S_I2C_ADDR, 0xFD, 1, &high, 1, 1000000) != HAL_OK) return -1;
    if (HAL_I2C_Mem_Read(i2c, MAX_M10S_I2C_ADDR, 0xFE, 1, &low, 1, 1000000)  != HAL_OK) return -1;
    
    btr = (high << 8) | (low & 0x00FF);
    
    return (int)btr;
}

void MAX_M10s_poll(I2C_HandleTypeDef* i2c) {
    HAL_I2C_Master_Receive(i2c, MAX_M10S_I2C_ADDR, &byte, 1, 10000);
    MAX_M10s_irq_handler(i2c);
}

void MAX_M10s_irq_handler(I2C_HandleTypeDef* i2c) {
    //HAL_I2C_Master_Receive(i2c, MAX_M10S_I2C_ADDR, &byte, 1, 10); // @FIXME(m1cha1s): Timeout set at random.
    
    if (!(packetInProgress || packetComplete)) {
        if (byte == '$') {
            packetInProgress = true;
            writeHead = 0;
            checksum[bankSel] = 0;
            invalidPacket[bankSel] = false;
            goto saveByte;
        } else {
            // @INFO(m1cha1s): The data is *garbage*, wait for packet start.
            writeHead = 0; // @TODO(m1cha1s): Probably redundant;
            return;
        }
    }
    
    if (packetComplete) {
        return; // Discard byte... The previous message has not been parsed yet...
    }
    
    if (writeHead && byte!='$' && rb[bankSel][writeHead-1]=='$') { // We have the start of a checksum region.
        checksumInProgress = true;
    }
    
    if (byte=='*') checksumInProgress = false;
    
    if (writeHead && byte=='\n' && rb[bankSel][writeHead-1]=='\r') { // We have the end of a message.
        packetComplete = true;
        packetInProgress = false;
        
        // Validate the checksum.
        // ___=
        // ^^
        
        if (*(uint16_t*)(&rb[bankSel][writeHead-3]) != checksum) {
            invalidPacket[bankSel] = true;
        }
    }
    
saveByte:
    rb[bankSel][writeHead++] = byte;
    if (checksumInProgress) checksum[bankSel] ^= byte;
}