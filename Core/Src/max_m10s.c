/*

===============================================================================

@NOTE: This code is in heavy development, it is currently in a *NON* functional state.
                   When it will work a pull request will be created :).
                   Also it looks horrible I know, but it's not done yet.
@NOTE: Apparently max NMEA message length, including <CR><LF>, is 82 bytes. 
       Possibly reduce read buffer to save ram.

===============================================================================

@TODO: Cleanup development class code to remove all the useless stuff.
@TODO: Change this to a DMA based system for reduced cpu load to enable more
       focus on flight software.

===============================================================================

*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "max_m10s.h"


// @NOTE: Documentation: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
const char* msgsOff[] = {
    "$PSRF103,00,01,00,01*25\r\n",
    "$PSRF103,01,01,00,01*24\r\n",
    "$PSRF103,02,01,00,01*27\r\n",
    "$PSRF103,03,01,00,01*26\r\n",
    "$PSRF103,04,01,00,01*21\r\n",
    "$PSRF103,05,01,00,01*20\r\n",
    "$PSRF103,06,01,00,01*23\r\n",
    "$PSRF103,07,01,00,01*22\r\n",
    "$PSRF103,08,01,00,01*2D\r\n",
    NULL,
};

const char enRMCMsg[] = "$PSRF103,04,01,01,01*20\r\n";

NMEA_RMC grmc = {0};

bool bankSel = 0;
char rb[2][MAX_M10S_RING_BUFFER_SIZE/2] = {0};
uint8_t checksum[2] = {0};

bool packetInProgress = false;
bool packetComplete = false;
bool checksumInProgress = false;
bool invalidPacket[2] = {false};
int writeHead = 0;

uint8_t byte = 0;

bool MAX_M10s_msgsOff(I2C_HandleTypeDef* i2c) {
    size_t i = 0;
    bool ok = false;
    do {
        const char* msg = msgsOff[i];
        ok = HAL_OK == HAL_I2C_Master_Transmit(i2c, MAX_M10S_I2C_ADDR, (void*)msg, strlen(msg), 100000000);
        i++;
        if (!msgsOff[i]) break;
    } while(ok);
    return ok;
}

bool MAX_M10s_init(I2C_HandleTypeDef* i2c) {
    if (!MAX_M10s_check_if_exists(i2c)) return false; // @INFO: We don't see the GPS.
    
    if (MAX_M10s_msgsOff(i2c)) return false; // @INFO: We failed to send the messages.
    
    // @INFO: Enable the G*RMC NMEA message.
    return HAL_OK != HAL_I2C_Master_Transmit(i2c, MAX_M10S_I2C_ADDR, (void*)enRMCMsg, strlen(enRMCMsg), 100000000);
}

inline bool MAX_M10s_reset(I2C_HandleTypeDef* i2c) { return MAX_M10s_init(i2c); }

void MAX_M10S_parse() {
    if (packetComplete) {
        bankSel = !bankSel;
        packetComplete = false;
    }
    
    char* msg = rb[!bankSel];

    char status = 'V';
    
    for (int i = 0; i < strlen(msg); i++) {
    	if (msg[i]==',') msg[i]=' ';
    }

    sscanf(msg, "$%*s %s %c %s %c %s %c %f %f %s",
        grmc.UTCtime,
        &status,
        grmc.lat,
        &grmc.nsInd,
        grmc.lon,
        &grmc.ewInd,
        &grmc.sog,
        &grmc.cog,
        grmc.date);
        
    grmc.status = status=='A';
}

inline NMEA_RMC MAX_M10s_getRMC() { return grmc; }

// @NOTE: This checks if MAX M10s is on the I2C bus, and returs true if so.
//        It works on my(m1cha1s) dev board so...
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
    HAL_I2C_Mem_Read(i2c, MAX_M10S_I2C_ADDR, 0xFF, 1, &byte, 1, 10000);
    MAX_M10s_irq_handler(i2c);
}

static uint8_t hexToBin(char hex) {
    if (hex >= 0x30 && hex < 0x3A) return hex - 0x30;
    if (hex >= 0x40 && hex < 0x47) return hex - 55;
    return 0;
}

void MAX_M10s_irq_handler(I2C_HandleTypeDef* i2c) {
    //HAL_I2C_Master_Receive(i2c, MAX_M10S_I2C_ADDR, &byte, 1, 10); // @FIXME: Timeout set at random.
    
    if (!(packetInProgress || packetComplete)) {
        if (byte == '$') {
            packetInProgress = true;
            writeHead = 0;
            checksum[bankSel] = 0;
            invalidPacket[bankSel] = false;
            goto saveByte;
        } else {
            // @NOTE: The data is *garbage*, wait for packet start.
            writeHead = 0; // @NOTE: Probably redundant;
            return;
        }
    }
    
    if (byte == '$') {
        //invalidPacket[bankSel] = true;
        return; // @NOTE: Skip redundant '$', if this doesn't work right, uncoment the line above.
    }
    
    if (packetComplete) {
        return; // @NOTE: Discard byte... The previous message has not been parsed yet...
    }
    
    if (writeHead && byte!='$' && rb[bankSel][writeHead-1]=='$') { // @NOTE: We have the start of a checksum region.
        checksumInProgress = true;
    }
    
    if (byte=='*') checksumInProgress = false;
    
    if (writeHead && byte=='\n' && rb[bankSel][writeHead-1]=='\r') { // @NOTE: We have the end of a message.
        packetComplete = true;
        packetInProgress = false;
        
        // @NOTE:Validate the checksum.
        
        uint8_t a = hexToBin(rb[bankSel][writeHead-3]);
        uint8_t b = hexToBin(rb[bankSel][writeHead-2]);
        
        uint8_t msgChk = (a<<4)|b;
        
        if (msgChk != checksum[bankSel]) {
            invalidPacket[bankSel] = true;
        }
    }
    
saveByte:
    rb[bankSel][writeHead++] = byte;
    if (checksumInProgress) checksum[bankSel] ^= byte;
}
