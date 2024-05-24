// @INFO: A simple program for calculating NMEA messages checksum.

#include <stdio.h>
#include <string.h>
#include <string.h>

#include "../Core/Inc/NMEAchecksum.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: checksum <nmea message>\n");
        return 1;
    }
    
    uint8_t cs = checksum(argv[1]);
    
    printf("%X\n", cs);
    
    return 0;
}