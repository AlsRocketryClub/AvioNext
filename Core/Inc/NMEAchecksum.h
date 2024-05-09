#ifndef _NMEAchecksum_h
#define _NMEAchecksum_h

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

uint8_t checksum(char* message) {
    uint8_t cs = 0;
    bool incs  = false;
    
    for (size_t i = 0; i < strlen(message); i++) {
        switch (message[i]) {
        case '$': incs = true; break;
        case '*': incs = false; break;
        default: {
            if (!incs) continue;
            cs ^= message[i];
        } break;
        }
    }
    
    return cs;
}

#endif