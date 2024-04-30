/*
 * max_m10s.h
 *
 *  Created on: Apr 30, 2024
 *      Author: msule
 */

#ifndef INC_MAX_M10S_H_
#define INC_MAX_M10S_H_

#include "main.h"

#define MAX_M10S_I2C_ADDR 0x42
#define MAX_M10S_RING_BUFFER_SIZE 1024

// FIXME(m1cha1s): Uncoment this before launch!!!
// #define GPS_CHECKSUM_EN

typedef struct {
    unsigned char h;
    unsigned char m;
    float s;
} GPS_time_t;

typedef struct {
    unsigned char day;
    unsigned char month;
    unsigned char year;
} GPS_date_t;

typedef enum {
    V = 0, // Data invalid
    A = 1, // Data valuid
} GPRMC_status_t;

typedef enum { N, S } GPRMC_NS_t;
typedef enum { E, W } GPRMC_EW_t;

typedef struct {
    GPS_time_t utc_time;
    GPRMC_status_t status;
    double lat;
    GPRMC_NS_t ns_ind;
    double lon;
    GPRMC_EW_t ew_ind;
    double og_speed; // In knots(why???)
    double og_course; // Course in degrees
    GPS_date_t date;
    // TODO(m1cha1s): There is also magentic variation 
} GPRMC_t;

int MAX_M10S_init(I2C_HandleTypeDef* i2c);
int MAX_M10S_deinit(I2C_HandleTypeDef* i2c);
int MAX_M10S_reset(I2C_HandleTypeDef* i2c);
GPRMC_t MAX_M10S_read(I2C_HandleTypeDef* i2c);

#endif /* INC_MAX_M10S_H_ */
