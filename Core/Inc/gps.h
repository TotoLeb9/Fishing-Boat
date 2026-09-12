/*
 * gps.h
 *
 *  Created on: Jan 18, 2026
 *      Author: totoleb
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "log.h"
#include <math.h>
#include "FreeRTOS.h"
#include <stdbool.h>

#define PAYLOAD_GPS_SIZE 256
#define EARTH_RADIUS_KM 6371.0f
#define DEG_TO_RAD      0.0174532925f
#define RAD_TO_DEG      57.2958f
#define MAX_POINT 50
#define GPS_CALIB_SAMPLES       20      /* 20 × 200ms = ~4 secondes   */
#define GPS_CALIB_HDOP_MAX      1.2f    /* plus strict que navigation  */
#define GPS_CALIB_SAT_MIN       6       /* minimum 6 sats              */
#define GPS_CALIB_FIX_MIN       2       /* EGNOS obligatoire           */
#define GPS_ALPHA 0.3
extern DMA_HandleTypeDef hdma_uart4_rx;

typedef struct {
    float       latitude;
    float       longitude;
    float       groundSpeed;
    float       azimuth;
    float       hdop;
    uint8_t     satellites;
    uint8_t     fixQuality;
    TickType_t  timestamp;
    uint8_t     isValid;
} GPS_Struct;

typedef struct {
    float lat_filtered;
    float lon_filtered;
} GPS_Filter;

typedef enum {
    GPS_CALIB_IDLE    = 0,
    GPS_CALIB_RUNNING = 1,
    GPS_CALIB_DONE    = 2,
} GPS_CalibState;

typedef struct {
    double         lat_acc;
    double         lon_acc;
    uint8_t        count;
    uint8_t        rejected;
    GPS_CalibState state;
} GPS_Calibration;

extern GPS_Calibration gpsCalib;
extern GPS_Filter gpsFilter;

typedef struct{
	float latitude;
	float longitude;
}GPS_Pos;

extern UART_HandleTypeDef huart4;
extern GPS_Struct gpsStructData;
extern uint8_t gps_data[PAYLOAD_GPS_SIZE];
extern GPS_Pos gpsPoint[MAX_POINT];

float GetDistanceHaversine(float lat_depart, float lat_arrive, float lon_depart, float lon_arrive);
float GetBearing(float latA, float lonA, float latB, float lonB);
void ParseGPS_RMC(char *gpsData);
uint8_t SetDataGps(const char* cmd);
uint8_t GPS_Checksum(const char* cmd);
void InitGpsValues(void);
//void ParseGPS_GLL(char* sentence);
void ParseGPS_GGA(char *gpsData);
uint8_t GPS_UpdateHomeCalib(void);
void SetHomeAccurate(void);
void GPS_ApplyFilter(void);
static inline float MilesToKm(float miles){
	return miles*1.852;
}


#endif /* INC_GPS_H_ */
