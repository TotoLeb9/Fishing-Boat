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
void ParseGPS_GLL(char* sentence);

static inline float MilesToKm(float miles){
	return miles*1.852;
}


#endif /* INC_GPS_H_ */
