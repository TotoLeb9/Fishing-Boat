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

#define PAYLOAD_GPS_SIZE 256

typedef struct{
	uint16_t azimuth;
	float longitude;
	float latitude;
	float groundSpeed;
}GPS_Struct;

extern UART_HandleTypeDef huart4;
extern GPS_Struct gpsStructData;
extern uint8_t gps_data[PAYLOAD_GPS_SIZE];

void ParseGPS_RMC(char *gpsData);
#endif /* INC_GPS_H_ */
