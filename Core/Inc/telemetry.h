/*
 * telemetry.h
 *
 *  Created on: Mar 4, 2026
 *      Author: totoleb
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

typedef struct __attribute__((packed))
{
	uint8_t header;
	uint16_t cap;
	uint8_t vitesse;
	uint8_t batterie;
	int32_t  latitude;    // lat * 1e6
	int32_t  longitude;
	uint16_t conso_live;
	uint16_t distance;
}Telemetry;


#endif /* INC_TELEMETRY_H_ */
