/*
 * gps.c
 *
 *  Created on: Jan 18, 2026
 *      Author: totoleb
 */

#include "gps.h"

GPS_Struct gpsStructData;
uint8_t gps_data[PAYLOAD_GPS_SIZE];


float ConvertToDecimal(float nmeaCoord) {
    int degrees = (int)(nmeaCoord / 100);
    float minutes = nmeaCoord - (degrees * 100);
    return degrees + (minutes / 60.0f);
}

void ParseGPS_RMC(char *gpsData) {
    char *p = strstr(gpsData, "RMC");
    if (p == NULL) return;
    while (*p && *p != ',') p++;
    if (*p == ',') p++;
    char field[32];
    int counter = 1;
    int i = 0;
    float temp_lat = 0.0f;
    float temp_lon = 0.0f;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        i = 0;
        while (*p && *p != ',' && *p != '*') {
            if (i < 30) field[i++] = *p;
            p++;
        }
        field[i] = '\0';

        switch(counter) {
            case 2:
                if (field[0] != 'A') {

                }
                break;

            case 3:
                temp_lat = ConvertToDecimal(atof(field));
                gpsStructData.latitude = temp_lat;
                break;

            case 4:
                if (field[0] == 'S') gpsStructData.latitude = -temp_lat;
                break;

            case 5:
                temp_lon = ConvertToDecimal(atof(field));
                gpsStructData.longitude = temp_lon;
                break;

            case 6:
                if (field[0] == 'W') gpsStructData.longitude = -temp_lon;
                break;

            case 7:
                gpsStructData.groundSpeed = atof(field) * 1.852f;
                break;

            case 8:
                if (field[0] != '\0') {
                    gpsStructData.azimuth = (uint16_t)(atof(field) + 0.5f);
                }
                break;
        }

        counter++;
        if (*p == ',') p++;

    }
    LOG_INFO("SPEED = %f , LATTITUDE = %f, LONGITUDE = %f, AZIMUTH = %u =\n\r", gpsStructData.groundSpeed, gpsStructData.latitude, gpsStructData.longitude, gpsStructData.azimuth);
}

