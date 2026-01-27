/*
 * gps.c
 *
 *  Created on: Jan 18, 2026
 *      Author: totoleb
 */

#include "gps.h"

GPS_Struct gpsStructData;
GPS_Pos pos_depart;

uint8_t gps_data[PAYLOAD_GPS_SIZE];


float ConvertToDecimal(float nmeaCoord) {
    int degrees = (int)(nmeaCoord / 100);
    float minutes = nmeaCoord - (degrees * 100);
    return degrees + (minutes / 60.0f);
}

float GetDistanceHaversine(float lat_depart, float lat_arrive, float lon_depart, float lon_arrive) {
    float lat1 = lat_depart * DEG_TO_RAD;
    float lat2 = lat_arrive * DEG_TO_RAD;
    float dLat = (lat_arrive - lat_depart) * DEG_TO_RAD;
    float dLon = (lon_arrive - lon_depart) * DEG_TO_RAD;
    float a = sinf(dLat / 2) * sinf(dLat / 2) +
              cosf(lat1) * cosf(lat2) *
              sinf(dLon / 2) * sinf(dLon / 2);

    float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
    return EARTH_RADIUS_KM * c * 1000.0f;
}

float GetBearing(float latA, float lonA, float latB, float lonB) {
    float phi1 = latA * DEG_TO_RAD;
    float phi2 = latB * DEG_TO_RAD;
    float deltaLambda = (lonB - lonA) * DEG_TO_RAD;
    float y = sinf(deltaLambda) * cosf(phi2);
    float x = cosf(phi1) * sinf(phi2) -
              sinf(phi1) * cosf(phi2) * cosf(deltaLambda);
    float theta = atan2f(y, x);
    float bearing = theta * RAD_TO_DEG;
    bearing = fmodf((bearing + 360.0f), 360.0f);
    return bearing;
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
                    gpsStructData.azimuth = (atof(field) + 0.5f);
                }
                break;
        }

        counter++;
        if (*p == ',') p++;

    }
    LOG_INFO("SPEED = %f , LATTITUDE = %f, LONGITUDE = %f, AZIMUTH = %u =\n\r", gpsStructData.groundSpeed, gpsStructData.latitude, gpsStructData.longitude, gpsStructData.azimuth);
}

