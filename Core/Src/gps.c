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
const uint8_t cmd_baud[] = "$PUBX,41,1,0007,0003,115200,0*18\r\n";
static const uint8_t ubx_rate_5hz[] = {
        0x06, 0x08, 0x06, 0x00,
        0xC8, 0x00,
        0x01, 0x00,
        0x00, 0x00
    };
static const uint8_t ubx_nav5_sea[] = {
        0x06, 0x24, 0x24, 0x00,
        0xFF, 0xFF,             /* mask : tous les champs           */
        0x05,                   /* dynModel = Sea                   */
        0x03,                   /* fixMode  = Auto 2D/3D            */
        0x00, 0x00, 0x00, 0x00, /* fixedAlt (non utilisé)           */
        0x10, 0x27, 0x00, 0x00, /* fixedAltVar                      */
        0x05,                   /* minElev  = 5°                    */
        0x00,                   /* drLimit                          */
        0xFA, 0x00,             /* pDop                             */
        0xFA, 0x00,             /* tDop                             */
        0x64, 0x00,             /* pAcc                             */
        0x2C, 0x01,             /* tAcc                             */
        0x14,                   /* staticHoldThresh  = 20 cm/s      */
        0x3C,                   /* dgnssTimeout      = 60 s         */
        0x00, 0x00,             /* reserved                         */
        0x14, 0x00,             /* staticHoldMaxDist = 20 m         */
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00
    };
static const uint8_t ubx_sbas[] = {
        0x06, 0x16, 0x08, 0x00,
        0x01,                   /* mode    : enable                 */
        0x07,                   /* usage   : range+corr+integrity   */
        0x03,                   /* maxSBAS = 3                      */
        0x00,                   /* reserved                         */
        0x00, 0x00, 0x00, 0x00  /* scanmode : auto                  */
    };
static const uint8_t ubx_nav_sbas_poll[] = {
        0xB5, 0x62,             /* sync                             */
        0x01, 0x32,             /* class NAV, id SBAS               */
        0x00, 0x00,             /* payload length = 0 (poll)        */
        0x33, 0x45              /* CK_A, CK_B                       */
    };
/**
 * @brief  Convertit une coordonnée NMEA (DDDMM.MMMM) en degrés décimaux.
 *         Ex : 4807.038 → 48.1173°
 */
float ConvertToDecimal(float nmeaCoord) {
    int degrees = (int)(nmeaCoord / 100);
    float minutes = nmeaCoord - (degrees * 100);
    return degrees + (minutes / 60.0f);
}


/**
 * @brief  Distance en mètres entre deux points (formule de Haversine).
 */
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

/**
 * @brief  Cap initial (bearing) de A vers B, en degrés (0–360°).
 */
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

static uint8_t GPS_NmeaChecksum(const char *s)
{
    uint8_t cs = 0;
    if (*s == '$') s++;
    while (*s && *s != '*') cs ^= (uint8_t)*s++;
    return cs;
}

static void GPS_SetNmea(const char *msgId, uint8_t ruart1)
{
    char buf[64], full[80];

    snprintf(buf, sizeof(buf), "$PUBX,40,%s,0,%d,0,0,0,0",
             msgId, ruart1);

    snprintf(full, sizeof(full), "%s*%02X\r\n",
             buf, GPS_NmeaChecksum(buf));

    HAL_UART_Transmit(&huart4, (uint8_t *)full, strlen(full), 200);
    HAL_Delay(50);
}

/**
 * @brief  Envoie un message UBX binaire avec checksum calculé.
 *         msg[] = class + id + len_lo + len_hi + payload  (sans sync ni CRC)
 */
static void GPS_SendUBX(const uint8_t *msg, uint16_t len)
{
    uint8_t buf[72];
    uint8_t ck_a = 0, ck_b = 0;

    if (len + 4 > sizeof(buf)) return;

    buf[0] = 0xB5;
    buf[1] = 0x62;
    memcpy(&buf[2], msg, len);

    for (uint16_t i = 0; i < len; i++) { ck_a += msg[i]; ck_b += ck_a; }
    buf[2 + len]     = ck_a;
    buf[2 + len + 1] = ck_b;

    HAL_UART_Transmit(&huart4, buf, len + 4, 200);
    HAL_Delay(50);
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
                	gpsStructData.isValid = 0;
                	    return;
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
    gpsStructData.isValid = 1;
    LOG_INFO("SPEED = %f , LATTITUDE = %f, LONGITUDE = %f, AZIMUTH = %f =\n\r", gpsStructData.groundSpeed, gpsStructData.latitude, gpsStructData.longitude, gpsStructData.azimuth);
}

void ParseGPS_GGA(char *gpsData)
{
    char *p = strstr(gpsData, "GGA");
    if (p == NULL) return;
    while (*p && *p != ',') p++;

    char    field[32];
    int     counter = 1;
    int     i       = 0;

    while (*p && *p != '*' && *p != '\r' && *p != '\n') {

        if (*p == ',') p++;

        i = 0;
        while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
            if (i < 30) field[i++] = *p;
            p++;
        }
        field[i] = '\0';

        switch (counter) {
            case 6:
                gpsStructData.fixQuality = (uint8_t)atoi(field);
                break;
            case 7:
                gpsStructData.satellites = (uint8_t)atoi(field);
                break;
            case 8:
                gpsStructData.hdop = atof(field);
                break;
            default:
                break;
        }

        counter++;

        /* on a tout ce qu'il faut après le champ 8 */
        if (counter > 8) break;
    }

    LOG_INFO("GGA | fix=%d  sats=%d  HDOP=%.1f\r\n",
             gpsStructData.fixQuality,
             gpsStructData.satellites,
             gpsStructData.hdop);
}
/*void ParseGPS_GLL(char* sentence)
{
    char* fields[8];
    char copy[100];
    strncpy(copy, sentence, sizeof(copy));

    uint8_t idx = 0;
    char* token = strtok(copy, ",");
    while(token && idx < 8) {
        fields[idx++] = token;
        token = strtok(NULL, ",");
    }
    if(idx < 7 || fields[6][0] != 'A') return;
    float raw_lat = atof(fields[1]);
    int deg_lat   = (int)(raw_lat / 100);
    float min_lat = raw_lat - (deg_lat * 100);
    gpsStructData.latitude = deg_lat + (min_lat / 60.0f);
    if(fields[2][0] == 'S') gpsStructData.latitude = -gpsStructData.latitude;
    float raw_lon = atof(fields[3]);
    int deg_lon   = (int)(raw_lon / 100);
    float min_lon = raw_lon - (deg_lon * 100);
    gpsStructData.longitude = deg_lon + (min_lon / 60.0f);
    if(fields[4][0] == 'W') gpsStructData.longitude = -gpsStructData.longitude;
    gpsStructData.isValid = 1;
    LOG_INFO("GLL fix: %.6f, %.6f\r\n",
              gpsStructData.latitude, gpsStructData.longitude);
}
*/

void InitGpsValues(void) {

    HAL_UART_Transmit(&huart4, cmd_baud, sizeof(cmd_baud)-1, 100);
    LOG_INFO("CMD baud sent\r\n");
    HAL_Delay(500);
    HAL_UART_DeInit(&huart4);
    huart4.Init.BaudRate = 115200;
    HAL_UART_Init(&huart4);
    HAL_Delay(100);
    LOG_INFO("UART @ 115200\r\n");
    GPS_SetNmea("GLL", 0);
	GPS_SetNmea("GSA", 0);
	GPS_SetNmea("GSV", 0);
	GPS_SetNmea("VTG", 0);
	GPS_SetNmea("ZDA", 0);
	GPS_SetNmea("GGA", 1);
	GPS_SetNmea("RMC", 1);
	GPS_SendUBX(ubx_rate_5hz, sizeof(ubx_rate_5hz));
	LOG_INFO("[GPS 3/6] Rate 5 Hz\r\n");
	GPS_SendUBX(ubx_nav5_sea, sizeof(ubx_nav5_sea));
	LOG_INFO("[GPS 4/6] NAV5 Sea\r\n");
	GPS_SendUBX(ubx_sbas, sizeof(ubx_sbas));
	LOG_INFO("[GPS 5/6] SBAS/EGNOS\r\n");
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart4, ubx_nav_sbas_poll, sizeof(ubx_nav_sbas_poll), 100);
	LOG_INFO("[GPS 6/6] SBAS poll\r\n");

}
