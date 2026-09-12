/*
 * esp_com.c
 *
 *  Created on: May 29, 2026
 *      Author: totoleb
 */


#include "esp_com.h"

int32_t g_dist_max  = 0;
int32_t g_dist_stop = 0;
int32_t g_speed_max = 0;
uint8_t nb_points_gps = 0;
uint8_t nb_largage = 0;

void parse_config_uart(const char* buf) {
    int dist_max, dist_stop, speed_max;
    if (sscanf(buf, "CONFIG:DIST_MAX=%d,DIST_STOP=%d,SPEED_MAX=%d",
               &dist_max, &dist_stop, &speed_max) == 3) {
        g_dist_max  = dist_max;
        g_dist_stop = dist_stop;
        g_speed_max = speed_max;
        LOG_INFO("Config: dist_max=%d dist_stop=%d speed_max=%d\r\n",
                 dist_max, dist_stop, speed_max);
    }
}

// parse "GPS_ARRAY:485200000/23500000,457500000/48300000\r\n"
void parse_gps_array_uart(const char* buf) {
    const char* p = strchr(buf, ':');
    if (!p) return;
    p++; // saute ':'

    nb_points_gps = 0;
    while (*p && nb_points_gps < MAX_LARGAGE) {
        int32_t lat, lon;
        if (sscanf(p, "%ld/%ld", &lat, &lon) == 2) {
            Pos_Largage[nb_points_gps].latitude  = lat;
            Pos_Largage[nb_points_gps].longitude = lon;
            nb_points_gps++;
        }
        p = strchr(p, ',');
        if (!p) break;
        p++; // saute ','
    }
    LOG_INFO("%u points GPS chargés\r\n", nb_points_gps);
}

// Envoie un point de largage à l'ESP32
// Format : "LARG:485200000|23500000\r\n"
void EnvoyerGpsLargage(GPS_Pos* pos) {
    char buf[64];
    int32_t lat = (int32_t)(pos->latitude  * 1e7f);
    int32_t lon = (int32_t)(pos->longitude * 1e7f);
    snprintf(buf, sizeof(buf), "LARG:%ld|%ld\r\n", (long)lat, (long)lon);
    HAL_UART_Transmit(&huart5, (uint8_t*)buf, strlen(buf), 200);
    LOG_INFO("Envoyé ESP32: %s", buf);
}

// Enregistre la position GPS actuelle comme point de largage
void EnregistrerLargage(void) {
    if (nb_largage >= MAX_LARGAGE) {
        LOG_INFO("MAX_LARGAGE atteint\r\n");
        return;
    }
    if (!gpsStructData.isValid) {
        LOG_INFO("GPS invalide, largage non enregistré\r\n");
        return;
    }

    Pos_Largage[nb_largage].latitude  = gpsStructData.latitude;
    Pos_Largage[nb_largage].longitude = gpsStructData.longitude;

    LOG_INFO("Largage[%u] enregistré: %.6f / %.6f\r\n",
             nb_largage,
             Pos_Largage[nb_largage].latitude,
             Pos_Largage[nb_largage].longitude);

    EnvoyerGpsLargage(&Pos_Largage[nb_largage]);
    nb_largage++;
}
