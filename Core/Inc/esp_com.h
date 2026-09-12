/*
 * esp_com.h
 *
 *  Created on: May 2, 2026
 *      Author: totoleb
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include "main.h"
#include <stdio.h>
#include "gps.h"
#include "rtos_task.h"
#ifndef INC_ESP_COM_H_
#define INC_ESP_COM_H_

extern uint8_t landingSpotNumber;
extern int32_t g_dist_max;
extern int32_t g_dist_stop;
extern int32_t g_speed_max;
extern uint8_t nb_points_gps;
extern UART_HandleTypeDef huart5;
extern GPS_Pos Pos_Largage[16];
static inline void wakeUpEsp(GPIO_TypeDef *gpio, uint16_t pin){
	HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
}

static inline void sleepModeEsp(GPIO_TypeDef *gpio, uint16_t pin){
	HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
}

void parse_gps_array_uart(const char* buf);
void parse_config_uart(const char* buf);
void EnvoyerGpsLargage(GPS_Pos* pos);
void EnregistrerLargage(void);

#endif /* INC_ESP_COM_H_ */
