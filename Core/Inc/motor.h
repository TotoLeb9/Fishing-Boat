/*
 * motor.h
 *
 *  Created on: Dec 11, 2025
 *      Author: totoleb
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Servo_SetAngleGauche(uint8_t angle);
void Servo_SetAngleDroit(uint8_t angle);
void Servo_SetAngleBas(uint8_t angle);
void ESC_SetThrottle_D(uint16_t pulse_us);
void ESC_SetThrottle_G(uint16_t pulse_us);
uint16_t map_joy_to_pwm(uint8_t joy_y_value);
int16_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void generate_motor_outputs(uint8_t joy_x_value, uint8_t joy_y_value, uint16_t *pwm_g, uint16_t *pwm_d);
void Handle_Joystick(uint8_t x, uint8_t y);

#endif /* INC_MOTOR_H_ */
