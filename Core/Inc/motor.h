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
#include <math.h>
#include <stdio.h>

#define PWM_FULL_REVERSE 2000 // Standard ESC
#define PWM_NEUTRAL      1500
#define PWM_FULL_FORWARD 1000//Standard ESC
#define PWM_HALF_RANGE (PWM_FULL_FORWARD - PWM_NEUTRAL)  // = 200
/* --- PARAMÈTRES PILOTAGE --- */
#define INPUT_DEADZONE 3     // Réduit car ta plage basse est courte
#define TURN_SENSITIVITY 0.5f  // Réduit pour plus de stabilité
#define ACCEL_RATE 0.05f       // Plus rapide pour plus de réactivité
#define STEERING_DEADZONE 0.05f   // 5 %
#define THROTTLE_DEADZONE 0.05f
#define MOTOR_LEFT_CORRECTION   1.00f // Réduit la puissance du moteur gauche de 5%
#define MOTOR_RIGHT_CORRECTION 0.85f   // Moteur droit = référence

/* --- CALIBRATION JOYSTICK (Tes valeurs réelles) --- */
#define JOY_MIN_X 0
#define JOY_MAX_X 255
#define JOY_CENTER_X 126       // Ajusté au milieu de 20 et 120

#define JOY_MIN_Y 0          // Ton minimum
#define JOY_MAX_Y 255         // Ton maximum
#define JOY_CENTER_Y 129         // Ton neutre réel

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
void ESC_Initialize(void);
void ESC_ZTW_Calibration_Bidirectional(void);
void ESC_ZTW_EnterProgramMode(void);
void ESC_ZTW_Set_RunningMode_Value2(void);
#endif /* INC_MOTOR_H_ */
