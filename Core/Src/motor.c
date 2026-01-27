/*
 * motor.c
 *
 * Created on: Dec 11, 2025
 * Author: totoleb
 */

#include "motor.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define PWM_FULL_REVERSE 1200
#define PWM_NEUTRAL      1500
#define PWM_FULL_FORWARD 1800

#define INPUT_DEADZONE 3
#define TURN_SENSITIVITY 0.6f
#define ACCEL_RATE 0.05f

#define JOY_MIN_X 20
#define JOY_MAX_X 66
#define JOY_CENTER_X 28

#define JOY_MIN_Y 16
#define JOY_MAX_Y 65
#define JOY_CENTER_Y 38

static float current_speed_g = 0.0f;
static float current_speed_d = 0.0f;

/**
* @brief Met le servo de gauche à un angle donné
*/
void Servo_SetAngleGauche(uint8_t angle)
{
	if (angle < 90) angle = 180;
	uint16_t pulse = 500 + ((2000 * angle) / 180);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}

/**
* @brief Met le servo de droite à un angle donné
*/
void Servo_SetAngleDroit(uint8_t angle)
{
	if (angle > 90) angle = 0;
	uint16_t pulse = 500 + ((2000 * angle) / 180);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

/**
* @brief Met le servo du bas à un angle donné
*/
void Servo_SetAngleBas(uint8_t angle)
{
	if (angle > 180) angle = 180;
	uint16_t pulse = 500 + ((2000 * angle) / 180);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

/**
 * @brief Convertit valeur joystick en float -1.0 à +1.0 avec zone morte
 * @param val Valeur brute du joystick
 * @param inverted 1 si l'axe est inversé (ex: Y où haut < bas), sinon 0
 */
float Normalize_Input(uint8_t val, uint8_t min, uint8_t center, uint8_t max, uint8_t inverted)
{
    if (abs((int)val - center) <= INPUT_DEADZONE) {
        return 0.0f;
    }
    float result = 0.0f;
    if (val < center) {
        float range = (float)(center - min);
        if (range <= 0) range = 1;
        result = (float)(val - center) / range;
    } else {
        float range = (float)(max - center);
        if (range <= 0) range = 1;
        result = (float)(val - center) / range;
    }
    if (result > 1.0f) result = 1.0f;
    if (result < -1.0f) result = -1.0f;
    return inverted ? -result : result;
}

/**
 * @brief Applique une rampe d'accélération pour éviter les chocs
 */
float Smooth_Transition(float current, float target)
{
    float diff = target - current;
    if (fabsf(diff) <= ACCEL_RATE) {
        return target;
    }
    if (diff > 0) return current + ACCEL_RATE;
    else return current - ACCEL_RATE;
}

/**
 * @brief Convertit le float final en impulsion PWM (us)
 */
uint16_t Float_To_PWM(float value)
{
    if (fabsf(value) < 0.01f) return PWM_NEUTRAL;

    if (value > 0.0f) {
        return (uint16_t)(PWM_NEUTRAL + (value * (PWM_FULL_FORWARD - PWM_NEUTRAL)));
    } else {
        return (uint16_t)(PWM_NEUTRAL + (value * (PWM_NEUTRAL - PWM_FULL_REVERSE)));
    }
}


void ESC_SetThrottle_D(uint16_t pulse_us) {
    if (pulse_us < PWM_FULL_REVERSE) pulse_us = PWM_FULL_REVERSE;
    if (pulse_us > PWM_FULL_FORWARD) pulse_us = PWM_FULL_FORWARD;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void ESC_SetThrottle_G(uint16_t pulse_us) {
    if (pulse_us < PWM_FULL_REVERSE) pulse_us = PWM_FULL_REVERSE;
    if (pulse_us > PWM_FULL_FORWARD) pulse_us = PWM_FULL_FORWARD;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);
}

/**
 * @brief Cœur du pilotage Différentiel
 */
void Handle_Joystick(uint8_t x, uint8_t y)
{
    float throttle = Normalize_Input(y, JOY_MIN_Y, JOY_CENTER_Y, JOY_MAX_Y, 0);
    float steering = Normalize_Input(x, JOY_MIN_X, JOY_CENTER_X, JOY_MAX_X, 0);
    float target_g = throttle + (steering * TURN_SENSITIVITY);
    float target_d = throttle - (steering * TURN_SENSITIVITY);
    float max_val = fmaxf(fabsf(target_g), fabsf(target_d));
    if (max_val > 1.0f) {
        target_g /= max_val;
        target_d /= max_val;
    }
    current_speed_g = Smooth_Transition(current_speed_g, target_g);
    current_speed_d = Smooth_Transition(current_speed_d, target_d);
    uint16_t pwm_g = Float_To_PWM(current_speed_g);
    uint16_t pwm_d = Float_To_PWM(current_speed_d);

    ESC_SetThrottle_G(pwm_g);
    ESC_SetThrottle_D(pwm_d);
}

/**
 * @brief Initialisation
 */
void ESC_Initialize(void)
{
    current_speed_g = 0.0f;
    current_speed_d = 0.0f;

    ESC_SetThrottle_G(PWM_NEUTRAL);
    ESC_SetThrottle_D(PWM_NEUTRAL);
    HAL_Delay(3000);
}
