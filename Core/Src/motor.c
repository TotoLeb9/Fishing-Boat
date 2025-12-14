/*
 * motor.c
 *
 * Created on: Dec 11, 2025
 * Author: totoleb
 */

#include "motor.h"
#include <stdlib.h>

#define PWM_MIN 1000
#define PWM_MAX 1300
#define PWM_NEUTRAL 1150
#define DEADZONE 3
#define JOY_MIN_X 16
#define JOY_MAX_X 70
#define JOY_CENTER_X 43
#define JOY_MIN_Y 24
#define JOY_MAX_Y 63
#define JOY_CENTER_Y 44
#define MIX_PERCENT 0.6f

void Servo_SetAngleGauche(uint8_t angle)
{
	if (angle < 90){
		HAL_Delay(10);
		angle = 180;
	}
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}

void Servo_SetAngleDroit(uint8_t angle)
{
	if (angle > 90){
		HAL_Delay(10);
		angle = 0;
	}
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

void Servo_SetAngleBas(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

void ESC_SetThrottle_D(uint16_t pulse_us)
{
    if(pulse_us < 1000) pulse_us = 1000;
    if(pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void ESC_SetThrottle_G(uint16_t pulse_us)
{
    if(pulse_us < 1000) pulse_us = 1000;
    if(pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t clamp_pwm(uint16_t v) {
    if (v < PWM_MIN) return PWM_MIN;
    if (v > PWM_MAX) return PWM_MAX;
    return v;
}

uint16_t map_joy_to_pwm(uint8_t y)
{
    if (y < JOY_MIN_Y) y = JOY_MIN_Y;
    if (y > JOY_MAX_Y) y = JOY_MAX_Y;

    if (y >= (JOY_CENTER_Y - DEADZONE) && y <= (JOY_CENTER_Y + DEADZONE)) {
        return PWM_NEUTRAL;
    }

    if (y > JOY_CENTER_Y + DEADZONE) {
        return map(y, JOY_CENTER_Y + DEADZONE, JOY_MAX_Y, PWM_NEUTRAL, PWM_MAX);
    }

    if (y < JOY_CENTER_Y - DEADZONE) {
        return map(y, JOY_MIN_Y, JOY_CENTER_Y - DEADZONE, PWM_MIN, PWM_NEUTRAL);
    }

    return PWM_NEUTRAL;
}


void Handle_Joystick(uint8_t x, uint8_t y)
{
    uint16_t pwm_base;


    pwm_base = map_joy_to_pwm(y);

    int16_t raw_delta = (int16_t)x - JOY_CENTER_X;
    float norm = 0.0f;
    if (raw_delta > DEADZONE) {
        float effective_delta = (float)(raw_delta - DEADZONE);
        float effective_range = (float)(JOY_MAX_X - JOY_CENTER_X - DEADZONE);
        if (effective_range > 0.0f) {
             norm = effective_delta / effective_range;
        } else {
             norm = 1.0f;
        }
    }
    else if (raw_delta < -DEADZONE) {
        float effective_delta = (float)(raw_delta + DEADZONE);
        float effective_range = (float)(JOY_CENTER_X - JOY_MIN_X - DEADZONE);
        if (effective_range > 0.0f) {
            norm = effective_delta / effective_range;
        } else {
            norm = -1.0f;
        }
    }
    else {
        norm = 0.0f;
    }

    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    float correction = norm * MIX_PERCENT * (PWM_MAX - PWM_MIN);

    if (pwm_base == PWM_NEUTRAL && norm == 0.0f) {
        ESC_SetThrottle_D(PWM_NEUTRAL);
        ESC_SetThrottle_G(PWM_NEUTRAL);
        return;
    }

    uint16_t pwm_d = clamp_pwm(pwm_base + (int16_t)correction);
    uint16_t pwm_g = clamp_pwm(pwm_base - (int16_t)correction);

    ESC_SetThrottle_D(pwm_d);
    ESC_SetThrottle_G(pwm_g);
}
