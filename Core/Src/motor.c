/*
 * motor.c
 *
 * Created on: Dec 11, 2025
 * Author: totoleb
 */

#include "motor.h"
#include <stdlib.h>
#include <math.h>
#define PWM_MIN 1000
#define PWM_MAX 1400
#define PWM_NEUTRAL 1200
#define DEADZONE 2
#define JOY_MIN_X 16
#define JOY_MAX_X 70
#define JOY_CENTER_X 26
#define JOY_MIN_Y 15
#define JOY_MAX_Y 63
#define JOY_CENTER_Y 44
#define MIX_PERCENT 0.6f


void Servo_SetAngleGauche(uint8_t angle)
{
	if (angle < 90){
		angle = 180;
	}
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}

void Servo_SetAngleDroit(uint8_t angle)
{
	if (angle > 90){
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


float map_to_float(uint8_t val, uint8_t min, uint8_t center, uint8_t max) {
    int16_t diff = (int16_t)val - center;

    if (abs(diff) <= DEADZONE) return 0.0f;

    if (diff > 0) {
        float range = (float)(max - center - DEADZONE);
        if (range <= 0) return 1.0f;
        float result = (float)(diff - DEADZONE) / range;
        return (result > 1.0f) ? 1.0f : result;
    } else {
        float range = (float)(center - DEADZONE - min);
        if (range <= 0) return -1.0f;
        float result = (float)(diff + DEADZONE) / range;
        return (result < -1.0f) ? -1.0f : result;
    }
}

void Handle_Joystick(uint8_t x, uint8_t y)
{
    float throttle = map_to_float(y, JOY_MIN_Y, JOY_CENTER_Y, JOY_MAX_Y);
    float turn = map_to_float(x, JOY_MIN_X, JOY_CENTER_X, JOY_MAX_X);

    float left = throttle + (turn * 0.7f);
    float right = throttle - (turn * 0.7f);

    float max_val = fmaxf(fabsf(left), fabsf(right));
    if (max_val > 1.0f) {
        left /= max_val;
        right /= max_val;
    }

    uint16_t pwm_g, pwm_d;

    if (throttle == 0.0f && turn == 0.0f) {
        pwm_g = PWM_NEUTRAL;
        pwm_d = PWM_NEUTRAL;
    } else {
        pwm_g = PWM_NEUTRAL + (int16_t)(left * (left > 0 ? (PWM_MAX - PWM_NEUTRAL) : (PWM_NEUTRAL - PWM_MIN)));
        pwm_d = PWM_NEUTRAL + (int16_t)(right * (right > 0 ? (PWM_MAX - PWM_NEUTRAL) : (PWM_NEUTRAL - PWM_MIN)));
    }

    ESC_SetThrottle_G(pwm_d);
    ESC_SetThrottle_D(pwm_g);
}
