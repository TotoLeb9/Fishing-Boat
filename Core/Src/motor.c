#include "motor.h"
#include <stdlib.h>
#include <math.h>

/* --- CONFIGURATION PWM --- */
#define PWM_FULL_REVERSE 1000 // Standard ESC
#define PWM_NEUTRAL      1500
#define PWM_FULL_FORWARD 2000 // Standard ESC

/* --- CALIBRATION JOYSTICK (Tes valeurs réelles) --- */
#define JOY_MIN_X 0
#define JOY_MAX_X 255
#define JOY_CENTER_X 126       // Ajusté au milieu de 20 et 120

#define JOY_MIN_Y 0          // Ton minimum
#define JOY_MAX_Y 255         // Ton maximum
#define JOY_CENTER_Y 129         // Ton neutre réel

static float current_speed_g = 0.0f;
static float current_speed_d = 0.0f;

/* --- CONTRÔLE BAS NIVEAU --- */

void ESC_SetThrottle_D(uint16_t pulse_us) {
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void ESC_SetThrottle_G(uint16_t pulse_us) {
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);
}

/* --- SERVOMOTEURS --- */

void Servo_SetAngleGauche(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}

void Servo_SetAngleDroit(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

void Servo_SetAngleBas(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse = 500 + ((2000 * angle) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

/* --- LOGIQUE DE CALCUL --- */

/**
 * Normalise l'entrée malgré l'asymétrie
 */
float Normalize_Input(uint8_t val, uint8_t min, uint8_t center, uint8_t max)
{
    int delta = (int)val - center;

    if (abs(delta) <= INPUT_DEADZONE) return 0.0f;

    float res;
    if (val < center) {
        // Zone basse (30 à 40)
        res = (float)(val - center) / (float)(center - min);
    } else {
        // Zone haute (40 à 120)
        res = (float)(val - center) / (float)(max - center);
    }

    if (res > 1.0f) res = 1.0f;
    if (res < -1.0f) res = -1.0f;
    return res;
}

float Smooth_Transition(float current, float target) {
    float diff = target - current;
    if (fabsf(diff) <= ACCEL_RATE) return target;
    return (diff > 0) ? (current + ACCEL_RATE) : (current - ACCEL_RATE);
}

uint16_t Float_To_PWM(float value) {
    return (uint16_t)(PWM_NEUTRAL + (value * 500.0f));
}

/**
 * Pilotage différentiel
 */
void Handle_Joystick(uint8_t x, uint8_t y)
{
    float throttle = Normalize_Input(y, JOY_MIN_Y, JOY_CENTER_Y, JOY_MAX_Y);
    float steering = Normalize_Input(x, JOY_MIN_X, JOY_CENTER_X, JOY_MAX_X);

    // Zone morte sur le steering
    if (fabsf(steering) < 0.20f) {
        steering = 0.0f;
    }

    // Mixage différentiel
    float target_g = throttle + (steering * TURN_SENSITIVITY);
    float target_d = throttle - (steering * TURN_SENSITIVITY);

    // Limitation
    float max_v = fmaxf(fabsf(target_g), fabsf(target_d));
    if (max_v > 1.0f) {
        target_g /= max_v;
        target_d /= max_v;
    }

    // Lissage
    current_speed_g = Smooth_Transition(current_speed_g, target_g);
    current_speed_d = Smooth_Transition(current_speed_d, target_d);

    // **APPLICATION DES CORRECTIONS AVANT ENVOI**
    float corrected_g = current_speed_g * MOTOR_LEFT_CORRECTION;
    float corrected_d = current_speed_d * MOTOR_RIGHT_CORRECTION;

    // Envoi aux moteurs
    ESC_SetThrottle_G(Float_To_PWM(corrected_g));
    ESC_SetThrottle_D(Float_To_PWM(corrected_d));
}

/**
 * Initialisation indispensable pour armer les ESC
 */
void ESC_Initialize(void)
{
    current_speed_g = 0.0f;
    current_speed_d = 0.0f;

    // Étape d'armement : Envoyer NEUTRE pendant 3 secondes
    ESC_SetThrottle_G(PWM_NEUTRAL);
    ESC_SetThrottle_D(PWM_NEUTRAL);
    HAL_Delay(3000);
}

void ESC_ZTW_Calibration_Bidirectional(void)
{
    printf("\n=== ZTW SHARK G2 CALIBRATION (FORWARD / REVERSE) ===\n\r");

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);

    printf("\n[1] NEUTRE (1500us)\n\r");
    printf("    -> Branchez la batterie ESC\n\r");
    HAL_Delay(4000);

    printf("\n[2] AVANT MAX (2000us)\n\r");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
    HAL_Delay(3000);

    printf("\n[3] ARRIÈRE MAX (1000us)\n\r");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
    HAL_Delay(3000);
    printf("\n[4] RETOUR NEUTRE (1500us)\n\r");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);
    HAL_Delay(3000);

    printf("\n✔ CALIBRATION TERMINÉE\n\r");
}


void ESC_ZTW_EnterProgramMode(void)
{
    printf("\n=== ZTW SHARK G2 : ENTER PROGRAM MODE ===\n\r");
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    printf("\n[1] FULL THROTTLE (2000us)\n\r");
    printf("    -> Débranchez la batterie ESC\n\r");

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);

    HAL_Delay(3000);
    printf("\n[2] BRANCHEZ LA BATTERIE ESC MAINTENANT\n\r");
    printf("    -> Attendez Beep-Beep- (2s)\n\r");

    HAL_Delay(2500);
    printf("\n[3] NE TOUCHEZ À RIEN\n\r");
    printf("    -> Beep-Beep- répété 4 fois\n\r");
    HAL_Delay(6000);
    printf("\n✔ PROGRAM MODE SHOULD BE ACTIVE\n\r");
    printf("Utilisez maintenant les impulsions throttle\n\r");
}

void ESC_ZTW_Set_RunningMode_Value2(void)
{
    printf("\n=== ZTW PROGRAM MODE : ITEM 1 / VALUE 2 ===\n\r");
    printf("\n[STEP 2] Attente du menu...\n\r");
    printf("         -> Écoutez : Beep- (Running Mode)\n\r");
    printf("         -> Passez au MIN dans les 3s\n\r");

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
    HAL_Delay(4500);
    printf("-> Sélection ITEM 1 (THROTTLE MIN)\n\r");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
    HAL_Delay(3000);
    printf("\n[VALUE] Attente Value 2 : Beep-Beep-\n\r");
    printf("        -> Passez au MAX pour valider\n\r");
    HAL_Delay(3500);
    printf("-> Validation VALUE 2 (THROTTLE MAX)\n\r");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
    HAL_Delay(3000);
    printf("\n✔ RUNNING MODE = VALUE 2 SAUVEGARDÉ\n\r");
    printf("Retour automatique au STEP #2 (menu principal)\n\r");
}
