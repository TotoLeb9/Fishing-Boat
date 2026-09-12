/*
 * rtos_task.h
 *
 *  Created on: Dec 15, 2025
 *      Author: totoleb
 */

#ifndef INC_RTOS_TASK_H_
#define INC_RTOS_TASK_H_
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "nrf.h"
#include "log.h"
#include "motor.h"
#include <stdbool.h>
#include "gy271.h"
#include "gps.h"
#include "battery.h"
#include "ekf.h"
#include "telemetry.h"
#include "esp_com.h"



#define LED         10
#define PHARE       20
#define SERVO_DROIT 30
#define SERVO_GAUCHE 40
#define SERVO_ARRIERE 50
#define MODE_BOAT 60
#define MID_LEFT_RIGHT 430
#define DEBOUNCE_TIME_MS 200
#define MAX_RETRY_VOLTAGE 5
#define START_FLAG 0x01
#define CMD_ALIVE_FLAG (1<<0)
#define NUMBER_CAPTURE 10
#define TIME_BEFORE_HOME 10
#define KP_HEADING  0.01f
#define HOME_THROTTLE  0.5f
#define SAMPLE_RATE_GPS_HOME 20
#define LEFT_DOOR_OPEN 0
#define RIGHT_DOOR_OPEN 0
#define INCREMENT_DOOR 15
#define HOME_TIMEOUT_MS   120000   // 2 minutes max
#define KD_HEADING        0.05f    // À ajuster selon comportement
#define SERVO_STEP_MS   20      // Période de mise à jour
#define SERVO_RATE       2
#define MAX_LARGAGE 16
#define UART_RX_BUF_SIZE  256
#define UART_TX_BUF_SIZE  256

typedef struct
{
    uint8_t x;          // Joystick X (direction)
    uint8_t y;          // Joystick Y (vitesse)
    uint32_t tick;      // Tick RTOS de réception
} JoyCmd_t;

extern volatile uint32_t bData;
extern QMC5883P_t mag;
extern QMC5883P_Data_t magData;
extern ADC_HandleTypeDef hadc1;
extern GPS_Pos pos_depart;
extern KalmanCap_t kalman;
extern KalmanConfig_t kalmanConfig;
/***************************************
 * TASK ATTRIBUTES
 **************************************/
extern const osThreadAttr_t listener_attr;
extern const osThreadAttr_t debugfifo_attr;
extern const osThreadAttr_t Motor_Attributes;
//extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t watchDogNRF_attributes;
extern const osThreadAttr_t compass_attributes;
extern const osThreadAttr_t gps_attributes;
extern const osThreadAttr_t battery_attributes;
extern const osThreadAttr_t returnHome_attributes;
extern const osThreadAttr_t servo_attributes;
extern const osThreadAttr_t espcom_attributes;
/************************************
 * HANDLER
 ***********************************/
//extern osThreadId_t defaultTaskHandle;
extern osThreadId_t DebugNrfFifoHandle;
extern osThreadId_t MotorTaskHandle;
extern osThreadId_t ListeningNrfHandle;
extern osThreadId_t WatchDogNRFHandle;
extern osThreadId_t CompassHandle;
extern osThreadId_t GpsHandle;
extern osThreadId_t BatteryHandle;
extern osThreadId_t ReturnHomeHandle;
extern osThreadId_t ServoTaskHandle;
extern osThreadId_t EspComHandle;

/***************************************
 * MUTEX ET QUEUE
 **************************************/
extern osMutexId_t nrfMutex;
extern const osMutexAttr_t nrfMutex_attributes;
extern QueueHandle_t joyQueue;

extern volatile uint8_t servo_angle_droit;
extern volatile uint8_t servo_angle_gauche;
extern int minimum_servo_gauche;
extern int packet;

/***************************
 * FONCTIONS ET TACHES
 **************************/

/**
 * @brief Tâche de démarrage
 * @details Reçoit la première valeur de batterie pour s'assurer
 * du bon fonctionnement du système
 * @param argument Paramètre RTOS (non utilisé)
 */
void StartDefaultTask(void *argument);

/**
 * @brief Tâche d'affichage LCD
 * @details Ecoute en temps réel les instructions reçu par la télécommande
 * et effectue les actions en conséquences, allumer les leds, phares et
 * tourner les moteurs
 * @param argument Paramètre RTOS (non utilisé)
 */
void ListeningNrf(void *argument);

/**
 * @brief Tâche de debug
 * @note Tâche optionnelle et commenté de base
 * @details Donne des informations en temps réel sur l'état
 * des registres du NRF. Permet de connaitre l'état des piles etc...
 * @param argument Paramètre RTOS (non utilisé)
 */
void DebugFifoNrf(void *argument);

void process_command(uint8_t command);
void MotorTask(void *argument);
void VoltageTask(void *argument);
void InitQueue(void);
void WatchDogNrfTask(void *argument);
void CompassTask(void *argument);
void GpsTask(void *argument);
void BatteryTask(void *argument);
void ReturnToHomeTask(void *argument);
void ServoTask(void *argument);
void EspComTask(void* argument);
#endif /* INC_RTOS_TASK_H_ */
