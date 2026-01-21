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
#include "ina219.h"
#include <stdbool.h>
#include "gy271.h"
#include "gps.h"

#define LED         10
#define PHARE       20
#define SERVO_DROIT 30
#define SERVO_GAUCHE 40
#define DEAD_ZONE 50
#define MID_LEFT_RIGHT 430
#define DEBOUNCE_TIME_MS 200
#define MAX_RETRY_VOLTAGE 5
#define START_FLAG 0x01
#define CMD_ALIVE_FLAG (1<<0)

typedef struct
{
    uint8_t x;          // Joystick X (direction)
    uint8_t y;          // Joystick Y (vitesse)
    uint32_t tick;      // Tick RTOS de réception
} JoyCmd_t;

extern QMC5883P_t mag;
extern QMC5883P_Data_t magData;

extern INA219_HandleTypedef ina219_sensor;
/***************************************
 * TASK ATTRIBUTES
 **************************************/
extern const osThreadAttr_t listener_attr;
extern const osThreadAttr_t voltageTask_attributes;
extern const osThreadAttr_t debugfifo_attr;
extern const osThreadAttr_t Motor_Attributes;
extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t watchDogNRF_attributes;
extern const osThreadAttr_t compass_attributes;
extern const osThreadAttr_t gps_attributes;
/************************************
 * HANDLER
 ***********************************/
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t DebugNrfFifoHandle;
extern osThreadId_t MotorTaskHandle;
extern osThreadId_t VoltageTaskHandle;
extern osThreadId_t ListeningNrfHandle;
extern osThreadId_t WatchDogNRFHandle;
extern osThreadId_t CompassHandle;
extern osThreadId_t GpsHandle;

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
#endif /* INC_RTOS_TASK_H_ */
