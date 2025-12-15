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

typedef struct
{
    uint8_t x;          // Joystick X (direction)
    uint8_t y;          // Joystick Y (vitesse)
    uint32_t tick;      // Tick RTOS de réception
} JoyCmd_t;


extern QueueHandle_t joyQueue;
extern const osThreadAttr_t defaultTask_attributes;
extern osThreadId_t ListeningNrfHandle;
extern const osThreadAttr_t listener_attr;
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t DebugNrfFifoHandle;
extern const osThreadAttr_t debugfifo_attr;
extern volatile uint8_t servo_angle_gauche;
extern int minimum_servo_gauche;
extern volatile uint8_t servo_angle_droit;


void StartDefaultTask(void *argument);
void ListeningNrf(void *argument);
void DebugFifoNrf(void *argument);
void process_command(uint8_t command);

#endif /* INC_RTOS_TASK_H_ */
