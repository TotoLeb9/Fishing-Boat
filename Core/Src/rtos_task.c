/*
 * rtos_task.c
 *
 *  Created on: Dec 15, 2025
 *      Author: totoleb
 */
#include "rtos_task.h"

#define LED         10
#define PHARE       20
#define SERVO_DROIT 30
#define SERVO_GAUCHE 40
#define DEAD_ZONE 50
#define MID_LEFT_RIGHT 430
#define DEBOUNCE_TIME_MS 200
volatile uint8_t servo_angle_gauche = 180;
int minimum_servo_gauche = 90;
volatile uint8_t servo_angle_droit = 0;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t ListeningNrfHandle;
const osThreadAttr_t listener_attr = {
    .name = "ListeningNRF",
    .priority = osPriorityHigh,
    .stack_size = 256 * 4
};

osThreadId_t DebugNrfFifoHandle;
const osThreadAttr_t debugfifo_attr = {
    .name = "DebugNrfFifo",
    .priority = osPriorityLow,
    .stack_size = 128 * 4
};

QueueHandle_t joyQueue = NULL;

void process_command(uint8_t command) {
	static uint32_t last_phare_time = 0;
	static uint32_t last_led_time = 0;
	uint32_t current_time = HAL_GetTick();
    switch (command) {
        case SERVO_DROIT:
            if (servo_angle_gauche > 90) servo_angle_gauche -= 30;
            else servo_angle_gauche=180;
            Servo_SetAngleGauche(servo_angle_gauche);
            printf("Commande : 0x%02X\r\n", command);
            printf("Servo angle: %d°\r\n", servo_angle_gauche);
            break;

        case SERVO_GAUCHE:
                    if (servo_angle_droit < 90) servo_angle_droit += 30;
                    else servo_angle_droit=0;
                    Servo_SetAngleDroit(servo_angle_droit);
                    printf("Commande : 0x%02X\r\n", command);
                    printf("Servo angle: %d°\r\n", servo_angle_droit);
                    break;
        case PHARE:
        	if (current_time - last_phare_time >= DEBOUNCE_TIME_MS) {
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        	printf("Commande : 0x%02X\r\n", command);
        	last_phare_time = current_time;
        	}
        	break;
        case LED:
        	if (current_time - last_led_time >= DEBOUNCE_TIME_MS) {
        	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        	printf("Commande  0x%02X\r\n", command);
        	last_led_time = current_time;
        	}
        	break;

        default:
            printf("Commande inconnue: 0x%02X\r\n", command);
            break;
    }
}

void InitQueue(void){
	joyQueue = xQueueCreate(
	        1,
	        sizeof(JoyCmd_t)
	    );
}

void StartDefaultTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void ListeningNrf(void *argument)
{
    uint8_t buffer[PAYLOAD_SIZE];

    for (;;)
    {
        if (nrf24_available())
        {
            if (nrf24_read(buffer, PAYLOAD_SIZE))
            {
                if (buffer[0] == 0xAA)
                {
                    JoyCmd_t cmd;
                    cmd.x = buffer[1];
                    cmd.y = buffer[2];

                    xQueueOverwrite(joyQueue, &cmd);
                }
                else if (buffer[0] == 0xBB || buffer[0] == 0xBA)
                {
                    process_command(buffer[1] & 0xFE);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void DebugFifoNrf(void *argument)
{

    for (;;)
    {
		uint8_t status_check, fifo_check;
		nrf24_read_register(NRF24_STATUS, &status_check, 1);
		nrf24_read_register(NRF24_FIFO_STATUS, &fifo_check, 1);
		LOG_INFO("[DEBUG] STATUS: 0x%02X | FIFO: 0x%02X | CE: %d\r\n",
		status_check, fifo_check,
		HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin));
		vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
