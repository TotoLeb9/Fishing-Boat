/**
 * @file    rtos_task.c
 * @brief   Gestion des tâches FreeRTOS et de l'interface utilisateur
 *
 * @details
 * - Gestion des boutons (EXTI + notifications)
 * - Communication NRF24 (TX/RX)
 * - Affichage LCD (ST7735)
 * - Supervision (Watchdog, tension batterie)
 *
 * @author  totoleb
 * @date    2025-12-17
 */
#include "rtos_task.h"

volatile uint8_t servo_angle_gauche = 180;
volatile uint8_t servo_angle_droit = 0;
volatile bool requestVoltage = false;
volatile uint32_t last_cmd_tick = 0;
volatile uint8_t nrfConnected = 0;

uint8_t result;
uint8_t payload[PAYLOAD_SIZE];
int minimum_servo_gauche = 90;


osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t ListeningNrfHandle;
const osThreadAttr_t listener_attr = {
    .name = "ListeningNRF",
    .priority =  osPriorityRealtime,
    .stack_size = 1024 * 4
};

osThreadId_t DebugNrfFifoHandle;
const osThreadAttr_t debugfifo_attr = {
    .name = "DebugNrfFifo",
    .priority = osPriorityLow,
    .stack_size = 128 * 4
};

osThreadId_t MotorTaskHandle;
const osThreadAttr_t Motor_Attributes = {
	.name = "MotorTask",
	.priority = osPriorityAboveNormal,
	.stack_size = 512 * 4
};

osThreadId_t VoltageTaskHandle;
const osThreadAttr_t voltageTask_attributes = {
		.name = "VoltageTask",
		.priority = osPriorityAboveNormal,
		.stack_size = 512 * 4
};

osThreadId_t WatchDogNRFHandle;
const osThreadAttr_t watchDogNRF_attributes = {
		.name = "watchDogNRFTask",
		.priority = osPriorityNormal,
		.stack_size = 512 * 4
};

osThreadId_t CompassHandle;
const osThreadAttr_t compass_attributes = {
		.name = "CompassTask",
		.priority = osPriorityNormal,
		.stack_size = 256 * 4
};

osThreadId_t GpsHandle;
const osThreadAttr_t gps_attributes = {
		.name = "GPSTask",
		.priority = osPriorityNormal,
		.stack_size = 512 * 4
};



const osMutexAttr_t nrfMutex_attributes = {
    .name = "nrfMutex"
};
QueueHandle_t joyQueue = NULL;
int packet = 0;
osMutexId_t nrfMutex;


void StartDefaultTask(void *argument)
{
    uint8_t local_payload[PAYLOAD_SIZE];
    uint8_t handshake_ok = 0;
    uint8_t retry = 0;
    osDelay(100);
    float Vbus = INA219_GetBusVoltage_V(&ina219_sensor);
    uint16_t vbus_mv = (uint16_t)(Vbus * 100.0f);
    memset(local_payload, 0, PAYLOAD_SIZE);
    local_payload[0] = 0xEE;
    local_payload[1] = (vbus_mv >> 8) & 0xFF;
    local_payload[2] = (vbus_mv & 0xFF);

    LOG_INFO("Envoi handshake, voltage: %u mV\r\n", vbus_mv);
    while(!handshake_ok && retry < 10)
    {
        if(nrf24_write(local_payload, PAYLOAD_SIZE))
        {
            handshake_ok = 1;
            LOG_INFO("Handshake envoyé !\r\n");
        }
        else
        {
            retry++;
            LOG_INFO("Retry handshake %u/10\r\n", retry);
            osDelay(100);
        }
    }

    if(!handshake_ok)
    {
        LOG_INFO("ERREUR: Handshake échoué !\r\n");
    }

    nrf24_start_listening();
    osDelay(10);
    osThreadFlagsSet(MotorTaskHandle, START_FLAG);
    osThreadFlagsSet(ListeningNrfHandle, START_FLAG);
    osThreadFlagsSet(VoltageTaskHandle, START_FLAG);
    osThreadFlagsSet(CompassHandle , START_FLAG);
    osThreadFlagsSet(GpsHandle , START_FLAG);
    LOG_INFO("Toutes les tâches démarrées\r\n");

    for(;;)
    {
        osDelay(1000);
    }
}

/**********************************************
 *
 **********************************************/
void process_command(uint8_t command) {
	static uint32_t last_phare_time = 0;
	static uint32_t last_led_time = 0;
	uint32_t current_time = HAL_GetTick();
    switch (command) {
        case SERVO_DROIT:
            if (servo_angle_gauche > 90) servo_angle_gauche -= 30;
            else servo_angle_gauche=180;
            Servo_SetAngleGauche(servo_angle_gauche);
            LOG_INFO("Commande : 0x%02X\r\n", command);
            LOG_INFO("Servo angle droit: %d°\r\n", servo_angle_gauche);
            break;

        case SERVO_GAUCHE:
                    if (servo_angle_droit < 90) servo_angle_droit += 30;
                    else servo_angle_droit=0;
                    Servo_SetAngleDroit(servo_angle_droit);
                    LOG_INFO("Commande : 0x%02X\r\n", command);
                    LOG_INFO("Servo angle gauche: %d°\r\n", servo_angle_droit);
                    break;
        case PHARE:
        	if (current_time - last_phare_time >= DEBOUNCE_TIME_MS) {
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
        	LOG_INFO("Commande : 0x%02X\r\n", command);
        	last_phare_time = current_time;
        	}
        	break;
        case LED:
        	if (current_time - last_led_time >= DEBOUNCE_TIME_MS) {
        	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        	LOG_INFO("Commande  0x%02X\r\n", command);
        	last_led_time = current_time;
        	}
        	break;

        default:
        	LOG_INFO("Commande inconnue: 0x%02X\r\n", command);
            break;
    }
}

void CompassTask(void *argument){
	osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
	for(;;){
		if (QMC5883P_ReadXYZ(&mag, &magData) == QMC_OK)
		  {
			  printf("X:%.2f Y:%.2f Z:%.2f %f \r\n",
					 magData.x, magData.y, magData.z , magData.heading);
		  }
		  float heading = QMC5883P_GetHeadingDeg(&mag, 0.0f);
		  LOG_INFO("HEADING = %f\n\r", heading);
		  osDelay(100);
	}

}


void WatchDogNrfTask(void *argument){
	uint32_t flags;
	for(;;){
		flags = osThreadFlagsWait(
		            CMD_ALIVE_FLAG,
		            osFlagsWaitAny,
		            300
			);
		if(!(flags & CMD_ALIVE_FLAG)){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			nrfConnected = 0;
		}
		else
		{
			nrfConnected = 1;
		}

	}
}

void MotorTask(void *argument)
{
    JoyCmd_t joy_values;
    osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    for (;;)
    {
        if (xQueueReceive(joyQueue, &joy_values, portMAX_DELAY) == pdPASS)
        {
        	Handle_Joystick(joy_values.x, joy_values.y);
        }
    }
}

void InitQueue(void){
	joyQueue = xQueueCreate(
	        1,
	        sizeof(JoyCmd_t)
	    );
}

void ListeningNrf(void *argument)
{
	osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    uint8_t buffer[PAYLOAD_SIZE];
    uint8_t buffer_tx[PAYLOAD_SIZE];
    for (;;)
    {
    	if (osMutexAcquire(nrfMutex, 10) == osOK) {


        if (nrf24_available())
        {
            if (nrf24_read(buffer, PAYLOAD_SIZE))
            {

                if (buffer[0] == 0xAA)
                {
                    JoyCmd_t cmd;
                    cmd.x = buffer[1];
                    cmd.y = buffer[2];
                    packet++;
                    printf("X:%u Y=%u\r\n",cmd.x,cmd.y);
                    osDelay(50);
                    xQueueOverwrite(joyQueue, &cmd);
                    osThreadFlagsSet(WatchDogNRFHandle, CMD_ALIVE_FLAG);
                }
                else if (buffer[0] == 0xBB || buffer[0] == 0xBA)
                {
                	packet++;
                	LOG_INFO("PKT:%d\n",packet);
                    process_command(buffer[1] & 0xFE);
                }
                else if (buffer[0] >= 0xC0 && buffer[0] <= 0xCF)
                {
                	 LOG_INFO("Demande voltage reçue\r\n");
					float Vbus = INA219_GetBusVoltage_V(&ina219_sensor);
					uint16_t vbus_mv = (uint16_t)(Vbus * 100.0f);
					memset(buffer_tx, 0, PAYLOAD_SIZE);
					buffer_tx[0] = 0xDD;
					buffer_tx[1] = (vbus_mv >> 8) & 0xFF;
					buffer_tx[2] = 0xAA;
					nrf24_stop_listening();
					osDelay(2);
					uint8_t tx_result = nrf24_write(buffer_tx, PAYLOAD_SIZE);
					LOG_INFO("Voltage envoyé: %u mV (result=%u)\r\n", vbus_mv, tx_result);
					osDelay(2);
					nrf24_start_listening();

                }

            }
            osMutexRelease(nrfMutex);
        }

        osDelay(5);
    }
}
}

void DebugFifoNrf(void *argument)
{
	osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    for (;;)
    {
    	float Vbus = INA219_GetBusVoltage_V(&ina219_sensor);
    	LOG_INFO("%d", (int)(Vbus * 1000));

		uint8_t status_check, fifo_check;
		nrf24_read_register(NRF24_STATUS, &status_check, 1);
		nrf24_read_register(NRF24_FIFO_STATUS, &fifo_check, 1);
		LOG_INFO("[DEBUG] STATUS: 0x%02X | FIFO: 0x%02X | CE: %d\r\n",
		status_check, fifo_check,
		HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin));
		vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance==UART4){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(GpsHandle, Size, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


void GpsTask(void *argument) {
    uint32_t size;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_data, PAYLOAD_GPS_SIZE);
    osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    for(;;) {
        if (xTaskNotifyWait(0, 0, &size, portMAX_DELAY) == pdPASS) {
            if (size > 0 && size < PAYLOAD_GPS_SIZE) {
                gps_data[size] = '\0';
                if (strstr((char*)gps_data, "RMC")) {
                    ParseGPS_RMC((char*)gps_data);
                }
            }
        }
    }
}
