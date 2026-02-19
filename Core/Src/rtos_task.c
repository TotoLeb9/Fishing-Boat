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
volatile uint32_t bData = 0;
volatile uint8_t homeReturn = 0;
volatile uint16_t adc_buffer[10];
uint8_t result;
uint8_t payload[PAYLOAD_SIZE];
uint8_t timer_home = 0;
int minimum_servo_gauche = 90;

KalmanCap_t kalman;
KalmanConfig_t kalmanConfig;

/*osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};*/

osThreadId_t ListeningNrfHandle;
const osThreadAttr_t listener_attr = {
    .name = "ListeningNRF",
    .priority =  osPriorityHigh,
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
		.stack_size = 256 * 4
};

osThreadId_t BatteryHandle;
const osThreadAttr_t battery_attributes = {
		.name = "BatteryTask",
		.priority = osPriorityNormal,
		.stack_size = 512 * 4
};

osThreadId_t ReturnHomeHandle;
const osThreadAttr_t returnHome_attributes = {
		.name = "ReturnToHomeTask",
		.priority = osPriorityNormal,
		.stack_size = 512 * 4
};

const osMutexAttr_t nrfMutex_attributes = {
    .name = "nrfMutex"
};
QueueHandle_t joyQueue = NULL;
int packet = 0;
osMutexId_t nrfMutex;

float GetAngularSpeed(float lHeading, float nHeading, TickType_t lTime, TickType_t nTime) {
    float diffHeading = nHeading - lHeading;
    if (diffHeading > 180.0f)  diffHeading -= 360.0f;
    if (diffHeading < -180.0f) diffHeading += 360.0f;
    float dt = (float)(nTime - lTime) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f) return 0.0f;
    return diffHeading / dt;
}

/*void StartDefaultTask(void *argument)
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
    osThreadFlagsSet(CompassHandle , START_FLAG);
    osThreadFlagsSet(GpsHandle , START_FLAG);
    osThreadFlagsSet(BatteryHandle, START_FLAG);
    LOG_INFO("Toutes les tâches démarrées\r\n");

    for(;;)
    {
        osDelay(1000);
    }
}
*/
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
                    if (servo_angle_droit < 90) servo_angle_droit += 15;
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

    TickType_t lastTime = xTaskGetTickCount();
    TickType_t nowTime;
    int init_attempts = 0;

    while(QMC5883P_ReadXYZ(&mag, &magData) != QMC_OK) {
        osDelay(10);
        if(++init_attempts > 100) break;
    }
    float start_heading = QMC5883P_GetHeadingDeg(&mag, 0.0f) + DECLINATION_FRANCE;
    KalmanConfigInit(&kalmanConfig, start_heading, 5.0f, 1.0f, 0.01f, 0.1f, 2.0f, 1.0f, 0.95f);
    KalmanInitWithConfig(&kalman, &kalmanConfig);

    for(;;){
        nowTime = xTaskGetTickCount();
        float dt = (float)(nowTime - lastTime) / 1000.0f;
        if (dt <= 0.0f) dt = 0.05f;

        if(QMC5883P_ReadXYZ(&mag, &magData) == QMC_OK){
            float rawHeading = QMC5883P_GetHeadingDeg(&mag, 0.0f) + DECLINATION_FRANCE;
            KalmanPredict(&kalman, dt);
            kalman.compass.cap_boussole = rawHeading;
            KalmanUpdate(&kalman);
        }

        lastTime = nowTime;
        osDelay(50);
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
                    osDelay(5);
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
    memset(&pos_depart, 0, sizeof(pos_depart));
    bool home_is_set = false;

    osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_data, PAYLOAD_GPS_SIZE);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    LOG_INFO("GPS START\r\n");

    for(;;) {
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &size, pdMS_TO_TICKS(2000)) == pdPASS) {

            if (size > 0 && size < PAYLOAD_GPS_SIZE) {
                gps_data[size] = '\0';
                if (strstr((char*)gps_data, "$GPRMC") || strstr((char*)gps_data, "$GNRMC")) {
                    ParseGPS_RMC((char*)gps_data);
                    KalmanUpdateGps(&kalman);
                    printf("%s\n\r",gps_data);
                    if(!home_is_set && gpsStructData.latitude != 0.0f) {
                        pos_depart.latitude = gpsStructData.latitude;
                        pos_depart.longitude = gpsStructData.longitude;
                        home_is_set = true;
                        LOG_INFO("HOME SET: %.6f, %.6f\r\n",
                                 pos_depart.latitude, pos_depart.longitude);
                    }
                }
            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_data, PAYLOAD_GPS_SIZE);
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

        } else {
            LOG_INFO("GPS timeout, restart DMA\r\n");
            HAL_UART_AbortReceive(&huart4);
            memset(gps_data, 0, PAYLOAD_GPS_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_data, PAYLOAD_GPS_SIZE);
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
        }

        osDelay(10);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(BatteryHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
			ESC_SetThrottle_D(1500); //Passage au neutre pour éviter que le bateau avance tout seul
			ESC_SetThrottle_G(1500);
			nrfConnected = 0;
			timer_home++;
			if(timer_home>TIME_BEFORE_HOME && !homeReturn){
				xTaskNotifyGive(ReturnHomeHandle);
				homeReturn = 1;
				timer_home=0;
			}
		}
		else
		{
			timer_home = 0;
			nrfConnected = 1;
			homeReturn = 0;
		}

	}
}

void BatteryTask(void *argument){
	osThreadFlagsWait(START_FLAG, osFlagsWaitAny, osWaitForever);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUMBER_CAPTURE);
    for(;;){
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == pdPASS) {
            uint32_t sum = 0;
            uint32_t mean = 0;
            float voltage_tmp = 0;
            float current_tmp = 0;
            for(int i = 0; i < NUMBER_CAPTURE ; i++) sum += adc_buffer[i];
            mean = (sum/NUMBER_CAPTURE);
            voltage_tmp = ADC_To_Voltage(mean);
            current_tmp = Voltage_To_Current(voltage_tmp);
            float v_sensor = voltage_tmp / 0.6644f;
            printf("COURANT = %f VOLTAGE = %f SENSOR = %f\n\r", current_tmp,voltage_tmp,v_sensor);

        }
        else {
            if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_BUSY_REG) {
                HAL_ADC_Stop_DMA(&hadc1);
                HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUMBER_CAPTURE);
            }
        }
        osDelay(50);
    }
}

void ReturnToHomeTask(void *argument){
	for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		while(homeReturn){
			osDelay(50);
			LOG_INFO("ANGLE : %f | VITESSE : %f\n\r",kalman.cap, kalman.vitesse_angulaire);
		}
	}
}
