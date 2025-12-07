/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - RECEIVER NRF24L01+
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf.h"
#include <stdio.h>
#include <string.h>
#include "ina219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RF_CHANNEL      76
#define DATA_RATE       0
#define TX_POWER        3
#define PAYLOAD_SIZE    32
#define TIMEOUT 5000
#define LED         10
#define PHARE       20
#define SERVO_DROIT 30
#define SERVO_GAUCHE 40
#define DEAD_ZONE 50
#define MID_LEFT_RIGHT 430
#define DEBOUNCE_TIME_MS 200
#define PWM_MIN 1000
#define PWM_MAX 1300
#define PWM_NEUTRAL 1150 // Milieu de la plage de vitesse
#define DEADZONE 3
#define JOY_MIN 24
#define JOY_MAX 62
#define JOY_CENTER 43
#define DEADZONE_MIN        (JOY_CENTER - JOY_CENTER) // 120
#define DEADZONE_MAX        (JOY_CENTER + JOY_CENTER) // 134

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_address[5] = {0xE6, 0xE7, 0xE7, 0xE7, 0xE7};
uint32_t packets_received = 0;
uint32_t last_received_time = 0;
volatile uint8_t servo_angle_gauche = 180;
int minimum_servo_gauche = 90;
volatile uint8_t servo_angle_droit = 0;
INA219_HandleTypedef ina219_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Servo_SetAngleGauche(uint8_t angle)
{
    // Conversion de l'angle (0–180°) en largeur d'impulsion (500–2500 µs)
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

void Servo_SetAngleHam(uint8_t angle)
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

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

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

uint16_t map_joy_to_pwm(uint8_t joy_y_value)
{

    const int32_t PWM_RANGE = PWM_MAX - PWM_MIN;
    int32_t output = ( (int32_t)joy_y_value - JOY_MIN ) * PWM_RANGE;
    output /= (JOY_MAX - JOY_MIN);
    output += PWM_MIN;
    return (uint16_t)output;
}

int16_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    return (int16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void generate_motor_outputs(uint8_t joy_x_value, uint8_t joy_y_value, uint16_t *pwm_g, uint16_t *pwm_d)
{
    const uint8_t JOY_CENTER_X = 24;
    const uint8_t JOY_CENTER_Y = 15;
    const uint16_t PWM_STOP = 1000;
    const uint16_t PWM_MAX_THR = 1300;
    const int16_t STEERING_FORCE = 150;

    int16_t throttle = PWM_STOP;
    int16_t steering = 0;

    if (joy_y_value > (JOY_CENTER_Y + DEADZONE))
    {
        throttle = map_value(joy_y_value, (JOY_CENTER_Y + DEADZONE), JOY_MAX, PWM_STOP, PWM_MAX_THR);
    }

    if (joy_x_value > (JOY_CENTER_X + DEADZONE))
    {

        steering = map_value(joy_x_value, (JOY_CENTER_X + DEADZONE), JOY_MAX, 0, STEERING_FORCE);
    }
    else if (joy_x_value < (JOY_CENTER_X - DEADZONE))
    {

        steering = map_value(joy_x_value, JOY_MIN, (JOY_CENTER_X - DEADZONE), -STEERING_FORCE, 0);
    }

    if (throttle == PWM_STOP) {
        steering = 0;
    }

    int32_t motor_g_raw = (int32_t)throttle + steering; // Moteur Gauche
    int32_t motor_d_raw = (int32_t)throttle - steering; // Moteur Droit

    if (motor_g_raw > PWM_MAX_THR) motor_g_raw = PWM_MAX_THR;
    if (motor_g_raw < PWM_STOP) motor_g_raw = PWM_STOP;

    if (motor_d_raw > PWM_MAX_THR) motor_d_raw = PWM_MAX_THR;
    if (motor_d_raw < PWM_STOP) motor_d_raw = PWM_STOP;

    *pwm_g = (uint16_t)motor_g_raw;
    *pwm_d = (uint16_t)motor_d_raw;
}

void Handle_Joystick(uint8_t x, uint8_t y){
	uint16_t pwm = map_joy_to_pwm(y);
	ESC_SetThrottle_D(pwm);
	ESC_SetThrottle_G(pwm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (INA219_Init(&ina219_sensor, &hi2c1, 0.1f, 3.2f) != HAL_OK) {
	  printf("BUG");
      }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  ESC_SetThrottle_G(1000);
  ESC_SetThrottle_D(1000);
  HAL_Delay(2000);
  Servo_SetAngleGauche(servo_angle_gauche);  // Position initiale 180°
  Servo_SetAngleDroit(servo_angle_droit);    // Position initiale 0°
  Servo_SetAngleHam(180);
  printf("\r\n========================================\r\n");
  printf("     NRF24L01+ - MODE RECEIVER\r\n");
  printf("========================================\r\n");
  HAL_Delay(500);
  // Initialisation NRF24
  printf("Initialisation NRF24...\r\n");
  if (nrf24_init(RF_CHANNEL, DATA_RATE, TX_POWER) != 0) {
      printf("ERROR INIT NRF24\r\n");
      Error_Handler();
  }
  HAL_Delay(100);
  // Configuration RX Addresses
  nrf24_set_rx_address(rx_address, 0);

  // Starting listening
  nrf24_start_listening();

  // ✅ DIAGNOSTICS COMPLETS
  uint8_t config, status, fifo, en_aa, en_rxaddr;
  uint8_t rx_addr_p0[5];


  nrf24_read_register(NRF24_CONFIG, &config, 1);
  nrf24_read_register(NRF24_STATUS, &status, 1);
  nrf24_read_register(NRF24_FIFO_STATUS, &fifo, 1);
  nrf24_read_register(NRF24_EN_AA, &en_aa, 1);
  nrf24_read_register(NRF24_EN_RXADDR, &en_rxaddr, 1);
  nrf24_read_register(NRF24_RX_ADDR_P0, rx_addr_p0, 5);

  printf("\n=== DIAGNOSTIC RECEPTEUR ===\r\n");
  printf("CONFIG: 0x%02X ", config);
  if (config & 0x01) printf("(RX mode OK)\r\n");
  else printf("(ERROR: TX mode!)\r\n");

  printf("STATUS: 0x%02X\r\n", status);
  printf("FIFO_STATUS: 0x%02X\r\n", fifo);
  printf("EN_AA: 0x%02X\r\n", en_aa);
  printf("EN_RXADDR: 0x%02X\r\n", en_rxaddr);

  printf("RX_ADDR_P0: %02X:%02X:%02X:%02X:%02X\r\n",
         rx_addr_p0[0], rx_addr_p0[1], rx_addr_p0[2],
         rx_addr_p0[3], rx_addr_p0[4]);

  printf("Adresse attendue: %02X:%02X:%02X:%02X:%02X\r\n",
         rx_address[0], rx_address[1], rx_address[2],
         rx_address[3], rx_address[4]);

  // Vérifier CE
  if (HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin) == GPIO_PIN_SET) {
      printf("CE: HIGH (OK)\r\n");
  } else {
      printf("CE: LOW (ERROR!)\r\n");
      printf("Forcing CE HIGH...\r\n");
      HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
  }

  printf("RF CHANNEL: %d\r\n", RF_CHANNEL);
  printf("========================================\r\n");
  printf("Waiting data..\r\n\r\n");

  last_received_time = HAL_GetTick();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Looking for available data

	  float Vbus = INA219_GetBusVoltage_V(&ina219_sensor);
	          float I_load = INA219_GetCurrent_A(&ina219_sensor);
	          float P_load = INA219_GetPower_W(&ina219_sensor);
	          float Vshunt = INA219_GetShuntVoltage_V(&ina219_sensor);
	          printf("VBus: %.2f V, I: %.2f A, P: %.2f W\r\n", Vbus, I_load, P_load);
	  static uint32_t last_status_check = 0;
	  if (HAL_GetTick() - last_status_check > 2000) {
	      uint8_t status_check, fifo_check;
	      nrf24_read_register(NRF24_STATUS, &status_check, 1);
	      nrf24_read_register(NRF24_FIFO_STATUS, &fifo_check, 1);

	      printf("[DEBUG] STATUS: 0x%02X | FIFO: 0x%02X | CE: %d\r\n",
	             status_check, fifo_check,
	             HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin));

	      last_status_check = HAL_GetTick();
	  }

	  // Looking for available data
	  if (nrf24_available()) {
	      uint8_t buffer[PAYLOAD_SIZE];
	      memset(buffer, 0, PAYLOAD_SIZE);

	      if (nrf24_read(buffer, PAYLOAD_SIZE)) {

	          if (buffer[0]==0xBB || buffer[0]==0xBA) {
	        	  printf("RX HEADER: 0x%02X | DATA: 0x%02X\r\n", buffer[0], buffer[1]);
	              packets_received++;
	              uint32_t current_time = HAL_GetTick();
	              uint32_t time_since_last = current_time - last_received_time;
	              last_received_time = current_time;
	              printf("RX: 0x%02X | CMD: 0x%02X | Next: 0x%02X\r\n",
	                                 buffer[0], buffer[1], buffer[2]);
	              uint8_t commande_recue = buffer[1] & 0xFE;
	              process_command(commande_recue);

	          }

	          else if (buffer[0] == 0xAA) {
	        	  printf("RX HEADER: 0x%02X | DATA: 0x%02X\r\n", buffer[0], buffer[1]);
	                  uint8_t val_x = buffer[1];
	                  uint8_t val_y = buffer[2];
	                  uint16_t pwm_motor_g, pwm_motor_d;

	                  generate_motor_outputs(val_x, val_y, &pwm_motor_g, &pwm_motor_d);
	                  //Handle_Joystick(val_x, val_y);
	                  ESC_SetThrottle_D(pwm_motor_g);
	                  ESC_SetThrottle_G(pwm_motor_d);
	                  printf("Joystick -> X: %d | Y: %d\r\n", val_x, val_y);



	              }
	      }

	  }

	  // Timeout if nothing is coming for 5 seconds
	  if ((HAL_GetTick() - last_received_time) > TIMEOUT) {
	      printf("[WARNING] No data receive in 5sec\r\n");
	      printf("  Total receive: %lu packets\r\n\r\n", packets_received);
	      last_received_time = HAL_GetTick();
	  }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, x_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PHARE_GPIO_Port, PHARE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : x_Pin LED_Pin */
  GPIO_InitStruct.Pin = x_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PHARE_Pin */
  GPIO_InitStruct.Pin = PHARE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PHARE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CSN_Pin NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin|NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
