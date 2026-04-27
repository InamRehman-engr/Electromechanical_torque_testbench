/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Torque + Current Measurement (HX711 + INA219 + Servo)
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
    .name       = "sensorTask",
    .stack_size = 512 * 4,                          // 2 KB — needs snprintf + floats
    .priority   = (osPriority_t) osPriorityNormal,
};
/* Servo task — sweeps servo continuously */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
    .name       = "servoTask",                      // MUST differ from sensorTask name
    .stack_size = 128 * 4,                          // 512 B — only GPIO writes, very small
    .priority   = (osPriority_t) osPriorityBelowNormal,
};
osThreadId_t relayTaskHandle;
const osThreadAttr_t relayTask_attributes = {
    .name       = "relayTask",                      // MUST differ from sensorTask name
    .stack_size = 128 * 4,                          // 512 B — only GPIO writes, very small
    .priority   = (osPriority_t) osPriorityBelowNormal,
};
osThreadId_t pwmTaskHandle;
const osThreadAttr_t pwmTask_attributes = {
    .name       = "pwmTask",                      // MUST differ from sensorTask name
    .stack_size = 128 * 4,                          // 512 B — only GPIO writes, very small
    .priority   = (osPriority_t) osPriorityBelowNormal,
};

osMutexId_t         i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes  = { .name = "i2cMutex"  };

osMutexId_t         uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = { .name = "uartMutex" };

// ── HX711 GPIO pins ───────────────────────────────────────────────────────────
#define HX711_DOUT_PORT     GPIOA
#define HX711_DOUT_PIN      GPIO_PIN_0
#define HX711_SCK_PORT      GPIOA
#define HX711_SCK_PIN       GPIO_PIN_1

// ── Torque sensor specifications ──────────────────────────────────────────────
#define SENSITIVITY_MV_PER_V    2.0f
#define EXCITATION_V            5.0f
#define FULL_SCALE_NM           10.0f
#define HX711_GAIN              512.0f
#define HX711_VREF_MV           (EXCITATION_V * 1000.0f)
#define HX711_COUNTS            8388608.0f
#define TORQUE_DEADBAND_NM      0.05f

// ── Servo pulse calibration ───────────────────────────────────────────────────
#define SERVO_PULSE_MIN_US      1000
#define SERVO_PULSE_MAX_US      2000
#define SERVO_PULSE_MID_US      1500

// ── INA219 register / config ──────────────────────────────────────────────────
#define INA219_ADDR                 (0x40 << 1)
#define INA219_REG_CONFIG           0x00
#define INA219_REG_SHUNT_VOLTAGE    0x01
#define INA219_REG_BUS_VOLTAGE      0x02
#define INA219_REG_POWER            0x03
#define INA219_REG_CURRENT          0x04
#define INA219_REG_CALIBRATION      0x05
#define INA219_CONFIG_VALUE         0x399F   // 32 V, 320 mV shunt, 12-bit, continuous
#define INA219_RSHUNT_OHMS          0.0319f
#define INA219_MAX_EXPECTED_A       3.2f
#define INA219_CURRENT_DEADBAND_A   0.02f

// ── Globals shared between main() initialisation and the RTOS task ────────────
static int32_t  hx_zero          = 0;
static float    COUNTS_PER_NM    = 0.0f;

static uint16_t ina219_cal_value     = 0;
static float    ina219_current_lsb_A = 0.0f;
static float    ina219_power_lsb_W   = 0.0f;
static float    ina219_zero_offset_A = 0.0f;

uint16_t pinB[4]={GPIO_PIN_3,GPIO_PIN_5,GPIO_PIN_4,GPIO_PIN_10};
uint16_t pinA[2]={GPIO_PIN_10,GPIO_PIN_8};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void SensorTask(void *argument);
void ServoTask(void *argument);
void RelayTask(void *argument);
void PWMTask(void *argument);
void Set_PWM(uint8_t pwm_value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ═════════════════════════════════════════════════════════════════════════════
//  SERVO
// ═════════════════════════════════════════════════════════════════════════════
void Servo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulse = SERVO_PULSE_MIN_US
                   + ((uint32_t)angle
                   * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / 180;
    Servo_SetPulse(pulse);
}

void Set_PWM(uint8_t pwm_value)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
}

// ═════════════════════════════════════════════════════════════════════════════
//  HX711
// ═════════════════════════════════════════════════════════════════════════════
void HX711_WaitReady(void)
{
    uint32_t timeout = 1000;
    while (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET)
    {
        HAL_Delay(1);
        if (--timeout == 0) return;
    }
}

int32_t HX711_Read(void)
{
    HX711_WaitReady();
    uint32_t raw = 0;

    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        raw = (raw << 1) | HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN);
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }

    // 25th pulse → Channel A, Gain 128 for NEXT reading
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

    // Sign-extend 24-bit two's complement → int32
    if (raw & 0x800000)
        raw |= 0xFF000000;

    return (int32_t)raw;
}

int32_t HX711_ReadAverage(uint8_t samples)
{
    int64_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
        sum += HX711_Read();
    return (int32_t)(sum / samples);
}

// ═════════════════════════════════════════════════════════════════════════════
//  INA219
// ═════════════════════════════════════════════════════════════════════════════
HAL_StatusTypeDef INA219_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;
    data[1] =  value       & 0xFF;
    return HAL_I2C_Mem_Write(&hi2c1, INA219_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef INA219_ReadReg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    *value = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

HAL_StatusTypeDef INA219_Init(void)
{
    ina219_current_lsb_A = INA219_MAX_EXPECTED_A / 32768.0f;
    ina219_power_lsb_W   = 20.0f * ina219_current_lsb_A;
    ina219_cal_value     = (uint16_t)(0.04096f /
                            (ina219_current_lsb_A * INA219_RSHUNT_OHMS));

    if (INA219_WriteReg(INA219_REG_CALIBRATION, ina219_cal_value) != HAL_OK) return HAL_ERROR;
    if (INA219_WriteReg(INA219_REG_CONFIG,      INA219_CONFIG_VALUE) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10);
    return HAL_OK;
}

float INA219_ReadCurrent_A(void)
{
    uint16_t reg;
    if (INA219_WriteReg(INA219_REG_CALIBRATION, ina219_cal_value) != HAL_OK) return NAN;
    if (INA219_ReadReg(INA219_REG_CURRENT, &reg)                  != HAL_OK) return NAN;
    return (int16_t)reg * ina219_current_lsb_A;
}

float INA219_ReadBusVoltage_V(void)
{
    uint16_t reg;
    if (INA219_ReadReg(INA219_REG_BUS_VOLTAGE, &reg) != HAL_OK) return NAN;
    reg >>= 3;
    return reg * 0.004f;
}

float INA219_TareCurrent(uint8_t samples)
{
    float   sum   = 0.0f;
    uint8_t valid = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        float val = INA219_ReadCurrent_A();
        if (!isnan(val)) { sum += val; valid++; }
        HAL_Delay(10);
    }
    if (valid == 0) return 0.0f;
    ina219_zero_offset_A = sum / valid;
    return ina219_zero_offset_A;
}

void Setpin(GPIO_TypeDef *GPIOx, uint16_t pin){
	HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);    // PA1 HIGH
}
void Resetpin(GPIO_TypeDef *GPIOx, uint16_t pin){
	HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);    // PA1 HIGH
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
  Resetpin(GPIOA,pinA[1]);
  	HAL_Delay(100);
  	for(int i=3;i>=0;i--){
  		Resetpin(GPIOB,pinB[i]);
  		HAL_Delay(100);
  	}
  	Resetpin(GPIOA,pinA[0]);
  	HAL_Delay(100);
      char msg[160];
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


    Setpin(GPIOB,pinB[0]); //RST
    HAL_Delay(1000);
    Resetpin(GPIOB,pinB[0]); // RST
    HAL_Delay(1000);
    Setpin(GPIOB,pinB[3]); // Torque
    HAL_Delay(1000);
    Setpin(GPIOB,pinB[1]); // RUN
    HAL_Delay(1000);
    Setpin(GPIOB,pinB[2]); //FWD
    HAL_Delay(2000);
    Resetpin(GPIOB,pinB[2]); //FWD
    HAL_Delay(1000);
    Setpin(GPIOA,pinA[0]); //REV
    HAL_Delay(2000);
    Resetpin(GPIOA,pinA[0]); //REV
    HAL_Delay(2000);




    // Pre-compute scale factor (stored in global)
    COUNTS_PER_NM = (HX711_COUNTS * HX711_GAIN
                     * SENSITIVITY_MV_PER_V * EXCITATION_V)
                    / (HX711_VREF_MV * 2.0f * FULL_SCALE_NM);

    // Start servo PWM and go to neutral
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_SetPulse(SERVO_PULSE_MIN_US);
    HAL_Delay(1000);
    Servo_SetPulse(SERVO_PULSE_MID_US);
    HAL_Delay(1000);
    Servo_SetPulse(SERVO_PULSE_MAX_US);
    HAL_Delay(1000);
    // Tare HX711 (store in global)
    hx_zero = HX711_ReadAverage(32);

    snprintf(msg, sizeof(msg),
             "=== SYSTEM READY ===\r\nZero offset = %ld\r\nCounts/Nm   = %.2f\r\n",
             hx_zero, COUNTS_PER_NM);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Init INA219 (store calibration in globals)
    if (INA219_Init() == HAL_OK)
    {
        float zero = INA219_TareCurrent(2);
        snprintf(msg, sizeof(msg),
                 "INA219 READY | CAL=%u | Current_LSB=%.8f A/bit | Zero=%.6f A\r\n",
                 ina219_cal_value, ina219_current_lsb_A, zero);
    }
    else
    {
        snprintf(msg, sizeof(msg), "INA219 INIT FAILED\r\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  i2cMutexHandle  = osMutexNew(&i2cMutex_attributes);   // 2. Create mutexes
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  /* Create mutexes BEFORE creating tasks */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  servoTaskHandle  = osThreadNew(ServoTask,  NULL, &servoTask_attributes);
//  relayTaskHandle  = osThreadNew(RelayTask,  NULL, &relayTask_attributes);
  pwmTaskHandle  = osThreadNew(PWMTask,  NULL, &pwmTask_attributes);


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
  /* USER CODE BEGIN 5 */

void PWMTask(void * argument){
	for(;;){
    for (uint8_t pwm = 0; pwm <= 250; pwm += 5)
		{
			Set_PWM(pwm);
			osDelay(100);
		}
		Set_PWM(255);   // ensure we hit 255 exactly
		osDelay(1000);

		for (int16_t pwm = 255; pwm >= 0; pwm -= 5)
		{
			Set_PWM((uint8_t)pwm);
			osDelay(100);
		}
		Set_PWM(0);     // ensure we hit 0 exactly
		osDelay(1000);
	}
}
void RelayTask(void *argument){

	for (;;){
	    Setpin(GPIOA,pinA[0]);
		osDelay(100);
		for(int i=0;i<4;i++){
			Setpin(GPIOB,pinB[i]);
			osDelay(100);
		}
		Setpin(GPIOA,pinA[1]);
		osDelay(100);

		Resetpin(GPIOA,pinA[0]);
		osDelay(100);
		for(int i=0;i<4;i++){
			Resetpin(GPIOB,pinB[i]);
			osDelay(100);
		}
		Resetpin(GPIOA,pinA[1]);
		osDelay(100);

	}

}

void ServoTask(void *argument)
  {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // ← once here

      for (;;){
            Servo_SetPulse(SERVO_PULSE_MIN_US);   // full CCW  (1000 µs)
			osDelay(500);
			Servo_SetPulse(SERVO_PULSE_MID_US);   // center    (1500 µs)
			osDelay(500);

			Servo_SetPulse(SERVO_PULSE_MAX_US);   // full CW   (2000 µs)
			osDelay(500);

			Servo_SetPulse(SERVO_PULSE_MID_US);   // back to center before repeating
			osDelay(500);
	  }
}

void SensorTask(void *argument)
{
   char  msg[160];
   float torque    = 0.0f;
   float current_A = 0.0f;
   float bus_V     = 0.0f;

   for (;;)
   {
       // ── HX711 torque reading ─────────────────────────────────────────────
       int32_t hx_raw = HX711_Read();
       int32_t hx_net = hx_raw - hx_zero;
       torque = (float)hx_net / COUNTS_PER_NM;

       if (fabsf(torque) < TORQUE_DEADBAND_NM)
           torque = 0.0f;

       // ── INA219 current + voltage reading ─────────────────────────────────
       float raw_A = INA219_ReadCurrent_A();
       if (!isnan(raw_A))
       {
           current_A = raw_A - ina219_zero_offset_A;
           if (fabsf(current_A) < INA219_CURRENT_DEADBAND_A)
               current_A = 0.0f;
       }
       else
       {
           current_A = NAN;
       }

       bus_V = INA219_ReadBusVoltage_V();

       // ── Transmit results over UART ────────────────────────────────────────
       if (!isnan(current_A) && !isnan(bus_V))
       {
           snprintf(msg, sizeof(msg),
                    "T=%7.4f Nm | I=%7.4f A | V=%6.3f V | hx_net=%ld | count_per_NM=%6.3f\r\n",
                    torque, current_A, bus_V, hx_net, COUNTS_PER_NM);
       }
       else
       {
           snprintf(msg, sizeof(msg),
                    "T=%7.4f Nm | INA219 READ ERROR\r\n",
                    torque);
       }

       HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

       osDelay(100);   // 10 Hz sample rate — adjust as needed
   }
}
  /* USER CODE END 5 */


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
