/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Torque + Current Measurement (HX711 + INA219 + Servo)
  ******************************************************************************
  */
/* USER CODE END Header */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name       = "defaultTask",
  .stack_size = 512 * 4,          // 2 KB  (was 128*4 = 512 B — too small for floats + snprintf)
  .priority   = (osPriority_t) osPriorityNormal,
};
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name       = "servoTask",
  .stack_size = 512 * 2,          // 2 KB  (was 128*4 = 512 B — too small for floats + snprintf)
  .priority   = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// ── HX711 GPIO pins ───────────────────────────────────────────────────────────
#define HX711_DOUT_PORT     GPIOA
#define HX711_DOUT_PIN      GPIO_PIN_0
#define HX711_SCK_PORT      GPIOA
#define HX711_SCK_PIN       GPIO_PIN_1

// ── Torque sensor specifications ──────────────────────────────────────────────
#define SENSITIVITY_MV_PER_V    2.0f
#define EXCITATION_V            5.0f
#define FULL_SCALE_NM           10.0f
#define HX711_GAIN              128.0f
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
#define INA219_RSHUNT_OHMS          0.1f
#define INA219_MAX_EXPECTED_A       3.2f
#define INA219_CURRENT_DEADBAND_A   0.02f

// ── Globals shared between main() initialisation and the RTOS task ────────────
static int32_t  hx_zero          = 0;
static float    COUNTS_PER_NM    = 0.0f;

static uint16_t ina219_cal_value     = 0;
static float    ina219_current_lsb_A = 0.0f;
static float    ina219_power_lsb_W   = 0.0f;
static float    ina219_zero_offset_A = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void ServoTask(void *argument);
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

/* USER CODE END 0 */

// ═════════════════════════════════════════════════════════════════════════════
//  main()  –  hardware init + tare ONLY, then hand off to RTOS
// ═════════════════════════════════════════════════════════════════════════════
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    char msg[160];

    // Pre-compute scale factor (stored in global)
    COUNTS_PER_NM = (HX711_COUNTS * HX711_GAIN
                     * SENSITIVITY_MV_PER_V * EXCITATION_V)
                    / (HX711_VREF_MV * 2.0f * FULL_SCALE_NM);

    // Start servo PWM and go to neutral
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

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

    // ── Start RTOS ────────────────────────────────────────────────────────────
    osKernelInitialize();
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    servoTaskHandle = osThreadNew(ServoTask, NULL, &servoTask_attributes);
    osKernelStart();

    // Never reached
    while (1) {

    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  RTOS default task  –  measurement loop runs here
// ═════════════════════════════════════════════════════════════════════════════
void ServoTask(void *argument){
	for(;;){
		Servo_SetPulse(SERVO_PULSE_MIN_US);
		osDelay(500);
		Servo_SetPulse(SERVO_PULSE_MID_US);
		osDelay(500);
		Servo_SetPulse(SERVO_PULSE_MAX_US);
		osDelay(500);
	}
}
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN 5 */
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
                     "T=%7.4f Nm | I=%7.4f A | V=%6.3f V\r\n",
                     torque, current_A, bus_V);
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
    /* USER CODE END 5 */
}

// ═════════════════════════════════════════════════════════════════════════════
//  Clock + Peripheral Init (unchanged from CubeMX generation)
// ═════════════════════════════════════════════════════════════════════════════
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 83;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 19999;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // SCK output — start LOW
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    // PA0 — DOUT input (HX711 data)
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA1 — SCK output (HX711 clock)
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA4 — analog (unused / ADC placeholder)
    GPIO_InitStruct.Pin  = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
