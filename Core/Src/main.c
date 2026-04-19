/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE BEGIN 0 */
#include <math.h>

// ── HX711 GPIO pins ───────────────────────────────────────────
#define HX711_DOUT_PORT     GPIOA
#define HX711_DOUT_PIN      GPIO_PIN_0
#define HX711_SCK_PORT      GPIOA
#define HX711_SCK_PIN       GPIO_PIN_1

// ── Torque sensor specifications ──────────────────────────────
#define SENSITIVITY_MV_PER_V    2.0f    // 2 mV/V from datasheet
#define EXCITATION_V            5.0f    // 5V excitation supply
#define FULL_SCALE_NM           10.0f   // 10 Nm max torque
#define HX711_GAIN              128.0f  // Channel A, Gain 128
#define HX711_VREF_MV           (EXCITATION_V * 1000.0f)  // 5000 mV
#define HX711_COUNTS            8388608.0f  // 2^23

// ── Torque noise floor (ignore readings below this) ───────────
#define TORQUE_DEADBAND_NM      0.05f   // 0.05 Nm deadband

// ── Servo pulse calibration ───────────────────────────────────
// Standard servo: 1000µs = 0°, 1500µs = 90°, 2000µs = 180°
// If your servo doesn't reach full angle, adjust these:
#define SERVO_PULSE_MIN_US      1000    // pulse for 0°   → increase if overshooting
#define SERVO_PULSE_MAX_US      2000    // pulse for 180° → decrease if overshooting
#define SERVO_PULSE_MID_US      1500    // pulse for 90°  (neutral)

// ── Servo functions ───────────────────────────────────────────
void Servo_SetPulse(uint16_t pulse_us)
{
    // Clamp to safe range
    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180) angle = 180;
    // Map 0-180° → SERVO_PULSE_MIN_US to SERVO_PULSE_MAX_US
    uint16_t pulse = SERVO_PULSE_MIN_US
                   + ((uint32_t)angle
                   * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / 180;
    Servo_SetPulse(pulse);
}

// ── HX711 functions ───────────────────────────────────────────
void HX711_WaitReady(void)
{
    // DOUT LOW means data is ready
    uint32_t timeout = 1000; // 1 second max wait
    while (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET)
    {
        HAL_Delay(1);
        if (--timeout == 0) return; // prevent infinite hang
    }
}

int32_t HX711_Read(void)
{
    HX711_WaitReady();

    uint32_t raw = 0;

    // Clock in 24 bits, MSB first
    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        raw = (raw << 1) | HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN);
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }

    // 25th clock pulse → selects Channel A, Gain 128 for NEXT reading
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  char    msg[140];
  int32_t hx_zero        = 0;
  int32_t hx_raw         = 0;
  int32_t hx_net         = 0;
  float   torque         = 0.0f;

  // ── Precompute scale factor (done ONCE, not in loop) ──────────
  // HX711 with Gain 128, 5V excitation, 2mV/V sensor:
  // Full scale sensor output = 2mV/V × 5V = 10mV
  // HX711 input range at Gain 128 = ±20mV
  // So 10mV sensor FSO maps to: 8388608 * (10/20) = 4194304 counts
  // counts_per_nm = 4194304 / 10 Nm = 419430.4 counts/Nm
  //
  // Formula:
  // counts_per_nm = (HX711_COUNTS * GAIN * SENSITIVITY_MV_PER_V * EXCITATION_V)
  //               / (HX711_VREF_MV * 2.0 * FULL_SCALE_NM)
  // HX711 Vref differential full scale = ±(Vcc/2) internally, effective ±20mV at gain 128
  const float COUNTS_PER_NM = (HX711_COUNTS * HX711_GAIN
                                * SENSITIVITY_MV_PER_V * EXCITATION_V)
                               / (HX711_VREF_MV * 2.0f * FULL_SCALE_NM);

  // Start servo and PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  Servo_SetPulse(SERVO_PULSE_MID_US);   // Start at 90° / neutral
  HAL_Delay(1000);                       // Wait for servo to reach position

  // ── Zero / Tare (no load applied at startup) ──────────────────
  hx_zero = HX711_ReadAverage(32);      // 32 samples for stable zero

  snprintf(msg, sizeof(msg),
           "=== SYSTEM READY ===\r\n"
           "Zero offset = %ld\r\n"
           "Counts/Nm   = %.2f\r\n",
           hx_zero, COUNTS_PER_NM);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ── Step 1: Move servo through positions ─────────────────
      Servo_SetAngle(0);
      HAL_Delay(1000);


      Servo_SetAngle(90);
      HAL_Delay(1000);

      Servo_SetAngle(180);
      HAL_Delay(1000);

      // ── Step 2: Settle then read HX711 ───────────────────────
      Servo_SetAngle(90);               // Return to neutral before reading
      HAL_Delay(500);                   // Wait for mechanical settling

      hx_raw = HX711_ReadAverage(16);   // 16 samples = good noise reduction
      hx_net = hx_raw - hx_zero;

      // ── Step 3: Compute torque ────────────────────────────────
      torque = (float)hx_net / COUNTS_PER_NM;

      // Apply deadband — zero out tiny noise readings
      if (fabsf(torque) < TORQUE_DEADBAND_NM)
          torque = 0.0f;

      // Clamp to sensor physical range
      if (torque >  FULL_SCALE_NM) torque =  FULL_SCALE_NM;
      if (torque < -FULL_SCALE_NM) torque = -FULL_SCALE_NM;

      // ── Step 4: Transmit result ───────────────────────────────
      snprintf(msg, sizeof(msg),
               "RAW=%ld | Zero=%ld | Net=%ld | Torque=%+.4f Nm\r\n",
               hx_raw, hx_zero, hx_net, torque);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
