/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX            4095.0f
#define ADC_VREF           3.3f    // STM32 Vref
#define SENSOR_MAX_VOLTAGE 5.0f    // AWM700 output at full-scale (change if 3.3V model)
#define SENSOR_MAX_FLOW    200.0f  // e.g., 200 SLPM (set as per your sensor model)

#define FLOW_THRESHOLD     30.0f   // user set setpoint (SLPM) - change as required
#define HYSTERESIS         2.0f    // SLPM hysteresis

// PWM mapping: we'll use a 0..1000 scale for duty control
#define PWM_SCALE_MAX      1000.0f
#define EXTEND_PWM_DUTY    700.0f  // 70% duty for extend
#define RETRACT_PWM_DUTY   700.0f  // 70% duty for retract

// Timeout safety (ms)
#define MAX_EXTEND_TIME_MS 10000UL // 10 s maximum extend time
#define MAX_RETRACT_TIME_MS 10000UL // 10 s maximum retract time

// ADC smoothing
#define FLOW_SAMPLES       8

/* ==== Pin definitions (match CubeMX) ==== */
#define DIR_Pin GPIO_PIN_0
#define DIR_GPIO_Port GPIOB

#define LIMIT_EXT_Pin GPIO_PIN_1
#define LIMIT_EXT_GPIO_Port GPIOB

#define LIMIT_RET_Pin GPIO_PIN_2
#define LIMIT_RET_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* ===== Types & globals ===== */
typedef enum {
  ST_CLOSED = 0,
  ST_OPENING,
  ST_OPEN,
  ST_CLOSING,
  ST_FAULT
} system_state_t;

volatile system_state_t sys_state = ST_CLOSED;
volatile float measured_flow = 0.0f;   // SLPM
float flow_history[FLOW_SAMPLES];
uint8_t flow_hist_idx = 0;
uint8_t flow_hist_cnt = 0;

/* Simple millis helper using HAL tick */
static inline uint32_t now_ms(void) { return HAL_GetTick(); }

/* ===== Forward declarations ===== */
float read_flow_adc_to_slpm(void);
void update_flow_filter(float sample);
void control_state_machine(void);
void actuator_extend_start(void);
void actuator_retract_start(void);
void actuator_stop(void);
bool is_limit_ext_pressed(void);
bool is_limit_ret_pressed(void);
void set_pwm_duty_percent(float duty_percent); // 0..100
void safety_fault(const char *msg);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float read_flow_adc_to_slpm(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return -1.0f; // error
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // Convert ADC -> voltage (assuming ADC Vref = ADC_VREF)
    float voltage = ((float)raw / ADC_MAX) * ADC_VREF;

    // If sensor output is up to SENSOR_MAX_VOLTAGE (> ADC_VREF), we assume a voltage divider is used.
    // Determine ratio R = ADC_VREF / SENSOR_MAX_VOLTAGE in wiring and scale accordingly.
    float scale_ratio = SENSOR_MAX_VOLTAGE > ADC_VREF ? (SENSOR_MAX_VOLTAGE / ADC_VREF) : 1.0f;
    float sensor_voltage = voltage * scale_ratio;

    // Map sensor_voltage (0..SENSOR_MAX_VOLTAGE) to flow (0..SENSOR_MAX_FLOW)
    if (sensor_voltage < 0.0f) sensor_voltage = 0.0f;
    if (sensor_voltage > SENSOR_MAX_VOLTAGE) sensor_voltage = SENSOR_MAX_VOLTAGE;

    float flow = (sensor_voltage / SENSOR_MAX_VOLTAGE) * SENSOR_MAX_FLOW;
    return flow;
}

void update_flow_filter(float sample)
{
    flow_history[flow_hist_idx++] = sample;
    if (flow_hist_idx >= FLOW_SAMPLES) flow_hist_idx = 0;
    if (flow_hist_cnt < FLOW_SAMPLES) flow_hist_cnt++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < flow_hist_cnt; ++i) sum += flow_history[i];
    measured_flow = sum / (float)flow_hist_cnt;
}

bool is_limit_ext_pressed(void)
{
#ifdef LIMIT_EXT_Pin
    return (HAL_GPIO_ReadPin(LIMIT_EXT_GPIO_Port, LIMIT_EXT_Pin) == GPIO_PIN_RESET);
#else
    return false; // not present
#endif
}

bool is_limit_ret_pressed(void)
{
#ifdef LIMIT_RET_Pin
    return (HAL_GPIO_ReadPin(LIMIT_RET_GPIO_Port, LIMIT_RET_Pin) == GPIO_PIN_RESET);
#else
    return false;
#endif
}

void set_pwm_duty_percent(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // Assuming TIM1 configured with ARR = pwm_period (e.g., 1000)
    uint32_t arr = htim1.Instance->ARR;
    uint32_t cc = (uint32_t)((duty_percent / 100.0f) * (float)arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cc);
}

void actuator_extend_start(void)
{
    // direction: HIGH = extend (adjust depending wiring)
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    set_pwm_duty_percent((EXTEND_PWM_DUTY / PWM_SCALE_MAX) * 100.0f);
    // ensure TIM PWM is running
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void actuator_retract_start(void)
{
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    set_pwm_duty_percent((RETRACT_PWM_DUTY / PWM_SCALE_MAX) * 100.0f);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void actuator_stop(void)
{
    set_pwm_duty_percent(0.0f);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/* fault handler */
void safety_fault(const char *msg)
{
    actuator_stop();
    sys_state = ST_FAULT;
    // Optionally blink LED or log message via UART
    // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void control_state_machine(void)
{
    static uint32_t action_start_ts = 0;

    switch (sys_state)
    {
        case ST_CLOSED:
        {
            // if flow exceeds threshold + hysteresis => open valve
            if (measured_flow >= (FLOW_THRESHOLD + HYSTERESIS))
            {
                action_start_ts = now_ms();
                actuator_extend_start();
                sys_state = ST_OPENING;
            }
            break;
        }

        case ST_OPENING:
        {
            // stop if limit switch triggered (fully extended)
            if (is_limit_ext_pressed())
            {
                actuator_stop();
                sys_state = ST_OPEN;
                break;
            }
            // timeout safety
            if ((now_ms() - action_start_ts) > MAX_EXTEND_TIME_MS)
            {
                safety_fault("EXTEND TIMEOUT");
                break;
            }
            // if flow drops below threshold - hysteresis while opening, begin retract
            if (measured_flow <= (FLOW_THRESHOLD - HYSTERESIS))
            {
                // start retracting
                action_start_ts = now_ms();
                actuator_retract_start();
                sys_state = ST_CLOSING;
            }
            break;
        }

        case ST_OPEN:
        {
            // while open, if flow drops below threshold - hysteresis => close
            if (measured_flow <= (FLOW_THRESHOLD - HYSTERESIS))
            {
                action_start_ts = now_ms();
                actuator_retract_start();
                sys_state = ST_CLOSING;
            }
            break;
        }

        case ST_CLOSING:
        {
            // stop if retracted limit reached
            if (is_limit_ret_pressed())
            {
                actuator_stop();
                sys_state = ST_CLOSED;
                break;
            }
            // timeout safety
            if ((now_ms() - action_start_ts) > MAX_RETRACT_TIME_MS)
            {
                safety_fault("RETRACT TIMEOUT");
                break;
            }
            // If while closing flow jumps above threshold + hysteresis then reopen
            if (measured_flow >= (FLOW_THRESHOLD + HYSTERESIS))
            {
                action_start_ts = now_ms();
                actuator_extend_start();
                sys_state = ST_OPENING;
            }
            break;
        }

        case ST_FAULT:
        {
            // remain stopped until reset
            // optionally check if user cleared fault and limit switches normal
            break;
        }
    }
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  memset(flow_history, 0, sizeof(flow_history));
  actuator_stop();
  uint32_t last_control_ms = now_ms();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Sample ADC and update filter ~ every 100 ms
	      float sample_flow = read_flow_adc_to_slpm();
	      if (sample_flow < 0.0f) {
	        // ADC error - fault
	        safety_fault("ADC READ ERROR");
	      } else {
	        update_flow_filter(sample_flow);
	      }

	      // Run control every 200 ms (adjust as needed)
	      if ((now_ms() - last_control_ms) >= 200U) {
	        control_state_machine();
	        last_control_ms = now_ms();
	      }

	      HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
