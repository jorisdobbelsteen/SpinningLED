/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Joris Dobbelsteen.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "button.h"
#include "status_led.h"
#include "motor_control.h"
#include "rotation_sensor.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum state_t {
  /* Note: FATAL_ERROR is handled by program counter, stuck in ErrorHandler() */
  /* STATE               PWM        LED               NEXT_STATE                             */
  STATE_ERROR,        // off        ERROR             btn_stop                  -> STOPPED
  STATE_INIT,         // off        INIT              both_button && powercycle -> CALIBRATE
                      //                              else                      -> STOPPED
  STATE_CALIBRATION,  // on (max)   TROTTLE_CALIB     !both_button              -> STOPPED
                      //                              break                     -> ERROR
  STATE_STOP,         // on (min)   STOPPING /        start_pressed             -> START
                      //            STOPPED           break                     -> ERROR
  STATE_START         // on (PID)   STARTING /        stop_pressed              -> STOP
                      //            STARTED           out_of_limits             -> ERROR
                      //                              break                     -> ERROR
} state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PULSES_PER_ROTATION 12
#define ROTATIONS_PER_SEC 15
#define RAMP_PULSE_PER_SEC (1 * PULSES_PER_ROTATION)
#define OVERSPEED_PULSES_PER_SEC (7 * PULSES_PER_ROTATION)
#define UNDERSPEED_PULSES_PER_SEC (7 * PULSES_PER_ROTATION)
#define STABLE_PULSES_PER_SEC (PULSES_PER_ROTATION / 2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static state_t global_state;
static int init_allow_calibration;
static uint32_t init_enter_tick;
static PIDController control_pid;
static volatile unsigned int control_pid_tick_counter;
static unsigned int control_pid_last_tick_counter;
static volatile int control_setpoint_millipulsesec;
static volatile int control_error;
static volatile int control_stable;
static int control_setpoint_internal_millipulsesec;
static int start_stop_stable;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PID_init(void) {
  control_pid.Kp =  0.0005f;
  control_pid.Ki =  0.04f;
  control_pid.Kd = -0.000f;
  control_pid.tau = 0.02f;
  control_pid.limMin = 0.0f;
  control_pid.limMax = 0.8f;
  control_pid.limMinInt = -0.1f;
  control_pid.limMaxInt = 0.7f;
  control_pid.T = 0.02f;

  PIDController_Init(&control_pid);

  control_error = 0;
  control_stable = 0;
  control_setpoint_millipulsesec = 0;
  control_setpoint_internal_millipulsesec = 0;
}

static void PID_tick(void) {
  control_pid_tick_counter++;

  // Load from volatile value
  int final_setpoint = control_setpoint_millipulsesec;

  // Slowly ramp to final setpoint
  int setpoint_e = final_setpoint - control_setpoint_internal_millipulsesec;
  int increment = RAMP_PULSE_PER_SEC * 1000 * control_pid.T;
  int decrement = -5 * RAMP_PULSE_PER_SEC * 1000 * control_pid.T;
  if (setpoint_e > increment)
    setpoint_e = increment;
  else if (setpoint_e < decrement)
    setpoint_e = decrement;
  control_setpoint_internal_millipulsesec += setpoint_e;

  // Acquire current speed
  int millipulses_per_sec = rotation_sensor_get_millipulses_per_sec();

  // Safety/Sanity check
  if (millipulses_per_sec > control_setpoint_internal_millipulsesec + OVERSPEED_PULSES_PER_SEC * 1000
      || millipulses_per_sec + UNDERSPEED_PULSES_PER_SEC * 1000 < control_setpoint_internal_millipulsesec) {
    control_error = 1;
    motor_control_set_pulse(MOTOR_PWM_MIN);
    return;
  }

  // PID loop
  float out = PIDController_Update(&control_pid,
                                   control_setpoint_internal_millipulsesec / (1000.0f * PULSES_PER_ROTATION),
                                   millipulses_per_sec / (1000.0f * PULSES_PER_ROTATION));

  // Set output
  motor_control_set_pulse(MOTOR_PWM_MIN + (int)(out * (MOTOR_PWM_MAX - MOTOR_PWM_MIN)));

  // Determine if loop is stable
  int final_error = millipulses_per_sec - final_setpoint;
  control_stable = final_error > -STABLE_PULSES_PER_SEC * 1000
          && final_error < STABLE_PULSES_PER_SEC * 1000;
}

static void PID_print_stats(void) {
  unsigned int c = control_pid_tick_counter;
  if (c - control_pid_last_tick_counter < 4)
    return;
  control_pid_last_tick_counter = c;

  printf("%03d  m %2.3f e %2.3f o %1.4f   i %1.4f d %1.4f\n",
         c % 1000,
         control_pid.prevMeasurement,
         control_pid.prevError,
         control_pid.out,
         control_pid.integrator,
         control_pid.differentiator);
}

static void enter_state(state_t state) {
  switch (state) {
    case STATE_ERROR:
      status_led_set(STATUS_LED_MODE_ERROR);
      puts("ERROR");
      motor_control_stop();
      break;
    case STATE_INIT:
      status_led_set(STATUS_LED_MODE_INIT);
      puts("INIT");
      init_enter_tick = HAL_GetTick();
      break;
    case STATE_CALIBRATION:
      status_led_set(STATUS_LED_MODE_THROTTLE_CALIB);
      puts("CALIBRATE");
      motor_control_start(MOTOR_PWM_MAX); // Set to maximum speed
      break;
    case STATE_STOP:
      status_led_set(STATUS_LED_MODE_STOPPING);
      puts("STOP");
      start_stop_stable = 0;
      motor_control_start(MOTOR_PWM_MIN); // Set to minimum speed
      break;
    case STATE_START:
      status_led_set(STATUS_LED_MODE_STARTING);
      puts("START");
      start_stop_stable = 0;
      PID_init();
      control_setpoint_millipulsesec = ROTATIONS_PER_SEC * PULSES_PER_ROTATION * 1000;
      motor_control_start_update_interrupt();
      break;
    // NOTE: Rely on compiler to give warning when this is incomplete
  }
}

static void exit_state(state_t state) {
  switch (state) {
    case STATE_ERROR:
      rotation_sensor_reset();
      break;
    case STATE_INIT:
      init_allow_calibration = 0;
      buttons_reset(); // ignore any pending presses
      break;
    case STATE_CALIBRATION:
      motor_control_set_pulse(MOTOR_PWM_MIN);
      buttons_reset(); // ignore any pending presses
      break;
    case STATE_STOP:
      break;
    case STATE_START:
      motor_control_stop_update_interrupt();
      break;
    // Rely on compiler to give warning when this is incomplete
  }
}

static void set_state(state_t next_state) {
  state_t previous_state = global_state;
  if (next_state != previous_state) {
    // Exit state is not aware which exit is performed
    exit_state(previous_state);
    // In enter_state, we can use "global_state" to get the next state
    enter_state(next_state);
    global_state = next_state;
  }
}

void buttons_start_pressed(void) {
  switch (global_state) {
    case STATE_STOP:
      set_state(STATE_START);
      break;
    default:
      break;
  }
}

void buttons_stop_pressed(void) {
  switch (global_state) {
    case STATE_ERROR:
      set_state(STATE_STOP);
      break;
    case STATE_START:
      set_state(STATE_STOP);
      break;
    default:
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &MotorTimer) {
    if (global_state == STATE_START) { // sanity check if we really should run the PWM
      if (control_error != 0) {
        motor_control_set_pulse(MOTOR_PWM_MIN);
        control_error++;
      } else {
        PID_tick();
      }
    } else {
      Error_Handler();
    }
  } /*else if (htim == &ButtonTimer) {
    IT_buttons_check();
  } */
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
  if (htim == &MotorTimer) {
    IT_motor_control_break();
  }
}

static void loop(void) {
  // Check motor state...
  if (motor_control_get_state() == MOTOR_CONTROL_BREAK) {
    puts("Motor BREAK detected");
    set_state(STATE_ERROR);
  }

  buttons_check();

  switch (global_state) {
    case STATE_INIT:
      // Ticks are in milliseconds
      if (HAL_GetTick() - init_enter_tick > 2000) {
        set_state(STATE_STOP);
      } else if (init_allow_calibration && buttons_both_pressed()) {
        set_state(STATE_CALIBRATION);
      }
      break;
    case STATE_CALIBRATION:
      if (!buttons_both_pressed()) {
        set_state(STATE_STOP);
      }
      break;
    case STATE_STOP: {
      const int stable = rotation_sensor_get_millipulses_per_sec() <= 333; // no pulse for 3 seconds
      if (stable != start_stop_stable) {
        status_led_set(stable ? STATUS_LED_MODE_STOPPED : STATUS_LED_MODE_STOPPING);
        start_stop_stable = stable;
      }
    } break;
    case STATE_START: {
      const int stable = control_stable;
      if (control_error > 5) { // give several pulses to allow setting PWM to minimum
        puts("Control ERROR detected");
        set_state(STATE_ERROR);
      } else  if (stable != start_stop_stable) {
        status_led_set(stable ? STATUS_LED_MODE_STARTED : STATUS_LED_MODE_STARTING);
        start_stop_stable = stable;
      }
      PID_print_stats();
    } break;
    default:
      break;
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

  HAL_SetTickFreq(HAL_TICK_FREQ_100HZ);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  init_allow_calibration =
    !__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) && // Option Byte Load reset
    // normal start has PIN reset !__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) && // Pin reset
    !__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) && // POR/PDR reset.
    !__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) &&  // Software reset.
    !__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) && // Independent Watchdog reset.
    !__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) && // Window Watchdog reset.
    !__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST); // Low Power Reset
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  PID_init();

  buttons_init();
  status_led_init();
  motor_control_init();
  rotation_sensor_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  global_state = STATE_INIT;
  enter_state(global_state);

  while (1)
  {
    loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    __WFE();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 31250-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 254;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1600-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 25-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 20000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 15;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STOP_BTN_Pin START_BTN_Pin */
  GPIO_InitStruct.Pin = STOP_BTN_Pin|START_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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

  // Force the motor control off on any error.
  motor_control_force_break();

  // Add status led
  status_led_set(STATUS_LED_MODE_FATAL);

  // Power cycle required, though we could trigger it ourselves here
  // TODO: Implement via pressing and releasing the stop button (needs debounce logic)
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  (void)file;
  (void)line;
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
