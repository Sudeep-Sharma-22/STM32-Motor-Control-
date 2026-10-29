/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

#include "Com/cmd.h"
#include "Com/munching.h"
#include "Com/packet.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t ESTOP = 0;
uint8_t TX_CPLT_FLAG = 1;
uint8_t UNKNOWN_CMD = 0;

/**
 * MICROCONTROLLER BOARD Mappings
 * ------------------------------
 * PWM2 -> MOTOR_1 -> Left Motor
 * PWM1 -> MOTOR_2 -> Right Motor
 */

// MOTOR_1 Direction -> A2
// MOTOR_2 Direction -> B5

#define MOT1_ENC htim2             // A5, B3
#define MOT1_PWM TIM_CHANNEL_2     // A6
#define MOT2_ENC htim5             // A0, A1
#define MOT2_PWM TIM_CHANNEL_1     // B6
#define LOOP_TIM htim10
#define PWM_TIM  htim1

size_t CDC_RX_PTR = 0; // CDC receive ring buffer
uint8_t CDC_RX_BUF[CDC_RX_SIZE] = { 0 };
uint8_t CDC_TX_BUF[CDC_TX_SIZE] = { 0 };

// rpm counters

uint32_t COUNTS_PER_REV = 23283; // 9031
float RPS_MAX = 1;

// filter constants

uint8_t use_bw = 1;
uint8_t use_ra = 1;

float BW_A[BW_N] = {
	2.83533457,
	-2.6839485,
	0.84809864
};
float BW_B[BW_N + 1] = {
	6.44099940e-05,
	1.93229982e-04,
	1.93229982e-04,
	6.44099940e-05
};

float RA_COEFF[BW_N] = {
	0.005,
	0.005,
	0.99,
};

// PI constants

float pi_kp = 2.95;
float pi_ki = 0.08;

// motor defines

PIController MOTOR_1 = {
	.unicorn = 0,
	.eps_set_point = 0,
	.last_encoder = 0,
	.enc_tim = &MOT1_ENC,
	.pwm_channel = MOT1_PWM,
	.dir_pin = MOT1_Dir_Pin,
	.dir_port = MOT1_Dir_GPIO_Port,
	.filter = {{ 0 }},
};
PIController MOTOR_2 = {
	.unicorn = 0,
	.eps_set_point = 0,
	.last_encoder = 0,
	.enc_tim = &MOT2_ENC,
	.pwm_channel = MOT2_PWM,
	.dir_pin = MOT2_Dir_Pin,
	.dir_port = MOT2_Dir_GPIO_Port,
	.filter = {{ 0 }},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t encoder_change(uint32_t encoder, PIController* motor) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(motor->enc_tim)) {
    	if (encoder <= motor->last_encoder)
    		return (motor->last_encoder - encoder);
    	else
    		return (motor->last_encoder + __HAL_TIM_GET_AUTORELOAD(motor->enc_tim) - encoder);
    } else {
    	if (encoder >= motor->last_encoder)
    		return (motor->last_encoder - encoder);
    	else
    		return (encoder + __HAL_TIM_GET_AUTORELOAD(motor->enc_tim) - motor->last_encoder);
    }
}

// rpm filter

/// Hard-coded 3rd-order Butterworth filter, defined for our timer sampling frequency
/// and estimated noise frequencies.
void bw_filter_rpm(FilterParams* filter) {
	if (use_bw) {
		float y_n = (BW_A[0] * filter->eps_y[2]) +
				(BW_A[1] * filter->eps_y[1]) +
				(BW_A[2] * filter->eps_y[0]) +
				(BW_B[0] * filter->eps_x[3]) +
				(BW_B[1] * filter->eps_x[2]) +
				(BW_B[2] * filter->eps_x[1]) +
				(BW_B[3] * filter->eps_x[0]);

		filter->eps_y[0] = filter->eps_y[1]; filter->eps_x[0] = filter->eps_x[1];
		filter->eps_y[1] = filter->eps_y[2]; filter->eps_x[1] = filter->eps_x[2];
		filter->eps_y[2] = y_n     	       ; filter->eps_x[2] = filter->eps_x[3];
	} else {
		filter->eps_y[0] = filter->eps_x[3];
		filter->eps_y[1] = filter->eps_x[3];
		filter->eps_y[2] = filter->eps_x[3];
	}
}

/// Running-average filter over the butterworth filter output.
void ra_filter_rpm(FilterParams* filter) {
	if (use_ra) {
		filter->eps_y[2] = (filter->eps_y[0] * RA_COEFF[0]) +
				(filter->eps_y[1] * RA_COEFF[1]) +
				(filter->eps_y[2] * RA_COEFF[2]);
	}
}

// PI controller

/// clamp duty cycle to bounds
float clamp(float val, float min, float max) {
	if      (val > max) return max;
	else if (val < min) return min;
	else                return val;
}

volatile uint32_t* pwm_channel_reg(TIM_TypeDef* timer, uint32_t ch) {
	switch(ch) {
	case TIM_CHANNEL_1: return &(timer->CCR1);
	case TIM_CHANNEL_2: return &(timer->CCR2);
	case TIM_CHANNEL_3: return &(timer->CCR3);
	case TIM_CHANNEL_4: return &(timer->CCR4);
	}

	return NULL;
}

void control_motor_pi(PIController* motor) {
	float error_t = motor->eps_set_point - motor->filter.eps_y[2];
	motor->unicorn = clamp(motor->unicorn + error_t, -1024000., 1024000.);

	float drive = (pi_kp * error_t) + (pi_ki * motor->unicorn);
	float duty_cycle = clamp(drive / COUNTS_PER_REV, -1., 1.);

	if (duty_cycle < 0.) {
		duty_cycle = -duty_cycle;
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 1);
	} else {
		HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, 0);
	}

	*pwm_channel_reg(PWM_TIM.Instance, motor->pwm_channel) = duty_cycle *
			__HAL_TIM_GET_AUTORELOAD(&PWM_TIM);
}

void stop_motor_pi(PIController* motor) {
	motor->eps_set_point = 0;
	*pwm_channel_reg(PWM_TIM.Instance, motor->pwm_channel) = 0;
}

// timer loop

float get_tim10_period() {
	return (float)(__HAL_TIM_GET_AUTORELOAD(&LOOP_TIM)) / HAL_RCC_GetPCLK1Freq();
}

void process_callback(PIController* motor, float dt) {
	uint32_t encoder = __HAL_TIM_GET_COUNTER(motor->enc_tim);
	int32_t delta_encoder = encoder_change(encoder, motor);

	motor->filter.eps_x[BW_N] = (delta_encoder / dt);

	bw_filter_rpm(&motor->filter);
	ra_filter_rpm(&motor->filter);

	motor->last_encoder = encoder;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == LOOP_TIM.Instance) {
		float period = get_tim10_period();

		process_callback(&MOTOR_1, period);
		process_callback(&MOTOR_2, period);
	}
}

// Command Processing

uint8_t CMD_BUF[16] = { 0 };

void echo(size_t ptr) {
	uint8_t echo_buf[32] = { 0 };
	EchoCommand echo = deserialize_echo(CMD_BUF + ptr, echo_buf);

	uint8_t command_buf[32] = { 0 };
	command_buf[0] = 1;
	uint8_t len = serialize_echo(&echo, command_buf + 1);

	while (!TX_CPLT_FLAG);
	memset(CDC_TX_BUF, 0, CDC_TX_SIZE);
	uint16_t packet_len = serialize_packet(command_buf, len + ptr, CDC_TX_BUF);
	CDC_Transmit_FS(CDC_TX_BUF, packet_len);
}

void process_command() {
	float rps_set;
	float float_recv;

	UNKNOWN_CMD = 0;

	size_t ptr = 0;

	uint8_t command = deserialize_byte(CMD_BUF); ptr += 1;

	switch (command) {
	case 1:
		echo(ptr);
		break;

	case 2:
		float_recv = deserialize_float(CMD_BUF + ptr);
		pi_kp = float_recv;
		break;

	case 3:
		float_recv = deserialize_float(CMD_BUF + ptr);
		pi_ki = float_recv;
		break;

	case 4: // RpmL
		float_recv = deserialize_float(CMD_BUF + ptr) / 60.;
		rps_set = clamp(float_recv, -RPS_MAX, RPS_MAX);
		MOTOR_1.eps_set_point = rps_set * COUNTS_PER_REV;
		break;

	case 5: // RpmR
		float_recv = deserialize_float(CMD_BUF + ptr) / 60.;
		rps_set = clamp(float_recv, 0 - RPS_MAX, RPS_MAX);
		MOTOR_2.eps_set_point = rps_set * COUNTS_PER_REV;
		break;

	default:
		UNKNOWN_CMD = 1;
	}

	memset(CMD_BUF, 0, 16);
}

void process_packets() {
	for (int i = 0; i < CDC_RX_SIZE - 1; i++) {
		uint16_t len = deserialize_u16(CDC_RX_BUF + i);

		uint16_t frame_ptr = i + len + 2;

		if (frame_ptr < CDC_RX_SIZE - 1) {
			uint16_t framing = deserialize_u16(CDC_RX_BUF + frame_ptr);

			if (framing == 0x0A0D) { // \r\n
				deserialize_packet(CDC_RX_BUF + i, CMD_BUF);
				process_command();
				i += len + 4;
			}
		}
	}

	CDC_RX_PTR = 0;
	memset(CDC_RX_BUF, 0, CDC_RX_SIZE);
}

// Other

void usb_configure_wait() {
	while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED);
}

uint8_t check_usb_status() {
	return (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED);
}

void stream_eps_values() {
	memset(CDC_TX_BUF, 0, CDC_TX_SIZE);
	int w = snprintf((char*)CDC_TX_BUF, 16, "%d,%d\n",
			(int)MOTOR_1.filter.eps_y[2],
			(int)MOTOR_2.filter.eps_y[2]);
	CDC_Transmit_FS(CDC_TX_BUF, w);
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
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&LOOP_TIM);
  HAL_TIM_Encoder_Start(&MOT1_ENC, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&MOT2_ENC, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);

  usb_configure_wait();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	ESTOP = check_usb_status() || ESTOP;

	process_packets();
	stream_eps_values();

	if (ESTOP) {
		stop_motor_pi(&MOTOR_1);
		stop_motor_pi(&MOTOR_2);
	} else {
		control_motor_pi(&MOTOR_1);
		control_motor_pi(&MOTOR_2);
	}

	HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  HAL_GPIO_WritePin(MOT2_Dir_GPIO_Port, MOT2_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOT1_Dir_GPIO_Port, MOT1_Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOT2_Dir_Pin */
  GPIO_InitStruct.Pin = MOT2_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MOT2_Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOT1_Dir_Pin */
  GPIO_InitStruct.Pin = MOT1_Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MOT1_Dir_GPIO_Port, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
