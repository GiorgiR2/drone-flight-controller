/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "motor.c"
#include "BMP180.h"
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 100
#define MAX_PWM 65000
uint16_t MIN_PWM = 40000;

bool calculate_data = false;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

float F[2*2]= {1, 0.01,
		       0, 1
};
float G[2*1] = {0.5*0.01*0.01,
		        0.004
};

float P[2*2] = {0, 0,
		        0, 0
};
//float Q[2*2] = G*~G*10.0f*10.0f;

// altitude and velocity
float S[2*1] = {0, 0};
float H[1*2] = {1, 0};

float I[2*2] = {1, 0,
		        0, 1};
float Acc[1*1]; // accelerometer measurement in centimeter/s**2

float K[2*1];
float R[1*1] = {30*30};

float L[1*1];
float M[1*1]; // just barometer measurement in centimeters
/*
void kalman_2d(float AccZ, float AltBaro){
	Acc[0] = AccZ;
	S = F*S + G*ACC;
	P = F*P*~F + Q;
	L = H*P*~H + R;
	K = P*~H*Invert(L);

	M[0] = AltBaro;
	S = S+K*(M-H*S);
	P = (I-K*H)*P;
}*/

typedef struct {
	uint16_t motor0;
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;

	bool stop_motors;
	bool pid_on;
} Drone;

Drone drone;

typedef struct {
	float alpha;

	float accel_roll;
	float accel_pitch;

	float gyro_roll;
	float gyro_pitch;
	float gyro_yaw;

	float gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;

	float accel_x_offset;
	float accel_y_offset;
	float accel_z_offset;

	float complementary_roll;
	float complementary_pitch;
} ComplementaryFilterRP;

// for UART receive
uint8_t rx_buffer[2];
uint8_t esp_buffer[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct LPF{
	float alpha;
	float offset;

	float input;
	float output;
} altitude;

void init_LPF(struct LPF *filter, float alpha){
	filter->alpha = alpha;
	filter->output = 0.0;
	filter->offset = 0.0;
}

void update_LPF(struct LPF *filter){
	filter->output = filter->alpha*filter->output + (1-filter->alpha)*filter->input;
}

void assign_motor_pwms(void){
	TIM1->CCR1 = drone.motor0;
	TIM1->CCR2 = drone.motor1;
	TIM1->CCR3 = drone.motor2;
	TIM1->CCR4 = drone.motor3;
}

void motor_zeros(void){
	drone.motor0 = 0;
	drone.motor1 = 0;
	drone.motor2 = 0;
	drone.motor3 = 0;

	assign_motor_pwms();
}

void motor0_plus(uint16_t pwm){
	if (drone.motor0 + pwm < MAX_PWM){
		drone.motor0 += pwm;
		TIM1->CCR1 = drone.motor0;
	}
}
void motor0_minus(uint16_t pwm){
	if (drone.motor0 - pwm > MIN_PWM){
		drone.motor0 -= pwm;
		TIM1->CCR1 = drone.motor0;
	}
}

void motor1_plus(uint16_t pwm){
	if (drone.motor1 + pwm < MAX_PWM){
		drone.motor1 += pwm;
		TIM1->CCR2 = drone.motor1;
	}
}
void motor1_minus(uint16_t pwm){
	if (drone.motor1 - pwm > MIN_PWM){
		drone.motor1 -= pwm;
		TIM1->CCR2 = drone.motor1;
	}
}

void motor2_plus(uint16_t pwm){
	if (drone.motor2 + pwm < MAX_PWM){
		drone.motor2 += pwm;
		TIM1->CCR3 = drone.motor2;
	}
}
void motor2_minus(uint16_t pwm){
	if (drone.motor2 - pwm > MIN_PWM){
		drone.motor2 -= pwm;
		TIM1->CCR3 = drone.motor2;
	}
}

void motor3_plus(uint16_t pwm){
	if (drone.motor3 + pwm < MAX_PWM){
		drone.motor3 += pwm;
		TIM1->CCR4 = drone.motor3;
	}
}
void motor3_minus(uint16_t pwm){
	if (drone.motor3 - pwm > MIN_PWM){
		drone.motor3 -= pwm;
		TIM1->CCR4 = drone.motor3;
	}
}

typedef int16_t(func)(void);

float getAVG(func f, float div){
	int16_t sum = 0;
	for (uint8_t i=0; i<20; i++){
		sum += f();
	}
	return (sum/20)/div;
}

void calculate_offsets(ComplementaryFilterRP *CPF){
	CPF->accel_x_offset = getAVG(get_x_a, 16384.0);
	CPF->accel_y_offset = getAVG(get_y_a, 16384.0);
	CPF->accel_z_offset = getAVG(get_z_a, 16384.0);

	CPF->gyro_x_offset = getAVG(get_x_g, 131.0);
	CPF->gyro_y_offset = getAVG(get_y_g, 131.0);
	CPF->gyro_z_offset = getAVG(get_z_g, 131.0);
}

typedef struct {
	float kp;
	float ki;
	float kd;

	float heading;
	float cumulative_error;
	float previous_error;
} Axis_PID;

float kp_roll  = 3.5;
float ki_roll  = 0.0;
float kd_roll  = 40.0;
float roll_heading     = 0.0;
float roll_cumulative_error = 0.0;
float roll_previous_error   = 0.0;

void PID_Roll(float pv){
	float error = pv - (float)roll_heading;
	float p = error * kp_roll;
	float i = roll_cumulative_error * ki_roll;
	float d = (error - roll_previous_error) * kd_roll;

	float total = p + i + d;

	if(total > 0){
		motor0_minus((uint16_t)total);
		motor1_minus((uint16_t)total);
		motor2_plus((uint16_t)total);
		motor3_plus((uint16_t)total);
	}
	else{
		total = 0-total;
		motor0_plus((uint16_t)total);
		motor1_plus((uint16_t)total);
		motor2_minus((uint16_t)total);
		motor3_minus((uint16_t)total);
	}

	roll_cumulative_error += error;
	roll_previous_error    = error;
}

float kp_pitch = 3.5;
float ki_pitch = 0.0;
float kd_pitch = 40.0;
float pitch_heading    = 0.0;
float pitch_cumulative_error = 0.0;
float pitch_previous_error   = 0.0;

void PID_Pitch(float pv){
	float error = pv - (float)pitch_heading;
	float p = error * kp_pitch;
	float i = pitch_cumulative_error * ki_pitch;
	float d = (error - pitch_previous_error) * kd_pitch;

	float total = p + i + d;

	if(total > 0){
		motor0_plus((uint16_t)total);
		motor1_minus((uint16_t)total);
		motor2_plus((uint16_t)total);
		motor3_minus((uint16_t)total);
	}
	else{
		total = 0-total;
		motor0_minus((uint16_t)total);
		motor1_plus((uint16_t)total);
		motor2_minus((uint16_t)total);
		motor3_plus((uint16_t)total);
	}

	pitch_cumulative_error += error;
	pitch_previous_error    = error;
}

float kp_yaw = 3.5;
float ki_yaw = 0.0;
float kd_yaw = 40.0;
float yaw_heading    = 0.0;
float yaw_cumulative_error = 0.0;
float yaw_previous_error   = 0.0;

void PID_Yaw(float pv){
	float error = pv - (float)yaw_heading;
	float p = error * kp_yaw;
	float i = yaw_cumulative_error * ki_yaw;
	float d = (error - yaw_previous_error) * kd_yaw;

	float total = p + i + d;

	if(pv > 0){
		motor0_plus((uint16_t)total);
		motor1_minus((uint16_t)total);
		motor2_minus((uint16_t)total);
		motor3_plus((uint16_t)total);
	}
	else{
		total = 0-total;
		motor0_minus((uint16_t)total);
		motor1_plus((uint16_t)total);
		motor2_plus((uint16_t)total);
		motor3_minus((uint16_t)total);
	}

	yaw_cumulative_error += error;
	yaw_previous_error    = error;
}

void landing(void){
	drone.pid_on = false;
	drone.stop_motors = false;

	while(drone.motor0 > 0){
		if(drone.motor0 > 500){
			drone.motor0 -= 500;
		}
		else{
			break;
		}

		if(drone.motor1 > 500){
			drone.motor1 -= 500;
		}
		else{
			break;
		}

		if(drone.motor2 > 500){
			drone.motor2 -= 500;
		}
		else{
			break;
		}

		if(drone.motor3 > 500){
			drone.motor3 -= 500;
		}
		else{
			break;
		}

		assign_motor_pwms();
		HAL_Delay(50);
	}
	motor_zeros();
}

void ESP_Init(void){
	uint8_t enable_connections[13] = "AT+CIPMUX=1\r\n"; // allow multiple connections
	uint8_t start_server[19] = "AT+CIPSERVER=1,80\r\n"; // bind server (port 80)

	for(uint16_t i=0; i<65535; i++);
	HAL_UART_Transmit(&huart3, (const uint8_t*)enable_connections, strlen((char*)enable_connections), TIMEOUT);

	for(uint16_t i=0; i<65535; i++);
	HAL_UART_Transmit(&huart3, (const uint8_t*)start_server, strlen((char*)start_server), TIMEOUT);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  ComplementaryFilterRP CPF = {.alpha = 0.98, .complementary_roll = 0.0F, .complementary_pitch = 0.0F, .gyro_yaw = 0.0F};

  drone.pid_on      = false;
  drone.stop_motors = false;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  motor_zeros();
  MPU6050_Init();
  ESP_Init();
  //init_LPF(&altitude, 0.99);
  //BMP180_start();

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 1);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)esp_buffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float dt = 0.0112;
  float AcX, AcY, AcZ;
  uint8_t pbuf[30];
  uint8_t counter = 0;
  //altitude.offset = BMP180_AVGAlt();
  calculate_offsets(&CPF);
  while(1){
	  if(calculate_data){
		  counter++;
		  calculate_data = false;
		  if (counter%10 == 0){
			  sprintf((char*)pbuf, "m0: %.03d ", drone.motor0);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "m1: %.03d ", drone.motor1);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "m2: %.03d ", drone.motor2);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "m3: %.03d\r\n", drone.motor3);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);

			  sprintf((char*)pbuf, "r %.03f\r\n", CPF.complementary_roll);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "p %.03f\r\n", CPF.complementary_pitch);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "y %.03f\r\n", CPF.gyro_yaw);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);

			  /*
			  sprintf((char*)pbuf, "baro: %.03f  ", baro);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "position: %.03f  ", position);
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);

			  sprintf((char*)pbuf, "Temp: %.03f ", BMP180_GetTemp());
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "Pressure: %.03f ", BMP180_GetPress(0));
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);
			  sprintf((char*)pbuf, "Altitude: %.03f\r\n", BMP180_GetAlt(0));
			  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), TIMEOUT);*/
		  }
/*
		  altitude.input = BMP180_GetAlt(0) - altitude.offset;
		  update_LPF(&altitude);
		  baro   = altitude.output;
		  accel  = get_z_a()/16384.0-CPF.accel_z_offset;
		  speed += accel*9.8*dt;
		  position = speed + accel*dt*dt/2;
*/

		  AcX = get_x_a()/16384.0 - CPF.accel_x_offset;
		  AcY = get_y_a()/16384.0 - CPF.accel_y_offset;
		  AcZ = get_z_a()/16384.0;// - CPF.accel_z_offset;

		  CPF.accel_roll  = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 57.3; // 57.296
		  CPF.accel_pitch = atan2(AcX, sqrt(AcY*AcY + AcZ*AcZ)) * 57.3; // 57.296

		  CPF.complementary_roll  = CPF.alpha*(CPF.complementary_roll  + (get_x_g()/131.0-CPF.gyro_x_offset)*dt) + (1-CPF.alpha)*CPF.accel_roll;
	  	  CPF.complementary_pitch = CPF.alpha*(CPF.complementary_pitch + (get_y_g()/131.0-CPF.gyro_y_offset)*dt) - (1-CPF.alpha)*CPF.accel_pitch;
		  CPF.gyro_yaw += (get_z_g()/131.0 - CPF.gyro_z_offset) * dt;

		  if(drone.pid_on == true){
			  PID_Roll(CPF.complementary_roll);
			  PID_Pitch(CPF.complementary_pitch);
			  //PID_Yaw(CPF.gyro_yaw);
		  }
		  if(drone.stop_motors){
			  landing();
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		//11.1 ms
		calculate_data = true;
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  if(huart->Instance == USART2){
	  HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_buffer, 1);
	  if(rx_buffer[0] == 'u'){
		  motor0_plus(500);
		  motor1_plus(500);
		  motor2_plus(500);
		  motor3_plus(500);
	  }
	  else if(rx_buffer[0] == 'd'){
		  motor0_minus(500);
		  motor1_minus(500);
		  motor2_minus(500);
		  motor3_minus(500);
	  }
	  else if(rx_buffer[0] == 's'){
		  //MIN_PWM = 0;
		  drone.pid_on = false;
		  motor_zeros();
	  }
	  else if(rx_buffer[0] == 'w'){
		  drone.stop_motors = true;
	  }

	  else if(rx_buffer[0] == 'r'){
		  motor0_plus(200);
		  motor1_plus(200);
		  motor2_minus(200);
		  motor3_minus(200);
	  }
	  else if(rx_buffer[0] == 'R'){
		  motor0_minus(200);
		  motor1_minus(200);
		  motor2_plus(200);
		  motor3_plus(200);
	  }

	  else if(rx_buffer[0] == 'p'){
		  motor0_minus(200);
		  motor1_plus(200);
		  motor2_minus(200);
		  motor3_plus(200);
	  }
	  else if(rx_buffer[0] == 'P'){
		  motor0_plus(200);
		  motor1_minus(200);
		  motor2_plus(200);
		  motor3_minus(200);
	  }

	  else if(rx_buffer[0] == 'y'){
		  motor0_plus(200);
		  motor1_minus(200);
		  motor2_minus(200);
		  motor3_plus(200);
	  }
	  else if(rx_buffer[0] == 'Y'){
		  motor0_minus(200);
		  motor1_plus(200);
		  motor2_plus(200);
		  motor3_minus(200);
	  }

	  else if(rx_buffer[0] == 'i'){
		  //MIN_PWM = 15000;
		  drone.pid_on = true;
	  }

	  else if(rx_buffer[0] == '1'){
		  motor0_plus(200);
	  }
	  else if(rx_buffer[0] == '2'){
		  motor1_plus(200);
	  }
	  else if(rx_buffer[0] == '3'){
		  motor2_plus(200);
	  }
	  else if(rx_buffer[0] == '4'){
		  motor3_plus(200);
	  }
	  else if(rx_buffer[0] == '!'){
		  motor0_minus(200);
	  }
	  else if(rx_buffer[0] == '@'){
		  motor1_minus(200);
	  }
	  else if(rx_buffer[0] == '#'){
		  motor2_minus(200);
	  }
	  else if(rx_buffer[0] == '$'){
		  motor3_minus(200);
	  }
  }
  else if(huart->Instance == USART3){
	  if(esp_buffer[0] == '1'){
		  motor0_plus(500);
		  motor1_plus(500);
		  motor2_plus(500);
		  motor3_plus(500);
	  }
	  else if(esp_buffer[0] == '2'){
		  motor0_minus(500);
		  motor1_minus(500);
		  motor2_minus(500);
		  motor3_minus(500);
	  }
	  else if(esp_buffer[0] == '3'){
		  //MIN_PWM = 0;
		  drone.pid_on = false;
		  motor_zeros();
	  }
	  else if(esp_buffer[0] == '4'){
		  drone.stop_motors = true;
	  }
	  else if(esp_buffer[0] == '5'){
		  motor0_plus(45000);
		  motor1_plus(45000);
		  motor2_plus(45000);
		  motor3_plus(45000);
		  //MIN_PWM = 22000;
		  drone.pid_on = true;
	  }
	  HAL_UART_Receive_IT(&huart3, (uint8_t*)esp_buffer, 1);
    }
}

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
