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
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include <math.h>

#define Delta_t 0.001;

#define PI 3.141592653589
#define SHAFT_PULSE_PER_ROT 28.0
#define GEAR_RATIO 210.0

#define PULSE_PER_REVOLUTION (GEAR_RATIO*SHAFT_PULSE_PER_ROT)
#define WHEEL_DIAMETER 4.55 //cm
#define WHEEL_RADIOUS 4.55/2 //cm
#define WHEEL_DISTANCE 16.05 //cm
#define WHEEL_CIRCUMFERENCE PI*WHEEL_DIAMETER
#define DIST_CONVERSION WHEEL_CIRCUMFERENCE/PULSE_PER_REVOLUTION

#define PWM_MIN 15

#define Encoder_Offset 5000

#define MAP_SIZE 400 // Example size, adapt to your actual map size
#define SCALE (MAP_SIZE / 4)
#define TOLERANCE 0.1
#define KFA 10.0
#define KFR 1.0
#define TS 0.6
#define KP 10.0
#define KI 0.0001
#define RATE 5

#define SAFETY_DISTANCE 100

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint8_t buff[4];
	float value;
} packet_buffer_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define BUFFER_SIZE 30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[BUFFER_SIZE];
uint8_t TxBuffer[BUFFER_SIZE];
float x_received, y_received, theta_received, right_value, left_value, middle_value;
uint8_t flag = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;
packet_buffer_t received_x, received_y, received_theta, _packetX, _packetY, _packet_theta, right_sensor_received, left_sensor_received, middle_sensor_received;


packet_buffer_t encoder_value,move1, move2, move3, move4;

packet_buffer_t Right_Wheel_Encoder_value;
packet_buffer_t Left_Wheel_Encoder_value;

int32_t Right_Wheel_Encoder_count;
int32_t Left_Wheel_Encoder_count;

char msg[250] = "";


double Distance_Right = 0.0;
double Distance_Left = 0.0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;
float d_theta_degree;

float right_velocity = 0.0;
float left_velocity = 0.0;
float target_theta;

float map[MAP_SIZE][MAP_SIZE];
float caminho_x[1000], caminho_y[1000];
int caminho_idx = 0.0;
float erroIntegral = 0.0;

float position_tolerance = 0.1;
float Kp = 1;

float Kp_linear = 9.3;
float Kp_angular = 0.5;

float target_pose_x = 0.0;
float target_pose_y =0.0;

float aux_x_target;

float aux_y_target;

float aux_theta_target = 0.0;
float aux_theta = 0.0;

float error_theta;

float angular_vel_left;
float angular_vel_right;

float dist_right_sensor = 0.0;
float dist_middle_sensor = 0.0;
float dist_left_sensor = 0.0;


float teste1 = 0.0;
float teste2 = 0.0;

uint32_t prevTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void receivedX();
void receivedY();
void receivedTheta();
void send_x();
void send_y();
void send_theta();
void received_right_sensor_value();
void received_left_sensor_value();
void received_middle_sensor_value();
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void setDirection(float left, float right){
	if(left > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	}else if(left == 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

	}

	if(right > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	}else if(right == 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  TIM1->CNT = Encoder_Offset;
  TIM2->CNT = Encoder_Offset;

  // Start Timers
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
 HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

 HAL_TIM_Base_Start_IT(&htim5);

 // Start SPI
 HAL_SPI_TransmitReceive_DMA(&hspi1, TxBuffer, RxBuffer, BUFFER_SIZE);


 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //PA1 TIM2 CH2

 //uint16_t readValue;
 //int speed = 1;
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while(1)
{
	// Simulação de leitura da pose atual do robô (substitua pela sua implementação real)

		          //---------------------------------------------------------------


//	if(right_value < SAFETY_DISTANCE  || left_value < SAFETY_DISTANCE){
////		aux_x_target = target_pose_x;
////		target_pose_x = x;
////
////		aux_y_target = target_pose_y;
////		target_pose_y = y;
//	}

//	else{


//					target_pose_x = aux_x_target;
//					target_pose_y = aux_y_target;

		          	float error_x = target_pose_x - x;
		            float error_y = target_pose_y - y;

		              // Calcula a distância e ângulo
		            float distance = sqrt(error_x * error_x + error_y * error_y);
		            target_theta = (atan2(error_y, error_x)*180)/PI;
		              // Calcula o erro angular
		            error_theta = target_theta - d_theta_degree;
		            if(error_theta < 1 && error_theta > -1) error_theta = 0;

		              // Normaliza o erro angular


		              // Controlador proporcional
		            float linear_velocity = Kp_linear * distance;
		            float angular_velocity = -Kp_angular * error_theta;

		              // Velocidades desejadas das rodas

		            float distance_to_target = sqrt(pow(target_pose_x - x, 2) + pow(target_pose_y - y, 2));
		            	              if (distance_to_target < position_tolerance) {
		            	            	  left_velocity = 0;
		            	            	  right_velocity = 0;
		            	              }
		              //RPM
		              left_velocity = (linear_velocity/WHEEL_RADIOUS)-(WHEEL_DISTANCE/2)*(angular_velocity/WHEEL_RADIOUS);
		              right_velocity = (linear_velocity/WHEEL_RADIOUS)+(WHEEL_DISTANCE/2)*(angular_velocity/WHEEL_RADIOUS);
	//	              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); //MOTOR DIREITO
	//	              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); //MOTOR ESQUERDO
	//
	//	              setDirection(0, 0);




		              //printf("Left Velocity: %.2f, Right Velocity: %.2f\n", left_velocity, right_velocity);
		              //printf("Target theta : %.2f\n", target_theta);

		              // Define a direção dos motores com base nas velocidades desejadas
		  //            if (left_velocity >= 0) {
		  //              //MotorLeftForward();
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_velocity);
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_velocity);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
		  //            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		  //
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  ////            	teste1 = teste1+1;
		  ////            	if (left_velocity > right_velocity) HAL_Delay(2000);
		  //            	HAL_Delay(300);
		  //
		  //            } else {
		  //              //MotorLeftBackward();
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_velocity);
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_velocity);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
		  //            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  //            	HAL_Delay(300);
		  //            }
		  //
		  //            if (right_velocity >= 0) {
		  //              //MotorRightForward();
		  //
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_velocity);
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_velocity);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
		  //            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  //            	teste2 = teste2+1;
		  //            	if (left_velocity < right_velocity) HAL_Delay(2000);
		  //            	HAL_Delay(300);
		  //            } else {
		  //              //MotorRightBackward();
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_velocity);
		  //            	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_velocity);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
		  //            	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		  //            	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		  //            	HAL_Delay(300);
		  //            }

		              // Verifica se o robô atingiu a posição alvo


		              HAL_Delay(100);
//	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  htim4.Init.Prescaler = 60-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 6000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 60000-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LED2_Pin|BIN2_Pin|BIN1_Pin
                          |AIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 LED2_Pin BIN2_Pin BIN1_Pin
                           AIN2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LED2_Pin|BIN2_Pin|BIN1_Pin
                          |AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AIN1_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AIN1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void receivedX() {

	received_x.buff[0] = RxBuffer[5];
	received_x.buff[1] = RxBuffer[6];
	received_x.buff[2] = RxBuffer[7];
	received_x.buff[3] = RxBuffer[8];

	x_received = received_x.value;
	target_pose_x = x_received;

}

void receivedY() {

	received_y.buff[0] = RxBuffer[9];
	received_y.buff[1] = RxBuffer[10];
	received_y.buff[2] = RxBuffer[11];
	received_y.buff[3] = RxBuffer[12];

	y_received = received_y.value;
	target_pose_y = y_received;
}

void receivedTheta() {

	received_theta.buff[0] = RxBuffer[13];
	received_theta.buff[1] = RxBuffer[14];
	received_theta.buff[2] = RxBuffer[15];
	received_theta.buff[3] = RxBuffer[16];

	theta_received = received_theta.value;

}

void received_right_sensor_value() {

	right_sensor_received.buff[0] = RxBuffer[17];
	right_sensor_received.buff[1] = RxBuffer[18];
	right_sensor_received.buff[2] = RxBuffer[19];
	right_sensor_received.buff[3] = RxBuffer[20];

	right_value = right_sensor_received.value;

}


void received_left_sensor_value() {

	left_sensor_received.buff[0] = RxBuffer[21];
	left_sensor_received.buff[1] = RxBuffer[22];
	left_sensor_received.buff[2] = RxBuffer[23];
	left_sensor_received.buff[3] = RxBuffer[24];

	left_value = left_sensor_received.value;

}


void received_middle_sensor_value() {

	middle_sensor_received.buff[0] = RxBuffer[25];
	middle_sensor_received.buff[1] = RxBuffer[26];
	middle_sensor_received.buff[2] = RxBuffer[27];
	middle_sensor_received.buff[3] = RxBuffer[28];

	middle_value = middle_sensor_received.value;

}

void send_x() {

	_packetX.value = x;
	TxBuffer[5] = _packetX.buff[0];
	TxBuffer[6] = _packetX.buff[1];
	TxBuffer[7] = _packetX.buff[2];
	TxBuffer[8] = _packetX.buff[3];

}

void send_y() {

	_packetY.value = y;
	TxBuffer[9] = _packetY.buff[0];
	TxBuffer[10] = _packetY.buff[1];
	TxBuffer[11] = _packetY.buff[2];
	TxBuffer[12] = _packetY.buff[3];

}

void send_theta() {

	_packet_theta.value = theta;
	TxBuffer[13] = _packet_theta.buff[0];
	TxBuffer[14] = _packet_theta.buff[1];
	TxBuffer[15] = _packet_theta.buff[2];
	TxBuffer[16] = _packet_theta.buff[3];


}





void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{

	received_right_sensor_value();
	received_left_sensor_value();
	received_middle_sensor_value();
	receivedX();
	receivedY();
	receivedTheta();
	send_x();
	send_y();
	send_theta();

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM5){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			int32_t Actual_Time = HAL_GetTick();

			Right_Wheel_Encoder_value.value = (int32_t) (TIM2->CNT - Encoder_Offset);
			Left_Wheel_Encoder_value.value = (int32_t) (TIM1->CNT - Encoder_Offset);

			TIM1->CNT = Encoder_Offset;
			TIM2->CNT = Encoder_Offset;

			Left_Wheel_Encoder_count = Left_Wheel_Encoder_value.value;
			Right_Wheel_Encoder_count = (int32_t) Right_Wheel_Encoder_value.value;

			Distance_Right = Right_Wheel_Encoder_count * DIST_CONVERSION;
			Distance_Left = Left_Wheel_Encoder_count * DIST_CONVERSION;

			float d = (Distance_Right + Distance_Left) / 2.0;
			float d_theta = (Distance_Left - Distance_Right) / WHEEL_DISTANCE;



			x +=  d *  cos(theta + (d_theta/2));
			y += d * sin(theta +  (d_theta/2));
			theta += d_theta;

			d_theta_degree = theta * (180 / PI);

			angular_vel_left = 60*(Left_Wheel_Encoder_count/0.001)/PULSE_PER_REVOLUTION;
			angular_vel_right = 60*(Right_Wheel_Encoder_count/0.001)/PULSE_PER_REVOLUTION;

			float erro_w_left = left_velocity - angular_vel_left;
			float erro_w_right = right_velocity - angular_vel_right;

			float Kp = 10;

			float pwmL = erro_w_left*Kp;
			float pwmR = erro_w_right*Kp;

			if(pwmL < -100 ) pwmL = -100;
			if(pwmL > 100 ) pwmL = 100;
			if(pwmR < -100 ) pwmR = -100;
			if(pwmR > 100 ) pwmR = 100;

			if(pwmL < 0) pwmL = -pwmL;
			if(pwmR < 0) pwmR = -pwmR;


			uint32_t newPwm_L = htim4.Instance->ARR*(pwmL*0.01);
			uint32_t newPwm_R = htim4.Instance->ARR*(pwmR*0.01);

			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, newPwm_R); //MOTOR DIREITO
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, newPwm_L); //MOTOR ESQUERDO

			if(d_theta_degree > 360) d_theta_degree -= 360;
			if(d_theta_degree < -360) d_theta_degree += 360;

			setDirection(erro_w_left, erro_w_right);
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
