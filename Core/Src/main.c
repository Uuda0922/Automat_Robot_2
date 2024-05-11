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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "mainpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN1_DEVICE_NUM     4
#define FIRST_GROUP_ID      0x200
#define MOTOR_SPEED_MAX     16384
#define CAN_DATA_SIZE       8
#define CAN1_RX_ID_START    0x201
#define MOTOR_ID            4
#define spi_enable      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define spi_disable     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
	float KP;
	float KI;
	float KD;
	int error;
	int I_error;
	int D_error;
	int lastError;

} PID_Variables;
typedef struct
{
	int LY;
	int LX;
	int RY;
	int RX;
	double X;
	double Y;
}joystick_variables;

typedef struct
{
	int target_pos;
	int32_t my_pos;
	float ControlSignal;
	int PWM;
}DC_Motor;
typedef struct
{
	int16_t setpoint;
	int16_t Out;
	uint16_t ID;
	int16_t en_speed;
} M3508_Variables;
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId WheelcontrolHandle;
osThreadId JoystickHandle;
osThreadId updowncontrolHandle;
osThreadId Target_posHandle;
osThreadId ColorHandle;
/* USER CODE BEGIN PV */
int16_t Accel_X_Raw;
int16_t Accel_Y_Raw;
int16_t Accel_Z_Raw;

double pi=3.14159265359;
double ML;
double MR;
extern int right_joy;
extern double left_joy, left_y;
double speed;
double degree_a;

float freq=0;
float blue_freq;
float red_freq;
float dif=0;
float cTime=0;
float pTime=0;
float dTime=0;
float Ax;
float Ay;
float Az;

int vel_up=0;
int IR = 0;
int motor_dir=0;
int is_first=0;
int ic1=0;
int ic2=0;
int red=0;
int blue=0;
int yellow=0;
int flag=0;
int BLDC = 0;
int color_flag=0;
int color_value=0;
int imu_x=0;
int switch_1;
int switch_2;
int switch_3;
int switch_4;
int switch_5;
int switch_6;
int target_pos;
int My_pos;
int shalgaj_duusah=0;
bool bombogav_cnt = false;
bool take_ball = false;
int32_t count;
uint8_t HC_PS2_RX[9];
uint8_t HC_PS2_TX[9]={0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};
uint8_t RxData[1];
int16_t sensor_buff[5];


uint8_t             data[8];
uint8_t 			buffer[8];
uint32_t            pTxMailbox;
uint8_t 			rxData[8];
PID_Variables M1_pid;
PID_Variables M2_pid;
PID_Variables M3_pid;
PID_Variables M4_pid;
PID_Variables DC_pid;

M3508_Variables M1;
M3508_Variables M2;
M3508_Variables M3;
M3508_Variables M4;


DC_Motor Motor;
joystick_variables PS2;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM13_Init(void);
void Wheeltask(void const * argument);
void task2_joystick(void const * argument);
void dcmotor(void const * argument);
void Colorcheck(void const * argument);
void StartTask05(void const * argument);

/* USER CODE BEGIN PFP */
//void calculatePID(void);
void motorspeed(void);
int Color_Sensor();
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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
//  while(1){
//	  if(mode=1){
//		  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET);
//		  TIM3->CCR4 = 40;
//	  }
//	  if(mode=0){
//	  		  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET);
//	  		  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET);
//	  		  TIM3->CCR4 = 1;
//	  	  }
//	  if(mode=-1){
//	  		  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET);
//	  		  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET);
//	  		  TIM3->CCR4 = 60;
//	  	  }
//	  if(mode=2){
//	 	  		  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET);
//	 	  		  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET);
//
//	 	  	  }
//	  if(((HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_4_GPIO_Port, SWITCH_4_Pin))==0)&& RX[4]==254){
//		mode=1;
//	  }
//	  if(((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin))==0)&& RX[4]==255){
//	  		mode=0;
//	  	  }
//	  if(((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin))==0)&& RX[4]==254){
//	  	  		mode=-1;
//	  	  	  }
//	  if(((HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_4_GPIO_Port, SWITCH_4_Pin))==0)&& RX[4]==255){
//	  	  		mode=2;
//	  	  	  }
//
//  }



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  /* definition and creation of Wheelcontrol */
  osThreadDef(Wheelcontrol, Wheeltask, osPriorityNormal, 0, 128);
  WheelcontrolHandle = osThreadCreate(osThread(Wheelcontrol), NULL);

  /* definition and creation of Joystick */
  osThreadDef(Joystick, task2_joystick, osPriorityNormal, 0, 2048);
  JoystickHandle = osThreadCreate(osThread(Joystick), NULL);

  /* definition and creation of updowncontrol */
  osThreadDef(updowncontrol, dcmotor, osPriorityNormal, 0, 512);
  updowncontrolHandle = osThreadCreate(osThread(updowncontrol), NULL);

  /* definition and creation of Target_pos */
  osThreadDef(Target_pos, Colorcheck, osPriorityNormal, 0, 512);
  Target_posHandle = osThreadCreate(osThread(Target_pos), NULL);

  /* definition and creation of Color */
  osThreadDef(Color, StartTask05, osPriorityIdle, 0, 128);
  ColorHandle = osThreadCreate(osThread(Color), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20404768;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  htim3.Init.Prescaler = 21385;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 25000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 2156;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim13, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_2A_GPIO_Port, MOTOR_2A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|MOTOR_1_B_Pin|MOTOR_1_A_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_2_Pin SWITCH_3_Pin SWITCH_6_Pin */
  GPIO_InitStruct.Pin = SWITCH_2_Pin|SWITCH_3_Pin|SWITCH_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_4_Pin */
  GPIO_InitStruct.Pin = SWITCH_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_2B_Pin */
  GPIO_InitStruct.Pin = MOTOR_2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTOR_2B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_2A_Pin */
  GPIO_InitStruct.Pin = MOTOR_2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_2A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Pin */
  GPIO_InitStruct.Pin = IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin MOTOR_1_B_Pin MOTOR_1_A_Pin S2_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|MOTOR_1_B_Pin|MOTOR_1_A_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_5_Pin SWITCH_1_Pin */
  GPIO_InitStruct.Pin = SWITCH_5_Pin|SWITCH_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_Pin */
  GPIO_InitStruct.Pin = S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MotorUp(void){
	HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET);
	TIM3 ->CCR4 = 40;
}
void MotorDown(void)
{

	HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET);
	TIM3 ->CCR4 = 65;
}
void MotorStop(void)
{
	TIM3 -> CCR4 = 1;
	HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	if(htim->Instance==TIM5){
//		Motor.my_pos = ((int32_t)__HAL_TIM_GET_COUNTER(htim)) / 100;
//	}
	if(htim-> Instance == TIM13){
		  if(is_first == 0){
			  ic1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			  is_first = 1;
		  }
		  else{
			  ic2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			  if(ic2>ic1)
			  {
				  dif = ic2 - ic1;
			  }
			  else if(ic1>ic2){
				  dif = (0xffffffff-ic1) + ic2;
			  }
			  freq = dif;
			  is_first = 0;
		  }

	}
}
//void calculatePID()
//{
//
//	cTime = HAL_GetTick();
//	Motor.my_pos = ((int32_t)TIM2->CNT)/100;
//	dTime = (cTime - pTime)/10000;
//	DC_pid.error = Motor.target_pos - Motor.my_pos;
//	DC_pid.I_error += (DC_pid.error * dTime);
//	DC_pid.D_error = (DC_pid.error - DC_pid.lastError)/dTime;
//	Motor.ControlSignal = (DC_pid.KP*DC_pid.error) + (DC_pid.I_error*DC_pid.KI) + (DC_pid.D_error*DC_pid.KD);
//	HAL_Delay(1);
//	pTime = cTime;
//	DC_pid.lastError = DC_pid.error;
//}
//void motorspeed(){
//	if(Motor.ControlSignal<0)
//		motor_dir = -1;
//	else if(Motor.ControlSignal>0)
//		motor_dir = 1;
//	else
//		motor_dir = 0;
//	Motor.PWM = (int)fabs(Motor.ControlSignal);
//	if(Motor.PWM > 120)
//		TIM3 -> CCR4 = 1;
////	TIM8 -> CCR2 = 50;
//	if(Motor.PWM < 120 && DC_pid.error != 0){
//		TIM3 -> CCR4 = 60;
//	}
//	if(motor_dir == 1){
//		MotorUp();
//	}else if(motor_dir == (-1)){
//		MotorDown();
//	}else{
//		MotorStop();
//	}
//}

int Color_Sensor(){
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, SET);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, RESET);
	// blue check
//	color_flag=1;
	HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
	HAL_Delay(10);
	red_freq=freq;
//	color_flag=0;
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, RESET);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, RESET);
	HAL_Delay(10);
	blue_freq=freq;
	HAL_TIM_IC_Stop_IT(&htim13, TIM_CHANNEL_1);
	if(blue_freq>=17000 && red_freq<=7200){
		//blue ball
		color_flag=-1;
	}
	if(red_freq>=14000 && blue_freq<=5500){
		//red ball
		color_flag=1;
	}
//	if(red_freq<=5000 && blue_freq<=5100){
//		// purple ball
//		color_flag=0;
//	}
	if(abs(red_freq-blue_freq)<=2500){
		color_flag=0;
	}
	return color_flag;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Wheeltask */
/**
  * @brief  Function implementing the Wheelcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Wheeltask */
void Wheeltask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task2_joystick */
/**
* @brief Function implementing the Joystick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2_joystick */
void task2_joystick(void const * argument)
{
  /* USER CODE BEGIN task2_joystick */
  /* Infinite loop */
  for(;;)
  {
	  count = TIM2->CNT;
		  spi_enable;
		  HAL_SPI_TransmitReceive(&hspi1, HC_PS2_TX, HC_PS2_RX, 9, 10);
		  spi_disable;
		  PS2.LY=-(HC_PS2_RX[8]-127);  //left_y;
		  PS2.LX=(HC_PS2_RX[7]-127);  //left_joy
		  PS2.RY=HC_PS2_RX[6]-128;
		  PS2.RX=HC_PS2_RX[5]-128;
		  PS2.RX = right_joy;
		  PS2.X=PS2.LX/(float)128;
		  PS2.Y=PS2.LY/(float)128;
	  	  speed=sqrt(PS2.X*PS2.X+PS2.Y*PS2.Y);
	  	  degree_a=atan2(PS2.Y,PS2.X);
	  	  ML=sin(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  	  MR=cos(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  	  if(HC_PS2_RX[4]==251){
	  		  BLDC = 1;
	  	  }else if(HC_PS2_RX[4]==254){
	  		  BLDC = 2;
	  	  }else if(HC_PS2_RX[4]==247){
	  		  yellow=1;
	  		  target_pos = 1820;
	  	  }else if(HC_PS2_RX[4]==253){
	  		  yellow=2;
	  		  target_pos = 0;
	  	  }
	  	  if((abs(PS2.LY) > 5 || abs(PS2.LX)>5 )&& abs(PS2.RX)<=5){
	  		  if(ML>12000 || MR>12000){
	  			  if(ML > MR){
	  				  MR=MR/ML*12000;
	  				  ML=12000;

	  			  }
	  			  else{
	  				  ML=(ML/MR)*12000;
	  				  MR=12000;
	  			  }
	  		  }
	  		  else if(ML<-12000 || MR<-12000){
	  			  if(ML < MR){
	  				  MR=-MR/ML*12000;
	  				  ML=-12000;

	  			  }
	  			  else{
	  				  ML=-ML/MR*12000;
	  				  MR=-12000;
	  			  }
	  		  }
	  		  else if(ML>12000 || MR<-12000){
	  			  if(ML > -MR){
	  				  MR=MR/ML*12000;
	  				  ML=12000;

	  			  }
	  			  else{
	  				  ML=-ML/MR*12000;
	  				  MR=-12000;
	  			  }
	  		  }
	  		  else if(MR>12000 || ML<-12000){
	  			  if(MR > -ML){
	  				  ML=ML/MR*12000;
	  				  MR=12000;

	  			  }
	  			  else{
	  				  MR=-MR/ML*12000;
	  				  ML=-12000;
	  			  }
	  		  }
	  		  M1.setpoint=-MR;
	  		  M2.setpoint=ML;
	  		  M4.setpoint=-ML;
	  		  M3.setpoint=MR;
	  	  }
	  	  else if(abs(PS2.LY)<=5 && abs(PS2.LX)<=5 && abs(PS2.RX)>5){
	  		  M1.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
	  		  M2.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
	  		  M3.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
	  		  M4.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
	  	  }
	  	  else if((abs(PS2.LY)>5 || abs(PS2.LX)>5) && abs(PS2.RX)>5){
	  		  PS2.X=PS2.LX/(float)128;
	  		  PS2.Y=PS2.LY/(float)128;
	  		  speed=sqrt(PS2.X*PS2.X+PS2.Y*PS2.Y);
	  //			  if(speed>=1){
	  ////				  speed=sign(speed);
	  //				  speed=1;
	  //			  }
	  //			  if(speed<=-1){
	  //				  speed=-1;
	  //			  }
	  		  degree_a=atan2(PS2.Y,PS2.X);
	  		  ML=sin(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  		  MR=cos(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  		  if(ML>12000 || MR>12000){
	  			  if(ML > MR){
	  				  MR=MR/ML*12000;
	  				  ML=12000;

	  			  }
	  			  else{
	  				  ML=ML/MR*12000;
	  				  MR=12000;
	  			  }
	  		  }
	  		  else if(ML<-12000 || MR<-12000){
	  			  if(ML < MR){
	  				  MR=-MR/ML*12000;
	  				  ML=-12000;

	  			  }
	  			  else{
	  				  ML=-ML/MR*12000;
	  				  MR=-12000;
	  			  }
	  		  }
	  		  else if(ML>12000 || MR<-12000){
	  			  if(ML > -MR){
	  				  MR=MR/ML*12000;
	  				  ML=12000;

	  			  }
	  			  else{
	  				  ML=-ML/MR*12000;
	  				  MR=-12000;
	  			  }
	  		  }
	  		  else if(MR>12000 || ML<-12000){
	  			  if(MR > -ML){
	  				  ML=ML/MR*12000;
	  				  MR=12000;

	  			  }
	  			  else{
	  				  MR=-MR/ML*12000;
	  				  ML=-12000;
	  			  }
	  		  }
	  		  M1.setpoint=-MR;
	  		  M2.setpoint=ML;
	  		  M4.setpoint=-ML;
	  		  M3.setpoint=MR;
	  		  M1.setpoint=M1.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
	  		  M2.setpoint=M2.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
	  		  M3.setpoint=M3.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
	  		  M4.setpoint=M4.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
	  	  }
	  	  else{
	  		  M1.setpoint=0;
	  		  M2.setpoint=0;
	  		  M3.setpoint=0;
	  		  M4.setpoint=0;
	  	  }
    osDelay(10);
  }
  /* USER CODE END task2_joystick */
}

/* USER CODE BEGIN Header_dcmotor */
/**
* @brief Function implementing the updowncontrol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dcmotor */
void dcmotor(void const * argument)
{
  /* USER CODE BEGIN dcmotor */
//	DC_pid.error = 0;
		HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
		TIM12->CCR1=924;
		TIM12->CCR2=924;

  /* Infinite loop */
  for(;;)
  {
//	  Color_Sensor();
	  switch_5 = HAL_GPIO_ReadPin(SWITCH_5_GPIO_Port, SWITCH_5_Pin);
	  switch_6 = HAL_GPIO_ReadPin(SWITCH_6_GPIO_Port, SWITCH_6_Pin);
	  IR=HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin);
	  if(BLDC==1 || flag==1){
	  			TIM12->CCR1 = 935;
	  			TIM12->CCR2 = 918;
	  			flag = 1;
	  			//HAL_Delay(500);
	  			if((HAL_GPIO_ReadPin(SWITCH_5_GPIO_Port, SWITCH_5_Pin)==0) || (HAL_GPIO_ReadPin(SWITCH_6_GPIO_Port, SWITCH_6_Pin)==0)){
	  				TIM12->CCR1 = 924;
	  				TIM12->CCR2 = 924;
	  //				HAL_Delay(2000);
	  				flag = 0;
	  				BLDC = 0;
//					color_value=0;
//					osDelay(200);
					color_value=Color_Sensor();
//					TIM12->CCR1 = 918;
//					TIM12->CCR2 = 918;
//					osDelay(300);
//					color_value=color_value + Color_Sensor();
//					osDelay(300);
//					TIM12->CCR1 = 924;
//					TIM12->CCR2 = 924;
//					color_value=color_value + Color_Sensor();
					if(color_value !=0){
						  yellow=1;
						  target_pos = 1820;
						  //shalgaj_duusah=1;
					}
					else{
						//shalgaj_duusah=0;
						TIM12->CCR1 = 918;
						TIM12->CCR2 = 935;
						osDelay(500);
						TIM12->CCR1 = 924;
						TIM12->CCR2 = 924;
						}

	  			}
	  		}
	  		else if(BLDC==2 && flag==0){
	  			//HAL_Delay(200);
	  			if((HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin)) == 1){
					TIM12->CCR1 = 924;
					TIM12->CCR2 = 924;
					osDelay(2000);
					take_ball = false;
					BLDC = 0;
					yellow=2;
					target_pos = 0;
				}else{
					TIM12->CCR1 = 918;
					TIM12->CCR2 = 935;
//					osDelay(1000);
//					TIM12->CCR1 = 924;
//					TIM12->CCR2 = 924;
//					osDelay(1000);
//					harah = 1;
					//break;
				}
	  		}
    osDelay(5);
  }
  /* USER CODE END dcmotor */
}

/* USER CODE BEGIN Header_Colorcheck */
/**
* @brief Function implementing the Target_pos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Colorcheck */
void Colorcheck(void const * argument)
{
  /* USER CODE BEGIN Colorcheck */
//	Motor.my_pos = TIM2->CNT=0;
//	DC_pid.KP = 0.7;
//	DC_pid.KI = 0.0005;
//	DC_pid.KD = 0.00003;
//	Motor.target_pos=0;
	switch_3 = HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin);

  /* Infinite loop */
  for(;;)
  {
	  switch_1 = HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin);
	  switch_2 = HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin);
	  switch_3 = HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin);
	  switch_4 = HAL_GPIO_ReadPin(SWITCH_4_GPIO_Port, SWITCH_4_Pin);
	  if(yellow==2 && target_pos == 0){//target pos = 0
	  		 if(((HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_4_GPIO_Port, SWITCH_4_Pin))==0)){
	  			 MotorStop();
	  //			 Motor.target_pos=0;
//	  			 TIM2->CNT=0;
	  			 My_pos=0;
	  			 yellow=0;
	  //			 break;
	  		 }else{
	  		 	 target_pos= 0;
//	  			 calculatePID();
	  			 MotorDown();
	  			 TIM12->CCR1 =924;
	  			TIM12->CCR2 =924;
	  		 }
	  	}else if(yellow==1 && target_pos == 1820){//target pos = 1800
	  		 if(((HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin))==0 || (HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin))==0)){
	  			 MotorStop();
	  			 My_pos = 1820;
	  			 target_pos = 0;
//	  			 TIM2->CNT = 182000;
	  			 yellow=0;
	  //			 break;
	  		 }else{
	  		 	 target_pos = 1820;
//	  			 calculatePID();
	  			 MotorUp();
	  		 }
	  	}else{
	  		if((HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin) == 0) || (HAL_GPIO_ReadPin(SWITCH_4_GPIO_Port, SWITCH_4_Pin) == 0)){
	  			MotorStop();
	  			yellow = 0;
	  		}
	  	 }
    osDelay(2);
  }
  /* USER CODE END Colorcheck */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Color thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
	setup();
  /* Infinite loop */
  for(;;)
  {

	  if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00000001;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11111110;
	  	  }
	  	  if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00000010;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11111101;
	  	  }
	  	  if(HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00000100;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11111011;
	  	  }
	  	  if(HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00001000;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11110111;
	  	  }
	  	  if(HAL_GPIO_ReadPin(SWITCH_5_GPIO_Port, SWITCH_5_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00010000;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11101111;
	  	  }
	  	  if(HAL_GPIO_ReadPin(SWITCH_5_GPIO_Port, SWITCH_5_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b00100000;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b11011111;
	  	  }
	  	  if(HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin)==1){
	  		  sensor_buff[4]=sensor_buff[4]|0b01000000;
	  	  }
	  	  else{
	  		  sensor_buff[4]=sensor_buff[4]&0b10111111;
	  	  }
	  	  sensor_buff[0]=M1.en_speed;
	  	  sensor_buff[1]=M2.en_speed;
	  	  sensor_buff[2]=M3.en_speed;
	  	  sensor_buff[3]=M4.en_speed;
	  	  loop();
    osDelay(10);
  }
  /* USER CODE END StartTask05 */
}

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
