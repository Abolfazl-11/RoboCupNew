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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pixy.h"
#include "mpu6050.h"
#include "Motors.h"
#include "Movement.h"
#include "SR04.h"

int SR_Read_TIM = 500;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Pixy Variables
BallTransform ballTransform;
int ballInView = 0;
int pixyChecked = 0;

// MPU6050 Variables
MPU6050_t mpu6050;
float gx, gy, gz;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
float sx, sy, sz;
typedef struct GY {
	float x;
	float y;
	float z;
} GY;
GY Gy = {0, 0, 0};


// Motors Variables
// For robot 1
Motors_t Motors = {
		0,
		0,
		0,
		1,
		0,
		1,
		0,
		0
};

// For robot 0
//Motors_t Motors = {
//		0,
//		0,
//		0,
//		0,
//		0,
//		0,
//		0,
//		1
//};

MotorDef_t Motor_1 = {1, 2, GPIO_PIN_8, GPIOA};

MotorDef_t Motor_2 = {1, 3, GPIO_PIN_4, GPIOA};

MotorDef_t Motor_3 = {2, 4, GPIO_PIN_12, GPIOA};

MotorDef_t Motor_4 = {1, 4, GPIO_PIN_6, GPIOA};

Motor_Defs MotorDefs = {
	&Motor_1,
	&Motor_2,
	&Motor_3,
	&Motor_4
};

// PID Control Derivative
double pve = 0;

// GetBall zones
enum Zones zone = FAR;

enum AttackZones attackZone = MIDDLE;

// Defining SRs
SRDef_t Sr_l = {GPIO_PIN_14, GPIOB, GPIO_PIN_15, GPIOB, 2};

SRDef_t Sr_r = {GPIO_PIN_1, GPIOA, GPIO_PIN_2, GPIOA, 3};

SRDef_t Sr_b = {GPIO_PIN_13, GPIOB, GPIO_PIN_12, GPIOB, 1};

SRDef_t Sr_f = {GPIO_PIN_0, GPIOA, GPIO_PIN_15, GPIOC, 0};

SRDatas_t SRDatas = {0, 0, 0, 0};

SRDef_t *Srs[4] = {&Sr_f, &Sr_b, &Sr_l, &Sr_r};

int MPUCollibrated = 0;

int InGoal = 0;

uint64_t noBallCounter = 0;
uint64_t backToGoalCounter = 0;

enum OutState { IN, OUT, HALTED };

enum OutState outState = IN;

int out_interrupt = 0;
int out_interrupt_counter = 0;

int outDir = 0;

int backingToGoal = 0;

// counter for the timer
uint64_t timcounter = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void CollibrateMPU6050(int samples) {
	for (int RateCalibrationNumber=0; RateCalibrationNumber < samples; RateCalibrationNumber++) {
	    MPU6050_Read_All(&hi2c2, &mpu6050, 0, 0, 0);
	    RateCalibrationRoll+=mpu6050.Gx;
	    RateCalibrationPitch+=mpu6050.Gy;
	    RateCalibrationYaw+=mpu6050.Gz;
	    HAL_Delay(1);
	}
	RateCalibrationRoll/=samples;
	RateCalibrationPitch/=samples;
	RateCalibrationYaw/=samples;

	MPUCollibrated = 1;
}

void SetupMPU6050(int cSamples) {
	  while (MPU6050_Init(&hi2c2) == 1);

	  CollibrateMPU6050(cSamples);

	  HAL_Delay(500);
}

void ReadMPU6050() {
	MPU6050_Read_All(&hi2c2, &mpu6050, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);

	sx += mpu6050.Gx;
	sy += mpu6050.Gy;
	sz += mpu6050.Gz;

	Gy.x = sx / 2000;
	Gy.y = sy / 2000;
	Gy.z = sz / 2000;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(timcounter % 10 == 0 && pixyChecked) {
		getBallPosition(&ballTransform, &ballInView);
	}

	if(timcounter % 2 == 0 && MPUCollibrated) {
		ReadMPU6050();
	}

	if((timcounter + 1) % SR_Read_TIM == 0) {
		ReadAllSRs(Srs, 4, &SRDatas);
	}

	timcounter++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2) {
		out_interrupt = 1;
		out_interrupt_counter++;
	}
}

void ReadOutDirection() {
	outDir = 0;
	outDir += 10 * !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	outDir += !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int teta;

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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  // setting up PWM
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  TIM2->CCR4 = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_Delay(200);

  SetupMPU6050(750);

  SetupPixy(&pixyChecked);

  HAL_Delay(200);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (out_interrupt) {
		  outState = OUT;
		  out_interrupt = 0;
	  }
	  switch(outState) {
	  case IN:
		  if (abs(Gy.z) > 2) {
			  RotateToZero(Gy.z, &pve, &Motors, &MotorDefs);
		  }
		  else {
			  setPWM(&Motor_1, Motors.pwm1, Motors.e1, &Motors);
			  setPWM(&Motor_2, Motors.pwm2, Motors.e2, &Motors);
			  setPWM(&Motor_3, Motors.pwm3, Motors.e3, &Motors);
			  setPWM(&Motor_4, Motors.pwm4, Motors.e4, &Motors);
		  }

	//	  if (zone == BALLIN) {
	//		  Attack(&Motors, &MotorDefs, &SRDatas, &attackZone, 35);
	//	  }
		  if (ballInView) {
			  GetBall(ballTransform.ballx, ballTransform.bally, 35, &zone, &Motors, &MotorDefs, &InGoal, &SRDatas);
			  noBallCounter = 0;
			  backToGoalCounter = 0;
			  SR_Read_TIM = 500;
			  backingToGoal = 0;
		  }

		  if (!ballInView) {
			  noBallCounter++;
		  }

		  if (noBallCounter >= 13500) {
			  noBallCounter = 0;
			  AllMotorsZero(&MotorDefs, &Motors);
			  backToGoalCounter++;

		  }

		  if (!InGoal && SRDatas.SR_b < GOALDIS_TH && backingToGoal) {
		  		InGoal = 1;
		  		AllMotorsZero(&MotorDefs, &Motors);
		  }

	//	   For robot 0
		  if (backToGoalCounter >= 2) {
			  backToGoalCounter = 0;
			  if (!InGoal) BackToGoal(&Motors, &MotorDefs, &InGoal, &SRDatas);
			  backingToGoal = 1;
			  SR_Read_TIM = 200;
		  }


		  if (InGoal && SRDatas.SR_b > GOALDIS_TH) {
			  InGoal = 0;
		  }
		  break;
	  case OUT:
		  if (out_interrupt_counter == 1) {
			  AllMotorsZero(&MotorDefs, &Motors);
			  ReadOutDirection();
		  }
		  outState = HALTED;
		  break;
	  case HALTED:
		  if (ballInView) {
			  if (ballTransform.ballx >= 0) teta = -(atan((double)ballTransform.bally / ballTransform.ballx) * RAD_TO_DEG - 90);
			  else if (ballTransform.ballx < 0) teta = -((atan((double)ballTransform.bally / ballTransform.ballx) + PI)* RAD_TO_DEG - 90);

			  switch (outDir) {
			  case 0:
				  // front
				  if(abs(teta) > 90) {
					  outState = IN;
				  }
				  break;
			  case 1:
				  //back
				  if(abs(teta) < 90) {
					  outState = IN;
				  }
				  break;
			  case 10:
				  if(teta < -10 && teta > -170) {
					  outState = IN;
				  }
				  break;
			  case 11:
				  if(teta > 10 && teta < 170) {
					  outState = IN;
				  }
				  break;
			  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
