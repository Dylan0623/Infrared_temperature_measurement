/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mlx90614.h"
#include "stdio.h"
#include "motor.h"
#include "uart_handler.h"
#include "control.h"
#include "linefinder.h"
#include "BPSim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int ID;
uint8_t len;	
int mode;
int times=0,recive_flag,mask,face,learn;
float Temperature,Temperature_Skin;
uint8_t Temperature_limit ;
char mask_flag[5],face_flag[5];
float skin_temp;
uint8_t	UART1RxBuffer[1],UART2RxBuffer[1],UART3RxBuffer[1],UART4RxBuffer[1];
char Uart_get[20];
uint16_t Distance;
float BP_in[2],BP_out[1];
unsigned char ucRxData[100];
unsigned char ucRxFinish=0;
static unsigned char ucCnt=0;
char screen_get[10];
unsigned char ucRxcnt = 0;
uint8_t value = 0;
char get_val[8],get_id[8],get_mode[8];
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,UART1RxBuffer,1);
	HAL_UART_Receive_IT(&huart2,UART2RxBuffer,1);
	HAL_UART_Receive_IT(&huart3,UART3RxBuffer,1);
	HAL_UART_Receive_IT(&huart4,UART4RxBuffer,1);
	HAL_TIM_Base_Start_IT(&htim5);	//开启定时器5，定时0.1s，均值滤波
	simInit();
	MLX90614_Init(&hi2c1);
  MLX90614_SetEmissivity(0.985); // Human skin
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		uartx_printf(huart1,"%d",mode);
//		Beep();
		if(mode == 1)
		{
			Control_Loop_A();
		}
		else if(mode == 2)
		{
			Control_Loop_B();
		}
		else if(mode == 3)
		{
			Control_Loop_C();
			HAL_Delay(50);
		}
		else if(mode == 4)
		{
			Control_Loop_D();
			HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	unsigned char temp=0;
	extern unsigned char UART1RxBuffer[1];
	
	__HAL_UART_CLEAR_PEFLAG(huart);
	
	if(huart->Instance == USART1)			//串口1接收中断处理
	{
		Camera_Get(UART1RxBuffer);
	}
	if(huart->Instance == USART2)			//串口2接收中断处理
	{
		temp = UART2RxBuffer[0];
		ucRxData[ucCnt++]=temp;
		if(temp=='m') 
			{ 
				ucRxFinish=1;
			  ucCnt=0;
			}
	}
	if(huart->Instance == USART3)			//串口3接收中断处理
	{
		Screen_get(UART3RxBuffer);
	}
	if(huart->Instance == UART4)			//串口4接收中断处理
	{
		Camera_Get(UART4RxBuffer);
	}
	HAL_UART_Receive_IT(&huart1,UART1RxBuffer,1);
	HAL_UART_Receive_IT(&huart2,UART2RxBuffer,1);
	HAL_UART_Receive_IT(&huart3,UART3RxBuffer,1);
	HAL_UART_Receive_IT(&huart4,UART4RxBuffer,1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//外部中断，每检测到一次输入信号的上升沿，计数值frequency自增
{
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //均值滤波算法，算脉冲更精准
{
	if(htim-> Instance == htim5.Instance) 
	{
		
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
