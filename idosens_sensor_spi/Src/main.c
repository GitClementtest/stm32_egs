/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */












  /* USER CODE END 2 */

  /* Infinite loop */

  while (1)
  {


	  /* USER CODE BEGIN WHILE */
	  	    uint8_t acc[10]={0,0,0,0,0,0,0,0,0,0};
	        uint8_t octet_read=0;
	        uint8_t data[3];
	  	  	uint8_t data_fifo[2];

	  	    uint8_t fifo_ctrl = 0x2e;//0b101110
	  	    data_fifo[0]=0b01101110; //écrit 1 octet


	  	   	 data_fifo[1]=0;
	  	   	//Configure FIFO, bypass mode p41
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data_fifo, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);

			data_fifo[1]=64; //(64)10 = stream, (32)10 fifo


			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data_fifo, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);


			//Ecrire (0)10 puis (64)10 dans CTRL0 ; 0b00011111, 0x1f
			uint8_t data_ctrl0[2];
			data_ctrl0[0]=0b01011111;
			data_ctrl0[1]=0;


			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data_ctrl0, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);

			/*
			data_ctrl0[1]=64;

			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,data_ctrl0, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
			*/

			//Config ctrl1 0x20 : Acceleration data-rate 3.125Hz (23)10

			uint8_t tab1[2];
			tab1[0]=0b0100000;
			tab1[1]=23;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,tab1, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);


			 //Lire 0x20 : 0b00100000
			octet_read=0b10100000;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);



			//Lire OUT_X_L_A (28h)
			octet_read=0b10101000;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
			acc[0]=data[0];

			//Lire OUT_X_H_A (29h)
			octet_read=0b10101001;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);

			acc[1]=data[0];

			//Lire OUT_Y_L_A (2ah),
			octet_read=0b10101010;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 2, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);

			acc[2]=data[0];

			//Lire OUT_Y_H_A (2bh),
			octet_read=0b10101011;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 2, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);

			acc[3]=data[0];

			//Lire OUT_Z_H_A (2Ch)
			octet_read=0b10101100;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 2, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
			acc[4]=data[0];

			//Lire OUT_Z_L_A (2dh)
			octet_read=0b10101101;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&octet_read, 2, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
			acc[5]=data[0];


			 uint8_t num =65;
			 uint8_t lettres[20];
			 sprintf(lettres,"XL : %d, XH : %d, YL : %d, YH : %d ZL : %d, ZH : %d \n\r",acc[0],acc[1],acc[2], acc[3], acc[4], acc[5]);
			 uint8_t len = strlen(lettres);
			 HAL_UART_Transmit(&huart1, lettres,len , HAL_MAX_DELAY);




	  		 HAL_Delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
