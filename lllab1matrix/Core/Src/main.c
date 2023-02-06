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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct _PortPin {
	GPIO_TypeDef *PORT;
	uint16_t PIN;
} PortPin;

PortPin R[4] = { { GPIOA, GPIO_PIN_10 }, { GPIOB, GPIO_PIN_3 }, { GPIOB,
		GPIO_PIN_5 }, { GPIOB, GPIO_PIN_4 } };

PortPin L[4] = { { GPIOA, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_7 }, { GPIOB,
		GPIO_PIN_6 }, { GPIOA, GPIO_PIN_7 } };
uint16_t Number[13];
int num = 0;
int state = 0;
int savenum = 0;
int fake = 0;
int chick = 0;
uint16_t ButtonMatrix = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadMatrixButton_1Row();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	  static uint32_t timestamp = 0;
	 	  		if (HAL_GetTick() >= timestamp) {
	 	  			timestamp = HAL_GetTick() + 10;
	 	  			ReadMatrixButton_1Row();

	 	  		}
	 	  		if (num == 0 & ButtonMatrix > 0) {
	 	  			chick = 1;
	 	  		}
	 	  		else {

	 	  			chick = 0;
	 	  		}
	 	  		num = ButtonMatrix;

	 	  		switch (state) {
	 	  		case 0:
	 	  			if(ButtonMatrix == 1 & chick == 1)
	 	  			{
	 	  				state = 0 ;
	 	  			}
	 	  			else if (ButtonMatrix == 512 & chick == 1) //6
	 	  					{
	 	  				Number[0] = 6;
	 	  				state = 1;
	 	  			}
	 	  			else if (ButtonMatrix != 512 & chick == 1) {
	 	  				state = 11;
	 	  			}

	 	  			break;
	 	  		case 1:
	 	  			if (ButtonMatrix == 8192 & chick == 1) // 4
	 	  					{
	 	  				Number[1] = 4;
	 	  				state = 2;
	 	  			}
	 	  			else if (ButtonMatrix != 8192 & chick == 1) {
	 	  				state = 11;
	 	  			}

	 	  			break;
	 	  		case 2:
	 	  			if (ButtonMatrix == 1024 & chick == 1) // 3
	 	  					{
	 	  				Number[2] = 3;
	 	  				state = 3;
	 	  			}
	 	  			else if (ButtonMatrix != 1024 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break;
	 	  		case 3:
	 	  			if (ButtonMatrix == 8192 & chick == 1) // 4
	 	  					{
	 	  				Number[3] = 4;
	 	  				state = 4;
	 	  			}
	 	  			else if (ButtonMatrix != 8192 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break;
	 	  		case 4:
	 	  			if (ButtonMatrix == 32768 & chick == 1) // 0
	 	  					{
	 	  				Number[4] = 0;
	 	  				state = 5;
	 	  			}
	 	  			else if (ButtonMatrix != 32768 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break;
	 	  		case 5:
	 	  			if (ButtonMatrix == 32 & chick == 1) // 5
	 	  					{
	 	  				Number[5] = 5;
	 	  				state = 6;
	 	  			}
	 	  			else if (ButtonMatrix != 32 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break ;
	 	  		case 6:
	 	  			if (ButtonMatrix == 32768 & chick == 1) // 0
	 	  					{
	 	  				Number[6] = 0;
	 	  				state = 7;
	 	  			}
	 	  			else if (ButtonMatrix != 32768 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break;
	 	  		case 7:
	 	  			if (ButtonMatrix == 32768 & chick == 1) // 0
	 	  					{
	 	  				Number[7] = 0;
	 	  				state = 8;
	 	  			}
	 	  			else if (ButtonMatrix != 32768 & chick == 1) {
	 	  				state = 11;
	 	  			}
	 	  			break;
	 	  		 case 8 :
	 	  			 if(ButtonMatrix == 32768 & chick == 1) // 0
	 	  			  {
	 	  			  	Number[8] = 0 ;
	 	  			  	 state = 9 ;
	 	  			 }
	 	  		     else if(ButtonMatrix != 32768 & chick == 1)
	 	  			  {
	 	  			    	state = 11 ;
	 	  			  }
	 	  			break ;
	 	  		 case 9 :
	 	  			 if(ButtonMatrix == 32 & chick == 1) // 5
	 	  			 {
	 	  				 Number[9] = 5 ;
	 	  				 state = 10 ;
	 	  			 }
	 	  			 else if (ButtonMatrix != 32 & chick == 1)
	 	  			 {
	 	  				 state = 11 ;
	 	  			 }
	 	  			 break ;
	 	  		 case 10 :
	 	  			 if(ButtonMatrix == 32768 & chick == 1 ) // 0
	 	  			 {
	 	  				 Number[10] = 0 ;
	 	  				 state = 12 ;
	 	  			 }
	 	  			 else if(ButtonMatrix == 32768 & chick == 1)
	 	  			 {
	 	  				 state = 11 ;
	 	  			 }
	 	  			 break ;

	 	  		case 11:
	 	  			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,RESET);
	 	  			if (ButtonMatrix == 1 & chick == 1) {
	 	  				state = 0;

	 	  				Number[0] = 0;
	 	  				Number[1] = 0;
	 	  				Number[2] = 0;
	 	  				Number[3] = 0;
	 	  				Number[4] = 0;
	 	  				Number[5] = 0;
	 	  				Number[6] = 0;
	 	  				Number[7] = 0;
	 	  				Number[8] = 0;
	 	  				Number[9] = 0;
	 	  				Number[10] = 0;

	 	  			}
	 	  			break;
	 	  		case 12:

	 	  			if (ButtonMatrix == 8 & chick == 1) {
	 	  				fake = 1 ;
	 	  				state = 13 ;
	 	  			}
	 	  			else if(ButtonMatrix != 8 & chick == 1)
	 	  			{
	 	  				state = 11 ;
	 	  			}
	 	  			break ;
	 	  		case 13 :
	 	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);
	 	  			if(ButtonMatrix == 1 & chick == 1)
	 	  			{
	 	  				state = 11 ;
	 	  			}

	 	  			break ;

	 	  		}

	 	  	}
  }
  /* USER CODE END 3 */


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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R2_Pin|R4_Pin|R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin R1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L4_Pin L1_Pin */
  GPIO_InitStruct.Pin = L4_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L2_Pin */
  GPIO_InitStruct.Pin = L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R4_Pin R3_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R4_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L3_Pin */
  GPIO_InitStruct.Pin = L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(L3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ReadMatrixButton_1Row() {
	static uint8_t X = 0;
	register int i;
	for (i = 0; i < 4; i++) {
		if (HAL_GPIO_ReadPin(L[i].PORT, L[i].PIN)) {
			ButtonMatrix &= ~(1 << (X * 4 + i));
		} else {
			ButtonMatrix |= 1 << (X * 4 + i);
		}
	}
	HAL_GPIO_WritePin(R[X].PORT, R[X].PIN, 1);
	HAL_GPIO_WritePin(R[(X + 1) % 4].PORT, R[(X + 1) % 4].PIN, 0);
	X++;
	X %= 4;
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
