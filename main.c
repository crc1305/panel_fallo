/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "math.h"
#include "string.h"

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int PM1=0; //PA0
int PM2=0; //PA1
int PM3=0; //PA2
int PM4=0; //PA3
int PM5=0; //PA4
int PM6=0; //PA5
int PM7=0; //PA6
int PM8=0; //PA7
int PM9=0; //PC4
int PM10=0; //PC5
int PM11=0; //PB0
int PM12=0; //PB1
int PM13=0; //PE7
int PM14=0; //PE8
int PM15=0; //PE9
int PM16=0; //PE10
int PM17=0; //PE11
int PM18=0; //PE12
int PM19=0; //PE13
int PM20=0; //PE14
int PM21=0; //PE15
int PM22=0; //PB10
int PM23=0; //PB11
int PM24=0; //PB12
int PM25=0; //PB13
int PM26=0; //PB14
int PM27=0; //PB15
int PM28=0; //PD8
int PM29=0; //PD9
int PM30=0; //PD10
int PM31=0; //PD11
int PM32=0; //PD12
int PM33=0; //PD13
int PM34=0; //PD14
int PM35=0; //PD15
int PM36=0; //PC6
int v = 1766;
int *verde = &v;
int r = 63488;
int *rojo = &r;
int a = 65504;
int *amarillo = &a;
int PM[36];
int cont[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char tag[36][6] = {"t0","t1","t2","t3","t4","t5","t6","t7","t8","t9","t10","t11","t12","t13","t14","t15","t16","t17","t18","t19","t20","t21","t22","t23","t24","t25","t26","t27","t28","t29","t30","t31","t32","t33","t34","t35"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd_end[3]={0xFF,0xFF,0xFF};  //comando final de secuencia
uint8_t rx_data[4];


void NextionSend(char * ID, int * color){
	char buf[50];
	int len = sprintf(buf,"%s.bco=\%d",ID,color);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, cmd_end, sizeof(cmd_end), HAL_MAX_DELAY);


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart1, rx_data, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  PM[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  PM[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	  PM[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	  PM[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	  PM[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	  PM[5] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	  //PM[6] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	  PM[7] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	  PM[8] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
	  PM[9] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	  PM[10] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	  PM[11] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	  PM[12] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
	  PM[13] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
	  PM[14] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	  PM[15] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10);
	  PM[16] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	  PM[17] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	  PM[18] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	  PM[19] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
	  PM[20] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
	  PM[21] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	  PM[22] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	  PM[23] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	  PM[24] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	  PM[25] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	  PM[26] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	  PM[27] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
	  PM[28] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
	  PM[29] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	  PM[30] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);
	  PM[31] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	  PM[32] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
	  PM[33] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
	  PM[34] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
	  PM[35] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	  //HAL_UART_Receive(&huart1, rx_data, 4, HAL_MAX_DELAY);

	  for(int i=0;i<5;i++){
		  if(PM[i]!=0){
			  if(cont[i]==0){

				  NextionSend(tag+i,*rojo); //rojo
				  cont[i]=1;

			  }
			  else if(cont[i]==1){
				  NextionSend(tag+i,*amarillo); //amarillo
				  cont[i]=0;
			  }
		  }
		  else NextionSend(tag+i,*verde);//verde
	  }



//HAL_UART_Receive(&huart1, rx_data, 4, 500);




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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(rx_data[2]== 0x02){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
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
