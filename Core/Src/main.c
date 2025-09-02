/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>// za sprintf
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;
uint32_t adcX, adcY;
char msg[100];
//broj ulaza
#define N 20
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
float getVoltageFromRaw(uint32_t adcVal);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Deo za pripremu podataka, citanje sa ulaza, racunanje delova formule suma

  uint32_t sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  uint32_t sumX2Y = 0, sumX3 = 0, sumX4 = 0;


  for (int i = 0; i < N; i++)
  {
	  // --- Čitanje X ---
	  HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
		  .Channel = ADC_CHANNEL_0, .Rank = 1, .SamplingTime = ADC_SAMPLETIME_12CYCLES_5});
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adcX = HAL_ADC_GetValue(&hadc1);

	  // --- Čitanje Y ---
	  HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
		  .Channel = ADC_CHANNEL_1, .Rank = 1, .SamplingTime = ADC_SAMPLETIME_12CYCLES_5});
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adcY = HAL_ADC_GetValue(&hadc1);

	  // --- Akumulacije ---
	  sumX  += adcX;
	  sumY  += adcY;
	  sumXY += (adcX * adcY);
	  sumX2 += (adcX * adcX);

	  sumX3 += adcX * adcX * adcX;
	  sumX4 += adcX * adcX * adcX * adcX;
	  sumX2Y += adcX * adcX * adcY;

	  HAL_Delay(100); // mali delay da smiri čitanje
  }

    // --- Izračunavanje koeficijenata ---

    // ----- Linearna regresija ----------
    float a_lin = ( (N * sumXY) - (sumX * sumY) ) / (float)( (N * sumX2) - (sumX * sumX) );
    float b_lin = ( (sumY - (a_lin * sumX)) / (float)N );

    // --- Slanje rezultata ---
    //sprintf(msg, "Linear regression: Y = %.3f * X + %.3f\r\n", a, b);

    int a_int = (int)(a_lin * 1000);  // 3 decimale
    int b_int = (int)(b_lin * 1000);  // 3 decimale

    sprintf(msg, "Linear regression: Y = %d.%03d * X + %d.%03d\r\n",
            a_int / 1000, a_int % 1000,
            b_int / 1000, b_int % 1000);


    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // --- Kvadratna regresija (Cramer) ---
     float A[3][3] = {
         {N,    sumX,   sumX2},
         {sumX, sumX2,  sumX3},
         {sumX2,sumX3,  sumX4}
     };
     float B[3] = {sumY, sumXY, sumX2Y};

     float detA =
             A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
           - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
           + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);

       float detC =
             B[0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
           - A[0][1]*(B[1]*A[2][2]-A[1][2]*B[2])
           + A[0][2]*(B[1]*A[2][1]-A[1][1]*B[2]);

       float detB =
             A[0][0]*(B[1]*A[2][2]-A[1][2]*B[2])
           - B[0]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
           + A[0][2]*(A[1][0]*B[2]-B[1]*A[2][0]);

       float detA2 =
             A[0][0]*(A[1][1]*B[2]-B[1]*A[2][1])
           - A[0][1]*(A[1][0]*B[2]-B[1]*A[2][0])
           + B[0]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);

       float c_quad = detC / detA;
       float b_quad = detB / detA;
       float a_quad = detA2 / detA;

       // Slanje rezultata, sada za polinomnu
       int aQ=(int)(a_quad*1000);
       int bQ=(int)(b_quad*1000);
       int cQ=(int)(c_quad*1000);

       sprintf(msg, "Quadratic: Y = %d.%03d*X^2 + %d.%03d*X + %d.%03d\r\n",
                 aQ/1000,
				 abs(aQ%1000),
				 bQ/1000,
				 abs(bQ%1000),
				 cQ/1000,
				 abs(cQ%1000));
       HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // LED ON/OFF
	  HAL_Delay(3000); // 500ms
	  char msg[] = "Hello from STM32  Marija Stefan Andjelina\r\n";
	     HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	     HAL_Delay(1000);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // --- Čitanje X (PA0 / ADC1_IN0) ---
	     /* HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
	          .Channel = ADC_CHANNEL_0,
	          .Rank = 1,
	          .SamplingTime = ADC_SAMPLETIME_12CYCLES_5
	      });
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	      adcX = HAL_ADC_GetValue(&hadc1);

	      // --- Čitanje Y (PA1 / ADC1_IN1) ---
	      HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
	          .Channel = ADC_CHANNEL_1,
	          .Rank = 1,
	          .SamplingTime = ADC_SAMPLETIME_12CYCLES_5
	      });
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	      adcY = HAL_ADC_GetValue(&hadc1);

	      // --- Slanje preko USART2 ---
	      sprintf(msg, "X=%d, Y=%d\r\n", adcX, adcY);
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	      HAL_Delay(3000); // ~5 puta u sekundi*/

	  ////////////////////////////////////////////////////////////////////////////////////////////////
	  //glavni deo predvidjanje i ispis
	  HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
	          .Channel = ADC_CHANNEL_0, .Rank = 1, .SamplingTime = ADC_SAMPLETIME_12CYCLES_5});
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adcX = HAL_ADC_GetValue(&hadc1);

	  HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
	          .Channel = ADC_CHANNEL_1, .Rank = 1, .SamplingTime = ADC_SAMPLETIME_12CYCLES_5});
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adcY = HAL_ADC_GetValue(&hadc1);

	  // Predikcije
	  float y_lin = a_lin*adcX + b_lin;
	  float y_quad = a_quad*adcX*adcX + b_quad*adcX + c_quad;
	  //    float y_pred = a * adcX + b;
	     // sprintf(msg, "X=%d -> Y_pred=%.2f\r\n", adcX, y_pred);
	  int yL_int = (int)(y_lin * 100); // skalirano na 2 decimale
	  int yQ_int = (int)(y_quad * 100);
/*
	      float voltageX = (adcX / 4095.0) * 3.3;  // ako je Vref = 3.3V
	      float voltageY = (adcY / 4095.0) * 3.3;
	      float voltageY_predicted = (y_int/4095.0)*3.3;
*/
	  float vX = getVoltageFromRaw(adcX);
	  float vY = getVoltageFromRaw(adcY);
	  float vLin = getVoltageFromRaw((uint32_t)yL_int);
	  float vQuad = getVoltageFromRaw((uint32_t)yQ_int);

	  sprintf(msg, "X=%d.%02d V, Y=%d.%02d V, Lin=%d.%02d, Quad=%d.%02d\r\n",
	          (int)vX, (int)((vX - (int)vX)*100),
	          (int)vY, (int)((vY - (int)vY)*100),
	          (int)vLin/100, (int)((vLin - (int)vLin)*100),
	          (int)vQuad/100, (int)((vQuad - (int)vQuad)*100));
/*
	      sprintf(msg, "X=%d, Y_real=%d, Y_lin_pred=%d.%02d, Y_quad=%d.%02d\r\n",
	    		  adcX,
				  adcY,                // realna vrednost sa ADC
				  yL_int/100, abs(yL_int%100),         // ceo deo
				  yQ_int/100, abs(yQ_int%100));        // decimalni deo
*/
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	      //staro ispod
	     /* int y_int = (int)(y_pred * 100); // skalirano na 2 decimale

	      sprintf(msg, "X=%d -> Y_pred=%d.%02d\r\n",
	              adcX,
	              y_int / 100,   // ceo deo
	              y_int % 100);  // decimalni deo
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);*/

	      HAL_Delay(500);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float getVoltageFromRaw(uint32_t adcVal) {
	return (adcVal / 4095.0) *3.3;
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
#ifdef USE_FULL_ASSERT
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
