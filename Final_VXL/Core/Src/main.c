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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"ws2812.h"
#include "stdlib.h"
#include "arm_math.h"
#include"ws2812_demos.h"
#include <stdio.h>
#define min(a,b) (((a)<(b))?(a):(b))
#define SamplesFFT 256
#define WS2812_NUM_LEDS 64
uint16_t colorValue[32]= {0x000F,0x03E0,0x03EF,0x7800,0x780F,0x7BE0,
		                  0xC618,0x001F,0x07E0,0x07FF,0xF800,0xF81F,
						  0xFFE0,0xFFFF,0xFD20,0XBC40,0XFC07,0x000F,
						  0x03E0,0x03EF,0x7800,0x780F,0x7BE0,0xC618,
						  0x001F,0x07E0,0x07FF,0xF800,0xF81F,0xFFE0,
						  0xFFFF,0xFD20};
uint32_t next_led = 0;
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adcValue[SamplesFFT];
float32_t input[SamplesFFT];
float32_t output[SamplesFFT];
uint32_t doConvert = SamplesFFT;
ws2812_handleTypeDef ws2812;
volatile uint8_t buffer_half = 0;         // Cờ nửa đầu buffer
volatile uint8_t buffer_full = 0;





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(doConvert >0) doConvert--;
}

// Done sending first half of the DMA buffer - this can now safely be updated
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {
	        buffer_half = 0; // Nửa đầu sẵn sàng cập nhật
	        ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[0]);
	    }
}

// Done sending the second half of the DMA buffer - this can now be safely updated
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {
	        buffer_full = 1; // Toàn bộ buffer đã truyền
	        buffer_half = 1; // Nửa sau sẵn sàng cập nhật
	        ws2812_update_buffer(&ws2812, &ws2812.dma_buffer[BUFFER_SIZE]);
	    }

}

void uart_debug(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}



void uart_debug_int(const char *label, int value) {
    char buffer[128];
    sprintf(buffer, "%s: %d\r\n", label, value);
    uart_debug(buffer);
}

void uart_debug_uint(const char *label, uint32_t value) {
    char buffer[128];
    sprintf(buffer, "%s: %lu\r\n", label, value);
    uart_debug(buffer);
}

void uart_debug_float(const char *label, float value) {
    char buffer[128];
    sprintf(buffer, "%s: %.4f\r\n", label, value);
    uart_debug(buffer);
}

void update_led_matrix(uint8_t *ledMatrix, ws2812_handleTypeDef *ws2812) {
    if (buffer_half == 0 || buffer_full == 1) { // Chỉ cập nhật khi buffer sẵn sàng
        zeroLedValues(ws2812);
        for (int col = 0; col < 8; col++) {
            for (int row = 0; row < 8; row++) {
                int ledIndex = col * 8 + row;
                setLedValues(ws2812, ledIndex, 0, 255, 0); // R=0, G=255, B=0 (màu xanh cố định)
            }
        }
        ws2812->is_dirty = 1;
        buffer_full = 0; // Đặt lại sau khi cập nhật
    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

   arm_rfft_fast_instance_f32 fftInstance;
   HAL_ADC_Start_DMA(&hadc1, adcValue, SamplesFFT);
   HAL_TIM_Base_Start_IT(&htim3);
   arm_rfft_fast_init_f32(&fftInstance, SamplesFFT);



       if (ws2812_init(&ws2812, &htim4, TIM_CHANNEL_1, WS2812_NUM_LEDS) != WS2812_Ok) {
           uart_debug("WS2812 Init Failed!\r\n");
           while (1);
       }
       uart_debug("Starting DMA...\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // LED debug
	   // HAL_Delay(500);
	 // uart_debug("UART Test OK\r\n");
	//  HAL_Delay(1000);
	  	 		if(doConvert == 0){
	  	 		  		doConvert = SamplesFFT;
	  	 		  		//FFT �?�?

	  	 		  		  	 		uart_debug("ADC Samples:");
	  	 		  		  	 		for (int i = 0; i < SamplesFFT; i+=8) {
	  	 		  		  	 		    char line[128];
	  	 		  		  	 		    int len = sprintf(line, "%4lu %4lu %4lu %4lu %4lu %4lu %4lu %4lu\r\n",
	  	 		  		  	 		                      adcValue[i], adcValue[i+1], adcValue[i+2], adcValue[i+3],
	  	 		  		  	 		                      adcValue[i+4], adcValue[i+5], adcValue[i+6], adcValue[i+7]);
	  	 		  		  	 		    HAL_UART_Transmit(&huart2, (uint8_t *)line, len, HAL_MAX_DELAY);
	  	 		  		  	 		}
	  	 		  		  	 float32_t dcOffset = 0.0f;
	  	 		  		  	 for (int i = 0; i < SamplesFFT; i++) {
	  	 		  		  	     dcOffset += (float32_t)adcValue[i];
	  	 		  		  	 }
	  	 		  		  	 dcOffset /= SamplesFFT;
	  	 		  		  	 for (int i = 0; i < SamplesFFT; i++) {
	  	 		  		  	     if (adcValue[i] >= 4095 || adcValue[i] <= 0) {
	  	 		  		  	         input[i] = 0.0f; // Bỏ qua giá trị rác hoặc bão hòa
	  	 		  		  	     } else {
	  	 		  		  	         input[i] = ((float32_t)adcValue[i] - dcOffset) / 2048.0f; // Chuẩn hóa [-1, 1]
	  	 		  		  	     }
	  	 		  		  	 }



	  	 		  	 uart_debug("Input for FFT:\r\n");
	  	 		  for (int i = 0; i < min(SamplesFFT, 64); i += 8) {
	  	 		                  char line[128];
	  	 		                  snprintf(line, sizeof(line), "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
	  	 		                           input[i], input[i+1], input[i+2], input[i+3],
	  	 		                           input[i+4], input[i+5], input[i+6], input[i+7]);
	  	 		                  if (HAL_UART_Transmit(&huart2, (uint8_t *)line, strlen(line), 1000) != HAL_OK) {
	  	 		                      // Xử lý lỗi
	  	 		                  }
	  	 		              }


	  	 			  		arm_rfft_fast_f32(&fftInstance, input, output, 0);



	  	 		             // Tính độ lớn
	  	 			  	float32_t magnitude[SamplesFFT/2];
	  	 			  	arm_cmplx_mag_f32(output, magnitude, SamplesFFT/2);

	  	 			 uart_debug("FFT Output (Magnitude):\r\n");
	  	 			         for (int i = 0; i < min(SamplesFFT/2, 16); i += 4) {
	  	 			             char line[64];
	  	 			             snprintf(line, sizeof(line), "%.4f %.4f %.4f %.4f\r\n",
	  	 			                      magnitude[i], magnitude[i+1], magnitude[i+2], magnitude[i+3]);
	  	 			             HAL_UART_Transmit(&huart2, (uint8_t *)line, strlen(line), 1000);
	  	 			         }

	  	 		float32_t globalMax = 0;
	  	 		        for (int i = 0; i < SamplesFFT/2; i++) {
	  	 		            if (magnitude[i] > globalMax) globalMax = magnitude[i];
	  	 		        }

	  	 		        // Ánh xạ phổ vào ma trận LED
	  	 		     uint8_t ledMatrix[8] = {0};
	  	 		        for (int band = 0; band < 8; band++) {
	  	 		            float32_t maxMagnitude = 0.0;
	  	 		            for (int i = band * 16; i < (band + 1) * 16 && i < SamplesFFT/2; i++) {
	  	 		                if (magnitude[i] > maxMagnitude) maxMagnitude = magnitude[i];
	  	 		            }
	  	 		            ledMatrix[band] = (uint8_t)((maxMagnitude / (globalMax + 1e-6)) * 8.0); // Chuẩn hóa
	  	 		            if (ledMatrix[band] > 8) ledMatrix[band] = 8;
	  	 		        }

	  	 		        // In giá trị cho ma trận LED (debug)
	  	 		        uart_debug("LED Matrix Columns (0-8):\r\n");


	  	 		        char line[256];
	  	 		        snprintf(line, sizeof(line), "%d %d %d %d %d %d %d %d\r\n",
	  	 		                 ledMatrix[0], ledMatrix[1], ledMatrix[2], ledMatrix[3],
	  	 		                 ledMatrix[4], ledMatrix[5], ledMatrix[6], ledMatrix[7]);
	  	 		        if (HAL_UART_Transmit(&huart2, (uint8_t *)line, strlen(line), 1000) != HAL_OK) {
	  	 		            uart_debug("UART Transmit Error\r\n");
	  	 		        }


	  	 		  		//출력 결과�? 128개로 분할
	  	 		//   update_led_matrix(ledMatrix, &ws2812);
	  	 		 ws2812_demo_task(&ws2812, HAL_GetTick(), &next_led, ledMatrix);

	  	 		  	  uart_debug("Spectrum displayed to matrix\r\n");
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 104;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  GPIO_InitStruct.Pin = GPIO_PIN_0; // PA0 cho ADC_CHANNEL_0
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Kéo xuống
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        doConvert = 0; // Đánh dấu đã thu thập đủ mẫu
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
