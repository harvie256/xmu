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
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "les_algorithm.h"

#include "decimation_filter.h"
#include "deci_filter_correction_header.h"

#include <string.h>
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

/* USER CODE BEGIN PV */

// ADC DMA Circular buffer
#define DMA_SAMPLE_BUFFER_LENGTH (16*8)
uint32_t adc_dma_buffer[DMA_SAMPLE_BUFFER_LENGTH] __attribute__((section(".AdcDmaBufSection"))); // Note this must be aligned on a 32byte boundary so we can invalidate the cache lines.

// ADC Main Circular Buffer
#define MAIN_SAMPLE_BUFFER_LENGTH (2048) // samples - 32bit ints
int32_t main_sample_buffer[8][MAIN_SAMPLE_BUFFER_LENGTH];// __attribute__((section(".ccmram-samplebuffer"))) ;

volatile uint32_t main_sample_buffer_head = 0;

volatile uint8_t f_buffer_loop = 0;
volatile uint8_t f_run_main_loop = 0;
volatile uint8_t f_reporting_cycle = 0;

volatile int32_t les_count = 0;
les_fundamental_output_t les_fundamental_output;
les_harmonics_output_t les_harmonics_output;

struct udp_pcb* upcb;

volatile uint32_t pps_timer_capture = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ETH_HandleTypeDef heth;


void start_adc_dma_reception(void){

	// wait to go low
	while((GPIOA->IDR & GPIO_PIN_4))
		  ;
	HAL_GPIO_WritePin(DEBUG_PIN_1_GPIO_Port, DEBUG_PIN_1_Pin,SET);

	// wait to go high again
	while(!(GPIOA->IDR & GPIO_PIN_4))
		  ;
	HAL_GPIO_WritePin(DEBUG_PIN_1_GPIO_Port, DEBUG_PIN_1_Pin,RESET);

	  do {
		  hspi3.Instance->RXDR;
	  } while(__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE) != RESET);

	HAL_SPI_Receive_DMA(&hspi3, (uint8_t*)adc_dma_buffer, 2*DMA_SAMPLE_BUFFER_LENGTH);
}

void dump_main_buffer(void){
	HAL_SPI_DMAStop(&hspi3);
	__NOP();
//	  uint8_t data[128];
//	  char* ptr = (char*)(data);
//	  for(int a = 0; a <sizeof(data); a++)
//		  ptr[a] = a;

	struct pbuf* p;

	uint16_t sample_size = MAIN_SAMPLE_BUFFER_LENGTH * sizeof(int32_t);
	uint16_t no_transmits = sample_size / 1024;

	uint8_t startMsg[] = {0xFF, 0xAA};
	uint8_t stopMsg[] = {0x55, 0x55};

	// Send the start sequence
	p = pbuf_alloc(PBUF_TRANSPORT, 2, PBUF_RAM);
	memcpy(p->payload, &startMsg, 2);
	udp_send(upcb, p);
	pbuf_free(p);

	// Send the main data
	for(int i = 0; i < no_transmits; i++){
		p = pbuf_alloc(PBUF_TRANSPORT, 1024, PBUF_RAM);
		memcpy(p->payload, ((uint8_t*)(main_sample_buffer[5])) + i*1024, 1024);
		udp_send(upcb, p);
		pbuf_free(p);
	}

	// Send the end sequence
	p = pbuf_alloc(PBUF_TRANSPORT, 2, PBUF_RAM);
	memcpy(p->payload, &stopMsg, 2);
	udp_send(upcb, p);
	pbuf_free(p);

	start_adc_dma_reception();
}

void cpy_buffer_int32(uint8_t* buffer, int32_t value){
	buffer[0] = (value >> 24) & 0xFF;
	buffer[1] = (value >> 16) & 0xFF;
	buffer[2] = (value >> 8) & 0xFF;
	buffer[3] = value & 0xFF;
}

void send_les_value(les_fundamental_output_t *fundamental_value, les_harmonics_output_t* harmonics_value){
	struct pbuf* p;

	uint8_t buffer[12 + 32];
	cpy_buffer_int32(buffer, fundamental_value->frequency);
	cpy_buffer_int32(buffer+4, fundamental_value->fundamental_mag[0]);
	cpy_buffer_int32(buffer+8, fundamental_value->phase[0]);

	for(uint8_t i=0; i<8; i++){
		cpy_buffer_int32(buffer+12+(4*i), harmonics_value->harmonic_mag[i]);
	}

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(buffer), PBUF_RAM);
	memcpy(p->payload, buffer, sizeof(buffer));
	udp_send(upcb, p);
	pbuf_free(p);

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim5){
		pps_timer_capture = htim5.Instance->CCR1;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_PLL2_DISABLE();
  __HAL_RCC_PLL2CLKOUT_ENABLE(RCC_PLL2_DIVP);
  __HAL_RCC_PLL2_ENABLE();
  HAL_Delay(100);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
//  ETH_MACFilterConfigTypeDef filterConfig;
//  HAL_ETH_GetMACFilterConfig(&heth, &filterConfig);
//  filterConfig.BroadcastFilter = DISABLE;
//  filterConfig.PromiscuousMode = ENABLE;
//  HAL_ETH_SetMACFilterConfig(&heth, &filterConfig);
//  HAL_ETH_GetMACFilterConfig(&heth, &filterConfig);

// From the timer testing
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  htim1.Instance->CCER |= 0x0001; // Set the CC1E Bit

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_Delay(10);
  start_adc_dma_reception();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Allocate the pcb for the UDP transfers.
  	upcb = udp_new();
	ip_addr_t destAddr;
	IP4_ADDR(&destAddr, 192, 168, 1, 99);
	uint16_t destprt = 60001;

	udp_connect(upcb, &destAddr, destprt);

  // Initialise the LES filters
  les_init();

//  uint16_t buffer_count = 0;
  while (1)
  {
		MX_LWIP_Process();

//		if(f_buffer_loop){
//			f_buffer_loop = 0;
//			buffer_count++;
//			if(buffer_count > 10){
//				dump_main_buffer();
//				buffer_count = 0;
//			}
//		}
		/* The LES algorithm */
		if(f_run_main_loop){
			f_run_main_loop = 0;

			if(f_reporting_cycle){

				HAL_GPIO_WritePin(DEBUG_PIN_1_GPIO_Port, DEBUG_PIN_1_Pin, RESET);

				if(les_calculate_harmonics(&les_harmonics_output)){
					__NOP();
					// Need to ignore the output since it wasn't calculated.
				}

				// Correct for attenuation through the biquad filters
				for(uint8_t i=0; i<8; i++){
					les_harmonics_output.harmonic_mag[i] = deci_correct_harmonic_mag(les_harmonics_output.harmonic_mag[i], i+1);
				}
				les_fundamental_output.fundamental_mag[0] = deci_correct_harmonic_mag(les_fundamental_output.fundamental_mag[0], 0);

				send_les_value(&les_fundamental_output, &les_harmonics_output);

				HAL_GPIO_WritePin(DEBUG_PIN_1_GPIO_Port, DEBUG_PIN_1_Pin, SET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 8;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLL2PCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void inc_main_buffer_head(void){
	main_sample_buffer_head++;
	if(main_sample_buffer_head >= MAIN_SAMPLE_BUFFER_LENGTH){
		main_sample_buffer_head = 0;
		f_buffer_loop = 1;
	}
}


void transfer_samples_to_main_buffer(uint32_t* dma_buffer){
	int32_t tmp;
	int32_t decimated_samples[3][8];

	// transfer the first set of samples into the main buffer
	for(int j = 0; j < 8; j++){ // j is the sample number
		for(int i = 0; i < 8; i++){ // i is the channel
			// Transfer from DMA buffer
			tmp = dma_buffer[i+(8*j)];
			tmp = ((tmp & 0x0000FFFF) << 16) | ((tmp & 0xFFFF0000) >> 16); // Swap upper and lower 16bit words
			tmp = (tmp << 8) >> 8; // Remove the CRC byte and sign extend the result

			main_sample_buffer[i][main_sample_buffer_head] = tmp; // Store into main buffer

			// Channels 5, 6 & 7 are L1, L2 & L3 respectively
			if(i >= 5){
				uint8_t deci_channel = i - 5;
				decimated_samples[deci_channel][j] = filter_sample(tmp, deci_channel);

				if(j == 7){
					// this will be the latest set of samples
					les_set_sample_buffer_value(deci_channel, decimated_samples[deci_channel][j]);
				}
			}


		}

		inc_main_buffer_head();
	}
	les_inc_buffer_head();

	les_count++;

//	// Work out if it's going to be an reporting cycle or not, as the magnitude needs to be calculate for this step.
	f_reporting_cycle = (les_count % 32 == 0) ? 1 : 0;

	run_les(f_reporting_cycle, &les_fundamental_output);

	f_run_main_loop = 1;
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)(adc_dma_buffer + (DMA_SAMPLE_BUFFER_LENGTH >> 1)), DMA_SAMPLE_BUFFER_LENGTH << 1);
	transfer_samples_to_main_buffer(adc_dma_buffer + (DMA_SAMPLE_BUFFER_LENGTH >> 1));
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	// The main sample buffer length is in 32b words, and we want to invalidate half of it, so multiply by 4, divide by 2 == << 1.
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_dma_buffer, DMA_SAMPLE_BUFFER_LENGTH << 1);
	transfer_samples_to_main_buffer(adc_dma_buffer);
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
