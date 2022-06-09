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
#include "stdlib.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "SSD1306.h"
#include "mmc_sd.h"
#include "fatfs.h"
#include "fops.h"
#include "File_Handling.h"
#include "waveplayer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int OLED_display_status = OLED_MENU;
int menu_select = MENU_FILE_BROWSER;
int globalTime = 0;
int lastEventTime = 0;

extern AUDIO_PLAYBACK_StateTypeDef AudioState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SD_init() {
	// One line of LCD contains 21 character
	char buf[22];
	sprintf(buf, "Loading SD card...");
	LCD_FStr(buf, 0, 0);
	LCD_Update();

	// Wait 1 second before loading SD card
	HAL_Delay(1000);

	// Clear LCD before showing anything
	LCD_Clear();

	sprintf(buf, "SD sectorcount\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);
 	sprintf(buf, "%u\n\r", SD_GetSectorCount());
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);

	MX_FATFS_Init();

	LCD_Clear();
	int mount = exf_mount();
	sprintf(buf, "Mount (%i)\n\r", mount);
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);
	sprintf(buf, "SD card mount");
	LCD_FStr(buf, 0, 0);
	sprintf(buf, "%s", mount == FR_OK ? "Success" : "Fail");
	LCD_FStr(buf, 0, 1);
	LCD_Update();
	HAL_Delay(1000);

	LCD_Clear();
	unsigned free = exf_getfree();
	sprintf(buf, "Free (%i)\n\r", free);
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);
	sprintf(buf, "SD card free space");
	LCD_FStr(buf, 0, 0);
	sprintf(buf, "%u MB", free);
	LCD_FStr(buf, 0, 1);
	LCD_Update();
	HAL_Delay(1000);
	return;
}
;

void OLED_task(void *pvParameters) {
	char buf[22];
	while (1) {
		if (OLED_display_status == OLED_MENU) {
			// Clear OLED before displaying stuff
			LCD_Clear();
			sprintf(buf, "%c %s",
					(menu_select == MENU_FILE_BROWSER) ? '>' : ' ',
					"File Browser");
			LCD_FStr(buf, 0, 0);
			sprintf(buf, "%c %s", (menu_select == MENU_PLAYER) ? '>' : ' ',
					"Player");
			LCD_FStr(buf, 0, 1);
			sprintf(buf, "%c %s", (menu_select == MENU_SETTING) ? '>' : ' ',
					"Setting");
			LCD_FStr(buf, 0, 2);
			sprintf(buf, "%c %s", (menu_select == MENU_ABOUT) ? '>' : ' ',
					"About");
			LCD_FStr(buf, 0, 3);
			LCD_Update();
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			LCD_Clear();
			sprintf(buf, "File Browser");
			LCD_FStr(buf, 0, 0);
			sprintf(buf, "Placeholder");
			LCD_FStr(buf, 0, 1);
			LCD_Update();
		} else if (OLED_display_status == OLED_PLAYER) {
			LCD_Clear();
			sprintf(buf, "Player");
			LCD_FStr(buf, 0, 0);
			sprintf(buf, "Placeholder");
			LCD_FStr(buf, 0, 1);
			LCD_Update();
		} else if (OLED_display_status == OLED_SETTING) {
			LCD_Clear();
			sprintf(buf, "Setting");
			LCD_FStr(buf, 0, 0);
			sprintf(buf, "Placeholder");
			LCD_FStr(buf, 0, 1);
			LCD_Update();
		} else if (OLED_display_status == OLED_ABOUT) {
			LCD_Clear();
			sprintf(buf, "AudioPlayer");
			LCD_FStr(buf, 0, 0);
			sprintf(buf, "NCKU CSIE EOS project");
			LCD_FStr(buf, 0, 1);
			sprintf(buf, "2022");
			LCD_FStr(buf, 0, 2);
			sprintf(buf, "Press any key to menu");
			LCD_FStr(buf, 0, 3);
			LCD_Update();
		}
		vTaskDelay(1000);
	}
	return;
}
;

void timer_task(void *pvParameters) {
	while (1) {
		globalTime++;
		vTaskDelay(1000);
	}
	return;
}

void audio_task(void * pvParameters){
	for(;;){
		int isFinished = 0;
		AUDIO_PLAYER_Start(0);

		while(!isFinished){
			AUDIO_PLAYER_Process(pdTRUE);

			if(AudioState == AUDIO_STATE_STOP){
				isFinished = 1;
			}
		}
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();

	// Init SD card here
	SD_init();

//	{
//		char buf[256];
//		DIR dir;
//		FRESULT res = FR_OK;
//		FILINFO fno;
//		char *fn;
//		res = f_opendir(&dir, USERPath);
//
//		sprintf(buf, "f_opendir: (%i)\n\r", res);
//		HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);
//
//		if (res == FR_OK) {
//			while (1) {
//				res = f_readdir(&dir, &fno);
//				// If read failed, then quit
//				if (res != FR_OK || fno.fname[0] == 0)
//					break;
//				// If file should be hidden, then don't show the file
//				if (fno.fname[0] == '.' || fno.fname[0] == '_')
//					continue;
//				fn = fno.fname;
//				if ((strstr(fn, "wav")) || (strstr(fn, "WAV"))) {
//					sprintf(buf, "file: %s\n\r", fn);
//					HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf),
//							0xffff);
//				}
//			}
//		}
//	}
//
//	while (1);
//	// Start task here
//	xTaskCreate(OLED_task, "OLED_task", STACK_SIZE, (void*) NULL, 10, NULL);
//	xTaskCreate(timer_task, "timer_task", STACK_SIZE, (void*) NULL, 1, NULL);
	xTaskCreate(audio_task, "audio_task", STACK_SIZE, (void*) NULL, 15, NULL);

	// Start scheduler here
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDcard_CS_GPIO_Port, SDcard_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SDcard_CS_Pin */
  GPIO_InitStruct.Pin = SDcard_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDcard_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((globalTime - lastEventTime) < 1)
		return;
	// Start interrupt event
	char buf[256];
	int button = -1;
	switch (GPIO_Pin) {
	case GPIO_PIN_0:
		button = 0;
		if (OLED_display_status == OLED_MENU) {
			// Do nothing when at menu
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_PLAYER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_SETTING) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_ABOUT) {
			// Press any key to return to menu
			OLED_display_status = OLED_MENU;
		}
		break;
	case GPIO_PIN_1:
		button = 1;
		if (OLED_display_status == OLED_MENU) {
			switch (menu_select) {
			case MENU_FILE_BROWSER:
				OLED_display_status = OLED_FILE_BROWSER;
				break;
			case MENU_PLAYER:
				OLED_display_status = OLED_PLAYER;
				break;
			case MENU_SETTING:
				OLED_display_status = OLED_SETTING;
				break;
			case MENU_ABOUT:
				OLED_display_status = OLED_ABOUT;
				break;
			default:
				break;
			}
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_PLAYER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_SETTING) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_ABOUT) {
			// Press any key to return to menu
			OLED_display_status = OLED_MENU;
		}
		break;
	case GPIO_PIN_2:
		button = 2;
		if (OLED_display_status == OLED_MENU) {
			// Do nothing
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_PLAYER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_SETTING) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_ABOUT) {
			// Press any key to return to menu
			OLED_display_status = OLED_MENU;
		}
		break;
	case GPIO_PIN_3:
		button = 3;
		if (OLED_display_status == OLED_MENU) {
			if (menu_select != MENU_FILE_BROWSER)
				menu_select--;
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_PLAYER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_SETTING) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_ABOUT) {
			// Press any key to return to menu
			OLED_display_status = OLED_MENU;
		}
		break;
	case GPIO_PIN_4:
		button = 4;
		if (OLED_display_status == OLED_MENU) {
			if (menu_select != MENU_ABOUT)
				menu_select++;
		} else if (OLED_display_status == OLED_FILE_BROWSER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_PLAYER) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_SETTING) {
			// When at file browser
			// Press first button to go back to menu
			OLED_display_status = OLED_MENU;
		} else if (OLED_display_status == OLED_ABOUT) {
			// Press any key to return to menu
			OLED_display_status = OLED_MENU;
		}
		break;
	default:
		return;
		break;
	}
	sprintf(buf, "External IO button %d pressed!\n\r", button);
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 0xffff);

	// Update lastEventTime
	lastEventTime = globalTime;
	return;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
	__disable_irq();
	while (1) {
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
