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
#include "mcp4728.h"
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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

dacChannelConfig config;
dacChannelConfig output;
dacChannelConfig channels;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef MCP4728_Write_VRef_Select(I2C_HandleTypeDef *I2CHandler, dacChannelConfig config){
	uint8_t data = config.channelVref | (MCP4728_CMD_VREFWRITE);
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, data, sizeof(data), HAL_MAX_DELAY);
	return ret;
}

HAL_StatusTypeDef MCP4728_Write_PWRDWN_Select(I2C_HandleTypeDef *I2CHandler, uint8_t command){
	uint8_t data[2];
	data[0] = MCP4728_CMD_PWRDWNWRITE | command<<2 | command;
	data[1] = (command << 6 |command << 4) & 0xF0 ;
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, data, sizeof(data), HAL_MAX_DELAY);
	return ret;
}

void MCP4728_Write_GeneralCall(I2C_HandleTypeDef *I2CHandler, uint8_t command)
{
	uint16_t zeoo = 0x00;
	uint16_t sizey = 0x1;
	HAL_I2C_Master_Transmit(I2CHandler, zeoo, &command, sizey, HAL_MAX_DELAY);

}

void MCP4728_Write_AllChannels_Same(I2C_HandleTypeDef *I2CHandler, uint16_t output)
{
	uint8_t buf[8];
	uint8_t lowByte = output & 0xff;
	uint8_t highByte = (output >> 8);
	for(int i = 0; i<8; i = i+2){
		buf[i] = 0x0f&highByte;
		buf[i+1] = lowByte;
	}
	HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY);
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_SWUPDATE);
}

void MCP4728_Write_AllChannels_Diff(I2C_HandleTypeDef *I2CHandler, dacChannelConfig output)
{
	uint8_t buf[8];
	for(uint8_t i = 0; i < 4; i++){
		uint8_t lowByte = output.channel_Val[i] & 0xff;
		uint8_t highByte = (output.channel_Val[i] >> 8);
		buf[i*2] =  0x0f&highByte;
		buf[(i*2)+1] = lowByte;
	}
	HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY);
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_SWUPDATE);
}

void MCP4728_Write_SingleChannel(I2C_HandleTypeDef *I2CHandler, uint8_t channel, uint16_t output)
{
	uint8_t buf[3];
	uint8_t lowByte = output & 0xff;
	uint8_t highByte = (output >> 8);
	buf[0] = MCP4728_CMD_DACWRITE_SINGLE | (channel<<1);
	buf[1] = (0<<7) | (channel<<5) | highByte;
	buf[2] = lowByte;
	HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY);
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_SWUPDATE);
}

void MCP4728_Init(I2C_HandleTypeDef *I2CHandler, dacChannelConfig output){
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_RESET);
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_WAKEUP);
	//MCP4728_Write_GeneralCall(I2CHandler, 0x0C);

	uint8_t buf[9];
	buf[0] = MCP4728_CMD_DACWRITE_SEQ;
	for(uint8_t i = 1; i <= 4; i++){
		buf[(i*2)+1] = 0x00;
		buf[(i*2)] = (0 << 7) | ((i-1)<<4) | 0x0;
	}

	HAL_I2C_Master_Transmit(I2CHandler, MCP4728_BASEADDR, buf, sizeof(buf), HAL_MAX_DELAY);
	//HAL_I2C_Master_Transmit(I2CHandler, 0xC8, buf, sizeof(buf), HAL_MAX_DELAY);
	//C0, C2, C4, C6, C8, CA, CC, CE
	//60, 61, 62, 63, 64, 65, 66, 67
	MCP4728_Write_GeneralCall(I2CHandler,MCP4728_GENERAL_SWUPDATE);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t outy = 0xC80; //3200 = 2.34 V

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
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  MCP4728_Init(&hi2c2, output);
  output.channelVref = 0x00;
  output.channel_Gain = 0x00;
  output.channel_Val[0] = 0xFFF;
  output.channel_Val[1] = 0x800;
  output.channel_Val[2] = 0x400;
  output.channel_Val[3] = 0x00;

  MCP4728_Write_AllChannels_Diff(&hi2c2, output);
  //MCP4728_Write_AllChannels_Same(&hi2c2, 2048);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
/*	  MCP4728_Write_SingleChannel(&hi2c2, MCP4728_CHANNEL_A, 4095); //3
	  MCP4728_Write_SingleChannel(&hi2c2, MCP4728_CHANNEL_B, 2048); //1.5
	  MCP4728_Write_SingleChannel(&hi2c2, MCP4728_CHANNEL_C, 1024);//0.75
	  MCP4728_Write_SingleChannel(&hi2c2, MCP4728_CHANNEL_D, 0); //0*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x2000090E;
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

 /* I2C2->TIMINGR = 0;
   I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC; //set prescaler to 0
   I2C2->TIMINGR |= 3 << 20; //scldel field to 3
   I2C2->TIMINGR |= 1 << 16; //sdadel field to 1
   I2C2->TIMINGR |= 3 << 8; // sclh to 3
   I2C2->TIMINGR |= 9 << 0; //scll to 9*/

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

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
