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
#include "lcd.h"
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
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setup_bb()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    //GPIOB->MODER &= ~(0xcf000000);
    GPIOB->MODER &= ~(GPIO_MODER_MODER12|GPIO_MODER_MODER13|GPIO_MODER_MODER15);
    GPIOB->MODER |= (GPIO_MODER_MODER12_0|GPIO_MODER_MODER13_0|GPIO_MODER_MODER15_0);
    //GPIOB->MODER |= 0x45000000; //set pb12,13,15 as gp output
    //GPIOB->ODR |= 0x1000; //set nss high pb 12
    GPIOB->ODR |= GPIO_ODR_12;
    //GPIOB->ODR &= ~(0x2000); //set sck low pb 13
    GPIOB->ODR &= ~GPIO_ODR_13;
    return;
}

void small_delay()
{
    nano_wait(10000);
    return;
}

void bb_write_bit(int inner)
{
    if (inner == 0)
    {
        GPIOB->ODR &= ~GPIO_ODR_15;
    }
    else
    {
        GPIOB->ODR |= GPIO_ODR_15;
    }
    //GPIOB->ODR |= inner<<14; //setting MOSI PB15 to inner
    small_delay();
    //GPIOB->ODR |= 0x2000; //sck pb13 to high
    GPIOB->ODR |= GPIO_ODR_13;
    small_delay();
    //GPIOB->ODR &= ~0x2000; //sck pb13 to low
    GPIOB->ODR &= ~GPIO_ODR_13;
    return;
}

void bb_write_byte(int inner)
{
    //bit 7:
    int new = inner & (0x80);
    int x = new>>7;
    bb_write_bit(x);
    //bit 6:
    int new1 = inner & (0x40);
    int x1 = new1 >> 6;
    bb_write_bit(x1);
    //bit 5:
    int new2 = inner & (0x20);
    int x2 = new2>>5;
    bb_write_bit(x2);
    //bit 4:
    int new3 = inner & (0x10);
    int x3 = new3 >> 4;
    bb_write_bit(x3);
    //bit 3:
    int new4 = inner & (0x8);
    int x4 = new4>>3;
    bb_write_bit(x4);
    //bit 2:
    int new5 = inner & (0x4);
    int x5 = new5 >> 2;
    bb_write_bit(x5);
    //bit 1:
    int new6 = inner & (0x2);
    int x6 = new6>>1;
    bb_write_bit(x6);
    //bit 0:
    int new7 = inner & (0x1);
    int x7 = new7;
    bb_write_bit(x7);
    return;
}

void bb_cmd(int inner)
{
    //GPIOB->ODR &= ~0x1000; //set nss low pb 12
    GPIOB->ODR &= ~GPIO_ODR_12;
    small_delay();
    bb_write_bit(0);
    bb_write_bit(0);
    bb_write_byte(inner);
    small_delay();
   // GPIOB->ODR |= 0x1000; //set nss high pb 12
    GPIOB->ODR |= GPIO_ODR_12;
    small_delay();
    return;
}

void bb_data(int inner)
{
    //GPIOB->ODR &= ~0x1000; //set nss low pb 12
    GPIOB->ODR &= ~GPIO_ODR_12;
    small_delay();
    bb_write_bit(1);
    bb_write_bit(0);
    bb_write_byte(inner);
    small_delay();
    GPIOB->ODR |= 0x1000; //set nss high pb 12
    small_delay();
    return;
}

void bb_init_oled()
{
    nano_wait(1000000);//1ms
    bb_cmd(0x38); // set for 8-bit operation
    bb_cmd(0x08); // turn display off
    bb_cmd(0x01); // clear display
    nano_wait(2000000); //2ms
    bb_cmd(0x06); // set the display to scroll
    bb_cmd(0x02); // move the cursor to the home position
    bb_cmd(0x0c); // turn the display on
    return;
}

void bb_display1(const char * stringy)
{
    bb_cmd(0x02); // move the cursor to the home position
    for(int i=0; stringy[i] != '\0'; i++)
    {
            bb_data(stringy[i]);
    }
    return;
}

void bb_display2(const char * stringy)
{
    bb_cmd(0xc0); // move the cursor to the home position
    for(int i=0; stringy[i] != '\0'; i++)
    {
        bb_data(stringy[i]);
    }
    return;
}

void setup_spi2()
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~0xcf000000;
    GPIOB->MODER |= 0x8a000000; //set pb12,13,15 as alternate
    //GPIOB->AFR[1] = 0; 12, 13, and 15
    SPI2->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR;
    SPI2->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP; //10  bit size
    SPI2->CR1 |= SPI_CR1_SPE; //enable
    return;
}

void spi_cmd(int inner)
{
    while((SPI2->SR & SPI_SR_TXE) == 0)
                ; // wait for the transmit buffer to be empty
        SPI2->DR = inner;
    return;
}

void spi_data(int inner)
{
    while((SPI2->SR & SPI_SR_TXE) == 0)
                   ; // wait for the transmit buffer to be empty
    SPI2->DR = (inner | 0x200);
}

void spi_init_oled()
{
    nano_wait(1000000);//1ms
    spi_cmd(0x38); // set for 8-bit operation
    spi_cmd(0x08); // turn display off
    spi_cmd(0x01); // clear display
    nano_wait(2000000); //2ms
    spi_cmd(0x06); // set the display to scroll
    spi_cmd(0x02); // move the cursor to the home position
    spi_cmd(0x0c); // turn the display on
    return;
}

void spi_display1(const char * stringy)
{
    spi_cmd(0x02); // move the cursor to the home position
    for(int i=0; stringy[i] != '\0'; i++)
    {
            spi_data(stringy[i]);
    }
    return;
}

void spi_display2(const char * stringy)
{
    spi_cmd(0xc0); // move the cursor to the home position
    for(int i=0; stringy[i] != '\0'; i++)
    {
            spi_data(stringy[i]);
    }
    return;
}

void spi_setup_dma(const short* shorty)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; //enable rcc to dma
    DMA1_Channel5->CPAR = &(SPI2->DR);
    DMA1_Channel5->CMAR = shorty;
    DMA1_Channel5->CNDTR = 34;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel5->CCR |= 0x400; //set to 16 bits msize
    DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5->CCR |= 0x100; //set to 16 bits mpize
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
    SPI2->CR2 |= SPI_CR2_TXDMAEN; //interrupt unmasked activate once tx empty
    return;
}

void enable_dma()
{
    DMA1_Channel5->CCR |= DMA_CCR_EN;
    return;
}

void setup_spi1()
{
	//pa5 = SPI1 sck
	//pa7 = sp1_mosi
	//pa4 = nss = cs
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOA->MODER &= ~0xcc00;
    GPIOA->MODER |= 0x8800;//pa5,7 alternate
    //AFR for 5 and 7

    //GPIOB->MODER &= ~0xf00000;
    //GPIOB->MODER |= 0x500000; //output pb11 for reset, pa4 for nss
    GPIOB->MODER &= ~0xC00000;
    GPIOA->MODER &= ~0x300;
    GPIOB->MODER |= 0x400000;
    GPIOA->MODER |= 0x100;

    GPIOA->MODER &= ~0xc0;
    GPIOA->MODER |= 0x40;//pa3 output for RS
    GPIOB->ODR |= 0x800;
    GPIOA->ODR |= 0x18;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR2 &= ~SPI_CR2_DS;
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE; //enable
    return;
}
// Write your subroutines above

void show_counter(short buffer[])
{
    for(int i=0; i<10000; i++) {
        char line[17];
        sprintf(line,"% 16d", i);
        for(int b=0; b<16; b++)
            buffer[1+b] = line[b] | 0x200;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //setup_spi1();
  LCD_Init(0,0,0);
  LCD_Clear(BLACK);
  LCD_DrawLine(10,20,100,200, WHITE);
  LCD_DrawRectangle(10,20,100,200, GREEN);
  LCD_DrawFillRectangle(120,20,220,200, RED);
  LCD_Circle(50, 260, 50, 1, BLUE);
  LCD_DrawFillTriangle(130,130, 130,200, 190,160, YELLOW);
  LCD_DrawChar(150,155, BLACK, WHITE, 'X', 16, 1);
  LCD_DrawString(140,60,  WHITE, BLACK, "ECE 362", 16, 0);
  LCD_DrawString(140,80,  WHITE, BLACK, "has the", 16, 1);
  LCD_DrawString(130,100, BLACK, GREEN, "best toys", 16, 0);
//LCD_DrawPicture(110,220,(const Picture *)&image);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while(1)
  //{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
/*

*/
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  //GPIOA->MODER &= ~0xc0;
  //GPIOA->MODER |= 0x40;//pa3 output for RS
  GPIOB->ODR |= 0x1;
  GPIOA->ODR |= 0x18;

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR2 &= ~SPI_CR2_DS;
  SPI1->CR1 &= ~(SPI_CR1_BR);
  SPI1->CR1 |= SPI_CR1_MSTR;
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR1 |= SPI_CR1_SPE; //enable

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
