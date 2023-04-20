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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	int mode_state;
	//1 = initialising
	//2 = cal_unflexed
	//3 = cal_flexed
	//4 = running
	//5 = idle

	//For use in running mode
	char yaw_mode[30]; //0 = rest, 1 = CW, 2 = CCW
	char roll_mode[30]; //0 = rest, 1 = right, 2 = left
	char throttle_mode[30]; //0 = rest, 1 = up, 2 = down
	char pitch_mode[30]; // 0 = rest, 1 = forward, 2 = backward

	float yaw_num;
	float roll_num;
	float throttle_num;
	float pitch_num;

	char titler[30];
	//For cal_unflexed
	//For cal_flexed
	//For initialising will print "INITIALISING" in center
	//For idle it will print "IDLE" in center
	//For running it will print "RUNNING" at top

	char commandy[200];
	char commandy2[200];
	//For cal_unflexed
	//For cal_flexed
	//For initialising, will print "Please Wait"
	//FOr idle, will print "Toggle switch to activate"

	int update_stat_title;
	int update_stat_commandy;
}DispState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;


DispState currDisp;
double a[100];
double b[100];
double X_arr[50];
double Y_arr[50];

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

void small_delay()
{
    nano_wait(10000);
    return;
}

void print_labels()
{
	LCD_DrawString(5,10,  WHITE, BLACK, "Aerial Ace Status Window", 16, 0);
	LCD_DrawFillRectangle(2,35,300,110, LGRAYBLUE);
	LCD_DrawString(5,45,  WHITE, LGRAYBLUE, "Current Mode", 16, 0);
	LCD_DrawLine(2,30,300, 30, WHITE);
	LCD_DrawLine(2,160,300,160, WHITE);
}

void print_title(DispState currDisp)
{
	LCD_DrawString(50,70,  WHITE, LGRAYBLUE, currDisp.titler, 16, 0);
	return;
}

void print_command(DispState currDisp)
{
	LCD_DrawString(0,120,  WHITE, BLACK, currDisp.commandy, 12, 0);
	if(currDisp.mode_state == 2 || currDisp.mode_state == 3)
	{
		LCD_DrawString(0,130,  WHITE, BLACK, currDisp.commandy2, 12, 0);
	}
	for(int i = 0; i < 1000; i++);
	return;
}

void print_stats(DispState currDisp)
{
	char yaw[30];
	char roll[30];
	char pitch[30];
	char throttle[30];

	 //LCD_Clear(BLACK);

	sprintf(yaw, "%.3f", currDisp.yaw_num);
	sprintf(roll, "%.3f", currDisp.roll_num);
	sprintf(pitch, "%.3f", currDisp.pitch_num);
	sprintf(throttle, "%.3f", currDisp.throttle_num);

	LCD_DrawLine(90,165,90,330, WHITE);
	LCD_DrawLine(160,165,160,330, WHITE);

	LCD_DrawString(10,175,  WHITE, BLACK, "PITCH", 16, 0); //90
	LCD_DrawString(10,210,  WHITE, BLACK, "YAW", 16, 0); //150
	LCD_DrawString(10,245,  WHITE, BLACK, "ROLL", 16, 0); //200
	LCD_DrawString(10,280,  WHITE, BLACK, "THROTTLE", 16, 0); //250

	/*float z = 1.34;
	  char abc[30];
	  sprintf(abc, "%.2f", z);*/

	LCD_DrawString(100,175,WHITE, BLACK,pitch , 16, 0);
	LCD_DrawString(100,210,WHITE, BLACK,yaw , 16, 0);
	LCD_DrawString(100,245,WHITE, BLACK,roll , 16, 0);
	LCD_DrawString(100,280,WHITE, BLACK,throttle , 16, 0);

	LCD_DrawString(170,175,WHITE, BLACK,currDisp.pitch_mode , 16, 0);
	LCD_DrawString(170,210,WHITE, BLACK,currDisp.yaw_mode , 16, 0);
	LCD_DrawString(170,245,WHITE, BLACK,currDisp.roll_mode , 16, 0);
	LCD_DrawString(170,280,WHITE, BLACK,currDisp.throttle_mode, 16, 0);
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




void print_progress(DispState currDisp, int time_secs, int CX, int CY, int radius, int curTim)
{
	int i, j;
	int X0;
	int X1;
	int X2;
	int Y0;
	int Y1;
	int Y2;
	int radius2 = round(radius - radius/7);

	int sidesAB = radius;
	double sideC;
	double Cy;
	double Cx;
	int CyR;
	int CxR;

	int startX = CX;
	int startY = CY - radius;

	double central_angleD = 360/time_secs;
	double other_angleD = (180-central_angleD)/2;

	double central_angleR = central_angleD * PI / 180;
	double other_angleR = other_angleD * PI / 180;

    char strs[3];

    if(curTim == 0)
    {
  	  X0 = CX;
  	  Y0 = CY - radius2;
    }
    else
    {
  	  X0 = X_arr[curTim - 1];
  	  Y0 = Y_arr[curTim - 1];
    }
		    X1 = CX;
		    Y1 = CY;
		    sidesAB = radius2;

			sideC = sqrt(2 * (sidesAB * sidesAB) - 2* (sidesAB * sidesAB) * cos(central_angleR));

		      Cy = sin(b[curTim]) * sideC;
		      Cx = cos(b[curTim]) * sideC;

		      CyR = round(Cy);
		      CxR = round(Cx);

		      X2 = X0 + CxR;
		      Y2 = Y0 + CyR;

		      X_arr[curTim] = X2;
		      Y_arr[curTim] = Y2;



		      sprintf(strs, "%d", time_secs - curTim);
			  LCD_DrawFillRectangle(CX-8, CY -8, CX+8, CY+8, BLACK);
		      //LCD_DrawString(CX,CY, WHITE, BLACK, "  ", 16, 1);
		      LCD_DrawFillTriangle(X0, Y0, X1, Y1, X2, Y2, BLACK);
		      LCD_DrawString(CX - 8,CY - 8, WHITE, BLACK, strs, 16, 1);

		  //LCD_DrawFillRectangle(100, 100, 130 , 240, BLACK);

}

void makeCircle(int time_secs, int CX, int CY, int radius)
{
	int i, j;
		int X0;
		int X1;
		int X2;
		int Y0;
		int Y1;
		int Y2;
		int radius2 = round(radius - radius/7);

		int sidesAB = radius;
		double sideC;
		double Cy;
		double Cx;
		int CyR;
		int CxR;

		int startX = CX;
		int startY = CY - radius;

		double central_angleD = 360/time_secs;
		double other_angleD = (180-central_angleD)/2;

		double central_angleR = central_angleD * PI / 180;
		double other_angleR = other_angleD * PI / 180;

	    char strs[3];

		//For progress indicator
	    for(i = 0; i < time_secs; i++)
	    {
	        if(i == 0)
	        {
	            a[i] = (90 - other_angleD);
	            b[i] = a[i] * PI/180;
	        }
	        else
	        {
	            a[i] = (360 - ((180 - a[i-1]) + 2*other_angleD));
	            b[i] = a[i] * PI/180;
	        }
	    }

	    X0 = startX;
	    Y0 = startY;
	    X1 = CX;
	    Y1 = CY; //245

	    for(i = 0; i < time_secs; i++)
	    	  {
	    		  sideC = sqrt(2 * (sidesAB * sidesAB) - 2* (sidesAB * sidesAB) * cos(central_angleR));

	    	      Cy = sin(b[i]) * sideC;
	    	      Cx = cos(b[i]) * sideC;

	    	      CyR = round(Cy);
	    	      CxR = round(Cx);

	    	      X2 = X0 + CxR;
	    	      Y2 = Y0 + CyR;
	    	      LCD_DrawFillTriangle(X0, Y0, X1, Y1, X2, Y2, BRED);
	    	      X0 = X2;
	    	      Y0 = Y2;
	    	  }
	    	  LCD_DrawFillRectangle(startX - 2, startY, CX , CY, BRED);
	    	  return;

}


void main_LCD(float r, float y, float t, float p, int statey, int time_secs, int CX, int CY, int radius)
{
	int i, j;

	if(r >= 0)
	{
		strncpy(currDisp.roll_mode, "RIGHT", 29);
	}
	else
	{
		strncpy(currDisp.roll_mode, "LEFT", 29);
	}

	if(y >= 0)
	{
		strncpy(currDisp.yaw_mode, "RIGHT", 29);
	}
	else
	{
		strncpy(currDisp.yaw_mode, "LEFT", 29);
	}

	if(p >= 0)
	{
		strncpy(currDisp.pitch_mode, "UP", 29);
	}
	else
	{
		strncpy(currDisp.pitch_mode, "DOWN", 29);
	}

	if(t >= 0)
	{
		strncpy(currDisp.throttle_mode, "UP", 29);
	}
	else
	{
		strncpy(currDisp.throttle_mode, "DOWN", 29);
	}


	currDisp.mode_state = statey;
	if(currDisp.mode_state == 1)
	{
		strncpy(currDisp.titler, "INITIALISING", 29);
		strncpy(currDisp.commandy, "...Loading...", 199);
	}
	else if(currDisp.mode_state == 2)
	{
		strncpy(currDisp.titler, "CALIBRATION", 29);
		strncpy(currDisp.commandy, "Please Unflex your fingers until finger", 199);
		strncpy(currDisp.commandy2, "angles are 0 degrees.", 199);
	}
	else if(currDisp.mode_state == 3)
		{
			strncpy(currDisp.titler, "CALIBRATION", 29);
			strncpy(currDisp.commandy, "Please Flex your fingers until finger", 199);
			strncpy(currDisp.commandy2, "angles are 90 degrees.", 199);
		}
	else if(currDisp.mode_state == 4)
		{
			strncpy(currDisp.titler, "ADVANCED", 29);
			strncpy(currDisp.commandy, "Toggle switch to change mode to standard!", 199);
		}
	else if(currDisp.mode_state == 5)
		{
			strncpy(currDisp.titler, "STANDARD", 29);
			strncpy(currDisp.commandy, "Toggle switch to change mode to advanced!", 199);
		}
		//1 = initialising
		//2 = cal_unflexed
		//3 = cal_flexed
		//4 = running
		//5 = idle



	currDisp.pitch_num = p;
	currDisp.yaw_num = y;
	currDisp.roll_num = r;
	currDisp.throttle_num = t;


	  print_labels();

	  print_title(currDisp);
	  if(currDisp.mode_state == 4)
	  {
		  print_stats(currDisp);
	  }
	  print_command(currDisp);
		//Progress bar
		/*int seg_per_sec = 30/timey;
		int i;
		int startX;
		int endX;

		//if cT = 0: 0-10, 10-20, 20-30
		//	 ct = 1: 30-40, 40-50, 50-60
		// ((curTim * seg_per_sec) + i) * 10
		for(i = 0; i < seg_per_sec; i++)
		{
			startX = ((curTim * seg_per_sec) + i) * 10;
			endX = ((curTim * seg_per_sec) + i + 1) * 10;
			LCD_DrawFillRectangle(startX,220,endX,250, LGRAYBLUE);
		}
	*/
	  return;
}
// Write your subroutines above


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

//LCD_DrawPicture(110,220,(const Picture *)&image);*/

  int statey = 2;
  float r = 3.2;
  float y = -3.1;
  float t = 1.62;
  float p = -0.075;
  int time_secs = 15;
  int CX  = 120;
  int CY = 240;
  int radius = 45;
  int radius2 = round(radius - radius/7);
  int i,j;

  main_LCD(r,  y,  t,  p,  statey, time_secs, CX, CY, radius);

  if(currDisp.mode_state == 2 || currDisp.mode_state == 3)
  {
	  makeCircle(time_secs, CX, CY, radius);
	  for(i = 0; i < time_secs; i++)
	  {
		  print_progress(currDisp, time_secs, CX, CY, radius, i);
		  for(j = 0; j < 500000; j++);
	  }
	  LCD_DrawFillRectangle(CX-10, CY -10, CX+10 , CY+10, BLACK);
	  LCD_DrawFillRectangle(CX-5, CY-radius2, CX+2 , CY, BLACK);
  }


  	  //20 to 280
  	  //,


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {

    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
  }
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
