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
#define SENSOR_BUS hi2c1
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lsm9ds1_reg.h"
#include "fusion.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define    BOOT_TIME            20 //ms

#define    WAIT_TIME_MAG        60 //ms
#define    WAIT_TIME_XL        200 //ms
#define    WAIT_TIME_GY        800 //ms

#define    SAMPLES               5 //number of samples

/* Self test results. */
#define    ST_PASS     0x1U
#define    ST_FAIL     0x0U

#define PI 3.142857

/* Self test limits in mgauss @ 12G*/
static const float min_st_mag_limit[] = {1000.0f, 1000.0f,  100.0f};
static const float max_st_mag_limit[] = {3000.0f, 3000.0f, 1000.0f};

/* Self test limits in mg @ 2g*/
static const float min_st_xl_limit[] = {70.0f, 70.0f,  70.0f};
static const float max_st_xl_limit[] = {1500.0f, 1500.0f, 1500.0f};

/* Self test limits in mdps @ 2000 dps*/
static const float min_st_gy_limit[] = {200000.0f, 200000.0f, 200000.0f};
static const float max_st_gy_limit[] = {800000.0f, 800000.0f, 800000.0f};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

static sensbus_t mag_bus = {&SENSOR_BUS, LSM9DS1_MAG_I2C_ADD_H, 0, 0};
static sensbus_t imu_bus = {&SENSOR_BUS, LSM9DS1_IMU_I2C_ADD_H, 0, 0};

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];
int k = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int32_t platform_write_imu(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_write_mag(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;
	reg |= 0x80;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	  return 0;
}

static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	  sensbus_t *sensbus = (sensbus_t *)handle;
	  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	  return 0;
}

static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t *)handle;
	  reg |= 0x80;
	  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	  return 0;
}

static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}

void imu_tester(void)
{


       /* Read samples in polling mode (no int) */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;

  platform_delay(BOOT_TIME);
   /* Check device ID */
   lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

   if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
	 k = 2;
     while (1) {
       /* manage here device not found */
    	 return 1;
     }
   }

   /* Restore default configuration */
     lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

     do {
       lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
     } while (rst);

     /* Enable Block Data Update */
     lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

     /* Set full scale */
       lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
       lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
       lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
       /* Configure filtering chain - See datasheet for filtering chain details */
       /* Accelerometer filtering chain */
       lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
       lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
       lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
       /* Gyroscope filtering chain */
       lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
       lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
       lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
       /* Set Output Data Rate / Power mode */
       lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
       lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

	    if ( reg.status_imu.xlda && reg.status_imu.gda ) {
	      /* Read imu data */
	      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	      lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
	                                   data_raw_acceleration);
	      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
	                                   data_raw_angular_rate);
	      acceleration_mg[0] = (lsm9ds1_from_fs4g_to_mg(
	                             data_raw_acceleration[0]) * 9.807) / 1000; //convert to m/s^2 from micro-gees
	      acceleration_mg[1] = (lsm9ds1_from_fs4g_to_mg(
	                             data_raw_acceleration[1]) * 9.807)/ 1000;
	      acceleration_mg[2] = (lsm9ds1_from_fs4g_to_mg(
	                             data_raw_acceleration[2]) * 9.807) / 1000;
	      angular_rate_mdps[0] = (lsm9ds1_from_fs2000dps_to_mdps(
	                               data_raw_angular_rate[0]) * 2 * PI)/360000;
	      angular_rate_mdps[1] = (lsm9ds1_from_fs2000dps_to_mdps(
                  	  	  	  	  data_raw_angular_rate[1]) * 2 * PI)/360000;
	      angular_rate_mdps[2] = (lsm9ds1_from_fs2000dps_to_mdps(
                  	  	  	  	  data_raw_angular_rate[2]) * 2 * PI)/360000;

	      //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

	      sprintf((char *)tx_buffer,
	              "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
	              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
	              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
	    }

	    if ( reg.status_mag.zyxda ) {
	      /* Read magnetometer data */
	      memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
	      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
	      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
	                                   data_raw_magnetic_field[0]) / 10;
	      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
	                                   data_raw_magnetic_field[1]) / 10;
	      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
	                                   data_raw_magnetic_field[2]) / 10;
	      sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
	              magnetic_field_mgauss[0], magnetic_field_mgauss[1],
	              magnetic_field_mgauss[2]);
	      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
	    }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
