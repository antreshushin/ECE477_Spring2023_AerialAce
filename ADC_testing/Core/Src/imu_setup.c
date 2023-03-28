/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm9ds1_reg.h"

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
#define SENSOR_BUS hi2c1

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

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


/* USER CODE END 0 */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
