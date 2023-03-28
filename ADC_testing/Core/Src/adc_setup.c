/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void ADC_Calibrate(ADC_TypeDef*);
static int ADC_Read(ADC_TypeDef*);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ADC_Calibrate(ADC_TypeDef* adc) {
  if (adc != ADC1 && adc != ADC2 && adc != ADC3 && adc != ADC4) {
	  return;
  }

  adc->CR &= ~ADC_CR_ADEN; // Disable ADC
  adc->CR |= ADC_CR_ADCALDIF; // Calibration for Single-ended input mode
  adc->CR |= ADC_CR_ADCAL; // Start ADC calibration
  while (adc->CR & ADC_CR_ADCAL);
}

static int ADC_Read(ADC_TypeDef* adc) {
	if (adc != ADC1 && adc != ADC2 && adc != ADC3 && adc != ADC4) {
		return 0;
	}
	return adc->DR;
}
