#pragma once
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;

	/* USER CODE BEGIN Private defines */

	/* USER CODE END Private defines */

	void MX_ADC1_Init(void);
	void MX_ADC2_Init(void);

	/* USER CODE BEGIN Prototypes */

	/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */
