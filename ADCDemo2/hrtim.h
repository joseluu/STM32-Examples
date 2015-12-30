#pragma once
#ifndef __hrtim_H
#define __hrtim_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

	extern HRTIM_HandleTypeDef hhrtim1;

	/* USER CODE BEGIN Private defines */

	/* USER CODE END Private defines */

	void MX_HRTIM1_Init(void);
            
	void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

	/* USER CODE BEGIN Prototypes */

	/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ hrtim_H */
