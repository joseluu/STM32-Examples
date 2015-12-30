#include "adc.h"



/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	    /**Common config 
	    */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc1);

	    /**Configure the ADC multi-mode 
	    */
	multimode.Mode = ADC_DUALMODE_REGSIMULT;
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
	HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

	    /**Configure Regular Channel 
	    */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	    /**Configure Regular Channel 
	    */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 2;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	    /**Common config 
	    */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 2;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc2);

	    /**Configure Regular Channel 
	    */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	    /**Configure Regular Channel 
	    */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 2;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

static int ADC12_CLK_ENABLED = 0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hadc->Instance == ADC1)
	{
	/* USER CODE BEGIN ADC1_MspInit 0 */

	  /* USER CODE END ADC1_MspInit 0 */
	    /* Peripheral clock enable */
		ADC12_CLK_ENABLED++;
		if (ADC12_CLK_ENABLED == 1) {
			__ADC12_CLK_ENABLE();
		}
  
	    /**ADC1 GPIO Configuration    
	    PA0     ------> ADC1_IN1 
	    */
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		    /* Peripheral DMA init*/
  
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_adc1);

		__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

		  /* USER CODE BEGIN ADC1_MspInit 1 */

		    /* USER CODE END ADC1_MspInit 1 */
	}
	else if (hadc->Instance == ADC2)
	{
	/* USER CODE BEGIN ADC2_MspInit 0 */

	  /* USER CODE END ADC2_MspInit 0 */
	    /* Peripheral clock enable */
		ADC12_CLK_ENABLED++;
		if (ADC12_CLK_ENABLED == 1) {
			__ADC12_CLK_ENABLE();
		}
  
	    /**ADC2 GPIO Configuration    
	    PA4     ------> ADC2_IN1
	    PA6     ------> ADC2_IN3 
	    */
		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /* USER CODE BEGIN ADC2_MspInit 1 */

		    /* USER CODE END ADC2_MspInit 1 */
	}
}
