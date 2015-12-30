#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{
	HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg;
	HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
	HRTIM_TimerCfgTypeDef pTimerCfg;
	HRTIM_SimpleOCChannelCfgTypeDef pSimpleOCChannelCfg;

	hhrtim1.Instance = HRTIM1;
	hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
	hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
	HAL_HRTIM_Init(&hhrtim1);

	HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14);

	HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10);

	pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
	pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD;
	HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg);

	pTimeBaseCfg.Period = 0x8000;
	pTimeBaseCfg.RepetitionCounter = 0x00;
	pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL4;
	pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
	HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

	pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
	pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
	pTimerCfg.DMASrcAddress = 0x0;
	pTimerCfg.DMADstAddress = 0x0;
	pTimerCfg.DMASize = 0x1;
	pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
	pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
	pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
	pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
	pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
	pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
	pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
	pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
	HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg);

	pTimeBaseCfg.Period = 0xFFFD;
	pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL4;
	HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg);

	pSimpleOCChannelCfg.Mode = HRTIM_BASICOCMODE_TOGGLE;
	pSimpleOCChannelCfg.Pulse = 0x8000;
	pSimpleOCChannelCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
	pSimpleOCChannelCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	HAL_HRTIM_SimpleOCChannelConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pSimpleOCChannelCfg);

	HAL_HRTIM_MspPostInit(&hhrtim1);

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hhrtim)
{

	if (hhrtim->Instance == HRTIM1)
	{
	/* USER CODE BEGIN HRTIM1_MspInit 0 */

	  /* USER CODE END HRTIM1_MspInit 0 */
	    /* Peripheral clock enable */
		__HRTIM1_CLK_ENABLE();

    /* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
	/* USER CODE BEGIN HRTIM1_MspInit 1 */

	    /* USER CODE END HRTIM1_MspInit 1 */
	}
}
void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hhrtim)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hhrtim->Instance == HRTIM1)
	{
	/* USER CODE BEGIN HRTIM1_MspPostInit 0 */

	  /* USER CODE END HRTIM1_MspPostInit 0 */
  
	    /**HRTIM1 GPIO Configuration    
	    PA8     ------> HRTIM1_CHA1 
	    */
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

		    /* USER CODE END HRTIM1_MspPostInit 1 */
	}

}
