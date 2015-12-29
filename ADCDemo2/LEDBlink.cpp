#include <stm32f3xx_hal.h>
#include <stm32f3xx.h>

#ifdef __cplusplus
extern "C"
#endif
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

#ifdef __cplusplus
extern "C"
#endif
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	    /* When the following line is hit, the variables contain the register values. */
	asm("bkpt 255");
	for (;;)
		;
}
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
#ifdef __cplusplus
extern "C"
#endif
void HardFault_Handler(void) __attribute__((naked));

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
#ifdef __cplusplus
extern "C"
#endif
void HardFault_Handler(void)
{
	__asm volatile
	(
	    " tst lr, #4                                                \n"
	    " ite eq                                                    \n"
	    " mrseq r0, msp                                             \n"
	    " mrsne r0, psp                                             \n"
	    " ldr r1, [r0, #24]                                         \n"
	    " ldr r2, handler2_address_const                            \n"
	    " bx r2                                                     \n"
	    " handler2_address_const: .word prvGetRegistersFromStack    \n"
	);
}

void SystemClock_Config();

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef  hdma_adc1;

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

		#ifndef DMA_WORKS
		    /* Peripheral DMA init*/
  
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
		HAL_DMA_Init(&hdma_adc1);

		__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

		#endif
		  /* USER CODE BEGIN ADC1_MspInit 1 */

		    /* USER CODE END ADC1_MspInit 1 */
	}
	

}

void ConfigureADC()
{
	ADC_ChannelConfTypeDef adcChannel;

	hadc1.Instance = ADC1;

	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;

	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;

	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;

	//hadc1.Init.EOCSelection = EOC_SEQ_CONV;//ch

	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;

	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc1);
 
	adcChannel.Channel = ADC_CHANNEL_1;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	adcChannel.Offset = 0;

	if (HAL_ADC_ConfigChannel(&hadc1, &adcChannel) != HAL_OK)
	{
		asm("bkpt 255");
	}

	adcChannel.Channel = ADC_CHANNEL_16;
	adcChannel.Rank = 2;
	adcChannel.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	adcChannel.Offset = 0;

	if (HAL_ADC_ConfigChannel(&hadc1, &adcChannel) != HAL_OK)
	{
		asm("bkpt 255");
	}
}

void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
	__DMA1_CLK_ENABLE();

	  /* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

void ConfigureDMA()
{
	__DMA1_CLK_ENABLE(); 
	#ifdef DMA_WORKS
		    /* Peripheral DMA init*/
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
		HAL_DMA_Init(&hdma_adc1);

		__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
	#endif
  /* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}


volatile uint32_t g_ADCValue = 0;
int g_MeasurementNumber = 0;

#include <numeric>

enum{ ADC_BUFFER_LENGTH = 2048 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

#define DMA 1


extern "C"
{
volatile	int g_DmaOffsetBeforeAveragingF, g_DmaOffsetAfterAveragingF;
volatile	int g_DmaOffsetBeforeAveragingH, g_DmaOffsetAfterAveragingH;
    
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
	{
		g_DmaOffsetBeforeAveragingF = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_ADCValue = std::accumulate(g_ADCBuffer + ADC_BUFFER_LENGTH / 2, g_ADCBuffer + ADC_BUFFER_LENGTH, 0) / (ADC_BUFFER_LENGTH / 2);
		g_DmaOffsetAfterAveragingF = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_MeasurementNumber += ADC_BUFFER_LENGTH; 
	}
    
	void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
	{
		g_DmaOffsetBeforeAveragingH = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_ADCValue = std::accumulate(g_ADCBuffer, g_ADCBuffer + ADC_BUFFER_LENGTH / 2, 0) / (ADC_BUFFER_LENGTH / 2);
		g_DmaOffsetAfterAveragingH = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_MeasurementNumber += ADC_BUFFER_LENGTH;
	}

#if DMA
	void DMA1_Channel1_IRQHandler()
	{
		HAL_DMA_IRQHandler(&hdma_adc1);
	}
#endif

	void ADC1_2_IRQHandler()
	{
		HAL_ADC_IRQHandler(&hadc1);
	}
}


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_DMA_Init();
	ConfigureADC();
    
	GPIO_InitTypeDef GPIO_InitStructure;
	__GPIOA_CLK_ENABLE();
 
	GPIO_InitStructure.Pin = GPIO_PIN_5;
 
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

#if DMA
	for (std::size_t i = 0; i < ADC_BUFFER_LENGTH; i++)
	{
		g_ADCBuffer[i]=0;
	}
	//ConfigureDMA();
	HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH);	
#else
	HAL_ADC_Start_IT(&hadc1);
#endif
 
	for (;;)
	{
		int onTime = g_ADCValue;
		int offTime = 4096 - onTime;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		for (int i = 0; i < onTime/10; i++)
			asm("nop");
 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		for (int i = 0; i < offTime/10; i++)
			asm("nop");
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
