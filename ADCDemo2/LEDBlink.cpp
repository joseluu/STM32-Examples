#include <stm32f3xx_hal.h>
#include <stm32f3xx.h>
#include "hrtim.h"
#include "adc.h"

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

void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
	__DMA1_CLK_ENABLE();

	  /* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


int g_MeasurementNumber = 0;

#include <numeric>


const int ADC_BUFFER_LENGTH_WORDS = 2048 ; //32 bit words

class MeasurementSet
{
public:
	unsigned short input1;
	unsigned short input2;
	unsigned short temperature;
	unsigned short input3;

	MeasurementSet()
	{
		input1 = 0;
		input2 = 0;
		temperature = 0;
		input3 = 0;
	}
	MeasurementSet(int i1, int i2, int t, int i3)
	{
		input1 = i1;
		input2 = i2;
		temperature = t;
		input3 = i3;
	}
	MeasurementSet operator/(
					const MeasurementSet& rhs)
	{
		MeasurementSet value(this->input1 / rhs.input1, 0, 0, 0);
		return value;
	}
	MeasurementSet operator/(int dividor)
	{
		MeasurementSet value(this->input1 / dividor, 
							this->input2 / dividor,
			this->temperature / dividor,
			this->input3 / dividor);
		return value;
	}
} ;
const int ADC_BUFFER_LENGTH = ADC_BUFFER_LENGTH_WORDS* sizeof(uint32_t) / sizeof(MeasurementSet);
MeasurementSet g_ADCBuffer[ADC_BUFFER_LENGTH];
MeasurementSet g_ADCValue;
MeasurementSet g_ADCValueNull( 0, 0, 0, 0 );

MeasurementSet measurementSetPlus(MeasurementSet &in1, MeasurementSet &in2)
{
	MeasurementSet value((in1.input1 + in2.input1)/2,
				 (in1.input2  + in2.input2)/2,
		(in1.temperature + in2.temperature)/2,
		(in1.input3  + in2.input3)/2);
	return value;
}
#define DMA 1

volatile bool bErrorADC;
extern "C"
{

	void   HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
	{
		bErrorADC = true;
	}
volatile	int g_DmaOffsetBeforeAveragingF, g_DmaOffsetAfterAveragingF;
volatile	int g_DmaOffsetBeforeAveragingH, g_DmaOffsetAfterAveragingH;
    
	volatile uint32_t lastTick;
	volatile uint32_t lastInterval; // ms buffer acquisition time
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
	{
		uint32_t thisTick = HAL_GetTick();
		lastInterval = thisTick - lastTick;
		lastTick = thisTick;
		g_DmaOffsetBeforeAveragingF = ADC_BUFFER_LENGTH_WORDS - DMA1_Channel1->CNDTR;
		g_ADCValue = std::accumulate(&g_ADCBuffer[ADC_BUFFER_LENGTH / 2], &g_ADCBuffer[ADC_BUFFER_LENGTH], g_ADCValueNull, measurementSetPlus);
		g_DmaOffsetAfterAveragingF = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_MeasurementNumber += ADC_BUFFER_LENGTH; 
	}
#if 1
	void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
	{
		g_DmaOffsetBeforeAveragingH = ADC_BUFFER_LENGTH_WORDS - DMA1_Channel1->CNDTR;
		g_ADCValue = std::accumulate(&g_ADCBuffer[0], &g_ADCBuffer[ADC_BUFFER_LENGTH /2], g_ADCValueNull, measurementSetPlus) ;
		g_DmaOffsetAfterAveragingH = ADC_BUFFER_LENGTH - DMA1_Channel1->CNDTR;
		g_MeasurementNumber += ADC_BUFFER_LENGTH;
	}
#endif

	volatile uint32_t lastTick_HRTIM;
	volatile uint32_t lastInterval_HRTIM; // ms buffer acquisition time

	void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim,
		uint32_t TimerIdx)
	{
		static uint32_t counter;
		counter++;
		if (counter < 1024)
			return;
		counter = 0;
		uint32_t thisTick_HRTIM = HAL_GetTick();
		lastInterval_HRTIM = thisTick_HRTIM - lastTick_HRTIM;
		lastTick_HRTIM = thisTick_HRTIM;
	}

	extern DMA_HandleTypeDef hdma_adc1;
	void DMA1_Channel1_IRQHandler()
	{
		HAL_DMA_IRQHandler(&hdma_adc1);
	}

	void HRTIM1_Master_IRQHandler()
	{
		HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_MASTER);
	}

	void ADC1_2_IRQHandler()
	{
		HAL_ADC_IRQHandler(&hadc1);
	}
}


void MX_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	__GPIOA_CLK_ENABLE();
 
	GPIO_InitStructure.Pin = GPIO_PIN_5;
 
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_HRTIM1_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();

	//HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TDCEN;
	HAL_HRTIM_SimpleBaseStart_IT(&hhrtim1, HRTIM_TIMERINDEX_MASTER);

	HAL_HRTIM_SimpleOCStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1);
	
#if DMA
	for (std::size_t i = 0; i < ADC_BUFFER_LENGTH; i++)
	{
		//g_ADCBuffer[i]={0,0,0,0};
	}
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)g_ADCBuffer, ADC_BUFFER_LENGTH_WORDS);
#else
	HAL_ADC_Start_IT(&hadc1);
#endif
 
	for (;;)
	{
		int onTime = g_ADCValue.input1;
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

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1 | RCC_PERIPHCLK_USART1
	                            | RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
