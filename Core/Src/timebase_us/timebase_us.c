#include "stm32f1xx.h"
#include "timebase_us.h"
#include "stdio.h"

#define TIMEBASE_US_INTERRUPT_PRIO			2


/* Counter that increments on every 2-bytes timer overflow */
static volatile uint64_t overflow_counter;
TIM_HandleTypeDef htim3;

bool timebase_us_init() {
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	__HAL_RCC_TIM3_CLK_ENABLE();

	HAL_NVIC_SetPriority(TIM3_IRQn, TIMEBASE_US_INTERRUPT_PRIO, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = (SystemCoreClock/2000000) - 1;//Prescaler for counting every processor tick
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim3) != HAL_OK){
		return false;
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		return false;
	}
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		return false;
	}
	return true;
}

uint16_t get_tick_us_lo16() {
	return TIM3->CNT;
}

uint64_t get_tick_us_overflow_cnt() {
	return overflow_counter;
}

uint64_t get_tick_us() {
//	UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	uint16_t lo16_cached = get_tick_us_lo16();
	uint64_t hi32_cached = overflow_counter;
	uint64_t res = hi32_cached*1000 + lo16_cached;
//	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	return res;
}

uint32_t get_tick_ms() {
	return overflow_counter;
}

void set_tick_us(uint64_t tick) {
    overflow_counter = tick/1000;
    TIM3->CNT = tick % 1000;
}

void timebase_us_callback() {
	overflow_counter++;
}
