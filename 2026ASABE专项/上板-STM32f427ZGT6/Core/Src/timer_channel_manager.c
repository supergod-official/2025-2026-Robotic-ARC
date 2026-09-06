#include "timer_channel_manager.h"

static volatile int32_t timer6overflowcount = 0;

int32_t TCM_Readtimer6count()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t overflow = timer6overflowcount;
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim6);
    if (!primask) __enable_irq();
    return overflow + count;
}
void TCM_AddTimer6Overflow(int32_t inc)
{
    timer6overflowcount += inc;
}











const uint32_t TCM_HAL_CH[CH_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static const PinConfig TIM1_PINS[CH_COUNT] = {
    [CH1] = {GPIOA, GPIO_PIN_8,  GPIO_AF1_TIM1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
    [CH2] = {GPIOA, GPIO_PIN_9,  GPIO_AF1_TIM1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
    [CH3] = {GPIOA, GPIO_PIN_10, GPIO_AF1_TIM1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
    [CH4] = {GPIOA, GPIO_PIN_11, GPIO_AF1_TIM1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
};

static const PinConfig TIM2_PINS[CH_COUNT] = {
    [CH1] = {GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH2] = {GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH3] = {GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH4] = {GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
};

static const PinConfig TIM3_PINS[CH_COUNT] = {
    [CH1] = {GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH2] = {GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH3] = {GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH4] = {GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
};

static const PinConfig TIM8_PINS[CH_COUNT] = {
    [CH1] = {GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH2] = {GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH3] = {GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
    [CH4] = {GPIOC, GPIO_PIN_9, GPIO_AF3_TIM8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM},
};
static const PinConfig* const TIMER_PIN_TABLE[TIMER_COUNT] = {
    [0]       = NULL,
    [TIMER_1] = TIM1_PINS,
    [TIMER_2] = TIM2_PINS,
    [TIMER_3] = TIM3_PINS,
    [TIMER_8] = TIM8_PINS,
    // 其他 timer 默认 NULL
};

static TIM_HandleTypeDef* timer_handles[TIMER_COUNT] = {0};

const PinConfig* TCM_GetPinConfig(Timer_ID id, ChannelIndex ch)
{
    if (id >= TIMER_COUNT) return NULL;
    if (ch >= CH_COUNT) return NULL;

    const PinConfig* table = TIMER_PIN_TABLE[id];
    if (!table) return NULL;
    return &table[ch];
}

TIM_HandleTypeDef* TCM_GetTimerHandle(Timer_ID id)
{
    if (id >= TIMER_COUNT) return NULL;
    return timer_handles[id];
}

void TCM_RegisterTimerHandle(Timer_ID id, TIM_HandleTypeDef* htim)
{
    if (id >= TIMER_COUNT) return;
    timer_handles[id] = htim;
}
