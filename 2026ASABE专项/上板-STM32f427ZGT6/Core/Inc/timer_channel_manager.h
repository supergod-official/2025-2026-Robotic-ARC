#ifndef TIMER_CHANNEL_MANAGER_H
#define TIMER_CHANNEL_MANAGER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    TIMER_1 = 1,
    TIMER_2,
    TIMER_3,
    TIMER_4,
    TIMER_5,
    TIMER_8,
    TIMER_9,
    TIMER_10,
    TIMER_11,
    TIMER_12,
    TIMER_13,
    TIMER_14,
    TIMER_COUNT   // = 15
} Timer_ID;

typedef enum {
    CH1 = 0, CH2, CH3, CH4, CH_COUNT
} ChannelIndex;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t alternate_func;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
} PinConfig;

extern const uint32_t TCM_HAL_CH[CH_COUNT];

const PinConfig* TCM_GetPinConfig(Timer_ID id, ChannelIndex ch);
TIM_HandleTypeDef* TCM_GetTimerHandle(Timer_ID id);
void TCM_RegisterTimerHandle(Timer_ID id, TIM_HandleTypeDef* htim);


extern TIM_HandleTypeDef htim6;
int32_t TCM_Readtimer6count();
void TCM_AddTimer6Overflow(int32_t inc);

#endif
// timer_channel.h