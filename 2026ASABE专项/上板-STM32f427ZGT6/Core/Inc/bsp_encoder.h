#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "timer_channel_manager.h"

typedef struct QEncoder QEncoder;
typedef enum {
    QENC_OK = 0,
    QENC_ERR_NULL_PTR,
    QENC_ERR_BAD_TIMER,
    QENC_ERR_NO_SLOT,
    QENC_ERR_BAD_PINCFG,
} QEnc_Status;

typedef struct {
    Timer_ID id;
    ChannelIndex chA;
    ChannelIndex chB;
} QEncoder_Config;

typedef struct {
    uint32_t flag;     // SR flags: TIM_FLAG_CCx
    uint32_t source;   // DIER enables: TIM_IT_CCx
    QEncoder* encoder;
    uint8_t inUse;
} EncoderSlot;

struct QEncoder {
    GPIO_TypeDef* port_a;
    GPIO_TypeDef* port_b;
    uint16_t pin_a;
    uint16_t pin_b;

    volatile int32_t cnt;
    uint8_t prev;

    float speed_enc;
    int8_t samplespeed;
    int32_t encoderlastcount;
    int32_t timer6lastcount;

    void (*handleChange)(QEncoder* self);
};
QEnc_Status QEncoder_ConfigApply(QEncoder* e, const QEncoder_Config* cfg);
void QEncoder_Init(QEncoder* e, Timer_ID id, ChannelIndex chA, ChannelIndex chB);
void QEncoder_HandleChange(QEncoder* e);
void _HAL_TIM_IRQHandler(Timer_ID id);
int32_t QEncoder_Read(QEncoder* e);
void QEncoder_updatespeed(QEncoder* e);
#endif
