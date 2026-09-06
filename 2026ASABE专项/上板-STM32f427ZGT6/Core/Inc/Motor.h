#ifndef __Motor_H
#define __Motor_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "timer_channel_manager.h"
typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_NULL_PTR,
    MOTOR_ERR_BAD_TIMER,
    MOTOR_ERR_BAD_GPIO,
} Motor_Status;

typedef struct {
    Timer_ID id;
    ChannelIndex ch;
    GPIO_TypeDef* port_in1;
    uint16_t pin_in1;
    GPIO_TypeDef* port_in2;
    uint16_t pin_in2;
    int16_t pwm_dead_value; // 死区值（>=0）
} Motor_Config;

typedef struct {
    GPIO_TypeDef* port_a;
    GPIO_TypeDef* port_b;
    uint16_t pin_a;
    uint16_t pin_b;

    Timer_ID id;   
    TIM_HandleTypeDef* htim;
    ChannelIndex ch;  

    volatile int16_t pwm;
    uint16_t pwm_max_value;      // PWM最大值（ARR值）
    uint16_t pwm_min_value;      // PWM最小值（死区）
    int8_t dir_state;            // -1: reverse, 0: brake/coast, 1: forward
}Motor;
Motor_Status Motor_ConfigApply(Motor* m, const Motor_Config* cfg);
void Motor_Init(Motor* m, Timer_ID id, ChannelIndex ch,
    GPIO_TypeDef* port_a, uint16_t pin_a, GPIO_TypeDef* port_b, uint16_t pin_b , int16_t pwm_dead_value);
void Motor_set_pwm(Motor* m, int16_t pwm);
void Motor_brake(Motor* m);

#endif