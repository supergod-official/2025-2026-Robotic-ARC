#ifndef __SOFT_MOTOR_H
#define __SOFT_MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    SOFT_MOTOR_OK = 0,
	SOFT_MOTOR_ERR_NULL_PTR,
	SOFT_MOTOR_ERR_BAD_GPIO,
	SOFT_MOTOR_ERR_PWM_OUT_OF_RANGE,
} Soft_Motor_Status;

typedef struct {
    GPIO_TypeDef* port_in1;
    uint16_t pin_in1;
    GPIO_TypeDef* port_in2;
    uint16_t pin_in2;
    GPIO_TypeDef* port_a;
    uint16_t pin_a;
    int16_t pwm_dead_value; // dead-zone compensation (>=0)
} Soft_Motor_Config;

typedef struct {
    GPIO_TypeDef* port_in1;
    GPIO_TypeDef* port_in2;
    uint16_t pin_in1;
    uint16_t pin_in2;

    GPIO_TypeDef* port_a;
    uint16_t pin_a;

    volatile int16_t pwm;     // requested PWM command
    volatile uint16_t pwm_abs; // duty in ticks
    int16_t pwm_max_value;
    int16_t pwm_min_value;
    int16_t period_ticks; // ARR-style period value
    int16_t counter;
    int8_t dir_state;     // -1: reverse, 0: brake/coast, 1: forward
} Soft_Motor;

Soft_Motor_Status Soft_Motor_ConfigApply(Soft_Motor* m, const Soft_Motor_Config* cfg);
void Soft_Motor_Init(Soft_Motor* m, GPIO_TypeDef* port_in1, uint16_t pin_in1,
                     GPIO_TypeDef* port_in2, uint16_t pin_in2,
                     GPIO_TypeDef* port_a, uint16_t pin_a, int16_t pwm_dead_value);
void Soft_Motor_set_pwm(Soft_Motor* m, int16_t pwm);
void Soft_Motor_brake(Soft_Motor* m);
void Soft_Motor_UpdatePWM(Soft_Motor* m);

#endif
