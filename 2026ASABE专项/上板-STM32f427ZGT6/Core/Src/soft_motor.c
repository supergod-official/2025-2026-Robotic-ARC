#include "soft_motor.h"

#define PWM_PERIOD_TICKS (10U - 1U) // 1 tick = 100us, 10 ticks = 1ms = 1kHz PWM   84MHz

#define MIN(a,b) ((a)<(b)?(a):(b))

/* -------------------- Helpers -------------------- */
static void enable_port_clock(GPIO_TypeDef* port)
{
    if (!port) return;
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
    else if (port == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
}

static inline void gpio_set(GPIO_TypeDef* port, uint16_t pin)
{
    port->BSRR = pin;
}

static inline void gpio_reset(GPIO_TypeDef* port, uint16_t pin)
{
    port->BSRR = ((uint32_t)pin << 16U);
}

static inline void soft_motor_set_dir(Soft_Motor* m, int8_t dir)
{
    if (!m || m->dir_state == dir) return;

    if (dir > 0) {
        gpio_set(m->port_in1, m->pin_in1);
        gpio_reset(m->port_in2, m->pin_in2);
    } else if (dir < 0) {
        gpio_reset(m->port_in1, m->pin_in1);
        gpio_set(m->port_in2, m->pin_in2);
    } else {
        gpio_reset(m->port_in1, m->pin_in1);
        gpio_reset(m->port_in2, m->pin_in2);
    }

    m->dir_state = dir;
}

/* Initialize soft motor */
void Soft_Motor_Init(Soft_Motor* m, GPIO_TypeDef* port_in1, uint16_t pin_in1,
                     GPIO_TypeDef* port_in2, uint16_t pin_in2,
                     GPIO_TypeDef* port_a, uint16_t pin_a, int16_t pwm_dead_value)
{
    if (m == NULL) return;

    m->port_in1 = port_in1;
    m->pin_in1 = pin_in1;
    m->port_in2 = port_in2;
    m->pin_in2 = pin_in2;
    m->port_a = port_a;
    m->pin_a = pin_a;

    m->pwm_min_value = (pwm_dead_value > 0) ? pwm_dead_value : 0;
    m->pwm_max_value = PWM_PERIOD_TICKS;
    m->period_ticks = PWM_PERIOD_TICKS;
    m->counter = 0;
    m->pwm = 0;
    m->pwm_abs = 0;
    m->dir_state = 0;

    enable_port_clock(m->port_in1);
    enable_port_clock(m->port_in2);
    enable_port_clock(m->port_a);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = m->pin_in1;
    HAL_GPIO_Init(m->port_in1, &GPIO_InitStruct);
    gpio_reset(m->port_in1, m->pin_in1);

    GPIO_InitStruct.Pin = m->pin_in2;
    HAL_GPIO_Init(m->port_in2, &GPIO_InitStruct);
    gpio_reset(m->port_in2, m->pin_in2);

    GPIO_InitStruct.Pin = m->pin_a;
    HAL_GPIO_Init(m->port_a, &GPIO_InitStruct);
    gpio_reset(m->port_a, m->pin_a);
}

/* Configure soft motor */
Soft_Motor_Status Soft_Motor_ConfigApply(Soft_Motor* m, const Soft_Motor_Config* cfg)
{
    if (m == NULL || cfg == NULL) {
        return SOFT_MOTOR_ERR_NULL_PTR;
    }

    m->port_in1 = cfg->port_in1;
    m->pin_in1 = cfg->pin_in1;
    m->port_in2 = cfg->port_in2;
    m->pin_in2 = cfg->pin_in2;
    m->port_a = cfg->port_a;
    m->pin_a = cfg->pin_a;

    m->pwm_min_value = (cfg->pwm_dead_value > 0) ? cfg->pwm_dead_value : 0;
    m->pwm_max_value = PWM_PERIOD_TICKS;
    m->period_ticks = PWM_PERIOD_TICKS;
    m->counter = 0;
    m->pwm = 0;
    m->pwm_abs = 0;
    m->dir_state = 0;

    enable_port_clock(m->port_in1);
    enable_port_clock(m->port_in2);
    enable_port_clock(m->port_a);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = m->pin_in1;
    HAL_GPIO_Init(m->port_in1, &GPIO_InitStruct);
    gpio_reset(m->port_in1, m->pin_in1);

    GPIO_InitStruct.Pin = m->pin_in2;
    HAL_GPIO_Init(m->port_in2, &GPIO_InitStruct);
    gpio_reset(m->port_in2, m->pin_in2);

    GPIO_InitStruct.Pin = m->pin_a;
    HAL_GPIO_Init(m->port_a, &GPIO_InitStruct);
    gpio_reset(m->port_a, m->pin_a);

    return SOFT_MOTOR_OK;
}

/* Set PWM command */
void Soft_Motor_set_pwm(Soft_Motor* m, int16_t pwm)
{
    if (!m) return;


    if (pwm < 0) {
        int32_t duty = (int32_t)(-pwm) + (int32_t)m->pwm_min_value;
        soft_motor_set_dir(m, -1);
        m->pwm_abs = (uint16_t)MIN(duty, m->pwm_max_value);
    } else {
        int32_t duty = (int32_t)pwm + (int32_t)m->pwm_min_value;
        soft_motor_set_dir(m, 1);
        m->pwm_abs = (uint16_t)MIN(duty, m->pwm_max_value);
    }

    m->pwm = pwm;
}

/* Brake motor */
void Soft_Motor_brake(Soft_Motor* m)
{
    if (m == NULL) return;

    m->pwm = 0;
    m->pwm_abs = 0;
    soft_motor_set_dir(m, 0);
    gpio_reset(m->port_a, m->pin_a);
}

/* PWM update on each timer interrupt */
void Soft_Motor_UpdatePWM(Soft_Motor* m)
{
    if (m == NULL) return;

    uint16_t high_ticks = m->pwm_abs;
    uint16_t period = (uint16_t)m->period_ticks + 1U; // ARR-style period

    // Rising edge at cycle start
    if (m->counter == 0) {
        if (high_ticks > 0U) {
            gpio_set(m->port_a, m->pin_a);
        } else {
            gpio_reset(m->port_a, m->pin_a);
        }
    }

    // Falling edge once per cycle
    if (high_ticks > 0U && (uint16_t)m->counter == high_ticks) {
        gpio_reset(m->port_a, m->pin_a);
    }

    m->counter++;
    if ((uint16_t)m->counter >= period) {
        m->counter = 0;
    }
}
