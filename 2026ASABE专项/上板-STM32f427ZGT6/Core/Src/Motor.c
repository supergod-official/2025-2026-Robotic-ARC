#include "Motor.h"
#define MIN(a,b) ((a)<(b)?(a):(b))

/* -------------------- 辅助函数 -------------------- */
static void enable_port_clock(GPIO_TypeDef* port)
{
    if (!port) return;
    // 根据端口使能对应的时�?
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

static inline void motor_set_dir(Motor* m, int8_t dir)
{
    if (!m || m->dir_state == dir) return;

    if (dir > 0) {
        gpio_set(m->port_a, m->pin_a);
        gpio_reset(m->port_b, m->pin_b);
    } else if (dir < 0) {
        gpio_reset(m->port_a, m->pin_a);
        gpio_set(m->port_b, m->pin_b);
    } else {
        gpio_reset(m->port_a, m->pin_a);
        gpio_reset(m->port_b, m->pin_b);
    }

    m->dir_state = dir;
}

static int motor_fields_valid(const Motor* m)
{
    if (!m) return 0;
    if ((uint32_t)m->id >= TIMER_COUNT) return 0;
    if ((uint32_t)m->ch >= CH_COUNT) return 0;
    if (!m->htim || !m->htim->Instance) return 0;
    if (!IS_GPIO_ALL_INSTANCE(m->port_a) || !IS_GPIO_ALL_INSTANCE(m->port_b)) return 0;
    return 1;
}
/**
 * @brief 初始化电机（PWM 输出，方向引脚）
 * 
 * @param m 电机对象指针
 * @param id 定时器ID
 * @param ch 通道�?
 * @param port_a 方向引脚1的端�?
 * @param pin_a 方向引脚1的引�?
 * @param port_b 方向引脚2的端�?
 * @param pin_b 方向引脚2的引�?
 */
void Motor_Init(Motor* m, Timer_ID id, ChannelIndex ch,
                GPIO_TypeDef* port_a, uint16_t pin_a,
                GPIO_TypeDef* port_b, uint16_t pin_b, int16_t pwm_dead_value)
{
    if (!m) return;
    m->id = id;
    m->ch = ch;
    m->port_a = port_a;
    m->pin_a = pin_a;
    m->port_b = port_b;
    m->pin_b = pin_b;
    m->htim = TCM_GetTimerHandle(id);
    if (!m->htim || !m->htim->Instance) {
        // 可以添加错误处理
        return;
    }

    m->pwm_max_value = m->htim->Instance->ARR;        // 获取 PWM 最大值（ARR 值，1000�?
    m->pwm_min_value = pwm_dead_value;                // 获取 PWM 最小值（死区,整数�?   
    m->pwm = 0;

    m->dir_state = 0;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    enable_port_clock(port_a);
    enable_port_clock(port_b);
    if (port_a) {
        GPIO_InitStruct.Pin = pin_a;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    // 推挽输出
        GPIO_InitStruct.Pull = GPIO_NOPULL;            // 不上拉不下拉
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 高速输�?
        HAL_GPIO_Init(port_a, &GPIO_InitStruct);
        gpio_reset(port_a, pin_a);
    }
    if (port_b) {
        GPIO_InitStruct.Pin = pin_b;
        HAL_GPIO_Init(port_b, &GPIO_InitStruct);
        gpio_reset(port_b, pin_b);
    }
}
Motor_Status Motor_ConfigApply(Motor* m, const Motor_Config* cfg)
{
    if (!m || !cfg) return MOTOR_ERR_NULL_PTR;
    if ((uint32_t)cfg->id >= TIMER_COUNT) return MOTOR_ERR_BAD_TIMER;
    if ((uint32_t)cfg->ch >= CH_COUNT) return MOTOR_ERR_BAD_TIMER;
    if (!IS_GPIO_ALL_INSTANCE(cfg->port_in1) || !IS_GPIO_ALL_INSTANCE(cfg->port_in2)) {
        return MOTOR_ERR_BAD_GPIO;
    }

    // 保存配置
    m->id     = cfg->id;
    m->ch     = cfg->ch;
    m->port_a = cfg->port_in1;
    m->pin_a  = cfg->pin_in1;
    m->port_b = cfg->port_in2;
    m->pin_b  = cfg->pin_in2;

    // 获取定时器句�?
    m->htim = TCM_GetTimerHandle(cfg->id);
    if (!m->htim || !m->htim->Instance) {
        return MOTOR_ERR_BAD_TIMER;
    }

    // PWM 上下�?
    m->pwm_max_value = (uint16_t)m->htim->Instance->ARR;
    m->pwm_min_value = (cfg->pwm_dead_value > 0) ? (uint16_t)cfg->pwm_dead_value : 0;
    m->pwm = 0;

    m->dir_state = 0;

    // GPIO 初始�?
    if (!m->port_a || !m->port_b) {
        return MOTOR_ERR_BAD_GPIO;
    }

    enable_port_clock(m->port_a);
    enable_port_clock(m->port_b);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = m->pin_a;
    HAL_GPIO_Init(m->port_a, &GPIO_InitStruct);
    gpio_reset(m->port_a, m->pin_a);

    GPIO_InitStruct.Pin = m->pin_b;
    HAL_GPIO_Init(m->port_b, &GPIO_InitStruct);
    gpio_reset(m->port_b, m->pin_b);

    HAL_TIM_PWM_Start(m->htim, TCM_HAL_CH[m->ch]);
    __HAL_TIM_SET_COMPARE(m->htim, TCM_HAL_CH[m->ch], 0);
    return MOTOR_OK;
}

/**
 * @brief 设置电机�?PWM �?
 * 
 * @param m 电机对象指针
 * @param pwm PWM值，范围�?~pwm_max_value
 */



void Motor_set_pwm(Motor* m, int16_t pwm)
{
    if (!motor_fields_valid(m)) return;

    if (pwm == 0) {
        motor_set_dir(m, 0);
        __HAL_TIM_SET_COMPARE(m->htim, TCM_HAL_CH[m->ch], 0);
        m->pwm = 0;
        return;
    }

    if (pwm < 0) {
        motor_set_dir(m, -1);
        int16_t duty = (int16_t)(-pwm) + (int32_t)m->pwm_min_value;
        __HAL_TIM_SET_COMPARE(m->htim, TCM_HAL_CH[m->ch], (uint16_t)MIN(duty, (int32_t)m->pwm_max_value));
    } else {
        motor_set_dir(m, 1);
        int16_t duty = (int16_t)pwm + (int32_t)m->pwm_min_value;
        __HAL_TIM_SET_COMPARE(m->htim, TCM_HAL_CH[m->ch], (uint16_t)MIN(duty, (int32_t)m->pwm_max_value));
    }
    m->pwm = pwm;
}
void Motor_brake(Motor* m)
{
    if (!motor_fields_valid(m)) return;

    // Braking: pull both direction pins low and clear PWM.
    motor_set_dir(m, 0);
    __HAL_TIM_SET_COMPARE(m->htim, TCM_HAL_CH[m->ch], 0);
    m->pwm = 0;
}
