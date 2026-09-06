#include "VisionSpeedTrack.h"

/**
 * @brief 轻量绝对值，避免额外依赖math库。
 */
static float vst_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/**
 * @brief 返回数值符号，用于静摩擦前馈方向。
 */
static float vst_signf(float v)
{
    if (v > 0.0f) return 1.0f;
    if (v < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * @brief 浮点限幅。
 */
static float vst_clampf(float v, float min_v, float max_v)
{
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

/**
 * @brief 初始化视觉速度追踪状态。
 */
void VisionSpeedTrack_Init(VisionSpeedTrack_State *state)
{
    VisionSpeedTrack_Reset(state);
}

/**
 * @brief 清除目标速度、积分和上次输出。
 */
void VisionSpeedTrack_Reset(VisionSpeedTrack_State *state)
{
    if (!state) return;

    state->v_ref = 0.0f;
    state->speed_integral = 0.0f;
    state->last_speed_error = 0.0f;
    state->last_pwm = 0.0f;
}

/**
 * @brief 外环用视觉误差更新目标速度，内环用编码器速度误差输出PWM。
 */
int16_t VisionSpeedTrack_Update(
    VisionSpeedTrack_State *state,
    const VisionSpeedTrack_Config *cfg,
    float pos_error_encoder,
    float image_error_speed_encoder,
    float encoder_speed,
    float dt_seconds,
    VisionSpeedTrack_Output *out)
{
    float decay;
    float speed_error;
    float speed_d;
    float v_ref_correction;
    float pwm;
    float ks_term;
    int16_t pwm_i;

    if (!state || !cfg) {
        return 0;
    }

    if (dt_seconds <= 0.0f) {
        dt_seconds = 0.001f;
    }

    // 外环：视觉误差产生v_ref修正量，再按dt积分到目标电机速度。
    // 这不是单层PID，而是带泄漏的速度参考积分器。
    decay = vst_clampf(cfg->decay, 0.0f, 1.0f);
    v_ref_correction = cfg->kp_img * pos_error_encoder
                     + cfg->kd_img * image_error_speed_encoder;
    state->v_ref = state->v_ref * decay + v_ref_correction * dt_seconds;
    state->v_ref = vst_clampf(state->v_ref, -cfg->v_ref_max, cfg->v_ref_max);

    if (vst_absf(state->v_ref) < cfg->speed_deadzone) {
        state->v_ref = 0.0f;
    }

    // 内环：编码器速度PI/PID，让实际速度跟随v_ref。
    speed_error = state->v_ref - encoder_speed;
    state->speed_integral += speed_error * dt_seconds;
    state->speed_integral = vst_clampf(
        state->speed_integral, -cfg->integral_max, cfg->integral_max);

    speed_d = (speed_error - state->last_speed_error) / dt_seconds;
    state->last_speed_error = speed_error;

    // 前馈：只在目标速度非零时补偿静摩擦，避免小噪声触发死区电压。
    ks_term = 0.0f;
    if (state->v_ref != 0.0f) {
        ks_term = cfg->ks * vst_signf(state->v_ref);
    }

    // 最终输出 = 速度反馈 + 静摩擦前馈 + 速度前馈。
    pwm = cfg->kp_speed * speed_error
        + cfg->ki_speed * state->speed_integral
        + cfg->kd_speed * speed_d
        + ks_term
        + cfg->kv * state->v_ref;

    pwm = vst_clampf(pwm, -cfg->pwm_max, cfg->pwm_max);
    if (vst_absf(pwm) < cfg->output_deadzone) {
        pwm = 0.0f;
    }

    state->last_pwm = pwm;
    pwm_i = (int16_t)pwm;

    if (out) {
        out->v_ref = state->v_ref;
        out->speed_error = speed_error;
        out->speed_integral = state->speed_integral;
        out->kp_term = cfg->kp_speed * speed_error;
        out->ki_term = cfg->ki_speed * state->speed_integral;
        out->kd_term = cfg->kd_speed * speed_d;
        out->ks_term = ks_term;
        out->kv_term = cfg->kv * state->v_ref;
        out->pwm = pwm_i;
    }

    return pwm_i;
}
