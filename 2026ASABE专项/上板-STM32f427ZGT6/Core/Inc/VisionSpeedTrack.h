#ifndef VISION_SPEED_TRACK_H
#define VISION_SPEED_TRACK_H

#include <stdint.h>

typedef struct {
    float kp_img;          // 视觉位置误差到v_ref修正量的比例
    float kd_img;          // 视觉相对速度到v_ref修正量的比例
    float kp_speed;        // 编码器速度环P
    float ki_speed;        // 编码器速度环I
    float kd_speed;        // 编码器速度环D，通常先设0
    float ks;              // 静摩擦补偿PWM
    float kv;              // 目标速度前馈系数，通常先设0
    float decay;           // v_ref泄漏保持系数，0~1
    float v_ref_max;       // 目标速度限幅，count/s
    float integral_max;    // 速度积分限幅
    float pwm_max;         // 输出PWM限幅
    float speed_deadzone;  // v_ref小于该值时置0
    float output_deadzone; // PWM小于该值时置0
} VisionSpeedTrack_Config;

typedef struct {
    float v_ref;            // 当前保持的目标编码器速度，count/s
    float speed_integral;   // 速度环积分
    float last_speed_error; // 上一次速度误差，用于D项
    float last_pwm;         // 上一次输出PWM，仅用于调试/保持扩展
} VisionSpeedTrack_State;

typedef struct {
    float v_ref;            // 本次目标速度
    float speed_error;      // v_ref - encoder_speed
    float speed_integral;   // 本次积分值
    float kp_term;          // 速度环P输出
    float ki_term;          // 速度环I输出
    float kd_term;          // 速度环D输出
    float ks_term;          // 静摩擦前馈输出
    float kv_term;          // 速度前馈输出
    int16_t pwm;            // 最终PWM
} VisionSpeedTrack_Output;

/**
 * @brief 初始化视觉速度追踪状态。
 */
void VisionSpeedTrack_Init(VisionSpeedTrack_State *state);

/**
 * @brief 清除目标速度、积分和调试状态。
 */
void VisionSpeedTrack_Reset(VisionSpeedTrack_State *state);

/**
 * @brief 用视觉误差积分修正目标速度，并通过编码器速度环计算PWM。
 * @param state 控制器运行状态，调用者为左右电机分别保存。
 * @param cfg 控制参数。
 * @param pos_error_encoder 视觉位置误差换算后的编码器误差，count。
 * @param image_error_speed_encoder 视觉Kalman速度换算后的编码器相对速度，count/s。
 * @param encoder_speed 当前编码器测速，count/s。
 * @param dt_seconds 本次控制周期，单位秒。
 * @param out 可选调试输出，不需要可传NULL。
 * @return 本次PWM输出。
 */
int16_t VisionSpeedTrack_Update(
    VisionSpeedTrack_State *state,
    const VisionSpeedTrack_Config *cfg,
    float pos_error_encoder,
    float image_error_speed_encoder,
    float encoder_speed,
    float dt_seconds,
    VisionSpeedTrack_Output *out);

#endif
