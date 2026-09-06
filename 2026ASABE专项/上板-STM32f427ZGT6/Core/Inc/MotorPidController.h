#ifndef __MotorPidController_H__
#define __MotorPidController_H__

#include "Motor.h"
#include "bsp_encoder.h"


typedef struct{
    float kp_v,ki_v,kd_v;
    float kp_w,ki_w,kd_w;
    float pulse_to_distance;
    float pulse_to_angel;
    float speed_flilter_alpha;
    int32_t Interval;      // 测速间隔
    float integral_max_v;   //积分上限
    float integral_max_w;   //积分上限
}controller_config;
//c11 不能初始化
typedef struct{

    Motor m_motor;
    QEncoder m_encoder;
    //编码器测速

    float pulse_to_distance;
    float pulse_to_angel;
    int32_t Interval;


    float speedw;
    float speedv;
    //PID
    float kp_v,ki_v,kd_v;
    float kp_w,ki_w,kd_w;
    //误差积分
    float integral_enc;
    float integral_v;
    float integral_w;
    //积分间隔
    int8_t integral_interval;
    int32_t lastTime;
    //积分上限
    float integral_max_v;
    float integral_max_w;
    //误差微分
    float last_error_v;
    float last_error_w;
    //速度滤波器
    float speed_flilter_alpha;
    float speed_v_filtered;
    float speed_w_filtered;
    //测试
    float ki_term;
    float kp_term;
    float kd_term;
}MotorPidController;
void MotorPidController_ConfigApply(MotorPidController *controller,
    const QEncoder_Config* enc_cfg, const Motor_Config* motor_cfg,const controller_config* ctrl_cfg);

void MotorPidController_updatespeed(MotorPidController *controller);
void Controller_SetPosition(MotorPidController *controller,int32_t encodercount,int32_t errorcount);
void Controller_SetSpeed(MotorPidController *controller,float T_speedw,float errorspeed);


void MotorPidController_setpwm(MotorPidController *controller,int16_t pwm);
void MotorPidController_Reset(MotorPidController *controller);
float MotorPidController_readspeedv(MotorPidController *controller);
float MotorPidController_readspeedw(MotorPidController *controller);
float MotorPidController_readspeed_enc(MotorPidController *controller);
int32_t MotorPidController_readencoder(MotorPidController *controller);



#endif