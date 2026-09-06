#include "MotorPidController.h"
#include <stdlib.h>
#include <math.h>

extern void Error_Handler(void);
extern MotorPidController qExtend;
//base timer6 0.01ms tick 2000ticks overflow(20ms)




static void constrain(float *value, float max, float min)
{
    if (*value > max) *value = max;
    if (*value < min) *value = min;
}





void MotorPidController_ConfigApply(MotorPidController *controller,
    const QEncoder_Config* enc_cfg, const Motor_Config* motor_cfg,const controller_config* ctrl_cfg)
{
  if (Motor_ConfigApply(&controller->m_motor, motor_cfg)!= MOTOR_OK) Error_Handler();
  if (QEncoder_ConfigApply(&controller->m_encoder, enc_cfg)!= QENC_OK) Error_Handler();
  controller->kp_v = ctrl_cfg->kp_v;
  controller->ki_v = ctrl_cfg->ki_v;
  controller->kd_v = ctrl_cfg->kd_v;
  controller->kp_w = ctrl_cfg->kp_w;
  controller->ki_w = ctrl_cfg->ki_w;
  controller->kd_w = ctrl_cfg->kd_w;
  controller->integral_max_v = ctrl_cfg->integral_max_v;
  controller->integral_max_w = ctrl_cfg->integral_max_w;
  controller->pulse_to_distance   = ctrl_cfg->pulse_to_distance;
  controller->pulse_to_angel      = ctrl_cfg->pulse_to_angel;
  controller->speed_flilter_alpha = ctrl_cfg->speed_flilter_alpha;
  controller->Interval            = ctrl_cfg->Interval ? ctrl_cfg->Interval : 20; // 默认20ms  没啥用

  controller->integral_interval = 100;    // 1ms
  controller->lastTime = 0;
}



void Controller_SetPosition(MotorPidController *controller,int32_t encodercount,int32_t errorcount)
{
  if (controller == &qExtend){
    int32_t encodertogo = encodercount - MotorPidController_readencoder(controller);
    if (encodertogo > 200){
      Motor_set_pwm(&controller->m_motor, 1000);
    } else if (encodertogo < -200){
      Motor_set_pwm(&controller->m_motor, -1000);
    } else if (abs(encodertogo) <= errorcount){
      Motor_brake(&controller->m_motor);
      controller->integral_enc = 0;    // 清除积分 or 冻结积分  这里稳态是0 固清除积分
    } else {
      int32_t CurrentTime = TCM_Readtimer6count();
      if (CurrentTime - controller->lastTime >= controller->integral_interval){    
        controller->integral_enc+= encodertogo * 1 / 1000.0f;   // 1ms 积分一次
        controller->lastTime = CurrentTime;
        constrain(&controller->integral_enc, controller->integral_max_v, -controller->integral_max_v);
      }
      controller->ki_term =controller->ki_v * controller->integral_enc ;
      controller->kp_term = controller->kp_v * encodertogo;
      controller->kd_term = controller->kd_v * controller->m_encoder.speed_enc;
      
      float pwm = controller->kp_term + controller->ki_term + controller->kd_term ;
      Motor_set_pwm(&controller->m_motor, (int16_t)pwm);
    }
  }
}

void Controller_SetSpeed(MotorPidController *controller,float T_speedw,float errorspeed)
{
  if (controller == &qExtend){
    controller->speedw = MotorPidController_readspeedw(controller);
    // 速度滤波
    controller->speed_w_filtered = controller->speed_w_filtered * (1- controller->speed_flilter_alpha) + controller->speedw * controller->speed_flilter_alpha;

    //误差积分
    float error = T_speedw - controller->speed_w_filtered;
    //冻结积分
    if (abs(error) >= errorspeed)
    {
      int32_t CurrentTime = TCM_Readtimer6count();
      if (CurrentTime - controller->lastTime >= controller->integral_interval){    
      controller->integral_w+= error * 1 / 1000.0f;   // 1ms 积分一次
      controller->lastTime = CurrentTime;
      constrain(&controller->integral_w, controller->integral_max_w, -controller->integral_max_w);
      }
    }
    controller->ki_term =controller->ki_w * controller->integral_w ;
    controller->kp_term = controller->kp_w * error;

    //计算PWM
    float pwm = controller->kp_term + controller->ki_term ;
    Motor_set_pwm(&controller->m_motor, (int16_t)pwm);
  }
}




void MotorPidController_Reset(MotorPidController *controller)
{
  controller->integral_enc = 0;
  controller->integral_w = 0;
  controller->integral_v = 0;
}




void MotorPidController_setpwm(MotorPidController *controller,int16_t pwm)
{
  Motor_set_pwm(&controller->m_motor, pwm);
}


int32_t MotorPidController_readencoder(MotorPidController *controller)
{
    return QEncoder_Read(&controller->m_encoder);
}
float MotorPidController_readspeedv(MotorPidController *controller)
{
  return controller->m_encoder.speed_enc * controller->pulse_to_distance;
}
float MotorPidController_readspeedw(MotorPidController *controller)
{
  return controller->m_encoder.speed_enc * controller->pulse_to_angel;
}
float MotorPidController_readspeed_enc(MotorPidController *controller)
{
  return controller->m_encoder.speed_enc;
}

