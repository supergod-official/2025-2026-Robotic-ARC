#ifndef __CAR___H__
#define __CAR___H__


#include "MotorPidController.h"
#include "soft_motor.h"
#include <stdint.h>

extern MotorPidController qExtend;
extern MotorPidController pExtend;
extern MotorPidController qRotate;
extern MotorPidController pRotate;
extern MotorPidController qOpenmv;
extern MotorPidController pOpenmv;
extern Soft_Motor qSowMove;
extern Soft_Motor pSowMove;
extern Soft_Motor qSowSeed;
extern Soft_Motor pSowSeed;
/* -------------------- with encoder -------------------- */
extern QEncoder_Config enc_cfg1;   //qExtend
extern Motor_Config motor_cfg1;

extern QEncoder_Config enc_cfg2;   //qRotate
extern Motor_Config motor_cfg2;

extern QEncoder_Config enc_cfg11;  //pExtend
extern Motor_Config motor_cfg11;


extern QEncoder_Config enc_cfg22;  //pRotate
extern Motor_Config motor_cfg22;


extern QEncoder_Config enc_cfg_mv_l;//qOpenmv
extern Motor_Config motor_cfg_mv_l;

extern QEncoder_Config enc_cfg_mv_r;//pOpenmv
extern Motor_Config motor_cfg_mv_r;
/* -------------------- without encoder -------------------- */
extern Soft_Motor_Config soft_motor_cfg_1_l;//qSowMove

extern Soft_Motor_Config soft_motor_cfg_1_r;//pSowMove

extern Soft_Motor_Config soft_motor_cfg_2_l;//qSowSeed

extern Soft_Motor_Config soft_motor_cfg_2_r;//pSowSeed

//pid
extern controller_config ctrl_cfg1;  //Extend
extern controller_config ctrl_cfg2;  //Rotate
extern controller_config ctrl_cfg_mv;//Openmv


extern volatile uint32_t VisionFrame_l_seq;
extern volatile uint32_t VisionFrame_r_seq;
extern volatile float pal_track_target_filter_alpha;
extern volatile float pal_track_speed_filter_alpha;
extern volatile float openmv_offset_filter_alpha;
typedef struct {
    uint8_t state;
    uint8_t target_in_limit;
    uint8_t ready;
    uint32_t frame_seq;
    int16_t raw_offset;
    int16_t raw_velocity_x;
    int32_t raw_delta_encoder;
    float raw_error_speed_encoder;
    float delta_encoder_f;
    int32_t delta_encoder;
    float error_speed_encoder;
} VisionTrackFilter_t;
extern VisionTrackFilter_t VisionFilter_l;
extern VisionTrackFilter_t VisionFilter_r;
typedef struct __attribute__((packed)){
    int16_t offset_x;
    int8_t offset_y;
    // OpenMV stable flag, used for OpenMV motor tracking.
    uint8_t is_plant_find;
    // Stable target inside the configured y-window, used for enter/leave judging.
    uint8_t in_window;
    uint8_t tail;
} MVPacket_t;
extern MVPacket_t MVpacket_l;
extern MVPacket_t MVpacket_r;
extern volatile uint32_t MVpacket_l_seq;
extern volatile uint32_t MVpacket_r_seq;
typedef struct {
    uint8_t ready;
    uint8_t is_plant_find;
    uint8_t in_window;
    int16_t raw_offset_x;
    int8_t raw_offset_y;
    float offset_x_f;
    float offset_y_f;
} MVPacketFilter_t;
extern MVPacketFilter_t MVFilter_l;
extern MVPacketFilter_t MVFilter_r;
typedef struct __attribute__((packed)){
    uint8_t is_enable;      
    float   car_speed;   
    uint8_t tail;
} ArduinoWorkCmd_t;
extern ArduinoWorkCmd_t ArduinoWorkCmd;
typedef struct
{
    uint8_t total_plants;     // 已识别植物总数
    uint8_t type_0_count;     // 0类植物数量
    uint8_t type_1_count;     // 1类植物数量
    uint8_t type_2_count;     // 2类植物数量
   
} PlantStats;

extern PlantStats plant_all;
typedef struct {
    uint8_t prev_mv;
    uint8_t in_window;
    uint8_t ever_seen_yellow;
    uint8_t ever_seen_green;
    uint8_t ever_seen_vision;
    uint8_t type_now;
} WindowJudge_t;
extern WindowJudge_t wj_l;
extern WindowJudge_t wj_r;
typedef struct {
    GPIO_TypeDef* port_led1;
    GPIO_TypeDef* port_led2;
    GPIO_TypeDef* port_led3;
    uint16_t pin_led1;
    uint16_t pin_led2;
    uint16_t pin_led3;
    uint8_t dir_state;
} Led_t;
extern Led_t LED_l;
extern Led_t LED_r;

void track_and_extend(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error, int32_t unstable_error, int32_t unstable_ticks);
void track_and_extend_left(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error, int32_t unstable_error, int32_t unstable_ticks);
void track_and_extend_right(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error, int32_t unstable_error, int32_t unstable_ticks);
void openmv_extend(int32_t mincontrol_count, int32_t errorcount);
void openmv_extend_left(int32_t mincontrol_count, int32_t errorcount);
void openmv_extend_right(int32_t mincontrol_count, int32_t errorcount);
void Execute_Sow_Sequence(WindowJudge_t *wj, MVPacket_t *MVpacket, Soft_Motor *move_m, Soft_Motor *seed_m);
void WindowJudge_Reset(WindowJudge_t *wj);
void WindowJudge_EnterWork(WindowJudge_t *wj, uint8_t curr_mv);
void set_waiting(MotorPidController *controller);
void PlantClassifier_Update(void);
void PlantClassifier_UpdateLeft(void);
void PlantClassifier_UpdateRight(void);
void is_should_plant(void);
uint8_t handle_plant(void);

int32_t calculate_angeltogo(MotorPidController *controller, int16_t offset);
void VisionTracking_ConvertTarget(
    MotorPidController *controller, int16_t offset_px, int16_t velocity_px_s,
    int32_t *delta_encoder, float *error_speed_encoder, uint8_t *target_in_limit);
void VisionTracking_FilterSet(VisionTrackFilter_t *filter, uint8_t state,
    int16_t offset, int16_t velocity_x,
    int32_t delta_encoder, float error_speed_encoder,
    uint8_t target_in_limit, uint32_t frame_seq);
void OpenMV_FilterSet(MVPacketFilter_t *filter, int16_t offset_x, int8_t offset_y,
    uint8_t is_plant_find, uint8_t in_window);
void VisionTracking_FilterUpdate(void);
void freeze_encodertogo(MotorPidController *controller,int32_t mincontrol_count, int32_t encodertogo, int32_t errorcount);
void zero_encodertogo(MotorPidController *controller,int32_t mincontrol_count, int32_t encodertogo, int32_t errorcount);
void set_orign(MotorPidController *controller);
void set_extendMAX(MotorPidController *controller);
void handle_palmotor(uint8_t state,
    MotorPidController *controller_r,MotorPidController *controller_e,
    int32_t encodertogo, float error_speed_encoder, uint8_t target_in_limit,
    int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error, int32_t unstable_error, int32_t unstable_ticks) ;
void handle_mvmotor(uint8_t is_blob_find, int16_t offset, MotorPidController *controller,
     int32_t mincontrol_count, int32_t errorcount) ;
void LED_UpdateByTypeAndDir(const WindowJudge_t *wj, Led_t *led);
void LED_UpdateByType(void);
#endif
