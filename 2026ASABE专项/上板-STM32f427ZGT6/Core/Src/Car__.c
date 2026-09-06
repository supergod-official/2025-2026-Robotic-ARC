#include "Car__.h"
#include <stdlib.h>   
#include <math.h>

static void process_packet(uint8_t is_plant_find, uint8_t state, PlantStats *plant_all, WindowJudge_t *wj);
static void handle_mv_plant(uint8_t is_plant_find);
typedef enum {
    SOW_WAIT_HOLD = 0,
    SOW_WAIT_GO_SEED,
    SOW_WAIT_GO_UP
} SowWaitDecision_t;
static SowWaitDecision_t Decide_Sow_Wait_By_MVPacket(const MVPacket_t *MVpacket , const WindowJudge_t *wj);
/**
 * @brief qExtend 左侧伸缩电机
 * 
 * @param enc_cfg1 & enc_cfg11 Extend
 * @param enc_cfg2 & enc_cfg22 Rotate
 * @param ctrl_cfg1 & ctrl_cfg2 pid
 * 

 */
/* -------------------- l extend——rotate配置 -------------------- */
MotorPidController qExtend;

QEncoder_Config enc_cfg1 = {
	// PA6 PA7
	.id  = TIMER_3,
	.chA = CH2,
	.chB = CH1,

};

Motor_Config motor_cfg1 = {
	// PD12
    .id = TIMER_4,
    .ch = CH1,
    .port_in1 = GPIOD,
    .pin_in1  = GPIO_PIN_3,
    .port_in2 = GPIOD,
    .pin_in2  = GPIO_PIN_4,
    .pwm_dead_value = 70,
};

/**
 * @brief qRotate 左侧旋转电机
 * 
 * @param pid控制ctrl_cfg2

 */
MotorPidController qRotate;
QEncoder_Config enc_cfg2 = {
	//PB0 PB1
	.id  = TIMER_3,
	.chA = CH4,
	.chB = CH3,
};

Motor_Config motor_cfg2 = {
	//PD13
    .id = TIMER_4,
    .ch = CH2,
    .port_in2 = GPIOD,
    .pin_in2  = GPIO_PIN_0,
    .port_in1 = GPIOD,
    .pin_in1  = GPIO_PIN_1,
    .pwm_dead_value = 100,
};

/* -------------------- r extend——rotate配置 -------------------- */
MotorPidController pExtend;

QEncoder_Config enc_cfg11 = {
	// PC6 PC7
	.id  = TIMER_8,
	.chA = CH1,
	.chB = CH2,

};

Motor_Config motor_cfg11 = {
	//PD14
    .id = TIMER_4,
    .ch = CH3,
    .port_in1 = GPIOG,
    .pin_in1  = GPIO_PIN_10,
    .port_in2 = GPIOG,
    .pin_in2  = GPIO_PIN_9,
    .pwm_dead_value = 70,
};


MotorPidController pRotate;

QEncoder_Config enc_cfg22 = {
	//PC8 PC9
	.id  = TIMER_8,
	.chA = CH3,
	.chB = CH4,

};

Motor_Config motor_cfg22 = {
	//PD15
    .id = TIMER_4,
    .ch = CH4,
    .port_in1 = GPIOG,
    .pin_in1  = GPIO_PIN_11,
    .port_in2 = GPIOG,
    .pin_in2  = GPIO_PIN_12,
    .pwm_dead_value = 100,
};
/* -------------------- Openmv extend配置 -------------------- */
MotorPidController pOpenmv;
MotorPidController qOpenmv;


QEncoder_Config enc_cfg_mv_l = {
	//PA0 pA1
	.id  = TIMER_2,
	.chA = CH2,
	.chB = CH1,

};

Motor_Config motor_cfg_mv_l = {
	//PE6
    .id = TIMER_9,
    .ch = CH2,
    .port_in2 = GPIOF,
    .pin_in2  = GPIO_PIN_2,
    .port_in1 = GPIOF,
    .pin_in1  = GPIO_PIN_1,
    .pwm_dead_value = 50,
};
QEncoder_Config enc_cfg_mv_r = {
	//PA2 PB11
	.id  = TIMER_2,
	.chA = CH3,
	.chB = CH4,



};

Motor_Config motor_cfg_mv_r = {
	//PE5
    .id = TIMER_9,
    .ch = CH1,
    .port_in1 = GPIOE,
    .pin_in1  = GPIO_PIN_3,
    .port_in2 = GPIOE,
    .pin_in2  = GPIO_PIN_2,
    .pwm_dead_value = 50,
};
/* -------------------- l 播种器配置 -------------------- */
Soft_Motor qSowMove;
Soft_Motor qSowSeed;

//seed
Soft_Motor_Config soft_motor_cfg_2_l = {
    .port_in2 = GPIOF,
    .pin_in2  = GPIO_PIN_14,
    .port_in1 = GPIOF,
    .pin_in1  = GPIO_PIN_13,
    .port_a = GPIOE,
    .pin_a  = GPIO_PIN_12,
    .pwm_dead_value = 0,
};

//move
Soft_Motor_Config soft_motor_cfg_1_l = {
    .port_in1 = GPIOG,
    .pin_in1  = GPIO_PIN_0,
    .port_in2 = GPIOG,
    .pin_in2  = GPIO_PIN_1,
    .port_a = GPIOE,
    .pin_a  = GPIO_PIN_13,
    .pwm_dead_value = 0,
};


/* -------------------- r 播种器配置 -------------------- */
Soft_Motor pSowMove;
Soft_Motor pSowSeed;

//seed
Soft_Motor_Config soft_motor_cfg_2_r = {
    .port_in1 = GPIOE,   
    .pin_in1  = GPIO_PIN_8,
    .port_in2 = GPIOE,
    .pin_in2  = GPIO_PIN_7,
    .port_a = GPIOE,
    .pin_a  = GPIO_PIN_14,
    .pwm_dead_value = 0,
};
//move
Soft_Motor_Config soft_motor_cfg_1_r = {
    .port_in1 = GPIOE,
    .pin_in1  = GPIO_PIN_11,
    .port_in2 = GPIOE,
    .pin_in2  = GPIO_PIN_10,
    .port_a = GPIOE,
    .pin_a  = GPIO_PIN_15,
    .pwm_dead_value = 0,
};
/* -------------------- LED配置 -------------------- */
//0: red
//1: green
//2: blue
Led_t LED_l = {
    .port_led1 = GPIOG, // l红
    .pin_led1 = GPIO_PIN_5,
    .port_led2 = GPIOC,
    .pin_led2 = GPIO_PIN_1, // l绿色
    .port_led3 = GPIOC,
    .pin_led3 = GPIO_PIN_0,  // l蓝色
    .dir_state = 3,
};
Led_t LED_r = {
    .port_led1 = GPIOC,    // r红色
    .pin_led1 = GPIO_PIN_3,
    .port_led2 = GPIOG,
    .pin_led2 = GPIO_PIN_4,  // r绿
    .port_led3 = GPIOC,
    .pin_led3 = GPIO_PIN_2,  // r 蓝
    .dir_state = 3,
};

void LED_UpdateByTypeAndDir(const WindowJudge_t *wj, Led_t *led)
{
    if (led == NULL || wj == NULL) return;

    // 仅当状态改变时才更新硬件
    if (wj->type_now == led->dir_state) return;

    led->dir_state = wj->type_now;  // 记录新状态

    // 1. 关闭所有LED（使用BSRR高16位复位）
    if (led->port_led1) led->port_led1->BSRR = (led->pin_led1 << 16);
    if (led->port_led2) led->port_led2->BSRR = (led->pin_led2 << 16);
    if (led->port_led3) led->port_led3->BSRR = (led->pin_led3 << 16);

    // 2. 根据状态点亮对应LED（使用BSRR低16位置位）
    switch (led->dir_state) {
        case 0:
            if (led->port_led1) led->port_led1->BSRR = led->pin_led1;
            break;
        case 1:
            if (led->port_led2) led->port_led2->BSRR = led->pin_led2;
            break;
        case 2:
            if (led->port_led3) led->port_led3->BSRR = led->pin_led3;
            break;
        case 255:  // 全灭（已关闭，无需操作）
        default:
            break;
    }
}

/* -------------------- 用户可调pid参数 -------------------- */
/**
 * @brief pid 参数
 * @param kp_v为主参数
 * @param kp_w为备用参数
 * @param integral_max_v：积分上限，防止积分过大导致过冲
 * @param pulse_to_distance：编码器脉冲转实际距离的比例，单位cm/pulse
 * @param pulse_to_angel：编码器脉冲转实际角度的比例，单位度/pulse

 */
//Extend pid
controller_config ctrl_cfg1 = {
    .kp_v = 1.0f,
    .ki_v = 8.0f,
    .kd_v = -0.05f,
    .kp_w = 0.2f,
    .ki_w = 10.0f,
    .kd_w = 0.0f,
    .integral_max_v = 10.0f,
    .integral_max_w = 8.0f,
    .pulse_to_distance = 0.005f,   // 每个脉冲对应的距离（单位：cm）
    .pulse_to_angel = 0.0,         // 每个脉冲对应的角度（单位：度）
    .speed_flilter_alpha = 0.002f, // 速度滤波器的平滑系数
    .Interval = 20,              
};
//Rotate pid
controller_config ctrl_cfg2 = {
    .kp_v = 1.0f,
    .ki_v = 5.0f,  // 5.0
    .kd_v = -0.05f,
    // Yellow tracking PID uses the w group.
    .kp_w = 0.6f,
    .ki_w = 8.0f,
    .kd_w = 0.08f,
    .integral_max_v = 10.0f,
    .integral_max_w = 80.0f,
    .pulse_to_distance = 0.0f,       // 每个脉冲对应的距离（单位：cm）
    .pulse_to_angel = 0.022727f,     // 每个脉冲对应的角度（单位：度）
    .speed_flilter_alpha = 0.002f,    // 速度滤波器的平滑系数
    .Interval = 20,         
};
//Openmv-extend pid
controller_config ctrl_cfg_mv = {
    .kp_v = 0.3f,
    .ki_v = 8.0f,
    .kd_v = -0.04f,
    .kp_w = 0.5f,
    .ki_w = 60.0f,
    .kd_w = -0.02f,
    .integral_max_v = 10.0f,
    .integral_max_w = 8.0f,
    .pulse_to_distance = 0.00533f,       // 每个脉冲对应的距离（单位：cm）
    .pulse_to_angel = 0.0f,     // 每个脉冲对应的角度（单位：度）
    .speed_flilter_alpha = 0.002f,    // 速度滤波器的平滑系数
    .Interval = 20,         
};

/* -------------------- UART && 视觉识别存储 -------------------- */
/**
 * @brief VisionFilter_l/r-树莓派; MVpacket_l-MVpacket_r; ArduinoWorkCmd-arduino;
 * @param VisionFilter.state: 0 1 2 empty single double
 * @param MVpacket.offset_x: 决定什么时候投种-取决于openmv的数据
 * @param MVpacket.offset_y: openmv-extend
 * @note OpenMV 串口现在发送 stable + dx + dy + in_window + tail。
 *       main.c 会把 stable 写入 is_plant_find。
 * @param ArduinoWorkCmd.is_enable: 转弯和过道均=0 ,工作时=1
 * @param pulse_to_angel：编码器脉冲转实际角度的比例，单位度/pulse

 */
volatile uint32_t VisionFrame_l_seq = 0;
volatile uint32_t VisionFrame_r_seq = 0;
VisionTrackFilter_t VisionFilter_l = {0};
VisionTrackFilter_t VisionFilter_r = {0};
MVPacketFilter_t MVFilter_l = {0};
MVPacketFilter_t MVFilter_r = {0};
MVPacket_t MVpacket_l = {
    .offset_x = 0,
    .offset_y = 0,
    .is_plant_find = 0,
    .in_window = 0,
    .tail = 0,
};
MVPacket_t MVpacket_r = {
    .offset_x = 0,
    .offset_y = 0,
    .is_plant_find = 0,
    .in_window = 0,
    .tail = 0,
};
volatile uint32_t MVpacket_l_seq = 0;
volatile uint32_t MVpacket_r_seq = 0;
ArduinoWorkCmd_t ArduinoWorkCmd = {
    .is_enable = 0,
    .car_speed = 0.0f,
    .tail = 0,
};
/**
 * @brief 识别分类 plant_all wj_l
 * @param plant_all 植物总数和各类数量统计
 * @param wj_l 左侧窗口控制和判断当前任务
 * @param wj_r 右侧窗口控制和判断当前任务
 */
PlantStats plant_all = {0};
WindowJudge_t wj_l = {
    .prev_mv = 0,
    .in_window = 0,
    .ever_seen_yellow = 0,
    .ever_seen_green = 0,
    .ever_seen_vision = 0,
    .type_now = 255,
};
WindowJudge_t wj_r = {
    .prev_mv = 0,
    .in_window = 0,
    .ever_seen_yellow = 0,
    .ever_seen_green = 0,
    .ever_seen_vision = 0,
    .type_now = 255,
};

/**
 * @brief 函数区域
 * 
 * @param 

 */
//opencv
#define half_vision 30
// Pi sends pixel offset from image center. For 640-wide input, half width is 320.
#define half_width 320
//openmv
#define half_mv_width 160
#define half_vision_cm 5.0f
//extend 
#define extend_max 1400
#define extend_wait 400
#define extend_pid_full_zone 2000
#define extend_pid_zone 150
#define extend_position_error 8
#define extend_fast_zone 650
#define extend_mid_zone 320
#define extend_fast_pwm 750
#define extend_mid_pwm 520
#define extend_approach_pwm 280
// rotate encoder safety limits (tune by mechanism)
#define rotate_encoder_min (-1300) //-200
#define rotate_encoder_max (1400)
#define rotate_extend_margin 10
// openmv encoder safety limits (relative encoder count, power-on origin = 0)
#define openmv_encoder_min (1000)
#define openmv_encoder_max (1900)

static inline int32_t clamp_int32(int32_t value, int32_t min, int32_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int8_t get_encoder_limit(MotorPidController *controller, int32_t *min, int32_t *max)
{
    if (controller == &qRotate || controller == &pRotate) {
        *min = rotate_encoder_min;
        *max = rotate_encoder_max;
        return 1;
    }
    if (controller == &qOpenmv || controller == &pOpenmv) {
        *min = openmv_encoder_min;
        *max = openmv_encoder_max;
        return 1;
    }
    return 0;
}

static int32_t clamp_encodertogo_with_limit(MotorPidController *controller, int32_t encodertogo)
{
    int32_t min_limit = 0;
    int32_t max_limit = 0;
    if (!controller || !get_encoder_limit(controller, &min_limit, &max_limit)) {
        return encodertogo;
    }

    int32_t current_encoder = MotorPidController_readencoder(controller);
    int32_t target_encoder = current_encoder + encodertogo;
    target_encoder = clamp_int32(target_encoder, min_limit, max_limit);
    return target_encoder - current_encoder;
}

static int8_t rotate_near_extend_limit(MotorPidController *controller)
{
    if (controller != &qRotate && controller != &pRotate) {
        return 0;
    }

    int32_t encoder = MotorPidController_readencoder(controller);
    return (encoder <= rotate_encoder_min + rotate_extend_margin ||
            encoder >= rotate_encoder_max - rotate_extend_margin);
}

static void constrain(float *value, float max, float min) {
    if (*value > max) *value = max;
    if (*value < min) *value = min;
}

#define PAL_TRACK_ERROR_SPEED_MAX 12000.0f
#define PID_INTEGRAL_MAX_DELTA_TICKS 100 // 1 ms, timer tick is 0.01 ms
#define PID_INTEGRAL_TICK_SECONDS 0.00001f

typedef struct {
    int8_t has_tracking_error;
    int8_t tracking_pid_active;
    int8_t extend_latched;
    int8_t is_target_stable;
    int32_t encodertogo;
    float error_speed_encoder;
    uint8_t target_in_limit;
    int32_t stable_start_time;
    int32_t unstable_start_time;
} PalTrackContext_t;

static PalTrackContext_t pal_track_l = {0};
static PalTrackContext_t pal_track_r = {0};

typedef struct {
    int8_t has_target_encoder;
    int32_t target_encoder;
    uint32_t last_frame_seq;
} MvTrackContext_t;

static MvTrackContext_t mv_track_l = {0};
static MvTrackContext_t mv_track_r = {0};

static PalTrackContext_t *get_pal_track_context(MotorPidController *controller)
{
    if (controller == &qRotate) return &pal_track_l;
    if (controller == &pRotate) return &pal_track_r;
    return &pal_track_l;
}

static MvTrackContext_t *get_mv_track_context(MotorPidController *controller)
{
    if (controller == &qOpenmv) return &mv_track_l;
    if (controller == &pOpenmv) return &mv_track_r;
    return &mv_track_l;
}

static uint32_t get_mv_packet_seq(MotorPidController *controller)
{
    if (controller == &qOpenmv) return MVpacket_l_seq;
    if (controller == &pOpenmv) return MVpacket_r_seq;
    return 0;
}

static int32_t target_encoder_from_offset(MotorPidController *controller, int16_t offset)
{
    int32_t current_encoder = MotorPidController_readencoder(controller);
    int32_t delta_encoder = calculate_angeltogo(controller, offset);
    int32_t limited_delta = clamp_encodertogo_with_limit(controller, delta_encoder);
    return current_encoder + limited_delta;
}

static int32_t encodertogo_from_target(MotorPidController *controller, int32_t target_encoder)
{
    int32_t current_encoder = MotorPidController_readencoder(controller);
    return clamp_encodertogo_with_limit(controller, target_encoder - current_encoder);
}

static void extend_goto_position_segment_pid(MotorPidController *controller, int32_t target_encoder, int32_t errorcount)
{
    int32_t current_encoder = MotorPidController_readencoder(controller);
    int32_t encodertogo = target_encoder - current_encoder;
    int32_t abs_error = abs((int)encodertogo);

    if (abs_error <= errorcount) {
        Motor_brake(&controller->m_motor);
        controller->integral_enc = 0;
        return;
    }

    if (abs_error <= extend_pid_zone) {
        zero_encodertogo(controller, extend_pid_full_zone, encodertogo, errorcount);
        return;
    }

    int32_t pwm = extend_approach_pwm;
    if (abs_error > extend_fast_zone) {
        pwm = extend_fast_pwm;
    } else if (abs_error > extend_mid_zone) {
        pwm = extend_mid_pwm;
    }

    Motor_set_pwm(&controller->m_motor, (encodertogo > 0) ? (int16_t)pwm : (int16_t)-pwm);
    controller->integral_enc = 0;
}




void track_and_extend_left(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error,
    int32_t unstable_error, int32_t unstable_ticks) {
    uint8_t state_l = 0;
    int32_t encodertogo_l = 0;
    float error_speed_encoder_l = 0.0f;
    uint8_t target_in_limit_l = 0U;

    __disable_irq();
    state_l = VisionFilter_l.state;
    encodertogo_l = VisionFilter_l.delta_encoder;
    error_speed_encoder_l = VisionFilter_l.error_speed_encoder;
    target_in_limit_l = VisionFilter_l.target_in_limit;
    __enable_irq();

    handle_palmotor(state_l, &qRotate,&qExtend,
                    encodertogo_l, error_speed_encoder_l, target_in_limit_l,
                    errorcount, minduring, freeze_integral_error,
                    unstable_error, unstable_ticks);
}

void track_and_extend_right(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error,
    int32_t unstable_error, int32_t unstable_ticks) {
    uint8_t state_r = 0;
    int32_t encodertogo_r = 0;
    float error_speed_encoder_r = 0.0f;
    uint8_t target_in_limit_r = 0U;

    __disable_irq();
    state_r = VisionFilter_r.state;
    encodertogo_r = VisionFilter_r.delta_encoder;
    error_speed_encoder_r = VisionFilter_r.error_speed_encoder;
    target_in_limit_r = VisionFilter_r.target_in_limit;
    __enable_irq();

    handle_palmotor(state_r, &pRotate, &pExtend,
                    encodertogo_r, error_speed_encoder_r, target_in_limit_r,
                    errorcount, minduring, freeze_integral_error,
                    unstable_error, unstable_ticks);
}

// Backward-compatible default: left mechanism.
void track_and_extend(int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error,
    int32_t unstable_error, int32_t unstable_ticks) {
    track_and_extend_left(errorcount, minduring, freeze_integral_error,
                          unstable_error, unstable_ticks);
}

void openmv_extend_left(int32_t mincontrol_count, int32_t errorcount){
    uint8_t is_plant_find_l = 0;
    int16_t offset_y_l = 0;

    __disable_irq();
    is_plant_find_l = MVpacket_l.is_plant_find;
    offset_y_l = MVpacket_l.offset_y;
    __enable_irq();

    handle_mvmotor(is_plant_find_l, offset_y_l, &qOpenmv, mincontrol_count, errorcount);
}

void openmv_extend_right(int32_t mincontrol_count, int32_t errorcount){
    uint8_t is_plant_find_r = 0;
    int16_t offset_y_r = 0;

    __disable_irq();
    is_plant_find_r = MVpacket_r.is_plant_find;
    offset_y_r = MVpacket_r.offset_y;
    __enable_irq();

    handle_mvmotor(is_plant_find_r, offset_y_r, &pOpenmv, mincontrol_count, errorcount);
}

// Backward-compatible default: left mechanism.
void openmv_extend(int32_t mincontrol_count, int32_t errorcount){
    openmv_extend_left(mincontrol_count, errorcount);
    openmv_extend_right(mincontrol_count, errorcount);
}

void PlantClassifier_UpdateLeft(void) {
    uint8_t mv_l = 0;
    uint8_t state_l = 0;
    __disable_irq();
    mv_l = MVpacket_l.in_window;
    state_l = VisionFilter_l.state;
    __enable_irq();

    process_packet(mv_l, state_l, &plant_all, &wj_l);
}

void PlantClassifier_UpdateRight(void) {
    uint8_t mv_r = 0;
    uint8_t state_r = 0;
    __disable_irq();
    mv_r = MVpacket_r.in_window;
    state_r = VisionFilter_r.state;
    __enable_irq();

    process_packet(mv_r, state_r, &plant_all, &wj_r);
}

void PlantClassifier_Update(void) {
    PlantClassifier_UpdateLeft();
    PlantClassifier_UpdateRight();
}

void LED_UpdateByType() {
    LED_UpdateByTypeAndDir(&wj_l, &LED_l);
    LED_UpdateByTypeAndDir(&wj_r, &LED_r);
}

static void handle_mv_plant(uint8_t is_plant_find)
{
    (void)is_plant_find;
}

void is_should_plant(void)
{
    handle_mv_plant(MVpacket_l.is_plant_find);
    handle_mv_plant(MVpacket_r.is_plant_find);
}

uint8_t handle_plant(void)
{
    return 0U;
}





int32_t calculate_angeltogo(MotorPidController *controller, int16_t offset)
{
    if (!controller) return 0;

    if (controller == &qRotate || controller == &pRotate) {
        if (controller->pulse_to_angel == 0.0f) return 0;
        float angle = atan2f((float)offset * tanf(half_vision * 3.1415926f / 180.0f), (float)half_width);
        return (int32_t)((angle * 180.0f / 3.1415926f) / controller->pulse_to_angel);
    }

    if (controller == &qOpenmv || controller == &pOpenmv) {
        if (controller->pulse_to_distance == 0.0f) return 0;
        float enc = ((float)offset / (float)half_mv_width) * half_vision_cm / controller->pulse_to_distance;
        return (int32_t)enc;
    }

    return 0;
}

void VisionTracking_ConvertTarget(
    MotorPidController *controller, int16_t offset_px, int16_t velocity_px_s,
    int32_t *delta_encoder, float *error_speed_encoder, uint8_t *target_in_limit)
{
    const float pi = 3.1415926f;
    int32_t local_delta_encoder = 0;
    float local_error_speed_encoder = 0.0f;
    uint8_t local_target_in_limit = 0U;

    if (controller && controller->pulse_to_angel != 0.0f) {
        const float image_scale =
            tanf((float)half_vision * pi / 180.0f) / (float)half_width;
        const float normalized_x = (float)offset_px * image_scale;
        const float encoder_per_rad = 180.0f / (pi * controller->pulse_to_angel);

        const int32_t current_encoder = MotorPidController_readencoder(controller);
        local_delta_encoder = (int32_t)lroundf(atanf(normalized_x) * encoder_per_rad);
        local_error_speed_encoder =
            image_scale * (float)velocity_px_s * encoder_per_rad /
            (1.0f + normalized_x * normalized_x);

        int32_t min_limit = 0;
        int32_t max_limit = 0;
        int32_t local_target_encoder = current_encoder + local_delta_encoder;
        if (get_encoder_limit(controller, &min_limit, &max_limit) &&
            local_target_encoder >= min_limit && local_target_encoder <= max_limit) {
            local_target_in_limit = 1U;
        }
    }

    if (delta_encoder) *delta_encoder = local_delta_encoder;
    if (error_speed_encoder) *error_speed_encoder = local_error_speed_encoder;
    if (target_in_limit) *target_in_limit = local_target_in_limit;
}

void VisionTracking_FilterSet(VisionTrackFilter_t *filter, uint8_t state,
                              int16_t offset,
                              int16_t velocity_x,
                              int32_t delta_encoder,
                              float error_speed_encoder,
                              uint8_t target_in_limit,
                              uint32_t frame_seq)
{
    if (!filter) {
        return;
    }

    uint8_t new_frame = (frame_seq != filter->frame_seq) || !filter->ready;

    filter->state = state;
    filter->target_in_limit = target_in_limit;

    if (new_frame || state != 2 || !target_in_limit) {
        filter->frame_seq = frame_seq;
        filter->raw_offset = offset;
        filter->raw_velocity_x = velocity_x;
        filter->raw_delta_encoder = delta_encoder;
        filter->raw_error_speed_encoder = error_speed_encoder;
    }
}

void OpenMV_FilterSet(MVPacketFilter_t *filter, int16_t offset_x, int8_t offset_y,
                      uint8_t is_plant_find, uint8_t in_window)
{
    if (!filter) {
        return;
    }

    filter->raw_offset_x = offset_x;
    filter->raw_offset_y = offset_y;
    filter->is_plant_find = is_plant_find;
    filter->in_window = in_window;
}

static void VisionTracking_FilterOne(VisionTrackFilter_t *filter)
{
    if (!filter) {
        return;
    }

    if (filter->state != 2 || !filter->target_in_limit) {
        filter->ready = 0;
        filter->delta_encoder = filter->raw_delta_encoder;
        filter->delta_encoder_f = (float)filter->raw_delta_encoder;
        filter->error_speed_encoder = 0.0f;
        return;
    }

    if (!filter->ready) {
        filter->delta_encoder_f = (float)filter->raw_delta_encoder;
        filter->error_speed_encoder = filter->raw_error_speed_encoder;
        filter->ready = 1;
    } else {
        filter->delta_encoder_f +=
            ((float)filter->raw_delta_encoder - filter->delta_encoder_f) *
            pal_track_target_filter_alpha;
        filter->error_speed_encoder +=
            (filter->raw_error_speed_encoder - filter->error_speed_encoder) *
            pal_track_speed_filter_alpha;
    }

    filter->delta_encoder = (int32_t)lroundf(filter->delta_encoder_f);
}

static int8_t OpenMV_ClampI8(int32_t value)
{
    if (value > 127) return 127;
    if (value < -128) return -128;
    return (int8_t)value;
}

static void OpenMV_FilterOne(MVPacketFilter_t *filter, MVPacket_t *packet,
                             volatile uint32_t *seq)
{
    if (!filter || !packet || !seq) {
        return;
    }

    if (filter->is_plant_find == 0U) {
        filter->ready = 0U;
        filter->offset_x_f = (float)filter->raw_offset_x;
        filter->offset_y_f = (float)filter->raw_offset_y;
    } else if (!filter->ready) {
        filter->offset_x_f = (float)filter->raw_offset_x;
        filter->offset_y_f = (float)filter->raw_offset_y;
        filter->ready = 1U;
    } else {
        filter->offset_x_f +=
            ((float)filter->raw_offset_x - filter->offset_x_f) *
            openmv_offset_filter_alpha;
        filter->offset_y_f +=
            ((float)filter->raw_offset_y - filter->offset_y_f) *
            openmv_offset_filter_alpha;
    }

    packet->offset_x = (int16_t)clamp_int32((int32_t)lroundf(filter->offset_x_f),
                                           -32768, 32767);
    packet->offset_y = OpenMV_ClampI8((int32_t)lroundf(filter->offset_y_f));
    packet->is_plant_find = filter->is_plant_find;
    packet->in_window = filter->in_window;
    packet->tail = 0xAA;
    (*seq)++;
}

void VisionTracking_FilterUpdate(void)
{
    VisionTracking_FilterOne(&VisionFilter_l);
    VisionTracking_FilterOne(&VisionFilter_r);
    OpenMV_FilterOne(&MVFilter_l, &MVpacket_l, &MVpacket_l_seq);
    OpenMV_FilterOne(&MVFilter_r, &MVpacket_r, &MVpacket_r_seq);
}

/**
 * @brief 在arduino发送的不工作命令enable=0下，该函数可以控制电机返航，即收回电机到开机时的位置
 * @param qExtend:返回开机位置encoder=0的位置，伸缩电机需要返回到初始位置以便下一次伸出
 * @param qRotate:返回开机位置encoder=0的位置，旋转电机需要返回到初始位置以便下一次旋转
 * @param qOpenmv:返回开机位置encoder=0的位置，伸缩电机需要返回到初始位置以便下一次伸缩
 */
void set_orign(MotorPidController *controller)
{
    if (controller == &qExtend || controller == &pExtend){  //0
    extend_goto_position_segment_pid(controller, 0, extend_position_error);
    }
    else if (controller == &qRotate || controller == &pRotate){//0
    zero_encodertogo(controller,440, 0-MotorPidController_readencoder(controller), 10);
    }
    else if (controller == &qOpenmv || controller == &pOpenmv){//0
    zero_encodertogo(controller,440, 0-MotorPidController_readencoder(controller), 10);
    }
}
/**
 * @brief 在arduino发送的工作命令enable=1下，该函数可以控制电机进入等待状态，等待状态下根据不同电机类型进行不同的处理
 * @param qExtend:等待保持在encodercount=400的位置
 * @param qRotate:等待保持在encoder=0，即垂直、初始位置
 * @param qOpenmv:等待保持在encoder=now_encoder，当前位置不动
 */
void set_waiting(MotorPidController *controller)
{   
    if (controller == &qExtend || controller == &pExtend){   // 400
    extend_goto_position_segment_pid(controller, extend_wait, extend_position_error);
    }
    else if (controller == &qRotate || controller == &pRotate){ //0
    zero_encodertogo(controller,440, 0-MotorPidController_readencoder(controller), 10);
    }
    else if (controller == &qOpenmv || controller == &pOpenmv){ // keep
    int32_t limit_encodertogo = clamp_encodertogo_with_limit(controller, 0);
    zero_encodertogo(controller,400, limit_encodertogo, 10);
    }
}
/**
 * @brief 扫黄电机之一，用于qExtend伸到最大值
 */
void set_extendMAX(MotorPidController *controller)
{   
    if (controller == &qExtend || controller == &pExtend){
    extend_goto_position_segment_pid(controller, extend_max, extend_position_error);
    }
}
void WindowJudge_Reset(WindowJudge_t *wj)
{
    wj->prev_mv = 0;
    wj->in_window = 0;
    wj->ever_seen_yellow = 0;
    wj->ever_seen_green = 0;
    wj->ever_seen_vision = 0;
    wj->type_now = 255;
}
/**
 * @brief 手动使能windowjudge进入工作状态，记录当前mv状态，设置in_window标志位，重置ever_seen和type_now
 * @param wj->in_window:用于判断当前是否在窗口内，窗口定义为植物离开openmv视野时window置1植物进入openmv视野时window置0（arduino从非工作区到工作区的第一个植物需要手动设置window=1），在window=1期间如果ever_seen_vision=1则根据state判断植物类型并统计数量，否则统计为0类植物，最后更新总数和当前类型
 * @param wj->type_now :用于记录当前识别到的植物类型，-1为未识别，0为空，1为绿，2为黄
 */
void WindowJudge_EnterWork(WindowJudge_t *wj , uint8_t curr_mv)
{
    wj->prev_mv = curr_mv;      
    wj->in_window = 1;          // setwindow：用于第一株
    wj->ever_seen_vision = 0;
    wj->ever_seen_green  = 0;
    wj->ever_seen_yellow = 0;
    wj->type_now = 255;
}
/**
 * @brief 主视觉识别分类函数，根据树莓派发送的state和openmv发送的is_plant_find进行分类统计，检测窗口为植物离开openmv视野时window置1植物进入openmv视野时window置0（arduino从非工作区到工作区的第一个植物需要手动设置window=1），在window=1期间如果ever_seen_vision=1则根据state判断植物类型并统计数量，否则统计为0类植物，最后更新总数和当前类型
 * @param state:树莓派识别到的目标类型，0为empty，1为绿色，2为黄色
 * @param plant_all: 植物总数和各类数量统计
 * @param wj: 左/右窗口控制和判断当前任务，wj->type_now用于记录当前识别到的植物类型，-1为未识别，0为空，1为绿，2为黄
 */
static void process_packet(uint8_t is_plant_find, uint8_t state, PlantStats *plant_all, WindowJudge_t *wj) {
    uint8_t curr_mv = is_plant_find;
    if (wj->in_window ) {
        wj->type_now = 255;
        if (state >= 1) {
            wj->ever_seen_vision = 1;
            if (state == 1) wj->ever_seen_green = 1;
            if (state == 2) wj->ever_seen_yellow = 1;
        }
    }
    if (!curr_mv && wj->prev_mv) {
        wj->in_window = 1;
        wj->ever_seen_vision = 0;
        wj->ever_seen_green = 0;
        wj->ever_seen_yellow = 0;
        if (state >= 1) {
            wj->ever_seen_vision = 1;
            if (state == 1) wj->ever_seen_green = 1;
            if (state == 2) wj->ever_seen_yellow = 1;
        }
    }
    if (curr_mv && !wj->prev_mv) {
        wj->in_window = 0;
        if (wj->ever_seen_vision == 1) {
            // both green and yellow are seen, prioritize yellow
            if (wj->ever_seen_yellow == 1) {
                plant_all->type_2_count++;
                wj->type_now = 2; 
            }
            else if (wj->ever_seen_green == 1) {
                plant_all->type_1_count++;
                wj->type_now = 1; 
            }
        }
        else {
            plant_all->type_0_count++;
            wj->type_now = 0;
        }
        plant_all->total_plants++;
    }
    wj->prev_mv = curr_mv;
}
/**
 * @brief 套筒下移后等待投种函数，根据openmv发送的is_plant_find判断是否投种。
 * @param MVpacket->offset_x ：投种位置判断
 * @param MVpacket->offset_y ：openmv extend电机对齐
 * @param MVpacket->is_plant_find
 */
static SowWaitDecision_t Decide_Sow_Wait_By_MVPacket(const MVPacket_t *MVpacket , const WindowJudge_t *wj)
{
    if (MVpacket == NULL) {
        return SOW_WAIT_GO_UP;
    }

    if (MVpacket->is_plant_find == 0U || wj->type_now != 0) {
        return SOW_WAIT_GO_UP;
    }

    int offset_y_abs = abs((int)MVpacket->offset_y);
    int offset_x_val = (int)MVpacket->offset_x;

    if (offset_y_abs < 30 &&
        offset_x_val > 60 &&
        offset_x_val < 100) {
        return SOW_WAIT_GO_SEED;
    }

    return SOW_WAIT_HOLD;
}

/**
 * @brief 自动化投种任务执行器
 * @param wj 指向对应的窗口判断结构体 (wj_l 或 wj_r)
 * @param move_m 电机1：负责垂直移动
 * @param seed_m 电机2：负责投种旋转
 */
void Execute_Sow_Sequence(WindowJudge_t *wj, MVPacket_t *MVpacket, Soft_Motor *move_m, Soft_Motor *seed_m)
{
    typedef enum {
        SOW_IDLE = 0,
        SOW_STEP1_DOWN,
        SOW_STEP2_WAIT,
        SOW_STEP3_SEED_FWD,
        SOW_STEP4_UP
    } SowSequence_t;

    typedef struct {
        SowSequence_t state;
        uint32_t timer_mark;
        uint8_t last_type_now;
    } SowSequenceCtx_t;

    static SowSequenceCtx_t ctx_l = { SOW_IDLE, 0U, 255U };
    static SowSequenceCtx_t ctx_r = { SOW_IDLE, 0U, 255U };
    static SowSequenceCtx_t ctx_default = { SOW_IDLE, 0U, 255U };

    SowSequenceCtx_t *ctx = &ctx_default;
    uint32_t now = TCM_Readtimer6count();  // 0.01 ms / tick

    if (wj == NULL || MVpacket == NULL || move_m == NULL || seed_m == NULL) {
        return;
    }

    if (wj == &wj_l) {
        ctx = &ctx_l;
    } else if (wj == &wj_r) {
        ctx = &ctx_r;
    }

    if (ctx->state == SOW_IDLE && wj->type_now == 0 && ctx->last_type_now != 0U) {
        ctx->state = SOW_STEP1_DOWN;
        ctx->timer_mark = now;
    }

    switch (ctx->state) {
        case SOW_STEP1_DOWN:
            Soft_Motor_set_pwm(move_m, 5);
            if (now - ctx->timer_mark >= 60000) {
                Soft_Motor_brake(move_m);
                ctx->state = SOW_STEP2_WAIT;
                ctx->timer_mark = now;
            }
            break;
        case SOW_STEP2_WAIT:
        {
            SowWaitDecision_t decision = Decide_Sow_Wait_By_MVPacket(MVpacket,wj);
            if (decision == SOW_WAIT_GO_SEED) {
                ctx->state = SOW_STEP3_SEED_FWD;
                ctx->timer_mark = now;
            } else if (decision == SOW_WAIT_GO_UP) {
                ctx->state = SOW_STEP4_UP;
                ctx->timer_mark = now;
            }
            break;
        }

        case SOW_STEP3_SEED_FWD:
			  if (now - ctx->timer_mark <= 3000){
		      Soft_Motor_set_pwm(seed_m, 9);
			  }
			  else if(now - ctx->timer_mark <= 11000){
		      Soft_Motor_set_pwm(seed_m, 2);

			  }	 else if(now- ctx->timer_mark <= 41000) {
			      Soft_Motor_brake(seed_m);

			  }
			  else if(now - ctx->timer_mark <= 44000) {
				  Soft_Motor_set_pwm(seed_m, -9);
			  } else if(now - ctx->timer_mark <= 52000) {
			      Soft_Motor_set_pwm(seed_m, -2);
			  }
            else if (now - ctx->timer_mark >= 52000) {
                Soft_Motor_brake(seed_m);
                ctx->state = SOW_STEP4_UP;
                ctx->timer_mark = now;
            }
            break;
        case SOW_STEP4_UP:
            if (now - ctx->timer_mark <= 5000) {
                Soft_Motor_brake(move_m);
            }
           else if (now - ctx->timer_mark <= 65000) {
                Soft_Motor_set_pwm(move_m, -5);
            }else if (now - ctx->timer_mark <= 70000) {
                Soft_Motor_set_pwm(move_m, -1);
            }else {
                Soft_Motor_brake(move_m);
                ctx->state = SOW_IDLE;
                ctx->timer_mark = now;
            }
            break;
        default:
            ctx->state = SOW_IDLE;
            break;
    }

    ctx->last_type_now = wj->type_now;
}



static void tracking_pid_control(MotorPidController *controller,
                                 int32_t mincontrol_count,
                                 int32_t encodertogo,
                                 float error_speed_encoder,
                                 int32_t freeze_integral_error);

/**
 * @brief 底层扫黄控制函数。树莓派发送目标位置差和Kalman速度，
 *        串口解析阶段已经换算为偏差编码器和偏差编码器速度。
 * @param state:树莓派识别到的目标类型，0为empty，1为绿色，2为黄色
 * @param controller_r: 用于控制旋转电机的pid控制器
 * @param controller_e: 用于控制伸缩电机的pid控制器
 * @param encodertogo: 滤波后的视觉偏差编码器，直接作为旋转控制误差
 * @param error_speed_encoder: Kalman vx换算后的偏差编码器速度，单位count/s
 * @param target_in_limit: 目标编码器位置是否位于机械限位内
 * @param errorcount：误差范围，绝对值小于等于errorcount时认为到达目标位置
 * @param minduring：目标稳定所需的最小时间，超过时extend电机进入set_extendMAX状态
 * @param freeze_integral_error：PID积分冻结阈值，绝对值小于等于该值时不再累加积分
 */
void handle_palmotor(uint8_t state,
    MotorPidController *controller_r,MotorPidController *controller_e,
    int32_t encodertogo, float error_speed_encoder, uint8_t target_in_limit,
    int32_t errorcount, int32_t minduring,
    int32_t freeze_integral_error,
    int32_t unstable_error, int32_t unstable_ticks) {
    PalTrackContext_t *ctx = get_pal_track_context(controller_r);
    int32_t now = TCM_Readtimer6count();

    if (state == 2) {
        ctx->encodertogo = encodertogo;
        ctx->error_speed_encoder = error_speed_encoder;
        ctx->target_in_limit = target_in_limit;
        ctx->has_tracking_error = 1;
    }

    if (state != 2 || !ctx->has_tracking_error) {
        if (ctx->has_tracking_error || ctx->extend_latched || ctx->tracking_pid_active) {
            MotorPidController_Reset(controller_r);
            if (controller_e) MotorPidController_Reset(controller_e);
        }
        ctx->has_tracking_error = 0;
        ctx->tracking_pid_active = 0;
        ctx->extend_latched = 0;
        ctx->is_target_stable = 0;
        ctx->stable_start_time = 0;
        ctx->unstable_start_time = 0;
        zero_encodertogo(
            controller_r, 440, -MotorPidController_readencoder(controller_r), 10);
        if (controller_e) set_waiting(controller_e);
        return;
    }

    if (!ctx->target_in_limit) {
        if (ctx->tracking_pid_active) {
            MotorPidController_Reset(controller_r);
            ctx->tracking_pid_active = 0;
        }
        zero_encodertogo(
            controller_r, 800, clamp_encodertogo_with_limit(controller_r, ctx->encodertogo), 10);
        ctx->extend_latched = 0;
        ctx->is_target_stable = 0;
        ctx->stable_start_time = 0;
        ctx->unstable_start_time = 0;
        if (controller_e) set_waiting(controller_e);
        return;
    }

    if (!ctx->tracking_pid_active) {
        MotorPidController_Reset(controller_r);
        controller_r->lastTime = now;
        ctx->tracking_pid_active = 1;
    }

    encodertogo = ctx->encodertogo;

    if (abs(encodertogo) > unstable_error) {
        if (ctx->unstable_start_time == 0) {
            ctx->unstable_start_time = now;
        } else if (now - ctx->unstable_start_time > unstable_ticks) {
            MotorPidController_Reset(controller_r);
            if (controller_e) MotorPidController_Reset(controller_e);
            ctx->has_tracking_error = 0;
            ctx->tracking_pid_active = 0;
            ctx->extend_latched = 0;
            ctx->is_target_stable = 0;
            ctx->stable_start_time = 0;
            ctx->unstable_start_time = 0;
            zero_encodertogo(
                controller_r, 440, -MotorPidController_readencoder(controller_r), 10);
            if (controller_e) set_waiting(controller_e);
            return;
        }
    } else {
        ctx->unstable_start_time = 0;
    }

    tracking_pid_control(controller_r, 1200, encodertogo,
                         ctx->error_speed_encoder, freeze_integral_error);

    if (rotate_near_extend_limit(controller_r)) {
        ctx->extend_latched = 0;
        ctx->is_target_stable = 0;
        ctx->stable_start_time = 0;
        ctx->unstable_start_time = 0;
        if (controller_e) set_waiting(controller_e);
        return;
    }

    if (!ctx->extend_latched && abs(encodertogo) < errorcount) {
        if (!ctx->is_target_stable) {
            ctx->stable_start_time = now;
            ctx->is_target_stable = 1;
        } else if (now - ctx->stable_start_time > minduring) {
            ctx->extend_latched = 1;
            ctx->unstable_start_time = 0;
        }
    } else if (!ctx->extend_latched) {
        ctx->is_target_stable = 0;
        ctx->stable_start_time = 0;
    }

    if (ctx->extend_latched) {
        if (controller_e) set_extendMAX(controller_e);
    } else {
        if (controller_e) set_waiting(controller_e);
    }
}
/**
 * @brief 底层openmv电机控制函数，适用于根据openmv的offset进行位置控制的场景,这里用于Openmv电机,is_blob_find = 1时根据offset进行控制，is_blob_find = 0时进入等待状态set_waiting();
 * @param is_blob_find:openmv是否稳定识别到了目标，1为识别到且稳定，0为未识别到或不稳定
 * @param offset: openmv识别到的目标与中心的dy偏移量，单位为像素，正负表示方向
 * @param errorcount：误差范围，绝对值小于等于errorcount时认为到达目标位置
 */
void handle_mvmotor(uint8_t is_blob_find, int16_t offset, MotorPidController *controller,
     int32_t mincontrol_count, int32_t errorcount) {
    MvTrackContext_t *ctx = get_mv_track_context(controller);
    if (is_blob_find == 1) {
        uint32_t current_frame_seq = get_mv_packet_seq(controller);
        if (!ctx->has_target_encoder || current_frame_seq != ctx->last_frame_seq) {
            ctx->target_encoder = target_encoder_from_offset(controller, offset);
            ctx->last_frame_seq = current_frame_seq;
            ctx->has_target_encoder = 1;
        }

        int32_t encodertogo = encodertogo_from_target(controller, ctx->target_encoder);
        zero_encodertogo(controller, mincontrol_count, encodertogo, errorcount);
    } else {
        ctx->has_target_encoder = 0;
        set_waiting(controller);
    }
}
/**
 * @brief 视觉追踪PID，结构类似freeze_encodertogo，用Kalman偏差速度作为D项。
 * @param controller: 旋转电机PID控制器
 * @param mincontrol_count: 大于该编码器误差时直接满PWM追踪
 * @param encodertogo: 滤波后的视觉偏差编码器，直接作为控制误差
 * @param error_speed_encoder: Kalman vx换算后的偏差编码器速度，单位count/s
 * @param freeze_integral_error: 积分冻结阈值，绝对值小于等于该值时不再累加积分
 */
static void tracking_pid_control(MotorPidController *controller,
                                 int32_t mincontrol_count,
                                 int32_t encodertogo,
                                 float error_speed_encoder,
                                 int32_t freeze_integral_error)
{
    constrain(&error_speed_encoder, PAL_TRACK_ERROR_SPEED_MAX, -PAL_TRACK_ERROR_SPEED_MAX);

    if (encodertogo > mincontrol_count) {
        Motor_set_pwm(&controller->m_motor, 800);
        controller->integral_enc = 0;
        return;
    } else if (encodertogo < -mincontrol_count) {
        Motor_set_pwm(&controller->m_motor, -800);
        controller->integral_enc = 0;
        return;
    } else if (abs((int)encodertogo) <= freeze_integral_error) {
        controller->kp_term = controller->kp_w * (float)encodertogo;
        controller->ki_term = controller->ki_w * controller->integral_enc;
        controller->kd_term = controller->kd_w * error_speed_encoder;

        float pwm = controller->kp_term + controller->ki_term + controller->kd_term;
        constrain(&pwm, 1000.0f, -1000.0f);
        Motor_set_pwm(&controller->m_motor, (int16_t)pwm);
        return;
    }

    const int32_t now = TCM_Readtimer6count();
    int32_t delta_ticks = now - controller->lastTime;
    if (delta_ticks > 0) {
        if (delta_ticks > PID_INTEGRAL_MAX_DELTA_TICKS) {
            delta_ticks = PID_INTEGRAL_MAX_DELTA_TICKS;
        }
        controller->integral_enc +=
            (float)encodertogo * ((float)delta_ticks * PID_INTEGRAL_TICK_SECONDS);
        controller->lastTime = now;
        constrain(&controller->integral_enc,
                  controller->integral_max_w, -controller->integral_max_w);
    }

    controller->kp_term = controller->kp_w * (float)encodertogo;
    controller->ki_term = controller->ki_w * controller->integral_enc;
    controller->kd_term = controller->kd_w * error_speed_encoder;

    float pwm = controller->kp_term + controller->ki_term + controller->kd_term;
    constrain(&pwm, 1000.0f, -1000.0f);

    Motor_set_pwm(&controller->m_motor, (int16_t)pwm);
}
/**
 * @brief 底层pid控制函数，冻结积分并保持，适用于稳态是运动的控制，这里用在用于Rotate电机
 * @param mincontrol_count:进入控制的最小encodertogo,大于mincontrol_count时pwm满偏(12v)
 * @param encodertogo：当前编码器与目标位置的差值，单位为编码器脉冲数
 * @param errorcount：误差范围，绝对值小于等于errorcount时认为到达目标位置，此时冻结integral_enc,其他keep
 */
void freeze_encodertogo(MotorPidController *controller,int32_t mincontrol_count, int32_t encodertogo, int32_t errorcount)
{
    if (encodertogo > mincontrol_count){
    Motor_set_pwm(&controller->m_motor, 800);
    controller->integral_enc = 0;
    } else if (encodertogo < -mincontrol_count){
    Motor_set_pwm(&controller->m_motor, -800);
    controller->integral_enc = 0;
    } else if (abs(encodertogo) <= errorcount) {
        controller->kp_term = 0;  // 死区内不再用P放大噪声（可选：用很小的kp_hold）
        controller->ki_term = controller->ki_v * controller->integral_enc;   // 积分保持
        controller->kd_term = controller->kd_v * controller->m_encoder.speed_enc;

        float pwm = controller->ki_term + controller->kd_term; // 速度阻尼帮助停抖，积分负责抗扰偏置
        Motor_set_pwm(&controller->m_motor, (int16_t)pwm);
    } else {
    int32_t CurrentTime = TCM_Readtimer6count();
    int32_t delta_ticks = CurrentTime - controller->lastTime;
    if (delta_ticks > 0) {
        if (delta_ticks > PID_INTEGRAL_MAX_DELTA_TICKS) {
            delta_ticks = PID_INTEGRAL_MAX_DELTA_TICKS;
        }
        controller->integral_enc += encodertogo * ((float)delta_ticks * PID_INTEGRAL_TICK_SECONDS);
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
/**
 * @brief 底层pid控制函数，清除积分并刹车，适用于稳态是静止的控制，这里用在Extend、Openmv电机
 * @note
 * @param mincontrol_count:进入控制的最小encodertogo,大于mincontrol_count时pwm满偏(12v)
 * @param encodertogo：当前编码器与目标位置的差值，单位为编码器脉冲数
 * @param errorcount：误差范围，绝对值小于等于errorcount时认为到达目标位置，此时清除integral_enc积分,in1 in2同时置0制动
 */
void zero_encodertogo(MotorPidController *controller,int32_t mincontrol_count, int32_t encodertogo, int32_t errorcount)
{

    if (encodertogo > mincontrol_count){
    Motor_set_pwm(&controller->m_motor, 800);
    controller->integral_enc = 0;
    } else if (encodertogo < -mincontrol_count){
    Motor_set_pwm(&controller->m_motor, -800);
    controller->integral_enc = 0;
    } else if (abs(encodertogo) <= errorcount){
        Motor_brake(&controller->m_motor);
        controller->integral_enc = 0;    // 清除积分  这里稳态是0 固清除积分
    } else {
    int32_t CurrentTime = TCM_Readtimer6count();
    int32_t delta_ticks = CurrentTime - controller->lastTime;
    if (delta_ticks > 0) {
        if (delta_ticks > PID_INTEGRAL_MAX_DELTA_TICKS) {
            delta_ticks = PID_INTEGRAL_MAX_DELTA_TICKS;
        }
        controller->integral_enc += encodertogo * ((float)delta_ticks * PID_INTEGRAL_TICK_SECONDS);
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
