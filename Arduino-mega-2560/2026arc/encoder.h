#ifndef CAR_H
#define CAR_H

#include <Arduino.h>

/*-----------------------------编码器引脚配置-----------------------------------*/
// Mega2560 的 PCINT2 组对应 A8-A15。四个编码器的 A/B 引脚只在这里设置。
// 交换某个编码器的 A、B 定义，该编码器的计数方向会自动反转。
#define COUNTER_d_A A8     
#define COUNTER_d_B A9
#define COUNTER_q_A A12
#define COUNTER_q_B A13
#define COUNTER_p_A A14
#define COUNTER_p_B A15
#define COUNTER_b_A A10
#define COUNTER_b_B A11

// Mega2560 上 A8-A15 按顺序对应 PINK 的 bit 0-bit 7。
// A8-A15定义为 ： 62-63-64-65-66-67-68-69 引脚号
// static_cast<uint8_t> 转换为如B00000000 , 无符号8位整数。
// constexpr 确保编译器就对这些值进行计算，避免运行时计算，提高效率。 A8 -> B00000001  A9 -> B00000010 ...
constexpr uint8_t ENCODER_D_A_MASK = static_cast<uint8_t>(1U << (COUNTER_d_A - A8));     // A8 -A8 = 0 B00000001
constexpr uint8_t ENCODER_D_B_MASK = static_cast<uint8_t>(1U << (COUNTER_d_B - A8));
constexpr uint8_t ENCODER_Q_A_MASK = B00010000;
constexpr uint8_t ENCODER_Q_B_MASK = B00100000;
constexpr uint8_t ENCODER_P_A_MASK = static_cast<uint8_t>(1U << (COUNTER_p_A - A8));
constexpr uint8_t ENCODER_P_B_MASK = static_cast<uint8_t>(1U << (COUNTER_p_B - A8));
constexpr uint8_t ENCODER_B_A_MASK = static_cast<uint8_t>(1U << (COUNTER_b_A - A8));
constexpr uint8_t ENCODER_B_B_MASK = static_cast<uint8_t>(1U << (COUNTER_b_B - A8));

constexpr uint8_t ENCODER_D_MASK = ENCODER_D_A_MASK | ENCODER_D_B_MASK; // B00000011
constexpr uint8_t ENCODER_Q_MASK = ENCODER_Q_A_MASK | ENCODER_Q_B_MASK;
constexpr uint8_t ENCODER_P_MASK = ENCODER_P_A_MASK | ENCODER_P_B_MASK;
constexpr uint8_t ENCODER_B_MASK = ENCODER_B_A_MASK | ENCODER_B_B_MASK;
constexpr uint8_t ENCODER_PCINT_MASK =
    ENCODER_D_MASK | ENCODER_Q_MASK | ENCODER_P_MASK | ENCODER_B_MASK;


// AB 状态使用 A 作为高位、B 作为低位。
// 索引为 (上一次 AB 状态 << 2) | 当前 AB 状态；非法双位跳变记为 0。
static const int8_t QUADRATURE_STEP_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


/*--------------------------编码器计数、测速相关变量------------------------------*/
//记录A8-A15引脚在变化前的引脚状态
volatile uint8_t lastState_PCINT = B00000000;
//用于存放编码器的计数值
volatile long pos_d = 0;
volatile long pos_q = 0;
volatile long pos_p = 0;
volatile long pos_b = 0;   // int 16    -2**15 2**15-1      2**15 * 0.1906 /1000   = 6.24m   3
//计算速度的时候用到的上一次测速时的编码器的计数值
volatile long lastPos_d = 0;
volatile long lastPos_q = 0;
volatile long lastPos_p = 0;
volatile long lastPos_b = 0;
//一个脉冲对应的轮子转过距离
#define COEFF_LENTH_IMPULSE2LINEAR 0.1904f
//脉冲速度转换为轮子线速度的比例系数
#define COEFF_SPEED_IMPULSE2LINEAR 9.52f
//每 20 ms 窗口的脉冲增量（计数）：需要速度(mm/s)时再乘以 COEFF_SPEED_IMPULSE2LINEAR
volatile long impulseSpeed_d = 0;
volatile long impulseSpeed_q = 0;
volatile long impulseSpeed_p = 0;
volatile long impulseSpeed_b = 0;
//由四个轮子各自转速合成的小车平移速度：换算 mm/s 时乘以 COEFF_SPEED_IMPULSE2LINEAR
volatile float impulseSpeed_X = 0; //前后方向速度，向前为正
volatile float impulseSpeed_Y = 0; //左右方向速度，向左为正




#endif
