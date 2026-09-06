#include <avr/interrupt.h>
#include <util/atomic.h>
#include <MsTimer2.h>
#include "encoder.h"

//函数声明
void encoderInit();
void calculateImpulseSpeed();
static inline int8_t readEncoderStep
(uint8_t previousPins, uint8_t currentPins, uint8_t phaseAMask, uint8_t phaseBMask);





void setup() {
    Serial.begin(115200);
    encoderInit();
    MsTimer2::set(20, calculateImpulseSpeed);
    MsTimer2::start();
    // 全局中断已在 encoderInit() 结束时恢复使能，无需再调用 sei()
}


void loop() {
    static int32_t a;
    static unsigned long lastPrintMs = 0;
    const unsigned long now = millis();
    a++;



    // 10ms 打印一次
    if (now - lastPrintMs < 10UL) {
        return;
    }
    
    // AVR 上 long 是 4 字节，复制计数值时要防止中断在中途改写。
    long countD;
    long countQ;
    long countP;
    long countB;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        countD = pos_d;
        countQ = pos_q;
        countP = pos_p;
        countB = pos_b;
    }

    // 需要速度(mm/s)时：读中断算好的平均计数值，在这里乘系数换算
    float vX, vY, vq;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        vX = impulseSpeed_X;
        vY = impulseSpeed_Y;
        vq = impulseSpeed_q;
    }
    vX = vX * COEFF_SPEED_IMPULSE2LINEAR; // 前为正
    vq = vq * COEFF_SPEED_IMPULSE2LINEAR;

    //Serial.print("d:");
    //Serial.print(countD);
    Serial.print(" q:");
    Serial.print(countQ);
    //Serial.print(" p:");
    //Serial.print(countP);
    //Serial.print(" b:");
    //Serial.print(countB);
    Serial.print(" vq:");
    Serial.print(vq);
    //Serial.print(" vX:");
    //Serial.print(vX);
    Serial.print(" delta:");
    Serial.println((double)(now - lastPrintMs) / a);
    a = 0;
    lastPrintMs = now;
}

static inline int8_t readEncoderStep(
    uint8_t previousPins,
    uint8_t currentPins,
    uint8_t phaseAMask,
    uint8_t phaseBMask
) {
    const uint8_t previousAB =
        ((previousPins & phaseAMask) ? 2U : 0U)    // 等价于 ((previousPins & phaseAMask) != 0) << 1
        | ((previousPins & phaseBMask) ? 1U : 0U); // 等价于 ((previousPins & phaseBMask) != 0) << 0
    const uint8_t currentAB =
        ((currentPins & phaseAMask) ? 2U : 0U)
        | ((currentPins & phaseBMask) ? 1U : 0U);

    return QUADRATURE_STEP_TABLE[(previousAB << 2) | currentAB]; // 组成 4 位二进制数，用于查表
}
void encoderInit() {
    pinMode(COUNTER_d_A, INPUT_PULLUP);
    pinMode(COUNTER_d_B, INPUT_PULLUP);
    pinMode(COUNTER_q_A, INPUT_PULLUP);
    pinMode(COUNTER_q_B, INPUT_PULLUP);
    pinMode(COUNTER_p_A, INPUT_PULLUP);
    pinMode(COUNTER_p_B, INPUT_PULLUP);
    pinMode(COUNTER_b_A, INPUT_PULLUP);
    pinMode(COUNTER_b_B, INPUT_PULLUP);

    
    PCICR |= (1 << PCIE2);           // 使能第二组PCINT
    PCMSK2 = ENCODER_PCINT_MASK;     // 使能编码器引脚中断
    lastState_PCINT = PINK;          //  记录真实初始状态
         
}
// 每 20 ms 由 MsTimer2 调用：4 次整数减 + 2 次浮点除，约 20 µs
// 变量含义与换算（需要速度 mm/s 时）：
//   轮增量 impulseSpeed_*：20 ms 窗口的计数增量 × COEFF_SPEED_IMPULSE2LINEAR（9.52）
//   车体 impulseSpeed_X/Y：已除 4 的平均计数值 × COEFF_SPEED_IMPULSE2LINEAR
void calculateImpulseSpeed(){
    impulseSpeed_p = pos_p - lastPos_p;
    impulseSpeed_b = pos_b - lastPos_b;
    impulseSpeed_q = pos_q - lastPos_q;
    impulseSpeed_d = pos_d - lastPos_d;
    impulseSpeed_X = (impulseSpeed_q + impulseSpeed_d + impulseSpeed_p + impulseSpeed_b) / 4.0f;  //  前为正
    impulseSpeed_Y = (impulseSpeed_q - impulseSpeed_d - impulseSpeed_p + impulseSpeed_b) / 4.0f;  //  右为正
    lastPos_p = pos_p;
    lastPos_b = pos_b;
    lastPos_q = pos_q;
    lastPos_d = pos_d;
}

ISR(PCINT2_vect) {
    // 中断中只读取一次端口，四个编码器都使用同一份稳定快照。
    const uint8_t current = PINK;
    const uint8_t previous = lastState_PCINT;
    const uint8_t changed = (current ^ previous) & ENCODER_PCINT_MASK;
    
    if (changed & ENCODER_D_MASK) {    //   B00001010 & B00000011 = B00000010 -> True
        pos_d += readEncoderStep(
            previous, current, ENCODER_D_A_MASK, ENCODER_D_B_MASK);
    }
    if (changed & ENCODER_Q_MASK) {
        pos_q += readEncoderStep(
            previous, current, ENCODER_Q_A_MASK, ENCODER_Q_B_MASK);
    }
    if (changed & ENCODER_P_MASK) {
        pos_p += readEncoderStep(
            previous, current, ENCODER_P_A_MASK, ENCODER_P_B_MASK);
    }
    if (changed & ENCODER_B_MASK) {
        pos_b += readEncoderStep(
            previous, current, ENCODER_B_A_MASK, ENCODER_B_B_MASK);
    }

    lastState_PCINT = current;
}







