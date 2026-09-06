#ifndef PCencoder_h
#define PCencoder_h

#if defined(ARDUINO_ARCH_AVR)
//#include "avr/PCencoderTimer2.h"
#else
#error "This library only supports boards with an AVR."
#endif
#define INTEGRAL_MAX 8.0f  // 积分上限
#define INTEGRAL_MIN -8.0f // 积分下限
#define DEAD_VOLTAGE 23.0f    // 死区电压
#define SPEED_ZERO_TIMEOUT_COUNTS 1200UL   // ≈ 80ms

#define PCENCODER_GROUP0_SLOTS   //  禁用
#define PCENCODER_GROUP1_SLOTS   //  禁用
#define PCENCODER_GROUP2_SLOTS   ,0,1,2,3//  禁用

#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdint.h>

// PCencoder 主类
class PCencoder
{
public:
    // 中断触发模式
    enum Mode : uint8_t { _CHANGE = 0, _RISING, _FALLING };
    class QEncoder;
    
private:
    // PCINT注册的编码器指针结构体
    struct Slot {
        bool inUse = false;
        uint8_t mask ;
        uint8_t groupA ;
        uint8_t groupB ;
        uint8_t maskA ;
        uint8_t maskB ;
        QEncoder* encoderPtr = nullptr;
    };
    /*链表没有预编译的优势，所以用数组代替
    struct SlotNode {
        Slot slot;      // 存储一个 Slot
        SlotNode* next; // 指向下一个节点
        SlotNode( Slot && theslot , SlotNode* thenext = nullptr)
          : slot(std::move(theslot)), next(thenext) {}
        SlotNode( const Slot & theslot , SlotNode* thenext = nullptr)
          : slot(theslot), next(thenext) {}
    };
    */
    // 静态成员声明
    static volatile uint8_t last_[3];
    static Slot slots_[3][8];

public:
    // 端口中断处理与注册接口
    class PCInt
    {
    public:
        static bool attach(uint8_t pin1, uint8_t pin2, Mode mode = _CHANGE, bool pullup = true);
        static void detach(uint8_t pin);
        static void handle(uint8_t g);
        static int findFreeSlot(uint8_t g);
        static int findEncoder(uint8_t g, QEncoder *encoder);
        
        // 主模板声明
        template<uint8_t G, uint8_t... Sequece>
        static void handleGroup();
        
        
    private:
        // 递归展开参数包的辅助模板函数
        template<uint8_t G>
        static void handleSlotsRecursive(uint8_t changed);
        template<uint8_t G, uint8_t First, uint8_t... Rest>  
        static void handleSlotsRecursive(uint8_t changed);
        template<uint8_t G, uint8_t I>
        static void handleSlot(uint8_t changed);
        static inline volatile uint8_t* pinReg(uint8_t g);
        static inline volatile uint8_t* mskReg(uint8_t g);
        static inline uint8_t pcicrMask(uint8_t g);
    
        friend void PCINT0_vect_func();
        friend void PCINT1_vect_func();
        friend void PCINT2_vect_func();
    };

    // 四分频编码器类
    class QEncoder
    {
        friend class PCencoder;
    public:
        QEncoder(uint8_t pinA, uint8_t pinB);
        ~QEncoder();
        bool begin(bool pullup = true);
        long read() const;
        void write(long v);
        void _handleChange(uint8_t idx);
        void handleChange();

    private:
        // M/T测速相关变量   分辨率为1/M2
        bool sampled = 0;                                            //  M/T测速是否采样完成标志
        bool sampleSpeed = 0;                                         //  M/T测速是否采样标志
        unsigned long  timerThisCount = 0;                            //  记录高频定时器这次值
        unsigned long  timerLastCount = 0;                            //  记录高频定时器上次值
        unsigned long  timerRelativeCount = 0;                        //  记录高频定时器相对值
        long long encoderThisCount = 0;                               //  记录编码器这次值
        long long encoderLastCount = 0;                               //  记录编码器上次值
        long long encoderRelativeCount = 0;                           //  记录编码器相对
        uint8_t a_, b_, maskA, maskB;
        volatile long cnt_ = 0;
        volatile uint8_t prev_ = 0;
        volatile uint8_t* pinRegA_;
        volatile uint8_t* pinRegB_;
        static const int8_t qtab_[16];
    };

    // 简单电机驱动封装
    class DCmotor
    {
    public:
        DCmotor(uint8_t IN1, uint8_t IN2, uint8_t PWM);
        ~DCmotor();
        void go(unsigned int value);
        void back(unsigned int value);
        void down();
        void brake();

    private:
        uint8_t in1, in2, pwm;
        bool isAttached = false;
    };

public:
    PCencoder();
    ~PCencoder();
    void brake();
    void on();
    bool attach(uint8_t in1, uint8_t in2, uint8_t pwm, 
                uint8_t en1, uint8_t en2, double k);
    long long readencoder() const;
    double readpwm() const;
    bool attachmotor(uint8_t in1, uint8_t in2, uint8_t pwm);
    bool attachEncoder(uint8_t en1, uint8_t en2);
    void setpid(double kp, double ki, double kd, double PULSE_TO_DISTANCE);
    void setspeed(double value);
    void sampleSpeed();
    void update();void updateSpeed();void updatePWM();void updateSpeed0();
    void motorGo(unsigned int v)  { if (m_motor) m_motor->go(v); }
    void motorBack(unsigned int v) { if (m_motor) m_motor->back(v); }
    void updatePWM_NEW();
    double getPWM();
    bool braked() const;
    double readspeed() const;
    double readspeedf() const;
    static void   _setTimer3();
    static void addTimerOverflow(uint32_t inc);
    static uint32_t readTimer3Counts()  ;
    void updateSpeedInloop();


    
    




private:
    QEncoder* m_encoder = nullptr;
    DCmotor* m_motor = nullptr;
    static volatile uint32_t timer3Overflow;                         // 静态计数器
    bool _brake = false;
    
    double kp = 0, ki = 0, kd = 0,PULSE_TO_DISTANCE = 0.0;
    bool running = false;
    unsigned long  CurrentTime = 0;unsigned long LastTime = 0;    //   更新pwm积分时间
    double speedf = 0;   // 滤波后的速度
    double speed = 0, speedError = 0, lastspeedError = 0;
    double speedErrorDerivative = 0, speedErrorIntegral = INTEGRAL_MAX/2;
    
    double T_speed = 0;
    double Interval = 0.02;                            //  M法默认时间  以及M/T法低频定时器采样时间
    double pwm = 0;
};



#endif // PCencoder_h