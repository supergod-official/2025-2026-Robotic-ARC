#ifndef PCencoder_h
#define PCencoder_h

#if defined(ARDUINO_ARCH_AVR)
#else
#error "This library only supports boards with an AVR."
#endif

#define SPEED_ZERO_TIMEOUT_COUNTS 1200UL

#define PCENCODER_GROUP0_SLOTS
#define PCENCODER_GROUP1_SLOTS
#define PCENCODER_GROUP2_SLOTS   ,0,1,2,3

#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stddef.h>

class PCencoder {
public:
    enum Mode : uint8_t { _CHANGE = 0, _RISING, _FALLING };
    class QEncoder;

private:
    struct Slot {
        bool inUse = false;
        uint8_t mask;
        uint8_t groupA;
        uint8_t groupB;
        uint8_t maskA;
        uint8_t maskB;
        QEncoder* encoderPtr = nullptr;
    };

    static volatile uint8_t last_[3];
    static Slot slots_[3][8];

public:
    class PCInt {
    public:
        static bool attach(uint8_t pin1, uint8_t pin2, Mode mode = _CHANGE, bool pullup = true);
        static void detach(uint8_t pin);
        static int findFreeSlot(uint8_t g);

        template<uint8_t G, uint8_t... Sequence>
        static void handleGroup();

    private:
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

    class QEncoder {
        friend class PCencoder;

    public:
        QEncoder(uint8_t pinA, uint8_t pinB);
        ~QEncoder();
        bool begin(bool pullup = true);
        long read() const;
        void write(long v);
        void _handleChange(uint8_t idx);

    private:
        bool sampled = 0;
        bool sampleSpeed = 0;
        unsigned long timerThisCount = 0;
        unsigned long timerLastCount = 0;
        long long encoderThisCount = 0;
        long long encoderLastCount = 0;
        uint8_t a_, b_, maskA, maskB;
        volatile long cnt_ = 0;
        volatile uint8_t prev_ = 0;
        volatile uint8_t* pinRegA_;
        volatile uint8_t* pinRegB_;
        static const int8_t qtab_[16];
    };

    class DCmotor {
    public:
        DCmotor(uint8_t IN1, uint8_t IN2, uint8_t PWM);
        ~DCmotor();
        void go(unsigned int value);
        void back(unsigned int value);
        void down();
        void brake();

    private:
        enum Direction : uint8_t {
            DIR_BRAKE = 0,
            DIR_GO,
            DIR_BACK
        };

        uint8_t in1, in2, pwm;
        bool isAttached = false;
        Direction currentDirection = DIR_BRAKE;
    };

public:
    PCencoder();
    ~PCencoder();

    bool attach(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t en1, uint8_t en2, double k);
    bool attachmotor(uint8_t in1, uint8_t in2, uint8_t pwm);
    bool attachEncoder(uint8_t en1, uint8_t en2);

    void setpid(double kp, double ki, double kd, double pulseToDistance, double deadvoltage = 0.0, double integralmax = 0.0);
    void setspeed(double value);
    void setPwm(int16_t pwm);
    void clearIntegral();

    void brake();
    void sampleSpeed();
    void requestMotionSample();
    void updateMotionInloop();
    void filterMotionInloop();

    void motorGo(unsigned int v) { if (m_motor) m_motor->go(v); }
    void motorBack(unsigned int v) { if (m_motor) m_motor->back(v); }

    long long readencoder() const;
    double readpwm() const;
    double readspeed() const;
    double readspeedf() const;
    double readaccel() const;
    double readaccelf() const;
    double readKpTerm() const;
    double readKiTerm() const;
    double readKdTerm() const;

    static void _setTimer3();
    static void addTimerOverflow(uint32_t inc);
    static uint32_t readTimer3Counts();

private:
    QEncoder* m_encoder = nullptr;
    DCmotor* m_motor = nullptr;
    static volatile uint32_t timer3Overflow;

    double kp = 0;
    double ki = 0;
    double kd = 0;
    double PULSE_TO_DISTANCE = 0.0;

    unsigned long CurrentTime = 0;
    unsigned long LastTime = 0;

    double speedf = 0;
    double speed_enc = 0;
    double speed = 0;
    double accel = 0;
    double accelf = 0;
    double lastSpeedForAccel = 0;
    bool hasSpeedForAccel = false;
    double speedError = 0;
    double speedErrorIntegral = 0;
    double kpTerm = 0;
    double kiTerm = 0;
    double kdTerm = 0;

    int8_t deadvoltage = 0;
    float integralmax = 0;
    double T_speed = 0;
    double Interval = 0.02;
    double pwm = 0;
};

#endif
