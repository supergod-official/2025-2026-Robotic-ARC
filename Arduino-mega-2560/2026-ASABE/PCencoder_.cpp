#include "PCencoder_.h"
#include <Arduino.h>
#include <avr/interrupt.h>

volatile uint32_t PCencoder::timer3Overflow = 0;
volatile uint8_t PCencoder::last_[3] = {0, 0, 0};
PCencoder::Slot PCencoder::slots_[3][8];

const int8_t PCencoder::QEncoder::qtab_[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline int pinToGroup(uint8_t pin) {
    volatile uint8_t *pcmsk = digitalPinToPCMSK(pin);
    if (pcmsk == &PCMSK0) return 0;
    if (pcmsk == &PCMSK1) return 1;
    if (pcmsk == &PCMSK2) return 2;
    return -1;
}

static inline int pinBitIndex(uint8_t pin) {
    return __builtin_ctz(digitalPinToBitMask(pin));
}

inline volatile uint8_t* PCencoder::PCInt::pinReg(uint8_t g) {
    static volatile uint8_t* regs[] = {&PINB, &PINJ, &PINK};
    return regs[g];
}

inline volatile uint8_t* PCencoder::PCInt::mskReg(uint8_t g) {
    static volatile uint8_t* regs[] = {&PCMSK0, &PCMSK1, &PCMSK2};
    return regs[g];
}

inline uint8_t PCencoder::PCInt::pcicrMask(uint8_t g) {
    return _BV(PCIE0 + g);
}

int PCencoder::PCInt::findFreeSlot(uint8_t g) {
    if (g >= 3) return -1;

    for (uint8_t i = 0; i < 8; i++) {
        if (!slots_[g][i].inUse) {
            return i;
        }
    }
    return -1;
}

bool PCencoder::PCInt::attach(uint8_t pin1, uint8_t pin2, Mode mode, bool pullup) {
    int g1 = pinToGroup(pin1);
    int g2 = pinToGroup(pin2);
    if (g1 < 0 || g2 < 0 ) return false;

    int idx1 = pinBitIndex(pin1);
    int idx2 = pinBitIndex(pin2);
    if (idx1 < 0 || idx2 < 0) return false;

    pinMode(pin1, pullup ? INPUT_PULLUP : INPUT);
    pinMode(pin2, pullup ? INPUT_PULLUP : INPUT);

    uint8_t bit1 = digitalPinToBitMask(pin1);
    uint8_t bit2 = digitalPinToBitMask(pin2);
    *mskReg(g1) |= bit1 | bit2;
    PCICR |= pcicrMask(g1);

    last_[g1] = *pinReg(g1);
    return true;
}

void PCencoder::PCInt::detach(uint8_t pin) {
    int g = pinToGroup(pin);
    if (g < 0) return;

    uint8_t bit = digitalPinToBitMask(pin);

    for (int i = 0; i < 8; ++i) {
        Slot &s = slots_[g][i];
        if (!s.inUse) continue;
        if (s.mask & bit) {
            s.mask &= ~bit;
            if (s.mask == 0) {
                s.inUse = false;
                s.encoderPtr = nullptr;
            }
        }
    }

    *mskReg(g) &= ~bit;
    if (*mskReg(g) == 0) {
        PCICR &= ~pcicrMask(g);
    }

    last_[g] = *pinReg(g);
}

template<uint8_t G>
inline void PCencoder::PCInt::handleSlotsRecursive(uint8_t changed) {}

template<uint8_t G, uint8_t First, uint8_t... Rest>
inline void PCencoder::PCInt::handleSlotsRecursive(uint8_t changed) {
    handleSlot<G, First>(changed);
    handleSlotsRecursive<G, Rest...>(changed);
}

template<uint8_t G, uint8_t I>
inline void PCencoder::PCInt::handleSlot(uint8_t changed) 
{
    const Slot &s = slots_[G][I];
    if (changed & s.mask) {
        uint8_t val = (((last_[s.groupA] & s.maskA) != 0) << 1) |
                      ((last_[s.groupB] & s.maskB) != 0);
        s.encoderPtr->_handleChange(val);
    }
}

template<uint8_t G, uint8_t... Sequence>
inline void PCencoder::PCInt::handleGroup() 
{
    uint8_t now = *pinReg(G);
    uint8_t changed = (now ^ last_[G]) & *mskReg(G);
    last_[G] = now;

    if constexpr (sizeof...(Sequence) > 0) {
        handleSlotsRecursive<G, Sequence...>(changed);
    }
}

PCencoder::QEncoder::QEncoder(uint8_t pinA, uint8_t pinB) 
    : a_(pinA), b_(pinB), cnt_(0), prev_(0) 
{
    pinRegA_ = portInputRegister(digitalPinToPort(a_));
    pinRegB_ = portInputRegister(digitalPinToPort(b_));
    maskA = digitalPinToBitMask(a_);
    maskB = digitalPinToBitMask(b_);
}
PCencoder::QEncoder::~QEncoder(){};
bool PCencoder::QEncoder::begin(bool pullup) {
    if (!PCInt::attach(a_, b_, _CHANGE, pullup)) return false;
    int ga = pinToGroup(a_);int gb = pinToGroup(b_);
    int ia = pinBitIndex(a_), ib = pinBitIndex(b_);
    int n = PCInt::findFreeSlot(ga); int m= PCInt::findFreeSlot(gb);
    if (n < 0 || m < 0) return false;      
    if (ga == gb) {                                                      
      slots_[ga][n].inUse = true;                                        
      slots_[ga][n].encoderPtr = this;                              
      slots_[ga][n].mask = 1 << ia | 1 << ib;
      slots_[ga][n].groupA = ga;
      slots_[ga][n].groupB = gb;
      slots_[ga][n].maskA = 1 << ia;
      slots_[ga][n].maskB = 1 << ib;
    }else{
      slots_[ga][n].inUse = true;
      slots_[ga][n].encoderPtr = this;
      slots_[ga][n].mask = 1 << ia;
      slots_[ga][n].groupA = ga;
      slots_[ga][n].groupB = gb;
      slots_[ga][n].maskA = 1 << ia;
      slots_[ga][n].maskB = 1 << ib;
      slots_[gb][m].inUse = true;
      slots_[gb][m].encoderPtr = this;
      slots_[gb][m].mask = 1 << ib;
      slots_[gb][m].groupA = ga;
      slots_[gb][m].groupB = gb;
      slots_[gb][m].maskA = 1 << ia;
      slots_[gb][m].maskB = 1 << ib;
    }
  
  prev_ = (((*pinRegA_ & maskA) != 0) << 1) |
          ((*pinRegB_ & maskB) != 0);
  return true;
}

void PCencoder::QEncoder::_handleChange(uint8_t idx) {
    uint8_t state = idx & 0x03;
    cnt_ += qtab_[((prev_ & 0x03) << 2) | state];
    prev_ = state;
    if (sampleSpeed) {
      timerLastCount = timerThisCount;
      timerThisCount = readTimer3Counts();
      encoderLastCount = encoderThisCount;
      encoderThisCount=cnt_;
      sampled = 1;
      sampleSpeed = 0;
    }
}

long PCencoder::QEncoder::read() const {
    uint8_t oldSREG = SREG;
    cli();
    long value = cnt_;
    SREG = oldSREG;
    return value;
}

void PCencoder::QEncoder::write(long v) {
    uint8_t oldSREG = SREG;
    cli();
    cnt_ = v;
    SREG = oldSREG;
}

PCencoder::DCmotor::DCmotor(uint8_t IN1, uint8_t IN2, uint8_t PWM) 
    : in1(IN1), in2(IN2), pwm(PWM) 
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
    isAttached = 1;
}
PCencoder::DCmotor::~DCmotor(){};

void PCencoder::DCmotor::go(unsigned int value){

  if(!isAttached)return;
  if (currentDirection != DIR_GO) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    currentDirection = DIR_GO;
  }
  if(value>255)value = 255;
  analogWrite(pwm,value);
}
void PCencoder::DCmotor::back(unsigned int value){
  if(!isAttached)return;
  if (currentDirection != DIR_BACK) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    currentDirection = DIR_BACK;
  }
  if(value>255)value = 255;
  analogWrite(pwm,value);
}
void PCencoder::DCmotor::down(){
  brake();
}
void PCencoder::DCmotor::brake(){
if(!isAttached)return;
 if (currentDirection != DIR_BRAKE) {
   digitalWrite(in1,LOW);
   digitalWrite(in2,LOW);
   currentDirection = DIR_BRAKE;
 } else {
   analogWrite(pwm, 0);
 }
}

PCencoder::PCencoder() : m_encoder(nullptr), m_motor(nullptr) {}
PCencoder::~PCencoder() {delete m_encoder;delete m_motor;}
bool PCencoder::attach(uint8_t in1, uint8_t in2, uint8_t pwm, 
                      uint8_t en1, uint8_t en2, double k) {

    if (!attachmotor(in1, in2, pwm)) {
        return false;
    }
    if (!attachEncoder(en1, en2)) {
 
        if (m_motor) { delete m_motor; m_motor = nullptr; }
        
        return false;
    }
    Interval = k;
    return true;
}
bool PCencoder::attachmotor(uint8_t in1, uint8_t in2, uint8_t pwm) {
    delete m_motor;
    m_motor = new DCmotor(in1, in2, pwm);
    return (m_motor != nullptr);
}
bool PCencoder::attachEncoder(uint8_t en1, uint8_t en2) {
    delete m_encoder;
    m_encoder = new QEncoder(en1, en2);
    if (!m_encoder->begin()) {
        delete m_encoder;
        m_encoder = nullptr;
        return false;
    }
    return true;
}
void PCencoder::setpid(double kp1,double ki1,double kd1,double PULSE_TO_DISTANCE1,double deadvoltage1,double integralmax1){
  kp=kp1;
  ki=ki1;
  kd=kd1;
  PULSE_TO_DISTANCE = PULSE_TO_DISTANCE1;
  deadvoltage = deadvoltage1;
  integralmax = integralmax1;
}
void PCencoder::setPwm(int16_t pwm){
  this->pwm = pwm;
  if (pwm >= 0){
    m_motor->go(pwm + deadvoltage);
  }else {
    m_motor->back(-pwm + deadvoltage);
  }
}
void PCencoder::setspeed(double value){
  const double dtMin = 0.0002;
  const double dtMax = 0.005;

  // 1. 滤波 目标速度T_speed
  T_speed=0.1 * value + 0.9 * T_speed;
  // 2. 计算两次采样的时间间隔
  CurrentTime =  micros();
  double dt = Interval;
  if (LastTime != 0) {
    dt = (double)(CurrentTime - LastTime) / 1000000.0;
    dt = constrain(dt, dtMin, dtMax);
  }
  LastTime = CurrentTime;
  speedError = T_speed - speedf;
  // 3. 计算pid积分项
  speedErrorIntegral += speedError * dt;
  speedErrorIntegral = constrain(speedErrorIntegral, -integralmax, integralmax);
  kpTerm = kp * speedError;
  kiTerm = ki * speedErrorIntegral;
  kdTerm = kd * accelf;
  pwm = kpTerm + kiTerm + kdTerm;
  // 4. 目标速度小于阈值时，直接刹车
  if (T_speed<0.01 && T_speed>-0.01){   
    if (pwm < 10 && pwm >-10) {
      m_motor->brake();
      return;}}
  //  5. 设置pwm
  if (pwm >= 0){
    m_motor->go(pwm + deadvoltage);
  }else {
    m_motor->back(-pwm + deadvoltage);
  }
}
void PCencoder::updateMotionInloop() {
  if (!m_encoder) return;
  if (m_encoder->sampled == 1) {
    long long encodercount = m_encoder->encoderThisCount - m_encoder->encoderLastCount;
    double timerInterval = (double)(m_encoder->timerThisCount - m_encoder->timerLastCount)/ 15625.0;
    if (timerInterval < 0.005) timerInterval = Interval;  
    speed_enc = (double)encodercount  / timerInterval;
    speed = speed_enc * PULSE_TO_DISTANCE;  
    if (hasSpeedForAccel && timerInterval > 0.0) {
      accel = (speed - lastSpeedForAccel) / timerInterval;
    } else {
      accel = 0;
      hasSpeedForAccel = true;
    }
    lastSpeedForAccel = speed;
    m_encoder->sampled = 0;
  }
}
void PCencoder::filterMotionInloop() {
  const double speedFilterAlpha = 0.1;
  const double accelFilterAlpha = 0.05;

  speedf = (1.0 - speedFilterAlpha) * speedf + speedFilterAlpha * speed;
  accelf = (1.0 - accelFilterAlpha) * accelf + accelFilterAlpha * accel;
}
void PCencoder::requestMotionSample(){
  if (!m_encoder) return;
  if (m_encoder->sampleSpeed == 1) {

        uint32_t now = readTimer3Counts();
        uint32_t dt  = now - m_encoder->timerThisCount;

        if (dt > SPEED_ZERO_TIMEOUT_COUNTS) {
            speed = 0;
            accel = 0;
            lastSpeedForAccel = 0;
            hasSpeedForAccel = false;
        }
    } 
    else {
        m_encoder->sampleSpeed = 1;
    }
}

void PCencoder::clearIntegral(){
  speedErrorIntegral=0;
  LastTime = 0;
}
void PCencoder::sampleSpeed(){
  if(m_encoder)
    m_encoder->sampleSpeed=1;
}
long long PCencoder::readencoder() const{
   return m_encoder ? m_encoder->read() : 0;
}
double PCencoder::readpwm() const{
   return pwm;
}
double PCencoder::readspeed() const{
  return speed;
}
double PCencoder::readspeedf() const{
  return speedf;
}
double PCencoder::readaccel() const{
  return accel;
}
double PCencoder::readaccelf() const{
  return accelf;
}
double PCencoder::readKpTerm() const{
  return kpTerm;
}
double PCencoder::readKiTerm() const{
  return kiTerm;
}
double PCencoder::readKdTerm() const{
  return kdTerm;
}
void PCencoder::brake() {
  m_motor->brake();
}

void PCencoder::_setTimer3()
{
    cli();

    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    TCCR3A = 0x00;
    TCCR3B = 0x00;

    TCCR3B |= (1 << CS12) | (0 << CS11) | (1 << CS10);

    TIMSK3 |= (1 << TOIE3);

    sei(); 
}
void PCencoder::addTimerOverflow(uint32_t inc)
{
    timer3Overflow += inc;
}

uint32_t PCencoder::readTimer3Counts()
{
    uint32_t ovf;
    uint16_t timer;

    uint8_t oldSREG = SREG;
    cli(); 

    ovf = timer3Overflow;
    timer = TCNT3;

    SREG = oldSREG; 

    return ovf + timer;
}

ISR(PCINT0_vect) { PCencoder::PCInt::handleGroup<0 PCENCODER_GROUP0_SLOTS>(); }
ISR(PCINT1_vect) { PCencoder::PCInt::handleGroup<1 PCENCODER_GROUP1_SLOTS>(); }
ISR(PCINT2_vect) { PCencoder::PCInt::handleGroup<2 PCENCODER_GROUP2_SLOTS>(); }
ISR(TIMER3_OVF_vect){PCencoder::addTimerOverflow(65536UL);}
