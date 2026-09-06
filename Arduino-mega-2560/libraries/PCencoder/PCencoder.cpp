#include "PCencoder.h"
#include <Arduino.h>
#include <avr/interrupt.h>



// 静态成员定义
volatile uint32_t PCencoder::timer3Overflow = 0;
volatile uint8_t PCencoder::last_[3] = {0, 0, 0};
PCencoder::Slot PCencoder::slots_[3][8];

// 四相编码转移表  
const int8_t PCencoder::QEncoder::qtab_[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* -------------------- 快速辅助函数 -------------------- */
// 内联函数，提高速度
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

/* -------------------- PCInt 实现 -------------------- */
// 内联辅助函数
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
// 在指定组中查找空闲槽位
int PCencoder::PCInt::findFreeSlot(uint8_t g) {
    if (g >= 3) return -1;

    for (uint8_t i = 0; i < 8; i++) {
        if (!slots_[g][i].inUse) {
            return i;
        }
    }
    return -1; // 没有空闲槽位
}
int PCencoder::PCInt::findEncoder(uint8_t g, QEncoder *encoder) {
    if (g >= 3) return -1;

    for (uint8_t i = 0; i < 8; i++) {
        if (slots_[g][i].encoderPtr == encoder) {
            return i;
        }
    }
    return -1;
}
/***********************not PCint return false（支持不同组）********************/
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

    // 从所有槽中移除该引脚对应的掩码，槽若无掩码则回收
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

    // 更新 PCMSK，若该组没有任何监控位则禁用 PCICR 对应位
    *mskReg(g) &= ~bit;
    if (*mskReg(g) == 0) {
        PCICR &= ~pcicrMask(g);
    }

    // 可选：更新 last_ 快照，保证下一次中断计算正确
    last_[g] = *pinReg(g);
}
/***********************模板元编程，在编译期完成所有"循环"和"递归"********************/
// 递归终止
template<uint8_t G>
inline void PCencoder::PCInt::handleSlotsRecursive(uint8_t changed) {}

// 递归展开
template<uint8_t G, uint8_t First, uint8_t... Rest>
inline void PCencoder::PCInt::handleSlotsRecursive(uint8_t changed) {
    handleSlot<G, First>(changed);
    handleSlotsRecursive<G, Rest...>(changed);
}
// 中断计数
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

/***********************参数包，ISR触发中断改值，计数********************/
// 
template<uint8_t G, uint8_t... Sequence>
inline void PCencoder::PCInt::handleGroup() 
{
    uint8_t now = *pinReg(G);
    uint8_t changed = (now ^ last_[G]) & *mskReg(G);
    last_[G] = now;

    // Sequence 为空时，不计数；非空时才递归处理
    if constexpr (sizeof...(Sequence) > 0) {
        handleSlotsRecursive<G, Sequence...>(changed);
    }
}


/***********************编码器的A/B相在不同一组PCInt则用这个*******************
    // 使用位操作而不是循环，提高速度
    if (changed & 0x01) slots_[g][0].encoderPtr->handleChange();
    if (changed & 0x02) slots_[g][1].encoderPtr->handleChange();
    if (changed & 0x04) slots_[g][2].encoderPtr->handleChange();
    if (changed & 0x08) slots_[g][3].encoderPtr->handleChange();
    if (changed & 0x10) slots_[g][4].encoderPtr->handleChange();
    if (changed & 0x20) slots_[g][5].encoderPtr->handleChange();
    if (changed & 0x40) slots_[g][6].encoderPtr->handleChange();
    if (changed & 0x80) slots_[g][7].encoderPtr->handleChange();*/

/* -------------------- QEncoder 实现 -------------------- */
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
    //Serial.print(ga);Serial.println(gb);
    //Serial.print(ia);Serial.println(ib);
    int n = PCInt::findFreeSlot(ga); int m= PCInt::findFreeSlot(gb);
    if (n < 0 || m < 0) return false;      
    if (ga == gb) {                                                      
      slots_[ga][n].inUse = true;                                        // findFreeSlot也需要优化b
      slots_[ga][n].encoderPtr = this;                                   //   可以采取hash表+链表的方式存储(没有预编译优势)
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
    //delay(5000);
    //Serial.println("Encoder attached at group ");

  //slots_用来存编码器，另一个用来存编码器两个不同组的引脚i,j  和  s[i][j]
  /***********************编码器的A/B相在不同一组PCInt则用这个*******************
  //int ia = pinBitIndex(a_), ib = pinBitIndex(b_);
  //slots_[ga][ia].encoderPtr = this;
  //slots_[gb][ib].encoderPtr = this;

  // 快速读取初始状态
  //portA = *portInputRegister(digitalPinToPort(a_));
  //portB = *portInputRegister(digitalPinToPort(b_));
  //maskA = digitalPinToBitMask(a_);
  //maskB = digitalPinToBitMask(b_);
  //prev_ = ((portB & maskB) ? 2 : 0) | ((portA & maskA) ? 1 : 0);*/
  
  return true;
}

//***********************编码器的A/B相在不同一组PCInt则用这个*******************/       
void PCencoder::QEncoder::handleChange() {
    
    uint8_t now = ((*pinRegA_ & maskA) ? 2 : 0) | ((*pinRegB_ & maskB) ? 1 : 0);
    uint8_t idx = (prev_ << 2) | now;
    
    // 从Flash中读取查找表
    cnt_ += qtab_[idx & 0x0F];
    prev_ = now;
    //Serial.println(now);
   //uint8_t curr = (digitalRead(b_) << 1) | digitalRead(a_);
   //uint8_t idx = (prev_ << 2) | curr;
   //cnt_ += qtab_[idx];
   //prev_ = curr;
   //Serial.println(curr);
}

//***********************每个编码器的A/B相在同一组PCInt则用这个*******************/
void PCencoder::QEncoder::_handleChange(uint8_t idx) {
    cnt_ += qtab_[prev_ << 2 | idx & 0x0F];
    prev_ = idx;
    if (sampleSpeed) {
      timerLastCount = timerThisCount;
      timerThisCount = readTimer3Counts();
      encoderLastCount = encoderThisCount;
      encoderThisCount=cnt_;
      //Serial.print("timerThisCount: ");
      //Serial.print((long)timerThisCount);
      //Serial.print("encoderThisCount: ");
      //Serial.println((long)encoderThisCount);
      sampled = 1;
      sampleSpeed = 0;
    }
}



long PCencoder::QEncoder::read() const {
    return cnt_;
}

void PCencoder::QEncoder::write(long v) {
    cnt_ = v;
}

/* -------------------- DCmotor 实现 -------------------- */
PCencoder::DCmotor::DCmotor(uint8_t IN1, uint8_t IN2, uint8_t PWM) 
    : in1(IN1), in2(IN2), pwm(PWM) 
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
    isAttached = 1;
}
PCencoder::DCmotor::~DCmotor(){};

void PCencoder::DCmotor::go(unsigned int value){

  if(!isAttached)return;
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  if(value>255)value = 255;
  analogWrite(pwm,value);
}
void PCencoder::DCmotor::back(unsigned int value){
  if(!isAttached)return;
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  if(value>255)value = 255;
  analogWrite(pwm,value);
}
void PCencoder::DCmotor::down(){
if(!isAttached)return;
 digitalWrite(in1,LOW);
 digitalWrite(in2,LOW);
}
void PCencoder::DCmotor::brake(){
if(!isAttached)return;
 digitalWrite(in1,LOW);
 digitalWrite(in2,LOW);
}
//********************************PCencoder 实现********************************/
PCencoder::PCencoder() : m_encoder(nullptr), m_motor(nullptr) {}
PCencoder::~PCencoder() {delete m_encoder;delete m_motor;}
bool PCencoder::attach(uint8_t in1, uint8_t in2, uint8_t pwm, 
                      uint8_t en1, uint8_t en2, double k) {

    if (!attachmotor(in1, in2, pwm)) {
        return false;
    }
    //delay(2000);
    if (!attachEncoder(en1, en2)) {
        // 如果编码器初始化失败，回滚已创建的电机
        if (m_motor) { delete m_motor; m_motor = nullptr; }
        
        //Serial.println("Encoder attach failed!");
        return false;
    }
    /*
    else {
      Serial.println("Encoder attached successfully!");
    }
    */
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
void PCencoder::setspeed(double value){
  T_speed=value;
}
void PCencoder::setpid(double kp1,double ki1,double kd1,double PULSE_TO_DISTANCE1){
  kp=kp1;
  ki=ki1;
  kd=kd1;
  PULSE_TO_DISTANCE = PULSE_TO_DISTANCE1;   // 30cm 1281
}
void PCencoder::updateSpeedInloop() {

  if (m_encoder->sampled == 1) {
    long long encodercount = m_encoder->encoderThisCount - m_encoder->encoderLastCount;
    //Serial.print("encodercount: ");
    //Serial.println((long)encodercount);
    double timerInterval = (double)(m_encoder->timerThisCount - m_encoder->timerLastCount)/ 15625.0;
    if (timerInterval < 0.01) timerInterval = Interval;   // 防止除0
    speed = (double)encodercount * PULSE_TO_DISTANCE / timerInterval;  // 30cm   1281
    m_encoder->sampled = 0;
  }

}

void PCencoder::updateSpeed(){
  if (!m_encoder) return;
  if (m_encoder->sampleSpeed == 0) {
    long long encodercount = m_encoder->encoderThisCount - m_encoder->encoderLastCount;
    double timerInterval = (double)(m_encoder->timerThisCount - m_encoder->timerLastCount)/ 15625.0;
    if (timerInterval < 0.01) timerInterval = Interval;   // 防止除0
    speed = (double)encodercount * 0.019 / timerInterval;  //  
    m_encoder->sampleSpeed = 1;
  } else {
    speed = 0;

  }
  
}
void PCencoder::updateSpeed0(){
  if (!m_encoder) return;
  if (m_encoder->sampleSpeed == 1) {

        uint32_t now = readTimer3Counts();
        uint32_t dt  = now - m_encoder->timerThisCount;

        // 超过 50ms 没有任何编码器变化 → 真正认为停转
        if (dt > SPEED_ZERO_TIMEOUT_COUNTS) {
            speed = 0;
        }
        // 否则：什么都不做（保持 speed / 让滤波器自己慢慢掉）
    } 
    else {
        // 没进入采样窗口，只是标记一下
        m_encoder->sampleSpeed = 1;
    }
}
void PCencoder::updatePWM_NEW() {        // 放在主循环中，因此不加微分项
    // ---- time ----
    CurrentTime = micros();
    long long delta_Interval = CurrentTime - LastTime;
    LastTime = CurrentTime;
    double dt = (double)delta_Interval / 1000000.0;
    if (dt <= 0) dt = 1e-6;

    // ---- speed filter ----
    if (speed > 0.5 || speed < -0.5) {          // 测得较准区间
        speedf = 0.7 * speedf + 0.3 * speed;
    } else {                                     // 低速更稳
        speedf = 0.9 * speedf + 0.1 * speed;
    }

    // --- 误差 ---
    speedError = T_speed - speedf;

    // --- 目标接近0才允许刹车 ---
    const double T_STOP = 0.5;  // cm/s，目标<0.5才认为要停
    if (fabs(T_speed) < T_STOP) {
      speedErrorIntegral = 0;
      m_motor->brake();
      return;
    }

    // --- 滞回阈值：按目标速度比例 ---
    double absT = fabs(T_speed);
    double E_START = max(1.5, 0.15 * absT); // cm/s
    double E_STOP  = max(0.7, 0.07 * absT); // cm/s

    static bool running = false;
    static double pwm_hold = 0;

    // --- 启停状态机（滞回）---
    if (!running) {
      if (fabs(speedError) > E_START) {
        running = true;
      } else {
        // 不需要启动控制：保持上一次PWM（或 down），但不积分
        pwm = pwm_hold;
        // 这里不要 brake！否则你会“该走却停”
      }
    } else {
      if (fabs(speedError) < E_STOP) {
        running = false;
        // 小误差区：冻结积分，保持PWM，不刹车
        pwm = pwm_hold;
      }
}

// --- 只有 running 时才更新积分和 PI ---
if (running) {
  speedErrorIntegral += speedError * dt;
  speedErrorIntegral = constrain(speedErrorIntegral, INTEGRAL_MIN, INTEGRAL_MAX);
  pwm = kp * speedError + ki * speedErrorIntegral;
  pwm_hold = pwm;  // 记录用于小误差保持
} else {
  // 可选：让积分慢慢泄放一点，防止长期偏置
  speedErrorIntegral *= 0.98;
}

// --- 输出限幅 ---
pwm = constrain(pwm, -255, 255);

// --- 驱动 ---
if (pwm >= 0) m_motor->go((unsigned int)(pwm + DEAD_VOLTAGE + 0.5));
else          m_motor->back((unsigned int)(-pwm + DEAD_VOLTAGE + 0.5));

}

void PCencoder::updatePWM(){        //   放在主循环中的updatePWM   因此不能有微分项，但积分项会更准
    //pid
    CurrentTime =  micros();
    long long delta_Interval = CurrentTime - LastTime;
    LastTime = CurrentTime;
    if (speed >0.3 || speed <-0.3) {      //  测不准的最小速度
      speedf = 0.7 * speedf + 0.3 * speed;   // 一阶低通滤波
    } else {
      speedf = 0.9 * speedf + 0.1 * speed;   // 一阶低通滤波
    }
    // --- 误差 ---
    speedError = T_speed - speedf;
     double speedError1 = T_speed - speed;

    if (speedError >= 0.0 || speedError < -0.0) {  //  冻结积分纯没用
      speedErrorIntegral += speedError*delta_Interval/1000000.0;
      speedErrorIntegral = constrain(speedErrorIntegral, INTEGRAL_MIN, INTEGRAL_MAX);
    } else {
      speedError = 0;
    }
    pwm = kp * speedError+ki*speedErrorIntegral ;  

    //Serial.print("pwm: ");
    //Serial.println(pwm);
    
  //motor
  //if (pwm>255 - DEAD_VOLTAGE)pwm=255 - DEAD_VOLTAGE;
  //if (pwm<-255 + DEAD_VOLTAGE)pwm=-255 + DEAD_VOLTAGE;

  //Serial
  //Serial.print("encoderCount: ");
  //Serial.println((long)encoderCount);
  
  //Serial.print("speed: ");
  //Serial.println(speed);
  //Serial.print("pwm: ");
  //Serial.println(pwm);
  
  if (T_speed<0.01 && T_speed>-0.01){    //目标速度为0时，pwm很小无效控制时直接刹车
    if (pwm < 10 && pwm >-10) {
      m_motor->brake();
      return;
    }
  }
  
  if (pwm >= 0){
    m_motor->go(pwm + DEAD_VOLTAGE);
  }else {
    m_motor->back(-pwm + DEAD_VOLTAGE);
  }
}
void PCencoder::update(){
  
  updateSpeed();

  //pid
  speedError = T_speed - speed;
  speedErrorDerivative = (speedError-lastspeedError)/Interval;
  speedErrorIntegral += speedError*Interval;
  lastspeedError = speedError;
    if (speedErrorIntegral > INTEGRAL_MAX) {
    speedErrorIntegral = INTEGRAL_MAX;
} else if (speedErrorIntegral < INTEGRAL_MIN) {
    speedErrorIntegral = INTEGRAL_MIN;
}
  pwm = kp * speedError
          +ki*speedErrorIntegral + kd*speedErrorDerivative;

  //motor
  //if (pwm>255 - DEAD_VOLTAGE)pwm=255 - DEAD_VOLTAGE;
  //if (pwm<-255 + DEAD_VOLTAGE)pwm=-255 + DEAD_VOLTAGE;

  //Serial
  //Serial.print("encoderCount: ");
  //Serial.println((long)encoderCount);
  
  //Serial.print("speed: ");
  //Serial.println(speed);
  //Serial.print("pwm: ");
  //Serial.println(pwm);
  
  if (T_speed<0.01 && T_speed>-0.01){    //目标速度为0时，pwm很小无效控制时直接刹车
    if (pwm < 10 && pwm >-10) {
      m_motor->brake();
      return;
    }
  }
  

  //if (pwm < 1 && pwm >-1){     //当pwm足够小时，防止电机pwm的来回震荡
  //  m_motor->go(0);
  //  return;
  //}
  if (pwm >= 0){

    m_motor->go(pwm + DEAD_VOLTAGE);
    //Serial.println("Ddddddd");
  }else {
    m_motor->back(-pwm + DEAD_VOLTAGE);
  }
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
void PCencoder::brake() {   //   brake 之后不会更新速度和pwm    需用on()恢复
  _brake = true;
  T_speed = 0;
  m_motor->brake();
}
void PCencoder::on() {
  _brake = false;
}
bool PCencoder::braked() const {
  return _brake;
}
void PCencoder::_setTimer3()
{
    cli();  // 关闭中断，防止配置过程中触发错误

    // 1. 关闭 Timer3 设置（确保寄存器不会残留）
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    // 2. 设置普通计数模式（Normal mode）
    // （WGM33:WGM30 = 0 0 0 0）
    TCCR3A = 0x00;
    TCCR3B = 0x00;

    // 3. 设置 1024 分频
    // CS12 = 1, CS11 = 0, CS10 = 1 → 分频 = 1024
    TCCR3B |= (1 << CS12) | (0 << CS11) | (1 << CS10);

    // 4. 允许溢出中断
    TIMSK3 |= (1 << TOIE3);

    sei();  // 开启全局中断
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
    cli(); // 进入原子区

    ovf = timer3Overflow;
    timer = TCNT3;

    SREG = oldSREG; // 恢复中断状态

    return ovf + timer;
}


// ISR 每组独立绑定（纯静态调用）
ISR(PCINT0_vect) { PCencoder::PCInt::handleGroup<0 PCENCODER_GROUP0_SLOTS>(); }
ISR(PCINT1_vect) { PCencoder::PCInt::handleGroup<1 PCENCODER_GROUP1_SLOTS>(); }
ISR(PCINT2_vect) { PCencoder::PCInt::handleGroup<2 PCENCODER_GROUP2_SLOTS>(); }
ISR(TIMER3_OVF_vect){PCencoder::addTimerOverflow(65536UL);}  // 正确溢出补偿}