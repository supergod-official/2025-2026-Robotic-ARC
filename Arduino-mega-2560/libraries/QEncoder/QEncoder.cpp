#include "QEncoder.h"

// 静态成员初始化
QEncoder* QEncoder::map_[80] = { nullptr };  // 存储每个引脚对应的 QEncoder 对象
const int8_t QEncoder::qtab_[16] = {
  0, -1, +1, 0,  +1, 0, 0, -1,
  -1, 0, 0, +1, 0, +1, -1, 0
};

// 构造函数
QEncoder::QEncoder(uint8_t pinA, uint8_t pinB) : a_(pinA), b_(pinB) {}

// 初始化编码器
bool QEncoder::begin(bool pullup) {
  map_[a_] = this;
  map_[b_] = this;

  pinMode(a_, pullup ? INPUT_PULLUP : INPUT);
  pinMode(b_, pullup ? INPUT_PULLUP : INPUT);

  // 读取初始状态
  prev_ = ((digitalRead(b_) ? 1 : 0) << 1) | (digitalRead(a_) ? 1 : 0);

  // 注册中断
  bool ok1 = PCInt::attach(a_, &QEncoder::thunk, PCInt::_CHANGE, pullup);
  bool ok2 = PCInt::attach(b_, &QEncoder::thunk, PCInt::_CHANGE, pullup);
  //Serial.println("ok1: " + String(ok1) + ", ok2: " + String(ok2));
  return ok1 && ok2;
}

// 读取当前计数
long QEncoder::read() const {
  noInterrupts();  // 禁止中断
  long v = cnt_;
  interrupts();    // 恢复中断
  return v;
}

// 设置计数
void QEncoder::write(long v) {
  noInterrupts();
  cnt_ = v;
  interrupts();
}

// 回调函数：处理编码器状态变化
/*
void QEncoder::thunk(uint8_t pin, ) {
  QEncoder* self = map_[pin];
  if (!self) return;  // 如果找不到对应的 QEncoder 对象，返回

  uint8_t a = digitalRead(self->a_) ? 1 : 0;
  uint8_t b = digitalRead(self->b_) ? 1 : 0;
  uint8_t curr = (b << 1) | a;
  uint8_t idx = (self->prev_ << 2) | curr;
  
  self->cnt_ += qtab_[idx];
  self->prev_ = curr;
}
*/
void QEncoder::thunk(uint8_t pin) {
  QEncoder* self = map_[pin];
  if (!self) return;

  uint8_t curr = (digitalRead(self->b_) << 1) | digitalRead(self->a_);
  uint8_t idx = (self->prev_ << 2) | curr;
  self->cnt_ += qtab_[idx];
  self->prev_ = curr;
}


