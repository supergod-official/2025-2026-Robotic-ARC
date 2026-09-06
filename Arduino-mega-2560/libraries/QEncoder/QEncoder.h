#ifndef QENCODER_H
#define QENCODER_H

#include <Arduino.h>
#include "PCInt.h"  // 引入 PCInt.h，用于中断处理

class QEncoder {
public:
  QEncoder(uint8_t pinA, uint8_t pinB);
  
  bool begin(bool pullup = true);  // 初始化
  long read() const;               // 获取当前计数
  void write(long v);              // 设置计数值

private:
  static void thunk(uint8_t pin);  // 静态回调函数
  
  uint8_t a_, b_;
  volatile long cnt_ = 0;           // 计数
  volatile uint8_t prev_ = 0;       // 上次的电平状态
  static QEncoder* map_[80];        // 将引脚号映射到 QEncoder 对象
  
  static const int8_t qtab_[16];    // 四倍频查找表
};

#endif
