#ifndef __MT_Encoder_H
#define __MT_Encoder_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

class QEncoder
{

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
