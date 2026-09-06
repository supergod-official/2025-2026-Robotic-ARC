#ifndef ultrasonic_h
#define ultrasonic_h


#define TRIGGER_PINf  A2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PINf     A3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCEf 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINl  A9  
#define ECHO_PINl     A8 
#define MAX_DISTANCEl 15

#define TRIGGER_PINr  A4  
#define ECHO_PINr     A5  
#define MAX_DISTANCEr 15

unsigned long US_prMicros = 0;

#define US_Interval   20000                         // 每 29 毫秒一次    hc_sr04最大检测距离500cm往返时间为29ms
#define US_TIME_TO_DISTANCE_CONVERSION  0.01717  // 超声波时间到距离的转换常数
 


#endif 