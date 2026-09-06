#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define pc_useTimer5
#define pc_useTimer1
#define pc_useTimer3
#define pc_useTimer4

// ---- Timer1 ----
#define USE_TIMER1_A   false
#define USE_TIMER1_B   false
#define USE_TIMER1_C   false

// ---- Timer3 ----
#define USE_TIMER3_A   false
#define USE_TIMER3_B   false
#define USE_TIMER3_C   false

// ---- Timer4 ----
#define USE_TIMER4_A   false
#define USE_TIMER4_B   false
#define USE_TIMER4_C   true

// ---- Timer5 ----
#define USE_TIMER5_A   true
#define USE_TIMER5_B   true
#define USE_TIMER5_C   true
typedef enum {_pcencoder_timer5,  // 添加前缀_pcencoder_Nbr_16timers  // 添加前缀} pcencoder_timer16_Sequence_t;  // 修改类型名称
              _pcencoder_Nbr_16timers  // 添加前缀
} pcencoder_timer16_Sequence_t;  // 修改类型名称

#endif // 结束条件编译