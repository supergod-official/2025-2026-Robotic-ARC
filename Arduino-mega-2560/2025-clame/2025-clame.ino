#include "PID.h"
#include <MsTimer2.h> 
#include <AccelStepper.h>
#include <Servo.h>


// Motor Connections (constant current, step/direction bipolar motor driver)

const int dirPinq = 6;
const int stepPinq = 22;
const int _enq=23;
const int dirPinp = 9;
const int stepPinp = 24;
const int _enp=25;


const float MAX_SPEED       = 3000.0;       // 最大速度   0.140625s   1800*0.140625=252      640  *   2   = 
const float ACCELERATION    = 15600.0;      // 加速度
const long TARGET_POSITION  = 1280;         // 转90°位置  360/1.8=200   200*32=6400   6400/5=1280  
const float L_first = 18.0f;         // 第一段：2000计数        5cm
const float _L      = 21.5f;        // 之后无限重复：1280计数   20cm
const float L_last  = 21.5f;         // 第一段：2000计数        5cm              200步时暂停    4-1=3   增加3cm
const float p       = 50.5f;         // 计数/单位换算因子
const float v_cmd0  = 80.0f;        // 目标速度（基准）
static bool _init    = false;        // 盲走复位
static bool d_init    = false; 
static bool ison     = false;
static bool d_start  = false;        //播种左边先开始
static bool b_start  = true;
bool _check          =false;
static bool dataReceived= false;
static bool Time_out=false;
float k1=0.5;float k2=0.5;
int _final=1;
float lastcount ;
unsigned long lastEncoderTime = 0;  // 上次读取编码器的时间
float lastEncoderAvg = 0.0;         // 上次编码器的平均值
float v=0.0,suduq,sudup,sudud,sudub;
int stop_count=0,                               step=0; 
int sow = 0;
bool servoReady = false;      // 用于标记舵机是否准备好
unsigned long servoStartTime = 0,T=0,ti=0; // 声明为全局变量
AccelStepper myStepperq(AccelStepper::DRIVER, stepPinq, dirPinq);
AccelStepper myStepperp(AccelStepper::DRIVER, stepPinp, dirPinp);
int posq;
int posp;
const int steps_per_move = 640;  // 每段走640步
static int y = 0;
static int y_sow = 18;
PID qm;
PID pm;
PID dm;
PID bm;
//Servo myservoq;
//Servo myservop;
bool b_left(const float ,  const float ) ;
bool b_right(const float ,  const float ) ;
bool b_up(const float ,  const float ) ;
bool b_dwon(const float ,  const float ) ;
bool bw_up(const float ,  const float);
bool check_sow(),check_stop(),time_out();
void light(unsigned long);
bool updateSowAndSpeed(float , float);
void Sow(float, float, unsigned long);
bool b_walk(const float, const float, const float);
bool b_walk_repeat( const float , const float , const float ,const float , const float );
enum MoveStage { STAGE_LEFT, STAGE_RIGHT, STAGE_UP, STAGE_DOWN, STAGE_DONE };
MoveStage moveStage = STAGE_UP;
enum Direction { DIR_LEFT, DIR_RIGHT, DIR_UP, DIR_DOWN };
bool b_walk_dir(const float , const float , const float , Direction ) ;
void walk_sequence();
bool b_stepper_move();
bool d_stepper_move();
struct Packet {
  //int8_t head1;   // 0x2C
  //int8_t head2;   // 0x12
  int  x;       // 数据1
  int  a;       // 数据2
  int8_t flag;    // 红线标志
  int8_t tail;    // 0x5B
};
Packet pkt;

int x_val ;
float a_val ;
void timerCallback() {
  qm.update();
  pm.update();
  dm.update();
  bm.update();
}



void setup(){
  
  Serial1.begin(115200);
  //Serial.begin(115200);
  //Serial.begin(115200);
  pinMode(_enq,OUTPUT);              
  pinMode(_enp,OUTPUT);
  digitalWrite(_enq,HIGH);           
  digitalWrite(_enp,HIGH);
  myStepperq.setMaxSpeed(MAX_SPEED); 
  myStepperp.setMaxSpeed(MAX_SPEED);
  myStepperq.setAcceleration(ACCELERATION);
  myStepperp.setAcceleration(ACCELERATION);

  //myservop.attach(A8);
  //myservoq.attach(A9);
  
  qm.attach(43,47,45,53,52,0.01);
  qm.setpid(1.4,0,0.5);
  pm.attach(40,42,44,50,51,0.01);
  pm.setpid(1.4,0,0.5);
  dm.attach(29,28,8,11,10,0.01);
  dm.setpid(1.4,0,0.5);
  bm.attach(27,26,7,12,13,0.01);
  bm.setpid(1.4,0,0.5);

  // ...
  MsTimer2::set(10, timerCallback);
  MsTimer2::start();
}

void loop() {
  myStepperp.run();
  myStepperq.run();
  //Serial.println(step);
  //Serial.print("sow");Serial.println(sow);
  //Serial.print("step");Serial.println(step);
  if (Serial1.available() >= 6) {
    
    byte buf[6];
    Serial1.readBytes(buf, 6);   

    // 检查帧头帧尾       buf[0] == 0x2C && buf[1] == 0x12 &&
    if ( buf[5] == 0x5B) {
      memcpy(&pkt, buf, 6);

      dataReceived = true;
      x_val=pkt.x/10.0;
      a_val=pkt.a/10.0;
      _final=pkt.flag/10;
      pkt.flag%=10;

      //Serial.print("x = "); Serial.println(x_val);
      //Serial.print("a = "); Serial.println(a_val);
      //Serial.print("flag = "); Serial.println(pkt.flag);
      if (step%2==1){if (millis()>=T && pkt.flag ==1){step+=1;T = millis() + 1000;}    // 每次step增加的间隔为500ms
      }else {if (millis()>=T && pkt.flag ==1){step+=1;T = millis() + 1200;}}
      if (step==14 && !ison){
        ison = true;
        lastcount = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
        //myservop.write(90);
        //myservoq.write(90);
      }
    } else {
      Serial.println("Invalid data header.");
      return;
        // 出错就直接结束这次 loop
    }
  }
  
  if (!dataReceived){ /*myservop.write(135);myservoq.write(60);*/return;}   //40

  if (!updateSowAndSpeed(80.0,20.0))
  {b_walk_repeat(L_first, _L,L_last, p, v_cmd0);
  

  }else{
  //light(2000);
  walk_sequence();
  } 
  suduq = v + k1*x_val + k2*a_val;
  sudup = v - k1*x_val - k2*a_val;
  sudud = v - k1*x_val + k2*a_val;
  sudub = v + k1*x_val - k2*a_val;
  
  //  v_turn  v_re  800ms
  //if(step==-500 || pkt.flag ==2){
  //  step=-500;
  //  qm.setspeed(0);
  //  pm.setspeed(0);
  //  dm.setspeed(0);
  //  bm.setspeed(0);
  //  return;
  //}
  
  
  qm.setspeed(suduq);pm.setspeed(sudup);dm.setspeed(sudud);bm.setspeed(sudub);

}

// 更新 sow 和 v 的逻辑
bool updateSowAndSpeed(float v_turn,float v_re) {
  if (step >= 14 && sow==0){return true;}
  //if (step ==13 && y>=1 && _final==1 ){k1=0;k2=0;}
  if (step <=13 && step % 2 == 1) {
    if (!_check) {           // 还没复位
        if (check_sow() || time_out()) {    
          y=0;
          _check = true;   // 标记复位完成
          Time_out = false;
          k1=0.5;       
          k2=0.5; 
          sow = 1;         
        } else {
          sow = 0;
          k1=2;       
          k2=2.5;       //1.5   1.5
          v = v_re;          //复位速度
        }
    } else {
      // 已经复位过了，持续 sow
      sow = 1;
    }
  } else if (step==0 ||  y>=y_sow || _check == false )  { //一直卡在复位进入非播种区 && y=0
    sow = 0; 
    y=0;
    _check = false;
    Time_out = false;
    v = v_turn;                   // 拐弯时速度
    if (step<14) {k1=1.0;k2=1.0;}    //     1.2  1.2
    if (step==14) {k1=1.2;k2=1.5;}
  }      
  return false;
   
}
bool time_out(){
  
  static unsigned long Time_;
  if(!Time_out) {  Time_=millis()+500;Time_out=true;return false;}
  if(millis() > Time_) {Time_out=false;return true;}
  return false;
}
bool time_out2(){
  
  static unsigned long Time_;
  if(!Time_out) {  Time_=millis()+1000;Time_out=true;return false;}
  if(millis() > Time_) {return true;}
  return false;
}
//----------------------------不停车-匀速-播种----------------------
void light(unsigned long servoDelayTime)
{ 
  //Serial.println("light10");
  if ( ((millis() - servoStartTime) >= servoDelayTime) ){        //&& ((millis() - servoStartTime) >= servoDelayTime)
    if (!servoReady) 
    {
      servoStartTime = millis();servoReady = true;v=50;
      k1=1.5;k2=1.5;      //   0.8   0.8
      return ;

    }
    //if (check_sow() || time_out2()){
    //  b_walk(50,p,40);  // 检查是否过了延迟时间  
    //  k1=0;k2=0;
    //}else{k1=0.8;k2=0.8;v=15;}
    //Serial.println("light");
    b_walk(70,p,70);  // 检查是否过了延迟时间  
    k1=0;k2=0;
                
  }
}


bool check_sow()
{ 

  bool t =-7 <= x_val && x_val <= 7 && -12 < a_val && a_val < 12 ;
  return t ;
}



// 检查编码器是否停止，若停止则返回 true
bool check_stop() {
  float currentEncoderAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
  
  // 检查编码器值是否在设定的时间内没有变化
  if (millis() - lastEncoderTime > 5) { // 5ms 检查一次
    if ((currentEncoderAvg - lastEncoderAvg) < 1.0) {  // 1.0 是一个容差值，代表编码器变化非常小
      return true;  // 停止
    }
    lastEncoderAvg = currentEncoderAvg;  // 更新编码器值
    lastEncoderTime = millis();          // 更新时间戳
  }
  
  return false;  // 没有停止
}


void walk_sequence() {
  switch (moveStage) {

    case STAGE_UP:
      if (bw_up(80, 80)) {
        moveStage = STAGE_LEFT;    // 上走完 -> 下
        d_init=false;
        k1=0.5;k2=0;
      }
      break;



    case STAGE_LEFT:
      if (b_walk(70,p,70)) {
        
        moveStage = STAGE_DONE;   // 左走完 -> 下一步右
        d_init=false;
      }
      break;

    case STAGE_DONE:
      // 所有方向走完，可停止或循环
      qm.brake();
      pm.brake();
      dm.brake();
      bm.brake();
      break;
  }
}
bool b_left(const float L, const float _v) {
  if (b_walk_dir(L, p, _v, DIR_LEFT)) return true;
  suduq = sudub = -v;
  sudup = sudud = v;
  return false;
}

bool b_right(const float L, const float _v) {
  if (b_walk_dir(L, p, _v, DIR_RIGHT)) return true;
  suduq = sudub = v;
  sudup = sudud = -v;
  return false;
}

bool b_up(const float L, const float _v) {
  if (b_walk_dir(L, p, _v, DIR_UP)) return true;
  suduq = sudup = v;
  sudub = sudud = v;
  return false;
}

bool b_down(const float L, const float _v) {
  if (b_walk_dir(L, p, _v, DIR_DOWN)) return true;
  suduq = sudup = -v;
  sudub = sudud = -v;
  return false;
}

bool bw_up(const float L,const float _v){
  static float startAvg = 0.0;
  static float targetDeltaCnt = 0;
  
  if (!d_init) {
    startAvg = lastcount;              // 用你在 step==14 时记录的 lastcount
    targetDeltaCnt =  L*p;   // 目标位置
    d_init = true;
  }
  //Serial.print(startAvg);
  //Serial.print(targetDeltaCnt);
  float nowAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
  //Serial.println(nowAvg);
  float delta  = nowAvg - startAvg;                // 已走的计数(正前进，负后退)
  float remainCnt = targetDeltaCnt - delta;        // 剩余计数

  v= remainCnt / targetDeltaCnt *_v;
  if (remainCnt < 50 && remainCnt>-50) {
    qm.brake();
    pm.brake();
    dm.brake();
    bm.brake();
  }else if (remainCnt >= 50){
    v=_v;
  }else {/*v= remainCnt / targetDeltaCnt *_v;*/v=-_v;}

  // 检查是否停止
  if ( bm.braked() && check_stop()) {
    qm.on();
    pm.on();
    dm.on();
    bm.on();
    return true;  // 停止，返回 true
  }                  // 未到达，下一次循环继续
  //sudup=suduq=sudub=sudud=v;
  return false;
}
bool b_walk_dir(const float L, const float p, const float _v, Direction dir) {
    static float startAvg = 0.0;
    static float targetDeltaCnt = 0;


    // 初始化阶段
    if (!d_init) {
        float qv = qm.readencoder();
        float pv = pm.readencoder();
        float dv = dm.readencoder();
        float bv = bm.readencoder();

        // 根据方向决定编码器符号（左/右/上下时部分反向）
        switch (dir) {
            case DIR_LEFT:
                qv = -qv; bv = -bv;
                break;
            case DIR_RIGHT:
                pv = -pv; dv = -dv;
                break;
            case DIR_UP:
                qv = -qv; pv = -pv;
                break;
            case DIR_DOWN:
                bv = -bv; dv = -dv;
                break;
        }

        startAvg = (qv + pv + dv + bv) / 4.0;
        targetDeltaCnt = L * p;
        d_init = true;
    }

    // 实时计算平均增量（考虑方向符号）
    float qv = qm.readencoder();
    float pv = pm.readencoder();
    float dv = dm.readencoder();
    float bv = bm.readencoder();

    switch (dir) {
        case DIR_LEFT:
            qv = -qv; bv = -bv;
            break;
        case DIR_RIGHT:
            pv = -pv; dv = -dv;
            break;
        case DIR_UP:
            
            break;
        case DIR_DOWN:
            bv = -bv; dv = -dv;qv = -qv;pv = -pv;
            break;
    }

    float nowAvg = (qv + pv + dv + bv) / 4.0;
    float delta = nowAvg - startAvg;
    float remainCnt = targetDeltaCnt - delta;

    v = remainCnt / targetDeltaCnt * _v;
    if (remainCnt < 100) {
        qm.brake(); pm.brake(); dm.brake(); bm.brake();
    } else if (remainCnt > 200) {
        v = _v;
    } else {
        v = remainCnt / targetDeltaCnt * _v;
    }

    // 检查停止条件
    if (bm.braked() && check_stop()) {
        qm.on(); pm.on(); dm.on(); bm.on();

        return true;
    }
    return false;
}

//----------------------------盲走-距离-速度---------------------

bool b_walk(const float L,const float p,const float _v) {
  
  static float startAvg = 0.0;
  static float targetDeltaCnt = 0;
  //static unsigned long t0 = 0;

  // 初始化：仅第一次或上次完成后再次调用时执行
  if (!_init) 
  {
    startAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
    targetDeltaCnt = L*p;           // 目标增量(可能为负表示后退)
    _init = true;
  }
  float nowAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
  float delta  = nowAvg - startAvg;                // 已走的计数(正前进，负后退)
  float remainCnt = targetDeltaCnt - delta;        // 剩余计数
  v= remainCnt / targetDeltaCnt *_v;
  if (remainCnt < 100) {
    qm.brake();
    pm.brake();
    dm.brake();
    bm.brake();
  }else if (remainCnt > 200){
    v=_v;
  }else {v= remainCnt / targetDeltaCnt *_v;}

  // 检查是否停止
  if ( bm.braked() && check_stop()) {
    qm.on();
    pm.on();
    dm.on();
    bm.on();
    return true;  // 停止，返回 true
  }                  // 未到达，下一次循环继续
  return false;
}
bool bp_walk(const float L,const float _v) {
  
  static float startAvg = 0.0;
  static float targetDeltaCnt = 0;
  //static unsigned long t0 = 0;

  // 初始化：仅第一次或上次完成后再次调用时执行
  if (!_init) 
  {
    startAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
    targetDeltaCnt = L*p;           // 目标增量(可能为负表示后退)
    _init = true;
  }
  float nowAvg = (qm.readencoder() + pm.readencoder() + dm.readencoder() + bm.readencoder()) / 4.0;
  float delta  = nowAvg - startAvg;                // 已走的计数(正前进，负后退)
  float remainCnt = targetDeltaCnt - delta;        // 剩余计数
  v= remainCnt / targetDeltaCnt *_v;
  if (remainCnt < 100) {
    qm.brake();
    pm.brake();
    dm.brake();
    bm.brake();
  }else if (remainCnt > 200){
    v=_v;
  }else {v= remainCnt / targetDeltaCnt *_v;}

  // 检查是否停止
  if ( bm.braked() && check_stop()) {
    qm.on();
    pm.on();
    dm.on();
    bm.on();
    return true;  // 停止，返回 true
  }                  // 未到达，下一次循环继续
  return false;
}


// 返回值：当 sow==0 时返回 true（表示“停止连续执行”）；其它时间返回 false（表示仍在执行/可继续调用）
bool b_walk_repeat( const float L_first, const float L_middle,const float L_last, const float p, const float _v) {

    // -------------退出播种循环--------------------------------
    if (sow == 0) 
    { 
      _init = false;
      y=0;
      return true;               
    }
    if (step%2==0)
    {
      y=y_sow-1;
    }

    // --------------进入mangzou距离选择--------------------------------------

    float L_curr;
    if (y == 0) {
      L_curr = L_first;
    } else if (y % (y_sow-1) == 0) {
      L_curr = L_last;
    } else {
      L_curr = L_middle;
    }

    //----------------盲走实现-----------------
    bool M,N,K;
    if (y < y_sow) {
      M = b_walk(L_curr, p, _v);
      N = b_stepper_move();
      K = d_stepper_move();
      if (M && N && K){ 
        y+=1;
        _init = false;


        if (y%2 == 0) {
        d_start = false;
        }else {
        b_start = false;}
      }


    } else {
      v=50;
    }


    
    return false;  
}


// 用于控制步进电机分两段行走
bool b_stepper_move( ) {
  
  static int sp_p = 2;
  static unsigned long last_time = 0;  // 用于记录上次的时间
  static const unsigned long pause_duration =     600;  // 设置延迟时间为    1000-525=0.475       300-700
  if (!b_start ) 
  {
    b_start=true;
    sp_p = 0;
    last_time = millis(); 
  }

  if (sp_p==2 && c) 
  {

    return true;
  }
  if (sp_p == 1) {
      unsigned long current_time = millis();
      if (current_time - last_time >= pause_duration) {
      }else {return false;}
  }

  if (!myStepperp.run() && sp_p!=2)
  {
    myStepperp.moveTo(myStepperp.currentPosition() + steps_per_move);
    sp_p+=1;
  }
    

    return false;  // 尚未完成
}
bool d_stepper_move() {
    
  static int sp_q = 2;
  static unsigned long last_time = 0;  // 用于记录上次的时间
  static const unsigned long pause_duration = 600;  // 设置延迟时间为1000-525=0.475
  if (!d_start ) 
  {
    d_start=true;
    sp_q = 0;
    last_time=millis();
  }
  if (sp_q==2 && myStepperq.distanceToGo() == 0) 
  {

    return true;
  }
  if (sp_q == 1) {
      unsigned long current_time = millis();
      if (current_time - last_time >= pause_duration) {
      }else {return false;}
  }
  if (!myStepperq.run() && sp_q!=2)
  {
    myStepperq.moveTo(myStepperq.currentPosition() + steps_per_move);
    sp_q+=1;
  }
    

    return false;  // 尚未完成
}
//----------------------------从-直流电机-主-步进电机--------------------
/*void check_follow(const float &L,const float &p) {
  static long start_Position = 0; 
  static int l_step = 1280;
  if (!myStepper.run()) 
  {
    start_Position = myStepper.currentPosition();  
    myStepper.moveTo( start_Position + l_step);
  }

  float nowstep = myStepper.currentPosition()
  float delta_step  = nowstep - start_Position;                // 已走的计数(正前进，负后退)
  float remainstep = l_step - delta_step;        // 剩余计数
}
*/







