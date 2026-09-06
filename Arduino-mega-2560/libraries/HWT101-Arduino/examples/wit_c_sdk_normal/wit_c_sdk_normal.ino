/*
此示例程序为传感器使用
HWT101不需要设置任何校准模式
初始化传感器
1.选择通信方式，后面传入的0x50是没用的
WitInit(WIT_PROTOCOL_NORMAL, 0x50);
2.传入Arduino往传感器发送数据的函数
WitSerialWriteRegister(SensorUartSend);
3.传入sREG数组数据更新提示函数
WitRegisterCallBack(SensorDataUpdata);
4.传入延时函数
WitDelayMsRegister(Delayms);


Arduino如何接收传感器的数据？

1.对传感器进行一系列设置
  包括波特率，传输速率，传输内容等

2.传感器会不断往Arduino的串口发送数据
  发送的数据会放在串口的接收寄存器中

3.主程序读取传感器发来的数据
  如果接收寄存器中有数据 Serial1.available()
  那么就每次读取一个字节 Serial1.read()
  并将读到的数据传入WitSerialDataIn()进行处理
  程序不断读取串口接收缓存里面的字节
  直到找到一条完整的数据（数据头正确，校验和也正确），一条完整的数据包含11个字节
  找到的完整数据会在WitSerialDataIn()调用CopeWitData(数据类型, 四个16字节数据)函数进行处理
  一条完整的数据接收并且处理完之后，程序会继续接收缓存里的字节，直到缓存中所有的数据被接收

4.CopeWitData()函数根据数据类型把四个16字节的数据存入sREG数组相应位置
  每当sREG数组中的数据被更新后，CopeWitData()函数会调用SensorDataUpdata()函数
  SensorDataUpdata()函数会更新s_cDataUpdate的值来指明什么数据被更新了

5.串口的数据全部被读完后，主程序检查s_cDataUpdate变量，根据变量的取值知道哪些数据发生了更新
  然后从sREG数组中读取数据

*/


/*
传感器和Arduino的硬件连接方式

  VCC <--->  5V/3.3V
  TX  <--->  19(RX1) 默认接串口1
  RX  <--->  18(TX1)
  GND <--->  GND

*/

#include <REG.h> //寄存器地址的宏定义+标志位设置值的宏定义
#include <wit_c_sdk.h> //接口函数变量的宏定义

#define ACC_UPDATE		0x01 //s_cDataUpdate的第一位置1说明加速度被更新
#define GYRO_UPDATE		0x02 //s_cDataUpdate的第二位置1说明角速度被更新
#define ANGLE_UPDATE	0x04 //s_cDataUpdate的第三位置1说明角度被更新
#define MAG_UPDATE		0x08 //s_cDataUpdate的第四位置1说明磁场被更新
#define READ_UPDATE		0x80 //s_cDataUpdate的第八位置1说明要读取的寄存器数据已更新
static volatile char s_cDataUpdate = 0;

//定义静态函数，该静态函数只能在本程序中被调用，别的程序没法调用该函数，防止重名函数导致程序混乱
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

void setup() {
  //Arduino与电脑之间的串行通信波特率
  Serial.begin(115200);
  //Arduino与传感器之间的串行通信波特率
  Serial1.begin(115200);
  //WIT_PROTOCOL_NORMAL宏定义为0，设置通信方式为普通模式，后面的地址没啥用，和串行通信没什么关系
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  //WitSerialWriteRegister()是传感器提供的skd中的一个函数
  //该函数传入一个在本Arduino程序中定义的函数SensorUartSend()的指针
  //指针被传入函数后，会修改sdk中一个全局变量p_WitSerialWriteFunc
  //传感器要往Arduino发数据的时候就会调用这里所写的函数，之所以这个函数被写在Arduino程序中，是为了修改方便
  WitSerialWriteRegister(SensorUartSend);
  //WitRegisterCallBack()是传感器提供的skd中的一个函数
  //该函数传入一个在本Arduino程序中定义的函数SensorDataUpdata()的指针
  //指针被传入函数后，会修改sdk中一个全局变量p_WitRegUpdateCbFunc
  //传感器要往Arduino发数据的时候就会调用这里所写的函数，之所以这个函数被写在Arduino程序中，是为了修改方便
  WitRegisterCallBack(SensorDataUpdata);
  //注册延时函数
  WitDelayMsRegister(Delayms);
}
float fGyro, fAngle;
void loop() {
  //传感器会按照设定的频率向Arduino的串口1发送数据
  //串口1接收到数据后会将它存放在
  while (Serial1.available())
  {
    WitSerialDataIn(Serial1.read());//读取串口1(传感器)数据
  }
  if (s_cDataUpdate) //在注册获取传感器数据回调函数时会对变量进行赋值
  {
    fGyro = sReg[57] / 32768.0f * 2000.0f; //算法公式
    fAngle = sReg[63] / 32768.0f * 180.0f; //算法公式
    if (s_cDataUpdate & GYRO_UPDATE)
    {
      Serial.print("GyroZ: ");
      Serial.print(fGyro, 1);
      Serial.print("\r\n");
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if (s_cDataUpdate & ANGLE_UPDATE)
    {
      Serial.print("AngleZ: ");
      Serial.print(fAngle, 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    s_cDataUpdate = 0;
  }
}



//Arduino往传感器发送数据使用的函数
//本质是把要发送给传感器的数据和数据个数传入函数，函数当中通过Arduino的串口1向传感器发送数据
//传感器本身带有保存设定的功能，使用的时候设定好传感器的参数然后保存就行，运行的过程中一般不需要往传感器发送数据
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  //这里规定了往Arduino的串口1发送数据，也可以改为其他的硬件串口
  Serial1.write(p_data, uiSize);
  //所有发送缓存中的数据全部发送出去后，传感器才会执行后续的操作
  Serial1.flush();
}
//Arduino的串口有数据传入之后，skd中的函数会把数据读到一个数组（sREG）里，并且调用“数组数据更新提示”函数
//也就是下面写的这个函数
//在用户自己编写的Arduino代码中有一个s_cDataUpdate
//“数组数据更新提示”函数会把s_cDataUpdate的某些二进制位的置1或者置0，表示数组里的数据发生了更新
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{ //uiReg是数组的下标，不同的下标位置存放了不同的数据，uiRegNum表示更新的数据的个数，比如AX，AY，AZ都更新，那么uiRegNum = 3
  int i;
  //这里之所以要循环是因为
  //传入函数的uiReg一般都是AX，GX，Roll所以需要循环一下确定是否存在GZ和Yaw
  for (i = 0; i < uiRegNum; i++)
  {
    switch (uiReg)
    {
      case GZ: //如果是Z轴角速度，GZ定义为0x39（在REG.h文件中）
        s_cDataUpdate |= GYRO_UPDATE; //把s_cDataUpdate变量的第二位置1
        break;
      case Yaw: //如果是Z轴角度，Yaw定义为0x3f
        s_cDataUpdate |= ANGLE_UPDATE; //把s_cDataUpdate变量的第三位置1
        break;
      default: //如果都不是
        s_cDataUpdate |= READ_UPDATE; //把s_cDataUpdate变量的第八位置1
        break;
    }
    //下标+1，和数据个数是一致的，因为一个下标存放一个数据
    uiReg++;
  }
}
//修改传感器的输出方式的时候需要通过串口往传感器发送数据
//传感器接收到数据之后需要一点时间来响应
//因此往传感器发送数据的时候需要插入一些延时
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}