/*
 * HWT101 
 *
 * 板子：Arduino Mega 2560
 * 接线：
 *   HWT101 TX  -> Mega RX1 (引脚 19)
 *   HWT101 RX  -> Mega TX1 (引脚 18)
 *   5V / GND 
 * 功能：
 *   读取陀螺仪 Z 轴角速度和偏航角
 *   顺时针转动时角度减小,0->-180->180->0  一圈360度 在-180和180会发生跳变
 * 初始化角度：
 *     WitSetYAW(0x5FFF); //   0x0000-0x7FFF 时 0x5FFF / 0xFFFF *360 = 90.0000° 
 *     WitSetYAW(0xC000); //   0x8000-0xFFFF 时 0xC000 / 0xFFFF *360 - 360 = -90.00
 * 注意:
 * 陀螺仪出厂默认波特率9600，使用 WitSetUartBaud() 可以设置为 115200,后续要用Serial1.begin(115200) 打开串口
 * 但偶尔快速断电重启后波特率会恢复为9600，所以需要重新设置波特率,即：Serial1.begin(9600),烧一次代码; 然后改回Serial1.begin(115200);
 * 否则，HWT101_read() 读取不到数据
 */

#include <wit_c_sdk.h>
#include <REG.h>

// 数据更新标志位（与 SDK 内部 s_cDataUpdate 配合使用）
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80
static volatile char s_cDataUpdate = 0;

// 解析结果
float rawGyroZ = 0.0f;   // Z 轴角速度，单位 °/s，量程 ±2000
float yawAngle = 0.0f;   // 偏航角，单位 °，范围 -180 ~ +180

/* ---------------- SDK 要求的三个回调函数 ---------------- */

// 把配置命令写回传感器（SDK 调用）
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    Serial1.write(p_data, uiSize);
    Serial1.flush();
}

// SDK 解析到数据时回调：记录更新了哪些寄存器
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    for (uint32_t i = 0; i < uiRegNum; i++) {
        switch (uiReg) {
            case GZ:   // 寄存器 0x39（57）：Z 轴角速度
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case Yaw:  // 寄存器 0x3F（63）：偏航角
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}

// SDK 需要延时时调用
static void Delayms(uint16_t ucMs) {
    delay(ucMs);
}

/* ---------------- 数据读取：喂字节 + 换算 ---------------- */

void HWT101_read() {
    while (Serial1.available()) {
        WitSerialDataIn(Serial1.read());   // 把收到的字节交给 SDK 解析
    }
    if (s_cDataUpdate) {
        // 16 位有符号数换算到物理量
        rawGyroZ = sReg[GZ]  / 32768.0f * 2000.0f;   // °/s
        yawAngle = sReg[Yaw] / 32768.0f * 180.0f;    // °
        s_cDataUpdate = 0;
    }
}

void setup() {
    Serial.begin(115200);      // USB 调试串口
    Serial1.begin(115200);     // 连接 HWT101 的串口

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);     // 普通协议，地址 0x50
    WitSerialWriteRegister(SensorUartSend); // 注册发送函数
    WitRegisterCallBack(SensorDataUpdata);  // 注册解析回调
    WitDelayMsRegister(Delayms);            // 注册延时函数

    WitSetUartBaud(WIT_BAUD_115200);        // 设置传感器波特率
    delay(300);
    WitSetYAW(0x5FFF);                      // 把当前角度设为 135 参考
                                          
    delay(300);
    WitSetOutputRate(RRATE_50HZ);           // 输出频率 50 Hz
    delay(300);

    Serial.println("HWT101 ready");
}

void loop() {
    HWT101_read();

    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        Serial.print("gyroZ: ");
        Serial.print(rawGyroZ, 2);
        Serial.print(" deg/s   yaw: ");
        Serial.print(yawAngle, 2);
        Serial.println(" deg");
    }
}
