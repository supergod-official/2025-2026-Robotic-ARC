#include "PC_carA.h"
#include "PCencoder_.h"
#include "ASABE_StateMachine.h"
#include <MsTimer2.h> 
#include <NewPing.h>
#include <REG.h> 
#include <wit_c_sdk.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <math.h>

#define useULtra_true 0
#define USE_FRONT_LASER 1
#define USE_BACK_LASER 1
#define USE_TEST_LOOP 0
#define MAIN_TASK 1
#define NAVIGATION_TASK 0
#define PLANT_ACTUATION_TASK 0
#define PLANT_ACTUATION_TASK2 0
#define FORWARD_HOLD_ANGLE_TEST 0
#define SPEED_MEASURE_TEST 0

#if PLANT_ACTUATION_TASK && PLANT_ACTUATION_TASK2
#error "Only one plant actuation task mode can be enabled"
#endif

#define useVl53_true (USE_FRONT_LASER || USE_BACK_LASER)
/*--------------------------------VL53L1X-------------------------------__*/

#define SHUTDOWN_PIN_F 48
#define INT_PIN_F 49
#define SHUTDOWN_PIN_B 51
#define INT_PIN_B 50
#define VL53L1X_ADDR_1 0x54
#define VL53L1X_ADDR_2 0x56
#define VL53L1X_ROI_WIDTH 12
#define VL53L1X_ROI_HEIGHT 8
#define VL53L1X_ROI_CENTER 195
#define VL53L1X_INIT_RETRY_COUNT 5
#define VL53L1X_MIN_VALID_CM 1.0f
#define VL53L1X_MAX_VALID_CM 350.0f
#define VL53L1X_VALID_RANGE_STATUS 0U
#define VL53L1X_STALE_TIMEOUT_MS 250UL
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
static uint8_t vl53StatusF = 255;
static uint8_t vl53StatusB = 255;
static float vl53ReadCmF = 0.0f;
static float vl53ReadCmB = 0.0f;
static unsigned long vl53LastValidMsF = 0;
static unsigned long vl53LastValidMsB = 0;
static uint8_t plantRouteStateDebug = 0;
static float plantDistanceToGoDebug = 0.0f;
static double plantCarSpeedDebug = 0.0;
static bool plantFrontLaserFreshDebug = false;
static bool plantBackLaserFreshDebug = false;
/*--------------------------------HWT101--------------------------------__*/
//闂勨偓閾昏桨鍗庨崣姗€鍣?
#define ACC_UPDATE		0x01 //s_cDataUpdate
#define GYRO_UPDATE		0x02 //s_cDataUpdate
#define ANGLE_UPDATE	0x04 //s_cDataUpdate
#define MAG_UPDATE		0x08 //s_cDataUpdate
#define READ_UPDATE		0x80 //s_cDataUpdate
static volatile char s_cDataUpdate = 0;


//鐎规矮绠熼棃娆愨偓浣稿毐閺?
static void HWT101_read();
static void VL53L1X_read();
static bool VL53L1X_beginWithRetry(SFEVL53L1X& sensor, const char* name);
static bool VL53L1X_isValidDistanceCm(float distanceCm, uint8_t rangeStatus);
static bool VL53L1X_isFresh(unsigned long lastValidMs);
static void Main_loop();
static void Test_loop();
static void sendTestUartStatus();
static float normalizeAngleDeg(float angle);
static float angleErrorDeg(float angle, float target);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);









#if useULtra_true
/*--------------------------------HC_SR04----------------------------------*/
#define TRIGGER_PINf  A2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PINf     A3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCEf 50.0f // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINl  A9  
#define ECHO_PINl     A8 
#define MAX_DISTANCEl 15.0f

#define TRIGGER_PINr  A4  
#define ECHO_PINr     A5  
#define MAX_DISTANCEr 15.0f

unsigned long US_prMicros = 0;

#define US_Interval   20000                    
#define US_TIME_TO_DISTANCE  0.01717  


NewPing sonarF(TRIGGER_PINf, ECHO_PINf, MAX_DISTANCEf); 
NewPing sonarL(TRIGGER_PINl, ECHO_PINl, MAX_DISTANCEl);
NewPing sonarR(TRIGGER_PINr, ECHO_PINr, MAX_DISTANCEr);

void US_Read(uint8_t currentState) {
    if (micros() - US_prMicros >= US_Interval) {
        US_prMicros = micros();
        switch (currentState) {
            case 0:
                sensorData.DistanceL = sonarL.ping() * US_TIME_TO_DISTANCE;
                sensorData.DistanceR = sonarR.ping() * US_TIME_TO_DISTANCE;
                if (sensorData.DistanceR < 0.1 && -0.1 < sensorData.DistanceR) 
                    sensorData.DistanceR = MAX_DISTANCEr;
                if (sensorData.DistanceL < 0.1 && -0.1 < sensorData.DistanceL) 
                    sensorData.DistanceL = MAX_DISTANCEl;
                break;
            case 1:
                sensorData.rawDistanceF = sonarF.ping() * US_TIME_TO_DISTANCE;
                sensorData.DistanceL = sonarL.ping() * US_TIME_TO_DISTANCE;
                sensorData.DistanceR = sonarR.ping() * US_TIME_TO_DISTANCE;

                if (sensorData.DistanceF < 0.1 && -0.1 < sensorData.DistanceF ) 
                    sensorData.rawDistanceF = MAX_DISTANCEf;
                if (sensorData.DistanceL < 0.1 && -0.1 < sensorData.DistanceL) 
                    sensorData.DistanceL = MAX_DISTANCEl;
                if (sensorData.DistanceR < 0.1 && -0.1 < sensorData.DistanceR) 
                    sensorData.DistanceR = MAX_DISTANCEr;
                break;
            case 2:
                sensorData.DistanceR = MAX_DISTANCEr;
                sensorData.DistanceL = MAX_DISTANCEl;
                sensorData.rawDistanceF = MAX_DISTANCEf;
                break;
            default:
                break;
        }
    }
    //Serial.println(sensorData.DistanceF);
    //Serial.println(sensorData.DistanceR);
}
#else 
    //"None"
#endif
void setup() {
    Wire.begin();
    Wire.setClock(400000); // Set I2C frequency to 400kHz
    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    Serial1.begin(115200);
    Serial2.begin(115200);

    if(!qm.attach(DIR_PIN_QM_1, DIR_PIN_QM_2, PWM_PIN_QM, PC_PIN_QM_1, PC_PIN_QM_2, 0.02)){Serial.print("bad attach qm"); while(1);}
    qm.setpid(m_pid.kp, m_pid.ki, m_pid.kd, m_pid.pulse_to_distance,m_pid.deadvoltage,m_pid.integralmax);   

    if(!pm.attach(DIR_PIN_PM_1, DIR_PIN_PM_2, PWM_PIN_PM, PC_PIN_PM_1, PC_PIN_PM_2, 0.02)){Serial.print("bad attach pm");while(1);}
    pm.setpid(m_pid.kp, m_pid.ki, m_pid.kd, m_pid.pulse_to_distance,m_pid.deadvoltage,m_pid.integralmax);

    if(!dm.attach(DIR_PIN_DM_1, DIR_PIN_DM_2, PWM_PIN_DM, PC_PIN_DM_1, PC_PIN_DM_2, 0.02)){Serial.print("bad attach dm");while(1);}
    dm.setpid(m_pid.kp, m_pid.ki, m_pid.kd, m_pid.pulse_to_distance,m_pid.deadvoltage,m_pid.integralmax);

    if(!bm.attach(DIR_PIN_BM_1, DIR_PIN_BM_2, PWM_PIN_BM, PC_PIN_BM_1, PC_PIN_BM_2, 0.02)){Serial.print("bad attach bm");while(1);}
    bm.setpid(m_pid.kp, m_pid.ki, m_pid.kd, m_pid.pulse_to_distance,m_pid.deadvoltage,m_pid.integralmax);
    
    //tuoluoyi---------------------------------------------------------
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    WitSetUartBaud(WIT_BAUD_115200);
    delay(300);
#if NAVIGATION_TASK
    WitSetYAW(0x4000); // 90 deg
#else
    WitSetYAW(0xC000); // 270 deg -> -90 deg
#endif
    delay(300);
    WitSetOutputRate(RRATE_50HZ);
    delay(300);
    
    // MT---------------------------------------------------------
    // Timer2 
    MsTimer2::set(0.02*1000, timerCallback);
    MsTimer2::start();
    // Timer3 
    PCencoder::_setTimer3();
    //婵€鍏?----------------------------------------------------------
    //IN
    // 1) Init the sensor whose XSHUT is always high (the D3-controlled sensor is kept LOW)

#if useVl53_true
    pinMode(SHUTDOWN_PIN_F, OUTPUT);
    pinMode(SHUTDOWN_PIN_B, OUTPUT);

    pinMode(INT_PIN_F,INPUT);
    pinMode(INT_PIN_B,INPUT);
    digitalWrite(SHUTDOWN_PIN_F, LOW);
    digitalWrite(SHUTDOWN_PIN_B, LOW);
    delay(100);

#if USE_FRONT_LASER
    digitalWrite(SHUTDOWN_PIN_F, HIGH); 
    delay(100);
    if (!VL53L1X_beginWithRetry(distanceSensor1, "Sensor1")) {
    Serial.println("Sensor1 failed to begin. Check I2C wiring and D3 XSHUT state. Freezing...");
    while (1);
    }
    Serial.println("Sensor1 ranging started.");
    distanceSensor1.setI2CAddress(VL53L1X_ADDR_1);
    delay(10);
    distanceSensor1.setDistanceModeShort();
    distanceSensor1.setTimingBudgetInMs(33); // 
    distanceSensor1.setIntermeasurementPeriod(40); 
    //distanceSensor1.setROI(VL53L1X_ROI_WIDTH, VL53L1X_ROI_HEIGHT, VL53L1X_ROI_CENTER);

#endif

#if USE_BACK_LASER
    delay(500);
    digitalWrite(SHUTDOWN_PIN_B, HIGH);     
    delay(100);
    if (!VL53L1X_beginWithRetry(distanceSensor2, "Sensor2")) {
    Serial.println("Sensor2 failed to begin. D3 may not be controlling XSHUT correctly. Freezing...");
    while (1);
    }
    Serial.println("Sensor2 ranging started.");
    distanceSensor2.setI2CAddress(VL53L1X_ADDR_2);
    delay(10);
    distanceSensor2.setDistanceModeShort();
    distanceSensor2.setTimingBudgetInMs(33); // 
    distanceSensor2.setIntermeasurementPeriod(40); 
    //distanceSensor2.setROI(VL53L1X_ROI_WIDTH, VL53L1X_ROI_HEIGHT, VL53L1X_ROI_CENTER);
#endif

#if USE_FRONT_LASER
    distanceSensor1.startRanging();
#endif
#if USE_BACK_LASER
    distanceSensor2.startRanging();
#endif
    
#else 
#endif
#if MAIN_TASK
    buildDefaultTaskQueue(); // Main task route.
#elif NAVIGATION_TASK
    buildNavigationTaskQueue();
#endif
    stateStartMs = millis();
    delay(30);
}
int32_t a = 0;
int32_t pre_ = 0;
int32_t now_ = 0;
static uint8_t testUartEnable = 1;
void loop() {
//任务一：导航任务（20%）
//跑全程竞速，不需要uart
#if NAVIGATION_TASK
    HWT101_read();
    VL53L1X_read();
    Car_updateMotionInloop();
    updateStateMachine();
//任务二：Plant Actuation Task （20%）
//机器人要在 5 分钟一条垄内往返跑。需要uart
#elif PLANT_ACTUATION_TASK
    HWT101_read();
    VL53L1X_read();
    Car_updateMotionInloop();
    static uint8_t routeState = 0;  // 0: straight work zone, 1: 180 deg turn
    static float routeTargetAngle = -90.0f;
    static unsigned long routeInTargetStartMs = 0;
    static unsigned long routeStateStartMs = 0;
    static unsigned long routeStraightForceStartMs = 0;
    static uint8_t routeStraightForceDone = 0;

    const float routeTargetDistance = 10.0f;
    const float routeDistanceTolerance = 2.0f;
    const float routeAngleTolerance = 2.0f;

    const unsigned long routeHoldMs = 500UL;
    const unsigned long routeStraightForceMs = 15000UL;
    const unsigned long routeStraightMinMs = 10000UL;
    const unsigned long routeTurnMinMs = 100UL;

    const double routeStraightSpeedLimit = 5.0;
    const double routeStraightIntegralLimit = 5.0;
    const double routeTurnSpeedLimit = 10.0;
    const double routeTurnIntegralLimit = 10.0;
    if (routeStateStartMs == 0) routeStateStartMs = millis();

    float angleToGo = angleErrorDeg(sensorData.fAngle, routeTargetAngle);
    float distanceToGo = sensorData.DistanceF - routeTargetDistance;
    double carSpeed = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;
    double carSpeedW = (qm.readspeed() - pm.readspeed() + dm.readspeed() - bm.readspeed()) / 4.0f;
    bool inTargetBand = false;
    bool routeBrakeOnly = false;
    bool frontLaserFresh = VL53L1X_isFresh(vl53LastValidMsF);
    plantRouteStateDebug = routeState;
    plantDistanceToGoDebug = distanceToGo;
    plantCarSpeedDebug = carSpeed;
    plantFrontLaserFreshDebug = frontLaserFresh;
    if (routeState == 0) {
        testUartEnable = 1;
        if (!routeStraightForceDone) {
            if (routeStraightForceStartMs == 0) {
                routeStraightForceStartMs = millis();
                Car_clearDistanceAngleIntegral();
            }
            if (millis() - routeStraightForceStartMs < routeStraightForceMs) {
                expectedVoltage.speedX = routeStraightSpeedLimit;
                expectedVoltage.speedZ = SpeedW_contorll_limited(
                    angleToGo, routeTurnSpeedLimit, routeTurnIntegralLimit);
                routeInTargetStartMs = 0;
            } else {
                routeStraightForceDone = 1;
                routeStraightForceStartMs = 0;
                Car_clearDistanceAngleIntegral();
            }
        }

        if (routeStraightForceDone && !frontLaserFresh) {
            expectedVoltage.speedX = 0;
            expectedVoltage.speedZ = 0;
            Car_clearIngral();
            routeBrakeOnly = true;
            routeInTargetStartMs = 0;
        } else if (routeStraightForceDone && distanceToGo > 20.0f) {
            expectedVoltage.speedX = routeStraightSpeedLimit;
            Car_clearDistanceAngleIntegral();
        } else if (routeStraightForceDone && distanceToGo < -20.0f) {
            expectedVoltage.speedX = -routeStraightSpeedLimit;
            Car_clearDistanceAngleIntegral();
        } else if (routeStraightForceDone && fabs(distanceToGo) < 2.0f) {
            expectedVoltage.speedX = 0;
        } else if (routeStraightForceDone) {
            expectedVoltage.speedX = Speedv_contorll_limited(
                distanceToGo, routeStraightSpeedLimit, routeStraightIntegralLimit);
        }
        if (routeStraightForceDone) {
            expectedVoltage.speedZ = SpeedW_contorll_limited(
                angleToGo, routeTurnSpeedLimit, routeTurnIntegralLimit);

            inTargetBand = (fabs(distanceToGo) < routeDistanceTolerance) && (fabs(carSpeed) < 5.0);
            if (inTargetBand) {
                if (routeInTargetStartMs == 0) routeInTargetStartMs = millis();
            } else {
                routeInTargetStartMs = 0;
            }
        }

        if (routeStraightForceDone &&
            routeInTargetStartMs != 0 &&
            millis() - routeInTargetStartMs >= routeHoldMs &&
            millis() - routeStateStartMs >= routeStraightMinMs) {
            routeState = 1;
            routeTargetAngle = normalizeAngleDeg(routeTargetAngle + 180.0f);
            routeInTargetStartMs = 0;
            routeStateStartMs = millis();
            routeStraightForceStartMs = 0;
            routeStraightForceDone = 0;
            testUartEnable = 0;
            expectedVoltage.speedX = 0;
            expectedVoltage.speedZ = 0;
            Car_clearIngral();
            routeBrakeOnly = true;
        }
    } else {
        testUartEnable = 0;
        angleToGo = angleErrorDeg(sensorData.fAngle, routeTargetAngle);
        if (angleToGo > 30.0f) {
            expectedVoltage.speedZ = routeTurnSpeedLimit;
            Car_clearDistanceAngleIntegral();
        } else if (angleToGo < -30.0f) {
            expectedVoltage.speedZ = -routeTurnSpeedLimit;
            Car_clearDistanceAngleIntegral();
        } else {
            expectedVoltage.speedZ = SpeedW_contorll_limited(
                angleToGo, routeTurnSpeedLimit, routeTurnIntegralLimit);
        }
        expectedVoltage.speedX = 0;

        inTargetBand = (fabs(angleToGo) < routeAngleTolerance) && (fabs(carSpeedW) < 1.5);
        if (inTargetBand) {
            if (routeInTargetStartMs == 0) routeInTargetStartMs = millis();
        } else {
            routeInTargetStartMs = 0;
        }

        if (routeInTargetStartMs != 0 &&
            millis() - routeInTargetStartMs >= routeHoldMs &&
            millis() - routeStateStartMs >= routeTurnMinMs) {
            routeState = 0;
            routeInTargetStartMs = 0;
            routeStateStartMs = millis();
            routeStraightForceStartMs = 0;
            routeStraightForceDone = 0;
            testUartEnable = 1;
            expectedVoltage.speedX = 0;
            expectedVoltage.speedZ = 0;
            Car_clearIngral();
            routeBrakeOnly = true;
        }
    }
    if (routeBrakeOnly) {
        Car_brake();
    } else {
        qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
        pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
        dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
        bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
    }
//任务三：主任务 （40%）
//10分钟跑全程，需要uart
#elif PLANT_ACTUATION_TASK2
    HWT101_read();
    VL53L1X_read();
    Car_updateMotionInloop();
    static uint8_t routeState = 0;  // 0: forward with front laser, 1: backward with back laser
    static unsigned long routeInTargetStartMs = 0;
    static unsigned long routeStateStartMs = 0;
    static unsigned long routeForceStartMs = 0;
    static uint8_t routeForceDone = 0;

    const float routeTargetAngle = -90.0f;
    const float routeForwardTargetDistance = 10.0f;
    const float routeBackTargetDistance = 10.0f;
    const float routeDistanceTolerance = 2.0f;

    const unsigned long routeHoldMs = 500UL;
    const unsigned long routeForceMoveMs = 15000UL;
    const unsigned long routeStateMinMs = 15000UL;

    const double routeMoveSpeedLimit = 5.0;
    const double routeMoveIntegralLimit = 5.0;
    const double routeAngleSpeedLimit = 10.0;
    const double routeAngleIntegralLimit = 10.0;

    unsigned long nowMs = millis();
    if (routeStateStartMs == 0) routeStateStartMs = nowMs;

    float angleToGo = angleErrorDeg(sensorData.fAngle, routeTargetAngle);
    float distanceToGo = 0.0f;
    bool activeLaserFresh = false;
    bool routeBrakeOnly = false;
    bool inTargetBand = false;
    bool frontLaserFresh = VL53L1X_isFresh(vl53LastValidMsF);
    bool backLaserFresh = VL53L1X_isFresh(vl53LastValidMsB);
    double carSpeed = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;

    testUartEnable = 1;
    if (routeState == 0) {
        distanceToGo = sensorData.DistanceF - routeForwardTargetDistance;
        activeLaserFresh = frontLaserFresh;
    } else {
        distanceToGo = routeBackTargetDistance - sensorData.DistanceB;
        activeLaserFresh = backLaserFresh;
    }

    plantRouteStateDebug = routeState;
    plantDistanceToGoDebug = distanceToGo;
    plantCarSpeedDebug = carSpeed;
    plantFrontLaserFreshDebug = frontLaserFresh;
    plantBackLaserFreshDebug = backLaserFresh;

    if (!routeForceDone) {
        if (routeForceStartMs == 0) {
            routeForceStartMs = nowMs;
            Car_clearDistanceAngleIntegral();
        }
        expectedVoltage.speedX = (routeState == 0) ? routeMoveSpeedLimit : -routeMoveSpeedLimit;
        expectedVoltage.speedZ = SpeedW_contorll_limited(
            angleToGo, routeAngleSpeedLimit, routeAngleIntegralLimit);
        routeInTargetStartMs = 0;

        if ((unsigned long)(nowMs - routeForceStartMs) >= routeForceMoveMs) {
            routeForceDone = 1;
            routeForceStartMs = 0;
            Car_clearDistanceAngleIntegral();
        }
    }

    if (routeForceDone) {
        if (!activeLaserFresh) {
            expectedVoltage.speedX = 0;
            expectedVoltage.speedZ = 0;
            Car_clearIngral();
            routeBrakeOnly = true;
            routeInTargetStartMs = 0;
        } else {
            if (distanceToGo > 20.0f) {
                expectedVoltage.speedX = routeMoveSpeedLimit;
                Car_clearDistanceAngleIntegral();
            } else if (distanceToGo < -20.0f) {
                expectedVoltage.speedX = -routeMoveSpeedLimit;
                Car_clearDistanceAngleIntegral();
            } else if (fabs(distanceToGo) < routeDistanceTolerance) {
                expectedVoltage.speedX = 0;
            } else {
                expectedVoltage.speedX = Speedv_contorll_limited(
                    distanceToGo, routeMoveSpeedLimit, routeMoveIntegralLimit);
            }
            expectedVoltage.speedZ = SpeedW_contorll_limited(
                angleToGo, routeAngleSpeedLimit, routeAngleIntegralLimit);

            inTargetBand = (fabs(distanceToGo) < routeDistanceTolerance) && (fabs(carSpeed) < 5.0);
            if (inTargetBand) {
                if (routeInTargetStartMs == 0) routeInTargetStartMs = nowMs;
            } else {
                routeInTargetStartMs = 0;
            }
        }
    }

    if (routeForceDone &&
        routeInTargetStartMs != 0 &&
        (unsigned long)(nowMs - routeInTargetStartMs) >= routeHoldMs &&
        (unsigned long)(nowMs - routeStateStartMs) >= routeStateMinMs) {
        routeState = (routeState == 0) ? 1 : 0;
        routeInTargetStartMs = 0;
        routeStateStartMs = nowMs;
        routeForceStartMs = 0;
        routeForceDone = 0;
        expectedVoltage.speedX = 0;
        expectedVoltage.speedZ = 0;
        Car_clearIngral();
        routeBrakeOnly = true;
    }

    if (routeBrakeOnly) {
        Car_brake();
    } else {
        qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
        pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
        dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
        bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
    }
#elif MAIN_TASK
    HWT101_read();
    VL53L1X_read();
    Car_updateMotionInloop();
    updateStateMachine();
    sendUartStatus();
    //qm.setPwm(100);
    //pm.setPwm(100);
    //dm.setPwm(100);
    //bm.setPwm(100);
#elif FORWARD_HOLD_ANGLE_TEST
    static bool straightRunInitialized = false;
    if (!straightRunInitialized) {
        Car_clearDistanceAngleIntegral();
        straightRunInitialized = true;
    }
    const double testSpeedX = 20.0;
    const float targetAngle = -90.0f;
    float angleToGo = sensorData.fAngle - targetAngle;
    float distancetogo = sensorData.DistanceF - 20.0f;
    expectedVoltage.speedX = Speedv_contorll(distancetogo);
    expectedVoltage.speedZ = SpeedW_contorll(angleToGo);
    //qm.setPwm(180);
    //dm.setPwm(180);
    qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
    dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
#elif SPEED_MEASURE_TEST
    qm.setPwm(100);
    pm.setPwm(100);
    dm.setPwm(100);
    bm.setPwm(100);
    //qm.setspeed(20);
    //pm.setspeed(-20);
    //dm.setspeed(20);
    //bm.setspeed(-20);
#endif
#if PLANT_ACTUATION_TASK || PLANT_ACTUATION_TASK2
    sendTestUartStatus();
#endif
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        Serial.print("route:");
#if PLANT_ACTUATION_TASK || PLANT_ACTUATION_TASK2
        Serial.print(plantRouteStateDebug);
        Serial.print(" dGo:");
        Serial.print(plantDistanceToGoDebug);
        Serial.print(" v:");
        Serial.print(plantCarSpeedDebug);
        Serial.print(" freshF:");
        Serial.print(plantFrontLaserFreshDebug);
        Serial.print(" freshB:");
        Serial.print(plantBackLaserFreshDebug);
#else
        Serial.print("-");
#endif
        Serial.print(" distF filter: ");
        Serial.print(sensorData.DistanceF);
        Serial.print(" rawF:");
        Serial.print(sensorData.rawDistanceF);
        Serial.print(" readF:");
        Serial.print(vl53ReadCmF);
        Serial.print(" stF:");
        Serial.print(vl53StatusF);
        Serial.print(" ageF:");
        Serial.print(vl53LastValidMsF == 0 ? 9999UL : millis() - vl53LastValidMsF);
        Serial.print(" distB filter: ");
        Serial.print(sensorData.DistanceB);
        Serial.print(" rawB:");
        Serial.print(sensorData.rawDistanceB);
        Serial.print(" readB:");
        Serial.print(vl53ReadCmB);
        Serial.print(" stB:");
        Serial.print(vl53StatusB);
        Serial.print(" ageB:");
        Serial.print(vl53LastValidMsB == 0 ? 9999UL : millis() - vl53LastValidMsB);
        Serial.print("angel:");
        Serial.println(sensorData.fAngle);
        //Serial.print(" F status:");
        //Serial.print(vl53StatusF);
        //Serial.print(" F read:");
        //Serial.print(vl53ReadCmF);
        //Serial.print(" B status:");
        //Serial.print(vl53StatusB);
        //Serial.print(" B read:");
        //Serial.print(vl53ReadCmB);
        //Serial.print("angel:");
        //Serial.println(sensorData.fAngle);
        //double avgKpTerm = (fabs(qm.readKpTerm()) + fabs(pm.readKpTerm()) + fabs(dm.readKpTerm()) + fabs(bm.readKpTerm())) / 4.0;
        //double avgKiTerm = (fabs(qm.readKiTerm()) + fabs(pm.readKiTerm()) + fabs(dm.readKiTerm()) + fabs(bm.readKiTerm())) / 4.0;
        //double avgKdTerm = (fabs(qm.readKdTerm()) + fabs(pm.readKdTerm()) + fabs(dm.readKdTerm()) + fabs(bm.readKdTerm())) / 4.0;
        //double avgPwm = (fabs(qm.readpwm()) + fabs(pm.readpwm()) + fabs(dm.readpwm()) + fabs(bm.readpwm())) / 4.0;
        //Serial.print("Speedqm: ");
        //Serial.print(qm.readspeed());
        //Serial.print(" Speedpm: ");
        //Serial.print(pm.readspeed());
        //Serial.print(" Speeddm: ");
        //Serial.print(dm.readspeed());
        //Serial.print(" Speedbm: ");
        //Serial.println(bm.readspeed());
#if FORWARD_HOLD_ANGLE_TEST
        Serial.print(" err: ");
        Serial.print(angleToGo);

        //Serial.print(" rawGyro: ");
        //Serial.print(sensorData.rawFGyro);
        //Serial.print(" fGyro: ");
        //Serial.print(sensorData.fGyro);
        Serial.print(" P: ");
        Serial.print(angleKpTerm);
        Serial.print(" I: ");
        Serial.print(angleKiTerm);
        Serial.print(" D: ");
        Serial.print(angleKdTerm);
        Serial.print(" speedz: ");
        Serial.println(expectedVoltage.speedZ);
#endif

        //Serial.print("inner avgAbs P: ");
        //Serial.print(avgKpTerm);
        //Serial.print(" I: ");
        //Serial.print(avgKiTerm);
        //Serial.print(" D: ");
        //Serial.print(avgKdTerm);
        //Serial.print(" pwm: ");
        //Serial.print(avgPwm);
        //Serial.print(" speed: ");
        //Serial.println(qm.readspeed());
        //Serial.print(" gyro raw: ");
        //Serial.print(sensorData.rawFGyro);
        //Serial.print(" gyro filter: ");
        //Serial.print(sensorData.fGyro);
        //Serial.print(" angle: ");
        //Serial.println(sensorData.fAngle);
        //Serial.print("distF raw: ");
        //Serial.print(sensorData.rawDistanceF);
        //Serial.print(" distF filter: ");
        //Serial.println(sensorData.DistanceF);
    }
  a++;
  if(a>=1000){
    a=0;
    now_ = millis();
    //Serial.println();
    Serial.print("delta:");
    Serial.println((now_-pre_)/1000.0);
    pre_ = now_;
  }
}


static void Test_loop() {
}

static void sendTestUartStatus() {
    static unsigned long lastSendMs = 0;
    if (millis() - lastSendMs < 20) {
        return;
    }
    lastSendMs = millis();
    uint8_t buffer[6];
    float carSpeed = (pm.readspeedf() + dm.readspeedf() + qm.readspeedf() + bm.readspeedf()) / 4.0f;
    buffer[0] = testUartEnable;
    memcpy(&buffer[1], &carSpeed, 4);
    buffer[5] = 0xAA;
    Serial2.write(buffer, 6);
}

static float normalizeAngleDeg(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float angleErrorDeg(float angle, float target) {
    return normalizeAngleDeg(angle - target);
}

/*--------------------------------HWT101CT 闂勨偓閾昏桨鍗庢导鐘冲妳閸?---------------------------------*/
void HWT101_read(){
  while (Serial1.available())
  {
    //Serial.println("1");
    WitSerialDataIn(Serial1.read());
  }
  if (s_cDataUpdate) 
  {
    sensorData.rawFGyro = sReg[57] / 32768.0f * 2000.0f; 
    sensorData.fAngle = sReg[63] / 32768.0f * 180.0f; 
    if (s_cDataUpdate & GYRO_UPDATE)
    {
      /*Serial.print("GyroZ: ");
      Serial.print(fGyro, 1);
      Serial.print("\r\n");
      s_cDataUpdate &= ~GYRO_UPDATE;*/
    }
    if (s_cDataUpdate & ANGLE_UPDATE)
    {
      /*Serial.print("AngleZ: ");
      Serial.print(fAngle, 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ANGLE_UPDATE;*/
    }
    s_cDataUpdate = 0;
  }
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  //鏉╂瑩鍣风憴鍕暰娴滃棗绶欰rduino閻ㄥ嫪瑕嗛崣?閸欐垿鈧焦鏆熼幑顕嗙礉娑旂喎褰叉禒銉︽暭娑撳搫鍙炬禒鏍畱绾兛娆㈡稉鎻掑經
  Serial1.write(p_data, uiSize);
  //閹碘偓閺堝褰傞柅浣虹处鐎涙ü鑵戦惃鍕殶閹诡喖鍙忛柈銊ュ絺闁礁鍤崢璇叉倵閿涘奔绱堕幇鐔锋珤閹靛秳绱伴幍褑顢戦崥搴ｇ敾閻ㄥ嫭鎼锋担?
  Serial1.flush();
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{ 
  int i;
  //鏉╂瑩鍣锋稊瀣娴犮儴顩﹀顏嗗箚閺勵垰娲滄稉?
  //娴肩姴鍙嗛崙鑺ユ殶閻ㄥ増iReg娑撯偓閼割剟鍏橀弰鐤塜閿涘瓘X閿涘oll閹碘偓娴犮儵娓剁憰浣告儕閻滎垯绔存稉瀣€樼€规碍妲搁崥锕€鐡ㄩ崷鈩匷閸滃aw
  for (i = 0; i < uiRegNum; i++)
  {
    switch (uiReg)
    {
      case GZ: //婵″倹鐏夐弰鐥佹潪纾嬵潡闁喎瀹抽敍瀛廧鐎规矮绠熸稉?x39閿涘牆婀猂EG.h閺傚洣娆㈡稉顓ㄧ礆
        s_cDataUpdate |= GYRO_UPDATE; //閹跺_cDataUpdate閸欐﹢鍣洪惃鍕儑娴滃奔缍呯純?
        break;
      case Yaw: //婵″倹鐏夐弰鐥佹潪纾嬵潡鎼达讣绱漎aw鐎规矮绠熸稉?x3f
        s_cDataUpdate |= ANGLE_UPDATE; //閹跺_cDataUpdate閸欐﹢鍣洪惃鍕儑娑撳缍呯純?
        break;
      default: //婵″倹鐏夐柈鎴掔瑝閺?
        s_cDataUpdate |= READ_UPDATE; //閹跺_cDataUpdate閸欐﹢鍣洪惃鍕儑閸忣偂缍呯純?
        break;
    }
    uiReg++;
  }
}


static void Delayms(uint16_t ucMs)
{
  delay(ucMs);

}
// VL53L1X濠碘偓閸忓绁寸捄婵呯炊閹扮喎娅?
static bool VL53L1X_beginWithRetry(SFEVL53L1X& sensor, const char* name) {
  for (uint8_t attempt = 0; attempt < VL53L1X_INIT_RETRY_COUNT; ++attempt) {
    if (sensor.begin() == 0) {
      return true;
    }
    Serial.print(name);
    Serial.print(" begin retry ");
    Serial.println(attempt + 1);
    delay(100);
  }
  return false;
}

static bool VL53L1X_isValidDistanceCm(float distanceCm, uint8_t rangeStatus) {
  (void)rangeStatus;
  return distanceCm >= VL53L1X_MIN_VALID_CM &&
         distanceCm <= VL53L1X_MAX_VALID_CM;
}

static bool VL53L1X_isFresh(unsigned long lastValidMs) {
  return lastValidMs != 0UL &&
         (unsigned long)(millis() - lastValidMs) <= VL53L1X_STALE_TIMEOUT_MS;
}

void VL53L1X_read(){

#if USE_FRONT_LASER
  if (digitalRead(INT_PIN_F) == HIGH) {
    if (distanceSensor1.checkForDataReady()){
      uint8_t rangeStatus = distanceSensor1.getRangeStatus();
      float distanceCm = distanceSensor1.getDistance() / 10.0f;
      vl53StatusF = rangeStatus;
      vl53ReadCmF = distanceCm;
      distanceSensor1.clearInterrupt();
      if (VL53L1X_isValidDistanceCm(distanceCm, rangeStatus)) {
        sensorData.rawDistanceF = distanceCm;
        vl53LastValidMsF = millis();
      }
      //Serial.print("0x00:");
      //Serial.println(sensorData.DistanceF);
    }
  }
#endif
#if USE_BACK_LASER
  if (digitalRead(INT_PIN_B) == HIGH) {
    if (distanceSensor2.checkForDataReady()){
      uint8_t rangeStatus = distanceSensor2.getRangeStatus();
      float distanceCm = distanceSensor2.getDistance() / 10.0f;
      vl53StatusB = rangeStatus;
      vl53ReadCmB = distanceCm;
      distanceSensor2.clearInterrupt();
      if (VL53L1X_isValidDistanceCm(distanceCm, rangeStatus)) {
        sensorData.rawDistanceB = distanceCm;
        vl53LastValidMsB = millis();
      }
      //Serial.print("0x01:");
      //.Serial.println(sensorData.DistanceB);
    }
  }
#endif
}

