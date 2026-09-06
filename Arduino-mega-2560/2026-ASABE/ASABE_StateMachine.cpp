#include "ASABE_StateMachine.h"


const uint8_t TASK_QUEUE_SIZE = 32;
StateTask taskQueue[TASK_QUEUE_SIZE];
uint8_t taskCount = 0;
uint8_t currentTaskIndex = 0;
bool taskRunning = false;
StateTask currentTask;
unsigned long inTargetBandStartMs = 0;
unsigned long stateStartMs = 0;
const unsigned long IN_TARGET_HOLD_MS = 500UL;
// 在全局变量区域添加
long long lastSendTime = 0,bootStartTime = 0;
bool bootTimeInitialized = false;
double distanceIntegral = 0.0;
unsigned long distancePidLastMicros = 0;
double angleIntegral = 0.0;
unsigned long anglePidLastMicros = 0;
double angleKpTerm = 0.0;
double angleKiTerm = 0.0;
double angleKdTerm = 0.0;
double distanceKpTerm = 0.0;
double distanceKiTerm = 0.0;
double distanceKdTerm = 0.0;
PCencoder qm;
PCencoder pm;
PCencoder dm;
PCencoder bm;
static long long blindEncoderStartQ = 0;
static long long blindEncoderStartP = 0;
static long long blindEncoderStartD = 0;
static long long blindEncoderStartB = 0;

ExpectedVoltage expectedVoltage = {0.0f, 0.0f, 0.0f};
SensorData sensorData = {
    100.0f,   // DistanceF
    0.0f,     // DistanceB

    -90.0f,   // fAngle
    0.0f,     // fGyro
    100.0f,   // rawDistanceF
    0.0f,     // rawDistanceB
    0.0f      // rawFGyro
};
// 外层pid
pidControll car_pid = {
    //距离pid
    0.9f,     // kp
    4.0f,     // ki
    -0.2f,     // kd
    //角度pid
    0.8f,     // kp_w
    3.0f,    // ki_w
    0.10f,   // kd_w for gyro damping
    0.0f,     // 
    0.0f,     // 
    1.0f,     // integralmaxv
    1.0f     // 角度积分上限
};
// 内层pid
pidControll m_pid = {
    4.0f,     // kp         5
    25.0f,     // ki        5
    -0.2f,     // kd
    0.0f,     // kp_w
    0.0f,     // ki_w
    0.0f,     // kd_w
    0.007933f, // pulse_to_distance
    20.0f,    // deadvoltage
    8.0f     // integralmax 15
};
/*
struct pidControll {
    float kp;
    float ki;
    float kd;
    float kp_w;
    float ki_w;
    float kd_w;
    float integral;
    float integral_w;
    float integralmax;
    float integralmax_w;
    float pulse_to_distance;
};
struct StateTask {
    ActionType type;
    float targetDistance;
    float targetAngle;
    float distanceTolerance;
    float angleTolerance;
    unsigned long minDuration;
};


*/
static void captureBlindEncoderStart() {
    blindEncoderStartQ = qm.readencoder();
    blindEncoderStartP = pm.readencoder();
    blindEncoderStartD = dm.readencoder();
    blindEncoderStartB = bm.readencoder();
}

bool startNextTask() {
    if (currentTaskIndex >= taskCount) {
        taskRunning = false;
        return false;
    }

    currentTask = taskQueue[currentTaskIndex++];
    captureBlindEncoderStart();
    taskRunning = true;
    inTargetBandStartMs = 0;
    stateStartMs = millis();
    distanceIntegral = 0.0;
    distancePidLastMicros = 0;
    angleIntegral = 0.0;
    anglePidLastMicros = 0;
    return true;
}

void enqueueTask(const StateTask& task) {
    if (taskCount < TASK_QUEUE_SIZE) {
        taskQueue[taskCount++] = task;
    }
}

void buildDefaultTaskQueue() {
    const float DEFAULT_DISTANCE_TOLERANCE_CM = 3.0f;
    const float DEFAULT_ANGLE_TOLERANCE_DEG = 5.0f;
    const unsigned long DEFAULT_MOVE_MIN_DURATION_MS = 1000UL;
    const unsigned long DEFAULT_TURN_MIN_DURATION_MS = 100UL;
    const unsigned long STOP_BRAKE_MS = 500UL;

    const float DEFAULT_WORK_MOVE_SPEED = 8.0f;
    const float DEFAULT_MOVE_SPEED = 8.0f;
    const float distancectl = 8.0f;
    // 向前直线 1 工作
    StateTask aheadTask = {
        ACTION_MOVE_TO_DISTANCE,
        distancectl,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    // Forward line 2 long distance
    StateTask aheadLongTask = {
        ACTION_MOVE_TO_DISTANCE,
        55.72f,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    // 向后直线 0 工作
    StateTask backTask = {
        ACTION_MOVE_TO_DISTANCE,
        distancectl,
        90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    StateTask backTask1 = {
        ACTION_MOVE_TO_DISTANCE,
        10.0f,
        90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };

    // Move left section
    // Back laser first, front laser after boundary
    // Back laser distance control
    StateTask left1Task = {
        ACTION_MOVE_TO_DISTANCE,
        65.0f,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_BACK,
        DEFAULT_MOVE_SPEED
    };
    // Front/back laser boundary
    StateTask left2Task = {
        ACTION_MOVE_TO_DISTANCE,
        51.52f,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask left3Task = {
        ACTION_MOVE_TO_DISTANCE,
        distancectl,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask timedForward0 = {
        ACTION_MOVE_TIMED,
        0.0f,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        15000UL,  // timedForward0 duration: 10.5 s
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    StateTask timedForward1 = {
        ACTION_MOVE_TIMED,
        0.0f,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        10500UL,   // timedForward1 duration: 10 s
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask blindForward1 = {
        ACTION_MOVE_BLIND_ENCODER,
        84.0f,
        0.0f,
        1.5,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask timedForward2 = {
        ACTION_MOVE_TIMED,
        0.0f,
        90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        15000UL,  // timedForward2 duration: 15 s
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    StateTask timedForward3 = {
        ACTION_MOVE_TIMED,
        0.0f,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        9000UL,   // timedForward3 duration: 10.0 s
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask blindForward2 = {
        ACTION_MOVE_BLIND_ENCODER,
        86.0f,
        0.0f,
        1.5,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        DEFAULT_MOVE_SPEED
    };
    StateTask timedForward4 = {
        ACTION_MOVE_TIMED,
        0.0f,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        18000UL,  // timedForward4 duration: 15 s
        1,
        DISTANCE_FRONT,
        DEFAULT_WORK_MOVE_SPEED
    };
    StateTask timedBack0 = {
        ACTION_MOVE_TIMED,
        0.0f,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        18000UL,  // timedBack0 duration: 18 s
        0,
        DISTANCE_BACK,
        -DEFAULT_WORK_MOVE_SPEED
    };
    StateTask AheadTask0 = {
        ACTION_MOVE_TO_DISTANCE,
        distancectl,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        DEFAULT_ANGLE_TOLERANCE_DEG,
        DEFAULT_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_BACK,
        DEFAULT_WORK_MOVE_SPEED
    };
    StateTask turnAhead = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        -90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        3.0f,
        DEFAULT_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    StateTask turnBack = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        90.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        3.0f,
        DEFAULT_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    StateTask turnLeft = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        0.0f,
        DEFAULT_DISTANCE_TOLERANCE_CM,
        3.0f,
        DEFAULT_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    StateTask stopTask = {
        ACTION_STOP,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0UL,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    StateTask stopTask0 = {
        ACTION_STOP,
        0.0f,
        -90.0f,
        0.0f,
        0.0f,
        STOP_BRAKE_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    enqueueTask(timedForward0); // work zone: timed straight forward, hold -90 deg.
    enqueueTask(aheadTask);     // work zone: front laser trims to 10 cm at -90 deg.
    enqueueTask(turnLeft);      // non-work zone: turn to 0 deg.

    //enqueueTask(timedForward1); // non-work zone: timed straight move at 0 deg.
    //enqueueTask(left1Task);   // disabled: back laser trims side distance to 65 cm.
    // blind 1: encoder forward 85 cm, same exit rule as straight distance control.
    enqueueTask(blindForward1); // non-work zone: blind encoder forward 85 cm, hold 0 deg.
    //1 盲走85cm 退出和直线一致

    enqueueTask(turnBack);      // non-work zone: turn to 90 deg.

    enqueueTask(timedForward2); // work zone: timed straight forward, hold 90 deg.
    enqueueTask(backTask);      // work zone: front laser trims to 10 cm at 90 deg.
    enqueueTask(turnLeft);      // non-work zone: turn to 0 deg.

    //enqueueTask(timedForward3); // non-work zone: timed straight move at 0 deg.
    // blind 2: encoder forward 80 cm, same exit rule as straight distance control.
    enqueueTask(blindForward2); // non-work zone: blind encoder forward 80 cm, hold 0 deg.
    //2 盲走80cm 退出和直线一致
    //enqueueTask(left2Task);     // non-work zone: front laser trims side distance to 42.72 cm.
    enqueueTask(turnAhead);     // non-work zone: turn to -90 deg.

    enqueueTask(timedForward4); // work zone: timed straight forward, hold -90 deg.
    enqueueTask(stopTask0);     // non-work zone: brake for 500 ms.

    enqueueTask(timedBack0);    // non-work zone: timed straight backward, hold -90 deg.
    enqueueTask(AheadTask0);    // non-work zone: keep backward direction, back laser trims to 10 cm.

    enqueueTask(turnLeft);      // non-work zone: turn to 0 deg.
    enqueueTask(left3Task);     // non-work zone: front laser trims side distance to 10 cm.

    enqueueTask(stopTask);      // non-work zone: final stop.
}

void buildNavigationTaskQueue() {
    const float NAV_AHEAD_DISTANCE_CM = 8.0f;
    const float NAV_DISTANCE_TOLERANCE_CM = 2.0f;
    const float NAV_ANGLE_TOLERANCE_DEG = 3.0f;
    const unsigned long NAV_MOVE_MIN_DURATION_MS = 1000UL;
    const unsigned long NAV_TURN_MIN_DURATION_MS = 100UL;

    const float NAV_WORK_MOVE_SPEED = 7.5f;
    const float NAV_RIGHT_SPEED = 7.5f;
    // 直线前进
    StateTask navAhead = {
        ACTION_MOVE_TO_DISTANCE,
        NAV_AHEAD_DISTANCE_CM,
        90.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        10000,
        1,
        DISTANCE_FRONT,
        NAV_WORK_MOVE_SPEED
    };
    // 后退
    StateTask navBack = {
        ACTION_MOVE_TO_DISTANCE,
        NAV_AHEAD_DISTANCE_CM,
        -90.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        10000,
        1,
        DISTANCE_FRONT,
        NAV_WORK_MOVE_SPEED
    };
    // Turn to 0 deg
    StateTask navTurnRight = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    // Turn to -90 deg
    StateTask navTurnBack = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        -90.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    StateTask navTurnAhead = {
        ACTION_TURN_TO_ANGLE,
        0.0f,
        90.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_TURN_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        0.0f
    };
    // 向右
    StateTask navRight = {
        ACTION_MOVE_TO_DISTANCE,
        40.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_BACK,
        NAV_RIGHT_SPEED
    };
    // 向右移动1
    StateTask navRight1 = {
        ACTION_MOVE_TIMED,
        55.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        5800UL,
        0,
        DISTANCE_BACK,
        NAV_RIGHT_SPEED
    };
    // back 2  ahead 3
    // 向右移动2
    StateTask navRight2 = {
        ACTION_MOVE_TIMED,
        85.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        5800UL,
        0,
        DISTANCE_FRONT,
        NAV_RIGHT_SPEED
    };
    // 向右移动3
    StateTask navRight3 = {
        ACTION_MOVE_TO_DISTANCE,
        43.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        NAV_RIGHT_SPEED
    };
    // 向右移动4
    StateTask navRight4 = {
        ACTION_MOVE_TO_DISTANCE,
        10.0f,
        0.0f,
        NAV_DISTANCE_TOLERANCE_CM,
        NAV_ANGLE_TOLERANCE_DEG,
        NAV_MOVE_MIN_DURATION_MS,
        0,
        DISTANCE_FRONT,
        NAV_RIGHT_SPEED
    };
    StateTask navStop = {
        ACTION_STOP,
        0.0f,
        90.0f,
        0.0f,
        0.0f,
        0UL,
        0,
        DISTANCE_FRONT,
        0.0f
    };

    enqueueTask(navAhead);     // work zone
    enqueueTask(navTurnRight); // non-work zone
    enqueueTask(navRight);     // non-work zone
    enqueueTask(navTurnBack);  // non-work zone
    enqueueTask(navBack);      // work zone
    enqueueTask(navTurnRight); // non-work zone
    enqueueTask(navRight1);    // non-work zone
    enqueueTask(navTurnAhead); // non-work zone
    enqueueTask(navAhead);     // work zone
    enqueueTask(navTurnRight); // non-work zone
    enqueueTask(navRight2);    // non-work zone
    enqueueTask(navTurnBack);  // non-work zone
    enqueueTask(navBack);      // work zone
    enqueueTask(navTurnRight); // non-work zone
    enqueueTask(navRight3);    // non-work zone
    enqueueTask(navTurnAhead); // non-work zone
    enqueueTask(navAhead);     // work zone
    enqueueTask(navTurnRight); // non-work zone
    enqueueTask(navRight4);    // non-work zone
    enqueueTask(navTurnBack);  // non-work zone
    enqueueTask(navBack);      // work zone
    enqueueTask(navStop);      // non-work zone
}
void timerCallback() {
    qm.requestMotionSample();
    pm.requestMotionSample();
    dm.requestMotionSample();
    bm.requestMotionSample();
}

void Car_updateMotionInloop() {
    qm.updateMotionInloop();
    pm.updateMotionInloop();
    dm.updateMotionInloop();
    bm.updateMotionInloop();

    qm.filterMotionInloop();
    pm.filterMotionInloop();
    dm.filterMotionInloop();
    bm.filterMotionInloop();

    const float gyroFilterAlpha = 0.06f;
    const float distanceFilterAlpha = 0.10f;
    static bool sensorFilterReady = false;

    if (!sensorFilterReady) {
        sensorData.fGyro = sensorData.rawFGyro;
        sensorData.DistanceF = sensorData.rawDistanceF;
        sensorData.DistanceB = sensorData.rawDistanceB;
        sensorFilterReady = true;
    } else {
        sensorData.fGyro =
            (1.0f - gyroFilterAlpha) * sensorData.fGyro +
            gyroFilterAlpha * sensorData.rawFGyro;
        sensorData.DistanceF =
            (1.0f - distanceFilterAlpha) * sensorData.DistanceF +
            distanceFilterAlpha * sensorData.rawDistanceF;
        sensorData.DistanceB =
            (1.0f - distanceFilterAlpha) * sensorData.DistanceB +
            distanceFilterAlpha * sensorData.rawDistanceB;
    }
}

void Car_brake() {
    qm.brake();
    pm.brake();
    dm.brake();
    bm.brake();
}

void Car_clearIngral() {
    qm.clearIntegral();
    pm.clearIntegral();
    dm.clearIntegral();
    bm.clearIntegral();
    Car_clearDistanceAngleIntegral();
}

void Car_clearDistanceIntegral() {
    distanceIntegral = 0.0;
    distancePidLastMicros = 0;
}

void Car_clearDistanceAngleIntegral() {
    Car_clearDistanceIntegral();
    angleIntegral = 0.0;
    anglePidLastMicros = 0;
}

double clampDouble(double value, double minValue, double maxValue) {
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

static float normalizeAngleErrorDeg(float angleToGo) {
    while (angleToGo > 180.0f) {
        angleToGo -= 360.0f;
    }
    while (angleToGo <= -180.0f) {
        angleToGo += 360.0f;
    }
    return angleToGo;
}

static double readTaskMoveSpeedLimit() {
    double speedLimit = fabs(currentTask.moveSpeed);
    if (speedLimit < 0.1) {
        speedLimit = 20.0;
    }
    return speedLimit;
}

double Speedv_contorll_limited(float distanceToGo, double speedLimit, double integralLimit) {
    const double dtMin = 0.0001;
    const double dtMax = 0.005;
    const float integralClearError = 1.0f;
    speedLimit = fabs(speedLimit);
    integralLimit = fabs(integralLimit);

    unsigned long nowMicros = micros();
    double dt = dtMax;

    if (distancePidLastMicros != 0) {
        dt = (double)(nowMicros - distancePidLastMicros) / 1000000.0;
        dt = clampDouble(dt, dtMin, dtMax);
    }
    distancePidLastMicros = nowMicros;

    if (fabs(distanceToGo) < integralClearError) {
        distanceIntegral = 0.0;
    } else {
        distanceIntegral += (double)distanceToGo * dt;
        distanceIntegral = clampDouble(
            distanceIntegral, -integralLimit, integralLimit);
    }

    double speedcar = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;
    distanceKpTerm = car_pid.kp * distanceToGo;
    distanceKiTerm = car_pid.ki * distanceIntegral;
    distanceKdTerm = car_pid.kd * speedcar;
    return clampDouble(distanceKpTerm + distanceKiTerm + distanceKdTerm, -speedLimit, speedLimit);
}

double Speedv_contorll(float distanceToGo) {
    return Speedv_contorll_limited(distanceToGo, readTaskMoveSpeedLimit(), car_pid.integralmax);
}

static double SpeedW_contorll_limited_impl(
    float angleToGo, double speedLimit, double integralLimit, bool clearIntegralNearTarget) {
    const double dtMin = 0.0001;
    const double dtMax = 0.005;
    const float integralClearError = 1.0f;
    speedLimit = fabs(speedLimit);
    integralLimit = fabs(integralLimit);

    unsigned long nowMicros = micros();
    double dt = dtMax;

    if (anglePidLastMicros != 0) {
        dt = (double)(nowMicros - anglePidLastMicros) / 1000000.0;
        dt = clampDouble(dt, dtMin, dtMax);
    }
    anglePidLastMicros = nowMicros;

    if (clearIntegralNearTarget && fabs(angleToGo) < integralClearError) {
        angleIntegral = 0.0;
    } else {
        angleIntegral += (double)angleToGo * dt;
        angleIntegral = clampDouble(
            angleIntegral, -integralLimit, integralLimit);
    }
    angleKpTerm = car_pid.kp_w * angleToGo;
    angleKiTerm = car_pid.ki_w * angleIntegral;
    angleKdTerm = car_pid.kd_w * sensorData.fGyro;

    return clampDouble(angleKpTerm + angleKiTerm + angleKdTerm, -speedLimit, speedLimit);
}

double SpeedW_contorll_limited(float angleToGo, double speedLimit, double integralLimit) {
    return SpeedW_contorll_limited_impl(angleToGo, speedLimit, integralLimit, true);
}

static double SpeedW_contorll_limited_keep_integral(
    float angleToGo, double speedLimit, double integralLimit) {
    return SpeedW_contorll_limited_impl(angleToGo, speedLimit, integralLimit, false);
}

double SpeedW_contorll(float angleToGo) {
    return SpeedW_contorll_limited(angleToGo, 8, car_pid.integralmaxw);
}

static double SpeedW_contorll_keep_integral(float angleToGo) {
    return SpeedW_contorll_limited_keep_integral(angleToGo, 8, car_pid.integralmaxw);
}

void Car_distanceContorll(float distancetogo, float mincontrolldistance, float errordistance) {
    float angleToGo = normalizeAngleErrorDeg(sensorData.fAngle - currentTask.targetAngle);
    double speedLimit = readTaskMoveSpeedLimit();

    if (distancetogo > mincontrolldistance) {
        expectedVoltage.speedX = speedLimit;
        Car_clearDistanceIntegral();
    } else if (distancetogo < -mincontrolldistance) {
        expectedVoltage.speedX = -speedLimit;
        Car_clearDistanceIntegral();

    } else if (fabs(distancetogo) < errordistance) {
        if (fabs(angleToGo) < 2.0f) {
            Car_brake();
            return;
        }
        expectedVoltage.speedX = 0;
    } else {
        expectedVoltage.speedX = Speedv_contorll(distancetogo);
    }
    expectedVoltage.speedZ = SpeedW_contorll(angleToGo);
    qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
    dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
}

void Car_angleContorll(float angletogo, float mincontrollangle, float errorangle) {
    if (angletogo > mincontrollangle) {
        expectedVoltage.speedZ = 7.0;
        Car_clearDistanceAngleIntegral();
    } else if (angletogo < -mincontrollangle) {
        expectedVoltage.speedZ = -7.0;
        Car_clearDistanceAngleIntegral();
    } else if (fabs(angletogo) < errorangle) {
        Car_brake();
        Car_clearIngral();
        return;
    } else {
        expectedVoltage.speedZ = SpeedW_contorll(angletogo);
    }

    expectedVoltage.speedX = 0;
    qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
    dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
    bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
}


static bool shouldSendWorkEnable() {
    if (taskRunning) {
        return currentTask.workSign == 1;
    }

    return currentTask.workSign == 1 &&
           currentTaskIndex < taskCount &&
           taskQueue[currentTaskIndex].workSign == 1;
}


void sendUartStatus() {
    if (!bootTimeInitialized) {
        lastSendTime = millis();
        bootStartTime = millis();
        bootTimeInitialized = true;
        return;
    }
    // Do not send in the first 2 seconds after startup

    if (millis() - lastSendTime >= 50) {
        lastSendTime = millis();
        uint8_t buffer[6];
        float carSpeed = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;
        
        // 判断是否在直线状态：当前任务是移动且角度接近目标角度
        bool isStraight = shouldSendWorkEnable();
                        
        //Serial.print("cartype:");
        //Serial.println(isStraight);
        buffer[0] = isStraight;
        memcpy(&buffer[1], &carSpeed, 4);
        buffer[5] = 0xAA;
        
        Serial2.write(buffer, 6);
    }
}

static float readTaskDistance(const StateTask& task) {
    return task.distanceSource == DISTANCE_BACK ? sensorData.DistanceB : sensorData.DistanceF;
}

static float readTaskDistanceError(const StateTask& task) {
    float distance = readTaskDistance(task);
    if (task.distanceSource == DISTANCE_BACK) {
        return task.targetDistance - distance;
    }
    return distance - task.targetDistance;
}

static float readBlindEncoderDistanceError(const StateTask& task) {
    const float cmPerPulse = m_pid.pulse_to_distance;
    if (fabs(cmPerPulse) < 0.000001f) {
        return 0.0f;
    }

    float targetEncoderCount = task.targetDistance / cmPerPulse;
    float encoderDeltaAverage =
        ((qm.readencoder() - blindEncoderStartQ) +
         (pm.readencoder() - blindEncoderStartP) +
         (dm.readencoder() - blindEncoderStartD) +
         (bm.readencoder() - blindEncoderStartB)) / 4.0f;
    float encoderToGo = targetEncoderCount - encoderDeltaAverage;

    return encoderToGo * cmPerPulse;
}

void updateStateMachine() {
    if (!taskRunning) {
        if (!startNextTask()) {
            Car_brake();
            return;
        }
    }

    switch (currentTask.type) {
        case ACTION_MOVE_TO_DISTANCE: {
            float distanceToGo = readTaskDistanceError(currentTask);
            float carspeed = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;

            Car_distanceContorll(distanceToGo, 20, 1.5);

            bool inTargetBand = fabs(distanceToGo) < currentTask.distanceTolerance;
            if (inTargetBand) {
                if (inTargetBandStartMs == 0) {
                    inTargetBandStartMs = millis();
                }
            } else {
                inTargetBandStartMs = 0;
            }

            if (inTargetBandStartMs != 0 &&
                (millis() - inTargetBandStartMs) >= IN_TARGET_HOLD_MS &&
                millis() - stateStartMs > currentTask.minDuration &&
                fabs(carspeed) < 1.0) {
                taskRunning = false;
            }
            break;
        }

        case ACTION_MOVE_BLIND_ENCODER: {
            float distanceToGo = readBlindEncoderDistanceError(currentTask);
            float carspeed = (pm.readspeed() + dm.readspeed() + qm.readspeed() + bm.readspeed()) / 4.0f;

            Car_distanceContorll(distanceToGo, 20, 1);

            bool inTargetBand = fabs(distanceToGo) < currentTask.distanceTolerance;
            if (inTargetBand) {
                if (inTargetBandStartMs == 0) {
                    inTargetBandStartMs = millis();
                }
            } else {
                inTargetBandStartMs = 0;
            }

            if (inTargetBandStartMs != 0 &&
                (millis() - inTargetBandStartMs) >= IN_TARGET_HOLD_MS &&
                millis() - stateStartMs > currentTask.minDuration &&
                fabs(carspeed) < 1.0) {
                taskRunning = false;
            }
            break;
        }

        case ACTION_MOVE_TIMED: {
            float angleToGo = normalizeAngleErrorDeg(sensorData.fAngle - currentTask.targetAngle);
            expectedVoltage.speedX = currentTask.moveSpeed;
            expectedVoltage.speedZ = SpeedW_contorll(angleToGo);
            qm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
            pm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);
            dm.setspeed(expectedVoltage.speedX + expectedVoltage.speedZ);
            bm.setspeed(expectedVoltage.speedX - expectedVoltage.speedZ);

            if (millis() - stateStartMs >= currentTask.minDuration) {
                Car_brake();
                Car_clearIngral();
                taskRunning = false;
            }
            break;
        }

        case ACTION_TURN_TO_ANGLE: {
            float angleToGo = normalizeAngleErrorDeg(sensorData.fAngle - currentTask.targetAngle);
            float carspeedw = (qm.readspeed() - pm.readspeed() + dm.readspeed() - bm.readspeed()) / 4.0f;

            Car_angleContorll(angleToGo, 20, 0.8);

            bool inTargetBand = fabs(angleToGo) < currentTask.angleTolerance;
            if (inTargetBand) {
                if (inTargetBandStartMs == 0) {
                    inTargetBandStartMs = millis();
                }
            } else {
                inTargetBandStartMs = 0;
            }

            if (inTargetBandStartMs != 0 &&
                (millis() - inTargetBandStartMs) >= IN_TARGET_HOLD_MS &&
                millis() - stateStartMs > currentTask.minDuration &&
                fabs(carspeedw) < 2) {
                taskRunning = false;
            }
            break;
        }

        case ACTION_STOP: {
            Car_brake();
            Car_clearIngral();
            if (millis() - stateStartMs >= currentTask.minDuration) {
                taskRunning = false;
            }
            break;
        }
    }
}
