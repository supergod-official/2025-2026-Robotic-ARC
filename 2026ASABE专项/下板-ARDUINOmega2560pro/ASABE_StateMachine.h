#pragma once

#include <Arduino.h>
#include "PCencoder_.h"
#include <math.h>

enum ActionType {
    ACTION_MOVE_TO_DISTANCE,
    ACTION_MOVE_BLIND_ENCODER,
    ACTION_MOVE_TIMED,
    ACTION_TURN_TO_ANGLE,
    ACTION_STOP
};

enum DistanceSource {
    DISTANCE_FRONT,
    DISTANCE_BACK
};

struct StateTask {
    ActionType type;
    float targetDistance;
    float targetAngle;
    float distanceTolerance;
    float angleTolerance;
    unsigned long minDuration;
    bool workSign;
    DistanceSource distanceSource;
    float moveSpeed;
};

struct pidControll {
    float kp;
    float ki;
    float kd;
    float kp_w;
    float ki_w;
    float kd_w;
    float pulse_to_distance;
    float deadvoltage;
    float integralmax;
    float integralmaxw;
};

struct SensorData {
    float DistanceF;
    float DistanceB;
    float fAngle;
    float fGyro;
    float rawDistanceF;
    float rawDistanceB;
    float rawFGyro;
};

struct ExpectedVoltage {
    double speedX;
    double speedY;
    double speedZ;
};

extern StateTask currentTask;
extern bool taskRunning;
extern uint8_t taskCount;
extern uint8_t currentTaskIndex;
extern uint8_t inRangeCount;
extern unsigned long stateStartMs;
extern SensorData sensorData;
extern ExpectedVoltage expectedVoltage;
extern PCencoder qm;
extern PCencoder pm;
extern PCencoder dm;
extern PCencoder bm;
extern pidControll m_pid;
extern pidControll car_pid;
extern double angleKpTerm;
extern double angleKiTerm;
extern double angleKdTerm;
extern double distanceKpTerm;
extern double distanceKiTerm;
extern double distanceKdTerm;

void enqueueTask(const StateTask& task);
bool startNextTask();
void buildDefaultTaskQueue();
void buildNavigationTaskQueue();
void updateStateMachine();
void Car_updateMotionInloop();
void timerCallback();
void Car_brake();
void Car_clearIngral();
void Car_clearDistanceIntegral();
void Car_clearDistanceAngleIntegral();
double Speedv_contorll(float distanceToGo);
double SpeedW_contorll(float angleToGo);
double Speedv_contorll_limited(float distanceToGo, double speedLimit, double integralLimit);
double SpeedW_contorll_limited(float angleToGo, double speedLimit, double integralLimit);
void Car_distanceContorll(float distancetogo, float mincontrolldistance, float errordistance);
void Car_angleContorll(float angletogo, float mincontrollangle, float errorangle);
void sendUartStatus();
