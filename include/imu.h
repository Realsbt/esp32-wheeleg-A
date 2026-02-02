#ifndef IMU_H
#define IMU_H
#include <Arduino.h>

typedef struct {
    float yaw, pitch, roll;
    float yawSpd, pitchSpd, rollSpd;
    float zAccel;
} IMUData;

extern IMUData imuData;

void IMU_Init();
void IMU_CalibrateAndSaveOffsets();
bool IMU_LoadOffsets();
void IMU_ClearOffsets();
void IMU_HandleSerialCommand();

#endif
