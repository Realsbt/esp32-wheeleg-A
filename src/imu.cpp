#include "imu.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_task_wdt.h>
#include <Preferences.h>

MPU6050 mpu;
IMUData imuData;
static Preferences preferences;

void IMU_CalibrateAndSaveOffsets() {
    Serial.println("IMU自动校准中，请保持静止...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    preferences.begin("imu_offset", false);
    preferences.putInt("xAcc", mpu.getXAccelOffset());
    preferences.putInt("yAcc", mpu.getYAccelOffset());
    preferences.putInt("zAcc", mpu.getZAccelOffset());
    preferences.putInt("xGyro", mpu.getXGyroOffset());
    preferences.putInt("yGyro", mpu.getYGyroOffset());
    preferences.putInt("zGyro", mpu.getZGyroOffset());
    preferences.end();
    Serial.println("IMU偏置参数已保存");
}

bool IMU_LoadOffsets() {
    preferences.begin("imu_offset", true);
    if (preferences.isKey("xAcc")) {
        mpu.setXAccelOffset(preferences.getInt("xAcc"));
        mpu.setYAccelOffset(preferences.getInt("yAcc"));
        mpu.setZAccelOffset(preferences.getInt("zAcc"));
        mpu.setXGyroOffset(preferences.getInt("xGyro"));
        mpu.setYGyroOffset(preferences.getInt("yGyro"));
        mpu.setZGyroOffset(preferences.getInt("zGyro"));
        preferences.end();
        Serial.println("IMU偏置参数已加载");
        return true;
    }
    preferences.end();
    return false;
}

void IMU_ClearOffsets() {
    preferences.begin("imu_offset", false);
    preferences.clear();
    preferences.end();
    Serial.println("IMU偏置参数已清除");
}


void IMU_Task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t fifoBuffer[64];
    int16_t yawRound = 0;
    float lastYaw = 0;
    while (1)
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
            int16_t gyroData[3];
            mpu.getRotation(&gyroData[0], &gyroData[1], &gyroData[2]);
            imuData.rollSpd = gyroData[0] / 16.4f * M_PI / 180.0f;  
            imuData.pitchSpd = gyroData[1] / 16.4f * M_PI / 180.0f;                                            
            imuData.yawSpd = gyroData[2] / 16.4f * M_PI / 180.0f;   
            float ypr[3];
            Quaternion q;
            VectorFloat gravity;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float yaw = -ypr[0];
            imuData.pitch = -ypr[1];
            imuData.roll = ypr[2];
            if (yaw - lastYaw > M_PI)
                yawRound--;
            else if (yaw - lastYaw < -M_PI)
                yawRound++;
            lastYaw = yaw;
            imuData.yaw = yaw + yawRound * 2 * M_PI;
            VectorInt16 rawAccel;
            mpu.dmpGetAccel(&rawAccel, fifoBuffer);
            VectorInt16 accel;
            mpu.dmpGetLinearAccel(&accel, &rawAccel, &gravity);
            imuData.zAccel = accel.z / 8192.0f * 9.8f;
        }
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

void IMU_Init()
{
    Wire.begin(21, 22);
    Wire.setClock(400000);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("Address: 0x" + String(mpu.getDeviceID(), HEX));
    mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);
    while(mpu.dmpInitialize() != 0);
    if (!IMU_LoadOffsets()) {
        IMU_CalibrateAndSaveOffsets();
    }
    mpu.setDMPEnabled(true);
    xTaskCreate(IMU_Task, "IMU_Task", 2048, NULL, 4, NULL);
}

