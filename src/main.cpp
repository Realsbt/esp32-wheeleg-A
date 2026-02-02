#include <Arduino.h>
#include "serial.h"
#include "can.h"
#include "imu.h"
#include "motor.h"
#include "ble.h"
#include "legs.h"
#include "ctrl.h"
#include "adc.h"

void setup()
{
//优先级1
Serial_Init();
//优先级4
CAN_Init();
IMU_Init();
Motor_InitAll();
// 优先级2
Legs_Init();
//状态更新优先级3    姿态控制优先级为1 
Ctrl_Init();    //注释掉电机不转了

//蓝牙遥控器捏
BLE_Init();  // 现在会自动启动Xbox处理任务
}

void loop()
{
    // Xbox手柄处理现在由FreeRTOS任务BLE_TestTask处理
    // 这里可以添加其他需要在主循环中执行的代码
    delay(100);
}