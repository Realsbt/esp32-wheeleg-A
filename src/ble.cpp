
#include <Arduino.h>
#include "ctrl.h"
#include "serial.h"
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

// Xbox手柄控制参数
#define MAX_SPEED           (0.8F)
#define MAX_YAWSPEED        (4.5F)    // 提高转向速度上限（配合yaw PID外环限幅6）
#define MAX_ROLL_OFFSET     (0.30F)   // rad，加大ROLL偏移幅度（约22.9度）
#define MAX_LEG_LENGTH      (0.115F)
#define MIN_LEG_LENGTH      (0.058F)
#define DEAD_ZONE           (3000)
// X键切换的最小间隔（毫秒），用于去抖与防止连发
#define TOGGLE_DEBOUNCE_MS  (300U)

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern void Ctrl_StandupPrepareTask(void *arg);
extern void Ctrl_JumpPrepareTask(void *arg);

// Xbox手柄实例 - 使用空MAC地址以启用自动扫描模式（通过设备名称"Xbox Wireless Controller"连接）
// 如果需要指定MAC地址，可以传入MAC地址字符串，例如: XboxSeriesXControllerESP32_asukiaaa::Core xboxController("57:5B:5D:FE:B3:C3");
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("");

// 摇杆死区处理
// 使用与手柄原始数据匹配的合理死区
const int deadZone = DEAD_ZONE;
const int deadZoneRoll = DEAD_ZONE / 10;

// 按钮状态记录，用于检测按钮按下事件
bool last_btnA = false;
bool last_btnB = false;
bool last_btnX = false;
bool last_btnY = false;

// 连接状态
bool deviceConnected = false;
bool crossStepToggled = false;
static uint32_t lastXToggleMs = 0;  // 记录上次X键切换时间
static uint32_t lastYToggleMs = 0;  // 记录上次Y键切换时间

// BLE输入开关：允许禁用对target的覆盖
static volatile bool sBleInputEnabled = true;

// 函数声明
void processControllerData(const XboxControllerNotificationParser& data);
void BLE_TestTask(void *arg);  // 添加任务函数声明

// 外部控制BLE输入开关
void BLE_SetInputEnabled(bool enabled) {
    sBleInputEnabled = enabled;
    Serial.printf("BLE input %s\n", enabled ? "enabled" : "disabled");
}

bool BLE_GetInputEnabled(void) {
    return sBleInputEnabled;
}

void BLE_Init(void)
{
    Serial.println("Starting Xbox Bluetooth Client");
    Serial.println("Auto-scan mode: Searching for 'Xbox Wireless Controller' by name");
    Serial.println("Make sure your Xbox controller is in pairing mode:");
    Serial.println("1. Press and hold Xbox button + Share button for 3 seconds");
    Serial.println("2. Controller LED should start flashing rapidly");
    Serial.println("3. The system will automatically connect when 'Xbox Wireless Controller' is detected");
    
    // 初始化Xbox控制器
    xboxController.begin();
    delay(500);  // 参考代码中的延时
    
    Serial.println("Xbox Controller initialized. Scanning for connection...");
    
    // 启动FreeRTOS任务处理Xbox手柄输入
    xTaskCreate(BLE_TestTask, "Xbox_ProcessTask", 4096, NULL, 1, NULL);
    Serial.println("Xbox processing task started");
}

// 处理Xbox手柄通知数据 - 简化版本，基于参考代码
void process_xbox_notif()
{
    xboxController.onLoop();
    
    if (xboxController.isConnected()) {
        if (!deviceConnected) {
            deviceConnected = true;
            Serial.println("Xbox Controller connected successfully!");
        }
        
        if (!xboxController.isWaitingForFirstNotification()) {
            // 处理控制器数据
            processControllerData(xboxController.xboxNotif);
        }
    } else {
        if (deviceConnected) {
            deviceConnected = false;
            Serial.println("Xbox Controller disconnected!");
            // 断开连接时停止所有运动（若BLE输入开启且串口队列不忙）
            if (sBleInputEnabled && !Serial_IsQueueBusy()) {
                target.speedCmd = 0.0f;
                target.yawSpeedCmd = 0.0f;
            }
        }
    }
}

// 处理控制器数据的函数
void processControllerData(const XboxControllerNotificationParser& data)
{
    // 手柄轴原始范围为 0..65535（uint16_t），中心约为 32767
    const int32_t JOY_MAX = XboxControllerNotificationParser::maxJoy;   // 65535
    const int32_t JOY_CENTER = JOY_MAX / 2;                              // 32767

    // 获取摇杆与扳机数据，并转换为以中心为 0 的有符号值
    int32_t leftStickY_raw = (int32_t)data.joyLVert - JOY_CENTER;   // 左摇杆Y轴（前后）
    int32_t leftStickX_raw = (int32_t)data.joyLHori - JOY_CENTER;   // 左摇杆X轴（左右）
    int32_t rightStickX_raw = (int32_t)data.joyRHori - JOY_CENTER;  // 右摇杆X轴（转向）
    int32_t rightStickY_raw = (int32_t)data.joyRVert - JOY_CENTER;  // 右摇杆Y轴（腿长）
    int16_t rightTrigger = data.trigRT;   // 右扳机（0..1023）
    int16_t leftTrigger = data.trigLT;    // 左扳机（0..1023）
    
    // 处理移动控制（左摇杆Y轴）——在BLE输入开启且串口队列不忙时才更新
    if (sBleInputEnabled && !Serial_IsQueueBusy()) {
        if (abs(leftStickY_raw) > deadZone) {
            float normY = (float)leftStickY_raw / (float)JOY_CENTER;  // [-1, 1]
            target.speedCmd = -normY * MAX_SPEED; // 方向取反匹配物理期望
        } else {
            target.speedCmd = 0.0f;
        }
    }
    
    // 处理转向控制（右摇杆X轴）——在BLE输入开启且串口队列不忙时才更新
    if (sBleInputEnabled && !Serial_IsQueueBusy()) {
        if (abs(rightStickX_raw) > deadZone) {
            float normYaw = (float)rightStickX_raw / (float)JOY_CENTER; // [-1, 1]
            target.yawSpeedCmd = -normYaw * MAX_YAWSPEED; // 方向取反匹配物理期望
        } else {
            target.yawSpeedCmd = 0.0f;
        }
    }

    // 处理ROLL偏移控制（左摇杆X轴）——在BLE输入开启且串口队列不忙时才更新
    if (sBleInputEnabled && !Serial_IsQueueBusy()) {
        if (abs(leftStickX_raw) > deadZoneRoll) {
            float normRoll = (float)leftStickX_raw / (float)JOY_CENTER; // [-1, 1]
            target.rollAngle = -normRoll * MAX_ROLL_OFFSET;
            Serial.printf("joyLHori=%u raw=%ld norm=%.3f roll=%.3f\n",
                        data.joyLHori,
                        (long)leftStickX_raw,
                        normRoll,
                        target.rollAngle);
        } else {
            target.rollAngle = 0.0f;
        }
    }
    // 处理腿长控制（右摇杆Y轴）：中心为中间腿长，方向取反——在BLE输入开启且串口队列不忙时才更新
    if (sBleInputEnabled && !Serial_IsQueueBusy()) {
        float normLeg = (float)rightStickY_raw / (float)JOY_CENTER; // [-1, 1]
        float mappedLeg = MIN_LEG_LENGTH + (MAX_LEG_LENGTH - MIN_LEG_LENGTH) * ((-normLeg + 1.0f) * 0.5f);
        target.legLength = _constrain(mappedLeg, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    }
    
    // 处理按钮事件（检测按下瞬间）
    bool current_btnA = data.btnA;
    bool current_btnB = data.btnB;
    bool current_btnX = data.btnX;
    bool current_btnY = data.btnY;
    
    // A键：起立
    if (current_btnA && !last_btnA) {
        if (standupState == StandupState_None) {
            xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
            standupState = StandupState_Standup;
            Serial.println("Standup command triggered");
        }
    }
    
    // Y键：切换平衡站立使能（开/关），带去抖
    if (current_btnY && !last_btnY) {
        uint32_t nowMs = millis();
        if (nowMs - lastYToggleMs >= TOGGLE_DEBOUNCE_MS) {
            balanceEnabled = !balanceEnabled;
            if (balanceEnabled) {
                Serial.println("Balance ENABLED via Y");
                // 若当前未处于站立状态，触发起立准备
                if (standupState == StandupState_None) {
                    xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
                    standupState = StandupState_Standup;
                }
            } else {
                Serial.println("Balance DISABLED via Y");
                // 关闭移动与交叉步，避免误动作
                target.speedCmd = 0.0f;
                target.yawSpeedCmd = 0.0f;
                target.crossStepEnabled = false;
            }
            lastYToggleMs = nowMs;
        }
    }
    
    // B键：跳跃
    if (current_btnB && !last_btnB) {
        xTaskCreate(Ctrl_JumpPrepareTask, "Ctrl_JumpPrepareTask", 4096, NULL, 1, NULL);
        Serial.println("Jump command triggered");
    }
    
    // X键：交叉步切换（带去抖/最小间隔）
    if (current_btnX && !last_btnX) {
        uint32_t nowMs = millis();
        if (nowMs - lastXToggleMs >= TOGGLE_DEBOUNCE_MS) {
            if (!crossStepToggled) {
                target.legLength = _constrain(target.legLength + 0.02f, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
                target.crossStepEnabled = true;
                Serial.println("Cross step enabled");
            } else {
                target.legLength = _constrain(target.legLength - 0.02f, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
                target.crossStepEnabled = false;
                Serial.println("Cross step disabled");
            }
            crossStepToggled = !crossStepToggled;
            lastXToggleMs = nowMs;
        }
    }
    
    // 更新按钮状态
    last_btnA = current_btnA;
    last_btnB = current_btnB;
    last_btnX = current_btnX;
    last_btnY = current_btnY;
}

// 获取连接状态
bool BLE_IsConnected(void)
{
    return deviceConnected;
}

// 主处理函数 - 在main loop中调用（兼容性保留）
void BLE_ProcessXboxInput(void)
{
    process_xbox_notif();
}

// FreeRTOS任务方式处理Xbox手柄输入
void BLE_TestTask(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        process_xbox_notif();
        vTaskDelayUntil(&xLastWakeTime, 20); // 50Hz更新频率
    }
}
