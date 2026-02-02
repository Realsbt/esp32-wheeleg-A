#include <Arduino.h>
#include "serial.h"
#include "imu.h"
#include "motor.h"
#include "legs.h"
#include "ctrl.h"
#include "adc.h"
#include "pid.h"
#include "ble.h"
#include <ctype.h>
#include <math.h>

#define MAX_SPEED           (0.8F)
#define MAX_YAWSPEED        (4.5F)
#define MAX_LEG_LENGTH      (0.115F)
#define MIN_LEG_LENGTH      (0.058F)
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 指令队列相关定义
#define MAX_COMMAND_QUEUE_SIZE 20
#define MAX_COMMAND_LENGTH 64

extern void Ctrl_StandupPrepareTask(void *arg);
extern void Ctrl_JumpPrepareTask(void *arg);

// 指令类型枚举
typedef enum {
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,
    CMD_JUMP,
    CMD_STANDUP,
    CMD_CROSSLEG,
    CMD_INCREASE_LEG,
    CMD_DECREASE_LEG,
    CMD_DELAY,  // 延时指令
    CMD_UNKNOWN
} CommandType;

// 指令执行状态
typedef enum {
    EXEC_IDLE,      // 空闲
    EXEC_RUNNING,   // 执行中
    EXEC_PAUSED,    // 暂停
    EXEC_STOPPED    // 停止
} ExecutionState;

typedef enum {
    CROSSLEG_IDLE,
    CROSSLEG_INCREASE_LEG,
    CROSSLEG_WAIT_INCREASE,
    CROSSLEG_DISPLAY,
    CROSSLEG_WAIT_DISPLAY,
    CROSSLEG_DECREASE_LEG,
    CROSSLEG_WAIT_DECREASE,
    CROSSLEG_COMPLETED
} CrossLegState;

// 单个指令结构
typedef struct {
    CommandType type;
    int param1;     // 速度/角度/增量
    int param2;     // 持续时间
    uint32_t startTime;  // 开始执行时间
    uint32_t phaseStartTime; // 阶段开始时间（用于状态机等）
    bool isCompleted;    // 是否完成
    float originalLegLength;      // 原始腿长（用于恢复）
    CrossLegState crossLegState; // 交叉腿动作状态
} Command;

// 指令队列结构
typedef struct {
    Command commands[MAX_COMMAND_QUEUE_SIZE];
    int front;
    int rear;
    int count;
    ExecutionState state;
    int currentCommandIndex;
    TaskHandle_t executorTaskHandle;
    SemaphoreHandle_t queueMutex;
} CommandQueue;

// 全局指令队列
static CommandQueue cmdQueue = {
    .commands = {},
    .front = 0,
    .rear = 0,
    .count = 0,
    .state = EXEC_IDLE,
    .currentCommandIndex = -1,
    .executorTaskHandle = NULL,
    .queueMutex = NULL
};

// 暴露队列忙碌状态：运行或暂停时视为“忙”，用于BLE侧输入仲裁
bool Serial_IsQueueBusy(void) {
    return cmdQueue.state == EXEC_RUNNING || cmdQueue.state == EXEC_PAUSED;
}

// 队列操作函数
bool isQueueEmpty() {
    return cmdQueue.count == 0;
}

bool isQueueFull() {
    return cmdQueue.count >= MAX_COMMAND_QUEUE_SIZE;
}

bool enqueueCommand(Command cmd) {
    if (xSemaphoreTake(cmdQueue.queueMutex, portMAX_DELAY) == pdTRUE) {
        if (isQueueFull()) {
            xSemaphoreGive(cmdQueue.queueMutex);
            return false;
        }
        
        cmdQueue.commands[cmdQueue.rear] = cmd;
        cmdQueue.rear = (cmdQueue.rear + 1) % MAX_COMMAND_QUEUE_SIZE;
        cmdQueue.count++;
        
        xSemaphoreGive(cmdQueue.queueMutex);
        return true;
    }
    return false;
}

bool dequeueCommand(Command* cmd) {
    if (xSemaphoreTake(cmdQueue.queueMutex, portMAX_DELAY) == pdTRUE) {
        if (isQueueEmpty()) {
            xSemaphoreGive(cmdQueue.queueMutex);
            return false;
        }
        
        *cmd = cmdQueue.commands[cmdQueue.front];
        cmdQueue.front = (cmdQueue.front + 1) % MAX_COMMAND_QUEUE_SIZE;
        cmdQueue.count--;
        
        xSemaphoreGive(cmdQueue.queueMutex);
        return true;
    }
    return false;
}
// 去除首尾空白字符的工具函数
static void trim_whitespace(char* s) {
    if (!s) return;
    // 去前导空白
    char* start = s;
    while (*start && isspace((unsigned char)*start)) start++;
    if (start != s) {
        memmove(s, start, strlen(start) + 1);
    }
    // 去末尾空白
    size_t len = strlen(s);
    if (len == 0) return;
    char* end = s + len - 1;
    while (end >= s && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
}
void clearQueue() {
    if (xSemaphoreTake(cmdQueue.queueMutex, portMAX_DELAY) == pdTRUE) {
        cmdQueue.front = 0;
        cmdQueue.rear = 0;
        cmdQueue.count = 0;
        cmdQueue.currentCommandIndex = -1;
        xSemaphoreGive(cmdQueue.queueMutex);
    }
}

// 解析指令字符串为指令类型
static void trim_whitespace(char* s);
CommandType parseCommandType(const char* cmdStr) {
    if (!cmdStr) return CMD_UNKNOWN;
    char buf[MAX_COMMAND_LENGTH];
    strncpy(buf, cmdStr, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    // 去除前后空格
    trim_whitespace(buf);
    // 转大写，大小写不敏感
    for (char* p = buf; *p; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }
    if (strcmp(buf, "FORWARD") == 0) return CMD_FORWARD;
    if (strcmp(buf, "BACKWARD") == 0) return CMD_BACKWARD;
    if (strcmp(buf, "LEFT") == 0) return CMD_LEFT;
    if (strcmp(buf, "RIGHT") == 0) return CMD_RIGHT;
    if (strcmp(buf, "STOP") == 0) return CMD_STOP;
    if (strcmp(buf, "JUMP") == 0) return CMD_JUMP;
    if (strcmp(buf, "STANDUP") == 0) return CMD_STANDUP;
    if (strcmp(buf, "CROSSLEG") == 0) return CMD_CROSSLEG;
    if (strcmp(buf, "INCREASELEGLENGTH") == 0) return CMD_INCREASE_LEG;
    if (strcmp(buf, "DECREASELEGLENGTH") == 0) return CMD_DECREASE_LEG;
    if (strcmp(buf, "DELAY") == 0) return CMD_DELAY;
    return CMD_UNKNOWN;
}

// 执行单个指令
bool executeCommand(Command* cmd) {
    if (cmd->isCompleted) return true;
    uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // 首次执行，记录开始时间
    if (cmd->startTime == 0) {
        cmd->startTime = currentTime;
        switch (cmd->type) {
            case CMD_FORWARD:
                target.speedCmd = cmd->param1 * 0.01f * MAX_SPEED;
                Serial.printf("开始向前走, 速度:%d 时间:%dms\n", cmd->param1, cmd->param2);
                break;
            case CMD_BACKWARD:
                target.speedCmd = -cmd->param1 * 0.01f * MAX_SPEED;
                Serial.printf("开始向后走, 速度:%d 时间:%dms\n", cmd->param1, cmd->param2);
                break;
            case CMD_LEFT:
                {
                    // 根据期望角度与持续时间计算目标角速度（弧度/秒），并夹紧到上限
                    float durationSec = (cmd->param2 > 0 ? (cmd->param2 / 1000.0f) : 3.0f);
                    float angleRad = ((cmd->param1 > 0 ? cmd->param1 : 90) * (float)M_PI) / 180.0f;
                    float yawSpeed = angleRad / durationSec; // 正为左转
                    if (yawSpeed > MAX_YAWSPEED) yawSpeed = MAX_YAWSPEED;
                    target.yawSpeedCmd = yawSpeed;
                    Serial.printf("开始左转, 角度:%d 时间:%dms (角速度=%.3frad/s)\n", cmd->param1, cmd->param2, yawSpeed);
                }
                break;
            case CMD_RIGHT:
                {
                    float durationSec = (cmd->param2 > 0 ? (cmd->param2 / 1000.0f) : 3.0f);
                    float angleRad = ((cmd->param1 > 0 ? cmd->param1 : 90) * (float)M_PI) / 180.0f;
                    float yawSpeed = angleRad / durationSec; // 正值，右转取负
                    if (yawSpeed > MAX_YAWSPEED) yawSpeed = MAX_YAWSPEED;
                    target.yawSpeedCmd = -yawSpeed;
                    Serial.printf("开始右转, 角度:%d 时间:%dms (角速度=%.3frad/s)\n", cmd->param1, cmd->param2, -yawSpeed);
                }
                break;
            case CMD_STOP:
                target.speedCmd = 0;
                target.yawSpeedCmd = 0;
                target.crossStepEnabled = false;
                Serial.printf("停止\n");
                cmd->isCompleted = true;
                return true;
            case CMD_JUMP:
                xTaskCreate(Ctrl_JumpPrepareTask, "Ctrl_JumpPrepareTask", 4096, NULL, 1, NULL);
                Serial.printf("跳跃开始\n");
                if (cmd->param2 == 0) cmd->param2 = 2000;
                break;
            case CMD_STANDUP:
                if(standupState == StandupState_None) {
                    xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
                    standupState = StandupState_Standup;
                    Serial.printf("劈叉开始\n");
                }
                if (cmd->param2 == 0) cmd->param2 = 3000;
                break;
            case CMD_CROSSLEG:
                if (cmd->crossLegState == CROSSLEG_IDLE) {
                    cmd->originalLegLength = target.legLength;
                    target.crossStepEnabled = true;
                    Serial.printf("开始交叉腿展示\n");
                    cmd->crossLegState = CROSSLEG_INCREASE_LEG;
                    cmd->phaseStartTime = currentTime; // 阶段计时从开始时刻起
                    if (cmd->param2 == 0) cmd->param2 = 5000; // 展示总时长5秒
                }
                break;
            case CMD_INCREASE_LEG:
                target.legLength = _constrain(target.legLength + 0.01f * cmd->param1, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
                Serial.printf("开始增加腿长 %d\n", cmd->param1);
                if (cmd->param2 == 0) cmd->param2 = 1000;
                break;
            case CMD_DECREASE_LEG:
                target.legLength = _constrain(target.legLength - 0.01f * cmd->param1, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
                Serial.printf("开始减少腿长 %d\n", cmd->param1);
                if (cmd->param2 == 0) cmd->param2 = 1000;
                break;
            case CMD_DELAY:
                Serial.printf("开始延时 %dms\n", cmd->param2);
                break;
            default:
                Serial.printf("未知指令类型\n");
                cmd->isCompleted = true;
                return true;
        }
    }
    // 在首次执行之外推进交叉腿状态机
    if (cmd->type == CMD_CROSSLEG && !cmd->isCompleted) {
        switch (cmd->crossLegState) {
            case CROSSLEG_INCREASE_LEG:
                if (currentTime - cmd->phaseStartTime >= 500) { // 0.5秒后增加腿长
                    target.legLength = _constrain(target.legLength + 0.01f, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
                    Serial.printf("交叉腿展示 - 腿长已调整\n");
                    cmd->crossLegState = CROSSLEG_DISPLAY;
                    cmd->phaseStartTime = currentTime;
                }
                break;
            case CROSSLEG_DISPLAY:
                // 展示阶段，等待总时长结束由通用逻辑收尾
                break;
            default:
                break;
        }
    }
    // 检查是否需要持续时间的指令是否完成
    if (cmd->param2 > 0) {
        uint32_t elapsed = currentTime - cmd->startTime;
        if (elapsed >= cmd->param2) {
            if (cmd->type == CMD_FORWARD || cmd->type == CMD_BACKWARD) {
                target.speedCmd = 0;
                Serial.printf("运动指令完成，自动停止\n");
            } else if (cmd->type == CMD_LEFT || cmd->type == CMD_RIGHT) {
                target.yawSpeedCmd = 0;
                Serial.printf("转向指令完成，自动停止\n");
            } else if (cmd->type == CMD_JUMP) {
                Serial.printf("跳跃动作完成\n");
            } else if (cmd->type == CMD_STANDUP) {
                Serial.printf("起立动作完成\n");
            } else if (cmd->type == CMD_CROSSLEG) {
                target.crossStepEnabled = false;
                target.legLength = cmd->originalLegLength;
                Serial.printf("交叉腿展示完成，动作已关闭，腿长已恢复\n");
            } else if (cmd->type == CMD_INCREASE_LEG) {
                Serial.printf("增加腿长完成\n");
            } else if (cmd->type == CMD_DECREASE_LEG) {
                Serial.printf("减少腿长完成\n");
            } else if (cmd->type == CMD_DELAY) {
                Serial.printf("延时完成\n");
            }
            cmd->isCompleted = true;
            return true;
        }
    }
    return false; // 指令还在执行中
}

// 指令执行器任务
void CommandExecutorTask(void *pvParameters) {
    Command currentCmd;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1) {
        if (cmdQueue.state == EXEC_RUNNING) {
            // 如果当前没有正在执行的指令，从队列获取下一个
            if (cmdQueue.currentCommandIndex == -1) {
                if (dequeueCommand(&currentCmd)) {
                    cmdQueue.currentCommandIndex = 0;
                    currentCmd.startTime = 0;  // 重置开始时间
                    currentCmd.phaseStartTime = 0; // 重置阶段开始时间
                    currentCmd.isCompleted = false;
                    currentCmd.crossLegState = CROSSLEG_IDLE; // 重置交叉腿状态
                    Serial.printf("开始执行新指令\n");
                } else {
                    // 队列为空，设置为空闲状态
                    cmdQueue.state = EXEC_IDLE;
                    Serial.printf("指令队列执行完成\n");
                }
            }
            
            // 执行当前指令
            if (cmdQueue.currentCommandIndex != -1) {
                if (executeCommand(&currentCmd)) {
                    // 当前指令完成，准备执行下一个
                    cmdQueue.currentCommandIndex = -1;
                    Serial.printf("指令执行完成\n");
                }
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50)); // 50ms检查一次
    }
}

// 添加指令到队列
bool addCommandToQueue(const char* cmdStr, const char* param1Str, const char* param2Str) {
    Command cmd = {
        .type = CMD_UNKNOWN,
        .param1 = 0,
        .param2 = 0,
        .startTime = 0,
        .phaseStartTime = 0,
        .isCompleted = false,
        .crossLegState = CROSSLEG_IDLE // 初始化交叉腿状态
    };
    cmd.type = parseCommandType(cmdStr);
    
    if (cmd.type == CMD_UNKNOWN) {
        Serial.printf("未知指令: %s\n", cmdStr);
        return false;
    }
    
    // 解析参数
    cmd.param1 = param1Str ? atoi(param1Str) : 0;
    cmd.param2 = param2Str ? atoi(param2Str) * 1000 : 0; // 转换为毫秒
    
    // 设置默认值
    switch (cmd.type) {
        case CMD_FORWARD:
        case CMD_BACKWARD:
             if (cmd.param1 == 0) cmd.param1 = 50;  // 默认速度50
             if (cmd.param2 == 0) cmd.param2 = 3000; // 默认3秒
             break;
         case CMD_LEFT:
         case CMD_RIGHT:
             if (cmd.param1 == 0) cmd.param1 = 90;   // 默认角度90 (虽然实际只用duration)
             if (cmd.param2 == 0) cmd.param2 = 3000; // 默认3秒
             break;
         case CMD_INCREASE_LEG:
         case CMD_DECREASE_LEG:
             if (cmd.param1 == 0) cmd.param1 = 2;    // 默认增量2
             break;
         case CMD_DELAY:
             if (cmd.param2 == 0) cmd.param2 = 1000; // 默认延时1秒
             break;
     }
     // 基本参数合法化与边界夹紧
     if (cmd.param1 < 0) cmd.param1 = 0;
     if (cmd.param2 < 0) cmd.param2 = 0;
     if (cmd.type == CMD_FORWARD || cmd.type == CMD_BACKWARD) {
         if (cmd.param1 < 1) cmd.param1 = 1;
         if (cmd.param1 > 100) cmd.param1 = 100;
     }
     if (cmd.type == CMD_INCREASE_LEG || cmd.type == CMD_DECREASE_LEG) {
         if (cmd.param1 < 1) cmd.param1 = 1;
         if (cmd.param1 > 10) cmd.param1 = 10;
     }
     if (!enqueueCommand(cmd)) {
         Serial.printf("指令队列已满，无法添加指令: %s\n", cmdStr);
         return false;
     }
     return true;
}

// 队列控制函数
void startQueueExecution() {
    cmdQueue.state = EXEC_RUNNING;
    Serial.printf("开始执行指令队列\n");
}

void pauseQueueExecution() {
    cmdQueue.state = EXEC_PAUSED;
    // 停止当前运动
    target.speedCmd = 0;
    target.yawSpeedCmd = 0;
    Serial.printf("暂停指令队列执行\n");
}

void resumeQueueExecution() {
    cmdQueue.state = EXEC_RUNNING;
    Serial.printf("恢复指令队列执行\n");
}

void stopQueueExecution() {
    cmdQueue.state = EXEC_STOPPED;
    // 停止当前运动
    target.speedCmd = 0;
    target.yawSpeedCmd = 0;
    // 清空队列
    clearQueue();
    Serial.printf("停止并清空指令队列\n");
}

// 检查是否为复合指令（包含分号）
bool isCompositeCommand(const char* cmd) {
    return strchr(cmd, ';') != NULL;
}

// 解析并入队一个复合或单条命令序列（以分号分隔），返回成功添加的数量
static int parseAndEnqueueSequence(const char* line) {
    if (!line) return 0;
    char sequenceBuffer[256];
    strncpy(sequenceBuffer, line, sizeof(sequenceBuffer) - 1);
    sequenceBuffer[sizeof(sequenceBuffer) - 1] = '\0';

    char *saveptr_seq = NULL;
    char *subCmd = strtok_r(sequenceBuffer, ";", &saveptr_seq);
    int addedCount = 0;
    while (subCmd != NULL) {
        // 拆出类型与参数
        char subBuf[128];
        strncpy(subBuf, subCmd, sizeof(subBuf) - 1);
        subBuf[sizeof(subBuf) - 1] = '\0';

        char *saveptr_sub = NULL;
        char *subType = strtok_r(subBuf, ",", &saveptr_sub);
        char *subParam1 = strtok_r(NULL, ",", &saveptr_sub);
        char *subParam2 = strtok_r(NULL, ",", &saveptr_sub);

        trim_whitespace(subType);
        trim_whitespace(subParam1);
        trim_whitespace(subParam2);

        if (subType != NULL) {
            if (addCommandToQueue(subType, subParam1, subParam2)) {
                addedCount++;
                Serial.printf("添加子命令到队列: %s\n", subType);
            } else {
                Serial.printf("添加子命令失败: %s\n", subType);
            }
        }
        subCmd = strtok_r(NULL, ";", &saveptr_seq);
    }
    return addedCount;
}

// 串口2指令处理任务
void UART2_CommandTask(void *pvParameters) {
    char rxBuffer[256] = {0};
    int bufIndex = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        while (Serial2.available() > 0) {
            char c = Serial2.read();

            if (c == '\n' || bufIndex >= 255) {
                rxBuffer[bufIndex] = '\0';
                // 兼容 CRLF，去掉行尾的 '\r'
                if (bufIndex > 0 && rxBuffer[bufIndex - 1] == '\r') {
                    rxBuffer[bufIndex - 1] = '\0';
                }
                Serial.printf("收到命令: %s\n", rxBuffer);
                bufIndex = 0;

                // 特殊控制指令
                if (strcmp(rxBuffer, "QUEUE_START") == 0) {
                    startQueueExecution();
                } else if (strcmp(rxBuffer, "QUEUE_PAUSE") == 0) {
                    pauseQueueExecution();
                } else if (strcmp(rxBuffer, "QUEUE_RESUME") == 0) {
                    resumeQueueExecution();
                } else if (strcmp(rxBuffer, "QUEUE_STOP") == 0) {
                    stopQueueExecution();
                } else if (strcmp(rxBuffer, "QUEUE_STATUS") == 0) {
                    Serial.printf("队列状态: %s, 队列长度: %d\n", 
                        (cmdQueue.state == EXEC_IDLE) ? "空闲" :
                        (cmdQueue.state == EXEC_RUNNING) ? "运行中" :
                        (cmdQueue.state == EXEC_PAUSED) ? "暂停" : "停止",
                        cmdQueue.count);
                } else if (strcmp(rxBuffer, "BALANCE_ON") == 0) {
                    balanceEnabled = true;
                    Serial.println("Balance ENABLED via serial");
                    if (standupState == StandupState_None) {
                        xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
                        standupState = StandupState_Standup;
                    }
                } else if (strcmp(rxBuffer, "BALANCE_OFF") == 0) {
                    balanceEnabled = false;
                    target.speedCmd = 0.0f;
                    target.yawSpeedCmd = 0.0f;
                    target.crossStepEnabled = false;
                    Serial.println("Balance DISABLED via serial");
                } else if (strcmp(rxBuffer, "BLE_DISABLE") == 0) {
                    BLE_SetInputEnabled(false);
                    Serial.println("BLE input disabled via serial");
                } else if (strcmp(rxBuffer, "BLE_ENABLE") == 0) {
                    BLE_SetInputEnabled(true);
                    Serial.println("BLE input enabled via serial");
                } else if (strcmp(rxBuffer, "BLE_STATUS") == 0) {
                    Serial.printf("BLE connected: %s, input enabled: %s\n",
                                  BLE_IsConnected() ? "yes" : "no",
                                  BLE_GetInputEnabled() ? "yes" : "no");
                }
                // 检查是否为复合指令
                else if (isCompositeCommand(rxBuffer)) {
                    Serial.printf("处理复合指令: %s\n", rxBuffer);
                    int addedCount = parseAndEnqueueSequence(rxBuffer);
                    Serial.printf("复合指令解析完成，添加了 %d 个指令到队列\n", addedCount);
                    if (addedCount > 0) {
                        if (cmdQueue.state != EXEC_RUNNING) {
                            startQueueExecution();
                        } else {
                            Serial.printf("队列正在执行，已追加 %d 条指令\n", addedCount);
                        }
                    }
                } else {
                    // 统一按“单条序列”处理（即使没有分号）
                    int addedCount = parseAndEnqueueSequence(rxBuffer);
                    if (addedCount > 0) {
                        Serial.printf("添加单条指令序列到队列，共 %d 条\n", addedCount);
                        if (cmdQueue.state != EXEC_RUNNING) {
                            startQueueExecution();
                        }
                    } else {
                        Serial.printf("无效命令\n");
                    }
                }
            }
            else {
                if (bufIndex < 255) {
                    rxBuffer[bufIndex++] = c;
                }
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

// 串口定时发送任务(调试用)
void Serial_Task(void *pvParameters) {
    while (1) {
        // 调试输出代码保持不变
        vTaskDelay(50);
    }
} 

// 串口模块初始化
void Serial_Init(void) {
    Serial.begin(115200);
    Serial.setTimeout(10);
    
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    
    // 初始化指令队列
    cmdQueue.front = 0;
    cmdQueue.rear = 0;
    cmdQueue.count = 0;
    cmdQueue.state = EXEC_IDLE;
    cmdQueue.currentCommandIndex = -1;
    cmdQueue.queueMutex = xSemaphoreCreateMutex();
    
    // 创建任务
    xTaskCreate(UART2_CommandTask, "UART2_CommandTask", 4096, NULL, 2, NULL);
    xTaskCreate(CommandExecutorTask, "CommandExecutorTask", 4096, NULL, 1, &cmdQueue.executorTaskHandle);
    xTaskCreate(Serial_Task, "Serial_Task", 4096, NULL, 1, NULL);
    
    Serial.printf("指令队列系统初始化完成\n");
}

