#ifndef BLE_H
#define BLE_H

// 注意：此模块现在使用Xbox手柄控制，但保持原有接口名称以兼容现有代码
void BLE_Init(void);                // 初始化Xbox手柄控制
void BLE_ProcessXboxInput(void);    // 处理Xbox手柄输入（在main loop中调用）
bool BLE_IsConnected(void);         // 检查Xbox手柄连接状态

// 允许外部（如串口）禁用/启用BLE对目标量的覆盖
void BLE_SetInputEnabled(bool enabled);
bool BLE_GetInputEnabled(void);

#endif