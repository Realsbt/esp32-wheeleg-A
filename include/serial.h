#ifndef SERIAL_H
#define SERIAL_H

void Serial_Init(void);

// 查询串口指令队列是否忙（运行或暂停），用于在BLE输入侧做仲裁
bool Serial_IsQueueBusy(void);

#endif