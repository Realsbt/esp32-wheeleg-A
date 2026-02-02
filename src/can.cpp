/*
 * CAN通信模块 - 用于ESP32与电机控制器之间的数据通信
 * 本文件实现了CAN总线的初始化、数据发送和接收功能
 */

#include "driver/twai.h"      // 包含ESP32的TWAI(CAN)驱动头文件，提供CAN通信的基本功能
#include <esp_task_wdt.h>     // 包含ESP32任务看门狗头文件，用于任务监控和超时保护
#include "can.h"              // 包含本模块的头文件，定义了CAN相关的函数声明
#include <string.h>           // 包含C标准库字符串处理函数，这里主要用memcpy函数
#include "motor.h"            // 包含电机控制模块头文件，用于更新电机状态数据

/*
 * CAN数据接收回调函数
 * 功能：当收到CAN数据时，根据不同的ID来更新对应电机的状态
 * 参数：id - CAN消息的标识符(ID)，用来区分不同的电机
 *      data - 指向接收到的8字节数据的指针
 */
void CAN_RecvCallback(uint32_t id, uint8_t *data)
{
	switch (id) // 根据接收到的CAN ID来判断是哪个电机发送的数据
	{
	case 0x101: // 如果ID是0x101，表示左腿第一个关节电机的数据
		Motor_Update(&leftJoint[0], data); // 更新左腿第一个关节电机的状态信息
		break; // 跳出switch语句
	case 0x102: // 如果ID是0x102，表示左腿第二个关节电机的数据
		Motor_Update(&leftJoint[1], data); // 更新左腿第二个关节电机的状态信息
		break; // 跳出switch语句
	case 0x103: // 如果ID是0x103，表示左侧轮子电机的数据
		Motor_Update(&leftWheel, data); // 更新左侧轮子电机的状态信息
		break; // 跳出switch语句
	case 0x105: // 如果ID是0x105，表示右腿第一个关节电机的数据
		Motor_Update(&rightJoint[0], data); // 更新右腿第一个关节电机的状态信息
		break; // 跳出switch语句
	case 0x106: // 如果ID是0x106，表示右腿第二个关节电机的数据
		Motor_Update(&rightJoint[1], data); // 更新右腿第二个关节电机的状态信息
		break; // 跳出switch语句
	case 0x107: // 如果ID是0x107，表示右侧轮子电机的数据
		Motor_Update(&rightWheel, data); // 更新右侧轮子电机的状态信息
		break; // 跳出switch语句
	}
}

/*
 * CAN通信测试任务函数（仅用于测试，正常运行时不启用）
 * 功能：定期向电机发送测试数据，用于验证CAN通信是否正常工作
 * 参数：arg - 任务参数（这里未使用）
 */
void CAN_TestTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前系统时间，用于精确的任务调度
	uint8_t data[8] = {0}; // 创建8字节的数据数组，并初始化为0，用于存放要发送的CAN数据
	
	while(1) // 无限循环，持续执行测试
	{
		// 准备发送给左侧电机的测试数据
		*(int16_t *)&data[0] = ((int16_t)(1 * 0000)); // 在数据的第0-1字节位置写入0（转矩命令）
		*(int16_t *)&data[2] = ((int16_t)(1 * 0000)); // 在数据的第2-3字节位置写入0（速度命令）
		*(int16_t *)&data[4] = ((int16_t)(1 * 2000)); // 在数据的第4-5字节位置写入2000（位置命令）
		CAN_SendFrame(0x100,data); // 向ID为0x100的左侧电机发送测试数据
		
		// 准备发送给右侧电机的测试数据
		*(int16_t *)&data[0] = ((int16_t)(1 * 0000)); // 在数据的第0-1字节位置写入0（转矩命令）
		*(int16_t *)&data[2] = ((int16_t)(1 * 0000)); // 在数据的第2-3字节位置写入0（速度命令）
		*(int16_t *)&data[4] = ((int16_t)(1 * 1000)); // 在数据的第4-5字节位置写入1000（位置命令）
		CAN_SendFrame(0x200,data); // 向ID为0x200的右侧电机发送测试数据
		
		vTaskDelayUntil(&xLastWakeTime, 2); // 等待2毫秒后再次执行，实现2ms的循环周期
	}
}

/*
 * CAN数据帧轮询接收任务函数
 * 功能：持续监听CAN总线上的数据，当有数据到达时立即处理
 * 参数：arg - 任务参数（这里未使用）
 */
void CAN_RecvTask(void *arg)
{
	twai_message_t msg; // 定义CAN消息结构体变量，用于存储接收到的CAN数据
	twai_status_info_t status; // 定义CAN状态信息结构体，用于获取CAN控制器的状态
	
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前系统时间，用于精确的任务调度
	while (1) // 无限循环，持续监听CAN数据
	{
		twai_get_status_info(&status); // 获取CAN控制器的当前状态信息
		// 循环处理接收缓冲区中的所有消息
		for(uint8_t i = 0; i < status.msgs_to_rx; i++) // msgs_to_rx表示待接收的消息数量
		{
			// 尝试从CAN接收缓冲区读取一条消息，超时时间为0（立即返回）
			if(twai_receive(&msg, 0) == ESP_OK) // 如果成功接收到消息
				CAN_RecvCallback(msg.identifier, msg.data); // 调用回调函数处理接收到的数据
		}
		vTaskDelayUntil(&xLastWakeTime, 2); // 等待2毫秒后再次检查，实现2ms的轮询周期
	}
}


/*
 * CAN通信外设初始化函数
 * 功能：配置并启动ESP32的CAN控制器，设置通信参数和创建接收任务
 */
void CAN_Init(void)
{
	// 配置CAN控制器的基本参数
	twai_general_config_t twai_conf = {
		.mode = TWAI_MODE_NORMAL,           // 设置为正常工作模式（非测试模式）
		.tx_io = GPIO_NUM_33,               // 设置CAN发送引脚为GPIO33
		.rx_io = GPIO_NUM_32,               // 设置CAN接收引脚为GPIO32
		.clkout_io = TWAI_IO_UNUSED,        // 时钟输出引脚不使用
		.bus_off_io = TWAI_IO_UNUSED,       // 总线关闭指示引脚不使用
		.tx_queue_len = 5,                  // 发送队列长度设为5个消息
		.rx_queue_len = 10,                 // 接收队列长度设为10个消息
		.alerts_enabled = TWAI_ALERT_NONE,  // 不启用任何警报功能
		.clkout_divider = 0,                // 时钟分频器设为0（不分频）
		.intr_flags = ESP_INTR_FLAG_LEVEL1  // 中断优先级设为1级
	};

	// 配置CAN通信的时序参数（波特率等）
	twai_timing_config_t twai_timing = TWAI_TIMING_CONFIG_1MBITS(); // 设置CAN通信速率为1Mbps

	// 配置CAN消息过滤器
	twai_filter_config_t twai_filter = {
		.acceptance_code = 0x00000000,      // 接收码设为0（接收所有消息）
		.acceptance_mask = 0xFFFFFFFF,      // 接收掩码设为全1（不过滤任何消息）
		.single_filter = true               // 使用单一过滤器模式
	};

	// 安装CAN驱动程序，传入配置参数
	twai_driver_install(&twai_conf, &twai_timing, &twai_filter);
	// 启动CAN控制器，开始正常通信
	twai_start();
	
	// 创建CAN测试任务（注释掉，仅在测试时使用）
	//xTaskCreate(CAN_TestTask, "CAN_TestTask", 2048, NULL, 5, NULL);
	
	// 创建CAN接收任务，用于处理接收到的CAN数据
	// 参数说明：任务函数、任务名称、堆栈大小(2048字节)、任务参数(NULL)、优先级(4)、任务句柄(NULL)
	xTaskCreate(CAN_RecvTask, "CAN_RecvTask", 2048, NULL, 4, NULL);
}


/*
 * 发送一帧CAN数据函数
 * 功能：向指定ID的设备发送8字节的CAN数据
 * 参数：id - CAN消息的标识符，用于指定接收设备
 *      data - 指向8字节数据的指针，包含要发送的具体数据内容
 */
void CAN_SendFrame(uint32_t id, uint8_t *data)
{
	twai_message_t msg; // 定义CAN消息结构体变量，用于构造要发送的消息
	msg.flags = 0; // 设置消息标志为0（标准帧，数据帧）
	msg.identifier = id; // 设置消息的标识符（ID），用于标识目标设备
	msg.data_length_code = 8; // 设置数据长度为8字节（CAN数据帧的标准长度）
	memcpy(msg.data, data, 8); // 将要发送的8字节数据复制到消息结构体中
	twai_transmit(&msg, 100); // 发送CAN消息，超时时间设为100毫秒
}
