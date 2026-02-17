#include "motor.h"
#include "can.h"
#include <esp_task_wdt.h>
#include <math.h>
#include <Preferences.h>
#include "imu.h"
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象
float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
float leftJointGain = 1.0f;
/******* 电机模块 *******/
static Preferences preferences; 
//初始化一个电机对象
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed))
{
	motor->speed = motor->angle = motor->voltage = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->calcRevVolt = calcRevVolt;
}

//只测量正转 后面有处理
float Motor_CalcRevVolt4310(float speed)
{
	//return 0.000002f * speed * speed * speed - 0.0002f * speed * speed + 0.1521f * speed;     //4310
	return 0.000002f * speed * speed * speed - 0.0002f * speed * speed + 0.0521f * speed;   //4010
}

//2804电机反电动势计算函数(输入速度，输出反电动势)，测量方法同上,轮电机多次实验发现不补偿效果可能更好一些
float Motor_CalcRevVolt2805(float speed)
{
	return 0.0000003f * speed * speed * speed - 0.00007f * speed * speed + 0.0175f * speed;
	return 0;
}


//由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
//补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
void Motor_UpdateVoltage(Motor *motor)
{
	float voltage = motor->torque / motor->torqueRatio * motorOutRatio;
	if (motor->speed >= 0)
		voltage += motor->calcRevVolt(motor->speed);
	else if (motor->speed < 0)
		voltage -= motor->calcRevVolt(-motor->speed);	//这段没问题 方向最后加上 先按照标量算

	//限幅
	if (voltage > motor->maxVoltage)
		voltage = motor->maxVoltage;
	else if (voltage < -motor->maxVoltage)
		voltage = -motor->maxVoltage;
	motor->voltage = voltage  * motor->dir;
}

//从CAN总线接收到的数据中解析出电机角度和速度
void Motor_Update(Motor *motor, uint8_t *data)
{
	int32_t raw = *(int32_t *)&data[0];
	motor->rawAngle = raw / 1000.0f; // 新增原始角度
	motor->angle = (motor->rawAngle - motor->offsetAngle) * motor->dir;
	motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, float torque)
{
	motor->torque = torque;
}

//电机指令发送任务
void Motor_SendTask(void *arg)
{
	uint8_t data[8] = {0};
	Motor* motorList[] = {&leftJoint[0], &leftJoint[1], &leftWheel, &rightJoint[0], &rightJoint[1], &rightWheel};
	while (1)
	{
		for (int i = 0; i < 6; i++)
			Motor_UpdateVoltage(motorList[i]); //计算补偿后的电机电压
		
		*(int16_t *)&data[0] = ((int16_t)(leftJoint[0].voltage * leftJointGain * 1000));
		*(int16_t *)&data[2] = ((int16_t)(leftJoint[1].voltage * leftJointGain * 1000));
		*(int16_t *)&data[4] = ((int16_t)(leftWheel.voltage * 1000));
		CAN_SendFrame(0x100, data);
		*(int16_t *)&data[0] = ((int16_t)(rightJoint[0].voltage * 1000));
		*(int16_t *)&data[2] = ((int16_t)(rightJoint[1].voltage * 1000));
		*(int16_t *)&data[4] = ((int16_t)(rightWheel.voltage * 1000));
		CAN_SendFrame(0x200, data);
		vTaskDelay(2);
	}
}

// 保存电机参数
void Motor_SaveParams(void)
{
    preferences.begin("motor", false);  // false表示读写模式
    // 保存4个关节电机的参数
    preferences.putFloat("l0_offset", leftJoint[0].offsetAngle);
    preferences.putInt("l0_dir", leftJoint[0].dir);
    preferences.putFloat("l1_offset", leftJoint[1].offsetAngle);
    preferences.putInt("l1_dir", leftJoint[1].dir);
    preferences.putFloat("r0_offset", rightJoint[0].offsetAngle);
    preferences.putInt("r0_dir", rightJoint[0].dir);
    preferences.putFloat("r1_offset", rightJoint[1].offsetAngle);
    preferences.putInt("r1_dir", rightJoint[1].dir);
    
    preferences.end();
    Serial.println("参数保存成功！");
}

// 从Flash加载电机参数
bool Motor_LoadParams(void)
{
    preferences.begin("motor", true);
    
    if (preferences.isKey("l0_offset")) {
        leftJoint[0].offsetAngle = preferences.getFloat("l0_offset");
        leftJoint[0].dir = preferences.getInt("l0_dir");
        if(leftJoint[0].dir != 1 && leftJoint[0].dir != -1) leftJoint[0].dir = 1;
        leftJoint[1].offsetAngle = preferences.getFloat("l1_offset");
        leftJoint[1].dir = preferences.getInt("l1_dir");
        if(leftJoint[1].dir != 1 && leftJoint[1].dir != -1) leftJoint[1].dir = 1;
        rightJoint[0].offsetAngle = preferences.getFloat("r0_offset");
        rightJoint[0].dir = preferences.getInt("r0_dir");
        if(rightJoint[0].dir != 1 && rightJoint[0].dir != -1) rightJoint[0].dir = 1;
        rightJoint[1].offsetAngle = preferences.getFloat("r1_offset");
        rightJoint[1].dir = preferences.getInt("r1_dir");
        if(rightJoint[1].dir != 1 && rightJoint[1].dir != -1) rightJoint[1].dir = 1;
        
        preferences.end();
        Serial.println("参数加载成功！");
        return true;
    }
    
    preferences.end();
    Serial.println("未找到保存的参数！");
    return false;
}

// 确定电机方向
void Motor_DetermineDirection(Motor *motor)
{
    int oldDir = motor->dir; // 记录原始dir
    motor->dir = 1; // 判定期间强制为正方向
    Motor_SetTorque(motor, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    float initialRaw = motor->rawAngle;
    Motor_SetTorque(motor, 0.2f);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    float newRaw = motor->rawAngle;
    Serial.printf("初始角度: %.3f, 新角度: %.3f\n", initialRaw, newRaw);
    float angleDiff = newRaw - initialRaw;
    Motor_SetTorque(motor, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (fabs(angleDiff) < 0.001f) {
        Serial.println("警告：电机未检测到角度变化，请检查连接和供电");
        motor->dir = oldDir; // 恢复原始dir
        return;
    }
    // 串口人工确认
    Serial.println("请观察电机是否按预期方向旋转（如逆时针），输入Y确认，N则反向");
    String input = "";
    while(1) {
        if(Serial.available()) {
            char c = Serial.read();
            if(c == '\r' || c == '\n') {
                if(input == "Y" || input == "y") {
                    motor->dir = 1;
                    break;
                } else if(input == "N" || input == "n") {
                    motor->dir = -1;
                    break;
                }
                input = "";
            } else {
                input += c;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    Serial.printf("电机方向最终确认: dir = %d\n", motor->dir);
}
// 校准零点偏移
void Motor_CalibrateOffset(Motor *motor, int jointIndex)
{	Motor_SetTorque(motor, 0); // 新增：校准前清零扭矩
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (motor->dir == 0) {
        Serial.println("错误：电机方向(dir)为0，使用默认值1");
        motor->dir = 1;
    }
    float currentAngle = motor->angle;
    float targetAngle = 0.0f;
    if (jointIndex == 1 || jointIndex == 3) {
        targetAngle = 3.14f;
    }
    motor->offsetAngle = (currentAngle - targetAngle) * motor->dir;
    Serial.printf("电机%d校准详情:\n", jointIndex + 1);
    Serial.printf("  - 当前角度: %.3f\n", currentAngle);
    Serial.printf("  - 目标角度: %.3f\n", targetAngle);
    Serial.printf("  - 方向值: %d\n", motor->dir);
    Serial.printf("  - 计算偏移: %.3f\n", motor->offsetAngle);
}
void Motor_ClearParams(void)
{
    preferences.begin("motor", false);
    preferences.clear();
    preferences.end();
    Serial.println("参数已清除！");
}

// 处理串口命令
bool logEnabled = false;
bool imuLogEnabled = false;

void Motor_HandleSerialCommand(void)
{
    String input = "";
    while(Serial.available()) {
        char c = Serial.read();
        if(c == '\n') {
            if(input == "clearmotor") {
                Motor_ClearParams();
                Serial.println("请重启设备以重新进行参数校准。");
            } else if(input == "imu") {
                IMU_CalibrateAndSaveOffsets();
            } else if(input == "clearimu") {
                IMU_ClearOffsets();
            } else if(input == "log") {
                logEnabled = true;
                Serial.println("持续输出关节角度已开启，输入'nolog'停止");
            } else if(input == "nolog") {
                logEnabled = false;
                Serial.println("持续输出关节角度已关闭");
            } else if(input == "imulog") {
                imuLogEnabled = true;
                Serial.println("IMU data output enabled, send 'noimulog' to stop");
            } else if(input == "noimulog") {
                imuLogEnabled = false;
                Serial.println("IMU data output disabled");
            }
            input = "";
        } else {
            input += c;
        }
    }
}

// 在Motor_SendTask或新建一个定时任务中增加如下逻辑：
void Log_Task(void *arg)
{
    while(1) {
        if(logEnabled) {
            Serial.printf("%.3f,%.3f,%.3f,%.3f\n", leftJoint[0].angle, leftJoint[1].angle, rightJoint[0].angle, rightJoint[1].angle);
        }
        if(imuLogEnabled) {
            Serial.printf("R:%.2f P:%.2f Y:%.2f | dR:%.2f dP:%.2f dY:%.2f\n",
                imuData.roll * 180.0f / M_PI,
                imuData.pitch * 180.0f / M_PI,
                imuData.yaw * 180.0f / M_PI,
                imuData.rollSpd * 180.0f / M_PI,
                imuData.pitchSpd * 180.0f / M_PI,
                imuData.yawSpd * 180.0f / M_PI);
        }
        vTaskDelay(100); // 100ms输出一次
    }
}


// 修改初始化函数
void Motor_InitAll(void)
{
    // 基础参数初始化
    Motor_Init(&leftJoint[0], 0, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&leftJoint[1], 0, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&leftWheel, 0, 4.0f, 0.01f, 1, Motor_CalcRevVolt2805);
    Motor_Init(&rightJoint[0], 0, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&rightJoint[1], 0, 9.69f, 0.0333f, 1, Motor_CalcRevVolt4310);
    Motor_Init(&rightWheel, 0, 4.0f, 0.01f, -1, Motor_CalcRevVolt2805);
    
    // 创建发送任务]
    xTaskCreate(Motor_SendTask, "Motor_SendTask", 2048, NULL, 4, NULL);
    
    // 等待CAN通信稳定
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 尝试加载保存的参数
    if (!Motor_LoadParams()) {
        Serial.println("\n需要进行电机参数校准，请按照提示操作：");
        
        // 步骤1：确定所有关节电机的方向
        Serial.println("开始确定电机方向，请确保电机有足够的活动空间...");
        Motor *motorList[] = {&leftJoint[0], &leftJoint[1], 
                             &rightJoint[0], &rightJoint[1]};
        for (int i = 0; i < 4; i++) {
            Serial.printf("正在确定第%d个关节电机的方向...\n", i + 1);
            Motor_DetermineDirection(motorList[i]);
            Serial.printf("电机%d方向: dir = %d\n", i + 1, motorList[i]->dir);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        
        // 步骤2：等待用户将腿部放置水平
        Serial.println("\n请将腿部放置水平位置，准备好后发送SET继续...");
        String input = "";
        while(1) {
            if(Serial.available()) {
                char c = Serial.read();
                if(c == '\r' || c == '\n') {  // 同时处理\r和\n
                    if(input == "SET") {
                        break;
                    }
                    input = "";
                } else {
                    input += c;
                }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        // 步骤3：校准零点偏移
        Serial.println("开始校准零点偏移...");
        for (int i = 0; i < 4; i++) {
            Serial.printf("正在校准第%d个关节电机的零点...\n", i + 1);
            Motor_CalibrateOffset(motorList[i], i);
            Serial.printf("电机%d参数: dir = %d, offsetAngle = %.3f\n", 
                         i + 1, motorList[i]->dir, motorList[i]->offsetAngle);
        }
        
        // 在保存参数前添加这段代码
        Serial.println("\n准备保存的最终参数：");
        for (int i = 0; i < 4; i++) {
        Serial.printf("电机%d最终参数: dir = %d, offsetAngle = %.3f\n", 
                     i + 1, motorList[i]->dir, motorList[i]->offsetAngle);
        }
        
        // 保存参数
        Motor_SaveParams();
    }
    
    // 所有初始化完成后，再创建串口命令处理任务
    xTaskCreate([](void* arg) {
        while(1) {
            Motor_HandleSerialCommand();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }, "Motor_SerialTask", 2048, NULL, 1, NULL);
	xTaskCreate(Log_Task, "Log_Task", 2048, NULL, 1, NULL);
}





