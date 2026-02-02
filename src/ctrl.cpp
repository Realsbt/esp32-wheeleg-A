#include <Arduino.h>
#include "pid.h"
#include "legs.h"
#include "motor.h"
#include "../include/matlab_code/leg_vmc_conv.h"
#include "../include/matlab_code/lqr_k.h"
#include "ctrl.h"
#include "imu.h"
#include <esp_task_wdt.h>
#include <math.h>

CascadePID legAnglePID, legLengthPID; //腿部角度和长度控制PID
CascadePID yawPID, rollPID; //机身yaw和roll控制PID


Target target = {0, 0, 0, 0, 0, 0, 0.07f};
StateVar stateVar;
StandupState standupState = StandupState_None; // 初始状态为None，不自动站立
bool balanceEnabled = false; // 平衡站立是否使能，默认关闭（需要按Y键启用）
GroundDetector groundDetector = {10, 10, true, false};	//离地检测器 默认状态为触地
//测试用
#define SIN_FREQUENCY_MS 5000 // 周期，单位毫秒
#define SIN_AMPLITUDE 0.04      // 幅度
#define SIN_OFFSET 0.1         // 偏移量

void vSinGeneratorTask(void *arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SIN_FREQUENCY_MS);

    // 获取当前系统时间
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 计算当前时刻的正弦函数值
        float sin_value = SIN_AMPLITUDE * sin((2 * M_PI * xTaskGetTickCount() / xFrequency))+SIN_OFFSET;
		target.legLength = sin_value; // 假设虚拟腿0的腿部长度为正弦函数值
        // 在这里可以将 sin_value 用于其他操作，比如输出到外设

        // 任务挂起，直到下一个周期
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

#define STEP_INTERVAL_MS 1000 // 跃变间隔，单位毫秒
#define STEP_AMPLITUDE 5      // 幅度
// 阶跃函数生成任务
void vStepGeneratorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(STEP_INTERVAL_MS);
    int step_value = 0;

    // 获取当前系统时间
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 每隔一定时间改变阶跃函数值
        step_value += STEP_AMPLITUDE;

        // 在这里可以将 step_value 用于其他操作，比如输出到外设

        // 任务挂起，直到下一个跃变间隔
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


//目标量更新任务(根据蓝牙收到的目标量计算实际控制算法的给定量)
void Ctrl_TargetUpdateTask(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.003f;
	while (1)
	{
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		speedSlopeStep = -(legLength - 0.07f) * 0.02f + 0.002f;

		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}

		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;

		//限制速度目标在当前速度的±0.3m/s内
		if(target.speed - stateVar.dx > 0.3f)
			target.speed = stateVar.dx + 0.3f;
		else if(target.speed - stateVar.dx < -0.3f)
			target.speed = stateVar.dx - 0.3f;

		//计算yaw方位角目标
		target.yawAngle += target.yawSpeedCmd * 0.004f;
		
		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}


//跳起
void Ctrl_JumpPrepareTask(void *arg)
{
    //将姿态放到最低
    target.legLength = 0.06f;
    vTaskDelay(500);
    standupState = StandupState_Prepare;
    //力矩输出到最大伸展
    //左腿
    Motor_SetTorque(&leftJoint[1], -1.3f);
    Motor_SetTorque(&leftJoint[0], 1.3f);
    Motor_SetTorque(&leftWheel, 0);
    //右腿
    Motor_SetTorque(&rightJoint[0], 1.3f);
    Motor_SetTorque(&rightJoint[1], -1.3f);
    Motor_SetTorque(&rightWheel, 0);
    vTaskDelay(100);

    //收起脚
    //左腿
    Motor_SetTorque(&leftJoint[1], 0.3f);
    Motor_SetTorque(&leftJoint[0], -0.3f);
    Motor_SetTorque(&leftWheel, 0);
    //右腿
    Motor_SetTorque(&rightJoint[0], -0.3f);
    Motor_SetTorque(&rightJoint[1], 0.3f);
    Motor_SetTorque(&rightWheel, 0);
    vTaskDelay(50);

    //打开关闭电机
    // Motor_SetTorque(&leftJoint[0], 0);
    // Motor_SetTorque(&leftJoint[1], 0);
    // Motor_SetTorque(&leftWheel, 0);
    // Motor_SetTorque(&rightJoint[0], 0);
    // Motor_SetTorque(&rightJoint[1], 0);
    // Motor_SetTorque(&rightWheel, 0);

    //完成动作
    standupState = StandupState_Standup;

    vTaskDelete(NULL);
}
void Ctrl_StandupPrepareTask(void *arg)
{
	standupState = StandupState_Prepare;

	//将左腿向后摆
	Motor_SetTorque(&leftJoint[1], 0.3f);
	Motor_SetTorque(&leftJoint[0], 0.3f);
	Motor_SetTorque(&leftWheel, -0.1);
    //将右腿向前摆
	Motor_SetTorque(&rightJoint[0], -0.3f);
	Motor_SetTorque(&rightJoint[1], -0.3f);
	Motor_SetTorque(&rightWheel, 0.1);
	while(leftLegPos.angle < M_3PI_4)
		vTaskDelay(5);
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	Motor_SetTorque(&leftWheel, 0);
	//vTaskDelay(1000);


	while(rightLegPos.angle > M_PI_4)
		vTaskDelay(5);
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	Motor_SetTorque(&rightWheel, 0.0);
	vTaskDelay(1000);

	//完成准备动作，关闭电机结束任务
	Motor_SetTorque(&leftJoint[0], 0);
	Motor_SetTorque(&leftJoint[1], 0);
	Motor_SetTorque(&leftWheel, 0);
	Motor_SetTorque(&rightJoint[0], 0);
	Motor_SetTorque(&rightJoint[1], 0);
	Motor_SetTorque(&rightWheel, 0);
	
	standupState = StandupState_Standup;

	vTaskDelete(NULL);
}


//没有起立
void CtrlBasic_Task(void *arg)
{
    const float wheelRadius = 0.0325; //m，车轮半径
    const float legMass = 0.08f; //kg，腿部质量

    TickType_t xLastWakeTime = xTaskGetTickCount();

	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.2f, 1.0f, 1.0f},
						{1.0f, 1.0f, 1.0f, 1.2f, 1.0f, 1.0f}};
	float lqrTpRatio = 0.65f, lqrTRatio = 0.30f;

	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.07f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	// 交叉步相关参数
	float thetaOffset = 0.0f; // 腿部角度偏移量
	float thetaAmplitude = 3.1415f / 6.0f; // 摆动幅度，π/6
	float thetaIncrement = 0.01f; // 每次增量
	bool increasing = true; // 角度是否在增加
    while (1)
    {
        // 未使能平衡站立：关闭所有电机输出并跳过控制计算
        if (!balanceEnabled)
        {
            Motor_SetTorque(&leftJoint[0], 0.0f);
            Motor_SetTorque(&leftJoint[1], 0.0f);
            Motor_SetTorque(&leftWheel, 0.0f);
            Motor_SetTorque(&rightJoint[0], 0.0f);
            Motor_SetTorque(&rightJoint[1], 0.0f);
            Motor_SetTorque(&rightWheel, 0.0f);
            standupState = StandupState_None;
            target.speedCmd = 0.0f;
            target.yawSpeedCmd = 0.0f;
            vTaskDelayUntil(&xLastWakeTime, 4);
            continue;
        }
		//计算状态变量
		stateVar.phi = imuData.pitch;
		stateVar.dPhi = imuData.pitchSpd;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imuData.pitch;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

        //如果正在站立准备状态，则不进行后续控制
        if(standupState == StandupState_Prepare)
        {
            vTaskDelayUntil(&xLastWakeTime, 4);
            continue;
        }
		// 交叉步实现
		if(target.crossStepEnabled && groundDetector.isTouchingGround && !groundDetector.isCuchioning)
		{
			// 更新theta偏移量
			if(increasing)
			{
				thetaOffset += thetaIncrement;
				if(thetaOffset >= thetaAmplitude)
				{
					thetaOffset = thetaAmplitude - thetaIncrement;
					increasing = false;
				}
			}
			else
			{
				thetaOffset -= thetaIncrement;
				if(thetaOffset <= -thetaAmplitude)
				{
					thetaOffset = -thetaAmplitude + thetaIncrement;
					increasing = true;
				}
			}
		}
		else
		{
			// 非交叉步模式，逐渐将偏移量归零
			if(thetaOffset > 0.01f)
				thetaOffset -= 0.01f;
			else if(thetaOffset < -0.01f)
				thetaOffset += 0.01f;
			else
				thetaOffset = 0;
		}
		//计算LQR反馈矩阵
		float kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
		if(groundDetector.isTouchingGround) //正常触地状态
		{
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 2; j++)
					k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
			}
		}
		else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
		{
			memset(k, 0, sizeof(k));
			k[1][0] = kRes[1] * -2;
			k[1][1] = kRes[3] * -10;
		}

		//准备状态变量
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//与给定量作差
		x[2] -= target.position;
		x[3] -= target.speed;

		//矩阵相乘，计算LQR输出
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

		//计算yaw轴PID输出
		PID_CascadeCalc(&yawPID, target.yawAngle, imuData.yaw, imuData.yawSpd);
		
		//设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
		if(groundDetector.isTouchingGround) //正常接地状态
		{
			Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
			Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
		}
		else //腿部离地状态，关闭车轮电机
		{
			Motor_SetTorque(&leftWheel, 0);
			Motor_SetTorque(&rightWheel, 0);
		}

		//根据离地状态修改目标腿长，并计算腿长PID输出
		PID_CascadeCalc(&legLengthPID, (groundDetector.isTouchingGround) ? target.legLength : 0.12f, legLength, dLegLength);
		//计算roll轴PID输出
		PID_CascadeCalc(&rollPID, target.rollAngle, imuData.roll, imuData.rollSpd);

		//根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
		float leftForce = legLengthPID.output + (groundDetector.isTouchingGround ? 6-rollPID.output : 0);
		float rightForce = legLengthPID.output + (groundDetector.isTouchingGround ? 6+rollPID.output : 0);
		//float leftForce = legLengthPID.output+6-rollPID.output ;
		//float rightForce = legLengthPID.output+6+rollPID.output ;	
		if(leftLegPos.length > 0.12f) //保护腿部不能伸太长
			leftForce -= (leftLegPos.length - 0.12f) * 100;
		if(rightLegPos.length > 0.12f)
			rightForce -= (rightLegPos.length - 0.12f) * 100;
		
		//计算左右腿的地面支持力
		groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imuData.zAccel);
		groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imuData.zAccel);
		//更新离地检测器数据
		static uint32_t lastTouchTime = 0;
		bool isTouchingGround = groundDetector.leftSupportForce > 3 && groundDetector.rightSupportForce > 3; //判断当前瞬间是否接地
		if(!isTouchingGround && millis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
			isTouchingGround = true;
		if(!groundDetector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
		{
			target.position = stateVar.x;
			groundDetector.isCuchioning = true;
			lastTouchTime = millis();
		}
		if(groundDetector.isCuchioning && legLength < target.legLength) //缓冲状态直到腿长压缩到目标腿长结束
			groundDetector.isCuchioning = false;
		groundDetector.isTouchingGround = isTouchingGround;

		// 修改左右腿角度差PID的目标值，实现交叉步
		float legAngleDiffTarget = 0;
		if(target.crossStepEnabled && groundDetector.isTouchingGround && !groundDetector.isCuchioning)
		{
			legAngleDiffTarget = 2 * thetaOffset; // 左右腿角度差为偏移量的2倍
		}
		
		//计算左右腿角度差PID输出
		PID_CascadeCalc(&legAnglePID, legAngleDiffTarget, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
		
		//计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
		float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
		float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
		
		//使用VMC计算各关节电机输出扭矩
		float leftJointTorque[2]={0};
		leg_vmc_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		float rightJointTorque[2]={0};
		leg_vmc_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
		
		//保护腿部角度不超限
		float leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
		float rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
		// 原阈值：leftTheta/rightTheta ±2，pitch ±1 → 调整为 ±2.5 和 ±1.5
		#define ENABLE_ANGLE_PROTECT 0
		#define PROTECT_CONDITION (ENABLE_ANGLE_PROTECT && (leftTheta < -2.5 || leftTheta > 2.5 || \
			rightTheta < -2.5 || rightTheta > 2.5 || \
			imuData.pitch > 1.5 || imuData.pitch < -1.5)) // 扩大后的腿部角度保护条件
		if(PROTECT_CONDITION) //当前达到保护条件
		{
			if(standupState == StandupState_None) //未处于起立过程中
			{
				//关闭所有电机
				Motor_SetTorque(&leftWheel, 0);
				Motor_SetTorque(&rightWheel, 0);
				Motor_SetTorque(&leftJoint[0], 0);
				Motor_SetTorque(&leftJoint[1], 0);
				Motor_SetTorque(&rightJoint[0], 0);
				Motor_SetTorque(&rightJoint[1], 0);
				//阻塞等待腿部角度回到安全范围，再等待4s后恢复控制(若中途触发了起立则在起立准备完成后直接跳出)
				while(PROTECT_CONDITION && standupState == StandupState_None)
				{
					leftTheta = leftLegPos.angle - imuData.pitch - M_PI_2;
					rightTheta = rightLegPos.angle - imuData.pitch - M_PI_2;
					vTaskDelay(100);
				}
				if(standupState == StandupState_None)
					vTaskDelay(2000);
				//退出保护后设定目标位置和yaw角度为当前值
				target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
				target.yawAngle = imuData.yaw;
				continue;
			}
			if(standupState == StandupState_Standup && (leftTheta < -1 || rightTheta > 1))
				standupState = StandupState_None;
		}
		else
		{
			if(standupState == StandupState_Standup) //未达到保护条件且处于起立过程中，说明起立完成，退出起立过程
				standupState = StandupState_None;
		}  

		//设定关节电机输出扭矩
		Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
		Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
		Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
		Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}

void Ctrl_Init(void)
{
	PID_Init(&rollPID.inner, 0.5, 0, 2.5, 0, 5);
	PID_Init(&rollPID.outer, 10, 0, 0, 0, 4);
	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);

	// 提高yaw PID的输出限幅以支持更快的转向速度
	PID_Init(&yawPID.inner, 0.015, 0, 0, 0, 0.3);  // 内环输出限幅从0.1提高到0.3
	PID_Init(&yawPID.outer, 15, 0, 0, 0, 6);       // 外环输出限幅从2提高到6，P增益从10提高到15

	PID_Init(&legLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
	PID_Init(&legLengthPID.outer, 5.0f, 0, 0.0f, 0.0f, 0.5f);
	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
	PID_Init(&legAnglePID.inner, 0.04, 0, 0, 0, 1);
	PID_Init(&legAnglePID.outer, 12, 0, 0, 0, 20);
	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);

	xTaskCreate(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL);
	vTaskDelay(2);
	//xTaskCreate(Ctrl_StandupPrepareTask, "StandupPrepare_Task", 4096, NULL, 1, NULL);
	//xTaskCreate(vSinGeneratorTask, "vSinGeneratorTask", 4096, NULL, 1, NULL);
	//xTaskCreate(VMC_TestTask, "VMC_TestTask", 4096, NULL, 1, NULL);
	xTaskCreate(CtrlBasic_Task, "CtrlBasic_Task", 4096, NULL, 1, NULL);
}