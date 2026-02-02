#include "legs.h"
#include "motor.h"
#include <esp_task_wdt.h>

#include "../include/matlab_code/leg_position.h"
#include "../include/matlab_code/leg_speed.h"

LegPos leftLegPos, rightLegPos; //左右腿部姿态

//腿部姿态更新任务(根据关节电机数据计算腿部姿态)
void LegPos_UpdateTask(void *arg)
{
	const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		float legPos[2], legSpd[2];

		//计算左腿位置
		leg_position(leftJoint[1].angle, leftJoint[0].angle, legPos);
		leftLegPos.length = legPos[0];
		leftLegPos.angle = legPos[1];

		//计算左腿速度
		leg_speed(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
		leftLegPos.dLength = legSpd[0];
		leftLegPos.dAngle = legSpd[1];

		// //计算左腿腿长加速度
		leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
		lastLeftDLength = leftLegPos.dLength;

		//计算右腿位置
		leg_position(rightJoint[1].angle, rightJoint[0].angle, legPos);
		rightLegPos.length = legPos[0];
		rightLegPos.angle = legPos[1];

		//计算右腿速度
		leg_speed(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
		rightLegPos.dLength = legSpd[0];
		rightLegPos.dAngle = legSpd[1];

		// //计算右腿腿长加速度
		rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
		lastRightDLength = rightLegPos.dLength;

		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次,上面那个需要换单位到s，同时时间差为4ms
	}
}

void Legs_Init(void)
{
	xTaskCreate(LegPos_UpdateTask, "LegPos_UpdateTask", 4096, NULL, 2, NULL);
	vTaskDelay(2);
}