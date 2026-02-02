#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
typedef struct 
{	float rawAngle;
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float voltage, maxVoltage; // V
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	int dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // 指向反电动势计算函数
} Motor;

extern Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象

void Motor_InitAll(void);
void Motor_Update(Motor *motor, uint8_t *data);
void Motor_SetTorque(Motor *motor, float torque);
extern float motorOutRatio; //电机输出电压比例，对所有电机同时有效

#endif