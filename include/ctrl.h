#ifndef CTRL_H
#define CTRL_H

#include "stdio.h"
#include "pid.h"
typedef struct 
{
   	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
	float target; // m
	bool crossStepEnabled; // 交叉步使能标志
}Target;


typedef struct 
{
	float theta, dTheta;
	float x, dx;
	float phi, dPhi;
} StateVar;

//站立过程状态枚举量
typedef enum  {
	StandupState_None,
	StandupState_Prepare,
	StandupState_Standup,
} StandupState;

typedef struct GroundDetector
{
    float leftSupportForce, rightSupportForce;
    bool isTouchingGround, isCuchioning;
} GroundDetector;


extern Target target;
extern StateVar stateVar;
extern StandupState standupState;
extern bool balanceEnabled; // 平衡站立使能标志

void Ctrl_Init(void);
void StandUp(void);

#endif