#ifndef LEGS_H
#define LEGS_H

//腿部姿态结构体
typedef struct 
{
    float angle, length;   // rad, m
    float dAngle, dLength; // rad/s, m/s
    float ddLength;		   // m/s^2
}LegPos;

extern LegPos leftLegPos, rightLegPos; //左右腿部姿态

void Legs_Init(void);
#endif