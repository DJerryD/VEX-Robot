#ifndef _BASICFUNCTION_H
#define _BASICFUNCTION_H
#include "vex.h"

//底盘移动基础程序
#define HONG 1
#define LAN 2
extern vex::timer T1,T2,TAcc,Tauto;
extern vex::timer T3;
extern vex::timer TACC;
extern vex::timer FINISH;
extern vex::timer TURN1;
extern vex::timer TURN2;
extern double GyroVal;
extern double InitGyroVal;
double abs(double v);
double sgn(double output);
int GyroDecoding();
int max(int a,int b);
int sign(int a);
int speedlimit(int speed,int max=80);


void LeftWheel(int speed);
void LeftWheel2(int speed);
void RightWheel(int speed);
void Move(int speed);
void Rotate(int speed);
void RightWheel2(int speed);
#endif