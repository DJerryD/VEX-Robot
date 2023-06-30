#ifndef _AUTOFUNCTION_H
#define _AUTOFUNCTION_H

#include "BasicFunction.h"

typedef struct
{ 
  float Kp,Ki,Kd;
  float error_now;
  float error_last;
  float error_previous;
}PID;
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int Error=0;
	int Integral=0;
	int Derivative=0;
	int LastError=0;
	int Output=0;
	bool insidetarget=0;
}TPID;

void InitialPID();

int updatePID(PID *pid, float error_now);

void PIDForward(int MoveTarget,int MaxSpeed=95,int AccTime=500,int tolerance=8,int Gyro0 = GyroVal);
void PIDTurnTo(int RotateTarget,float tolerance=2,int MaxSpeed =100);
void InitialTPID();

void UpdatePIDController(TPID *PIDController, int Error,int tolerance);
void Forward_time(int speed,int time);

void Forward_distance(float goal,float maxspeed=100,float endspeed=0);

void PIDForward(int MoveTarget,float tolerance=10,int MaxSpeed = 95,int Gyro0 = GyroVal);

void PIDForward_1111(int MoveTarget,float tolerance=10,int MaxSpeed = 95);

void PIDForward_3(int MoveTarget,float tolerance=10,int MaxSpeed = 20,int Gyro0 = GyroVal);

void PIDForward2(int MoveTarget,float tolerance=8.0,int MaxSpeed = 95,int wait_change = 0,int Gyro0 = GyroVal);

void TPIDTurnTo(int RotateTarget,float tolerance=2,int MaxSpeed =50);
 #endif