#include "AutoFunction.h"
PID PIDencoder;
PID PIDgyro;

void InitialPID()
{
  PIDencoder.Kp=0.22;
  PIDencoder.Ki=0.002;
  PIDencoder.Kd=0;
  PIDencoder.error_now=0;
  PIDencoder.error_last=0;
  PIDencoder.error_previous=0;

  PIDgyro.Kp=0.8;
  PIDgyro.Ki=0.005;
  PIDgyro.Kd=0;
  PIDgyro.error_now=0;
  PIDgyro.error_last=0;
  PIDgyro.error_previous=0;

}

int updatePID(PID *pid, float error_now)//增量型pid 
{
	float Increase;	

	Increase =  pid->Kp * (error_now - pid->error_last)   //比例P
			  + pid->Ki * error_now      //积分I
			  + pid->Kd * (error_now - 2 * pid->error_last + pid->error_previous);  //微分D
	
	pid->error_previous = pid->error_last;	// 更新前次误差
	pid->error_last = error_now;		  	// 更新上次误差
	return (int)Increase;	// 返回增量
}

void PIDForward(int MoveTarget,int MaxSpeed,int AccTime,int tolerance,int Gyro0)
{
  PID *pid=NULL;
  pid = &PIDencoder;
  T1.clear();
  T2.clear();
  TAcc.clear();
  leftA.resetRotation();
  int output=0,final_output=0;
  bool inside=false;
  while(!inside || T2.time()<100)
  {
    if(abs(MoveTarget-leftA.rotation(deg))<=tolerance) inside=true;
    else T2.clear();

    output += updatePID(pid,MoveTarget-leftA.rotation(deg));
    final_output = speedlimit(output,MaxSpeed);

    if(TAcc.time()<AccTime && MoveTarget>0) //加速度限制
    {
      final_output = 100*TAcc.time()/AccTime;

      LeftWheel(final_output+1.8*(Gyro0 - GyroVal));
      RightWheel(final_output);
    }
    else 
    {
      LeftWheel(final_output+1.8*(Gyro0 - GyroVal));
      RightWheel(final_output);
    }

    wait(6,msec);

    if(T1.time()>5000) break;
  }
LeftWheel(0);
RightWheel(0);
}

void PIDTurnTo(int RotateTarget,float tolerance,int MaxSpeed )
{
  PID *pid=NULL;
  pid = &PIDgyro;
  T1.clear();
  T2.clear();
  int output=0,final_output=0;
  bool inside=false;
  while(!inside || T2.time()<150)
  {
    if(abs(RotateTarget-GyroVal)<=tolerance) inside=true;
    else T2.clear();

    output += updatePID(pid, RotateTarget-GyroVal);
    final_output = speedlimit(output,MaxSpeed);
    Rotate(final_output);
    wait(10,msec);
    if(T1.time()>5000) break;
  }
Rotate(0);
}


int OutPut_limit=0;

TPID TPIDEncoder;
TPID TPIDGyro;


void InitialTPID()
{
	TPIDEncoder.Kp=0.21;//0.214;//0.190;//0.5*0.6+0.05;
	TPIDEncoder.Ki=0;//0.0015;//0.002;//110;
	TPIDEncoder.Kd=0;//0.05;//0.173;//0.5*0.6*0.12*110;
    
  TPIDGyro.Kp=0.5;
  TPIDGyro.Ki=0.002;
  TPIDGyro.Kd=0;
}


void UpdatePIDController(TPID *PIDController, int Error,int tolerance)
{
    
	PIDController->Error = Error;

	if (abs(Error)<=tolerance) PIDController->insidetarget=true;
	else {PIDController->insidetarget=false; T2.clear();}

	if (abs(Error)<30 && Error!=0) PIDController->Integral += PIDController->Error;
	else PIDController->Integral=0;

	PIDController->Derivative = PIDController->Error - PIDController->LastError;
	PIDController->LastError  = PIDController->Error;

	PIDController->Output  =  (int)((float) PIDController->Error * PIDController->Kp)
	+   ((float)PIDController->Integral   * PIDController->Ki)
	+   ((float)PIDController->Derivative * PIDController->Kd);
}

//////////////////////////////////////////////////////////////////////////
///////////////////////////////PID Controller////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void Forward_time(int speed,int time)
{
  LeftWheel2(speedlimit(speed,100));
  RightWheel2(speedlimit(speed,100));
  wait(time, msec);
  leftA.stop(hold);
  leftB.stop(hold);
  rightA.stop(hold);
  rightB.stop(hold);
}

void Forward_distance(float goal,float maxspeed,float endspeed)
{
  TAcc.clear();
  rightA.resetRotation();
  if(endspeed==0) endspeed =20;
  int OutPut = 0, value_now = 0;
  while(1)
  {
    value_now = rightA.rotation(vex::rotationUnits::deg);
    if(TAcc.time()<500)  //limit the acceleration
    {
      OutPut=maxspeed*TAcc.time()/500;
    }
    else
    {
      OutPut = (endspeed-maxspeed)/goal * value_now + maxspeed;
    }
    
    if(value_now>=goal)
    {
      Move(0);
      break;
    }
    Move(OutPut);
  }
  Move(0);
}

void PIDForward(int MoveTarget,float tolerance,int MaxSpeed,int Gyro0 )
{ 
    int wait=500;
    int maxspeed = 0;
    TPID *PID = NULL;
    PID=&TPIDEncoder;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    TAcc.clear();
    rightA.resetRotation();
    rightB.resetRotation();
    Controller1.Screen.clearScreen();
    while(PID->insidetarget==false||T2.time()<100)
    {
        UpdatePIDController( PID,  MoveTarget - rightA.rotation(vex::rotationUnits::deg),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        
        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            OutPut_limit=100*TAcc.time()/wait;
            //OutPut_limit=sign(OutPut_limit)*OutPut_limit*OutPut_limit/100;
        }
        if(MoveTarget>0)
        {
          LeftWheel2(OutPut_limit+1*(Gyro0 - GyroVal));
          //LeftWheel2(OutPut_limit);
          RightWheel2(OutPut_limit);
        }
        else if(MoveTarget<0)
        {
          Move(OutPut_limit);
        }
        maxspeed = max(maxspeed,PID->Output);
        vex::task::sleep(6);
        if (T1.time()>5000) break;

        
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("goal=%d",MoveTarget);
        Controller1.Screen.newLine();
        Controller1.Screen.print("A=%2f",rightA.rotation(vex::rotationUnits::deg));
        Controller1.Screen.newLine();
        Controller1.Screen.print("B=%2f",rightB.rotation(vex::rotationUnits::deg));
        
    }
    
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Finish!,max=%d",maxspeed);
    
    Move(0);
}
void PIDForward_1111(int MoveTarget,float tolerance,int MaxSpeed)
{ 
    int wait=500;
    int maxspeed = 0;
    TPID *PID = NULL;
    PID=&TPIDEncoder;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    TAcc.clear();
    rightA.resetRotation();
    rightB.resetRotation();
    Controller1.Screen.clearScreen();
    while(PID->insidetarget==false||T2.time()<100)
    {
        UpdatePIDController( PID,  MoveTarget - rightA.rotation(vex::rotationUnits::deg),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        
        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            OutPut_limit=100*TAcc.time()/wait;
            //OutPut_limit=sign(OutPut_limit)*OutPut_limit*OutPut_limit/100;
        }
        if(MoveTarget>0)
        {
          LeftWheel2(OutPut_limit);
          //LeftWheel2(OutPut_limit);
          RightWheel2(OutPut_limit);
        }
        else if(MoveTarget<0)
        {
          Move(OutPut_limit);
        }
        maxspeed = max(maxspeed,PID->Output);
        vex::task::sleep(6);
        if (T1.time()>5000) break;

        
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("goal=%d",MoveTarget);
        Controller1.Screen.newLine();
        Controller1.Screen.print("A=%2f",rightA.rotation(vex::rotationUnits::deg));
        Controller1.Screen.newLine();
        Controller1.Screen.print("B=%2f",rightB.rotation(vex::rotationUnits::deg));
        
    }
    
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Finish!,max=%d",maxspeed);
    
    Move(0);
}
void PIDForward_3(int MoveTarget,float tolerance,int MaxSpeed ,int Gyro0 )
{ 
    int wait=500;
    int maxspeed = 0;
    TPID *PID = NULL;
    PID=&TPIDEncoder;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    TAcc.clear();
    rightA.resetRotation();
    rightB.resetRotation();
    Controller1.Screen.clearScreen();
    while(PID->insidetarget==false||T2.time()<100)
    {
        UpdatePIDController( PID,  MoveTarget - rightA.rotation(vex::rotationUnits::deg),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        
        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            OutPut_limit=100*TAcc.time()/wait;
            //OutPut_limit=sign(OutPut_limit)*OutPut_limit*OutPut_limit/100;
        }
        if(MoveTarget>0)
        {
          LeftWheel2(OutPut_limit+1*(Gyro0 - GyroVal));
          //LeftWheel2(OutPut_limit);
          RightWheel2(OutPut_limit);
        }
        else if(MoveTarget<0)
        {
          Move(OutPut_limit);
        }
        maxspeed = max(maxspeed,PID->Output);
        vex::task::sleep(6);
        if (T1.time()>5000) break;

        
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("goal=%d",MoveTarget);
        Controller1.Screen.newLine();
        Controller1.Screen.print("A=%2f",rightA.rotation(vex::rotationUnits::deg));
        Controller1.Screen.newLine();
        Controller1.Screen.print("B=%2f",rightB.rotation(vex::rotationUnits::deg));
        
    }
    
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Finish!,max=%d",maxspeed);
    
    Move(0);
}

void PIDForward2(int MoveTarget,float tolerance,int MaxSpeed,int wait_change,int Gyro0 )
{ 
    int wait=500;
    tolerance = 15;
    TPID *PID = NULL;
    PID=&TPIDEncoder;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    TAcc.clear();
    rightA.resetRotation();

    while(PID->insidetarget==false||T2.time()<100)
    {
        UpdatePIDController( PID,  MoveTarget - rightA.rotation(vex::rotationUnits::deg),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        
        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            OutPut_limit=OutPut_limit*TAcc.time()/wait;
            //OutPut_limit=sign(OutPut_limit)*OutPut_limit*OutPut_limit/100;
        }
        {
          if(rightA.rotation(vex::rotationUnits::deg)<wait_change)
          {
            Move(OutPut_limit);
          }
          else
          {
            LeftWheel(OutPut_limit);
            RightWheel(OutPut_limit-1*(Gyro0 - GyroVal));
          }
        }
        
        vex::task::sleep(6);
        if (T1.time()>5000) break;
    }
    //Controller1.Screen.print("goal=%d",MoveTarget);
    //Controller1.Screen.newLine();
    //Controller1.Screen.print("A=%2f",rightMotorA.rotation(vex::rotationUnits::deg));
    //Controller1.Screen.newLine();
    //Controller1.Screen.print("B=%2f",rightMotorB.rotation(vex::rotationUnits::deg));
    Move(0);

}



void TPIDTurnTo(int RotateTarget,float tolerance,int MaxSpeed )
{     
    TPID *PID=NULL;
    PID=&TPIDGyro;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    Controller1.Screen.clearScreen();
    while(PID->insidetarget==false||T2.time()<200)
    {
        UpdatePIDController( PID,  RotateTarget - GyroVal,tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        Rotate(OutPut_limit);
        vex::task::sleep(10);//10
        if (T1.time()>5000) break;
        
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("goal=%d",RotateTarget);
        Controller1.Screen.newLine();
        Controller1.Screen.print("Gyro=%2f",GyroVal);
        
        
    }
    Controller1.Screen.newLine();
    Controller1.Screen.print("Finish !");
    
    Rotate(0);
}

/*
void auto15()
{

  //改动
  //1.	去注释后PID部分的名称重合问题。分别为TPID/PID
  //2.	函数重载错误，PIDTurnTo函数对于TPID/PID 更名为TPIDTurnTo

  //缺失
  //0.  执行动作函数func 需要补充
  //1.  动作状态机的实现 在6月24日程序里
  //2.  PID带状态机的实现函数 


  //可用
  //vex::task Task1 = vex::task(func);
  //转：TPIDTurnTo(int RotateTarget,float tolerance=2,int MaxSpeed =50)
  //PIDTurnTo(int RotateTarget,float tolerance=2,int MaxSpeed =100)
  //直行：
  //按时间：Forward_time(int speed,int time)
  //按距离：Forward_distance(float goal,float maxspeed=100,float endspeed=0)
  //TPID：区别
  //PIDForward(int MoveTarget,float tolerance=10,int MaxSpeed = 95,int Gyro0 = GyroVal) TPID
  //PIDForward2(int MoveTarget,float tolerance=8.0,int MaxSpeed = 95,int wait_change = 0,int Gyro0 = GyroVal)
  //PIDForward_3(int MoveTarget,float tolerance=10,int MaxSpeed = 20,int Gyro0 = GyroVal)
  //PIDForward_1111(int MoveTarget,float tolerance=10,int MaxSpeed = 95)
  //PID：
  //PIDForward(int MoveTarget,int MaxSpeed=95,int AccTime=500,int tolerance=8,int Gyro0 = GyroVal) PID






  //初始化
  InitialPID();
  InitialTPID();

  //陀螺仪初始化矫正 pre_auton

  //action


  //

}
*/
