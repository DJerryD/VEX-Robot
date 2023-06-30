#include "BasicFunction.h"
vex::timer T1,T2,TAcc,Tauto;
vex::timer T3;
vex::timer TACC;
vex::timer FINISH;
vex::timer TURN1;
vex::timer TURN2;


double abs(double v)
{
  if(v<0)
    return -v;
  return v;
}

double sgn(double output)
{
    if(output<0)
        return -1.0;
    else
        return 1.0;
}


double GyroVal;
double InitGyroVal=0;

int GyroDecoding()
{
  int T=0;
  float a=0,b=180;
  Brain.Screen.setFont(vex::prop40);
  
  while(1)
  {
    a=Gyro.orientation(yaw,degrees);
    if(a-b<-300)  T=T-1;
    else if(a-b>300)  T=T+1;
    GyroVal=a-360*T;
    b=a;
    Brain.Screen.printAt(1,40,"GYRO=%.3f",GyroVal-InitGyroVal);
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("GYRO=%.3f",GyroVal-InitGyroVal);
    wait(10, msec);
  }
}
vex::task Task1 = vex::task(GyroDecoding);


int max(int a,int b)
{
  return a>b ? a:b;
}



int sign(int a)
{
    if(a>0) return 1;
    else if(a<0) return -1;
    else return 0;
}

int speedlimit(int speed,int max)
{
    return abs(speed)>max ? sign(speed)*max : speed;
}
//左转
void LeftWheel(int speed)
{
  if(speed==0)
  {
    leftA.stop(coast);
    leftB.stop(coast);
    leftC.stop(coast);
  }
  else
  {
    leftA.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
    leftB.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
    leftC.spin(vex::directionType::rev,speed,vex::voltageUnits::volt);
  }
}

void LeftWheel2(int speed)
{
  leftA.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
  leftB.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
  leftC.spin(vex::directionType::rev,speed,vex::voltageUnits::volt);
}

void RightWheel(int speed)
{
  if(speed==0)
  {
    rightA.stop(coast);
    rightB.stop(coast);
    rightC.stop(coast);
  }
  else
  {
    rightA.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
    rightB.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
    rightC.spin(vex::directionType::rev,speed,vex::voltageUnits::volt);
  }
}

void RightWheel2(int speed)
{

  rightA.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
  rightB.spin(vex::directionType::fwd,speed,vex::voltageUnits::volt);
  rightC.spin(vex::directionType::rev,speed,vex::voltageUnits::volt);
}

void Rotate(int speed)
{
    LeftWheel2(speed);
    RightWheel2(-1*speed);
}
void Move(int speed)
{
  LeftWheel2(speed);
  RightWheel2(speed);
}

