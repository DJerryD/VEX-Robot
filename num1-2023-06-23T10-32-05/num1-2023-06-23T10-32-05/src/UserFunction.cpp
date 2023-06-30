#include "UserFunction.h"

// void Chassis()
// {
//   double ch3value=Controller1.Axis3.position(percent);
//   double ch4value=Controller1.Axis1.position(percent);//单遥感，双摇杆
// //防止摇杆死区
//   if(fabs(ch3value)<10)
//   {
//     ch3value=0;
//   }
//   if(fabs(ch4value)<10)
//   {
//     ch4value=0;
//   }
// //旋转
//   LeftWheel(ch3value+ch4value);
//   RightWheel(ch3value-ch4value);
// }

void Chassis()
{
  double ch3value = Controller1.Axis3.position(percent);
  double ch4value = Controller1.Axis1.position(percent); // 单遥感，双摇杆
  
  // 防止摇杆死区
  if (fabs(ch3value) < 10)
  {
    ch3value = 0;
  }
  if (fabs(ch4value) < 10)
  {
    ch4value = 0;
  }
  
  // 映射到电机速度范围
  double maxSpeed = 1.0; // 设置最大速度值，可以根据实际情况进行调整
 // int leftSpeed = static_cast<int>((ch3value + ch4value) * maxSpeed / 100.0);
 // int rightSpeed = static_cast<int>((ch3value - ch4value) * maxSpeed / 100.0);
 double leftSpeed = (ch3value + ch4value) * 12/100.0;
double rightSpeed =(ch3value - ch4value) * 12/100.0;
  
  
  // 控制电机运动
  LeftWheel(leftSpeed*maxSpeed);
  RightWheel(rightSpeed*maxSpeed);
}




//发射
int dis;
void shootpre(){
  dis = Distance1.objectDistance(vex::distanceUnits::mm);
  while (dis > 60) {
    HAHAHA.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    HAHAHAR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    wait(100, msec);
    HAHAHA.stop();
    HAHAHAR.stop();
    dis = Distance1.objectDistance(vex::distanceUnits::mm); // 更新 dis 的值
  }
  HAHAHA.stop(vex::brakeType::brake);
  HAHAHAR.stop(vex::brakeType::brake);
}
void fashe(){
  if(Controller1.ButtonR1.pressing()){
    dis = Distance1.objectDistance(vex::distanceUnits::mm);
     while (dis > 17) {
    HAHAHA.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    HAHAHAR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    wait(100, msec);
    HAHAHA.stop();
    HAHAHAR.stop();
    dis = Distance1.objectDistance(vex::distanceUnits::mm); // 更新 dis 的值
    }
    HAHAHA.stop(vex::brakeType::brake);
    HAHAHAR.stop(vex::brakeType::brake);
    wait(500, msec);
    HAHAHA.spinFor(forward,60,degrees,100,velocityUnits::pct,false);
    HAHAHAR.spinFor(forward,60,degrees,100,velocityUnits::pct);
    wait(500, msec);
    shootpre();
  }
}
