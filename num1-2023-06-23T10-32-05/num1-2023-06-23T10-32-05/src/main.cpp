#include "vex.h"
#include "includes.h"
// ---- START VEXCODE CONFIGURED DEVICES ----
/*
接口说明：见robot-config.cpp
操作说明：
底盘双摇杆操作：左Y控制前后，右X控制左右
L1 intake吸飞盘，L2 intake反转
R1发射飞盘
组合键，防误操作
UP+B 弹射伸展
=================================
以下调试时用，比赛中不能按

Down+B 弹射活塞复位
Down+R2 取消Shooter自动复位,此时按R2操作撞针
调试阶段，LEFT键启动自动

*/

// ---- END VEXCODE CONFIGURED DEVICES ----
using namespace vex;

competition Competition;

//多线程显示区域
void pre_auton(void) 
{
  vexcodeInit();
  // InitialPID();
  Open.set(false);
  Open1.set(false);
  shootpre();
  Gyro.calibrate();
  wait(2,seconds);
  
}

//自动模式
void autonomous(void) 
{
  auto1();
}


//手动模式
void usercontrol(void) {
  while (1) 
  {
    // UserCollect();//手动程序
    Chassis();//底盘控制程序
    // thread t(Shoot);//多线程控制撞针程序
    fashe();//发射
     if(Controller1.ButtonA.pressing())
Open.set(false);

     if(Controller1.ButtonB.pressing())
Open.set(true);
 if(Controller1.ButtonX.pressing())
 itk.spin(forward, -100, pct);
 else 
 itk.spin(forward, 0, pct);
    wait(5, msec);  
  }

}

int main() {
  // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
