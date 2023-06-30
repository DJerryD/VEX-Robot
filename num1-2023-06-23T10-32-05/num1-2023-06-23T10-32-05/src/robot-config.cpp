#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
//蓝色：6：1，红色：36：1，绿色：18：1
//motor INTAKEL = motor(PORT7, ratio18_1, false);
//motor INTAKER= motor(PORT7, ratio18_1, true);//收球电机
motor itk = motor(PORT11, ratio18_1, false);
motor HAHAHA = motor(PORT19, ratio18_1, false);
motor HAHAHAR = motor(PORT20, ratio18_1, true);//发射准备电机

motor leftA = motor(PORT12, ratio6_1, true);//左前电机
motor leftB = motor(PORT17, ratio6_1, true);//左中电机
motor leftC = motor(PORT18, ratio6_1, true);//左后电机（无动力传动所以）
motor rightA = motor(PORT13, ratio6_1, false);//右前电机
motor rightB = motor(PORT15, ratio6_1, false);//右中电机
motor rightC = motor(PORT14, ratio6_1, false);//右后电机（无动力传动所以需要反转）

inertial Gyro = inertial(0);//陀螺仪
distance Distance1 = distance(PORT16);//距离传感器
controller Controller1 = controller(primary);//遥控器

digital_out Open = digital_out(Brain.ThreeWirePort.H);//发射
digital_out Open1 = digital_out(Brain.ThreeWirePort.B); //收球抬起


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}