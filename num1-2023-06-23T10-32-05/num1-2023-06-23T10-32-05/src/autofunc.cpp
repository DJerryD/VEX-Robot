#include "autofunc.h"
int steps = 0;
#include "BasicFunction.h"
#include "vex.h"
#include <iostream>

// void collectspin(int deg) //宣传滚筒（参数：角度）
// {
//   collectL.resetRotation();
//   while (abs(collectL.rotation(vex::rotationUnits::deg)) < abs(deg)) {
//     collectL.spin(reverse, 50, percent);
//     collectR.spin(reverse, 50, percent);
//     vex::task::sleep(10);
//   }
//   collectL.stop();
//   collectR.stop();
// }

// void intake(int speed) {
//   if (speed == 0) {
//     collectL.stop();
//     collectR.stop();
//   }
//   collectL.spin(fwd, speed, pct);
//   collectR.spin(fwd, speed, pct);
// }

// void shootpre() //发射准备
// {
//   while (!LimitSwitch.pressing()) {
//     shoot_L.spin(forward, 100, percent);
//     shoot_R.spin(forward, 100, percent);
//   }
//   shoot_L.stop(brake);
//   shoot_R.stop(brake);
// }

void shoot() //发射
{
  shoot_L.spin(forward, 100, percent);
  shoot_R.spin(forward, 100, percent);
  vex::task::sleep(400); //等待500ms = 0.5s
  shoot_L.stop(brake);
  shoot_R.stop(brake);
  vex::task::sleep(80);
}

void just_run(int output) //直线行走
{
  leftA.spin(fwd, output, pct);
  leftB.spin(fwd, output, pct);
  rightA.spin(fwd, output, pct);
  rightB.spin(fwd, output, pct);
}

double return_angle(int dec_flag) {
  return Gyro.rotation(vex::rotationUnits::deg);
}

/*
just run
无PID
包含一个方向偏移
转弯
*/
void just_run_straight(double output,
                       double newgyro) //按照一定的速度，陀螺仪的角度去走
{
  double angle_err = return_angle() - newgyro;
  double Kp = 0.7;
  angle_err *= Kp;
  leftA.spin(fwd, output, pct);
  leftB.spin(fwd, output, pct);
  rightA.spin(fwd, output - angle_err, pct);
  rightB.spin(fwd, output - angle_err, pct);
}

void just_right_run_straight(double output, double newgyro) //向右平移
{
  double angle_err = return_angle() - newgyro;
  if (abs(leftA.velocity(vex::velocityUnits::pct)) < 10)
    angle_err = 0;
  double Kp = 1; // 1.5//0.7

  angle_err *= Kp;
  double outputL = output;
  double outputR = outputL - angle_err;
  leftA.spin(vex::directionType::fwd, outputL, vex::velocityUnits::pct);
  leftB.spin(vex::directionType::rev, outputR, vex::velocityUnits::pct);
  rightA.spin(vex::directionType::rev, outputL, vex::velocityUnits::pct);
  rightB.spin(vex::directionType::fwd, outputR, vex::velocityUnits::pct);
}

void just_stop(int stopmod) {
  if (stopmod == 1) {
    leftA.stop(vex::brakeType::brake);
    leftB.stop(vex::brakeType::brake);
    rightA.stop(vex::brakeType::brake);
    rightB.stop(vex::brakeType::brake);
  } else if (stopmod == 0) {
    leftA.stop(vex::brakeType::coast);
    leftB.stop(vex::brakeType::coast);
    rightA.stop(vex::brakeType::coast);
    rightB.stop(vex::brakeType::coast);
  }
  leftA.stop();
  leftB.stop();
  rightA.stop();
  rightB.stop();
}

void just_right_spin(int output) // turn
{
  leftA.spin(vex::directionType::fwd, output, vex::velocityUnits::pct);
  leftB.spin(vex::directionType::fwd, output, vex::velocityUnits::pct);
  rightA.spin(vex::directionType::rev, output, vex::velocityUnits::pct);
  rightB.spin(vex::directionType::rev, output, vex::velocityUnits::pct);
}
/*
just run
*/
void timed_run(double time, int v) // ok
{
  T1.clear();
  double angle = return_angle();
  while (T1.time() < time) {
    just_run_straight(v, angle);
  }
  just_stop(1);
}
/*
PID
某方向平移
转弯
直行
*/
// dec_point 末尾限速的开始位置 change_steps 执行的动作
// start_point 动作执行的位置
void improved_pid_move(double aim, int newgyro, double speed_limit,
                       double speed_limit2, int dec_point, int change_steps,
                       int start_point) // ok
{
  double degree = (aim / 31.4) * 828;
  double Kp = 0.08;  // 0.4
  double Ki = 0.002; // 0.001
  double Kd = 0.003; // 0.173
  double Kt = 0;     // 0.13
  double Ktv = 0;    // 0.08
  double value_now = 0;
  double EI = 0, ED = 0;
  int sampletime = 10;
  double err_now = 0;
  double err_last = 0;
  double max_v = speed_limit;

  double value_now_L = 0;
  double value_last_L = 0;
  double value_now_R = 0;
  double value_last_R = 0;
  double outputL, outputR;
  double ET = 0;
  double ETV = 0;

  double sum_dec = 0;

  double K_gyro = 0.7; // 0.7
  double angle_err = 0;

  double acc = 0.2; // 0.15

  // Set the encoder value to 0.
  leftA.resetRotation();
  rightA.resetRotation();
  T1.clear();
  T2.clear();
  T3.clear();
  TACC.clear();

  //画一个红色方块
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(100, 100, 200, 100);
  while (1) {

    max_v = acc * TACC.time();
    if (TACC.time() > 500)
      max_v = speed_limit;

    if (max_v >= speed_limit) {
      max_v = speed_limit;
    }
    if (dec_point != -1) {
      if (abs(leftA.rotation(vex::rotationUnits::deg)) > dec_point) {
        max_v =
            speed_limit -
            (abs(leftA.rotation(vex::rotationUnits::deg)) - dec_point) / 50.0;
        if (max_v < speed_limit2) {
          max_v = speed_limit2;
        }
      }
    }

    // if(abs(value_now)<100)max_v=65;
    // if(abs(aim-value_now)<100)max_v=65;

    if (change_steps != -1) {
      if (start_point <= abs(value_now_L)) {
        steps = change_steps;
        change_steps = -1;
      }
    }

    value_now = leftA.rotation(vex::rotationUnits::deg);
    if (T2.time() > 100) {
      T2.clear();
      value_last_R = value_now_R;
      value_last_L = value_now_L;
      value_now_R = rightA.rotation(vex::rotationUnits::deg);
      value_now_L = leftA.rotation(vex::rotationUnits::deg);

      ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);

      ET = value_now_R - value_now_L;
      sum_dec += Ktv * ETV;
    }
    // std::cout<<"T3:"<<T3.time()<<std::endl;
    if (T3.time() > sampletime) {
      T3.clear();
      EI = EI + err_now;
      err_last = err_now;
      err_now = degree - value_now;
      ED = err_now - err_last;
    }

    if (abs(err_now) > 100) {
      EI = 0;
    }

    outputL = Kp * err_now + Ki * EI + Kd * ED;
    // outputR=outputL-sum_dec-Kt*ET;
    if (abs(outputL) > max_v)
      outputL = sgn(outputL) * max_v;
    angle_err = return_angle() - newgyro;
    // if(TACC.time()<500)angle_err=0;

    // if(abs(LeftMotor1.velocity(vex::velocityUnits::pct))<10)angle_err=0;
    if (abs(angle_err) < 0.7)
      angle_err = 0;
    outputR = outputL - Kt * ET - Ktv * ETV + K_gyro * (angle_err);
    if (abs(outputR) > 100)
      outputR = sgn(outputR) * 100;
    if (outputL == 0)
      outputR = 0;
    leftA.spin(fwd, outputL, pct);
    leftB.spin(fwd, outputL, pct);
    rightA.spin(fwd, outputR, pct);
    rightB.spin(fwd, outputR, pct);

    // err_last_L=err_now_L;

    vex::task::sleep(sampletime);
    if (abs(err_now) > 40)
      T1.clear();
    if ((T1.time()) > 150) {
      just_stop();
      break;
    }
    // std::cout<<"angle:"<<return_angle()<<"err_now:"<<err_now<<std::endl;
    // std::cout<<"value_now:"<<value_now<<"value_now_R:"<<value_now_R<<"value_now_L:"<<value_now_L<<std::endl;
  }
  std::cout << "直线运动结束：" << std::endl;
  std::cout << "time:" << TACC.time() << std::endl;
  std::cout << "err_now:" << err_now << std::endl;
  just_stop(1);
  //画一个绿色方块
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(100, 100, 200, 100);
  // vex::task::sleep(2000);
  std::cout << "angle:" << return_angle() << std::endl;
}

/*
转弯
PID和直接转
*/
void new_gyro_turn(double aim, int max_speed, int switch_left,
                   int switch_right) {
  double err = 0;
  double speed = 0;
  double Kp =
      0.7; // 0.7=====================================================================================

  double value_now = return_angle();

  /*
  if(aim<value_now)
  {
      Kp=0.60;//0.62
  }
  */
  while (1) {
    value_now = return_angle();
    err = (value_now - aim);
    speed = Kp * (value_now - aim);
    speed = (abs(speed) > 3) ? speed : sgn(speed) * 3;
    if (abs(speed) > max_speed) {
      speed = sgn(speed) * max_speed;
    }
    leftA.spin(vex::directionType::fwd, -switch_left * speed,
               vex::velocityUnits::pct);
    rightA.spin(vex::directionType::fwd, switch_right * speed,
                vex::velocityUnits::pct);
    leftB.spin(vex::directionType::fwd, -switch_left * speed,
               vex::velocityUnits::pct);
    rightB.spin(vex::directionType::fwd, switch_right * speed,
                vex::velocityUnits::pct);
    if (abs(err) >=
        2) //=================================================================
    {
      T1.clear();
    }
    if (T1.time() > 200) // 200
    {
      break;
    }
    vex::task::sleep(20);
  }
  just_stop(0);
  // vex::task::sleep(2000);
}

void stop() {
  while (abs(leftA.velocity(vex::velocityUnits::pct)) > 2) {
    just_right_spin(-leftA.velocity(vex::velocityUnits::pct));
  }
  leftA.stop(vex::brakeType::brake);
  rightA.stop(vex::brakeType::brake);
  leftB.stop(vex::brakeType::brake);
  rightB.stop(vex::brakeType::brake);
}

void new_gyro_pid_turn(double aim, int max_speed, int dec_flag) // ok
{
  /********testing**********/
  double Kp = 1.5;   // 0.775
  double Ki = 0.005; // 0.001
  double Kd = 4;     // 3
  double err_now = 0;
  double err_last = 0;

  double value_now = 0;

  double EI = 0, ED = 0;
  double output;
  int sampletime = 10; // 10
  TACC.clear();
  T1.clear();
  /*
      if(aim<return_angle())
      {
        Kp=0.7;//0.7
        Ki=0.002;
        Kd=0.009;
      }
  */

  value_now = return_angle(dec_flag);
  err_now = aim - value_now;
  if (abs(err_now) < 100) {
    Kp = 0.775; // 0.775//=======================================================
    Ki = 0.005; // 0.001
    Kd = 4;     // 3
  }

  // Brain.Screen.drawRectangle(1,1,400,400,vex::color::red);
  while (1) {

    value_now = return_angle(dec_flag);
    err_now = aim - value_now;
    EI = EI + err_now;
    if (abs(err_now) > 15) // 15
      EI = 0;
    ED = err_now - err_last;

    output = Kp * err_now + Ki * EI + Kd * ED;
    if (abs(output) > max_speed)
      output = sgn(output) * max_speed;

    just_right_spin(output);

    err_last = err_now;

    vex::task::sleep(sampletime);
    if (abs(err_now) >
        2) //========================================================
      T1.clear();
    if ((T1.time()) > 200) // 200
    {
      just_stop();
      break;
    }
    // std::cout<<"angle:"<<return_angle()<<std::endl;
  }
  std::cout << "转弯完成:" << std::endl;
  std::cout << "time:" << TACC.time() << std::endl;
  std::cout << "angle:" << return_angle() << std::endl;
  // Brain.Screen.drawRectangle(1,1,400,400,vex::color::green);
  just_stop(1);
}

void run_a_distance_but_not_stop(int distance, int speed, double direction,
                                 int step, int action_distance, bool no_acc) {
  TACC.clear();
  double speed_limit = 0;
  double ACC = 0.2;
  leftA.resetRotation();
  while (abs(leftA.rotation(vex::rotationUnits::deg)) < abs(distance)) {
    speed_limit = sgn(speed) * ACC * TACC.time();
    if (abs(speed_limit) > abs(speed)) {
      speed_limit = speed;
    }
    if (no_acc) {
      speed_limit = speed;
    }
    just_run_straight(speed_limit, direction);
    if (action_distance != -1) {
      if (abs(leftA.rotation(vex::rotationUnits::deg)) > abs(action_distance)) {
        steps = step;
      }
    }
    vex::task::sleep(10);
  }
}

//动作设定

// int some_actions()
// {
//     while(1)
//     {
//       vex::task::sleep(10);
//       switch(steps)
//       {
//         case 0:break;
//         case 1:prepare_to_shoot();steps=0;break;
//         case 2:shoot_action();steps=0;break;
//         case 3:lift_control(0);steps=0;break;
//         case 4:lift_control(1,70);steps=0;break;//collect balls
//         case 5:lift_control(2,90);steps=0;break;//gua
//         case 6:lift_control(1,70);steps=0;break;//turn disk//85
//         case 7:vex::task::sleep(0);lift_control(0,90);steps=0;break;//shuai
//         disk//370 case 8:lift_control(3,35);steps=0;break;//prepare to lift
//         case 9:vex::task::sleep(500);lift_control(0,60);steps=0;break;//climb
//         case 10:lift_control(4,100);steps=0;break;//prepare climb
//         case
//         11:lift_control(4,60);vex::task::sleep(500);lift_control(1,70);steps=0;break;//touch
//         balls default:steps=0;break;
//       }
//     }
//     return 0;
// }
