#include "includes.h"
#include <iostream>
//可用子程序格式
/*
collectspin(转筒角度);//输入对应的角度，吸盘马达旋转相应的度数
shootpre();//发射撞针复位，回到最底部
shoot();//撞针发射一次
just_run(底盘速度)//直线行走，但是不会停止
just_run_straight(底盘速度,前进角度);//输入对应的角度，底盘会按照陀螺仪反馈的角度前进
just_stop(刹车模式);//1:代表半刹车，0代表滑行
timed_run(前进时间,前进速度);//输入对应的秒数，机器会以指定的速度移动相应的时间
improved_pid_move(前进距离,前进角度);//PID前进
new_gyro_pid_turn(转弯角度,最大速度);//PID转弯
intake(吸盘速度);//吸盘
*/

void auto1(){
//   Tauto.clear(); //计时器清零
//   intake(-80);
//   improved_pid_move(68,0);
// intake(0);
//   new_gyro_pid_turn(34); 
//   improved_pid_move(35, 34);
//   shoot();
//   shootpre();
//   intake(-80);
//   improved_pid_move(10, 34);
//   improved_pid_move(-10, 34);
//   wait(1500, msec);
//   shoot();
//   //shootpre();
//   intake(-80);
//   new_gyro_pid_turn(142);
//   improved_pid_move(135, 142);
//    intake(0);
//   //new_gyro_pid_turn(180);
//   improved_pid_move(15, 180);
//   collectspin(-600); 
//   std::cout << "自动结束时间：" << Tauto.time() << std::endl;
}
void auto2() {
//   Tauto.clear();
//   just_stop(1);
//   std::cout << "开始吧" << std::endl;
//   collectspin(600); //转动滚桶
//   std::cout << "开始吧" << std::endl;
//   improved_pid_move(-10, 0); //前进距离，加角度
//   new_gyro_pid_turn(-102); //转动到多少度
//   intake(-70);//70%的速度吸飞盘
//   improved_pid_move(124, -102);//在-102度的方向前进124cm
//   improved_pid_move(-6, -102);//后退6cm
//   intake(0);//吸球停
//   new_gyro_pid_turn(-214);//继续转，转到-214度
//   improved_pid_move(76, -214);//在-214度方向前进76cm
//   wait(500, msec);//等停稳了再射球
//   shoot();//发射
//   shootpre();//撞针恢复
//   intake(-100);//启动吸飞盘
//   improved_pid_move(10, -214);//前进10cm
//  improved_pid_move(-10, -214);//后退10cm
//   wait(2000, msec);//等待飞盘进入
//   shoot();//发射
//   std::cout << "自动结束时间：" << Tauto.time() << std::endl;
}


  