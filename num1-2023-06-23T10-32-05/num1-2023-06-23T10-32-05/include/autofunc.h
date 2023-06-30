#ifndef _AUTOFUNC_H
#define _AUTOFUNC_H
//int steps=0;

void collectspin(int deg);
void intake(int speed);

void shootpre();

void shoot();

void just_run(int output);

double return_angle(int dec_flag=0);

/*
just run
无PID
包含一个方向偏移
转弯

*/
void just_run_straight(double output,double newgyro);
void just_right_run_straight(double output,double newgyro);
void just_stop(int stopmod=1);
void just_right_spin(int output);
/*
just run
*/
void timed_run(double time,int v);
/*
PID
某方向平移
转弯
直行
*/
//dec_point 末尾限速的开始位置 change_steps 执行的动作
//start_point 动作执行的位置
void improved_pid_move(double aim,int newgyro,double speed_limit=90,double speed_limit2=40,int dec_point=-1,int change_steps=-1,int start_point=0);
/*
转弯
PID和直接转
*/
void new_gyro_turn(double aim,int max_speed=100,int switch_left=1,int switch_right=1);
void stop();
void new_gyro_pid_turn(double aim,int max_speed=100,int dec_flag=0);
void run_a_distance_but_not_stop(int distance,int speed,double direction,int step=0,int action_distance=-1,bool no_acc=false);
//动作设定

#endif
