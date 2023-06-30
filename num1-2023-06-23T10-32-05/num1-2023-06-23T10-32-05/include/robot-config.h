using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor INTAKEL;
extern motor INTAKER;
extern motor itk;
extern motor HAHAHA;
extern motor HAHAHAR;
extern motor leftA;
extern motor leftB;
extern motor leftC;
extern motor rightA;
extern motor rightB;
extern motor rightC;
extern motor shoot_L;
extern motor shoot_R;

extern inertial Gyro;
extern distance Distance1;
extern controller Controller1;

extern digital_out Open;
extern digital_out Open1;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );