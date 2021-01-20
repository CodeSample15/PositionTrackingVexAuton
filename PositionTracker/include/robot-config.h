using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial inertia;
extern rotation vertencoder;
extern rotation strafeencoder;
extern motor leftfront;
extern motor leftback;
extern motor rightfront;
extern motor rightback;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );