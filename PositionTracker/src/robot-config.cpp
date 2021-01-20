#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial inertia = inertial(PORT1);
rotation vertencoder = rotation(PORT2, false);
rotation strafeencoder = rotation(PORT3, false);
motor leftfront = motor(PORT4, ratio18_1, false);
motor leftback = motor(PORT5, ratio18_1, false);
motor rightfront = motor(PORT6, ratio18_1, false);
motor rightback = motor(PORT7, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}