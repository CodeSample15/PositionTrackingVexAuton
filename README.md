# PositionTrackingVexAuton

Robot configuration: X drive with two odometers, one for horizontal and one for verticle tracking.


Usage

Update the following arrays in the source code:

const int positions[?][2] = {}; //put in desired x y waypoints here

int rotatedPoints[?][2]; //the only thing that you need to change here is the size of the array. Make the ? the same value as the ? in the positions array.


To use the code, call the moveTo() function with the index of the waypoint you want to move to(located in the positions array) and the speed of the robot.
For turning, use either rightinertialturn or leftinertialturn with the amount that you want the robot to turn and the speed you want it to move at as arguments.

The last parameter of the moveTo() function is a boolean where you can tell the robot to use a ramp up and ramp down function for moving (true), or simply slow down when it reaches a certain distance from the end goal and keep a constant speed the rest of the time(false)
