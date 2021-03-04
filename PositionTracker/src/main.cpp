/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lukec                                            */
/*    Created:      Tue Nov 03 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// leftfront            motor         20              
// leftback             motor         13              
// rightfront           motor         14              
// rightback            motor         18              
// inertia              inertial      10              
// strafeencoder        encoder       E, F            
// vertencoder          encoder       C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

//positiont tracking
int xPos = 0;
int yPos = 0;

//debug variables
int yG = 0;
int xG = 0;
int xposG = 0;
int yposG = 0;
int debugX = 0;
int debugY = 0;

bool moving = false;


//Change these to put in waypoints for the robot
const int positions[5][2] = 
{
  {0, 3100},
  {-3000, 3100},
  {-2000, 200},
  {-1000, 1000},
  {0, 0}
};

int rotatedPoints[5][2]; //keeps track of the points once the robot has to make a rotation

//For debugging the different variables
void display() {
  while(true) {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Vertencoder: %f", vertencoder.position(degrees));

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Strafeencoder: %f", strafeencoder.position(degrees));

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Is moving: %f", moving);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Next Y Position: %d", yG);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Next X Position: %d", xG);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("X Position: %d", xposG);

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Y Position: %d", yposG);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("X math value: %d", debugX);

    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("Y math value: %d", debugY);

    wait(15, msec);
    Brain.Screen.clearScreen();

  }
}

//Iterates though every index in the rotatedPoints array and rotates the point by the amount that the robot is turning
void rotate(int degree, int cx, int cy) {
  int arraySize = sizeof(rotatedPoints) / sizeof(rotatedPoints[0]);

  double degrees = ((degree) * (3.145926/180)); //converting the degrees into radians so that sin and cos works

  for(int i=0; i<arraySize; i++) {
      int x = rotatedPoints[i][0];
      int y = rotatedPoints[i][1];

      //rotating the point in the array around the current x y position of the robot (Math found online)
      rotatedPoints[i][0] = round(cos(degrees) * (x - cx) - sin(degrees) * (y - cy) + cx);
      rotatedPoints[i][1] = round(sin(degrees) * (x - cx) + cos(degrees) * (y - cy) + cy);
    }
}

//copies all of the x y positions from the positions array to the rotatedPoints array so that the user doesn't have to do that manually
void initRotations() {
  int arraySize = sizeof(positions) / sizeof(positions[0]);

  for(int i=0; i<arraySize; i++){
    rotatedPoints[i][0] = positions[i][0];
    rotatedPoints[i][1] = positions[i][1];
  }
}

//method for easily calculating distance between two points
float distanceXY(int x, int y, int x2, int y2) {
  float dx = x - x2;
  float dy = y - y2;

  return sqrt(dx*dx + dy*dy);
}


//For ramping up and ramping down
float calculateSpeed(float distanceToFinish, float endGoal) {
  float rampUpK = .1;
  float rampDownK = .1;

  float halfWay = endGoal / 2;

  float perc = 0;

  if(distanceToFinish < halfWay){
    //ramp up
    float left = endGoal - distanceToFinish;
    perc = endGoal / left+1;
    perc *= perc;
    perc *= rampUpK;
  }
  else {
    //ramp down
    perc = endGoal / distanceToFinish+1;
    perc *= perc;
    perc *= rampDownK;
  }

  return perc;
}


//Move to: takes in a point index number and a speed value and travels to that point based off of its current x y position
void moveTo(int posIndex, float speed, bool usePSpeed) {
  //resetting the encoders
  vertencoder.setPosition(0, degrees);
  strafeencoder.setPosition(0, degrees);

  //getting the points that the user wanted
  int x = rotatedPoints[posIndex][0];
  int y = rotatedPoints[posIndex][1];

  xG = x;
  yG = y;

  moving = true; //for debugging... not really useful in any way. Feel free to delete
  
  //Remembering where the robot started for calculations
  int startPosX = xPos;
  int startPosY = yPos;

  float originalSpeed = speed; //saving the original speed so that the robot can manipulate it's speed without going too fast or too slow

  //storing the x y positions in new variables so that the code doesn't use the main variables
  int currentX = xPos; 
  int currentY = yPos;

  //moving until the distance between the x and y points are less than 10 plus the speed divided by 10. This value can be lowered, but the robot might overshoot and wobble a bit. It can also be raised, but the robot will stop moving further from the point but with less chance of wobbling
  while(!(abs(x-currentX) < 10 + (originalSpeed/10) && abs(y-currentY) < 10 + (originalSpeed/10))) {
    //Creating a 2D vector for the robot to travel on to get to the assigned point
    double xValue = (x - currentX);
    double yValue = (y - currentY);

    debugX = x;
    debugY = y;

    //Normalizing the vector so that the code can have full control over the robot's speed 
    float length = sqrt(xValue * xValue + yValue * yValue);

    xValue /= length;
    yValue /= length;
    
    //Creating the motor speeds. (replacing x and y controller input with xValue and yValue since they mimic controller input)
    double frontLeft = (double)((yValue + xValue));
    double backLeft = (double)((yValue - xValue));
    double frontRight = (double)((yValue - xValue));
    double backRight = (double)((yValue + xValue));

    //calculating the ramp up and down speeds
    float currentProgress = distanceXY(currentX, currentY, startPosX, startPosY);
    float totalProgress = distanceXY(startPosX, startPosY, x, y);

    speed = originalSpeed;
    if(usePSpeed)
      speed *= calculateSpeed(currentProgress, totalProgress);
    else{
      if(distanceXY(xPos, yPos, x, y) <= 300) {
        speed = originalSpeed / 4;
      }
      else {
        speed = originalSpeed;
      }
    }

    //applying motor speeds to motors and spinning
    leftfront.setVelocity(frontLeft * speed, vex::velocityUnits::pct);
    leftback.setVelocity(backLeft * speed, vex::velocityUnits::pct);
    rightfront.setVelocity(frontRight * speed, vex::velocityUnits::pct);
    rightback.setVelocity(backRight * speed, vex::velocityUnits::pct);
    
    leftfront.spin(forward);
    leftback.spin(forward);
    rightfront.spin(forward);
    rightback.spin(forward);

    //updating the positions so that the robot knows how far away it is from the assigned point
    currentX = vertencoder.position(degrees) + startPosX;
    currentY = strafeencoder.position(degrees) + startPosY;

    //for debugging
    xposG = xPos;
    yposG = yPos;
  }

  //setting the x y position of the robot to the points it just traveled to
  xPos = rotatedPoints[posIndex][0];
  yPos = rotatedPoints[posIndex][1];

  //updatting the debugging variables one last time
  xposG = xPos;
  yposG = yPos;

  //stopping the motors
  leftfront.stop();
  leftback.stop();
  rightfront.stop();
  rightback.stop();
  moving = false; //for debugging
}

void rightinertialturn(double goaldegrees)
{
  //recallibrating the inertial senser
  inertia.calibrate();
  while (inertia.isCalibrating()) {
    wait(.3, seconds);
  }

  //using the calculateSpeed method to ramp up and down the turn speed
  while(inertia.rotation(degrees) < goaldegrees)
  {
    float speed = calculateSpeed(inertia.rotation(degrees), goaldegrees);
    speed *= 40;

    leftfront.setVelocity(speed, vex::velocityUnits::pct);
    leftback.setVelocity(speed, vex::velocityUnits::pct);
    rightfront.setVelocity(-speed, vex::velocityUnits::pct);
    rightback.setVelocity(-speed, vex::velocityUnits::pct);
  
    leftfront.spin(forward);
    leftback.spin(forward);
    rightfront.spin(forward);
    rightback.spin(forward);
  }

  rotate(goaldegrees, xPos, yPos); //position tracking

  leftfront.stop();
  leftback.stop();
  rightfront.stop();
  rightback.stop();

  wait (1,seconds);
}

void leftinertialturn(double goaldegrees)
{
  goaldegrees *= -1;

  //recallibrating the inertial senser
  inertia.calibrate();
  while (inertia.isCalibrating()) {
    wait(.3, seconds);
  }

  //using the calculateSpeed method to ramp up and down the turn speed
  while(inertia.rotation(degrees) > goaldegrees)
  {
    float speed = calculateSpeed(-inertia.rotation(degrees), -goaldegrees);
    speed *= 40;

    leftfront.setVelocity(-speed, vex::velocityUnits::pct);
    leftback.setVelocity(-speed, vex::velocityUnits::pct);
    rightfront.setVelocity(speed, vex::velocityUnits::pct);
    rightback.setVelocity(speed, vex::velocityUnits::pct);
  
    leftfront.spin(forward);
    leftback.spin(forward);
    rightfront.spin(forward);
    rightback.spin(forward);
  }

  rotate(goaldegrees, xPos, yPos); //position tracking

  leftfront.stop();
  leftback.stop();
  rightfront.stop();
  rightback.stop();

  wait (1,seconds);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  initRotations();
  thread t(display); //for printing the different debug values to the brain's screen

  //example path
  wait(3, seconds);
  moveTo(0 ,90, true);
  wait(.5, seconds);
  rightinertialturn(90);
  wait(.5, seconds);
  moveTo(1, 60, true);
  wait(.5, seconds);
  moveTo(2, 60, true);
  wait(.5, seconds);
  moveTo(3, 60, true);
  wait(.5, seconds);
  moveTo(4, 40, true);
}
