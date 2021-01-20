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

int xPos = 0;
int yPos = 0;
int yawValue = 0;
int yG = 0;
int xG = 0;
int xposG = 0;
int yposG = 0;
int debugX = 0;
int debugY = 0;

bool moving = false;

const int positions[5][2] = 
{
  {0, 3100},
  {-3000, 3100},
  {-2000, 200},
  {-1000, 1000},
  {0, 0}
};

int rotatedPoints[5][2];

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
    Brain.Screen.print("Yaw Value: %d", yawValue);

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("X Position: %d", xposG);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Y Position: %d", yposG);

    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("X math value: %d", debugX);

    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("Y math value: %d", debugY);

    wait(15, msec);
    Brain.Screen.clearScreen();

  }
}

void rotate(int degree, int cx, int cy) {
  int arraySize = sizeof(rotatedPoints) / sizeof(rotatedPoints[0]);

  double degrees = ((degree) * (3.145926/180));

  for(int i=0; i<arraySize; i++) {
      int x = positions[i][0];
      int y = positions[i][1];

      rotatedPoints[i][0] = round(cos(degrees) * (x - cx) - sin(degrees) * (y - cy) + cx);
      rotatedPoints[i][1] = round(sin(degrees) * (x - cx) + cos(degrees) * (y - cy) + cy);
    }
}

void initRotations() {
  int arraySize = sizeof(positions) / sizeof(positions[0]);

  for(int i=0; i<arraySize; i++){
    rotatedPoints[i][0] = positions[i][0];
    rotatedPoints[i][1] = positions[i][1];
  }
}

float distanceXY(int x, int y, int x2, int y2) {
  float dx = x - x2;
  float dy = y - y2;

  return sqrt(dx*dx + dy*dy);
}

void moveTo(int posIndex, float speed) {
  //resetting the encoders
  vertencoder.setPosition(0, degrees);
  strafeencoder.setPosition(0, degrees);

  int x = rotatedPoints[posIndex][0];
  int y = rotatedPoints[posIndex][1];

  xG = x;
  yG = y;

  moving = true;
  
  //calculating motor speeds and direction
  int startPosX = xPos;
  int startPosY = yPos;

  float originalSpeed = speed;

  int currentX = xPos;
  int currentY = yPos;

  while(!(abs(x-currentX) < 7 && abs(y-currentY) < 7)) {
    double xValue = (x - currentX);
    double yValue = (y - currentY);

    debugX = x;
    debugY = y;

    //Normalizing the vector
    float length = sqrt(xValue * xValue + yValue * yValue);

    xValue /= length;
    yValue /= length;
    
    //applying those values to the motors
    double frontLeft = (double)((yValue + xValue));
    double backLeft = (double)((yValue - xValue));
    double frontRight = (double)((yValue - xValue));
    double backRight = (double)((yValue + xValue));

    if(distanceXY(currentX, currentY, x, y) <= 500) {
      speed = originalSpeed / 4;
    }
    else {
      speed = originalSpeed;
    }

    leftfront.setVelocity(frontLeft * speed, vex::velocityUnits::pct);
    leftback.setVelocity(backLeft * speed, vex::velocityUnits::pct);
    rightfront.setVelocity(frontRight * speed, vex::velocityUnits::pct);
    rightback.setVelocity(backRight * speed, vex::velocityUnits::pct);
    
    leftfront.spin(forward);
    leftback.spin(forward);
    rightfront.spin(forward);
    rightback.spin(forward);

    currentX = vertencoder.position(degrees) + startPosX;
    currentY = strafeencoder.position(degrees) + startPosY;

    xposG = xPos;
    yposG = yPos;
  }

  xPos = rotatedPoints[posIndex][0];
  yPos = rotatedPoints[posIndex][1];

  xposG = xPos;
  yposG = yPos;

  leftfront.stop();
  leftback.stop();
  rightfront.stop();
  rightback.stop();
  moving = false;
}

void rightinertialturn(double goaldegrees)
{
  inertia.calibrate();
  while (inertia.isCalibrating()) {
    wait(.3, seconds);
  }

  leftfront.setVelocity(20, vex::velocityUnits::pct);
  leftback.setVelocity(20, vex::velocityUnits::pct);
  rightfront.setVelocity(20, vex::velocityUnits::pct);
  rightback.setVelocity(20, vex::velocityUnits::pct);
  
  while(inertia.rotation(degrees) < goaldegrees)
  {
    leftfront.spin(forward);
    leftback.spin(forward);
    rightfront.spin(reverse);
    rightback.spin(reverse);
  }


  rotate(goaldegrees, xPos, yPos);

  leftfront.stop();
  leftback.stop();
  rightfront.stop();
  rightback.stop();

  wait (1,seconds);
}

void leftinertialturn(double goaldegrees)
{
  goaldegrees *= -1;

  inertia.calibrate();
  while (inertia.isCalibrating()) {
    wait(.3, seconds);
  }

  leftfront.setVelocity(20, vex::velocityUnits::pct);
  leftback.setVelocity(20, vex::velocityUnits::pct);
  rightfront.setVelocity(20, vex::velocityUnits::pct);
  rightback.setVelocity(20, vex::velocityUnits::pct);

  while(inertia.rotation(degrees) > goaldegrees)
  {
    leftfront.spin(reverse);
    leftback.spin(reverse);
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
  thread t(display);

  wait(3, seconds);
  moveTo(0 ,50);
  wait(.5, seconds);
  rightinertialturn(90);
  wait(.5, seconds);
  moveTo(1, 40);
  wait(.5, seconds);
  moveTo(2, 40);
  wait(.5, seconds);
  moveTo(3, 50);
  wait(.5, seconds);
  moveTo(4, 30);
}
