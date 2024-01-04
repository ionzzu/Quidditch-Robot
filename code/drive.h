#include <SoftPWMServo.h>

//global variables
const int leftServoPin = 32;
const int rightServoPin = 33;

int leftServoSpeed = 0;
int rightServoSpeed = 0;
int servoArmPosition = 80;

//servo initializations
SoftServo leftServo;
SoftServo rightServo;
SoftServo servoArm;

void setupDrive(void) {
  SoftPWMServoInit();

  //attach servo
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  servoArm.attach(30);
  servoArm.write(80);
}

int rightServoMap() { //maps the right servo speed from -100 to +100
  int servoSpeed = map(rightServoSpeed, -100, 100, 1300, 1700);
  return servoSpeed;
}
int leftServoMap() { //maps the left servo speed from -100 to +100
  int servoSpeed = map(leftServoSpeed, -100, 100, 1300, 1700);
  return servoSpeed;
}

//drive functions
void moveForward(int moveSpeed) { //move forward at moveSpeed of -100 to +100
  rightServoSpeed = moveSpeed;
  leftServoSpeed = -moveSpeed;
  SoftPWMServoServoWrite(rightServoPin, rightServoMap());
  SoftPWMServoServoWrite(leftServoPin, leftServoMap());
  return;
}
void moveBackward(int moveSpeed) { //move backward at moveSpeed of -100 to +100
  rightServoSpeed = -moveSpeed;
  leftServoSpeed = moveSpeed;
  SoftPWMServoServoWrite(rightServoPin, rightServoMap());
  SoftPWMServoServoWrite(leftServoPin, leftServoMap());
  return;
}
void turnRight(int moveSpeed) { //rotate right at moveSpeed of -100 to +100
  rightServoSpeed = moveSpeed;
  leftServoSpeed = moveSpeed;
  SoftPWMServoServoWrite(rightServoPin, rightServoMap());
  SoftPWMServoServoWrite(leftServoPin, leftServoMap());
  return;
}
void turnLeft(int moveSpeed) { //rotate left at moveSpeed of -100 to +100
  rightServoSpeed = -moveSpeed;
  leftServoSpeed = -moveSpeed;
  SoftPWMServoServoWrite(rightServoPin, rightServoMap());
  SoftPWMServoServoWrite(leftServoPin, leftServoMap());
  return;
}
void stopMove() { //stop movement
  rightServoSpeed = 0;
  leftServoSpeed = 0;
  SoftPWMServoServoWrite(rightServoPin, rightServoMap());
  SoftPWMServoServoWrite(leftServoPin, leftServoMap());
  return;
}
