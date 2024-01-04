#include "radio.h"
#include "pid_ir.h"
#include "drive.h"

#define MY_ROBOT_ID 1

//global variables
long irDistancePinBack = A9;
long irDistancePinFront = A8;
float xpos;
float ypos;
float theta;
float closerHyp;
float dropAngle;
int ballHunt;
int team = 0;
int dropZone = 0; //boundary to drop off balls

//function initializations
float newBallHyp(int i);
float currentAngle();
float robotToBallAngle(int j);
void align(float alignAngle);

RobotPose robot; //initialize a simpler RobotPose variable

void setup() {
  Serial.begin(115200);
  ME401_Radio_initialize(); // Initialize the RFM69HCW radio

  //initialize distance sensor inputs
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(36, INPUT);

  setupDrive(); //initialize our drive function
  setupPIDandIR(); //initialize PID and IR interrupt
}

void loop() {
  theta = currentAngle();
  updateRobotPoseAndBallPositions();

  switch (state) {
    case (MAIN):
      runMainState();
      break;
    case (FRONT):
      runFrontState();
      break;
    case (BACK):
      runBackState();
      break;
    case (HUNTING):
      runHuntingState();
      break;
    case (RETRIEVE):
      runRetrieveState();
      break;
    case (SNITCH):
      runSnitchState();
      break;
    default:
      Serial.println("UNKNOWN STATE");
      break;
  }
}

//state functions
void runMainState() {
  Serial.println("MAIN STATE");
  RobotPose robot = getRobotPose(MY_ROBOT_ID);

  //team selection
  if (digitalRead(36) == HIGH)
    team = 1; // on beacon
  else if (digitalRead(36) == LOW)
    team = 2; // no beacon

  int seen = 0;
  int beacon = 0;
  delay(2000);
  
  //team selection function
  if (team == 1) { //determines which side we are on if team 1
    while (sweep()) {} //run sweep function until IR is seen
    theta = currentAngle();

    SoftPWMServoServoWrite(32, 1500);
    //Serial.println("TO BEACON");

    if (theta < 90 && theta > -90) {
      Serial.println("Left side");
      dropAngle = 0;
      dropZone = 200;
      delay(1000);
    }
    else if (theta > 90 || theta < -90) {
      Serial.println("Right side");
      dropAngle = 180;
      dropZone = 1750;
      delay(1000);
    }
  }
  else if (team == 2) { //determines which side we are on if team 2
    while (sweep()) {} //run sweep function until IR is seen
    theta = currentAngle();

    SoftPWMServoServoWrite(32, 1500);
    //Serial.println("Not our BEACON");

    if (theta > 90 || theta < -90) {
      Serial.println("Left side");
      dropAngle = 0;
      dropZone = 200;

      //Serial.println("Go to right side");
    }
    else if (theta < 90 && theta > -90) {
      Serial.println("Right side");
      dropAngle = 180;
      dropZone = 1750;
    }
  }
  Serial.print("Backed off"); //found side
  delay(1000);

  BallPosition ballPos[20];
  int numBalls = getBallPositions(ballPos);
  Serial.print("NUM BALLS: ");
  Serial.println(numBalls);
  printBallPositions(numBalls, ballPositions);

  state = HUNTING;
}

void runFrontState() { //backs robot up if front limit switch is activated
  Serial.println("FRONT STATE");
  moveBackward(75);
  delay(250);
  stopMove();
  state = SaveState;
}

void runBackState() { //moves robot forward if back limit switch is activated
  Serial.println("BACK STATE");
  moveForward(75);
  delay(250);
  stopMove();
  state = SaveState;
}

void runHuntingState() { //finds a ball to approach
  Serial.println("HUNTING STATE");

  float ballHyp = 99999; // initialize a large number as robot-ball distance
  for (int j = 0; j < numBalls; j++) { //loop to obtain shortest distance between robot and ball
    updateRobotPoseAndBallPositions();
    RobotPose robot = getRobotPose(1);

    if (newBallHyp(j) < ballHyp) { // get balls only from one side only depending on team
      if (dropAngle == 0 && ballPositions[j].x > 650) {
        ballHyp = newBallHyp(j);
        ballHunt = j; //ball number to hunt
      }
      else if (dropAngle == 180 && ballPositions[j].x < 1750) {
        ballHyp = newBallHyp(j);
        ballHunt = j; //ball number to hunt
      }
    }
  }
  detectSnitch(); //check if snitch is in arena
  state = RETRIEVE;
}

void runRetrieveState() { //approaches ball and brings it back to our side
  Serial.println("RETRIEVE STATE");

  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);

  //local variables for IR distance sensors and robot-to-ball angle
  int irDistanceValueBack = analogRead(irDistancePinBack);
  int irDistanceValueFront = analogRead(irDistancePinFront);
  float realDistanceBack = (irDistanceValueBack * 0.0615); - 18.4;
  float realDistanceFront = (irDistanceValueFront * 0.0513); - 27.385;
  float useAngle = robotToBallAngle(ballHunt);

  if (useAngle > 10 || useAngle < -10)
    align(useAngle); //turn to angle
  if ((useAngle <= 10 && useAngle >= -10) && newBallHyp(ballHunt >= 300))
    approachBall();
  if ((useAngle <= 10 && useAngle >= -10) && newBallHyp(ballHunt < 300))
    moveToBall();// moves forward on top of ball
  if (realDistanceBack < 7.5 || realDistanceFront < 7.5) {
    servoArmPosition = 55;
    servoArm.write(servoArmPosition);
    dropBall(); // drops ball off at designated location
  }
  detectSnitch(); //check if snitch is in arena
}

void runSnitchState() {
  Serial.println("SNITCH STATE");
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);

  //local variables for IR distance sensors and robot-to-ball angle
  int irDistanceValueBack = analogRead(irDistancePinBack);
  int irDistanceValueFront = analogRead(irDistancePinFront);
  float realDistanceBack = (irDistanceValueBack * 0.0615) - 18.4;
  float realDistanceFront = (irDistanceValueFront * 0.0513) - 27.385;
  float useAngle = robotToBallAngle(ballHunt);

  if (useAngle > 10 || useAngle < -10)
    align(useAngle); //turn to angle
  if ((useAngle <= 10 && useAngle >= -10) && newBallHyp(ballHunt >= 300))
    approachBall();
  if ((useAngle <= 10 && useAngle >= -10) && newBallHyp(ballHunt < 300))
    moveToBall();// moves forward on top of ball
  if (realDistanceBack < 7.5 || realDistanceFront < 7.5) {
    servoArmPosition = 55;
    servoArm.write(servoArmPosition); //captures the ball
    stopMove();
  }
}

//iterative calculation functions
float newBallHyp(int i) { //calculates hypotenuse between ball and robot
  float diffX = abs(robotPoses[1].x - ballPositions[i].x);
  float diffY = abs(robotPoses[1].y - ballPositions[i].y);
  float hyp = sqrt(sq(diffX) + sq(diffY));
  return hyp;
}
float currentAngle() {
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);
  theta = (180 * (robot.theta)) / (1000 * PI);
  return theta;
}
float robotToBallAngle(int j) { //returns the angle between the robot and the ball
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);
  theta = (PI * currentAngle()) / 180; // converting to radians --> used below
  xpos = cos(theta) * (robot.x - ballPositions[j].x) + sin(theta) * (robot.y - ballPositions[j].y);
  ypos = -sin(theta) * (robot.x - ballPositions[j].x) + cos(theta) * (robot.y - ballPositions[j].y);
  float ballAngle = (180 * atan2(ypos, xpos)) / PI;
  return ballAngle;
}
float sweep() { //sweeps the IR sensor to register which side the IR beacon is on
  if (readIRFrequency() != CORNER) {
    //Serial.print("Can't see corner:");
    //Serial.println(frequency);

    setpoint += 5; // increments by 5
    //Serial.println(setpoint);
    delay (100);
    if (setpoint > 50) { // will rotate up to 50 degree
      setpoint = -30; // make turn
      Serial.println(setpoint);
    }
    //Serial.println("DC"); // sweep functionality
    if (setpoint == -20) {
      // turn robot
      //Serial.print("TURN");
      SoftPWMServoServoWrite(32, 1650);
      SoftPWMServoServoWrite(33, 1650);
      delay(250);
      SoftPWMServoServoWrite(32, 1500);
      SoftPWMServoServoWrite(33, 1500);
    }
  }
  else if (readIRFrequency() == CORNER)
  {
    Serial.println("Corner found");
    delay(100);
    SoftPWMServoServoWrite(32, 1500);
    SoftPWMServoServoWrite(33, 1500);
    return false;
  }
  return true;
  delay(100);
}

//iterative movement and detection functions
void detectSnitch() { //function to determine if a snitch is on the field
  BallPosition ballPos[20];
  int numBallsColors = getBallPositions(ballPos);
  float ballHyp = 99999;
  for (int j = 0; j < numBallsColors; j++) {
    updateRobotPoseAndBallPositions();
    RobotPose robot = getRobotPose(1);
    if (ballPositions[j].hue < 30 ) {
      Serial.println("SNITCH DETECTED");
      ballHyp = newBallHyp(j);
      ballHunt = j;
      state = SNITCH;
    }
  }
}
void align(float alignAngle) { //align robot to a zero angle
  if (alignAngle < 50 && alignAngle > 10)
    turnLeft(8);
  else if (alignAngle > -50 && alignAngle < -10)
    turnRight(8);
  else if (alignAngle >= 50 && alignAngle <= 150)
    turnLeft(12);
  else  if (alignAngle <= -50 && alignAngle >= -150)
    turnRight(12);
  else  if ((alignAngle < -150 && alignAngle >= -180) || (alignAngle > 150 && alignAngle <= 180))
    turnRight(12);
  else
    stopMove();
  return;
}
void approachBall() { //incrementally approach ball by half the distance between robot and ball
  closerHyp = newBallHyp(ballHunt);
  if (closerHyp >= newBallHyp(ballHunt) / 2) {
    moveForward(50);
    state = RETRIEVE;
  }
}
void moveToBall() { //move forward at a set rate to help the IR distance sensors detect ball 
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);
  if (newBallHyp(ballHunt) <= 25) {
    moveForward(25);
    delay(1000);
  }
  else if (newBallHyp(ballHunt) <= 50) {
    moveForward(30);
    delay(1000);
  }
  else if (newBallHyp(ballHunt) <= 100) {
    moveForward(35);
    delay(1000);
  }
  else if (newBallHyp(ballHunt) <= 150) {
    moveForward(50);
    delay(1000);
  }
  else if (newBallHyp(ballHunt) <= 200) {
    moveForward(45);
    delay(1000);
  }
  return;
}
void dropBall() { //face the side for our team, move to it, and drop the ball off
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);
  theta = currentAngle();
  if (dropAngle == 0) {
    FaceLeft();
    moveToDropZone();
  }
  else if (dropAngle == 180) {
    FaceRight();
    moveToDropZone();
  }
  servoArmPosition = 80;
  servoArm.write(servoArmPosition); //releases ball
  moveBackward(50);
  delay(1000);
  state = HUNTING;
}
void moveToDropZone() { //approach the designated drop zone until it is reached
  updateRobotPoseAndBallPositions();
  RobotPose robot = getRobotPose(1);

  if (team == 1 && dropZone == 200) {
    while (robot.x > 200 ) {
      updateRobotPoseAndBallPositions();
      robot = getRobotPose(1);
      moveForward(50);
    }
  }
  else if (team == 1 && dropZone == 1750) {
    while (robot.x < 1750 ) {
      updateRobotPoseAndBallPositions();
      robot = getRobotPose(1);
      moveForward(50);
    }
  }
  else if (team == 2 && dropZone == 1750) { // go right
    while (robot.x < 1750) {
      updateRobotPoseAndBallPositions();
      robot = getRobotPose(1);
      moveForward(50);
    }
  }
  else if (team == 2 && dropZone == 200) { // go left
    while (robot.x > 200) {
      updateRobotPoseAndBallPositions();
      robot = getRobotPose(1);
      moveForward(50);
    }
  }
  stopMove();
  delay(100);
}
void FaceRight() { //make robot face right side of field
  while (theta > -170 && theta < 170) {
    theta = currentAngle(); // update positions
    if (theta > 0)
      turnLeft(10);
    else if (theta < 0)
      turnRight(10);
  }
  stopMove();
}
void FaceLeft() { //make robot face left side of field
  while (theta < -10 || theta > 10) {
    theta = currentAngle(); // update positions
    if (theta < 0)
      turnLeft(10);
    else if (theta > 0)
      turnRight(10);
  }
  stopMove();
}
