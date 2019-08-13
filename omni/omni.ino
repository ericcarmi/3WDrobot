/* Motor control and encoder reading for 3WD omni-directional robot
   Uses the Arduino Due -- alotta digital, all of which have interrupts!
   Controls 3 motor speeds and directions.
   Receive velocity commands and send encoder data from/to RPI
*/
#include "omni3robot.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <omnirobotmessages/Arduinostate.h>

// Create the robot
Robot robot;

// ROS node and publisher
ros::NodeHandle nh;
omnirobotmessages::Arduinostate ardPub_msg;
ros::Publisher pub("arduinoPub", &ardPub_msg);

// Setup ROS subscriber with callback function
void messageCb( const omnirobotmessages::Arduinostate& ardSub_msg){
  // toggleX is a bool, telling whether or not to change some variables
  if(ardSub_msg.togglePosition)
  {
    robot.motor1.moveByN(ardSub_msg.x);
    robot.motor2.moveByN(ardSub_msg.y);
    robot.motor3.moveByN(ardSub_msg.theta);
  }
  if(ardSub_msg.toggleVelocity)
  {
    robot.vel2phi(ardSub_msg.vel_x,ardSub_msg.vel_y,ardSub_msg.vel_theta);
  }
  if(ardSub_msg.toggleServo)
  {
    robot.vision.setPosition(ardSub_msg.servoangle);
  }
  if(ardSub_msg.toggleFunction)
  {
    if(ardSub_msg.functionSelect == 1)
    {
      robot.outlineBasicPathTime(ardSub_msg.functionIntParam,50,ardSub_msg.functionFloatParam);
    }
    if(ardSub_msg.functionSelect == 2)
    {
      robot.calibrateWithPaths(ardSub_msg.functionIntParam,50,ardSub_msg.functionFloatParam);
    }
  }
  // Select one of the built-in functions to execute
  // Various calibration modes


}
ros::Subscriber<omnirobotmessages::Arduinostate> sub("arduinoSub", &messageCb );

// vx = 80, phi = 28.87
// vx = 70, phi = 25.26
// vx = 10, phi = 3.61
//         dphi = 0.361
// vtheta = 10, phi = 47.92
// vmax = dphi*706
// change v by about 3*phi to change by 1 pwm

int P = 4;
const float sq3b2 = sqrt(3)/2.0;
float n1 = 0.0f;
float n2 = 0.0f;
float n3 = 0.0f;
float n11 = 0.0f;
float n22 = 0.0f;
float n33 = 0.0f;
float n1q = 0.0f;
float n2q = 0.0f;
float n3q = 0.0f;


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  robot.motor1 = *new Motor();
  robot.motor2 = *new Motor();
  robot.motor3 = *new Motor();

  robot.vision = *new Vision();
  robot.vision.Setup();
 // robot.vision.

  robot.motor1.setencoderPins(45,43);
  robot.motor2.setencoderPins(53,51);
  robot.motor3.setencoderPins(49,47);
  // Forward, Backward, PWM
  robot.motor1.setdriverPins(24,22,4); // Below USB Battery, camera -- This is the front. Forward is to the robot's right
  robot.motor2.setdriverPins(28,26,3); // Near motor drivers, back right motor
  robot.motor3.setdriverPins(32,30,2); // Below arduino, back left

  // Attach encoder pins to interrupts so that they are read quickly enough to get good data
  // Numrotations or velocity of the motor can be accessed in the loop
  attachInterrupt(digitalPinToInterrupt(robot.motor1.encoderpins[0]), m1EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor1.encoderpins[1]), m1EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor2.encoderpins[0]), m2EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor2.encoderpins[1]), m2EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor3.encoderpins[0]), m3EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor3.encoderpins[1]), m3EncoderB, CHANGE);

  robot.vision.setPosition(70);
}

void loop() {

  // Update robot time
  robot.currentTime = millis();
  robot.samplePeriod = 0.001f * (float)(robot.currentTime - robot.lastTime);
  robot.lastTime = robot.currentTime;
  //robot.floatTime = Ts;

  // Update Kalman filter and controls
  robot.DRflag = robot.motor1.DRflag or robot.motor2.DRflag or robot.motor3.DRflag;
  if(robot.DRflag)
  {
    robot.updateDR();
  }

  if(robot.pathFlag)
  {
    robot.currentPathTime += robot.samplePeriod;
    robot.checkPathTime();
  }
  robot.getPosition();
  // Get variables for publishing
  ardPub_msg.x = (int)robot.motor1.numrotations;
  ardPub_msg.y = (int)robot.motor2.numrotations;
  ardPub_msg.theta = (int)robot.motor3.numrotations;
  ardPub_msg.vel_x = (robot.motor1.numrotations - n1)/robot.samplePeriod;
  ardPub_msg.vel_y = (robot.motor2.numrotations - n2)/robot.samplePeriod;
  ardPub_msg.vel_theta = (robot.motor3.numrotations - n3)/robot.samplePeriod;
  ardPub_msg.servoangle = robot.vision.angle;
  pub.publish(&ardPub_msg);
  nh.spinOnce();
  n1 = robot.motor1.numrotations;
  n2 = robot.motor2.numrotations;
  n3 = robot.motor3.numrotations;
}


// Should all encoders be updated whenever any of them are change?
// Could also be called from the robot struct, so update position estimation at the same time
void m1EncoderA()
{
  robot.motor1.encoderstates[1] = robot.motor1.encoderstates[0];
  robot.motor1.encoderstates[0] = !robot.motor1.encoderstates[0];
  robot.motor1.updateencoderState();
}
void m1EncoderB()
{
  robot.motor1.encoderstates[3] = robot.motor1.encoderstates[2];
  robot.motor1.encoderstates[2] = !robot.motor1.encoderstates[2];
  robot.motor1.updateencoderState();
}
void m2EncoderA()
{
  robot.motor2.encoderstates[1] = robot.motor2.encoderstates[0];
  robot.motor2.encoderstates[0] = !robot.motor2.encoderstates[0];
  robot.motor2.updateencoderState();
}
void m2EncoderB()
{
  robot.motor2.encoderstates[3] = robot.motor2.encoderstates[2];
  robot.motor2.encoderstates[2] = !robot.motor2.encoderstates[2];
  robot.motor2.updateencoderState();
}
void m3EncoderA()
{
  robot.motor3.encoderstates[1] = robot.motor3.encoderstates[0];
  robot.motor3.encoderstates[0] = !robot.motor3.encoderstates[0];
  robot.motor3.updateencoderState();
}
void m3EncoderB()
{
  robot.motor3.encoderstates[3] = robot.motor3.encoderstates[2];
  robot.motor3.encoderstates[2] = !robot.motor3.encoderstates[2];
  robot.motor3.updateencoderState();
}
