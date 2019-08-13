// vision.h
// Vision control for the mounted servo
#include <Servo.h>
Servo camservo;

struct Vision{
  int servoPin = 5;
  int offset = 0;
  int angle = 0;

  void Setup()
  {
    camservo.attach(servoPin);

  }

  void setOffset(int x)
  {
	  offset = x;
  }

  void setPosition(int x)
  {
    angle = x + offset;
	  camservo.write(angle);
  }
};
