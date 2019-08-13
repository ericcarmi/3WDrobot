#include "Motor.h"
#include "vision.h"
//#include "path.h"
# define sqrt3by2 0.8660254037844385965883021

struct Robot{
    Motor motor1, motor2, motor3;
    // Motor1 odometer increases moving to the right; 2 and 3 when moving forward

    Vision vision;

    float posx, posy, theta, vel_x, vel_y, vel_theta;
    float ang2 = -2*PI/3; // CW from motor 1 which is at front
    float ang3 = 2*PI/3;  // CCW
    float phi[3];
    float directionvector[2] = {0.0,1.0};
    float R = 0.116; // Branch length, from center to wheel
    float wheelradius = 0.024;  // 48mm diameter
    unsigned long currentTime, lastTime;
    float phiScaleFactor = 1.5;
    int localDRcount = 0;
    int drtemp1 = 0;
    int drtemp2 = 0;
    int minPWM = 1;
    float floatTime = 0.0f;
    float samplePeriod = 0.0f;
    float pathEndTime = 0.0f;
    float currentPathTime = 0.0f;
    bool DRflag;
    bool pathFlag = false;
    int pwmdelta1 = 0;
    int pwmdelta2 = 0;
    int pwmdelta3 = 0;

    // Directions for paths as described is orthogonal to wheel motion and direction pins
    int basicPaths[10][6] = {   {0,1,1,0,1,0},  // 0 Forward
                               {0,1,1,0,0,1},   // 1 Back
                               {2,1,1,1,0,0},   // 2 Left
                               {2,1,1,0,1,1},   // 3 Right
                               {1,0,1,1,0,0},   // 4 Forward left
                               {1,0,1,0,0,1},   // 5 Back right
                               {1,1,0,0,1,0},   // 6 Forward right
                               {1,1,0,1,0,0},   // 7 Back left
                               {1,1,1,0,0,0},
                               {1,1,1,1,1,1}};


    void outlineBasicPath(int x,int pwm)
    {
        motor1.setPWM(basicPaths[x][0]*pwm);
        motor2.setPWM(basicPaths[x][1]*pwm);
        motor3.setPWM(basicPaths[x][2]*pwm);
        motor1.setDirection(basicPaths[x][3]);
        motor2.setDirection(basicPaths[x][4]);
        motor3.setDirection(basicPaths[x][5]);

    }

    void outlineBasicPathTime(int x,int pwm, float t)
    {
        motor1.setPWM(basicPaths[x][0]*pwm);
        motor2.setPWM(basicPaths[x][1]*pwm);
        if(x == 2 or x == 3)
        {
          motor3.setPWM(basicPaths[x][2]*pwm-29);
        }
        else{motor3.setPWM(basicPaths[x][2]*pwm);}
        motor1.setDirection(basicPaths[x][3]);
        motor2.setDirection(basicPaths[x][4]);
        motor3.setDirection(basicPaths[x][5]);
        pathEndTime = t;
        currentPathTime = 0.0f;
        pathFlag = true;
    }

    void pathTransition(int x1, int x2, int pwm, float t)
    {
        pwmdelta1 = basicPaths[x2][0] - basicPaths[x1][0];
        pwmdelta2 = basicPaths[x2][1] - basicPaths[x1][1];
        pwmdelta3 = basicPaths[x2][2] - basicPaths[x1][2];

    }

    void checkPathTime()
    {
      if( (int)currentPathTime > (int)pathEndTime )
      {
        stopMoving();
        pathFlag = false;
        currentPathTime = 0.0f;
      }
    }

    void stopMoving()
    {
        motor1.setPWM(0);
        motor2.setPWM(0);
        motor3.setPWM(0);
    }

    // Get offset from encoders over one path time
    void calibrateWithPaths(int x, int pwm, float t)
    {
      int n1 = motor1.numrotations;
      int n2 = motor2.numrotations;
      int n3 = motor3.numrotations;
      outlineBasicPathTime(x, pwm, t);
      int n1b = motor1.numrotations;
      int n2b = motor2.numrotations;
      int n3b = motor3.numrotations;

      if(x == 0 or x == 1)
      {
        int delta = abs(n2b - n2) - abs(n3b - n3);

        if( delta > 0) // Motor 2 changed more
        {
            motor3.pwmOffset = delta*0.1;
        }
        else if(delta < 0)
        {
          motor2.pwmOffset = delta*0.1;
        }
      }


    }

    void moveByN(int N1, int N2, int N3)
    {
        motor1.moveByN(N1);
        motor2.moveByN(N2);
        motor3.moveByN(N3);
        DRflag = true;
    }

    void setPWM(int x1, int x2, int x3)
    {
      if( x1 > 0){motor1.setDirection(HIGH);}
      else{motor1.setDirection(LOW);}
      if( x2 > 0){motor2.setDirection(HIGH);}
      else{motor2.setDirection(LOW);}
      if( x3 > 0){motor3.setDirection(HIGH);}
      else{motor3.setDirection(LOW);}
      motor1.setPWM(abs(x1));
      motor2.setPWM(abs(x2));
      motor3.setPWM(abs(x3));
    }

    void updateDR()
    {
        motor1.updateDR(floatTime);
        motor2.updateDR(floatTime);
        motor3.updateDR(floatTime);

    }
    // Convert desired x,y and angular velocity into rotational velcoities
    // Theta should always be 0? Local orientation is fixed, changing theta offsets everything
    void vel2phi(float vx, float vy, float vtheta)
    {
        float temp1 = cos(theta);
        phi[0] = (vy + R*vtheta)/wheelradius;
        phi[1] = (sqrt3by2*vx - 0.5*vy + R*vtheta)/wheelradius;
        phi[2] = (-sqrt3by2*vx - 0.5*vy + R*vtheta)/wheelradius;

        if(phi[0] > 0){ motor1.setDirection(HIGH);}
        else{motor1.setDirection(LOW);}
        if(phi[1] > 0){ motor2.setDirection(HIGH);}
        else{motor2.setDirection(LOW);}
        if(phi[2] > 0){ motor3.setDirection(HIGH);}
        else{motor3.setDirection(LOW);}

        motor1.setPWM(abs(phi[0]));
        motor2.setPWM(abs(phi[1]));
        motor3.setPWM(abs(phi[2]));

    }


    void getPosition()
    {
        posx =  -2*sin(PI/3)*motor2.numrotations + 2*cos(PI/6)*motor3.numrotations;
        posy = 2*motor1.numrotations + cos(PI/3)*motor2.numrotations - 2*sin(PI/6)*motor3.numrotations;
        theta = R * (motor1.numrotations + motor2.numrotations + motor3.numrotations);

    }

    // This is a measurement that can be compared with the desired for PID control

    void setPosition(float targetx, float targety, float targettheta)
    {
        float temp1 = cos(theta);
        // Calculate number of rotations for each motor
        phi[0] = (targety + R*targettheta)/wheelradius;
        phi[1] = (sqrt3by2*targetx - 0.5*targety + R*targettheta)/wheelradius;
        phi[2] = (-sqrt3by2*targetx - 0.5*targety + R*targettheta)/wheelradius;

        motor1.moveByN(phi[0]);
        motor2.moveByN(phi[1]);
        motor3.moveByN(phi[2]);

        motor1.DRflag=true;
        motor2.DRflag=true;
        motor3.DRflag=true;
        DRflag=true;
    }
    // This is a calibration function for determining the min PWM value to achieve movement, varies with terrain
    // Test values and increase until movement is detected
    void getMinPWM()
    {
        motor1.temprotations = motor1.numrotations;
        for(int i = 0; i < 50; i++)
        {
            motor1.setPWM(i);
            delay(10);
            if((motor1.temprotations - motor1.numrotations) > 0)
            {
               minPWM = i;
               break;
            }
        }
    }

    void tracePaths()
    {
      int P = 4;
      int pwm = 40;
    if(floatTime < P)
  {
    outlineBasicPath(0,pwm);
  }
  if(floatTime < 2*P && floatTime > P)
  {
    outlineBasicPath(1,pwm);
  }

  if(floatTime < 3*P && floatTime > 2*P)
  {
    outlineBasicPath(2,pwm);
  }
  if(floatTime < 4*P && floatTime > 3*P)
  {
    outlineBasicPath(3,pwm);
  }
  if(floatTime > 4*P)
  {
    floatTime = 0.0f;
  }
    }

};
