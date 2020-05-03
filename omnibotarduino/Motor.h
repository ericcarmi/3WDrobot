struct Motor{
  int encoderpins[2];
  int driverpins[3];
  int forward;
  int pwm;
  int encoderstates[4]; // A(k), A(k-1), B(k), B(k-1)
  int numrotations;
  int lastnumrotations;
  int inverserotations = 0;
  int temprotations;
  int diffnumrotations = 0;
  int encoderdirection;
  int encoderphase[2];
  int encoderphaseD;
  bool DRflag = false;
  float oneRotation = 567.0f; // Number of encoder clicks
  float wheelDistance = PI * 4.8/oneRotation;
  // 48 mm wheel diameter
  unsigned long currentTime;
  unsigned long lastTime;
  float velocity = 0.0f;
  float velocity_1 = 0.0f;
  float velocity_mem[5] = {0.0f,0.0f,0.0f,0.0f,0.0f};
  float vel_avg = 0.0f;
  float dv1 = 0.0f;
  float dt = 0.0f;
  int zerocounter = 0;

  // Filter variables for smoothing
  float filtin = 0.0f;
  float filtin_1 = 0.0f;
  float filtout = 0.0f;
  float filtout_1 = 0.0f;
  // First order Butterworth lowpass, cutoff at 0.01*pi
  float b0 =  0.0154662914031034;
  float b1 = b0;
  float a0 = 1.0f;
  float a1 = -0.9690674171937933;

  int rotationError = 0;
  int targetNumRotations = 0;
  float rotationError_1 = 0.0f;
  float rotationErrorIntegral = 0.0f;
  float rotationErrorIntegral_1 = 0.0f;
  float PIout = 0.0f;

  float Kp_rotErr = 0.1f;
  float Ki_rotErr = 0.005f;

  int pwmOffset = 0;
  int pwmError = 0;
  int targetPWM = 0;
  float pwmError_1 = 0.0f;
  float pwmErrorIntegral = 0.0f;
  float pwmErrorIntegral_1 = 0.0f;
  float pwmPIout = 0.0f;

  float Kp_pwmErr = 0.5f;
  float Ki_pwmErr = 0.05f;

  void setencoderPins(int p1, int p2)
  {
    encoderpins[0] = p1;
    encoderpins[1] = p2;
    encoderstates[0] = LOW;
    encoderstates[1] = LOW;
    encoderstates[2] = LOW;
    encoderstates[3] = LOW;
    encoderphase[0] = 0;
    encoderphase[1] = 0;
    encoderphaseD = 0;
    numrotations = 0;
    lastnumrotations = 0;
    pinMode(encoderpins[0],INPUT);
    pinMode(encoderpins[1],INPUT);

  }

  void setdriverPins(int p1, int p2, int p3) // Forward, Backward, PWM/speed
  {
    driverpins[0] = p1;
    driverpins[1] = p2;
    driverpins[2] = p3;

    pinMode(driverpins[0],OUTPUT);
    pinMode(driverpins[1],OUTPUT);
    pinMode(driverpins[2],OUTPUT);

  }

  void setPWM(int x)
  {
    pwm = x + pwmOffset;
    analogWrite(driverpins[2],pwm);
  }

  void setDirection(int dir)
  {
    digitalWrite(driverpins[0],dir);
    digitalWrite(driverpins[1],!dir);
    forward=dir;
  }

  void updateencoderState()
  {
      if(encoderstates[0])
      {
          if(encoderstates[2])
          {
              encoderphase[0] = 3;
              encoderphaseD = encoderphase[0] - encoderphase[1];
              if(encoderphaseD == 2){numrotations--;}
              else if(encoderphaseD == 1){numrotations++;}
          }
          else
          {
              encoderphase[0] = 2;
              encoderphaseD = encoderphase[0] - encoderphase[1];
              if(encoderphaseD == -1){numrotations--;}
              else if(encoderphaseD == 2){numrotations++;}
          }
      }
      else
      {
          if(encoderstates[2])
          {
              encoderphase[0] = 1;
              encoderphaseD = encoderphase[0] - encoderphase[1];
              if(encoderphaseD == 1){numrotations--;}
              else if(encoderphaseD == -2){numrotations++;}
          }
          else
          {
              encoderphase[0] = 0;
              encoderphaseD = encoderphase[0] - encoderphase[1];
              if(encoderphaseD == -2){numrotations--;}
              else if(encoderphaseD == -1){numrotations++;}
          }
      }

      inverserotations = -1*numrotations;
      encoderphase[1] = encoderphase[0];
      currentTime = millis();
      dt = (float)(currentTime-lastTime)*0.001f;
      lastTime = currentTime;
      // Update velocity based on difference between numrotations and lastrotation


      diffnumrotations = numrotations - lastnumrotations;
      lastnumrotations = numrotations;

      velocity = diffnumrotations* 0.048f/dt/oneRotation;


      // First order filter on velocity
    //  dv1 = 0.5*(velocity + velocity_mem[1]);
    //  vel_avg = (dv1 + velocity_mem[1] + velocity_mem[2] + velocity_mem[3] + velocity_mem[4])/5.0;
    //  velocity_mem[4] = velocity_mem[3];
    //  velocity_mem[3] = velocity_mem[2];
    //  velocity_mem[2] = velocity_mem[1];
    //  velocity_mem[1] = velocity_mem[0];
    //  velocity_mem[0] = vel_avg;
  }

  void velmemdelay(float x)
  {
    velocity_mem[4] = velocity_mem[3];
    velocity_mem[3] = velocity_mem[2];
    velocity_mem[2] = velocity_mem[1];
    velocity_mem[1] = velocity_mem[0];
    velocity_mem[0] = x;
  }

  // FIR low pass filter. Convolve [1,1] 3 times, or binom(4)
  void computeVelfilter(float x)
  {
      filtin = x;
      filtout = a1*filtout_1 + b0*filtin + b1*filtin_1;
      filtin_1 = filtin;
      filtout_1 = filtout;
  }
  void computeVelAvg()
  {
      vel_avg = (velocity_mem[0] + velocity_mem[1] + velocity_mem[2] + velocity_mem[3] + velocity_mem[4])/5.0;
  }


  // Set desired number of rotations...convert this to mm eventually
  void moveByN(int target)
  {
      targetNumRotations = target;
      DRflag = true;
  }

  void setPWMTarget(int x)
  {
      targetPWM = x;
  }


  void pwmPID()
  {
        pwmError = targetPWM - pwm;
        // PI control
        pwmErrorIntegral = pwmErrorIntegral_1 +  (float)(pwmError);
        if(pwmErrorIntegral > 10)
        {
          pwmErrorIntegral = 0;
        }
        pwmPIout = (float)(Kp_pwmErr * pwmError + Ki_pwmErr * pwmErrorIntegral);
        if(pwmPIout > 0)
        {
          setDirection(HIGH);
        }
        else
        {
          setDirection(LOW);
        }
        setPWM(abs(pwmPIout));
        pwmError_1 = pwmError;
        pwmErrorIntegral_1 = pwmErrorIntegral;
  }


  // Update dead reckoning. Call this in the main loop
  void updateDR(float T)
  {
        rotationError = numrotations - targetNumRotations;
        // PI control
        rotationErrorIntegral = rotationErrorIntegral_1 + T* (float)(rotationError);
        if(rotationErrorIntegral > 10)
        {
          rotationErrorIntegral = 0;
        }
        PIout = (float)(Kp_rotErr * rotationError +  Ki_rotErr *rotationErrorIntegral);
        if(PIout > 0)
        {
          setDirection(HIGH);
        }
        else
        {
          setDirection(LOW);
        }
        setPWM(fabs(PIout));
        
        if(rotationError < 20)
        {
          if(rotationError_1 > rotationError)
          {
            zerocounter += 1;
          }
        }
        if(zerocounter > 40)
        {
          DRflag = false;
          zerocounter = 0;
        }

        rotationError_1 = rotationError;
        rotationErrorIntegral_1 = rotationErrorIntegral;

  }

};
