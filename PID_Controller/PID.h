#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "TeensyThreads.h"

// heater pin
#define PWM_PIN 28

/*
 * 
 * Set Kp to 0 to remove proportional component of PID controller 
 * Set Ki to 0 to remove integral component of PID controller 
 * Set Kd to 0 to remove derivative component of PID controller 
 * Set tau to 0 to remove low pass filter
 * SP_PID is setpoint you want system to reach 
 * 
 */
 

// pid constants 
#define KP_PID              0.1f
#define kI_PID              0.03f
#define KD_PID              0.01f
#define TAU_PID             0.01f 
#define INTEGRATE_MAX_PID   128.0f
#define INTEGRATE_MIN_PID   -5.0f
#define OUT_MAX_PID         255.0f
#define OUT_MIN_PID         0.0f  
#define SAMPLE_TIME         1000.0f
#define SP_PID              30.0f


class PID
{
  public:
    // PID constructor init values
    PID(float KpPID, float kiPID, float KdPID,
        float TauPID,
        float integrateMaxPID, float integrateMinPID,
        float outMaxPID, float outMinPID,
        float SampleTime,
        float spPID);

    // compute control signal given current measurement 
    // return integer controlSignal (pwm takes integer values)
    int control(float measure);


  private:

    // negate integral windup 
    void LimitIntegral();

    // limit control signal output 
    void LimitPidOutput();
  
    // pid coefficients 
    float kp;
    float ki;
    float kd;

    // low pass filter constant 
    float tau; 

    // integratal limits to negate integrator wind up 
    float integralMax;
    float integralMin;

    // output limits 
    float outputMax;
    float outputMin; 

    // sampling time (milliseconds) 
    float T;

    // controller variables
    float setpoint;
    float integral;
    float derivative;
    float prevError; 
    float prevMeasure;
    /* PID output */
    float controlSignal; 
};


class Heater
{
  public: 
    Heater(int pinNum);
    void pwm(int value);

  private: 
    int pin; 
};

#endif 
