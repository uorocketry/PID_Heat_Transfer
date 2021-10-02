#include "PID.h"

PID::PID(float KpPID, float kiPID, float KdPID,
         float TauPID,
         float integrateMaxPID, float integrateMinPID,
         float outMaxPID, float outMinPID,
         float SampleTime,
         float spPID)
{
  // pid coefficients
  kp = KpPID;
  ki = kiPID;
  kd = KdPID;

  // low pass filter constant
  tau = TauPID;

  // integral limits to negate integrator wind up 
  integralMax = integrateMaxPID; 
  integralMin = integrateMinPID;

  // output limits 
  outputMax = outMaxPID;
  outputMin = outMinPID;

  // sampling time (milliseconds) 
  T = SampleTime;

  // desired system output 
  setpoint = spPID;

  // initial controller values 
  integral = 0.0f;
  derivative = 0.0f; 
  prevError = 0.0f;
  prevMeasure = 0.0f;
}


/*
 * Difference Equation:
 * 
 * P:   p[n] = Kp * e[n]
 * I:   i[n] = ((Ki * T)/2.0f) * (e[n] + e[n-1]) + i[n-1]
 * D:   d[n] = ((-2.0f * Kd)/(2.0f * tau + T)) * (m[n] - m[n-1]) + ((2.0f * tau - T)/(2.0f * tau + T)) * d[n-1]
 * 
 */

int PID::control(float measure)
{ 
  float error = setpoint - measure;

  float proportional = kp * error; 
  integral = ((ki * T) / 2.0f) * (error - prevError) + integral; 
  // derivative on measurement with low pass filter 
  derivative = ((-2.0f * kd)/(2.0f * tau + T)) * (measure - prevMeasure) + ((2.0f * tau - T)/(2.0f * tau + T)) * derivative;

  // negate integral windup 
  LimitIntegral();

  // parallel pid controller 
  controlSignal = proportional + integral + derivative;

  // limit control signal output 
  LimitPidOutput();

  // store to be used as prev values next function call 
  prevError = error;
  prevMeasure = measure;

  // return output of pid 
  return int(controlSignal);
} 


void PID::LimitIntegral()
{
  if (integral > integralMax)
  {
    integral = integralMax;
  }
  else if (integral < integralMin)
  {
    integral = integralMin;
  }
}


void PID::LimitPidOutput()
{
  if (controlSignal > outputMax)
  {
    controlSignal = outputMax;
  }
  else if (controlSignal < outputMin) 
  {
    controlSignal = outputMin;
  }
}


Heater::Heater(int pinNum)
{
  pin = pinNum;
}


void Heater::pwm(int value)
{
  analogWrite(pin, value);
}
