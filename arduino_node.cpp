// Copyright 2016 AUV-IITK
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <Wire.h>
// #include "MS5837.h"

#define pwmPinWest 2
#define pwmPinEast 3
#define directionPinEast1 30
#define directionPinEast2 31
#define directionPinWest1 32
#define directionPinWest2 33

#define pwmPinNorthSway 6
#define pwmPinSouthSway 7
#define directionPinSouthSway1 23
#define directionPinSouthSway2 22
#define directionPinNorthSway1 24
#define directionPinNorthSway2 25

#define pwmPinNorthUp 5
#define pwmPinSouthUp 4
#define directionPinNorthUp1 26
#define directionPinNorthUp2 27
#define directionPinSouthUp1 28
#define directionPinSouthUp2 29

#define analogPinPressureSensor A0

// MS5837 sensor;

// float last_pressure_sensor_value, pressure_sensor_value;
// std_msgs::Float64 voltage;
ros::NodeHandle nh;

void thrusterNorthUp(int pwm, int isUpward)
{
  pwm = abs(pwm);
  analogWrite(pwmPinNorthUp, 255 - pwm);
  if (isUpward)
  {
    digitalWrite(directionPinNorthUp1, HIGH);
    digitalWrite(directionPinNorthUp2, LOW);
  }
  else
  {
    digitalWrite(directionPinNorthUp1, LOW);
    digitalWrite(directionPinNorthUp2, HIGH);
  }
}

void thrusterSouthUp(int pwm, int isUpward)
{
  pwm = abs(pwm);
  analogWrite(pwmPinSouthUp, 255 - pwm);
  if (isUpward)
  {
    digitalWrite(directionPinSouthUp1, LOW);
    digitalWrite(directionPinSouthUp2, HIGH);
  }
  else
  {
    digitalWrite(directionPinSouthUp1, HIGH);
    digitalWrite(directionPinSouthUp2, LOW);
  }
}

void thrusterNorthSway(int pwm, int isRight)
{
  if(pwm != 0) pwm = map(abs(pwm), 1, 255, 40, 255);
  analogWrite(pwmPinNorthSway, 255 - pwm);
  if (isRight)
  {
    digitalWrite(directionPinNorthSway1, HIGH);
    digitalWrite(directionPinNorthSway2, LOW);
  }
  else
  {
    digitalWrite(directionPinNorthSway1, LOW);
    digitalWrite(directionPinNorthSway2, HIGH);
  }
}

void thrusterSouthSway(int pwm, int isRight)
{
  if(pwm != 0) pwm = map(abs(pwm), 1, 255, 43, 255);
  analogWrite(pwmPinSouthSway, 255 - pwm);
  if (isRight)
  {
    digitalWrite(directionPinSouthSway1, HIGH);
    digitalWrite(directionPinSouthSway2, LOW);
  }
  else
  {
    digitalWrite(directionPinSouthSway1, LOW);
    digitalWrite(directionPinSouthSway2, HIGH);
  }
}

void thrusterEast(int pwm, int isForward)
{
  if(pwm != 0) pwm = map(abs(pwm), 1, 255, 30, 255);
  analogWrite(pwmPinEast, 255 - pwm);
  if (isForward)
  {
    digitalWrite(directionPinEast1, HIGH);
    digitalWrite(directionPinEast2, LOW);
  }
  else
  {
    digitalWrite(directionPinEast1, LOW);
    digitalWrite(directionPinEast2, HIGH);
  }
}

void thrusterWest(int pwm, int isForward)
{
  if(pwm != 0) pwm = map(abs(pwm), 1, 255, 24, 255);
  analogWrite(pwmPinWest, 255 - pwm);
  if (isForward)
  {
    digitalWrite(directionPinWest1, HIGH);
    digitalWrite(directionPinWest2, LOW);
  }
  else
  {
    digitalWrite(directionPinWest1, LOW);
    digitalWrite(directionPinWest2, HIGH);
  }
}

void PWMCbForwardRight(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterWest(msg.data, true);
  }
  else
  {
    thrusterWest(msg.data, false);
  }
}

void PWMCbForwardLeft(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterEast(msg.data, true);
  }
  else
  {
    thrusterEast(msg.data, false);
  }
}

void PWMCbSidewardFront(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterNorthSway(pwm, true);
  }
  else
  {
    thrusterNorthSway(pwm, false);
  }
}

void PWMCbSidewardBack(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterSouthSway(pwm, true);
  }
  else
  {
    thrusterSouthSway(pwm, false);
  }
}

void PWMCbUpwardFront(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterNorthUp(pwm, true);
  }
  else
  {
    thrusterNorthUp(pwm, false);
  }
}

void PWMCbUpwardBack(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  if (pwm > 0)
  {
    thrusterSouthUp(pwm, true);
  }
  else
  {
    thrusterSouthUp(pwm, false);
  }
}

void PWMCbTurn(const std_msgs::Int32& msg)
{
  //   if (msg.data > 0)
  //   {
  //     thrusterEast(msg.data, true);
  //     thrusterWest(msg.data, false);
  //   }
  //   else
  //   {
  //     thrusterEast(msg.data, false);
  //     thrusterWest(msg.data, true);
  //   }
  // }
  // else
  // {
  int pwm = msg.data;
    if (pwm > 0)
    {
      thrusterNorthSway(pwm, true);
      thrusterSouthSway(pwm, false);
    }
    else
    {
      thrusterNorthSway(pwm, false);
      thrusterSouthSway(pwm, true);
    }
}

ros::Subscriber<std_msgs::Int32> subPwmForwardRight("/pwm/forwardRight", &PWMCbForwardRight);
ros::Subscriber<std_msgs::Int32> subPwmSidewardFront("/pwm/sidewardFront", &PWMCbSidewardFront);
ros::Subscriber<std_msgs::Int32> subPwmUpwardFront("/pwm/upwardFront", &PWMCbUpwardFront);

ros::Subscriber<std_msgs::Int32> subPwmForwardLeft("/pwm/forwardLeft", &PWMCbForwardLeft);
ros::Subscriber<std_msgs::Int32> subPwmSidewardBack("/pwm/sidewardBack", &PWMCbSidewardBack);
ros::Subscriber<std_msgs::Int32> subPwmUpwardBack("/pwm/upwardBack", &PWMCbUpwardBack);
ros::Subscriber<std_msgs::Int32> subPwmTurn("/pwm/turn", &PWMCbTurn);
// ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);

void setup()
{
  nh.initNode();
  Wire.begin();

  // sensor.init();

  // sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
  pinMode(pwmPinEast, OUTPUT);
  pinMode(directionPinEast1, OUTPUT);
  pinMode(directionPinEast2, OUTPUT);
  pinMode(pwmPinWest, OUTPUT);
  pinMode(directionPinWest1, OUTPUT);
  pinMode(directionPinWest2, OUTPUT);

  pinMode(directionPinSouthSway1, OUTPUT);
  pinMode(directionPinSouthSway2, OUTPUT);
  pinMode(pwmPinNorthSway, OUTPUT);
  pinMode(directionPinNorthSway2, OUTPUT);
  pinMode(pwmPinSouthSway, OUTPUT);
  pinMode(directionPinNorthSway1, OUTPUT);

  pinMode(directionPinSouthUp1, OUTPUT);
  pinMode(directionPinSouthUp2, OUTPUT);
  pinMode(pwmPinNorthUp, OUTPUT);
  pinMode(directionPinNorthUp2, OUTPUT);
  pinMode(pwmPinSouthUp, OUTPUT);
  pinMode(directionPinNorthUp1, OUTPUT);

  nh.subscribe(subPwmForwardRight);
  nh.subscribe(subPwmSidewardFront);
  nh.subscribe(subPwmUpwardFront);
  nh.subscribe(subPwmForwardLeft);
  nh.subscribe(subPwmSidewardBack);
  nh.subscribe(subPwmUpwardBack);
  nh.subscribe(subPwmTurn);
  // nh.advertise(ps_voltage);

  Serial.begin(57600);
  std_msgs::Int32 msg;
  msg.data = 0;
  PWMCbForwardRight(msg);
  PWMCbSidewardFront(msg);
  PWMCbUpwardFront(msg);
  PWMCbTurn(msg);

  PWMCbForwardLeft(msg);
  PWMCbSidewardBack(msg);
  PWMCbUpwardBack(msg);

  // sensor.read();
  // last_pressure_sensor_value = -(sensor.depth() * 100);
}

void loop()
{
  // sensor.read();
  // voltage.data made -ve because pressure sensor data should increase going up
  // pressure_sensor_value = -(sensor.depth() * 100);
  // to avoid random high values
  // if (abs(last_pressure_sensor_value - pressure_sensor_value) < 100)
  // {
  //   voltage.data = 0.7 * pressure_sensor_value + 0.3 * last_pressure_sensor_value;
  //   ps_voltage.publish(&voltage);
  //   last_pressure_sensor_value = pressure_sensor_value;
  // }



  delay(200);
  nh.spinOnce();
}
