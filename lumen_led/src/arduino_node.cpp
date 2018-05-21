#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <Wire.h>


int bLedOn = 0;

void ledFunction(const std_msgs::Int32& msg) {
  bLedOn = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> subLedOn("/pwm/led", &ledFunction);

byte servoPin = 9;
Servo servo;

void setup() {
  nh.initNode();
  Wire.begin();
  nh.subscribe(subLedOn);
  servo.attach(servoPin);
  servo.writeMicroseconds(1100); // Send off signal
}

void loop() {
  if(bLedOn) servo.writeMicroseconds(1700); // Send on signal
  else servo.writeMicroseconds(1100); // Send off signal
}
