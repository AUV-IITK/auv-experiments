// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
using std::string;

float presentAngularPosition = 0;
float previousAngularPosition = 0;
float fixedAngularPosition, error, output, error_val;
bool initData = false;

std_msgs::Int32 pwm_sidewardFront;
std_msgs::Int32 pwm_sidewardBack;
std_msgs::Int32 pwm_turn;

int sidewardFront_PWM_data_ = 0;
int sidewardBack_PWM_data_ = 0;
int turn_PWM_data_ = 0;

// ros::Publisher PWM_turn;
ros::Publisher PWM_sidewardFront;
ros::Publisher PWM_sidewardBack;

// ros::Publisher
float p = 2.4;
float i = 0.5;
float d = 0.35;
float band = 1.0;

double previoustime, presenttime;

void turningOutputPWMMapping(float output) // to keep PWM values within a limit
{
  float maxOutput = 1000, minOutput = -maxOutput;
  float scale = 255 / maxOutput;
  if (output > maxOutput)
    output = maxOutput;
  if (output < minOutput)
    output = minOutput;
  float temp = output * scale;
  int output_pwm = static_cast<int>(temp);
  if (output_pwm > 255)
    output_pwm = 255;
  if (output_pwm < -255)
    output_pwm = -255;
  // pwm_turn.data = output_pwm;
  turn_PWM_data_ = output_pwm;
}

void PWMsideward(std_msgs::Int32 msg)
{
  // pwm_sidewardFront.data = msg.data;
  sidewardFront_PWM_data_ = msg.data;

  // pwm_sidewardBack.data = msg.data;
  sidewardBack_PWM_data_ = msg.data;
}
// void publish(std_msgs::Int32 pwm)
// {
//     pwm_sidewardFront.data = pwm_sidewardFront.data + pwm.data;
//     pwm_sidewardBack.data = pwm_sidewardBack.data - pwm.data;
// 
//     PWM_sidewardFront.publish(pwm_sidewardFront);
//     PWM_sidewardBack.publish(pwm_sidewardBack);
// }

void publish(int sidewardFront_PWM_data_, int sidewardBack_PWM_data_, int turn_PWM_data_)
{

	pwm_sidewardFront.data = sidewardFront_PWM_data_ + turn_PWM_data_;
	pwm_sidewardBack.data = sidewardBack_PWM_data_ - turn_PWM_data_;

	PWM_sidewardFront.publish(pwm_sidewardFront);
	PWM_sidewardBack.publish(pwm_sidewardBack);

	std::cout << "Data Published on SidewardFront thruster: " << pwm_sidewardFront.data << std::endl;
	std::cout << "Data published on SidewardBack thruster: " << pwm_sidewardBack.data << std::endl;

}
void yawCb(std_msgs::Float64 msg)
{
    double initial_loop_time = ros::Time::now().toNSec();

    // this is used to set the final angle after getting the value of first intial
    // position
    if (initData == false)
    {
        presenttime = ros::Time::now().toSec();
        fixedAngularPosition = msg.data;
        presentAngularPosition = fixedAngularPosition;
        initData = true;

        std::cout << "Initial angular position fixed: " << fixedAngularPosition << std::endl;
    }
    else
    {
        previousAngularPosition = presentAngularPosition;
        presentAngularPosition = msg.data;

        previoustime = presenttime;
        presenttime = ros::Time::now().toSec();

        if (fixedAngularPosition >= 180)
            fixedAngularPosition = fixedAngularPosition - 360;
        else if (fixedAngularPosition <= -180)
            fixedAngularPosition = fixedAngularPosition + 360;

        float derivative = 0, integral = 0;
        double dt = presenttime - previoustime;

        error = fixedAngularPosition - presentAngularPosition;
        if (error < 0)
            error_val = error + 360;
        else
            error_val = error - 360;

        if (abs(error_val) < abs(error))
            error = error_val;

        std::cout << "ERROR: " << error << std::endl;
        integral += (error * dt);
        derivative = (presentAngularPosition - previousAngularPosition) / dt;
        output = (p * error) + (i * integral) + (d * derivative);

        turningOutputPWMMapping(output);

        if (error < band && error > -band)
        {
            std::cout << "ERROR: " << "1" << std::endl;
            // pwm_turn.data = 0;
	    turn_PWM_data_ = 0;
        }

        double final_loop_time = ros::Time::now().toNSec();

        double total_loop_time = final_loop_time - initial_loop_time;

        std::cout << "Total loop time: " << total_loop_time << std::endl;

//        pwm_sidewardFront.data = pwm_sidewardFront.data + pwm_turn.data;
//        pwm_sidewardBack.data = pwm_sidewardBack.data - pwm_turn.data;

        // PWM_turn.publish(pwm_turn);
//        PWM_sidewardFront.publish(pwm_sidewardFront);
//        PWM_sidewardBack.publish(pwm_sidewardBack);
//        publish(pwm_turn);
    }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sideward_yaw");
	ros::NodeHandle nh_;

	std::cout << "Sideward_Yaw initiated, p: " << " " << "i: " << " " << "d: " <<std::endl;

	ros::Subscriber yaw = nh_.subscribe<std_msgs::Float64>("/varun/motion/yaw", 1000, &yawCb);
	ros::Subscriber pwm_sideward = nh_.subscribe<std_msgs::Int32>("/pwm/sideward", 1000, &PWMsideward);
	PWM_sidewardFront = nh_.advertise<std_msgs::Int32>("/pwm/sidewardFront", 400);
	PWM_sidewardBack = nh_.advertise<std_msgs::Int32>("/pwm/sidewardBack", 400);
	
	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		publish(sidewardFront_PWM_data_, sidewardBack_PWM_data_, turn_PWM_data_);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
