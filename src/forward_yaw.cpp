// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
// #include <dynamic_reconfigure/server.h>
// #include <motion_turn/pidConfig.h>
#include <string>
using std::string;

float presentAngularPosition = 0;
float previousAngularPosition = 0;
float fixedAngularPosition, error, output, error_val;
bool initData = false;

std_msgs::Int32 pwm_forward_left;  // pwm to be send to arduino
std_msgs::Int32 pwm_forward_right;

std_msgs::Int32 pwm_sideward_front;
std_msgs::Int32 pwm_sideward_back;

std_msgs::Int32 pwm_turn;

ros::Publisher PWM_turn;
// ros::Publisher PWM_forward;

ros::Publisher forwardLeftThrusterPublisher;
ros::Publisher forwardRightThrusterPublisher;

// ros::Publisher sidewardBackThrusterPublisher;
// ros::Publisher sidewardFrontThrusterPublisher;

// ros::Publisher
float p = 4.0;
float i = 0.0;
float d = 0.35;
float band = 0.1;
// float p, i, d, band, p_stablize, i_stablize, d_stablize, p_turn, i_turn, d_turn, band_stablize, band_turn;
double previoustime, presenttime;

// void setPID(float new_p_stablize, float new_p_turn, float new_i_stablize, float new_i_turn, float new_d_stablize,
//             float new_d_turn, float new_band_stablize, float new_band_turn)
// {
//  p_stablize = new_p_stablize;
//  p_turn = new_p_turn;
//  i_stablize = new_i_stablize;
//  i_turn = new_i_turn;
//  d_stablize = new_d_stablize;
//  d_turn = new_d_turn;
//  band_stablize = new_band_stablize;
//  band_turn = new_band_turn;
// }
//
// // dynamic reconfig
// void callback(motion_turn::pidConfig &config, double level)
// {
//   ROS_INFO("%s TurnServer: Reconfigure Request: p_stablize=%f p_turn=%f "
//            "i_stablize=%f i_turn=%f d_stablize=%f d_turn=%f error band_turn=%f",
//            ros::this_node::getName().c_str(), config.p_stablize_, config.p_turn_, config.i_stablize_, config.i_turn_,
//            config.d_stablize_, config.d_turn_, config.band_turn_);
//   setPID(config.p_stablize_, config.p_turn_, config.i_stablize_, config.i_turn_, config.d_stablize_, config.d_turn_,
//                  config.band_stablize_, config.band_turn_);
// }

void turningOutputPWMMapping(float output)
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
  pwm_turn.data = output_pwm;
}

void yawCb(std_msgs::Float64 msg)
{
  // this is used to set the final angle after getting the value of first intial
  // position
  if (initData == false)
  {
    presenttime = ros::Time::now().toSec();
    fixedAngularPosition = msg.data;
    presentAngularPosition = fixedAngularPosition;
    initData = true;
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

    // if (goal->loop > 10000)
    // {
    //   p = p_stablize;
    //   i = i_stablize;
    //   d = d_stablize;
    //   band = band_stablize;
    // }
    //
    // else
    // {
      // p = p_turn;
      // i = i_turn;
      // d = d_turn;
      // band = band_turn;
    // }

    error = fixedAngularPosition - presentAngularPosition;
    if (error < 0)
      error_val = error + 360;
    else
      error_val = error - 360;

    if (abs(error_val) < abs(error))
      error = error_val;

    std::cout<<error;
    std::cout<<"\n";
    integral += (error * dt);
    derivative = (presentAngularPosition - previousAngularPosition) / dt;
    output = (p * error) + (i * integral) + (d * derivative);

    turningOutputPWMMapping(output);

    if (error < band && error > -band)
    {
      std::cout<<"1";
      pwm_turn.data = 0;
    }


    // sidewardBackThrusterPublisher.publish(pwm_turn);
    // sidewardFrontThrusterPublisher.publish(pwm_turn);
    PWM_turn.publish(pwm_turn);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_yaw");
  ros::NodeHandle n;
  ros::NodeHandle nh_;
  // double p_stablize, p_turn, i_stablize, i_turn, d_stablize, d_turn, band_stablize, band_turn;
  // n.getParam("motion_straight/p_stablize", p_stablize);
  // n.getParam("motion_straight/p_turn", p_turn);
  // n.getParam("motion_straight/i_stablize", i_stablize);
  // n.getParam("motion_straight/i_turn", i_turn);
  // n.getParam("motion_straight/d_stablize", d_stablize);
  // n.getParam("motion_straight/d_turn", d_turn);
  // n.getParam("motion_straight/band_stablize", band_stablize);
  // n.getParam("motion_straight/band_turn", band_turn);

  ros::Subscriber yaw = n.subscribe<std_msgs::Float64>("/varun/motion/yaw", 1000, &yawCb);
  PWM_turn = nh_.advertise<std_msgs::Int32>("/pwm/turn", 1000);
  // PWM_forward = nh_.advertise<std_msgs::Int32>("/pwm/forward", 1000);

  forwardLeftThrusterPublisher = nh_.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
  forwardRightThrusterPublisher = nh_.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);

  // sidewardBackThrusterPublisher = nh_.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
  // sidewardFrontThrusterPublisher = nh_.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);

  // register dynamic reconfig server.
  // dynamic_reconfigure::Server<motion_turn::pidConfig> server;
  // dynamic_reconfigure::Server<motion_turn::pidConfig>::CallbackType f;
  // f = boost::bind(&callback, _1, _2);
  // server.setCallback(f);
  // set launch file pid
  /*motion_turn::pidConfig config;
  config.p_stablize = p_stablize;
  config.p_turn = p_turn;
  config.i_stablize = i_stablize;
  config.i_turn = i_turn;
  config.d_stablize = d_stablize;
  config.d_turn = d_turn;
  config.band_stablize = band_stablize;
  config.band_turn = band_turn;
  callback(config, 0);*/

  while(ros::ok())
  {
    // pwm_forward.data = 200;
    // PWM_forward.publish(pwm_forward);

    pwm_forward_left.data = 128;
    pwm_forward_right.data = 180;

    forwardLeftThrusterPublisher.publish(pwm_forward_left);
    forwardRightThrusterPublisher.publish(pwm_forward_right);

    ros::spinOnce();
  }

  return 0;
}
