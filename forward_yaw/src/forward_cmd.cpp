// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

using std::string;

int main(int argc, char **argv) {

    ros::init(argc, argv, "forward_cmd");
    ros::NodeHandle nh;
    ros::Publisher rightThrusterPWMPub = nh.advertise<std_msgs::Int32>("/forward/pwm/right", 1000);
    ros::Publisher leftThrusterPWMPub = nh.advertise<std_msgs::Int32>("/forward/pwm/left", 1000);

    ros::Rate loop_rate(1000);

    std_msgs::Int32 rightThrusterPWM.data = std::atoi(argv[1]);
    std_msgs::Int32 leftThrusterPWM.data = std::atoi(argv[2]);

    while(ros::ok()) {
        
        

    }
}