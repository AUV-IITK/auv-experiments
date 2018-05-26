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

	// std::cout << "srgc" << argc << std::endl;

	if (argc != 3)
	{
		std::cout << "Invalid Input!!!!" << std::endl;
		return -1;
	}
	ros::Publisher rightThrusterPWMPub = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
	ros::Publisher leftThrusterPWMPub = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);

	ros::Rate loop_rate(100);

	std_msgs::Int32 rightThrusterPWM;
	rightThrusterPWM.data = std::atoi(argv[1]);
	std_msgs::Int32 leftThrusterPWM;
	leftThrusterPWM.data = std::atoi(argv[2]);

	while(ros::ok()) {

		rightThrusterPWMPub.publish(rightThrusterPWM);
		leftThrusterPWMPub.publish(leftThrusterPWM);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
