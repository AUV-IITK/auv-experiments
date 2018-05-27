// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

using std::string;

int main(int argc, char **argv) {

	ros::init(argc, argv, "sideward_cmd");
	ros::NodeHandle nh;

	if (argc != 2)
	{
		std::cout << "Invalid Input!!!!" << std::endl;
		return -1;
	}
	ros::Publisher ThrusterPWMPub = nh.advertise<std_msgs::Int32>("/pwm/sideward", 1000);

	ros::Rate loop_rate(50);

	std_msgs::Int32 ThrusterPWM;
	ThrusterPWM.data = std::atoi(argv[1]);

	while(ros::ok()) {

		ThrusterPWMPub.publish(ThrusterPWM);

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
