// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include <errorToPWM.h>

ErrorDescriptor angle("ANGLE");
ErrorDescriptor x_coord("X_COORD");
ErrorDescriptor y_coord("Y_COORD");
ErrorDescriptor z_coord("Z_COORD");

int count_buoy = 0;
int count_imu = 0;

void buoyDataCallback(const std_msgs::Float64MultiArrayPtr &_msg) {
    if (count_buoy == 0) {
        x_coord.setReference(0);
        y_coord.setReference(0);
        z_coord.setReference(0);
        count_buoy++;
    }
    x_coord.errorToPWM(_msg->data[1]);
    y_coord.errorToPWM(_msg->data[2]);
    z_coord.errorToPWM(_msg->data[3]);
}

void imuDataCallback(const std_msgs::Float64Ptr &_msg) {
    if (count_imu == 0) {
        angle.setReference(_msg->data);
    }
    angle.errorToPWM(_msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_buoy");
    ros::NodeHandle nh;
    ros::Subscriber buoyDataListener = nh.subscribe("/varun/ip/buoy", 1000, &buoyDataCallback);
    ros::Subscriber imuDataListener = nh.subscribe("/varun/imu", 1000, &imuDataCallback);
    
    ros::Publisher frontSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);
    ros::Publisher backSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
    ros::Publisher rightForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);
    ros::Publisher leftForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
    ros::Publisher frontUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardFront", 1000);
    ros::Publisher backUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardBack", 1000);

    std_msgs::Int32 pwm_sideward_front;
    std_msgs::Int32 pwm_sideward_back;
    std_msgs::Int32 pwm_forward_right;
    std_msgs::Int32 pwm_forward_left;
    std_msgs::Int32 pwm_upward_front;
    std_msgs::Int32 pwm_upward_back;

    ros::Rate loop_rate(100);

    while(ros::ok()) {

        pwm_sideward_front.data = x_coord.getPWM() + angle.getPWM();
        pwm_sideward_back.data = x_coord.getPWM() - angle.getPWM();

        pwm_forward_left.data = z_coord.getPWM();
        pwm_forward_right.data = z_coord.getPWM();

        pwm_upward_back.data = y_coord.getPWM();
        pwm_upward_back.data = y_coord.getPWM();

        frontSidewardPublisher.publish(pwm_sideward_front);
        backSidewardPublisher.publish(pwm_sideward_back);

        rightForwardPublisher.publish(pwm_forward_right);
        leftForwardPublisher.publish(pwm_forward_left);

        frontUpwardPublisher.publish(pwm_upward_front);
        backUpwardPublisher.publish(pwm_upward_back); 

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}