// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include <errorToPWM.h>
#include <geometry_msgs/PointStamped.h>

ErrorDescriptor angle("ANGLE");
ErrorDescriptor x_coord("X_COORD"); // x_coord along the width of the screen
ErrorDescriptor y_coord("Y_COORD"); // y_coord along the height
ErrorDescriptor z_coord("Z_COORD"); // z_coord for distance

int count_buoy = 0;
int count_imu = 0;

double move_forward_duration = 0;
double move_forward_start = 0;

bool move_forward_signal = false;
bool move_backward_signal = false;

void buoyDataCallback(const geometry_msgs::PointStampedPtr &_msg) {
    if (count_buoy == 0) {
        x_coord.setReference(0);
        y_coord.setReference(0);
        z_coord.setReference(0);
        count_buoy++;
    }
    if (_msg->point.x <= 30 && count_buoy == 1) {
        move_forward_signal = true;
        move_forward_start = ros::Time::now().toSec();
        count_buoy++;
        ROS_INFO("MOVING FORWARD WITH CONSTANT PWM NOW!!!!");
    }

    if (ros::Time::now().toSec() >= move_forward_start + move_forward_duration && count_buoy == 2) {
        move_backward_signal = true;
        move_forward_signal = false;
        count_buoy++;
        ROS_INFO("MOVING BACKWARD WITH CONSTANT PWM NOW!!!!");
    }

    if (_msg->point.x >= 30 && count_buoy == 3) {
        move_backward_signal = false;
        x_coord.setReference(0);
        y_coord.setReference(0);
        z_coord.setReference(50);
        ROS_INFO("NOW MOVING A BACK LITTLE BIT!!!! ");
    }

    if (move_forward_signal) {
        x_coord.errorToPWM(0);
        y_coord.errorToPWM(0);
        z_coord.errorToPWM(30);        
    }
    else if (move_backward_signal) {
        x_coord.errorToPWM(0);
        y_coord.errorToPWM(0);
        z_coord.errorToPWM(-30);                
    }
    else {
        // assuming point.y is for sideward position
        // point.x for distance
        // point.z for depth
        x_coord.errorToPWM(_msg->point.y); 
        y_coord.errorToPWM(_msg->point.z);
        z_coord.errorToPWM(_msg->point.x);
    }
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
    ros::Subscriber buoyDataListener = nh.subscribe("/threshold/center_coordinates", 1000, &buoyDataCallback);
    ros::Subscriber imuDataListener = nh.subscribe("/varun/sensors/imu/yaw", 1000, &imuDataCallback);
    
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