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
ErrorDescriptor x_coord("X_COORD"); // x_coord for forward direction
ErrorDescriptor y_coord("Y_COORD"); // y_coord for sideward direction
ErrorDescriptor z_coord("Z_COORD"); // z_coord for upward direction

int count_torpedo = 0;
int count_imu = 0;

double move_forward_duration = 0;
double move_backward_duration = 0;
double move_forward_start = 0;
double move_backward_start = 0;

bool stop_signal = false; // buoy task ends
bool move_forward_signal = false;
bool move_backward_signal = false;

void torpedoDataCallback(const geometry_msgs::PointStampedPtr &_msg) {
    if (count_torpedo == 0) {
        x_coord.setReference(30);
        y_coord.setReference(0);
        // z_coord.setReference(0);
        count_torpedo++;
    }
    //if (_msg->point.x <= 30 && count_buoy == 1) {
        //move_forward_signal = true;
        //move_forward_start = ros::Time::now().toSec();
        //count_buoy++;
        //ROS_INFO("MOVING FORWARD WITH CONSTANT PWM NOW!!!!");
    //}

    //if (ros::Time::now().toSec() >= move_forward_start + move_forward_duration && count_buoy == 2) {
        //move_backward_signal = true;
        //move_forward_signal = false;
        //move_backward_start = ros::Time::now().toSec();
        //count_buoy++;
        //ROS_INFO("MOVING BACKWARD WITH CONSTANT PWM NOW!!!!");
    //}

    //if (ros::Time::now().toSec() >= move_backward_start + move_backward_duration && count_buoy == 3) {
        //move_backward_signal = false;
        //stop_signal = true;
        // x_coord.setReference(50);
       // y_coord.setReference(0);
        // z_coord.setReference(0);
        //ROS_INFO("NOW STOPING!!!! ");
        //count_buoy++;
    //}

    //if (move_forward_signal) {
        // x_coord.errorToPWM(30);
        //y_coord.errorToPWM(0);
        // z_coord.errorToPWM(0);
    //}
    //else if (move_backward_signal) {
        // x_coord.errorToPWM(-30);
        //y_coord.errorToPWM(0);
        // z_coord.errorToPWM(0);
    //}
    //else if (stop_signal) {
        //y_coord.errorToPWM(0);
    //}
    //else {
        // assuming point.y is for sideward position
        // point.x for distance
        // point.z for depth
         x_coord.errorToPWM(_msg->point.x);
         y_coord.errorToPWM(_msg->point.y);
        // z_coord.errorToPWM(_msg->point.z);
    //}
}

void imuDataCallback(const std_msgs::Float64Ptr &_msg) {
    if (count_imu == 0) {
        angle.setReference(_msg->data);
        count_imu++;
    }
    angle.errorToPWM(_msg->data);
    std::cout << "getting imu callback" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_torpedo");
    ros::NodeHandle nh;
    ros::Subscriber torpedoDataListener = nh.subscribe("/torpedo_task/torpedo_coordinates", 1000, &torpedoDataCallback);
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

    angle.setPID(2.4, 0, 0.5, 1);
    y_coord.setPID(-1.6, 0, -0.3, 15);
    z_coord.setPID(0, 0, 0, 0);
    x_coord.setPID(-7.5, 0, -2, 10);
    move_forward_duration = 2;
    move_backward_duration = 10;

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        pwm_sideward_front.data = y_coord.getPWM() + angle.getPWM();
        pwm_sideward_back.data = y_coord.getPWM() - angle.getPWM();

        //if (move_forward_signal) {
            //pwm_forward_left.data = 100;
            //pwm_forward_right.data = 100;
        //}
        //else if (move_backward_signal) {
            //pwm_forward_left.data = -150;
            //pwm_forward_right.data = -150;
        //}
        //else if (stop_signal) {
            //pwm_forward_left.data = 0;
            //pwm_forward_right.data = 0;
        //}
        //else {
            //pwm_forward_left.data = 150;
            //pwm_forward_right.data = 150;
        //}

        pwm_forward_left.data = x_coord.getPWM();
        pwm_forward_right.data = x_coord.getPWM();

        pwm_upward_front.data = z_coord.getPWM();
        pwm_upward_back.data = z_coord.getPWM();

        frontSidewardPublisher.publish(pwm_sideward_front);
        backSidewardPublisher.publish(pwm_sideward_back);

        rightForwardPublisher.publish(pwm_forward_right);
        leftForwardPublisher.publish(pwm_forward_left);

        frontUpwardPublisher.publish(pwm_upward_front);
        backUpwardPublisher.publish(pwm_upward_back);

        std::cout << "----------------------------------" << std::endl;
        ROS_INFO("PWM forward_right : %d", pwm_forward_right.data);
        ROS_INFO("PWM forward_left : %d", pwm_forward_left.data);
        ROS_INFO("PWM sideward_front : %d", pwm_sideward_front.data);
        ROS_INFO("PWM sideward_back : %d", pwm_sideward_back.data);
        //ROS_INFO("PWM upward_front : %d", pwm_upward_front.data);
        //ROS_INFO("PWM upward_back : %d", pwm_upward_back.data);
        ROS_INFO("PWM turn : %d", angle.getPWM());

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
