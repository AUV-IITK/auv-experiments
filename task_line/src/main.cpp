// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <errorToPWM.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

ErrorDescriptor angle("ANGLE");
ErrorDescriptor x_coord("X_COORD"); // x_coord along forward direction
ErrorDescriptor y_coord("Y_COORD"); // y_coord along sideward direction

int angle_count = 0;
int coord_count = 0;
int depth_count  = 0;
double imu_angle_ = 0;

void lineCenterDataCallback(const geometry_msgs::Pose2DPtr &_msg) {
    if (coord_count == 0) {
        coord_count++;
        y_coord.setReference(0);
        x_coord.setReference(0);
    }
    double square = pow(_msg->x, 2) + pow(_msg->y, 2);
    y_coord.errorToPWM(_msg->y);
    x_coord.errorToPWM(_msg->x);
    
    if (angle_count == 0) {
        angle.setReference(13);
        angle_count++;
    }
    angle.errorToPWM(_msg->theta);

    //std::cerr << "bc" << std::endl;
}

void lineAngleDataCallback(const std_msgs::Float64Ptr &_msg) {
    //if (angle_count == 0) {
        //angle.setReference(imu_angle_+ _msg->data);
        //angle_count++;
    //}
    
    //std::cerr << "bc2" << std::endl;
}

void imuDataCallback(const std_msgs::Float64Ptr &_msg) {
    //imu_angle_ = _msg->data+11;
    
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_line");

    ros::NodeHandle nh;

    ros::Subscriber lineCenterDataListener = nh.subscribe("/line_task/line_coordinates", 1000, &lineCenterDataCallback);
    //ros::Subscriber lineAngleDataListener = nh.subscribe("/line/line_angle", 1000, &lineAngleDataCallback);
    ros::Subscriber imuDataListener = nh.subscribe("/varun/sensors/imu/yaw", 1000, &imuDataCallback);

    ros::Publisher frontSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);
    ros::Publisher backSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
    ros::Publisher rightForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);
    ros::Publisher leftForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
    ros::Publisher turnPublisher = nh.advertise<std_msgs::Int32>("/pwm/turn/test", 1000);
    ros::Publisher sidePublisher = nh.advertise<std_msgs::Int32>("/pwm/sideward", 1000);

    std_msgs::Int32 pwm_sideward_front;
    std_msgs::Int32 pwm_sideward_back;
    std_msgs::Int32 pwm_forward_right;
    std_msgs::Int32 pwm_forward_left;
    std_msgs::Int32 pwm_turn;
    std_msgs::Int32 pwm_sideward;

    angle.setPID(2.4, 0, 0.5, 1);
    x_coord.setPID(0, 0, 0, 0);
    y_coord.setPID(-1.6, 0, -0.3, 15);

    ros::Rate loop_rate(50);

    bool move_forward_signal = false;
    //double delay_time = 5;
    //double present_time = 0;
    //double previous_time = 0;

    while (ros::ok()) {
        pwm_sideward_front.data = y_coord.getPWM() - angle.getPWM();
        pwm_sideward_back.data = y_coord.getPWM() + angle.getPWM();
        pwm_turn.data = angle.getPWM();
        pwm_sideward.data = y_coord.getPWM();

        //present_time = ros::Time::now().toSec();
        //previous_time = present_time;
        std::cerr << "Y Current value " << y_coord.getCurrentValue() << std::endl;
        std::cerr << "Angle Current value " << angle.getCurrentValue() << std::endl;
        if (abs(angle.getCurrentValue()+6) < 1 && abs(y_coord.getCurrentValue()) < 15 && angle_count == 1) {
            move_forward_signal = true;
            std::cerr << "MOVE FORWARD SIGNAL RECEIVED" << std::endl;
        }
        if (move_forward_signal) {
            pwm_forward_left.data = 100;
            pwm_forward_right.data = 100;
        }
        else {
            pwm_forward_left.data = x_coord.getPWM();
            pwm_forward_right.data = x_coord.getPWM();
        }

//        pwm_upward_front.data = z_coord.getPWM();
//        pwm_upward_back.data = z_coord.getPWM();

        frontSidewardPublisher.publish(pwm_sideward_front);
        backSidewardPublisher.publish(pwm_sideward_back);

        rightForwardPublisher.publish(pwm_forward_right);
        leftForwardPublisher.publish(pwm_forward_left);

        turnPublisher.publish(pwm_turn);
        sidePublisher.publish(pwm_sideward);

        std::cout << "-------------------------------------------" << std::endl;
        ROS_INFO("PWM forward_right : %d", pwm_forward_right.data);
        ROS_INFO("PWM forward_left : %d", pwm_forward_left.data);
        ROS_INFO("PWM sideward_front : %d", pwm_sideward_front.data);
        ROS_INFO("PWM sideward_back : %d", pwm_sideward_back.data);
        std::cerr << "PWM angle: " << angle.getPWM() <<  std::endl;
        std::cerr << "PWM side: " << y_coord.getPWM() << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

}
