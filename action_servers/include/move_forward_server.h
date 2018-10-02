#ifndef MOVE_FORWARD_SERVER_H
#define MOVE_FORWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <action_servers/anglePIDAction.h>
#include <action_servers/upwardPIDAction.h>
#include <action_servers/sidewardPIDAction.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include <boost/thread.hpp>

class moveForward {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<action_servers::upwardPIDAction> upwardPIDClient_sensor_;
    actionlib::SimpleActionClient<action_servers::upwardPIDAction> upwardPIDClient_vision_;
    actionlib::SimpleActionClient<action_servers::anglePIDAction> anglePIDClient_sensor_;
    actionlib::SimpleActionClient<action_servers::anglePIDAction> anglePIDClient_vision_;
    actionlib::SimpleActionClient<action_servers::sidewardPIDAction> sidewardPIDClient_;
    ros::Subscriber angle_sub_;
    ros::Subscriber depth_sub_;

    action_servers::sidewardPIDGoal sideward_PID_goal;
    action_servers::upwardPIDGoal upward_PID_goal;
    action_servers::anglePIDGoal angle_PID_goal;

    double angle;
    double depth;
    boost::thread* spin_thread;
    std::string upward_type_;
    std::string angle_type_;

public:

    moveForward(int pwm_);
    ~moveForward();

    void setActive(bool);
    void spinThread();
    void setReferenceAngle(double);
    void setReferenceDepth(double);
    void setDataSource(std::string, std::string);
    void imuAngleCB(const std_msgs::Float64Ptr &_msg);
    void depthCB(const std_msgs::Float64Ptr &_msg); 
};
#endif // MOVE_FORWARD_SERVER_H
