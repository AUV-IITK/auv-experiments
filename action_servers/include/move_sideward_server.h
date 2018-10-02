#ifndef MOVE_SIDEWARD_SERVER_H
#define MOVE_SIDEWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_servers/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <string>

class moveSideward {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<action_servers::anglePIDAction> anglePIDClient;    
    action_servers::anglePIDGoal angle_PID_goal;
    ros::Subscriber sub_;
    double angle;
    boost::thread* spin_thread;

public:

    moveSideward(int);
    ~moveSideward();

    void setActive(bool);
    void spinThread();
    void imuAngleCB(const std_msgs::Float64Ptr &_msg);
};
#endif // MOVE_SIDEWARD_SERVER_H
