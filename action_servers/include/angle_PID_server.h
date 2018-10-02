#ifndef ANGLE_PID_SERVER_H
#define ANGLE_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_servers/anglePIDAction.h>
#include <errorToPWM.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

class anglePIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<action_servers::anglePIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    action_servers::anglePIDFeedback feedback_;
    action_servers::anglePIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor angle;

    std::string type;

public:

    anglePIDAction(std::string, std::string);
    ~anglePIDAction();
    void goalCB();
    void preemptCB();
    void sensorCB(const std_msgs::Float32ConstPtr&);
    void visionCB(const geometry_msgs::Pose2DConstPtr&);
    void setDataSource(std::string);

};
#endif // ANGLE_PID_SERVER_H
