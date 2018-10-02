#ifndef MOVE_SIDEWARD_SERVER_H
#define MOVE_SIDEWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_servers/sidewardPIDAction.h>
#include <errorToPWM.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

class sidewardPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<action_servers::sidewardPIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    action_servers::sidewardPIDFeedback feedback_;
    action_servers::sidewardPIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor y_coord;

public:

    sidewardPIDAction(std::string);
    ~sidewardPIDAction();
    void goalCB();
    void preemptCB();
    void visionCB(const geometry_msgs::PointStampedConstPtr&);

};
#endif // MOVE_SIDEWARD_SERVER_H
