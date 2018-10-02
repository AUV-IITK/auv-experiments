#include <ros/ros.h>
#include <buoy.h>
#include <line.h>
#include <gate.h>
#include <straight_server.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    lineTask line_;
    line_.setActive(true);
    ros::Duration(5).sleep();
    line_.setActive(false);
    buoy buoy_;
    buoy_.setActive(true);
    // TRANSITION FROM BUOY TASK TO GATE TASK //
    gateTask gate_;
    gate_.setActive(true);
    ros::Duration(5).sleep();
    // TRANSITION FROM GATE TASK TO TORPEDO TASK //
    ros::spin();
    return 0;
}