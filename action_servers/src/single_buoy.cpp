#include <single_buoy.h>

singleBuoy::singleBuoy(): move_forward_(150), move_sideward_(100), move_straight_(100), forwardPIDClient("forwardPID") {
    sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &singleBuoy::forwardCB, this);
}
singleBuoy::~singleBuoy() {}

void singleBuoy::setActive(bool status) {
    move_forward_.setDataSource("SENSOR", "VISION");
    move_forward_.setActive(true);

    while(forward_distance_ >= 60) {
        continue;
    }
    move_forward_.setActive(false);

    move_straight_.setActive(true);
    ros::Duration(6).sleep();
    move_straight_.setActive(false);
    move_straight_.setActive(true);
    ros::Duration(10).sleep();
    move_straight_.setActive(false);

    ROS_INFO("Waiting for action server to start.");
    forwardPIDClient.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    action_servers::forwardPIDGoal goal;
    goal.target_distance = 100;
    forwardPIDClient.sendGoal(goal);

    bool finished_before_timeout = forwardPIDClient.waitForResult(ros::Duration(15.0));
    
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = forwardPIDClient.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

}

void singleBuoy::forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    forward_distance_ = _msg->point.x;
    depth_ = _msg->point.z;
}