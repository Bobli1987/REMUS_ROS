#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");

    // create the action client
    actionlib::SimpleActionClient<ship_los::WaypointTrackingAction> ac("guidance_node", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started.");

    ship_los::WaypointTrackingGoal goal;

    goal.pos_x = {0};
    goal.pos_y = {1};

    ac.sendGoal(goal);

    ros::spin();

    return 0;

}
