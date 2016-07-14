#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const ship_los::WaypointTrackingResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->final_wpt);
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active.");
}

// Called every time feedback is received for the goal
void feedbackCb(const ship_los::WaypointTrackingFeedbackConstPtr& feedback)
{
    ROS_INFO("The base waypoint now is wpt%u.", feedback->base_wpt);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");

    // create the action client
    actionlib::SimpleActionClient<ship_los::WaypointTrackingAction> ac("guidance_node", true);

    ROS_INFO("Waiting for an action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started.");

    ship_los::WaypointTrackingGoal goal;

    goal.pos_x = {0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372};
    goal.pos_y = {-1.50, 0.00, 1.50, 2.00, -2.00, -1.50, 0.00, 1.50};

    if (goal.pos_x.size() != goal.pos_y.size())
    {
        ROS_FATAL_STREAM("The sizes of the coordinates don't match");
        exit(1);
    }

    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ROS_INFO("Waypoints sent.");

    ros::spin();

    return 0;

}
