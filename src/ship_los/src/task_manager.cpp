#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"

class WaypointTrackingClient
{
private:
    actionlib::SimpleActionClient<ship_los::WaypointTrackingAction> ac;

public:
    // constructor
    WaypointTrackingClient(): ac("guidance_node", true)
    {
        ROS_INFO("Waiting for an action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started.");
    }

    void send_goal(const std::vector<double> &wpt_x, const std::vector<double> &wpt_y) {

        // define the waypoints
        ship_los::WaypointTrackingGoal goal;
        goal.pos_x = wpt_x;
        goal.pos_y = wpt_y;

        if (goal.pos_x.size() != goal.pos_y.size())
        {
            ROS_ERROR_STREAM("The sizes of the coordinates don't match. No waypoints sent.");
            return;
        }

        if (goal.pos_x.empty() || goal.pos_y.empty())
        {
            ROS_ERROR_STREAM("The coordinates are empty. No waypoints sent");
            return;
        }

        ac.sendGoal(goal,
                    boost::bind(&WaypointTrackingClient::doneCb, this, _1, _2),
                    boost::bind(&WaypointTrackingClient::activeCb, this),
                    boost::bind(&WaypointTrackingClient::feedbackCb, this, _1));

        ROS_INFO("The goal with %lu waypoints were sent.", goal.pos_x.size());
        ROS_INFO("The coordinates of the waypoints are:");
        ROS_INFO("wpt ----- x (m) ------- y (m)");

        for (uint32_t i = 0; i < goal.pos_x.size(); ++i)
        {
            ROS_INFO("%2u ------ %5.2f ------- %5.2f",
                     i, goal.pos_x[i],  goal.pos_y[i]);
        }
    }

    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const ship_los::WaypointTrackingResultConstPtr& result)
    {
        ROS_INFO("Waypoint-tracking finished in state [%s]", state.toString().c_str());
        ROS_INFO("The last waypoint reached: wpt%i", result->final_wpt);
    }

    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("The goal just went active.");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const ship_los::WaypointTrackingFeedbackConstPtr& feedback)
    {
        ROS_INFO("The base waypoint is now wpt%u.", feedback->base_wpt);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");

    std::vector<double> wpt_xcoor = {0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372};
    std::vector<double> wpt_ycoor = {-1.50, 0.00, 1.50, 2.00, -2.00, -1.50, 0.00, 1.50};

    WaypointTrackingClient client;
    client.send_goal(wpt_xcoor, wpt_ycoor);

    ros::spin();

    return 0;
}
