#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"
#include "ship_los/pose.h"
#include "ship_los/task_input.h"

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

    void send_goal(std::array<double, 3> &ship_pos, const std::vector<double> &wpt_x, const std::vector<double> &wpt_y) {

        // define the waypoints
        ship_los::WaypointTrackingGoal goal;
        goal.pos_x = wpt_x;
        goal.pos_y = wpt_y;

        // insert the ship position as the fisrt waypoint
        goal.pos_x.insert(goal.pos_x.begin(), ship_pos[0]);
        goal.pos_y.insert(goal.pos_y.begin(), ship_pos[1]);

        // send the initial heading to initialze the los guidance controller
        goal.init_heading = ship_pos[2];

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

// waypoints database
std::vector<double> wpt_xcoor = {0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372};
std::vector<double> wpt_ycoor = {-1.50, 0.00, 1.50, 2.00, -2.00, -1.50, 0.00, 1.50};

// store the current position of the ship
std::array<double ,3> ship_pos;

void callback_pos(const ship_los::pose &msg_pos)
{
    ship_pos[0] = msg_pos.x;
    ship_pos[1] = msg_pos.y;
    ship_pos[2] = msg_pos.heading;
}

bool received_command = false;
bool userCallback(ship_los::task_input::Request &req, ship_los::task_input::Response &resp)
{
    if (req.command == "overwrite" )
        received_command = true;

    ROS_INFO("Received command from the user.");
    resp.feedback = "command received";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle nh;

    // define parameters for the ship's initial position
    const std::string PARAM_1 = "~init_x";
    const std::string PARAM_2 = "~init_y";
    const std::string PARAM_3 = "~init_heading";
    double init_x, init_y, init_heading;
    bool ok_1 = ros::param::get(PARAM_1, init_x);
    bool ok_2 = ros::param::get(PARAM_2, init_y);
    bool ok_3 = ros::param::get(PARAM_3, init_heading);

    if ( ok_1 && ok_2 && ok_3 )
    {
        ship_pos[0] = init_x;
        ship_pos[1] = init_y;
        ship_pos[2] = init_heading;
    }

    // subscriber to ship/pos
    ros::Subscriber sub_pos = nh.subscribe("ship/pose", 1000, &callback_pos);

    // service responding to user inputs
    ros::ServiceServer srv_user = nh.advertiseService("task_command", &userCallback);

    // create an actionlib client and send goals
    WaypointTrackingClient client;

    ros::Rate r(10);

    while (ros::ok()) {
        if (received_command) {
            client.send_goal(ship_pos, wpt_xcoor, wpt_ycoor);
            received_command = false;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
