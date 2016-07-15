#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"
#include "ship_los/pose.h"
#include "ship_los/task_input.h"

// forward declaration
class WaypointTrackingClient;
class WaypointTrackingMissionManager;

class WaypointTrackingMission
{
    friend class WaypointTrackingClient;
    friend class WaypointTrackingMissionManager;

private:
    // waypoints
    std::vector<double> wpt_xcoor_;
    std::vector<double> wpt_ycoor_;

    // ID
    std::string id_;

    // mission progress
    uint32_t progress_;

public:
    // constructor
    WaypointTrackingMission() = default;
    WaypointTrackingMission(const std::vector<double> &wpt_xcoor, const std::vector<double> &wpt_ycoor,
                            const std::string &mission_name):
        wpt_xcoor_(wpt_xcoor), wpt_ycoor_(wpt_ycoor), id_(mission_name) {
        progress_ = 0;
    }
};

class WaypointTrackingClient
{
public:
    actionlib::SimpleActionClient<ship_los::WaypointTrackingAction> ac;
    WaypointTrackingMission *ptr_mission_ = nullptr;

    // constructor
    WaypointTrackingClient(): ac("guidance_node", true)
    {
        ROS_INFO("Waiting for an action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started.");
    }

    void send_goal(std::array<double, 3> &ship_pos, WaypointTrackingMission *mission)
    {
        // define the waypoints
        ship_los::WaypointTrackingGoal goal;
        goal.pos_x = mission->wpt_xcoor_;
        goal.pos_y = mission->wpt_ycoor_;

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

        if (ptr_mission_ != nullptr)
        {
            ROS_WARN("------ Previous mission ended (%u/%lu) -------",
                     ptr_mission_->progress_, ptr_mission_->wpt_xcoor_.size());
        }

        // link the pointer to the current mission
        ptr_mission_ = mission;

        ROS_INFO("----- New mission sent to the server -----");
        ROS_INFO("Mission name: %s", ptr_mission_->id_.c_str());
        ROS_INFO("Number of waypoints: %lu", goal.pos_x.size()-1);
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
        ROS_INFO("------------------------------------------");
    }

    // Called once when the goal becomes active
    void activeCb()
    {      
        ROS_INFO("The goal just went active.");
        ROS_INFO("The ship is moving towards wpt1.");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const ship_los::WaypointTrackingFeedbackConstPtr& feedback)
    {
        ptr_mission_->progress_ = feedback->base_wpt;
        ROS_INFO("The base waypoint is now wpt%u.", ptr_mission_->progress_);
    }
};

class WaypointTrackingMissionManager
{
public:
    // the member data
    std::vector<std::vector<double>> wpt_group1 = { {0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372},
                                                    {-1.50, 0.00, 1.50, 2.00, -2.00, -1.50, 0.00, 1.50} };
    std::vector<std::vector<double>> wpt_group2 = { {5.00, 7.00, 9.00, 11.00, 13.00},
                                                    {0.00, 1.00, 1.50, 1.00, 0.00} };
    std::vector<std::vector<double>> wpt_group3 = { {5.00}, {-2.00} };

    WaypointTrackingMission mission1, mission2, mission3;
    std::map<std::string, WaypointTrackingMission> mission_database;
    std::set<std::string> mission_set;
    std::vector<std::string> mission_queue;

    // constructor
    WaypointTrackingMissionManager()
    {
        // initialize the waypoint-tracking missions
        mission1 = WaypointTrackingMission(wpt_group1[0], wpt_group1[1], "mission1");
        mission2 = WaypointTrackingMission(wpt_group2[0], wpt_group2[1], "mission2");
        mission3 = WaypointTrackingMission(wpt_group3[0], wpt_group3[1], "mission3");

        // initialize the associative container for the missions
        mission_database = { {mission1.id_, mission1}, {mission2.id_, mission2}, {mission3.id_, mission3} };

        // initialize the name set
        mission_set = { mission1.id_, mission2.id_, mission3.id_ };
    }
};

// store the real-time position of the ship
std::array<double ,3> ship_pos;

void callback_pos(const ship_los::pose &msg_pos)
{
    ship_pos[0] = msg_pos.x;
    ship_pos[1] = msg_pos.y;
    ship_pos[2] = msg_pos.heading;
}

bool received_command = false;
// mission name from the user
std::string mission_name;

bool userCallback(ship_los::task_input::Request &req, ship_los::task_input::Response &resp)
{
    if (req.command == "overwrite" )
    {
        mission_name = req.groupName;
        received_command = true;
    }

    ROS_WARN("Received command from the user.");
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

    // create a waypoint-tracking mission manager
    WaypointTrackingMissionManager manager;

    ros::Rate r(10);

    while (ros::ok()) {
        if (received_command)
        {
            if ( manager.mission_set.find(mission_name) != manager.mission_set.end() )
            {
                ROS_INFO("The mission exists in the database.");
                client.send_goal(ship_pos, &(manager.mission_database.at(mission_name)));
                manager.mission_queue.clear();
                manager.mission_queue.push_back(mission_name);
            }
            else
            {
                ROS_ERROR("The mission doesn't exist in the database.");
            }

            // reset the helper variables
            received_command = false;
            mission_name = "";
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
