#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ship_los/WaypointTrackingAction.h"
#include "ship_los/pose.h"
#include "ship_los/task_input.h"

class WaypointTrackingMission
{
private:
    // waypoints
    std::vector<double> wpt_xcoor_;
    std::vector<double> wpt_ycoor_;

    // ID
    std::string id_;

    // mission progress
    uint32_t progress_ = 0;

public:
    // constructor
    WaypointTrackingMission() = default;
    WaypointTrackingMission(const std::vector<double> &wpt_xcoor, const std::vector<double> &wpt_ycoor,
                            const std::string &mission_name):
        wpt_xcoor_(wpt_xcoor), wpt_ycoor_(wpt_ycoor), id_(mission_name) {}

    // member function resetting the progress
    void reset_progress()
    {
        progress_ = 0;
        ROS_INFO("The progress of mission %s is reset.", id_.c_str());
    }

    void set_progress(uint32_t p)
    {
        if (p > wpt_xcoor_.size())
        {
            ROS_ERROR("set_progress failed: the progress shouldn't be larger than the waypoint number.");
            return;
        }
        progress_ = p;
    }

    std::string get_name() const { return id_; }
    uint32_t get_progress() const { return progress_; }
    size_t get_size() const { return wpt_xcoor_.size(); }
    std::vector<double> get_xcoor() const { return wpt_xcoor_; }
    std::vector<double> get_ycoor() const { return wpt_ycoor_; }
};

class WaypointTrackingClient
{
private:
    actionlib::SimpleActionClient<ship_los::WaypointTrackingAction> ac;
    WaypointTrackingMission *ptr_mission_ = nullptr;

    // initial progress of the mission
    uint32_t init_progress_;

    // the id of the current mission
    std::string mission_id_;
    // whether the current mission is done
    bool is_done_ = false;

public:
    // constructor
    WaypointTrackingClient(const std::string &server_name): ac(server_name, true)
    {
        ROS_INFO("Waiting for an action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started.");
    }

    void send_goal(const std::array<double, 3> &ship_pos, WaypointTrackingMission *mission)
    {
        // define the waypoints
        ship_los::WaypointTrackingGoal goal;

        goal.pos_x = { ship_pos[0] };
        goal.pos_y = { ship_pos[1] };

        std::vector<double> xcoor = mission->get_xcoor();
        std::vector<double> ycoor = mission->get_ycoor();

        if (mission->get_progress() < mission->get_size())
        {
            for (auto p = xcoor.begin()+mission->get_progress(); p != xcoor.end(); ++p)
            {
                goal.pos_x.push_back(*p);
            }

            for (auto p = ycoor.begin()+mission->get_progress(); p != ycoor.end(); ++p)
            {
                goal.pos_y.push_back(*p);
            }
        }
        else
        {
            ROS_ERROR("%s is already completed.", mission->get_name().c_str());
            ROS_ERROR("Reset its progress before run it again.");
            return;
        }

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

        if (ptr_mission_ != nullptr)
        {
            ROS_WARN("------ Previous mission ended (%u/%lu) ------",
                     ptr_mission_->get_progress(), ptr_mission_->get_size());
        }

        // link the pointer to the current mission
        ptr_mission_ = mission;

        // reset the status variables
        init_progress_ = ptr_mission_->get_progress();
        mission_id_ = ptr_mission_->get_name();
        is_done_ = false;

        // display the information of the mission
        ROS_INFO("----- New mission sent to the server -----");
        ROS_INFO("Mission name: %s", ptr_mission_->get_name().c_str());
        ROS_INFO("Initial progress: %u/%lu", init_progress_, ptr_mission_->get_size());
        ROS_INFO("Number of sent waypoints: %lu", goal.pos_x.size()-1);
        ROS_INFO("The coordinates of the waypoints are:");
        ROS_INFO("wpt ----- x (m) ------- y (m)");

        for (uint32_t i = 0; i < goal.pos_x.size(); ++i)
        {
            if (i > 0)
            {
                ROS_INFO("%2u ------ %5.2f ------- %5.2f",
                         i+ptr_mission_->get_progress(), goal.pos_x[i],  goal.pos_y[i]);
            }
            else
            {
                ROS_INFO("%2u ------ %5.2f ------- %5.2f",
                         i, goal.pos_x[i],  goal.pos_y[i]);
            }
        }

        // send the goal to the action server
        ac.sendGoal(goal,
                    boost::bind(&WaypointTrackingClient::doneCb, this, _1, _2),
                    boost::bind(&WaypointTrackingClient::activeCb, this),
                    boost::bind(&WaypointTrackingClient::feedbackCb, this, _1));
    }

    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const ship_los::WaypointTrackingResultConstPtr& result)
    {
        is_done_ = true;
        ROS_INFO("Waypoint-tracking finished in state [%s]", state.toString().c_str());
        ROS_INFO("The last waypoint reached: wpt%i", result->final_wpt);
        ROS_INFO("------------------------------------------");
    }

    // Called once when the goal becomes active
    void activeCb()
    {      
        ROS_INFO("The goal just went active.");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const ship_los::WaypointTrackingFeedbackConstPtr& feedback)
    {
        ptr_mission_->set_progress(feedback->progress + init_progress_);
        ROS_INFO("Progress of the mission: %u/%lu", ptr_mission_->get_progress(), ptr_mission_->get_size());
    }

    // return whether the current mission is finished or not
    bool is_done() const { return is_done_; }

    // return the id of the current mission
    std::string mission_id() const { return mission_id_; }
};

class WaypointTrackingMissionManager
{
public:
    // the member data
    std::vector<std::vector<double>> wpt_group1 = { {0.372, -0.628, 0.372, 1.872, 6.872, 8.372, 9.372, 8.372},
                                                    {-1.50, 0.00, 1.50, 2.00, -2.00, -1.50, 0.00, 1.50} };
    std::vector<std::vector<double>> wpt_group2 = { {6.00, 7.00, 9.00, 11.00, 12.00, 11.00},
                                                    {4.00, 6.00, 6.50, 6.00, 4.00, 2.00} };
    std::vector<std::vector<double>> wpt_group3 = { {5.00}, {-2.00} };

    WaypointTrackingMission mission1, mission2, mission3;
    std::map<std::string, WaypointTrackingMission> mission_database_;
    std::set<std::string> mission_set_;
    std::deque<std::string> mission_queue_;

    // constructor
    WaypointTrackingMissionManager()
    {
        // initialize the waypoint-tracking missions
        mission1 = WaypointTrackingMission(wpt_group1[0], wpt_group1[1], "mission1");
        mission2 = WaypointTrackingMission(wpt_group2[0], wpt_group2[1], "mission2");
        mission3 = WaypointTrackingMission(wpt_group3[0], wpt_group3[1], "mission3");

        // initialize the associative container for the missions
        mission_database_ = { {mission1.get_name(), mission1}, {mission2.get_name(), mission2}, {mission3.get_name(), mission3} };

        // initialize the name set
        mission_set_ = { mission1.get_name(), mission2.get_name(), mission3.get_name() };
    }
};

class UserInterface
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pos_;
    ros::ServiceServer srv_user_;

    // store the real-time position of the ship
    std::array<double,3> ship_pos_;

    // helper variable
    bool received_command_ = false;

    // command inputs from the user
    std::string mission_name_;
    std::string command_name_;

    // command set
    const std::vector<std::string> command_set_ = {"overwrite", "append", "insert"};

public:
    // constructor
    UserInterface(){
        // subscriber to ship/pos
        sub_pos_ = nh_.subscribe("ship/pose", 1000, &UserInterface::callback_pos, this);

        // service responding to user inputs
        srv_user_ = nh_.advertiseService("task_command", &UserInterface::userCallback, this);
    }

    // return received_command_
    bool command_available() { return received_command_; }

    // set ship_pos_
    void set_pose(const double &x, const double &y, const double &heading)
    {
        ship_pos_[0] = x;
        ship_pos_[1] = y;
        ship_pos_[2] = heading;
    }

    // return ship_pos_
    const std::array<double, 3> &get_pos() const { return ship_pos_; }

    // callback function of the subscriber to ship/pose
    void callback_pos(const ship_los::poseConstPtr &msg_pos)
    {
        ship_pos_[0] = msg_pos->x;
        ship_pos_[1] = msg_pos->y;
        ship_pos_[2] = msg_pos->heading;
    }

    // callback of the service for user inputs
    bool userCallback(ship_los::task_input::Request &req, ship_los::task_input::Response &resp)
    {
        ROS_WARN("Received command from the user.");
        resp.feedback = "User\'s command received.";

        if ( std::find(command_set_.begin(), command_set_.end(), req.command) == command_set_.end() )
        {
            ROS_ERROR("The command is not valid.");
            resp.feedback += " Your command is not valid.";
        }
        else
        {
            mission_name_ = req.groupName;
            command_name_ = req.command;
            received_command_ = true;
        }

        return true;
    }

    // receive user command and manage waypoint-tracking mission
    void CommandCallback(WaypointTrackingClient &client, WaypointTrackingMissionManager &manager)
    {
        if ( manager.mission_set_.find(mission_name_) != manager.mission_set_.end() )
        {
            ROS_WARN("%s exists in the database.", mission_name_.c_str());

            if (command_name_ == "append")
            {
                manager.mission_queue_.push_back(mission_name_);
                if (manager.mission_queue_.size() == 1)
                {
                   client.send_goal(ship_pos_, &(manager.mission_database_.at(mission_name_)));
                }
                else
                {
                    ROS_WARN("%s is appended to the queue.", mission_name_.c_str());
                }
            }
            if (command_name_ == "overwrite")
            {
                if ( !manager.mission_queue_.empty() )
                {
                    manager.mission_queue_.front() = mission_name_;
                }
                else
                {
                    manager.mission_queue_.push_back(mission_name_);
                }
                client.send_goal(ship_pos_, &(manager.mission_database_.at(mission_name_)));
            }
            if (command_name_ == "insert")
            {
                manager.mission_queue_.push_front(mission_name_);
                client.send_goal(ship_pos_, &(manager.mission_database_.at(mission_name_)));
            }

            std::string mission_queue;
            for (auto iter = manager.mission_queue_.cbegin(); iter != manager.mission_queue_.cend(); ++iter)
            {
                mission_queue += *iter;
                mission_queue += " -> ";
            }
            ROS_WARN("Mission queue: %s", mission_queue.c_str());
        }
        else
        {
            ROS_ERROR("The mission doesn't exist in the database.");
        }

        // reset the flag and mission_name_
        received_command_ = false;
        mission_name_.clear();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");

    // create the user interface
    UserInterface user_interface;

    // define parameters for the ship's initial position
    const std::string PARAM_1 = "~init_x";
    const std::string PARAM_2 = "~init_y";
    const std::string PARAM_3 = "~init_heading";
    double init_x, init_y, init_heading;
    bool ok_1 = ros::param::get(PARAM_1, init_x);
    bool ok_2 = ros::param::get(PARAM_2, init_y);
    bool ok_3 = ros::param::get(PARAM_3, init_heading);

    // create an actionlib client and send goals
    WaypointTrackingClient client("guidance_node");

    // create a waypoint-tracking mission manager
    WaypointTrackingMissionManager manager;

    // initialize the ship position
    if ( ok_1 && ok_2 && ok_3 )
        user_interface.set_pose(init_x, init_y, init_heading);

    ros::Rate r(10);

    while (ros::ok())
    {
        // keep checking the incoming user command
        if (user_interface.command_available())
        {
            user_interface.CommandCallback(client, manager);
        }

        // keep checking whether the current mission is done
        if (client.is_done())
        {
            if ( !manager.mission_queue_.empty() )
            {
                manager.mission_queue_.pop_front();
                if ( !manager.mission_queue_.empty() )
                {
                    ROS_WARN("Send the next mission in the queue.");
                    client.send_goal(user_interface.get_pos(), &(manager.mission_database_.at(manager.mission_queue_.front())));
                }
            }
            else
            {
                ROS_WARN_THROTTLE(10, "%s is finished. No more missions in the queue.",
                                                  client.mission_id().c_str());
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
