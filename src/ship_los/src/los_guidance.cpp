#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <boost/numeric/odeint.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "ship_los/pose.h"
#include "ship_los/course.h"
#include "ship_los/waypoint.h"
#include "los_guidance_law.h"
#include "ship_los/WaypointTrackingAction.h"

using namespace std;
using namespace boost::numeric::odeint;


class WaypointTracking
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ship_los::WaypointTrackingAction> as_;

    // the timer
    ros::Timer timer_;

    // the publishers
    ros::Publisher pub_markers_;
    ros::Publisher pub_markerArray_;
    ros::Publisher pub_course_;

    // the messages
    visualization_msgs::Marker marker_wpt_;
    visualization_msgs::Marker marker_bulb_;
    visualization_msgs::MarkerArray marker_textArray_;
    ship_los::course msg_course_;

    // the subscriber
    ros::Subscriber sub_pos_;

    // the server
     ros::ServiceServer srv_insert_;

    // the waypoints
    vector<array<double, 2>> waypoints_ = { {0.372, -1.50},
                                           {-0.628, 0.00},
                                           {0.372, 1.50},
                                           {1.872, 2.00},
                                           {6.872, -2.00},
                                           {8.372, -1.50},
                                           {9.372, 0.00},
                                           {8.372, 1.50} };

    // the number of the waypoint which the ship just passed
    uint32_t wpt_num_ = 0;
    vector<array<double, 2>>::iterator iter_;

    // generated course infomation
    vector<double> course_state_ ;

    // radius of the capture circle
    double radius_;

    // los guidance controlller
    los_guidance_law::LosGuidanceLaw los_controller_;

    // action feedback and result
    ship_los::WaypointTrackingFeedback feedback_;
    ship_los::WaypointTrackingResult result_;

public:
    WaypointTracking(std::string name):
        as_(nh_, name, false)
    {
        // register the goal and feedback callbacks
        as_.registerGoalCallback(boost::bind(&WaypointTracking::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&WaypointTracking::preemptCB, this));

        // the timer to control the publish to rviz
        timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointTracking::timerCallback, this);

        // subscriber
        sub_pos_ = nh_.subscribe("ship/pose", 1000, &WaypointTracking::callback_pos, this);

        // publishers
        pub_course_ = nh_.advertise<ship_los::course>("ship/course", 1000);
        pub_markers_ = nh_.advertise<visualization_msgs::Marker>("ship/waypoints", 1000);
        pub_markerArray_ = nh_.advertise<visualization_msgs::MarkerArray>("ship/waypoint_label", 1000);

        // define the service used to add waypoints
        srv_insert_ = nh_.advertiseService("insert_waypoint", &WaypointTracking::insertCallback, this);

        // define the marker as the waypoints
        marker_wpt_.header.frame_id = "world";
        marker_wpt_.header.stamp = ros::Time::now();
        marker_wpt_.ns = "waypoints/viz";
        marker_wpt_.action = visualization_msgs::Marker::ADD;
        marker_wpt_.pose.orientation.w = 1.0;
        marker_wpt_.id = 0;
        marker_wpt_.type = visualization_msgs::Marker::LINE_LIST;
        marker_wpt_.scale.x = 0.1; // line width
        marker_wpt_.color.g = 1.0;
        marker_wpt_.color.a = 1.0;
        marker_wpt_.lifetime = ros::Duration();

        for (uint32_t i = 0; i < waypoints_.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = waypoints_[i][0];
            p.y = -waypoints_[i][1];
            p.z = 0;
            marker_wpt_.points.push_back(p);
            p.z += 1.5;
            marker_wpt_.points.push_back(p);
        }

        // define a marker to highlight the waypoint to which the ship is pointed
        marker_bulb_.header.frame_id = "world";
        marker_bulb_.header.stamp = ros::Time::now();
        marker_bulb_.ns = "waypoints/viz";
        marker_bulb_.action = visualization_msgs::Marker::ADD;
        marker_bulb_.id = 1;
        marker_bulb_.type = visualization_msgs::Marker::SPHERE;
        marker_bulb_.scale.x = 0.2;
        marker_bulb_.scale.y = 0.2;
        marker_bulb_.scale.z = 0.2;
        marker_bulb_.color.r = 1.0; // red
        marker_bulb_.color.a = 1.0;
        marker_bulb_.lifetime = ros::Duration();

        // initialize the los guidance controlller
        course_state_ = {1.78, 0, 0};
        radius_ = 1.0;
        los_controller_ = los_guidance_law::LosGuidanceLaw(course_state_, radius_, 0.05);

        // start the action server
        as_.start();
        ROS_INFO("server starts");
    }

    ~WaypointTracking(void) {}

    void goalCB()
    {
        iter_ = ++waypoints_.begin();
        ROS_INFO("Goal received");
        as_.acceptNewGoal();
        ROS_INFO("start working");
    }

    void preemptCB()
    {
        ROS_INFO("Preempted");
        // set the action state to preempted
        result_.final_index = wpt_num_;
        as_.setPreempted(result_);
    }

    void callback_pos(const ship_los::pose &msg_pos);
    bool insertCallback(ship_los::waypoint::Request &req, ship_los::waypoint::Response &resp);
    void timerCallback(const ros::TimerEvent &event);

};

// callback of the subscriber, where desired course is computed and published
void WaypointTracking::callback_pos(const ship_los::pose &msg_pos)
{
    if (!as_.isActive()) {
        ROS_INFO("action server is not active");
        return; }

    ROS_INFO("action server is active");

    if (wpt_num_ < waypoints_.size()-1)
    {
        array<double, 3> course_info = los_guidance_law::ComputeCourse(los_controller_, msg_pos.x, msg_pos.y, msg_pos.heading,
                                                    waypoints_[wpt_num_][0], waypoints_[wpt_num_+1][0],
                                                    waypoints_[wpt_num_][1], waypoints_[wpt_num_+1][1]);

        // publish the course info
        msg_course_.angle = course_info[0];
        msg_course_.rate = course_info[1];
        msg_course_.acceleration = course_info[2];
        pub_course_.publish(msg_course_);

        // the distance between the ship and the waypoint to which it points
        double distance = sqrt(pow(msg_pos.x - waypoints_[wpt_num_+1][0], 2) +
                               pow(msg_pos.y - waypoints_[wpt_num_+1][1], 2));
        ROS_INFO("Base: wpt%u, Distance to wpt%u: %5.2f m", wpt_num_, wpt_num_+1, distance);

        // change the waypoint if necessary
        if ( distance <= radius_)
        {
            ++ wpt_num_;
            ++ iter_;
            ROS_INFO("Change the current waypoint to: wpt%u", wpt_num_);
        }
    }
    else
    {
        ROS_WARN("The ship has reached the last waypoint.");
    }


    // publish the markers to rviz
    if (wpt_num_ < waypoints_.size()-1) {
        marker_bulb_.pose.position.x = waypoints_[wpt_num_+1][0];
        marker_bulb_.pose.position.y = -waypoints_[wpt_num_+1][1];
        marker_bulb_.pose.position.z = 1.5;
    }
}

// callback of the service server
bool WaypointTracking::insertCallback(ship_los::waypoint::Request &req, ship_los::waypoint::Response &resp)
{
    iter_ = waypoints_.insert(iter_, {req.x, req.y});
    string feedback = "The waypoint is inserted as wpt";
    resp.feedback =  feedback + to_string(wpt_num_+1);

    // send the new waypoint to rviz
    geometry_msgs::Point p;
    p.x = req.x;
    p.y = -req.y;
    p.z = 0;
    marker_wpt_.points.push_back(p);
    p.z += 1.5;
    marker_wpt_.points.push_back(p);

    return true;
}

// callback of the timer
void WaypointTracking::timerCallback(const ros::TimerEvent &event)
{
    // define the waypoint labels
    marker_textArray_.markers.clear();

    for (uint32_t i = 0; i < waypoints_.size(); ++i)
    {
        visualization_msgs::Marker marker_text;
        string name = "wpt";
        marker_text.header.frame_id = "world";
        marker_text.header.stamp = ros::Time::now();
        marker_text.ns = "waypoints/viz";
        marker_text.action = visualization_msgs::Marker::ADD;
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.id = i+2;
        marker_text.scale.z = 0.2;
        marker_text.text = name + to_string(i);
        marker_text.pose.position.x = waypoints_[i][0];
        marker_text.pose.position.y = -waypoints_[i][1]-0.2;
        marker_text.pose.position.z = 1.0;
        marker_text.color.r = 1.0; // white
        marker_text.color.b = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.a = 1.0;

        marker_textArray_.markers.push_back(marker_text);
    }

    pub_markers_.publish(marker_wpt_);
    pub_markers_.publish(marker_bulb_);
    pub_markerArray_.publish(marker_textArray_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_guidance");

    WaypointTracking waypoint_tracking(ros::this_node::getName());

    ros::spin();

    return 0;
}
