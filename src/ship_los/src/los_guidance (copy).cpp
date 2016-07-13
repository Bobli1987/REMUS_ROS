#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "ship_los/pose.h"
#include "ship_los/course.h"
#include "ship_los/waypoint.h"
#include <cmath>
#include <boost/numeric/odeint.hpp>
#include <boost/math/special_functions/sign.hpp>
#include "los_guidance_law.h"

using namespace std;
using namespace boost::numeric::odeint;

// the waypoints
vector<array<double, 2>> waypoints = { {0.372, -1.50},
                                       {-0.628, 0.00},
                                       {0.372, 1.50},
                                       {1.872, 2.00},
                                       {6.872, -2.00},
                                       {8.372, -1.50},
                                       {9.372, 0.00},
                                       {8.372, 1.50} };

// the publishers
ros::Publisher *pubPtr_course;
ros::Publisher *pubPtr_markers;
ros::Publisher *pubPtr_markerArray;

// the messages
visualization_msgs::Marker marker_wpt;
visualization_msgs::Marker marker_bulb;
visualization_msgs::MarkerArray marker_textArray;
ship_los::course msg_course;

// the number of the waypoint which the ship just passed
uint32_t wpt_num = 0;
auto iter = ++waypoints.begin();

// generated course infomation
vector<double> course_state = {1.78, 0, 0};

// radius of the capture circle
const double radius = 1.0;

// initialize the los guidance controlller
los_guidance_law::LosGuidanceLaw los_controller = los_guidance_law::LosGuidanceLaw(course_state, radius, 0.05);

// callback of the service server
bool insertCallback(ship_los::waypoint::Request &req, ship_los::waypoint::Response &resp)
{
    iter = waypoints.insert(iter, {req.x, req.y});
    string feedback = "The waypoint is inserted as wpt";
    resp.feedback =  feedback + to_string(wpt_num+1);

    // send the new waypoint to rviz
    geometry_msgs::Point p;
    p.x = req.x;
    p.y = -req.y;
    p.z = 0;
    marker_wpt.points.push_back(p);
    p.z += 1.5;
    marker_wpt.points.push_back(p);

    return true;
}

// callback of the subscriber, where desired course is computed and published
void callback_pos(const ship_los::pose &msg_pos)
{
    if (wpt_num < waypoints.size()-1)
    {
        array<double, 3> course_info = los_guidance_law::ComputeCourse(los_controller, msg_pos.x, msg_pos.y, msg_pos.heading,
                                                    waypoints[wpt_num][0], waypoints[wpt_num+1][0],
                                                    waypoints[wpt_num][1], waypoints[wpt_num+1][1]);

        // publish the course info
        msg_course.angle = course_info[0];
        msg_course.rate = course_info[1];
        msg_course.acceleration = course_info[2];
        pubPtr_course->publish(msg_course);

        // the distance between the ship and the waypoint to which it points
        double distance = sqrt(pow(msg_pos.x - waypoints[wpt_num+1][0], 2) +
                               pow(msg_pos.y - waypoints[wpt_num+1][1], 2));
        ROS_INFO("Base: wpt%u, Distance to wpt%u: %5.2f m", wpt_num, wpt_num+1, distance);

        // change the waypoint if necessary
        if ( distance <= radius)
        {
            ++ wpt_num;
            ++ iter;
            ROS_INFO("Change the current waypoint to: wpt%u", wpt_num);
        }
    }
    else
    {
        ROS_WARN("The ship has reached the last waypoint.");
    }


    // publish the markers to rviz
    if (wpt_num < waypoints.size()-1) {
        marker_bulb.pose.position.x = waypoints[wpt_num+1][0];
        marker_bulb.pose.position.y = -waypoints[wpt_num+1][1];
        marker_bulb.pose.position.z = 1.5;
    }
}

void timerCallback(const ros::TimerEvent)
{
    // define the waypoint labels
    marker_textArray.markers.clear();

    for (uint32_t i = 0; i < waypoints.size(); ++i)
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
        marker_text.pose.position.x = waypoints[i][0];
        marker_text.pose.position.y = -waypoints[i][1]-0.2;
        marker_text.pose.position.z = 1.0;
        marker_text.color.r = 1.0; // white
        marker_text.color.b = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.a = 1.0;

        marker_textArray.markers.push_back(marker_text);
    }

    pubPtr_markers->publish(marker_wpt);
    pubPtr_markers->publish(marker_bulb);
    pubPtr_markerArray->publish(marker_textArray);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_guidance");
    ros::NodeHandle nh;

    pubPtr_markers = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("ship/waypoints", 1000));
    pubPtr_markerArray = new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("ship/waypoint_label", 1000));
    pubPtr_course = new ros::Publisher(nh.advertise<ship_los::course>("ship/course", 1000));

    // create a timer to control the publishing of waypoints
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), &timerCallback);

    // define the service used to add waypoints
    ros::ServiceServer srv_insert = nh.advertiseService("insert_waypoint", &insertCallback);

    ros::Subscriber sub_pos = nh.subscribe("ship/pose", 1000, &callback_pos);

    // define the marker as the waypoints
    marker_wpt.header.frame_id = "world";
    marker_wpt.header.stamp = ros::Time::now();
    marker_wpt.ns = "waypoints/viz";
    marker_wpt.action = visualization_msgs::Marker::ADD;
    marker_wpt.pose.orientation.w = 1.0;
    marker_wpt.id = 0;
    marker_wpt.type = visualization_msgs::Marker::LINE_LIST;
    marker_wpt.scale.x = 0.1; // line width
    marker_wpt.color.g = 1.0;
    marker_wpt.color.a = 1.0;
    marker_wpt.lifetime = ros::Duration();

    for (uint32_t i = 0; i < waypoints.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = waypoints[i][0];
        p.y = -waypoints[i][1];
        p.z = 0;
        marker_wpt.points.push_back(p);
        p.z += 1.5;
        marker_wpt.points.push_back(p);
    }

    // define a marker to highlight the waypoint to which the ship is pointed
    marker_bulb.header.frame_id = "world";
    marker_bulb.header.stamp = ros::Time::now();
    marker_bulb.ns = "waypoints/viz";
    marker_bulb.action = visualization_msgs::Marker::ADD;
    marker_bulb.id = 1;
    marker_bulb.type = visualization_msgs::Marker::SPHERE;
    marker_bulb.scale.x = 0.2;
    marker_bulb.scale.y = 0.2;
    marker_bulb.scale.z = 0.2;
    marker_bulb.color.r = 1.0; // red
    marker_bulb.color.a = 1.0;
    marker_bulb.lifetime = ros::Duration();


    ros::spin();

    delete pubPtr_markers;
    delete pubPtr_markerArray;
    delete pubPtr_course;

    return 0;
}
