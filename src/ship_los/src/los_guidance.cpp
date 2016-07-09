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
ros::Publisher *pub_markers;
ros::Publisher *pub_markerArray;

// the messages
visualization_msgs::Marker marker_wpt;
visualization_msgs::Marker marker_bulb;
visualization_msgs::MarkerArray marker_textArray;
ship_los::course msg_course;

// the number of the waypoint which the ship just passed
uint32_t wpt_num = 0;
auto iter = ++waypoints.begin();

// desired course angle
double course_angle;

// the reference model for trajectory generation
const double omega = 0.4;
const double zeta = 1.0;
void reference_model(const vector<double> &x, vector<double> &dxdt, const double /* t */)
{
    dxdt[0] = x[1];
    dxdt[1] = x[2];
    dxdt[2] = -pow(omega,3)*x[0] - (2*zeta+1)*pow(omega,2)*x[1] - (2*zeta+1)*omega*x[2] + \
            pow(omega,3)*course_angle;
}

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
    // the radius of the circle based on enclosure-based steering
    double radius = 1.0;

    // the intersection point
    vector<double> los_point(2, 0);

    // filtered course infomation
    static vector<double> course_state = {1.78, 0, 0};

    if (wpt_num < waypoints.size()-1)
    {
        // intersection feasibility
        bool feasible = true;

        // solve the intersection point
        double dx = waypoints[wpt_num+1][0] - waypoints[wpt_num][0];
        double dy = waypoints[wpt_num+1][1] - waypoints[wpt_num][1];
        if (fabs(dx) > numeric_limits<double>::epsilon())
        {
            double x = msg_pos.x;
            double y = msg_pos.y;
            double d = dy/dx;
            double e = waypoints[wpt_num][0];
            double f = waypoints[wpt_num][1];
            double g = f - d*e;
            double a = 1 + pow(d,2);
            double b = 2*(d*g - d*y - x);
            double c = pow(x,2) + pow(y,2) + pow(g,2) - 2*g*y - pow(radius,2);

            if (pow(b,2) - 4*a*c >= 0) {
                if (dx > 0) {
                    los_point[0] = (-b + sqrt(pow(b,2) - 4*a*c))/(2*a);
                }
                if (dx < 0) {
                    los_point[0] = (-b - sqrt(pow(b,2) - 4*a*c))/(2*a);
                }
                los_point[1] = d*(los_point[0] - waypoints[wpt_num][0]) + waypoints[wpt_num][1];
            }
            else
            {                
                feasible = false;
            }
        }
        else
        {
            if (pow(radius,2) - pow((waypoints[wpt_num+1][0] - msg_pos.x),2) >=0 )
            {
                los_point[0] = waypoints[wpt_num+1][0];
                if (dy > 0) {
                    los_point[1] = msg_pos.y + sqrt(pow(radius,2) - pow((los_point[0] - msg_pos.x),2));
                }
                if (dy < 0) {
                    los_point[1] = msg_pos.y - sqrt(pow(radius,2) - pow((los_point[0] - msg_pos.x),2));
                }
            }
            else
            {
                feasible = false;
            }
        }

        // compute the desired course angle using the intersection point
        if (feasible)
        {
            course_angle = atan2(los_point[1] - msg_pos.y, los_point[0] - msg_pos.x);

            if (fabs(course_angle - msg_pos.heading) > M_PI)
            {
                course_state[0] = course_angle;
                course_state[1] = 0;
                course_state[2] = 0;

                ROS_ERROR_STREAM("This is the probelm I haven't solved perfectly");

                course_angle += boost::math::sign(msg_pos.heading)*2*M_PI;
                msg_course.angle = msg_pos.heading + (course_angle - msg_pos.heading) * 0.2;
                msg_course.rate = 0;
                msg_course.acceleration = 0;
            }
            else
            {
                // low-pass filter the obtained course angle, yielding rate and acceleration
                // the end time of the integration should be the publising rate of ship/pose
                size_t steps = integrate(reference_model, course_state, 0.0, 0.05, 0.01);

                // publish the desired course message
                msg_course.angle = course_state[0];
                msg_course.rate = course_state[1];
                msg_course.acceleration = course_state[2];
            }

            pubPtr_course->publish(msg_course);
        }
        else
        {
            // no course message published
            ROS_WARN_STREAM("There are no intersections");
        }


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
    // define the waypoint banners
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

    pub_markers->publish(marker_wpt);
    pub_markers->publish(marker_bulb);
    pub_markerArray->publish(marker_textArray);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_guidance");
    ros::NodeHandle nh;

    pub_markers = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("ship/waypoints", 1000));
    pub_markerArray = new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("ship/waypoint_banner", 1000));
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

    delete pub_markers;
    delete pub_markerArray;
    delete pubPtr_course;

    return 0;
}
