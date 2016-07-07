#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ship_los/pose.h>
#include <ship_los/course.h>
#include <cmath>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;

// the waypoints
vector<vector<double>> waypoints;

// the publishers
ros::Publisher *pubPtr_course;
ros::Publisher *pub_markers;

// the messages
visualization_msgs::Marker marker_wpt;
visualization_msgs::Marker marker_bulb;
ship_los::course msg_course;


// the number of the waypoint to which the ship should be pointed
uint32_t wpt_num = 1;

// desired course angle
double course_angle;

// the reference model for trajectory generation
const double omega = 0.2;
const double zeta = 0.1;
void reference_model(const vector<double> &x, vector<double> &dxdt, const double /* t */)
{
    dxdt[0] = x[1];
    dxdt[1] = x[2];
    dxdt[2] = -pow(omega,3)*x[0] - (2*zeta+1)*pow(omega,2)*x[1] - (2*zeta+1)*omega*x[2] + \
            pow(omega,3)*course_angle;
}

// callback of the timer used to publish the markers to rviz
void timerCallback(const ros::TimerEvent&)
{
    marker_bulb.pose.position.x = waypoints[wpt_num][0];
    marker_bulb.pose.position.y = -waypoints[wpt_num][1];
    marker_bulb.pose.position.z = 1.5;

    pub_markers->publish(marker_wpt);
    pub_markers->publish(marker_bulb);
}

// callback of the subscriber, where desired course is computed and published
void callback_pos(const ship_los::pose &msg_pos)
{
    double distance = sqrt(pow(msg_pos.x - waypoints[wpt_num+1][0], 2) +
                           pow(msg_pos.y - waypoints[wpt_num+1][1], 2));
    ROS_INFO("Current: wpt%u, Distance to wpt%u: %5.2f m", wpt_num, wpt_num+1, distance);

    // change the waypoint
    if ( distance <= 2.5 )
    {
        ++ wpt_num;
        ROS_INFO("Change the waypoint to: wpt%u", wpt_num);
    }

    // compute the desired course angle using the current waypoint
    course_angle = atan2(waypoints[wpt_num][1] - msg_pos.y, waypoints[wpt_num][0] - msg_pos.x);
    // low-pass filter the course angle, and obtain rate and acceleration
    static vector<double> x = {0, 0, 0};
    // the end time of the integration should be the publising rate of ship/pose
    size_t steps = integrate(reference_model, x , 0.0 , 0.05 , 0.01);
    // publish the desired course message
    msg_course.angle = x[0];
    msg_course.rate = x[1];
    msg_course.acceleration = x[2];
    pubPtr_course->publish(msg_course);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_guidance");
    ros::NodeHandle nh;

    // create a timer to publish the markers to rviz
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), &timerCallback);
    pub_markers = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("ship/waypoints", 1000));
    pubPtr_course = new ros::Publisher(nh.advertise<ship_los::course>("ship/course", 1000));

    ros::Subscriber sub_pos = nh.subscribe("ship/pose", 1000, &callback_pos);

    // define the waypoints
    waypoints = { {0.372, -1.50},
                  {-0.628, 0.00},
                  {0.372, 1.50},
                  {1.872, 2.00},
                  {6.872, -2.00},
                  {8.372, -1.50},
                  {9.372, 0.00},
                  {8.372, 1.50} };

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

    // define a marker to highlight the current waypoint
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
    delete pubPtr_course;

    return 0;
}
