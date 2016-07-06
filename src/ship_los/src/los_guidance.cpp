#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace std;

// the waypoints
vector<vector<double>> waypoints;
visualization_msgs::Marker marker_wp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_guidance");
    ros::NodeHandle nh;
    ros::Publisher pub_wp = nh.advertise<visualization_msgs::Marker>("ship/waypoints", 1000);

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
    marker_wp.header.frame_id = "world";
    marker_wp.header.stamp = ros::Time::now();
    marker_wp.ns = "waypoints/viz";
    marker_wp.action = visualization_msgs::Marker::ADD;
    marker_wp.pose.orientation.w = 1.0;
    marker_wp.id = 0;
    marker_wp.type = visualization_msgs::Marker::LINE_LIST;
    marker_wp.scale.x = 0.1; // line width
    marker_wp.color.g = 1.0;
    marker_wp.color.a = 1.0;

    for (uint32_t i = 0; i < waypoints.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = waypoints[i][0];
        p.y = -waypoints[i][1];
        p.z = 0;
        marker_wp.points.push_back(p);
        p.z += 1.5;
        marker_wp.points.push_back(p);
    }

    ros::Rate r(10);

    while (ros::ok()) {
        pub_wp.publish(marker_wp);
        r.sleep();
    }

    return 0;
}
