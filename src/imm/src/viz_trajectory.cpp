#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

ros::Publisher *pubPtr_trajectory;
visualization_msgs::Marker trajectory;
geometry_msgs::Point p;

void callback_pos(const geometry_msgs::Point &msg_pos)
{
    p.x = msg_pos.x;
    p.y = msg_pos.y;
    p.z = msg_pos.z;
    trajectory.points.push_back(p);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viz_trajectory");
    ros::NodeHandle nh;
    pubPtr_trajectory = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("remus_trajectory", 1000));
    ros::Subscriber sub_pos = nh.subscribe("remus_pos", 1000, &callback_pos);

    // define the marker
    trajectory.header.frame_id = "world";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "remus_viz";
    trajectory.action = visualization_msgs::Marker::ADD;
    trajectory.pose.orientation.w = 1.0;
    trajectory.id = 0;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.scale.x = 0.1; // line width
    trajectory.color.r = 1.0;
    trajectory.color.a = 1.0;

    ros::Rate r(5);

    while (ros::ok()) {
        pubPtr_trajectory->publish(trajectory);
        ros::spinOnce();
        r.sleep();
    }

    delete pubPtr_trajectory;

    return 0;
}
