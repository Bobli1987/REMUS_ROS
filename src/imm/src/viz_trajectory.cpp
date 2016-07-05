#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <imm/pose.h>

ros::Publisher *pubPtr_trajectory;
visualization_msgs::Marker trajectory;
geometry_msgs::Point p;

void callback_pos(const imm::pose &msg_pos)
{
    // update the points of the line_strip
    p.x = msg_pos.x;
    p.y = -msg_pos.y; // add the minus sign because of the transform from world to the earth frame
    p.z = -msg_pos.z;
    trajectory.points.push_back(p);

    tf::Transform transform;
    tf::Quaternion q;

    // define the Earth-fixed frame
    static tf::TransformBroadcaster br_ef;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(M_PI, 0, 0);
    transform.setRotation(q);
    br_ef.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Earth-fixed frame"));

    // define the body frame
    static tf::TransformBroadcaster br_bf;
    transform.setOrigin(tf::Vector3(msg_pos.x, msg_pos.y, msg_pos.z));
    q.setRPY(msg_pos.roll, msg_pos.pitch, msg_pos.yaw);
    transform.setRotation(q);
    br_bf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Earth-fixed frame", "REMUS"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "viz_trajectory");
    ros::NodeHandle nh;
    pubPtr_trajectory = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("remus_trajectory", 1000));
    ros::Subscriber sub_pos = nh.subscribe("remus_pose", 1000, &callback_pos);



    // define the marker as the trajectory
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

    // control the publishing rate
    ros::Rate r(5);

    while (ros::ok()) {
        pubPtr_trajectory->publish(trajectory);
        ros::spinOnce();
        r.sleep();
    }

    delete pubPtr_trajectory;

    return 0;
}
