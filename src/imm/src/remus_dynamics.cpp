#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "remus.h"
#include "imm/euler.h"
#include "imm/control.h"

// the publishers and messages
ros::Publisher *pubPtr_vel;
ros::Publisher *pubPtr_rel_vel;
ros::Publisher *pubPtr_pos;
ros::Publisher *pubPtr_euler;
geometry_msgs::Twist msg_vel;
geometry_msgs::Twist msg_rel_vel;
geometry_msgs::Point msg_pos;
imm::euler msg_euler;

// the vehicle for the simulation
Remus vehicle = Remus({1.0, 0, 0, 0, 0 ,0}, {0, 0, 0, 0, 0, 0}, {0, 0.1, 0}, 4);

// the time step of the ode solver and message publishing
double step_size = 0.05;

void timerCallback(const ros::TimerEvent&)
{
    // publish the motion information of the vehicle
    ROS_INFO("Time: %5.2f s, Tunnel thrust: %5.2f N",
             vehicle.current_time_, vehicle.actuation_[1]);
    msg_vel.linear.x = vehicle.velocity_[0];
    msg_vel.linear.y = vehicle.velocity_[1];
    msg_vel.linear.z = vehicle.velocity_[2];
    msg_vel.angular.x = vehicle.velocity_[3];
    msg_vel.angular.y = vehicle.velocity_[4];
    msg_vel.angular.z = vehicle.velocity_[5];

    msg_rel_vel.linear.x = vehicle.relative_velocity_[0];
    msg_rel_vel.linear.y = vehicle.relative_velocity_[1];
    msg_rel_vel.linear.z = vehicle.relative_velocity_[2];
    msg_rel_vel.angular.x = vehicle.relative_velocity_[3];
    msg_rel_vel.angular.y = vehicle.relative_velocity_[4];
    msg_rel_vel.angular.z = vehicle.relative_velocity_[5];

    msg_pos.x = vehicle.position_[0];
    msg_pos.y = vehicle.position_[1];
    msg_pos.z = vehicle.position_[2];
    msg_euler.roll = vehicle.position_[3];
    msg_euler.pitch = vehicle.position_[4];
    msg_euler.yaw = vehicle.position_[5];

    pubPtr_vel->publish(msg_vel);
    pubPtr_rel_vel->publish(msg_rel_vel);
    pubPtr_pos->publish(msg_pos);
    pubPtr_euler->publish(msg_euler);

    // run a one-step simulation
    RunRemus(vehicle, 1, step_size);
}

void callback_sub(const imm::control &msg_ctrl)
{
    vehicle.actuation_[1] = msg_ctrl.thrust;
    vehicle.actuation_[3] = msg_ctrl.roll_torque;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "remus_dynamics");
    ros::NodeHandle nh;

    // create a timer to run a one-step simulation and publish messages
    ros::Timer timer = nh.createTimer(ros::Duration(step_size), &timerCallback);

    // the publishing rate is controlled by the timer
    pubPtr_vel = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("remus_vel", 1000));
    pubPtr_rel_vel = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("remus_relVel", 1000));
    pubPtr_pos = new ros::Publisher(nh.advertise<geometry_msgs::Point>("remus_pos", 1000));
    pubPtr_euler = new ros::Publisher(nh.advertise<imm::euler>("remus_euler", 1000));

    // create a subscriber to the control signal
    ros::Subscriber sub_ctrl = nh.subscribe("actuation", 1000, &callback_sub);

    ros::spin();

    delete pubPtr_vel;
    delete pubPtr_pos;
    delete pubPtr_euler;

    return 0;
}
