#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include "ship.h"
#include "heading_controller.h"
#include "ship_los/course.h"
#include "ship_los/control.h"
#include "ship_los/pose.h"

// actuation computed by the controller
std::vector<double> actuation;

// the time step of the controller
double step_size = 0.1;

// variables used for the controller
double u, v, r, r_d, dr_d, psi, psi_d;

// the publisher and message
ros::Publisher *pubPtr_control;
ship_los::control msg_actuation;

// the path controller
Ship vehicle = Ship();
HeadingController controller = HeadingController(vehicle, 0.75, 25, 10, 2.5);

void callback_pose(const ship_los::pose &msg_pose)
{
    psi = msg_pose.heading;
}

void callback_vel(const geometry_msgs::Twist &msg_vel)
{
    u = msg_vel.linear.x;
    v = msg_vel.linear.y;
    r = msg_vel.angular.z;
}

void callback_course(const ship_los::course &msg_course)
{
    r_d = msg_course.rate;
    dr_d = msg_course.acceleration;
    psi_d = msg_course.angle;
}

void timerCallback(const ros::TimerEvent)
{
    // compute the actuation
    actuation = ComputeActuation(controller, u, v, r, psi, psi_d, r_d, dr_d, step_size);

    // publish the control signal
    msg_actuation.surge = actuation[0];
    msg_actuation.sway = actuation[1];
    msg_actuation.yaw = actuation[2];
    msg_actuation.z1 = controller.z1_;
    msg_actuation.z21 = controller.z21_;
    msg_actuation.z22 = controller.z22_;
    msg_actuation.z23 = controller.z23_;

    pubPtr_control->publish(msg_actuation);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_controller");
    ros::NodeHandle nh;

    // create a timer to control the publishing of control commands
    ros::Timer timer = nh.createTimer(ros::Duration(step_size), &timerCallback);

    // create a publisher and two subscribers
    pubPtr_control = new ros::Publisher(nh.advertise<ship_los::control>("ship/actuation", 1000));
    ros::Subscriber sub_pose = nh.subscribe("ship/pose", 1000, &callback_pose);
    ros::Subscriber sub_vel = nh.subscribe("ship/vel", 1000, &callback_vel);
    ros::Subscriber sub_course = nh.subscribe("ship/course", 1000, &callback_course);

    ros::spin();

    delete pubPtr_control;

    return 0;
}
