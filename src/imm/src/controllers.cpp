#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include "controller.h"
#include "imm/pose.h"
#include "imm/control.h"
#include "remus.h"
#include "depth_controller.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Vector6d rvelocity;
Vector6d position;

// actuation computed by the controllers
std::vector<double> actuation_1;
std::vector<double> actuation_2;

// the time step of the controller
double step_size = 0.1;

// reference heading angle
//double heading_ref = std::atan(-0.1);
double heading_ref = 0;

// the publisher and message
ros::Publisher *pubPtr_control;
imm::control msg_actuation;

// the moving mass controller
Remus vehicle = Remus();
MovingMassController controller(vehicle, 0.3, 0.8, 1, 1, 1);

// the depth controller
DepthController depth_controller(-0.772, 10.345, 0.21);

void callback_pose(const imm::pose &msg_pose)
{
    position << 0, 0, msg_pose.z, msg_pose.roll, msg_pose.pitch, msg_pose.yaw;
}

void callback_rvel(const geometry_msgs::Twist &msg_rvel)
{
    rvelocity << 0, msg_rvel.linear.y, 0, msg_rvel.angular.x, 0, msg_rvel.angular.z;
}

void timerCallback(const ros::TimerEvent)
{
    // compute the actuation
    actuation_1 = controller.ComputeActuation(rvelocity, position, heading_ref, step_size);
    actuation_2 = depth_controller.ComputeActuation(position[2], position[4], 1, step_size);

    // publish the control signal
    msg_actuation.mass_position = actuation_1[0];
    msg_actuation.thrust = actuation_1[1];
    msg_actuation.roll_torque = actuation_1[2];
    msg_actuation.heave_force = actuation_2[0];
    msg_actuation.pitch_torque = actuation_2[1];

    ROS_INFO("Mass position: %5.2f cm, Tunnel thrust: %5.2f N", actuation_1[0]*100, actuation_1[1]);
    pubPtr_control->publish(msg_actuation);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;

    // create a timer to control the publising
    ros::Timer timer = nh.createTimer(ros::Duration(step_size), &timerCallback);

    // create a publisher and two subscribers
    pubPtr_control = new ros::Publisher(nh.advertise<imm::control>("actuation", 1000));
    ros::Subscriber sub_pose = nh.subscribe("remus_pose", 1000, &callback_pose);
    ros::Subscriber sub_relVel = nh.subscribe("remus_relVel", 1000, &callback_rvel);

    ros::spin();

    delete pubPtr_control;

    return 0;
}
