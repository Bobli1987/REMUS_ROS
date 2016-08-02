//
// Created by lither on 4/1/2016.
//

#ifndef REMUSIMM_CURRENT_H
#define REMUSIMM_CURRENT_H

#include <iostream>
#include <string>
#include <Eigen/Dense>


// Class OceanCurrent
class OceanCurrent
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

public:
    // constructor
    OceanCurrent(const std::vector<double> &uniform_velocity): uniform_velocity_(uniform_velocity) {}
    OceanCurrent(): OceanCurrent({0, 0, 0}) {}
    // return the uniform velocity of the current
    Vector6d GetCurrentVelocity() const {
        Vector6d velocity;
        velocity << uniform_velocity_[0], uniform_velocity_[1], uniform_velocity_[2], 0, 0, 0;
        return velocity;
    }

private:
    // ocean current default velocity
    std::vector<double> uniform_velocity_;
};

#endif //REMUSIMM_CURRENT_H
