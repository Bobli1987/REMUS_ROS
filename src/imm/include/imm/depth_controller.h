#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include <vector>
#include <cmath>
#include <deque>

class DepthController
{
public:
    // constructor
    DepthController(const double &gamma, const double &kp, const double &taud):
        gamma_(gamma), kp_(kp), taud_(taud) {}

    // the member function used to compute actuations in heave and pitch
    std::vector<double> ComputeActuation(const double&, const double&, const double&, const double&);

private:
    // control parameters
    double gamma_, kp_, taud_;

    // deque used to store e_{theta}
    std::deque<double> e_theta_;

    // rudder angle
    double rudder_angle_ = 0;

    // elapsed time since current simulation starts
    double current_time_ = 0.0;
    // output function
    void OutputData(const std::string&, const std::string&);

};
// save control data into text files
void DepthController::OutputData(const std::string &mode,
                const std::string &control_file = "/home/bo/Documents/imm/dcontrol_file.txt")
{
    std::ofstream control_out;
    if (mode == "trunc") {
        control_out.open(control_file);
    } else {
        control_out.open(control_file, std::ofstream::app);
    }
    // velocity
    control_out << std::fixed << std::setprecision(2) << current_time_ << '\t';     // first column is time
    control_out << std::setprecision(2);
    control_out << std::scientific << rudder_angle_*180/M_PI << std::endl;

    // close the files
    control_out.close();
}

// define the member function
std::vector<double> DepthController::ComputeActuation(const double &z, const double &theta,
                                                      const double &zd, const double &step_size)
{
    double e_theta = gamma_*(zd- z) - theta;
    // store e_theta
    e_theta_.push_back(e_theta);

    if (e_theta_.size() > 4)
        e_theta_.pop_front();

    // compute the time derivative of e_theta
    double de_theta;

    switch (e_theta_.size()) {
    case 1:
        de_theta = 0;
        break;
    case 2:
        de_theta = (e_theta_.back() - e_theta_.front())/step_size;
        break;
    case 3:
        de_theta = (0.5*e_theta_.at(0) - 2*e_theta_.at(1) + 1.5*e_theta_.at(2))/step_size;
        break;
    case 4:
        de_theta = (-e_theta_.at(0)/3 + 1.5*e_theta_.at(1) - 3*e_theta_.at(2) + 11*e_theta_.at(3)/6)/step_size;
        break;
    default:
        break;
    }

    // compute the rudder angle
    double delta = -kp_*(e_theta + taud_*de_theta);
    // set the upper limit
    delta = (delta > 13 * M_PI/180) ? 13*M_PI/180 : delta;
    delta = (delta < -13 * M_PI/180) ? -13*M_PI/180 : delta;

    // return the actuation
    std::vector<double> actuation;
    actuation.push_back(-50.6*delta); // heave force
    actuation.push_back(-34.6*delta); // pitch moment

    // save the data
    rudder_angle_ = delta;
    current_time_ += step_size;
    DepthController::OutputData("app");

    return actuation;
}

#endif // DEPTH_CONTROLLER_H
