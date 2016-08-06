/*
 * Created by lither on 3/28/2016.
 *
 * Class Remus is defined in the header file.
 *
 */

#ifndef REMUSIMM_REMUS_H
#define REMUSIMM_REMUS_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include "current.h"

// Class Remus
class Remus
{
    friend class MovingMassController;
    friend void RunRemus(Remus&, const size_t&, const double&);

    // new matrix types
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

public:
    // constructors
    Remus(const std::vector<double> &init_velocity, const std::vector<double> &init_position,
          const std::vector<double> &current_velocity, const double &main_thrust, const bool &control_on);
    Remus(): Remus({1.5, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0}, 9, true) {}
    Remus(const std::vector<double> &init_velocity): Remus(init_velocity, {0, 0, 0, 0, 0, 0}, {0, 0, 0}, 9, true) {}
    Remus(const std::vector<double> &init_velocity, const std::vector<double> &init_position,
          const std::vector<double> &current_velocity, const double &main_thrust):
            Remus(init_velocity, init_position, current_velocity, main_thrust, true) {}

private:
    // parameters of the vehicle
    const double gravity_ = 9.81;
    const double mass_ = 3.05e1, displaced_mass_ = 3.12e1;
    // center of gravity
    const Eigen::Vector3d cog_ = {0.0, 0.0, 1.96e-2};
    // center of buoyancy
    const Eigen::Vector3d cob_ = {0.0, 0.0, 0.0};
    // moment of inertia
    const double Ixx_ = 1.77e-1, Iyy_ = 3.45, Izz_ = 3.45;
    const double Ixy_ = 0.0, Ixz_ = 0.0, Iyz_ = 0.0;
    // added mass
    const double a11_ = 9.30e-1, a22_ = 3.55e1, a33_ = 3.55e1;
    const double a44_ = 7.04e-2, a55_ = 4.88, a66_ = 4.88;
    const double a24_ = 0.0, a26_ = -1.93, a35_ = 1.93, a46_ = 0.0;
    // viscous damping coefficients
    const double Xuu_ = -3.87, Yvv_ = -1e2, Yrr_ = 6.32e-1;
    const double Zww_ = -1e2, Zqq_ = -6.32e-1;
    const double Kp_ = -1.60e-3, Kpp_ = -1.30e-1;
    const double Mww_ = -7.38, Mqq_ = -1.88e2, Nvv_ = 7.38, Nrr_ = -9.40e1;
    const double Yuv_ = -1.79e1, Yur_ = 8.62, Zuw_ = -1.79e1, Zuq_ = -8.62;
    const double Kup_ = 0, Muw_ = -4.57, Muq_ = 6.4, Nuv_ = 4.57, Nur_ = -6.4;

    // parameters of the simulation
    // step size of the current simulation
    double step_size_ = 0.0;
    // step number of the current simulation
    size_t step_number_ = 0;
    // counter of steps
    size_t step_counter_ = 0;

public:
    // ocean current model
    OceanCurrent ocean_current_;
    // controller switch
    bool control_on_ = true;

    // elapsed time since current simulation starts
    double current_time_ = 0.0;
    // initial velocity and position(and orientation) of the vehicle
    std::vector<double> init_velocity_, init_position_;
    // current velocity and position of the vehicle
    Vector6d velocity_, position_;
    // current euler angle of the vehicle
    Eigen::Vector3d euler_;
    // current quaternion of the vehicle
    Eigen::Vector4d quaternion_;
    // relative velocity with respect to the ocean current
    Vector6d relative_velocity_;
    // time vector of the simulation
    std::vector<double> time_vec_ = {0};
    // array of velocity and position vectors during the simulation
    std::vector<Vector6d> velocity_history_, position_history_, relative_velocity_history_;
    // actuation acting on the vehicle
    Vector6d actuation_;

    // 3X3 matrix which maps a vector to the Lie algebra of SO(3)
    inline Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d &vec) const;
    // rotation matrix declaration
    static Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d &angle);
    // transformation from Euler angle representation to quaternion
    static Eigen::Vector4d Euler2q(const Eigen::Vector3d &angle);
    // angular transformation matrix declaration
    static Eigen::Matrix3d AngularTransMatrix(const Eigen::Vector3d &angle);
    // transformation matrix from the body-fixed frame to the earth-fixed frame
    static Matrix6d TransformationMatrix(const Eigen::Vector3d &angle);

    // inertia moment of the rigid body
    Eigen::Matrix3d InertiaMoment() const;
    // inertia matrix of the rigid body
    Matrix6d RigidBodyInertiaMatrix() const;
    // centripetal-coriolis matrix of the rigid body
    Matrix6d RigidBodyCCMatrix(const Vector6d &vec) const;
    // added mass matrix
    Matrix6d AddedMassMatrix() const;
    // centripetal-coriolis matrix of added mass
    Matrix6d AddedMassCCMatrix(const Vector6d &vec) const;
    // inertia moment of the displaced fluid
    Eigen::Matrix3d DisplacedMassInertiaMoment() const;
    // inertia matrix of the displaced fluid
    Matrix6d DisplacedMassInertiaMatrix() const;
    // centripetal-coriolis matrix of the displaced fluid
    Matrix6d DisplacedMassCCMatrix(const Vector6d &vec) const;

    // viscous hydrodynamic damping force
    Vector6d ViscousDampingVector(const Vector6d &vec) const;
    // restoring force due to gravity and buoyancy
    Vector6d RestoringForceVector(const Vector6d &vec) const;

    // time derivative of the system state
    Vector12d StateDerivative(const Vector6d &velocity, const Vector6d &position, const double /* t */) const;

    // operator function to integrate
    void operator() (const std::vector<double> &state, std::vector<double> &state_derivative, const double);

    // The ocean current velocity in the body-fixed reference frame
    Vector6d CurrentVelocity(const Vector6d&) const;
    // The ocean current acceleration in the body-fixed reference frame
    Vector6d CurrentAcceleration(const Vector6d&, const Vector6d&) const;
};
// constructor definition
Remus::Remus(const std::vector<double> &init_velocity, const std::vector<double> &init_position,
             const std::vector<double> &current_velocity, const double &main_thrust, const bool &control_on):
        init_velocity_(init_velocity), init_position_(init_position) {
    velocity_ << init_velocity[0], init_velocity[1], init_velocity[2], init_velocity[3], init_velocity[4], init_velocity[5];
    position_ << init_position[0], init_position[1], init_position[2], init_position[3], init_position[4], init_position[5];
    euler_ << init_position[3], init_position[4], init_position[5];
    quaternion_ = Euler2q(euler_);
    ocean_current_ = OceanCurrent(current_velocity);
    relative_velocity_ = velocity_ - CurrentVelocity(position_);
    velocity_history_ = {velocity_};
    position_history_ = {position_};
    relative_velocity_history_ = {relative_velocity_};
    actuation_ << main_thrust, 0, 0, 0, 0, 0;
    control_on_ = control_on;
}

// The ocean current velocity with respect to the body-fixed frame
Remus::Vector6d Remus::CurrentVelocity(const Vector6d &position) const {
    Eigen::Vector3d angle;
    angle << position[3], position[4], position[5];
    return TransformationMatrix(angle).inverse() * ocean_current_.GetCurrentVelocity();
}

// The ocean current acceleration with respect to the body-fixed frame
Remus::Vector6d Remus::CurrentAcceleration(const Vector6d &velocity, const Vector6d &position) const {
    Vector6d current_acceleration;
    Eigen::Vector3d angular_velocity;
    angular_velocity << velocity[3], velocity[4], velocity[5];
    current_acceleration << - CrossProductOperator(angular_velocity) * CurrentVelocity(position).head(3), Eigen::Vector3d::Zero();
    return current_acceleration;
}

// cross product operator
Eigen::Matrix3d Remus::CrossProductOperator(const Eigen::Vector3d &vec) const {
    Eigen::Matrix3d cross_product;
    cross_product << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
    return cross_product;
}

// rotation matrix definition
Eigen::Matrix3d Remus::RotationMatrix(const Eigen::Vector3d &angle) {
    Eigen::Matrix3d rz, ry, rx;
    rz << cos(angle[2]), -sin(angle[2]), 0, sin(angle[2]), cos(angle[2]), 0, 0, 0, 1;
    ry << cos(angle[1]), 0, sin(angle[1]), 0, 1, 0, -sin(angle[1]), 0, cos(angle[1]);
    rx << 1, 0, 0, 0, cos(angle[0]), -sin(angle[0]), 0, sin(angle[0]), cos(angle[0]);
    return rz * ry * rx;
}

// transformation from rotation matrix to an unit quaternion
Eigen::Vector4d Remus::Euler2q(const Eigen::Vector3d &angle) {
    Eigen::Vector4d q;
    float p1, p2, p3, p4;
    Eigen::Matrix3d R = RotationMatrix(angle);
    float trace = R.trace();
    q << R(0,0), R(1,1), R(2,2), trace;
    Eigen::Vector4d::Index maxRow, maxCol;
    float max = q.maxCoeff(&maxRow, &maxCol);
    float p_i = std::sqrt(1 + 2*max - trace);

    switch (maxRow) {
    case 0:
        p1 = p_i;
        p2 = (R(1,0) + R(0,1))/p_i;
        p3 = (R(0,2) + R(2,0))/p_i;
        p4 = (R(2,1) - R(1,2))/p_i;
        break;
    case 1:
        p1 = (R(1,0) + R(0,1))/p_i;
        p2 = p_i;
        p3 = (R(2,1) + R(1,2))/p_i;
        p4 = (R(0,2) - R(2,0))/p_i;
        break;
    case 2:
        p1 = (R(0,2) + R(2,0))/p_i;
        p2 = (R(2,1) + R(1,2))/p_i;
        p3 = p_i;
        p4 = (R(1,0) - R(0,1))/p_i;
        break;
    case 3:
        p1 = (R(2,1) - R(1,2))/p_i;
        p2 = (R(0,2) - R(2,0))/p_i;
        p3 = (R(1,0) - R(0,1))/p_i;
        p4 = p_i;
        break;
    }
    q << 0.5*p1, 0.5*p2, 0.5*p3, 0.5*p4;
    q = q/q.dot(q);
    return q;
}

// angular transformation matrix definition
Eigen::Matrix3d Remus::AngularTransMatrix(const Eigen::Vector3d &angle) {
    if (std::fabs(std::fabs(angle[1]) - M_PI/2) <= M_PI/6)
        std::cerr << "Warning: the pitch motion is almost singular!" << std::endl;
    Eigen::Matrix3d ang_trans;
    ang_trans << 1, sin(angle[0])*tan(angle[1]), cos(angle[0])*tan(angle[1]),
                 0, cos(angle[0]), -sin(angle[0]),
                 0, sin(angle[0])/cos(angle[1]), cos(angle[0])/cos((angle[1]));
    return ang_trans;
}

// transformation matrix
Remus::Matrix6d Remus::TransformationMatrix(const Eigen::Vector3d &angle) {
    Matrix6d trans_mat;
    Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
    trans_mat << RotationMatrix(angle), zeros, zeros, AngularTransMatrix(angle);
    return trans_mat;
}

// inertia moment of the rigid body
Eigen::Matrix3d Remus::InertiaMoment() const {
    Eigen::Matrix3d inertia_mat;
    inertia_mat << Ixx_, -Ixy_, -Ixz_, -Ixy_, Iyy_, -Iyz_, -Ixz_, -Iyz_, Izz_;
    return inertia_mat;
}

// inertia matrix of the rigid body
Remus::Matrix6d Remus::RigidBodyInertiaMatrix() const {
    Matrix6d inertia_mat;
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    inertia_mat << mass_ * identity, -mass_ * CrossProductOperator(cog_),
                   mass_ * CrossProductOperator(cog_), InertiaMoment();
    return inertia_mat;
}

// centripetal-coriolis matrix of rigid body
Remus::Matrix6d Remus::RigidBodyCCMatrix(const Vector6d &vec) const {
    Eigen::Vector3d angular_vel = {vec[3], vec[4], vec[5]};
    Matrix6d cc_mat;
    cc_mat << mass_ * CrossProductOperator(angular_vel),
             -mass_ * CrossProductOperator(angular_vel) * CrossProductOperator(cog_),
              mass_ * CrossProductOperator(cog_) * CrossProductOperator(angular_vel),
             -CrossProductOperator(InertiaMoment() * angular_vel);
    return cc_mat;
}

// added mass matrix
Remus::Matrix6d Remus::AddedMassMatrix() const {
    Matrix6d added_mass;
    added_mass << a11_, 0, 0, 0, 0, 0,
                   0, a22_, 0, a24_, 0, a26_,
                   0, 0, a33_, 0, a35_, 0,
                   0, a24_, 0, a44_, 0, a46_,
                   0, 0, a35_, 0, a55_, 0,
                   0, a26_, 0, a46_, 0, a66_;
    return added_mass;
}

// centripetal-coriolis matrix of added mass
Remus::Matrix6d Remus::AddedMassCCMatrix(const Vector6d &vec) const {
    Matrix6d temp;
    Eigen::Vector3d lin_vel = {vec[0], vec[1], vec[2]};
    Eigen::Vector3d ang_vel = {vec[3], vec[4], vec[5]};
    temp << CrossProductOperator(ang_vel), Eigen::Matrix3d::Zero(),
            CrossProductOperator(lin_vel), CrossProductOperator(ang_vel);
    return temp * AddedMassMatrix();
}

// inertia moment of the displaced fluid
Eigen::Matrix3d Remus::DisplacedMassInertiaMoment() const {
    return InertiaMoment();
}

// inertia matrix of the displaced fluid
Remus::Matrix6d Remus::DisplacedMassInertiaMatrix() const {
    Matrix6d inertia_mat;
    inertia_mat << displaced_mass_ * Eigen::Matrix3d::Identity(), -displaced_mass_ * CrossProductOperator(cob_),
                   displaced_mass_ * CrossProductOperator(cob_), DisplacedMassInertiaMoment();
    return inertia_mat;
}

// centripetal-coriolis matrix of the displaced fluid
Remus::Matrix6d Remus::DisplacedMassCCMatrix(const Vector6d &vec) const {
    Eigen::Vector3d angular_vel = {vec[3], vec[4], vec[5]};
    Matrix6d cc_mat;
    cc_mat << displaced_mass_ * CrossProductOperator(angular_vel),
             -displaced_mass_ * CrossProductOperator(angular_vel) * CrossProductOperator(cob_),
              displaced_mass_ * CrossProductOperator(cob_) * CrossProductOperator(angular_vel),
             -CrossProductOperator(DisplacedMassInertiaMoment() * angular_vel);
    return cc_mat;
}

// viscous hydrodynamic damping vector
Remus::Vector6d Remus::ViscousDampingVector(const Vector6d &vec) const {
    Vector6d damping1, damping2;
    damping1 << Xuu_ * std::fabs(vec[0])*vec[0],
                Yvv_ * std::fabs(vec[1])*vec[1] + Yrr_ * std::fabs(vec[5])*vec[5],
                Zww_ * std::fabs(vec[2])*vec[2] + Zqq_ * std::fabs(vec[4])*vec[4],
                Kp_ * vec[3] + Kpp_ * std::fabs(vec[3])*vec[3],
                Mww_ * std::fabs(vec[2])*vec[2] + Mqq_ * std::fabs(vec[4])*vec[4],
                Nvv_ * std::fabs(vec[1])*vec[1] + Nrr_ * std::fabs(vec[5])*vec[5];
    damping2 << 0,
                Yuv_ * vec[0] * vec[1] + Yur_ * vec[0] * vec[5],
                Zuw_ * vec[0] * vec[2] + Zuq_ * vec[0] * vec[4],
                Kup_ * vec[0] * vec[3],
                Muw_ * vec[0] * vec[2] + Muq_ * vec[0] * vec[4],
                Nuv_ * vec[0] * vec[1] + Nur_ * vec[0] * vec[5];
    return damping1 + damping2;
}

// restoring force vector
Remus::Vector6d Remus::RestoringForceVector(const Vector6d &vec) const {
    Vector6d restoring;
    Eigen::Vector3d angle = {vec[3], vec[4], vec[5]};
    Eigen::Vector3d vg = {0, 0, mass_*gravity_};
    Eigen::Vector3d vb = {0, 0, displaced_mass_*gravity_};
    restoring << RotationMatrix(angle).transpose() * (vg - vb),
                 CrossProductOperator(cog_) * RotationMatrix(angle).transpose() * vg \
                -CrossProductOperator(cob_) * RotationMatrix(angle).transpose() * vb;
    return restoring;
}

//// time derivative of the system state
//Remus::Vector12d Remus::StateDerivative(const Vector6d &velocity, const Vector6d &position, const double) const {
//    Vector12d state_derivative;
//    Vector6d vec1, vec2;
//    Eigen::Vector3d angle;
//    angle << position[3], position[4], position[5];
//    // ignore some dofs
//    Vector6d diag1;
//    diag1 << 1, 1, 0, 1, 0, 1; // ignore heave and pitch
//    Vector6d current_velocity = CurrentVelocity(position);
//    Vector6d relative_velocity = diag1.asDiagonal() * (velocity - current_velocity);
//    Matrix6d total_mass_matrix = RigidBodyInertiaMatrix() + AddedMassMatrix();
//    vec1 = total_mass_matrix.inverse() *
//           ((DisplacedMassInertiaMatrix() + AddedMassMatrix()) * CurrentAcceleration(velocity, position)
//            + DisplacedMassCCMatrix(velocity) * current_velocity
//            - AddedMassCCMatrix(relative_velocity) * relative_velocity
//            - RigidBodyCCMatrix(velocity) * velocity
//            + ViscousDampingVector(relative_velocity)
//            + RestoringForceVector(position)
//            + actuation_);
//    vec2 = TransformationMatrix(angle) * velocity;
//    state_derivative << vec1, vec2;
//    // ignore some dofs
//    Vector12d diag2;
//    diag2 << 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1; // ignore heave and pitch
//    return diag2.asDiagonal() * state_derivative;
//}

// time derivative of the system state
Remus::Vector12d Remus::StateDerivative(const Vector6d &velocity, const Vector6d &position, const double) const {
    Vector12d state_derivative;
    Vector6d vec1, vec2;
    Eigen::Vector3d angle(position[3], position[4], position[5]);

    Vector6d current_velocity = CurrentVelocity(position);
    Vector6d relative_velocity = velocity - current_velocity;
    Matrix6d total_mass_matrix = RigidBodyInertiaMatrix() + AddedMassMatrix();

    vec1 = total_mass_matrix.inverse() *
           ((DisplacedMassInertiaMatrix() + AddedMassMatrix()) * CurrentAcceleration(velocity, position)
            + DisplacedMassCCMatrix(velocity) * current_velocity
            - RigidBodyCCMatrix(velocity) * velocity
            - AddedMassCCMatrix(relative_velocity) * relative_velocity
            + ViscousDampingVector(relative_velocity)
            + RestoringForceVector(position)
            + actuation_);
    vec2 = TransformationMatrix(angle) * velocity;
    state_derivative << vec1, vec2;

    return state_derivative;
}

// class operator to integrate
void Remus::operator()(const std::vector<double> &state, std::vector<double> &state_derivative, const double t) {
    Vector6d velocity, position;
    Vector12d dy;
    velocity << state[0], state[1], state[2],
                state[3], state[4], state[5];
    position << state[6], state[7], state[8],
                state[9], state[10], state[11];
    dy = StateDerivative(velocity, position, t);
    for (size_t index = 0; index < 12; ++index) {
        state_derivative[index] = dy[index];
    }
}
// save velocity and position data into text files
void OutputData(const Remus &vehicle, const std::string &mode = "trunc",
                const std::string &velocity_file = "/home/bo/Documents/imm/velocity_file.txt",
                const std::string &position_file = "/home/bo/Documents/imm/position_file.txt",
                const std::string &rvelocity_file = "/home/bo/Documents/imm/relative_vel_file.txt",
                const std::string &control_file = "/home/bo/Documents/imm/actuation_file.txt")
{
    std::ofstream velocity_out, position_out, rvelocity_out, control_out;
    // open the files
    if (mode == "trunc") {
        velocity_out.open(velocity_file);
        position_out.open(position_file);
        rvelocity_out.open(rvelocity_file);
        control_out.open(control_file);
    } else {
        velocity_out.open(velocity_file, std::ofstream::app);
        position_out.open(position_file, std::ofstream::app);
        rvelocity_out.open(rvelocity_file, std::ofstream::app);
        control_out.open(control_file, std::ofstream::app);
    }
    // velocity
    velocity_out << std::fixed << std::setprecision(2) << vehicle.current_time_ << '\t';     // first column is time
    velocity_out << std::setprecision(5);
    for (size_t index = 0; index < 6; ++index) {      // the remaining columns are data
        velocity_out << std::scientific << vehicle.velocity_[index] << '\t';
    }
    velocity_out << std::endl;

    // position
    position_out << std::fixed << std::setprecision(2) << vehicle.current_time_ << '\t';
    position_out << std::setprecision(5);
    for (size_t index = 0; index < 6; ++index) {
        position_out << std::scientific << vehicle.position_[index] << '\t';
    }
    position_out << std::endl;

    // relative velocity
    rvelocity_out << std::fixed << std::setprecision(2) << vehicle.current_time_ << '\t';
    rvelocity_out << std::setprecision(5);
    for (size_t index = 0; index < 6; ++index) {
        rvelocity_out << std::scientific << vehicle.relative_velocity_[index] << '\t';
    }
    rvelocity_out << std::endl;

    // actuation
    control_out << std::fixed << std::setprecision(2) << vehicle.current_time_ << '\t';
    control_out << std::setprecision(5);
    for (size_t index = 0; index < 6; ++index) {
        control_out << std::scientific << vehicle.actuation_[index] << '\t';
    }
    control_out << std::endl;

    // close the files
    velocity_out.close();
    position_out.close();
    rvelocity_out.close();
    control_out.close();
}

// conduct a simulation of the vehicle's motion
void RunRemus(Remus &vehicle, const size_t &step_number = 600, const double &step_size = 0.1) {
    vehicle.step_number_ = step_number;
    vehicle.step_size_ = step_size;
    // ode solver
    boost::numeric::odeint::runge_kutta_dopri5 <std::vector<double>> stepper;

    std::vector<double> state;

    for (size_t index = 0; index < 6; ++index) {
        state.push_back(vehicle.velocity_[index]);
    }
    for (size_t index = 0; index < 6; ++index) {
        state.push_back(vehicle.position_[index]);
    }
    for (size_t counter = 1; counter <= step_number; ++counter) {
        // call the ode solver
        stepper.do_step(vehicle, state, vehicle.current_time_, step_size);

        // update the data members of the object
        ++vehicle.step_counter_;
        vehicle.current_time_ += step_size;
        vehicle.time_vec_.push_back(vehicle.current_time_);
        vehicle.velocity_ << state[0], state[1], state[2], state[3], state[4], state[5];
        vehicle.position_ << state[6], state[7], state[8], state[9], state[10], state[11];
        // wrap the euler angle into [-pi, pi)
        vehicle.position_[3] = remainder(vehicle.position_[3], 2*M_PI);
        vehicle.position_[4] = remainder(vehicle.position_[4], 2*M_PI);
        vehicle.position_[5] = remainder(vehicle.position_[5], 2*M_PI);
        vehicle.euler_ << vehicle.position_[3], vehicle.position_[4], vehicle.position_[5];
//        vehicle.quaternion_ = vehicle.Euler2q(vehicle.euler_);
        vehicle.relative_velocity_ = vehicle.velocity_ - vehicle.CurrentVelocity(vehicle.position_);
        vehicle.velocity_history_.push_back(vehicle.velocity_);
        vehicle.position_history_.push_back(vehicle.position_);
        vehicle.relative_velocity_history_.push_back(vehicle.relative_velocity_);
    }

    // write the current velocity and position data into the output files
    OutputData(vehicle, "app");
}

#endif //REMUSIMM_REMUS_H
