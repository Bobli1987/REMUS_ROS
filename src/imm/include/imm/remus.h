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

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

// constant pi
const double pi = boost::math::constants::pi<double>();
// new matrix types
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 12, 1> Vector12d;

// Class Remus
class Remus
{
    friend class MovingMassController;
    friend void RunRemus(Remus&, const size_t&, const double&);

public:
    // constructors
    Remus(const vector<double> &init_velocity, const vector<double> &init_position,
          const vector<double> &current_velocity, const double &main_thrust, const bool &control_on);
    Remus(): Remus({1.5, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0}, 9, true) {}
    Remus(const vector<double> &init_velocity): Remus(init_velocity, {0, 0, 0, 0, 0, 0}, {0, 0, 0}, 9, true) {}
    Remus(const vector<double> &init_velocity, const vector<double> &init_position,
          const vector<double> &current_velocity, const double &main_thrust):
            Remus(init_velocity, init_position, current_velocity, main_thrust, true) {}

private:
    // parameters of the vehicle
    const double gravity_ = 9.81;
    const double mass_ = 3.05e1, displaced_mass_ = 3.12e1;
    // center of gravity
    const Vector3d cog_ = {0.0, 0.0, 1.96e-2};
    // center of buoyancy
    const Vector3d cob_ = {0.0, 0.0, 0.0};
    // moment of inertia
    const double Ixx_ = 1.77e-1, Iyy_ = 3.45, Izz_ = 3.45;
    const double Ixy_ = 0.0, Ixz_ = 0.0, Iyz_ = 0.0;
    // added mass
    const double a11_ = 9.30e-1, a22_ = 3.55e1, a33_ = 3.55e1;
    const double a44_ = 7.04e-2, a55_ = 4.88, a66_ = 4.88;
    const double a24_ = 0.0, a26_ = -1.93, a35_ = 1.93, a46_ = 0.0;
    // viscous damping coefficients
    const double Xuu_ = -3.87, Yvv_ = -1e2, Yrr_ = 6.32e-1;
    const double Zww_ = 0.0, Zqq_ = 0.0;
    const double Kp_ = -1.60e-3, Kpp_ = -1.30e-1;
    const double Mww_ = 0.0, Mqq_ = 0.0, Nvv_ = 7.38, Nrr_ = -9.40e1;
    const double Yuv_ = -1.79e1, Yur_ = 8.62, Zuw_ = 0.0, Zuq_ = 0.0;
    const double Kup_ = 0, Muw_ = 0.0, Muq_ = 0.0, Nuv_ = 4.57, Nur_ = -6.4;

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
    vector<double> init_velocity_, init_position_;
    // current velocity and position of the vehicle
    Vector6d velocity_, position_;
    // current euler angle of the vehicle
    Vector3d euler_;
    // current quaternion of the vehicle
    Vector4d quaternion_;
    // relative velocity with respect to the ocean current
    Vector6d relative_velocity_;
    // time vector of the simulation
    vector<double> time_vec_ = {0};
    // array of velocity and position vectors during the simulation
    vector<Vector6d> velocity_history_, position_history_, relative_velocity_history_;
    // actuation acting on the vehicle
    Vector6d actuation_;

    // 3X3 matrix which maps a vector to the Lie algebra of SO(3)
    inline Matrix3d CrossProductOperator(const Vector3d &vec) const;
    // rotation matrix declaration
    static Matrix3d RotationMatrix(const Vector3d &angle);
    // transformation from Euler angle representation to quaternion
    static Vector4d Euler2q(const Vector3d &angle);
    // angular transformation matrix declaration
    static Matrix3d AngularTransMatrix(const Vector3d &angle);
    // transformation matrix from the body-fixed frame to the earth-fixed frame
    static Matrix6d TransformationMatrix(const Vector3d &angle);

    // inertia moment of the rigid body
    Matrix3d InertiaMoment() const;
    // inertia matrix of the rigid body
    Matrix6d RigidBodyInertiaMatrix() const;
    // centripetal-coriolis matrix of the rigid body
    Matrix6d RigidBodyCCMatrix(const Vector6d &vec) const;
    // added mass matrix
    Matrix6d AddedMassMatrix() const;
    // centripetal-coriolis matrix of added mass
    Matrix6d AddedMassCCMatrix(const Vector6d &vec) const;
    // inertia moment of the displaced fluid
    Matrix3d DisplacedMassInertiaMoment() const;
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
    void operator() (const vector<double> &state, vector<double> &state_derivative, const double);

    // The ocean current velocity in the body-fixed reference frame
    Vector6d CurrentVelocity(const Vector6d&) const;
    // The ocean current acceleration in the body-fixed reference frame
    Vector6d CurrentAcceleration(const Vector6d&, const Vector6d&) const;
};
// constructor definition
Remus::Remus(const vector<double> &init_velocity, const vector<double> &init_position,
             const vector<double> &current_velocity, const double &main_thrust, const bool &control_on):
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
Vector6d Remus::CurrentVelocity(const Vector6d &position) const {
    Vector3d angle;
    angle << position[3], position[4], position[5];
    return TransformationMatrix(angle).inverse() * ocean_current_.GetCurrentVelocity();
}

// The ocean current acceleration with respect to the body-fixed frame
Vector6d Remus::CurrentAcceleration(const Vector6d &velocity, const Vector6d &position) const {
    Vector6d current_acceleration;
    Vector3d angular_velocity;
    angular_velocity << velocity[3], velocity[4], velocity[5];
    current_acceleration << - CrossProductOperator(angular_velocity) * CurrentVelocity(position).head(3), Vector3d::Zero();
    return current_acceleration;
}

// cross product operator
Matrix3d Remus::CrossProductOperator(const Vector3d &vec) const {
    Matrix3d cross_product;
    cross_product << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
    return cross_product;
}

// rotation matrix definition
Matrix3d Remus::RotationMatrix(const Vector3d &angle) {
    Matrix3d rz, ry, rx;
    rz << cos(angle[2]), -sin(angle[2]), 0, sin(angle[2]), cos(angle[2]), 0, 0, 0, 1;
    ry << cos(angle[1]), 0, sin(angle[1]), 0, 1, 0, -sin(angle[1]), 0, cos(angle[1]);
    rx << 1, 0, 0, 0, cos(angle[0]), -sin(angle[0]), 0, sin(angle[0]), cos(angle[0]);
    return rz * ry * rx;
}

// transformation from rotation matrix to an unit quaternion
Vector4d Remus::Euler2q(const Vector3d &angle) {
    Vector4d q;
    float p1, p2, p3, p4;
    Matrix3d R = RotationMatrix(angle);
    float trace = R.trace();
    q << R(0,0), R(1,1), R(2,2), trace;
    Vector4d::Index maxRow, maxCol;
    float max = q.maxCoeff(&maxRow, &maxCol);
    float p_i = std::sqrt(1 + 2*max - trace);

    switch (maxRow) {
    case 0: {
        p1 = p_i;
        p2 = (R(1,0) + R(0,1))/p_i;
        p3 = (R(0,2) + R(2,0))/p_i;
        p4 = (R(2,1) - R(1,2))/p_i;
    }
    case 1: {
        p1 = (R(1,0) + R(0,1))/p_i;
        p2 = p_i;
        p3 = (R(2,1) + R(1,2))/p_i;
        p4 = (R(0,2) - R(2,0))/p_i;
    }
    case 2: {
        p1 = (R(0,2) + R(2,0))/p_i;
        p2 = (R(2,1) + R(1,2))/p_i;
        p3 = p_i;
        p4 = (R(1,0) - R(0,1))/p_i;
    }
    case 3: {
        p1 = (R(2,1) - R(1,2))/p_i;
        p2 = (R(0,2) - R(2,0))/p_i;
        p3 = (R(1,0) - R(0,1))/p_i;
        p4 = p_i;
    }
    }
    q << 0.5*p1, 0.5*p2, 0.5*p3, 0.5*p4;
    q = q/q.dot(q);
    return q;
}

// angular transformation matrix definition
Matrix3d Remus::AngularTransMatrix(const Vector3d &angle) {
    if (abs(abs(angle[1]) - pi/2) <= pi/6)
        cerr << "Warning: the pitch motion is almost singular!" << endl;
    Matrix3d ang_trans;
    ang_trans << 1, sin(angle[0])*tan(angle[1]), cos(angle[0])*tan(angle[1]),
                 0, cos(angle[0]), -sin(angle[0]),
                 0, sin(angle[0])/cos(angle[1]), cos(angle[0])/cos((angle[1]));
    return ang_trans;
}

// transformation matrix
Matrix6d Remus::TransformationMatrix(const Vector3d &angle) {
    Matrix6d trans_mat;
    Matrix3d zeros = Matrix3d::Zero();
    trans_mat << RotationMatrix(angle), zeros, zeros, AngularTransMatrix(angle);
    return trans_mat;
}

// inertia moment of the rigid body
Matrix3d Remus::InertiaMoment() const {
    Matrix3d inertia_mat;
    inertia_mat << Ixx_, -Ixy_, -Ixz_, -Ixy_, Iyy_, -Iyz_, -Ixz_, -Iyz_, Izz_;
    return inertia_mat;
}

// inertia matrix of the rigid body
Matrix6d Remus::RigidBodyInertiaMatrix() const {
    Matrix6d inertia_mat;
    Matrix3d identity = Matrix3d::Identity();
    inertia_mat << mass_ * identity, -mass_ * CrossProductOperator(cog_),
                   mass_ * CrossProductOperator(cog_), InertiaMoment();
    return inertia_mat;
}

// centripetal-coriolis matrix of rigid body
Matrix6d Remus::RigidBodyCCMatrix(const Vector6d &vec) const {
    Vector3d angular_vel = {vec[3], vec[4], vec[5]};
    Matrix6d cc_mat;
    cc_mat << mass_ * CrossProductOperator(angular_vel),
             -mass_ * CrossProductOperator(angular_vel) * CrossProductOperator(cog_),
              mass_ * CrossProductOperator(cog_) * CrossProductOperator(angular_vel),
             -CrossProductOperator(InertiaMoment() * angular_vel);
    return cc_mat;
}

// added mass matrix
Matrix6d Remus::AddedMassMatrix() const {
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
Matrix6d Remus::AddedMassCCMatrix(const Vector6d &vec) const {
    Matrix6d temp;
    Vector3d lin_vel = {vec[0], vec[1], vec[2]};
    Vector3d ang_vel = {vec[3], vec[4], vec[5]};
    temp << CrossProductOperator(ang_vel), Matrix3d::Zero(),
            CrossProductOperator(lin_vel), CrossProductOperator(ang_vel);
    return temp * AddedMassMatrix();
}

// inertia moment of the displaced fluid
Matrix3d Remus::DisplacedMassInertiaMoment() const {
    return InertiaMoment();
}

// inertia matrix of the displaced fluid
Matrix6d Remus::DisplacedMassInertiaMatrix() const {
    Matrix6d inertia_mat;
    inertia_mat << displaced_mass_ * Matrix3d::Identity(), -displaced_mass_ * CrossProductOperator(cob_),
                   displaced_mass_ * CrossProductOperator(cob_), DisplacedMassInertiaMoment();
    return inertia_mat;
}

// centripetal-coriolis matrix of the displaced fluid
Matrix6d Remus::DisplacedMassCCMatrix(const Vector6d &vec) const {
    Vector3d angular_vel = {vec[3], vec[4], vec[5]};
    Matrix6d cc_mat;
    cc_mat << displaced_mass_ * CrossProductOperator(angular_vel),
             -displaced_mass_ * CrossProductOperator(angular_vel) * CrossProductOperator(cob_),
              displaced_mass_ * CrossProductOperator(cob_) * CrossProductOperator(angular_vel),
             -CrossProductOperator(DisplacedMassInertiaMoment() * angular_vel);
    return cc_mat;
}

// viscous hydrodynamic damping vector
Vector6d Remus::ViscousDampingVector(const Vector6d &vec) const {
    Vector6d damping1, damping2;
    damping1 << Xuu_ * abs(vec[0])*vec[0],
                Yvv_ * abs(vec[1])*vec[1] + Yrr_ * abs(vec[5])*vec[5],
                Zww_ * abs(vec[2])*vec[2] + Zqq_ * abs(vec[4])*vec[4],
                Kp_ * vec[3] + Kpp_ * abs(vec[3])*vec[3],
                Mww_ * abs(vec[2])*vec[2] + Mqq_ * abs(vec[4])*vec[4],
                Nvv_ * abs(vec[1])*vec[1] + Nrr_ * abs(vec[5])*vec[5];
    damping2 << 0,
                Yuv_ * vec[0] * vec[1] + Yur_ * vec[0] * vec[5],
                Zuw_ * vec[0] * vec[2] + Zuq_ * vec[0] * vec[4],
                Kup_ * vec[0] * vec[3],
                Muw_ * vec[0] * vec[2] + Muq_ * vec[0] * vec[4],
                Nuv_ * vec[0] * vec[1] + Nur_ * vec[0] * vec[5];
    return damping1 + damping2;
}

// restoring force vector
Vector6d Remus::RestoringForceVector(const Vector6d &vec) const {
    Vector6d restoring;
    Vector3d angle = {vec[3], vec[4], vec[5]};
    Vector3d vg = {0, 0, mass_*gravity_};
    Vector3d vb = {0, 0, displaced_mass_*gravity_};
    restoring << RotationMatrix(angle).transpose() * (vg - vb),
                 CrossProductOperator(cog_) * RotationMatrix(angle).transpose() * vg \
                -CrossProductOperator(cob_) * RotationMatrix(angle).transpose() * vb;
    return restoring;
}

// time derivative of the system state
Vector12d Remus::StateDerivative(const Vector6d &velocity, const Vector6d &position, const double) const {
    Vector12d state_derivative;
    Vector6d vec1, vec2;
    Vector3d angle;
    angle << position[3], position[4], position[5];
    // set heave and pitch relative velocity to zero
    Vector6d diag1;
    diag1 << 1, 1, 0, 1, 0, 1;
    Vector6d current_velocity = CurrentVelocity(position);
    Vector6d relative_velocity = diag1.asDiagonal() * (velocity - current_velocity);
    Matrix6d total_mass_matrix = RigidBodyInertiaMatrix() + AddedMassMatrix();
    vec1 = total_mass_matrix.inverse() *
           ((DisplacedMassInertiaMatrix() + AddedMassMatrix()) * CurrentAcceleration(velocity, position)
            + DisplacedMassCCMatrix(velocity) * current_velocity
            - AddedMassCCMatrix(relative_velocity) * relative_velocity
            - RigidBodyCCMatrix(velocity) * velocity
            + ViscousDampingVector(relative_velocity)
            + RestoringForceVector(position)
            + actuation_);
    vec2 = TransformationMatrix(angle) * velocity;
    state_derivative << vec1, vec2;
    // ignore heave and pitch motion
    Vector12d diag2;
    diag2 << 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1;
    return diag2.asDiagonal() * state_derivative;
}

// class operator to integrate
void Remus::operator()(const vector<double> &state, vector<double> &state_derivative, const double t) {
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

// conduct a simulation of the vehicle's motion
void RunRemus(Remus &vehicle, const size_t &step_number = 600, const double &step_size = 0.1) {
    vehicle.step_number_ = step_number;
    vehicle.step_size_ = step_size;
    // ode solver
    runge_kutta_dopri5 <vector<double>> stepper;

    vector<double> state;

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
//        vehicle.euler_ << state[9], state[10], state[11];
//        vehicle.quaternion_ = vehicle.Euler2q(vehicle.euler_);
        vehicle.relative_velocity_ = vehicle.velocity_ - vehicle.CurrentVelocity(vehicle.position_);
        vehicle.velocity_history_.push_back(vehicle.velocity_);
        vehicle.position_history_.push_back(vehicle.position_);
        vehicle.relative_velocity_history_.push_back(vehicle.relative_velocity_);
    }
}

#endif //REMUSIMM_REMUS_H
