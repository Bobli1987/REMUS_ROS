//
// Created by Bo on 4/5/2016.
//

#ifndef REMUSIMM_CONTROLLER_H
#define REMUSIMM_CONTROLLER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <boost/math/special_functions/sign.hpp>
#include <boost/numeric/odeint.hpp>
#include "remus.h"

// control the position of the internal moving mass to achieve heading stabilization
class MovingMassController
{
    // new matrix types
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

public:
    // constructor
    MovingMassController(const Remus&, const double&, const double&, const double&, const double&, const double&);
    MovingMassController(const Remus& vehicle): MovingMassController(vehicle, 0.5, 1.0, 1.0, 1.0, 1.0) {}

private:
    double mass_, Ixx_, a22_, a44_, gravity_;
    Eigen::Vector3d cog_;
    // tunnel thruster's position in z direction
    double tunnel_pos_;
    // controller parameters
    const double k0_, k1_, k2_, k3_, k4_;
    // intermediate parameters
    double A_, B_, C_, E_, F_, L_, M_, N_, Nq_, epsilon1_, epsilon2_, epsilon3_, epsilon4_;
    double beta1_, beta2_, beta3_, beta4_, gamma1_, gamma2_, gamma3_, gamma4_;
    double G_, dG_, H_, dH_, Gamma1_, Gamma2_, I_, dI_, J_, dJ_, Gamma4_;
    double R1_, R2_, G2_, H2_, I2_, J2_, N2_, N2q_, Gamma12_, Gamma22_, Gamma42_, epsilon12_, epsilon32_, epsilon42_;
    double U0_, mv_, g1_, g2_, g3_;

    // inline member methods
    inline double f1(const double&) const;
    inline double pf1pr(const double&) const;
    inline double ppf1prr(const double&) const;
    inline double f2(const double&, const double&) const ;
    inline double pf2pv(const double&) const;
    inline double pf2pr(const double&) const;
    inline double ppf2pvv(const double&) const;
    inline double ppf2prr(const double&) const;
    inline double f3(const double&, const double&, const double&, const double&) const;
    inline double dr(const double&, const double&) const;
    inline double ddr(const double&, const double&, const double&) const;
    inline double dddr(const double&, const double&, const double&, const double&) const;
    inline double dv(const double&, const double&, const double&) const;
    inline double ddv(const double&, const double&, const double&, const double&) const;
    inline double sat(const double&) const;

public:
    double mass_position_ = 0;
    // the member method used to compute yv
    std::vector<double> ComputeActuation(const Vector6d&, const Vector6d&, const double&, const double&);
};
// constructor
MovingMassController::MovingMassController(const Remus &vehicle, const double &k0, const double &k1, const double &k2,
                                           const double &k3, const double &k4):
        k0_(k0), k1_(k1), k2_(k2), k3_(k3), k4_(k4) {
    mass_ = vehicle.mass_;
    a22_ = vehicle.a22_;
    Ixx_ = vehicle.Ixx_;
    a44_ = vehicle.a44_;
    gravity_ = vehicle.gravity_;
    cog_ = vehicle.cog_;
    tunnel_pos_ = mass_/(mass_+a22_)*cog_[2];
    mv_ = vehicle.mass_/6;
    A_ = (vehicle.mass_+vehicle.a22_)/(vehicle.mass_+vehicle.a11_);
    B_ = -(vehicle.mass_+vehicle.a11_)/(vehicle.mass_+vehicle.a22_);
    C_ = -(vehicle.a22_-vehicle.a11_)/(vehicle.Izz_+vehicle.a66_);
    E_ = (vehicle.mass_*vehicle.cog_[0]+vehicle.a26_)/(vehicle.mass_+vehicle.a22_);
    F_ = (vehicle.mass_*vehicle.cog_[0]+vehicle.a26_)/(vehicle.Izz_+vehicle.a66_);
    L_ = -vehicle.mass_*vehicle.cog_[2]/(vehicle.mass_+vehicle.a22_);
    M_ = -vehicle.mass_*vehicle.cog_[2]/(vehicle.Ixx_+vehicle.a44_);
    N_ = vehicle.Kp_/(vehicle.Ixx_+vehicle.a44_);
    Nq_ = vehicle.Kpp_/(vehicle.Ixx_+vehicle.a44_);
    epsilon1_ = (vehicle.mass_-vehicle.displaced_mass_)*vehicle.gravity_/(vehicle.mass_+vehicle.a22_);
    epsilon2_ = vehicle.mass_*vehicle.gravity_*vehicle.cog_[0]/(vehicle.Izz_+vehicle.a66_);
    epsilon3_ = -vehicle.mass_*vehicle.gravity_*vehicle.cog_[2]/(vehicle.Ixx_+vehicle.a44_);
    epsilon4_ = mv_*vehicle.gravity_/(vehicle.Ixx_+vehicle.a44_);
    beta1_ = vehicle.Yvv_/(vehicle.mass_+vehicle.a22_);
    beta2_ = vehicle.Yrr_/(vehicle.mass_+vehicle.a22_);
    beta3_ = vehicle.Yuv_/(vehicle.mass_+vehicle.a22_);
    beta4_ = vehicle.Yur_/(vehicle.mass_+vehicle.a22_);
    gamma1_ = vehicle.Nvv_/(vehicle.Izz_+vehicle.a66_);
    gamma2_ = vehicle.Nrr_/(vehicle.Izz_+vehicle.a66_);
    gamma3_ = vehicle.Nuv_/(vehicle.Izz_+vehicle.a66_);
    gamma4_ = vehicle.Nur_/(vehicle.Izz_+vehicle.a66_);
    G_ = -C_*E_/(1-E_*F_); dG_ = (beta3_-E_*gamma3_)/(1-E_*F_);
    H_ = (B_+E_*F_)/(1-E_*F_); dH_ = (beta4_-E_*gamma4_)/(1-E_*F_);
    Gamma1_ = (beta1_-E_*gamma1_)/(1-E_*F_); Gamma2_ = (beta2_-E_*gamma2_)/(1-E_*F_);
    I_ = C_/(1-E_*F_); dI_ = (gamma3_-F_*beta3_)/(1-E_*F_);
    J_ = -F_*(1+B_)/(1-E_*F_); dJ_ = (gamma4_-F_*beta4_)/(1-E_*F_);
    Gamma4_ = (gamma2_-F_*beta2_)/(1-E_*F_);
    R1_ = L_/(1-E_*F_); R2_ = (epsilon1_-epsilon2_*E_)/(1-E_*F_);
    G2_ = (G_+dG_)/(1-M_*R1_); H2_ = (H_+dH_+M_*R1_)/(1-M_*R1_);
    I2_ = (I_+dI_-M_*R1_*(C_+gamma3_))/(1-M_*R1_);
    J2_ = (J_+dJ_-M_*R1_*gamma4_)/(1-M_*R1_);
    N2_ = N_/(1-M_*R1_); N2q_ = Nq_/(1-M_*R1_);
    Gamma12_ = Gamma1_/(1-M_*R1_); Gamma22_ = Gamma2_/(1-M_*R1_);
    Gamma42_ = (Gamma4_-M_*R1_*gamma2_)/(1-M_*R1_);
    epsilon12_ = (R2_-epsilon3_*R1_)/(1-M_*R1_);
    epsilon32_ = (epsilon3_-M_*R2_)/(1-M_*R1_);
    epsilon42_ = -R1_*epsilon4_/(1-M_*R1_);
    U0_ = sqrt(-vehicle.actuation_[0]/vehicle.Xuu_);
    g1_ = I2_*U0_;
    g2_ = epsilon12_;
    g3_ = mv_*vehicle.gravity_/(1-M_*R1_)/(vehicle.Ixx_+vehicle.a44_);
}
// member methods
double MovingMassController::f1(const double &r) const { return J2_*U0_*r + Gamma42_*r*std::fabs(r); }
double MovingMassController::pf1pr(const double &r) const { return J2_*U0_ + 2*Gamma42_*std::fabs(r); }
double MovingMassController::ppf1prr(const double &r) const { return 2*Gamma42_*boost::math::sign(r); }
double MovingMassController::f2(const double &v, const double &r) const { return G2_*U0_*v + H2_*U0_*r +
            Gamma12_*v*std::fabs(v) + Gamma22_*r*std::fabs(r); }
double MovingMassController::pf2pv(const double &v) const { return G2_*U0_ + 2*Gamma12_*std::fabs(v); }
double MovingMassController::pf2pr(const double &r) const { return H2_*U0_ + 2*Gamma22_*std::fabs(r); }
double MovingMassController::ppf2pvv(const double &v) const { return 2*Gamma12_*boost::math::sign(v); }
double MovingMassController::ppf2prr(const double &r) const { return 2*Gamma22_*boost::math::sign(r); }
double MovingMassController::f3(const double &v, const double &r, const double &p, const double &phi) const {
    return -M_*G2_*U0_*v - M_*(H2_+1)*U0_*r + N2_*p - M_*Gamma12_*v*std::fabs(v) - M_*Gamma22_*r*std::fabs(r) \
            + N2q_*p*std::fabs(p) + epsilon32_*sin(phi);
}
double MovingMassController::dr(const double &v, const double &r) const { return f1(r) + g1_*v; }
double MovingMassController::ddr(const double &v, const double &r, const double &phi) const {
    return pf1pr(r)*dr(v,r) + g1_*dv(v,r,phi);
}
double MovingMassController::dddr(const double &v, const double &r, const double &p, const double &phi) const {
    return ppf1prr(r)*pow(dr(v,r),2) + pf1pr(r)*ddr(v,r,phi) + g1_*ddv(v,r,p,phi);
}
double MovingMassController::dv(const double &v, const double &r, const double &phi) const {
    return f2(v,r) + g2_*phi;
}
double MovingMassController::ddv(const double &v, const double &r, const double &p, const double &phi) const {
    return pf2pv(v)*dv(v,r,phi) + pf2pr(r)*dr(v,r) + g2_*p;
}

double MovingMassController::sat(const double &x) const {
    return ( std::fabs(x) <= 1 ? x : boost::math::sign(x) );
}
std::vector<double> MovingMassController::ComputeActuation(const Vector6d &rvelocity, const Vector6d &position,
                                                      const double &heading_ref, const double &step_size) {
    double v = rvelocity[1]; // sway velocity
    double r = rvelocity[5]; // yaw rate
    double p = rvelocity[3]; // roll rate
    double phi = position[3]; // roll angle
    double psi = position[5]; // yaw angle
    // heading error
    double epsi = psi - heading_ref;
    double alpha0 = -k0_*epsi;
    double dalpha0 = -k0_*r;
    double ddalpha0 = -k0_*(f1(r) + g1_*v);
    double dddalpha0 = -k0_*pf1pr(r)*dr(v,r) - k0_*g1_*dv(v,r,phi);
    double ddddalpha0 = -k0_*ppf1prr(r)*pow(dr(v,r),2) - k0_*pf1pr(r)*ddr(v,r,phi) - k0_*g1_*ddv(v,r,p,phi);
    double z1 = r - alpha0;
    double alpha1 = (-k1_*z1-epsi-f1(r)+dalpha0)/g1_;
    double dalpha1 = (-r-(k1_+pf1pr(r))*dr(v,r)+k1_*dalpha0+ddalpha0)/g1_;
    double ddalpah1 = (-dr(v,r)-ppf1prr(r)*pow(dr(v,r),2)-(k1_+pf1pr(r))*ddr(v,r,phi)+k1_*ddalpha0+dddalpha0)/g1_;
    double dddalpha1 = (-ddr(v,r,phi)-2*ppf1prr(r)*dr(v,r)*ddv(v,r,p,phi)-(k1_+pf1pr(r))*dddr(v,r,p,phi)
                        -ppf1prr(r)*dr(v,r)*ddr(v,r,phi)+k1_*dddalpha0+ddddalpha0)/g1_;
    double z2 = v - alpha1;
    double alpha2 = (-k2_*z2-g1_*z1-f2(v,r)+dalpha1)/g2_;
    double dalpah2 = (-(k2_+pf2pv(v))*dv(v,r,phi)-(g1_+pf2pr(r))*dr(v,r)+g1_*dalpha0+k2_*dalpha1+ddalpah1)/g2_;
    double ddalpha2 = (-(k2_+pf2pv(v))*ddv(v,r,p,phi)-(g1_+pf2pr(r))*ddr(v,r,phi)+g1_*ddalpha0+k2_*ddalpah1
                       +dddalpha1-ppf2pvv(v)*pow(dv(v,r,phi),2)-ppf2prr(r)*pow(dr(v,r),2))/g2_;
    double z3 = phi - alpha2;
    double alpha3 = -k3_*z3 - g2_*z2 + dalpah2;
    double dalpha3 = -k3_*(p-dalpah2) - g2_*(dv(v,r,phi) - dalpha1) + ddalpha2;
    double z4 = p - alpha3;

    std::vector<double> errors = {epsi, z1, z2, z3, z4};
    // actuation stores both the mass position, tunnel thrust and the roll torque
    std::vector<double> actuation(3, 0); // initialized as {0, 0, 0}

    // set saturation to the mass position
    double mass_pos = (-k4_*z4-z3-f3(v,r,p,phi)+dalpha3)/g3_;
    mass_pos = std::fabs(mass_pos) > 0.06 ? boost::math::sign(mass_pos)*0.06 : mass_pos;
    mass_pos = std::fabs(mass_pos-mass_position_) > 0.06*step_size ? (mass_position_ +
            boost::math::sign(mass_pos-mass_position_)*0.06*step_size) : mass_pos;
    // internal moving mass position
    actuation[0] = mass_pos;
    // tunnel thrust along y axis
    actuation[1] = - std::fabs(epsilon42_*actuation[0])*sat(errors[2]/1e-4);
    actuation[1] *= ((mass_+a22_)*(Ixx_+a44_)-pow(mass_*cog_[2],2))/(Ixx_+a44_-tunnel_pos_*mass_*cog_[2]);
    // roll torque due to the shift of CG
    actuation[2] = mv_ * gravity_ * mass_pos*cos(phi);
    actuation[2] -= actuation[1] * tunnel_pos_;

    mass_position_ = mass_pos;

    return actuation;
}

#endif //REMUSIMM_CONTROLLER_H
