#ifndef MDFLY_PID_CONTROLLER_H
#define MDFLY_PID_CONTROLLER_H

#include <array>

namespace mdfly {

class PIDController {
public:
    double k_p_V = 0.5;
    double k_i_V = 0.1;
    
    double k_p_phi = 1.0;
    double k_i_phi = 0.0;
    double k_d_phi = 0.5;
    
    double k_p_theta = -4.0;
    double k_i_theta = -0.75;
    double k_d_theta = -0.1;

    double delta_a_min = -30.0 * 3.141592653589793 / 180.0;
    double delta_e_min = -30.0 * 3.141592653589793 / 180.0;
    double delta_a_max = 30.0 * 3.141592653589793 / 180.0;
    double delta_e_max = 35.0 * 3.141592653589793 / 180.0;

    double dt = 0.01;

    double va_r = 0.0;
    double phi_r = 0.0;
    double theta_r = 0.0;

    double int_va = 0.0;
    double int_roll = 0.0;
    double int_pitch = 0.0;

    PIDController(double dt = 0.01);

    void set_reference(double phi, double theta, double va);
    void reset();
    std::array<double, 3> get_action(double phi, double theta, double va, const std::array<double, 3>& omega);
};

} // namespace mdfly

#endif // MDFLY_PID_CONTROLLER_H
