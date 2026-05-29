#include "mdfly/pid_controller.h"
#include <algorithm>

namespace mdfly {

PIDController::PIDController(double dt) : dt(dt) {}

void PIDController::set_reference(double phi, double theta, double va) {
    phi_r = phi;
    theta_r = theta;
    va_r = va;
}

void PIDController::reset() {
    int_va = 0.0;
    int_roll = 0.0;
    int_pitch = 0.0;
}

std::array<double, 3> PIDController::get_action(double phi, double theta, double va, const std::array<double, 3>& omega) {
    double e_V_a = va - va_r;
    double e_phi = phi - phi_r;
    double e_theta = theta - theta_r;

    int_va += dt * e_V_a;
    int_roll += dt * e_phi;
    int_pitch += dt * e_theta;

    double delta_t = 0.0 - k_p_V * e_V_a - k_i_V * int_va;
    double delta_a = - k_p_phi * e_phi - k_i_phi * int_roll - k_d_phi * omega[0];
    double delta_e = 0.0 - k_p_theta * e_theta - k_i_theta * int_pitch - k_d_theta * omega[1];

    // Constrain commands
    delta_t = std::clamp(delta_t, 0.0, 1.0);
    delta_a = std::clamp(delta_a, delta_a_min, delta_a_max);
    delta_e = std::clamp(delta_e, delta_e_min, delta_e_max);

    return {delta_e, delta_a, delta_t};
}

} // namespace mdfly
