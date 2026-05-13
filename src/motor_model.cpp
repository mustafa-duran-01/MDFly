#include "mdfly/motor_model.h"
#include "json.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <algorithm>

using json = nlohmann::json;

namespace mdfly {

void MotorModel::load(const std::string& config_path) {
    std::ifstream f(config_path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open motor config file: " + config_path);
    }
    json j;
    f >> j;

    if (j.contains("motor")) {
        auto m = j["motor"];
        Kv = m.value("Kv", Kv);
        Rm = m.value("Rm", Rm);
        I0 = m.value("I0", I0);
        Vbat = m.value("Vbat", Vbat);
        Jm = m.value("Jm", Jm);
    }

    if (j.contains("propeller")) {
        auto p = j["propeller"];
        diameter = p.value("diameter", diameter);
        pitch = p.value("pitch", pitch);
        if (p.contains("Ct_coefs")) {
            Ct_coefs = p["Ct_coefs"].get<std::vector<double>>();
        }
        if (p.contains("Cq_coefs")) {
            Cq_coefs = p["Cq_coefs"].get<std::vector<double>>();
        }
    }
}

void MotorModel::compute_dynamics(double Va, double omega, double throttle, double rho,
                                  double& thrust, double& torque, double& omega_dot) const {
    // 1. Calculate Back-EMF
    // RPM = omega * 30 / pi
    double E = (omega * 30.0) / (M_PI * Kv);

    // 2. Motor current
    double V_applied = throttle * Vbat;
    double I = (V_applied - E) / Rm;
    if (I < 0.0) I = 0.0;

    // 3. Motor electrical torque
    // Kt = 30 / (pi * Kv)
    double Kt = 30.0 / (M_PI * Kv);
    double Qm = 0.0;
    if (I > I0) {
        Qm = Kt * (I - I0);
    }

    // 4. Propeller aerodynamic torque and thrust
    double n = omega / (2.0 * M_PI); // RPS (revolutions per second)
    
    double Ct = 0.0;
    double Cq = 0.0;
    double Qp = 0.0;
    double T = 0.0;

    if (n > 0.05) { // Prop is spinning
        double J = Va / (n * diameter);
        
        if (Ct_coefs.size() >= 3) {
            Ct = Ct_coefs[0] + Ct_coefs[1] * J + Ct_coefs[2] * J * J;
        }
        if (Ct < 0.0) Ct = 0.0;

        if (Cq_coefs.size() >= 3) {
            Cq = Cq_coefs[0] + Cq_coefs[1] * J + Cq_coefs[2] * J * J;
        }
        if (Cq < 0.0) Cq = 0.0;

        Qp = Cq * rho * n * n * std::pow(diameter, 5);
        T = Ct * rho * n * n * std::pow(diameter, 4);
    }

    thrust = T;
    torque = Qp;

    // 5. Rotational acceleration
    omega_dot = (Qm - Qp) / Jm;
}

} // namespace mdfly
