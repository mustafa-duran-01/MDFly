#ifndef MDFLY_MOTOR_MODEL_H
#define MDFLY_MOTOR_MODEL_H

#include <string>
#include <vector>

namespace mdfly {

class MotorModel {
public:
    // Motor Parameters
    double Kv = 400.0;    // RPM/V
    double Rm = 0.04;     // Ohms
    double I0 = 1.0;      // Amps
    double Vbat = 22.2;   // Volts
    double Jm = 0.0001;   // kg*m^2 (inertia)

    // Propeller Parameters
    double diameter = 0.381; // m
    double pitch = 0.127;    // m
    std::vector<double> Ct_coefs = {0.11, -0.06, -0.08};
    std::vector<double> Cq_coefs = {0.009, -0.003, -0.005};

    MotorModel() = default;

    void load(const std::string& config_path);

    // Compute thrust (T), torque (Qp), and acceleration (omega_dot)
    // Va: airspeed (m/s), omega: motor angular speed (rad/s), throttle: 0 to 1
    void compute_dynamics(double Va, double omega, double throttle, double rho,
                          double& thrust, double& torque, double& omega_dot) const;
};

} // namespace mdfly

#endif // MDFLY_MOTOR_MODEL_H
