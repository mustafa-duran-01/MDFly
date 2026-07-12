#ifndef MDFLY_SIMULATOR_H
#define MDFLY_SIMULATOR_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <array>
#include "variable.h"
#include "attitude.h"
#include "wind.h"
#include "actuation.h"
#include "motor_model.h"
#include "camera_model.h"
#include "sensor_model.h"

namespace mdfly {

struct AeroCoefficients {
    // Mass & Inertia
    double mass = 1.0;
    double Jx = 1.0;
    double Jy = 1.0;
    double Jz = 1.0;
    double Jxz = 0.0;

    // Geometry
    double S_wing = 1.0;
    double b = 1.0;
    double c = 1.0;
    double ar = 1.0;

    // Propeller / Motor
    double S_prop = 0.0;
    double C_prop = 0.0;
    double k_motor = 0.0;
    double k_T_P = 0.0;
    double k_Omega = 0.0;

    // Aerodynamics (Lift)
    double C_L_0 = 0.0;
    double C_L_alpha = 0.0;
    double C_L_q = 0.0;
    double C_L_delta_e = 0.0;
    double C_L_alpha_dot = 0.0; // [NEW / MISSING]
    double C_L_delta_a = 0.0;    // [NEW / MISSING]
    double C_L_delta_r = 0.0;    // [NEW / MISSING]

    // Aerodynamics (Drag)
    double C_D_0 = 0.0;
    double C_D_alpha1 = 0.0;
    double C_D_alpha2 = 0.0;
    double C_D_beta1 = 0.0;
    double C_D_beta2 = 0.0;
    double C_D_delta_e = 0.0;
    double C_D_p = 0.0;
    double C_D_q = 0.0;
    double C_D_delta_a = 0.0;    // [NEW / MISSING]
    double C_D_delta_r = 0.0;    // [NEW / MISSING]
    double C_D_alpha_dot = 0.0;  // [NEW / MISSING]
    double Oswald_e = 1.0;

    // Aerodynamics (Sideforce)
    double C_Y_0 = 0.0;
    double C_Y_beta = 0.0;
    double C_Y_p = 0.0;
    double C_Y_r = 0.0;
    double C_Y_delta_a = 0.0;
    double C_Y_delta_r = 0.0;
    double C_Y_q = 0.0;          // [NEW / MISSING]
    double C_Y_delta_e = 0.0;    // [NEW / MISSING]

    // Aerodynamics (Rolling Moment)
    double C_l_0 = 0.0;
    double C_l_beta = 0.0;
    double C_l_p = 0.0;
    double C_l_r = 0.0;
    double C_l_delta_a = 0.0;
    double C_l_delta_r = 0.0;
    double C_l_q = 0.0;          // [NEW / MISSING]
    double C_l_delta_e = 0.0;    // [NEW / MISSING]

    // Aerodynamics (Pitching Moment)
    double C_m_0 = 0.0;
    double C_m_alpha = 0.0;
    double C_m_q = 0.0;
    double C_m_delta_e = 0.0;
    double C_m_fp = 0.0;
    double C_m_beta = 0.0;       // [NEW / MISSING]
    double C_m_delta_a = 0.0;    // [NEW / MISSING]
    double C_m_delta_r = 0.0;    // [NEW / MISSING]
    double C_m_alpha_dot = 0.0;  // [NEW / MISSING]

    // Aerodynamics (Yawing Moment)
    double C_n_0 = 0.0;
    double C_n_beta = 0.0;
    double C_n_p = 0.0;
    double C_n_r = 0.0;
    double C_n_delta_a = 0.0;
    double C_n_delta_r = 0.0;
    double C_n_q = 0.0;          // [NEW / MISSING]
    double C_n_delta_e = 0.0;    // [NEW / MISSING]

    // Stall Model parameters
    double M = 50.0;
    double a_0 = 0.267;
};

class MDFly {
public:
    double dt = 0.01;
    double rho = 1.225;
    double g = 9.81;

    AeroCoefficients params;
    double I[3][3] = {{0.0}};
    double gammas[9] = {0.0};

    std::map<std::string, std::unique_ptr<Variable>> state;
    std::vector<std::string> attitude_states = {"roll", "pitch", "yaw"};
    std::vector<std::string> actuator_states = {"elevator", "aileron", "rudder", "throttle", "elevon_left", "elevon_right"};
    std::vector<std::string> model_inputs = {"elevator", "aileron", "rudder", "throttle"};
    std::vector<std::string> energy_states;
    std::vector<std::string> momentum_states;

    std::vector<std::string> attitude_states_with_constraints;

    AttitudeQuaternion attitude;
    Actuation actuation;
    Wind wind;
    MotorModel motor;
    CameraModel camera;
    SensorModel sensor;
    double last_motor_omega_dot = 0.0;
    double last_gear_cmd = 1.0;
    std::vector<TargetObject> targets;
    std::vector<BoundingBox2D> target_bboxes;

    int cur_sim_step = 0;

    MDFly() = default;

    void add_target(const TargetObject& target);
    void load_motor_prop_config(const std::string& config_path);

    void load_config(const std::string& config_path);
    void load_parameters(const std::string& param_path);
    
    void seed(unsigned int seed_val);
    void reset(const std::map<std::string, double>& state_init = {});
    
    bool step(const std::vector<double>& commands, std::string& term_reason);

    std::vector<double> get_states_vector(const std::vector<std::string>& states_list, const std::string& attribute = "value") const;

    // ODE RHS function
    std::vector<double> ode_dynamics(double t, const std::vector<double>& y, const std::vector<double>& control_sp);

    // Dynamic forces and moments in aircraft body frame
    std::pair<std::array<double, 3>, std::array<double, 3>> forces(
        const std::array<double, 4>& att_q,
        const std::array<double, 3>& omega,
        const std::array<double, 3>& vel,
        const std::array<double, 4>& controls
    );

    // Euler angles rotation matrix functions (for rotation calculations)
    std::array<std::array<double, 3>, 3> rot_b_v(const std::array<double, 3>& euler) const;
    std::array<std::array<double, 3>, 3> rot_b_v(const std::array<double, 4>& att_q) const;
    std::array<std::array<double, 3>, 3> rot_v_b(const std::array<double, 3>& euler) const;

private:
    void calculate_gammas();
    std::array<double, 3> calculate_airspeed_factors(const std::array<double, 4>& att_q, const std::array<double, 3>& vel);
    void set_states_from_ode_solution(const std::vector<double>& ode_sol, bool save);
};

} // namespace mdfly

#endif // MDFLY_SIMULATOR_H
